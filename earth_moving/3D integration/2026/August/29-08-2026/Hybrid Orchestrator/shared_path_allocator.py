"""Shared canonical map plus cached rover-specific task overlays."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Any, Dict, Iterable, Optional, Set, Tuple

from rover_planning_overlay import (
    build_profile_overlay_snapshot,
    normalize_target_candidate_value_mode,
    normalize_target_path_mode,
    normalize_target_source_mode,
    world_path_is_sane,
)
from rover_profiles import ROVER_TYPES, TARGET_TASK, HIGHWAY_TASK
from pebble_profiles import (
    DEFAULT_MATERIAL_VALUE_MODE, MASS_MODE, PebbleInstance,
    annotate_material_cells, expand_positions_for_planning,
    normalize_material_mode,
)

CellKey = Tuple[int, int]


@dataclass(frozen=True)
class SharedMapBuildRequest:
    epoch: int
    env_radius: float
    target_zone_radius: float
    shovel_width: float
    pebbles_xy: Tuple[Tuple[float, float], ...]
    use_spillage_model: bool
    visualize_potential: bool
    pebble_materials: Tuple[Any, ...] = ()
    material_value_mode: str = DEFAULT_MATERIAL_VALUE_MODE
    target_zone: Any = None
    target_path_mode: str = "material_aware"
    target_candidate_value_mode: str = "material_aware"


@dataclass
class SharedMapSnapshot:
    epoch: int
    env_radius: float
    env_2d: Any
    coord_converter: Any
    built_wall_time: float
    pebbles_count: int
    pebbles_material_mass: float = 0.0
    material_value_mode: str = DEFAULT_MATERIAL_VALUE_MODE
    target_path_mode: str = "material_aware"
    target_candidate_value_mode: str = "material_aware"
    overlay_cache: Dict[Any, Any] = field(default_factory=dict, repr=False)
    overlay_lock: Any = field(default_factory=threading.RLock, repr=False)


@dataclass(frozen=True)
class PathAllocationRequest:
    agent_id: str
    plan_id: int
    map_epoch: int
    agent_pose_xy: Tuple[float, float]
    reserved_path_cells: Tuple[CellKey, ...] = ()
    reserved_object_cells: Tuple[CellKey, ...] = ()
    consumed_object_cells: Tuple[CellKey, ...] = ()
    rover_type: Any = None
    # Kept for compatibility with older callers; RoverType is authoritative.
    shovel_width: float = 0.22
    overlay_cell_size: float = 0.0
    reservation_radius: float = 0.14
    # None uses the rover profile policy; True/False overrides it for this run.
    target_root_sources_only: Optional[bool] = None
    # Explicit geometric/graph source rule.  None preserves the legacy bool or
    # rover-profile behavior above; choices are all, root, and convex_hull.
    target_source_mode: Optional[str] = None
    # Per-rover preference retained across allocations for percentage-policy
    # hysteresis.  None starts dynamic profiles in highway-preferred mode.
    previous_task_preference: Optional[str] = None
    # Temporary world-space circles created by optional congestion-aware
    # reassignment.  Only this rover sees them; normal allocation is unchanged.
    excluded_task_zones: Tuple[Tuple[float, float, float], ...] = ()


@dataclass
class PathAllocationResult:
    agent_id: str
    plan_id: int
    map_epoch: int
    env_radius: float
    env_2d: Any
    coord_converter: Any
    best_cell: Any = None
    best_choice: str = "target"
    best_path_info: Optional[Dict[str, Any]] = None
    reserved_path_cells: Set[CellKey] = field(default_factory=set)
    reserved_object_cells: Set[CellKey] = field(default_factory=set)
    error: Optional[str] = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)


def normalize_cell_key(value: Any) -> Optional[CellKey]:
    if value is None:
        return None
    if hasattr(value, "x") and hasattr(value, "y"):
        return int(value.x), int(value.y)
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (tuple, list)) and len(value) >= 2:
        return int(value[0]), int(value[1])
    return None


def normalize_cell_keys(values: Iterable[Any]) -> Set[CellKey]:
    out = set()
    for value in values:
        key = normalize_cell_key(value)
        if key is not None:
            out.add(key)
    return out


def path_cells(path_info: Dict[str, Any]) -> Set[CellKey]:
    cells = normalize_cell_keys(path_info.get("path", []))
    cells.update(normalize_cell_keys((path_info.get("impacted_cells", {}) or {}).keys()))
    return cells


def object_cells_for_path(root_cell: Any, path_info: Dict[str, Any]) -> Set[CellKey]:
    cells = set()
    root_key = normalize_cell_key(root_cell)
    if root_key is not None:
        cells.add(root_key)
    for cell in path_info.get("path", []) or []:
        key = normalize_cell_key(cell)
        if key is not None and getattr(cell, "num_objects", 0) > 0:
            cells.add(key)
    cells.update(normalize_cell_keys((path_info.get("impacted_cells", {}) or {}).keys()))
    return cells


def build_shared_map_snapshot(request, compute_2d_env, coordinate_converter_cls):
    material_mode = normalize_material_mode(request.material_value_mode)
    target_path_mode = normalize_target_path_mode(request.target_path_mode)
    target_candidate_value_mode = normalize_target_candidate_value_mode(
        request.target_candidate_value_mode)
    instances = tuple(request.pebble_materials)
    if len(instances) != len(request.pebbles_xy):
        instances = tuple(
            PebbleInstance(index, "legacy", 1.0, 1, (0.3, 0.3, 0.3, 1.0))
            for index in range(len(request.pebbles_xy))
        )
    physical_pebbles_3d = [(float(x), float(y), 0.01) for x, y in request.pebbles_xy]
    planning_pebbles_3d = expand_positions_for_planning(
        physical_pebbles_3d, instances, material_mode)
    env_2d = compute_2d_env(
        request.env_radius, request.target_zone_radius, request.shovel_width,
        planning_pebbles_3d, manual_mode=False,
        use_spillage_model=request.use_spillage_model,
        visualize_potential=request.visualize_potential,
        target_zone=request.target_zone)
    converter = coordinate_converter_cls(
        request.env_radius, request.target_zone_radius, request.shovel_width,
        target_zone=request.target_zone)
    annotate_material_cells(env_2d, converter, request.pebbles_xy, instances)
    env_2d.material_value_mode = material_mode
    return SharedMapSnapshot(
        epoch=request.epoch,
        env_radius=request.env_radius,
        env_2d=env_2d,
        coord_converter=converter,
        built_wall_time=time.time(),
        pebbles_count=len(request.pebbles_xy),
        pebbles_material_mass=sum(instance.material_mass for instance in instances),
        material_value_mode=material_mode,
        target_path_mode=target_path_mode,
        target_candidate_value_mode=target_candidate_value_mode,
    )


def get_profile_overlay(snapshot, rover_type):
    """Return the immutable overlay cached for this map epoch and geometry."""
    target_path_mode = getattr(snapshot, "target_path_mode", "material_aware")
    target_candidate_value_mode = getattr(
        snapshot, "target_candidate_value_mode", "material_aware")
    key = (
        snapshot.epoch,
        rover_type.overlay_cache_key,
        target_path_mode,
        target_candidate_value_mode,
    )
    with snapshot.overlay_lock:
        overlay = snapshot.overlay_cache.get(key)
        if overlay is None:
            overlay = build_profile_overlay_snapshot(
                snapshot.epoch, snapshot.env_2d, snapshot.coord_converter, rover_type,
                material_value_mode=snapshot.material_value_mode,
                target_path_mode=target_path_mode,
                target_candidate_value_mode=target_candidate_value_mode)
            snapshot.overlay_cache[key] = overlay
        return overlay


def _task_weight(policy, task_type):
    if task_type == TARGET_TASK:
        return float(policy.target_weight)
    if task_type == HIGHWAY_TASK:
        return float(policy.highway_weight)
    return 1.0


def candidate_intersects_excluded_task_zone(candidate, excluded_zones):
    """Check source and push endpoints against temporary congestion circles."""
    task_points = [candidate.source.world_center]
    world_path = candidate.path_info.get("world_path") or ()
    if world_path:
        task_points.extend((world_path[0], world_path[-1]))
    return any(
        math.hypot(
            float(point[0]) - float(zone_x),
            float(point[1]) - float(zone_y),
        ) < float(zone_radius)
        for zone_x, zone_y, zone_radius in excluded_zones
        for point in task_points
    )


def _candidate_capacity_metrics(candidate, rover_type, policy):
    """Return uncapped load ratios and the adjustable capacity-priority tier."""
    info = candidate.path_info
    object_capacity = max(float(rover_type.capabilities.capacity_objects), 1e-12)
    mass_capacity = max(float(rover_type.capabilities.capacity_mass), 1e-12)
    collectable_objects = float(
        info.get(
            "collectable_objects",
            getattr(candidate, "expected_collected_objects", candidate.expected_collected),
        )
    )
    collectable_mass = float(
        info.get(
            "collectable_material_mass",
            getattr(candidate, "expected_collected_mass", candidate.expected_collected),
        )
    )
    object_ratio = collectable_objects / object_capacity
    mass_ratio = collectable_mass / mass_capacity
    load_ratio = max(object_ratio, mass_ratio)
    minimum = float(policy.preferred_capacity_min_fraction)
    maximum = float(policy.preferred_capacity_max_fraction)
    if load_ratio + 1e-12 < minimum:
        capacity_class = "underfilled"
        capacity_tier = 1
    elif load_ratio <= maximum + 1e-12:
        capacity_class = "fit"
        capacity_tier = 0
    else:
        capacity_class = "overcapacity"
        capacity_tier = 0 if policy.overcapacity_behavior == "allow" else 1
    reject_overcapacity = False
    if capacity_class == "overcapacity":
        reject_overcapacity = policy.overcapacity_behavior == "reject"
        maximum_allowed = policy.max_overcapacity_fraction
        if maximum_allowed is not None and load_ratio > float(maximum_allowed) + 1e-12:
            reject_overcapacity = True
    return {
        "collectable_objects_uncapped": collectable_objects,
        "collectable_material_mass_uncapped": collectable_mass,
        "capacity_object_load_ratio": object_ratio,
        "capacity_mass_load_ratio": mass_ratio,
        "capacity_effective_load_ratio": load_ratio,
        "capacity_class": capacity_class,
        "capacity_tier": capacity_tier,
        "reject_overcapacity": reject_overcapacity,
    }


def _cell_is_in_target(snapshot, cell):
    converter_check = getattr(snapshot.coord_converter, "is_in_target_zone", None)
    if callable(converter_check):
        try:
            return bool(converter_check(int(cell.x), int(cell.y)))
        except (TypeError, ValueError):
            pass
    return bool(getattr(cell, "is_target_zone", False))


def _candidate_matches_target_source_mode(candidate, target_source_mode):
    mode = normalize_target_source_mode(target_source_mode)
    if mode == "root":
        return bool(candidate.is_root_source)
    if mode == "convex_hull":
        return bool(getattr(candidate, "is_convex_hull_source", False))
    return True


def calculate_target_ready_fraction(
        snapshot, overlay, policy, root_sources_only=False,
        target_source_mode=None):
    """Measure the physical share of outside pebbles ready for direct target work.

    The denominator is every physical pebble outside the target.  The numerator
    includes pebbles whose canonical cell has sufficiently high normalized
    potential and belongs to a geometrically feasible target candidate.  Live
    reservations are intentionally excluded so preference does not oscillate
    as rovers claim and release tasks.
    """
    outside_cells = [
        cell for cell in getattr(snapshot.env_2d, "cells_with_objects", ())
        if not _cell_is_in_target(snapshot, cell)
    ]
    all_outside_cells = [
        cell for cell in getattr(snapshot.env_2d, "all_cells", ())
        if not _cell_is_in_target(snapshot, cell)
    ]
    max_potential = max(
        (float(getattr(cell, "heat_map", 0.0)) for cell in all_outside_cells),
        default=0.0,
    )
    potential_ratio = float(policy.good_location_potential_ratio)
    minimum_potential = max_potential * potential_ratio

    source_mode = normalize_target_source_mode(
        target_source_mode
        if target_source_mode is not None
        else ("root" if root_sources_only else "all")
    )
    target_ready_cell_keys = set()
    for candidate in overlay.candidates:
        if candidate.task_type != TARGET_TASK:
            continue
        if not _candidate_matches_target_source_mode(candidate, source_mode):
            continue
        target_ready_cell_keys.update(
            (int(cell.x), int(cell.y))
            for cell in candidate.source.canonical_cells
            if not _cell_is_in_target(snapshot, cell)
        )

    outside_count = sum(
        float(getattr(cell, "physical_object_count", 0.0))
        for cell in outside_cells
    )
    ready_count = 0.0
    if max_potential > 1e-12:
        ready_count = sum(
            float(getattr(cell, "physical_object_count", 0.0))
            for cell in outside_cells
            if (int(cell.x), int(cell.y)) in target_ready_cell_keys
            and float(getattr(cell, "heat_map", 0.0)) + 1e-12
            >= minimum_potential
        )
    ready_fraction = ready_count / outside_count if outside_count > 1e-12 else 0.0
    live_pebble_count = max(float(getattr(snapshot, "pebbles_count", 0.0)), 0.0)
    remaining_fraction = (
        outside_count / live_pebble_count if live_pebble_count > 1e-12 else 0.0
    )
    return {
        "live_pebble_count": live_pebble_count,
        "outside_pebble_count": outside_count,
        "remaining_outside_pebble_fraction": remaining_fraction,
        "target_ready_pebble_count": ready_count,
        "target_ready_fraction": ready_fraction,
        "good_location_potential_ratio": potential_ratio,
        "maximum_potential": max_potential,
        "minimum_good_potential": minimum_potential,
        "target_ready_source_cells": len(target_ready_cell_keys),
        "target_source_mode": source_mode,
    }


def allocate_path_from_shared_map(snapshot, request):
    try:
        rover_type = request.rover_type or ROVER_TYPES["small"]
        policy = rover_type.policy
        capacity = float(
            rover_type.capabilities.capacity_mass
            if snapshot.material_value_mode == MASS_MODE
            else rover_type.capabilities.capacity_objects
        )
        overlay = get_profile_overlay(snapshot, rover_type)
        diagnostics = {
            "overlay_sources": len(overlay.sources),
            "overlay_candidates": len(overlay.candidates),
            "target_path_mode": getattr(snapshot, "target_path_mode", "material_aware"),
            "target_candidate_value_mode": getattr(
                snapshot, "target_candidate_value_mode", "material_aware"),
            "candidate_tasks": {},
            "candidate_guides": {},
            "overlay_generation_failures": dict(
                getattr(overlay, "generation_failures", ()) or (),
            ),
            "rejections": {},
        }
        for candidate in overlay.candidates:
            task = str(candidate.task_type)
            diagnostics["candidate_tasks"][task] = diagnostics["candidate_tasks"].get(task, 0) + 1
            guide_mode = str(candidate.path_info.get("guide_mode", "canonical"))
            diagnostics["candidate_guides"][guide_mode] = diagnostics["candidate_guides"].get(guide_mode, 0) + 1

        def reject(reason):
            diagnostics["rejections"][reason] = diagnostics["rejections"].get(reason, 0) + 1

        requested_source_mode = getattr(request, "target_source_mode", None)
        root_sources_only = (
            bool(policy.target_root_sources_only)
            if request.target_root_sources_only is None
            else bool(request.target_root_sources_only)
        )
        target_source_mode = normalize_target_source_mode(
            requested_source_mode
            if requested_source_mode is not None
            else ("root" if root_sources_only else "all")
        )
        root_sources_only = target_source_mode == "root"
        blocked_paths = set(request.reserved_path_cells)
        blocked_objects = set(request.reserved_object_cells)
        blocked_objects.update(request.consumed_object_cells)
        max_heat = max((candidate.source.heat for candidate in overlay.candidates), default=0.0)

        percentage_metrics = calculate_target_ready_fraction(
            snapshot,
            overlay,
            policy,
            root_sources_only=root_sources_only,
            target_source_mode=target_source_mode,
        )
        selection_tiers = policy.selection_tiers(
            percentage_metrics["target_ready_fraction"],
            request.previous_task_preference,
            percentage_metrics["remaining_outside_pebble_fraction"],
        )
        effective_preference = (
            next(iter(selection_tiers[0]))
            if selection_tiers and len(selection_tiers[0]) == 1
            else None
        )
        diagnostics.update(percentage_metrics)
        diagnostics.update({
            "percentage_policy_enabled": bool(policy.uses_percentage_policy),
            "task_policy_mode": policy.task_policy_mode,
            "target_preference_enter_fraction": policy.target_preference_enter_fraction,
            "target_preference_exit_fraction": policy.target_preference_exit_fraction,
            "allow_highway_fallback_when_target_preferred": (
                policy.allow_highway_fallback_when_target_preferred
            ),
            "allow_target_fallback_when_highway_preferred": (
                policy.allow_target_fallback_when_highway_preferred
            ),
            "endgame_target_only_remaining_fraction": (
                policy.endgame_target_only_remaining_fraction
            ),
            "endgame_target_only_active": bool(
                policy.task_policy_mode == "dynamic"
                and policy.endgame_target_only_remaining_fraction is not None
                and percentage_metrics["remaining_outside_pebble_fraction"]
                <= float(policy.endgame_target_only_remaining_fraction) + 1e-12
            ),
            "previous_task_preference": request.previous_task_preference,
            "task_preference": effective_preference,
            "selection_tiers": [sorted(tier) for tier in selection_tiers],
            "capacity_policy": {
                "preferred_min_fraction": policy.preferred_capacity_min_fraction,
                "preferred_max_fraction": policy.preferred_capacity_max_fraction,
                "minimum_capacity_utilization": policy.minimum_capacity_utilization,
                "utilization_definition": "max(object_load_ratio,mass_load_ratio)",
                "overcapacity_behavior": policy.overcapacity_behavior,
                "max_overcapacity_fraction": policy.max_overcapacity_fraction,
                "capacity_fit_before_task_fallback": (
                    policy.capacity_fit_before_task_fallback
                ),
            },
            "capacity_classes": {},
            "target_source_mode": target_source_mode,
            "convex_hull_source_cells": len(getattr(
                overlay, "convex_hull_source_cell_keys", ()) or ()),
            "convex_hull_definition": getattr(
                overlay, "convex_hull_definition",
                "outside_material_plus_target_boundary"),
        })

        best = None
        best_score = -float("inf")
        best_rank = (float("inf"), float("inf"))
        best_task_tier = float("inf")
        best_capacity_tier = float("inf")
        best_capacity_metrics = None
        tier_by_task = {
            task_type: tier_index
            for tier_index, tier in enumerate(selection_tiers)
            for task_type in tier
        }
        for candidate in overlay.candidates:
            if not policy.permits(candidate.task_type):
                reject("task_not_permitted")
                continue
            if candidate.task_type not in tier_by_task:
                reject("task_disabled_by_percentage_policy")
                continue
            if candidate_intersects_excluded_task_zone(
                candidate, request.excluded_task_zones,
            ):
                reject("temporary_congestion_zone")
                continue
            if (
                candidate.task_type == TARGET_TASK
                and not _candidate_matches_target_source_mode(
                    candidate, target_source_mode)
            ):
                reject(
                    "not_root_source"
                    if target_source_mode == "root"
                    else "not_convex_hull_source"
                )
                continue
            task_tier = tier_by_task.get(candidate.task_type, len(selection_tiers))
            if not world_path_is_sane(
                candidate.path_info.get("world_path"), snapshot.env_radius, tolerance=0.25
            ):
                reject("world_path_outside_map")
                continue
            if candidate.expected_collected < float(policy.minimum_task_objects):
                reject("below_minimum_task")
                continue
            capacity_metrics = _candidate_capacity_metrics(candidate, rover_type, policy)
            # Use physical count and mass together so the hard floor has the
            # same meaning in count-mode and mass-mode experiments.
            utilization = capacity_metrics["capacity_effective_load_ratio"]
            if utilization + 1e-12 < float(policy.minimum_capacity_utilization):
                reject("below_capacity_utilization")
                continue
            capacity_class = capacity_metrics["capacity_class"]
            diagnostics["capacity_classes"][capacity_class] = (
                diagnostics["capacity_classes"].get(capacity_class, 0) + 1
            )
            if capacity_metrics["reject_overcapacity"]:
                reject("overcapacity_rejected")
                continue
            capacity_tier = int(capacity_metrics["capacity_tier"])
            rank = (
                (capacity_tier, task_tier)
                if policy.capacity_fit_before_task_fallback
                else (task_tier, capacity_tier)
            )
            if rank > best_rank:
                reject("lower_combined_preference_tier")
                continue
            required_paths = set(candidate.reserved_path_cells)
            required_objects = set(candidate.reserved_object_cells)
            if required_paths.intersection(blocked_paths):
                reject("reserved_path_conflict")
                continue
            if required_objects.intersection(blocked_objects):
                reject("reserved_object_conflict")
                continue

            dist = math.hypot(
                request.agent_pose_xy[0] - candidate.source.world_center[0],
                request.agent_pose_xy[1] - candidate.source.world_center[1])
            heat = candidate.source.heat / max_heat if max_heat > 1e-12 else 0.0
            task_weight = _task_weight(policy, candidate.task_type)
            score = (
                float(policy.delivered_weight) * task_weight * candidate.expected_delivered
                + float(policy.heat_weight) * heat
                + float(policy.capacity_utilization_weight) * utilization
                - float(policy.approach_distance_weight) * dist
                - float(policy.spillage_weight) * candidate.expected_spillage
            )
            if rank < best_rank or (rank == best_rank and score > best_score):
                best_rank = rank
                best_task_tier = task_tier
                best_capacity_tier = capacity_tier
                best_score = score
                best = candidate
                best_capacity_metrics = capacity_metrics

        if best is None:
            return PathAllocationResult(
                request.agent_id, request.plan_id, request.map_epoch, snapshot.env_radius,
                snapshot.env_2d, snapshot.coord_converter,
                diagnostics=diagnostics)

        path_info = dict(best.path_info)
        # Preserve the allocator's raw prediction separately from physical
        # post-push measurements written by benchmark_telemetry.py.
        path_info["map_epoch"] = int(request.map_epoch)
        path_info["expected_collected"] = float(best.expected_collected)
        path_info["expected_delivered"] = float(best.expected_delivered)
        path_info["expected_spillage"] = float(best.expected_spillage)
        path_info["allocation_score"] = best_score
        path_info["rover_type"] = rover_type.name
        path_info["policy_allowed_tasks"] = sorted(policy.allowed_tasks)
        path_info["policy_task_mode"] = policy.task_policy_mode
        path_info["policy_fallback_order"] = list(policy.task_fallback_order)
        path_info["policy_target_root_sources_only"] = root_sources_only
        path_info["policy_target_source_mode"] = target_source_mode
        path_info["policy_percentage_enabled"] = bool(policy.uses_percentage_policy)
        path_info["policy_good_location_potential_ratio"] = percentage_metrics[
            "good_location_potential_ratio"
        ]
        path_info["policy_target_preference_enter_fraction"] = (
            policy.target_preference_enter_fraction
        )
        path_info["policy_target_preference_exit_fraction"] = (
            policy.target_preference_exit_fraction
        )
        path_info["policy_allow_highway_fallback_when_target_preferred"] = (
            policy.allow_highway_fallback_when_target_preferred
        )
        path_info["policy_allow_target_fallback_when_highway_preferred"] = (
            policy.allow_target_fallback_when_highway_preferred
        )
        path_info["policy_endgame_target_only_remaining_fraction"] = (
            policy.endgame_target_only_remaining_fraction
        )
        path_info["policy_endgame_target_only_active"] = diagnostics[
            "endgame_target_only_active"
        ]
        path_info["policy_previous_task_preference"] = request.previous_task_preference
        path_info["policy_task_preference"] = effective_preference
        path_info["policy_selection_tiers"] = [sorted(tier) for tier in selection_tiers]
        path_info["target_ready_fraction"] = percentage_metrics["target_ready_fraction"]
        path_info["target_ready_pebble_count"] = percentage_metrics[
            "target_ready_pebble_count"
        ]
        path_info["outside_pebble_count"] = percentage_metrics["outside_pebble_count"]
        path_info["remaining_outside_pebble_fraction"] = percentage_metrics[
            "remaining_outside_pebble_fraction"
        ]
        path_info["selected_task_tier"] = int(best_task_tier)
        path_info["selected_capacity_tier"] = int(best_capacity_tier)
        path_info["selected_combined_rank"] = list(best_rank)
        path_info.update(best_capacity_metrics or {})
        path_info["policy_preferred_capacity_min_fraction"] = (
            policy.preferred_capacity_min_fraction
        )
        path_info["policy_minimum_capacity_utilization"] = (
            policy.minimum_capacity_utilization
        )
        path_info["capacity_utilization_definition"] = (
            "max(object_load_ratio,mass_load_ratio)"
        )
        path_info["policy_preferred_capacity_max_fraction"] = (
            policy.preferred_capacity_max_fraction
        )
        path_info["policy_overcapacity_behavior"] = policy.overcapacity_behavior
        path_info["policy_max_overcapacity_fraction"] = (
            policy.max_overcapacity_fraction
        )
        path_info["policy_capacity_fit_before_task_fallback"] = (
            policy.capacity_fit_before_task_fallback
        )
        path_info["material_value_mode"] = snapshot.material_value_mode
        path_info["capacity_quantity"] = capacity
        return PathAllocationResult(
            agent_id=request.agent_id,
            plan_id=request.plan_id,
            map_epoch=request.map_epoch,
            env_radius=snapshot.env_radius,
            env_2d=snapshot.env_2d,
            coord_converter=snapshot.coord_converter,
            best_cell=best.representative_cell,
            best_choice=best.task_type,
            best_path_info=path_info,
            reserved_path_cells=set(best.reserved_path_cells),
            reserved_object_cells=set(best.reserved_object_cells),
            error=None,
            diagnostics=diagnostics)
    except Exception as exc:
        return PathAllocationResult(
            request.agent_id, request.plan_id, request.map_epoch, snapshot.env_radius,
            snapshot.env_2d, snapshot.coord_converter, error=str(exc))




