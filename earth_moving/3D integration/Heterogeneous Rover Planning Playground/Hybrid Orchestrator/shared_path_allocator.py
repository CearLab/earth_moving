"""Shared canonical map plus cached rover-specific task overlays."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Any, Dict, Iterable, Optional, Set, Tuple

from rover_planning_overlay import build_profile_overlay_snapshot, world_path_is_sane
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
    )


def get_profile_overlay(snapshot, rover_type):
    """Return the immutable overlay cached for this map epoch and geometry."""
    key = (snapshot.epoch, rover_type.overlay_cache_key)
    with snapshot.overlay_lock:
        overlay = snapshot.overlay_cache.get(key)
        if overlay is None:
            overlay = build_profile_overlay_snapshot(
                snapshot.epoch, snapshot.env_2d, snapshot.coord_converter, rover_type,
                material_value_mode=snapshot.material_value_mode)
            snapshot.overlay_cache[key] = overlay
        return overlay


def _task_weight(policy, task_type):
    if task_type == TARGET_TASK:
        return float(policy.target_weight)
    if task_type == HIGHWAY_TASK:
        return float(policy.highway_weight)
    return 1.0


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
        blocked_paths = set(request.reserved_path_cells)
        blocked_objects = set(request.reserved_object_cells)
        blocked_objects.update(request.consumed_object_cells)
        max_heat = max((candidate.source.heat for candidate in overlay.candidates), default=0.0)

        best = None
        best_score = -float("inf")
        best_tier = float("inf")
        selection_tiers = policy.selection_tiers()
        tier_by_task = {
            task_type: tier_index
            for tier_index, tier in enumerate(selection_tiers)
            for task_type in tier
        }
        for candidate in overlay.candidates:
            if not policy.permits(candidate.task_type):
                continue
            candidate_tier = tier_by_task.get(candidate.task_type, len(selection_tiers))
            if candidate_tier > best_tier:
                continue
            if not world_path_is_sane(
                candidate.path_info.get("world_path"), snapshot.env_radius, tolerance=0.25
            ):
                continue
            if candidate.expected_collected < float(policy.minimum_task_objects):
                continue
            utilization = candidate.expected_collected / max(capacity, 1.0)
            if utilization + 1e-12 < float(policy.minimum_capacity_utilization):
                continue
            required_paths = set(candidate.reserved_path_cells)
            required_objects = set(candidate.reserved_object_cells)
            if required_paths.intersection(blocked_paths):
                continue
            if required_objects.intersection(blocked_objects):
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
            if candidate_tier < best_tier or (candidate_tier == best_tier and score > best_score):
                best_tier = candidate_tier
                best_score = score
                best = candidate

        if best is None:
            return PathAllocationResult(
                request.agent_id, request.plan_id, request.map_epoch, snapshot.env_radius,
                snapshot.env_2d, snapshot.coord_converter)

        path_info = dict(best.path_info)
        path_info["allocation_score"] = best_score
        path_info["rover_type"] = rover_type.name
        path_info["policy_allowed_tasks"] = sorted(policy.allowed_tasks)
        path_info["policy_fallback_order"] = list(policy.task_fallback_order)
        path_info["selected_task_tier"] = int(best_tier)
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
            error=None)
    except Exception as exc:
        return PathAllocationResult(
            request.agent_id, request.plan_id, request.map_epoch, snapshot.env_radius,
            snapshot.env_2d, snapshot.coord_converter, error=str(exc))




