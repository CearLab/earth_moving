"""
orchestrator_hybrid_multi_astar_scheduled.py

Multi-rover hybrid orchestrator that combines:
- Federico/shared-map multi-agent task allocation
- A* approach trajectory generation from orchestrator_hybrid_astar.py
- progress-aware A* path tracking and conflict scheduling from
  multi_astar_priority_scheduling3.py

The older orchestrators are left untouched. This runner is intended as the
integration point for multi-rover A* planning, time/ETA conflict handling,
slow/wait/replan behavior, emergency stops, and close-pair backoff.
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
from dataclasses import replace
from functools import lru_cache
import math
import os
import sys
import time
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
PATH_TRACKING_DIR = os.path.abspath(os.path.join(HERE, "..", "Path Tracking"))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if PATH_TRACKING_DIR not in sys.path:
    sys.path.insert(0, PATH_TRACKING_DIR)

import pybullet as p

import main as _planning_2d
import rover_planning_overlay as _planning_overlay
import orchestrator_hybrid_multi_shared_safe as _shared
import multi_astar_priority_scheduling3 as sched
from rover_profiles import ROVER_PROFILES, resolve_profiles
from rover_planning_overlay import bounded_approach_gate, world_path_is_sane
from multi_agent_collision_safety import SafetyAgentContext
from cooperative_collision_recovery import (
    CooperativeRecoveryConfig, CooperativeRecoveryManager, RecoveryAgent,
)
from fleet_deadlock_recovery import (
    EscapeRoute, FleetDeadlockConfig, FleetDeadlockRecoveryManager,
    FleetRecoveryAgent,
)
from prioritized_trajectory_reservations import (
    PrioritizedReservationConfig,
    PrioritizedTrajectoryCoordinator,
    ReservationAgent,
)
from congestion_aware_reassignment import (
    ApproachCongestionMonitor, ApproachObservation, CongestionConfig,
)
from simulation_event_logger import SimulationEventLogger
from benchmark_telemetry import BenchmarkTelemetry
from pebble_profiles import (
    DEFAULT_MATERIAL_VALUE_MODE, DEFAULT_PEBBLE_DISTRIBUTION,
    MATERIAL_VALUE_MODES, summarize_instances,
)
from scenario_configs import (
    DEFAULT_SCENARIO_NAME, SCENARIOS, get_scenario,
    baseline_circle_uniform, mixed_pebbles_circle,
    rectangle_target, heterogeneous_cooperation,
    ellipse_target, semicircle_target, l_shape_target, off_center_target,
    concave_target, amorphous_target,
)
from target_zones import resolve_target_zone
from astar_path_following_flowfield import (
    AStarGridPlanner,
    build_obstacle_field,
    cluster_pebbles_by_distance,
    polyline_is_free,
    resample_polyline,
    smooth_path_collision_checked,
    split_pebbles_by_cluster_size,
)


_base = _shared._base
SIMULATION_CONFIG = _shared.SIMULATION_CONFIG
SPILLAGE_CONFIG = _base.SPILLAGE_CONFIG
GATE_CONFIG = _base.GATE_CONFIG
ROVER_POSE_PRESETS = _shared.ROVER_POSE_PRESETS


ASTAR_APPROACH_CONFIG = {
    "cell_size": 0.08,
    "rover_radius": 0.15,
    "pebble_radius": 0.05,
    "clearance": 0.01,
    "env_margin": 0.60,
    "smoothing_iterations": 5,
    "smoothing_cut": 0.25,
    "resample_spacing": 0.08,
    "target_zone_guard_margin": 0.10,
    "allow_relaxation": True,
    "relax_min_cluster_size": 2,
    "relax_progressive": True,
    "relax_max_ignored_cluster_size": None,
    "relax_cluster_link_radius": 0.35,
    "relax_cluster_extra_gap": 0.10,
}


class TargetKeepoutBlocked(RuntimeError):
    """Approach planning was requested before the rover footprint cleared the target."""

    def __init__(self, agent_id, signed_clearance, required_clearance):
        self.agent_id = str(agent_id)
        self.signed_clearance = float(signed_clearance)
        self.required_clearance = float(required_clearance)
        super().__init__(
            f"{self.agent_id}: approach cannot start inside the target keepout; "
            "the rover must complete TARGET_EXIT first."
        )

ROLLBACK_AFTER_PUSH_SEC = 1.0
ROLLBACK_SPEED = -0.5
TARGET_EXIT_REVERSE_SPEED = -0.35
TARGET_EXIT_CLEARANCE_EXTRA = 0.05
TARGET_EXIT_TURN_GAIN = 4.0
TARGET_EXIT_TURN_LIMIT = 5.0
TARGET_EXIT_GOAL_EXTRA = 0.08
UNFINISHED_MATERIAL_REFRESH_LIMIT = 2

# ===================== EASY EDIT SETTINGS =====================
# These defaults are used when you click Play without terminal arguments.
# Change them here, save, and run again.
# Choose exactly one bare name below (quotes are optional):
#   rectangle_target
#   mixed_pebbles_circle
#   baseline_circle_uniform
#   heterogeneous_cooperation
#   ellipse_target
#   semicircle_target
#   l_shape_target
#   off_center_target
#   concave_target
#   amorphous_target
SCENARIO_NAME = concave_target
# None keeps the scenario's own center. Set (x, y) to move any selected shape.
# Example: TARGET_ZONE_CENTER = (1.25, 0.65)
TARGET_ZONE_CENTER = (1.25, 0.65)
SHOW_3D_PATHS = False              # A* approach, connector, 2D push, and extension lines.
SHOW_3D_PLANNING_BOUNDARIES = False # Nominal/expanded planning rings in PyBullet.
SHOW_3D_CONFLICT_MARKERS = False   # Scheduler conflict/debug markers.
FLOW_FIELD_VIS_MODE = "never"      # "never", "ask", or "always".
DEFAULT_PUSH_EXTRA_DISTANCE = 0.25 # Extra meters at the end of each 2D push path.
DEFAULT_MAP_UPDATE_INTERVAL = 12.0 # Minimum seconds between completed-path 2D map rebuilds.
DEFAULT_APPROACH_REPLAN_INTERVAL = 2.5 # Seconds between approach-only A* replans.
DEFAULT_NAVIGATION_OUTSIDE_MARGIN = 1.00 # Approach/recovery room beyond the material map.
# None = use each rover profile; True/False = override all rover profiles.
TARGET_ROOT_SOURCES_ONLY = None
# None preserves the legacy profile/root flag.  Explicit choices are all,
# root (push-graph relationship), or convex_hull (geometric exposure).
TARGET_SOURCE_MODE = None
# Direct-target comparison controls.  The normal/full planner remains the
# default.  RUN_DIRECT_PATH_COMPARISON.py sets these explicitly per preset.
DIRECT_TARGET_PATH_MODE = "material_aware"       # material_aware/straight_nearest
TARGET_CANDIDATE_VALUE_MODE = "material_aware"  # material_aware/source_only
COMPARISON_LABEL = None
CAPACITY_SCORING_MODE = "profile_cap"  # profile_cap/uncapped_corridor
ENABLE_EVENT_LOG = True             # Automatic JSONL telemetry + CSV task summary.
DEFAULT_EVENT_LOG_POSE_INTERVAL = 0.50
# Benchmark telemetry is intentionally separate from controller telemetry so it
# can be enabled/disabled without changing rover motion or planner decisions.
ENABLE_BENCHMARK_TELEMETRY = True
BENCHMARK_CAPTURE_IMAGES = True
BENCHMARK_CAPTURE_EVERY_PUSH = True
BENCHMARK_SAVE_COMPRESSED_SNAPSHOTS = True
BENCHMARK_POST_PUSH_OBSERVATION_DELAY = 1.00
BENCHMARK_PUSH_CORRIDOR_MARGIN = 0.08
# Fixed across every benchmark run; unlike planner_metrics this is independent
# of HIGHWAY_THRESHOLD_RATIO and is therefore safe for cross-run comparison.
BENCHMARK_REFERENCE_HIGHWAY_RATIO = 0.50
BENCHMARK_DELIVERY_MILESTONES = (0.25, 0.50, 0.75, 0.90, 1.00)
BENCHMARK_IMAGE_DPI = 150
PEBBLE_MATERIAL_MODE = DEFAULT_MATERIAL_VALUE_MODE  # "mass" or "count" thesis comparison.
PEBBLE_DISTRIBUTION = dict(DEFAULT_PEBBLE_DISTRIBUTION)
OUT_OF_BOUNDS_MARGIN = 0.75         # Quarantine a rover beyond the configured arena.
PATH_ANOMALY_MARGIN = 1.00          # Reject a controller path outside the arena envelope.
DEFAULT_ROVER_PROFILES = "small,large,small" # Used when clicking Play with the default 3 rovers.

# Per-profile task allocation knobs. Fractions are 0.0-1.0, so 0.30 means 30%.
# task_policy_mode: "dynamic", "target_only", or "highway_only".
ROVER_POLICY_OVERRIDES = {
    "small": {
        "task_policy_mode": "dynamic",
        "good_location_potential_ratio": 0.65,
        "target_preference_enter_fraction": 0.80,
        "target_preference_exit_fraction": 0.70,
        "allow_highway_fallback_when_target_preferred": True,
        "allow_target_fallback_when_highway_preferred": True,
        "endgame_target_only_remaining_fraction": 0.20,
        "preferred_capacity_min_fraction": 0.35,
        "preferred_capacity_max_fraction": 1.00,
        "minimum_capacity_utilization": 0.00, # Hard reject floor; small accepts cleanup loads.
        "overcapacity_behavior": "deprioritize",  # allow/deprioritize/reject
        "max_overcapacity_fraction": None,
        "capacity_fit_before_task_fallback": True,
    },
    "large": {
        "task_policy_mode": "target_only",
        "good_location_potential_ratio": 0.65,
        "preferred_capacity_min_fraction": 0.35,
        "preferred_capacity_max_fraction": 1.00,
        "minimum_capacity_utilization": 0.35, # Large waits/parks below 35% effective load.
        "overcapacity_behavior": "deprioritize",
        "max_overcapacity_fraction": None,
        "capacity_fit_before_task_fallback": True,
    },
}

# 2D visibility, path-generation, and highway-target knobs.
MAX_PATH_LENGTH_FACTOR = 2.50
TARGET_VISIBILITY_MODE = "fixed"       # "fixed" or "dynamic"
TARGET_VISIBILITY_ANGLE = 30.0          # degrees, used in fixed mode
HIGHWAY_VISIBILITY_MODE = "fixed"      # "fixed" or "dynamic"
HIGHWAY_VISIBILITY_ANGLE = 60.0         # degrees, used in fixed mode
DYNAMIC_VISIBILITY_MIN_ANGLE = 30.0     # far from destination
DYNAMIC_VISIBILITY_MAX_ANGLE = 60.0     # close to destination
TARGET_VISIBILITY_RANGE_FACTOR = 1.00   # 1.0 reaches up to target distance
HIGHWAY_MIN_HEAT_RATIO = 0.30
HIGHWAY_THRESHOLD_RATIO = 0.50
HIGHWAY_HEAT_WEIGHT = 0.70
HIGHWAY_DISTANCE_WEIGHT = 0.30
# ==============================================================


def _resolved_profiles_with_overrides(spec, rover_count):
    profiles = resolve_profiles(spec, rover_count)
    configured = []
    for profile in profiles:
        overrides = ROVER_POLICY_OVERRIDES.get(profile.name, {})
        configured.append(
            replace(profile, policy=replace(profile.policy, **overrides))
            if overrides else profile
        )
    return configured


def _policy_log_config(policy):
    return {
        "mode": policy.task_policy_mode,
        "allowed_tasks": sorted(policy.allowed_tasks),
        "good_location_potential_ratio": policy.good_location_potential_ratio,
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
        "delivered_weight": policy.delivered_weight,
        "heat_weight": policy.heat_weight,
        "capacity_utilization_weight": policy.capacity_utilization_weight,
        "approach_distance_weight": policy.approach_distance_weight,
        "spillage_weight": policy.spillage_weight,
        "preferred_capacity_min_fraction": policy.preferred_capacity_min_fraction,
        "preferred_capacity_max_fraction": policy.preferred_capacity_max_fraction,
        "minimum_capacity_utilization": policy.minimum_capacity_utilization,
        "capacity_utilization_definition": "max(object_load_ratio,mass_load_ratio)",
        "overcapacity_behavior": policy.overcapacity_behavior,
        "max_overcapacity_fraction": policy.max_overcapacity_fraction,
        "capacity_fit_before_task_fallback": policy.capacity_fit_before_task_fallback,
    }


def _configure_2d_planner():
    return _planning_2d.configure_planning_hyperparameters(
        max_path_length_factor=MAX_PATH_LENGTH_FACTOR,
        target_visibility_mode=TARGET_VISIBILITY_MODE,
        target_visibility_angle=TARGET_VISIBILITY_ANGLE,
        highway_visibility_mode=HIGHWAY_VISIBILITY_MODE,
        highway_visibility_angle=HIGHWAY_VISIBILITY_ANGLE,
        dynamic_visibility_min_angle=DYNAMIC_VISIBILITY_MIN_ANGLE,
        dynamic_visibility_max_angle=DYNAMIC_VISIBILITY_MAX_ANGLE,
        target_visibility_range_factor=TARGET_VISIBILITY_RANGE_FACTOR,
        highway_min_heat_ratio=HIGHWAY_MIN_HEAT_RATIO,
        highway_threshold_ratio=HIGHWAY_THRESHOLD_RATIO,
        highway_heat_weight=HIGHWAY_HEAT_WEIGHT,
        highway_distance_weight=HIGHWAY_DISTANCE_WEIGHT,
    )

DRAW_EXECUTION_PATHS_DEFAULT = SHOW_3D_PATHS
GATE_TURN_TOL_DEG = 8.0
GATE_TURN_KP = 5.0
DEFAULT_PATH_STOP_S = 0.05
DEFAULT_GOAL_DIST_TOL = 0.06
DEFAULT_GOAL_REMAINING_S_TOL = 0.10
DEFAULT_GOAL_DONE_DIST_TOL = 0.10
APPROACH_REPLAN_MIN_GAP = 0.75
ENABLE_IDLE_PARKING = True
DEFAULT_PARKING_NO_TASK_GRACE = 2.0
DEFAULT_PARKING_RETRY_INTERVAL = 1.5
DEFAULT_PARKING_TASK_POLL_INTERVAL = 2.0
DEFAULT_PARKING_RING_OFFSET = 0.65
DEFAULT_PARKING_SLOT_COUNT = 12
DEFAULT_COOPERATIVE_CLEAR_AFTER = 2.5
ENABLE_FLEET_DEADLOCK_RECOVERY = True
DEFAULT_FLEET_DEADLOCK_STUCK_DURATION = 3.0
DEFAULT_FLEET_DEADLOCK_ESCAPE_DISTANCE = 1.10
DEFAULT_FLEET_DEADLOCK_RELEASE_HOLD = 0.80
ENABLE_PRIORITY_TRAJECTORY_RESERVATIONS = True
DEFAULT_PRIORITY_RESERVATION_HORIZON = 7.0
DEFAULT_PRIORITY_RESERVATION_MARGIN = 0.16
DEFAULT_PRIORITY_STALLED_AFTER = 0.90
ENABLE_CONGESTION_AWARE_REASSIGNMENT = False
DEFAULT_CONGESTION_WINDOW = 20.0
DEFAULT_CONGESTION_MIN_GATE_PROGRESS = 0.15
DEFAULT_CONGESTION_CONFLICT_FRACTION = 0.50
DEFAULT_CONGESTION_FAILED_REPLANS = 3
DEFAULT_CONGESTION_SAME_BLOCKER_SAMPLES = 5
DEFAULT_CONGESTION_COOLDOWN = 40.0
DEFAULT_CONGESTION_ZONE_RADIUS = 1.35
DEFAULT_CONGESTION_REARM = 15.0
PROTECTED_PHASE_PRIORITIES = {
    # Wide gaps guarantee task phase dominates any rover-type tie-break bias.
    "PUSH": 0.0,
    "TARGET_EXIT": 5.0,
    "TURN_TO_PUSH": 10.0,
    "GROUP_ESCAPE": 15.0,
    "APPROACH": 20.0,
    "ROLLBACK": 30.0,
    "PARKING": 40.0,
    "PLANNING": 45.0,
    "PARKED": 50.0,
    "IDLE": 60.0,
}
APPROACH_REPLAN_KEEP_OUT_RADIUS = 0.32
APPROACH_REPLAN_KEEP_OUT_SAMPLES = 10


class ScheduledTaskPrioritySafety(_shared.DeadlockAwareCollisionSafety):
    """Safety policy that preserves earth-moving task priorities."""

    def priority_for_context(self, ctx: SafetyAgentContext) -> float:
        return float(ctx.priority)

    def with_priorities(self, contexts):
        out = []
        for ctx in contexts:
            out.append(
                SafetyAgentContext(
                    idx=ctx.idx,
                    agent_id=ctx.agent_id,
                    state=ctx.state,
                    active=ctx.active,
                    path_constrained=ctx.path_constrained,
                    path_points=ctx.path_points,
                    goal=ctx.goal,
                    priority=float(ctx.priority),
                    collision_radius=float(ctx.collision_radius),
                )
            )
        return out

    def _choose_deadlock_winner(self, group: Sequence[SafetyAgentContext]):
        return min(group, key=lambda ctx: (float(ctx.priority), int(ctx.idx))) if group else None

    def release_winner_for_target_exit(self, winner_idx: int, sim_time: float):
        """Release a PUSH-era episode before its winner begins target exit."""
        episode = self.episode
        if not episode.active or episode.winner_idx != int(winner_idx):
            return None
        previous = {
            "winner_idx": int(episode.winner_idx),
            "yielder_indices": sorted(int(idx) for idx in episode.yielder_indices),
            "phase": getattr(episode.phase, "value", str(episode.phase)),
        }
        episode.active = False
        self._active_deadlock = False
        self.rearm_until = max(
            float(self.rearm_until),
            float(sim_time) + float(self.config.rearm_delay),
        )
        return previous


def _as_xy_tuple(point) -> Tuple[float, float]:
    return float(point[0]), float(point[1])


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _required_env_radius(points, margin: float = 0.60) -> float:
    max_radius = 0.0
    for point in points:
        if point is None:
            continue
        radius = math.hypot(float(point[0]), float(point[1]))
        if not math.isfinite(radius):
            raise ValueError("non-finite waypoint rejected before A* frame construction")
        max_radius = max(max_radius, radius)
    return max_radius + float(margin)


def _polyline_metrics(points):
    pts = [(float(point[0]), float(point[1])) for point in points]
    if not pts:
        return {"waypoints": 0, "length_m": 0.0, "max_radius_m": 0.0, "max_segment_m": 0.0}
    segments = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    return {
        "waypoints": len(pts),
        "length_m": sum(segments),
        "max_radius_m": max(math.hypot(x, y) for x, y in pts),
        "max_segment_m": max(segments, default=0.0),
    }


def _extend_polyline_end(points, extra_distance: float):
    pts = [np.array(pt, dtype=float) for pt in points]
    extra_distance = max(0.0, float(extra_distance))
    if extra_distance <= 1e-9 or len(pts) < 2:
        return pts, []

    end = pts[-1]
    tangent = None
    for idx in range(len(pts) - 2, -1, -1):
        delta = end - pts[idx]
        dist = float(np.linalg.norm(delta))
        if dist > 1e-6:
            tangent = delta / dist
            break
    if tangent is None:
        return pts, []

    extension_end = end + extra_distance * tangent
    extension = [end.copy(), extension_end]
    return _append_polyline(pts, [extension_end]), extension


def _rover_keepout_points(center_xy, radius=APPROACH_REPLAN_KEEP_OUT_RADIUS):
    center = np.array(center_xy, dtype=float)
    points = [tuple(center)]
    samples = max(4, int(APPROACH_REPLAN_KEEP_OUT_SAMPLES))
    for k in range(samples):
        angle = 2.0 * math.pi * float(k) / float(samples)
        offset = float(radius) * np.array([math.cos(angle), math.sin(angle)], dtype=float)
        points.append(tuple(center + offset))
    return points


def _remove_debug_items(item_ids):
    if not item_ids or not p.isConnected():
        return
    for item_id in item_ids:
        try:
            p.removeUserDebugItem(item_id)
        except Exception:
            pass


def _draw_debug_polyline(points, color, z=0.08, width=2.0, life_time=0.0):
    if not p.isConnected() or points is None or len(points) < 2:
        return []
    ids = []
    for a, b in zip(points[:-1], points[1:]):
        try:
            ids.append(
                p.addUserDebugLine(
                    [float(a[0]), float(a[1]), z],
                    [float(b[0]), float(b[1]), z],
                    color,
                    lineWidth=width,
                    lifeTime=life_time,
                )
            )
        except Exception:
            pass
    return ids


@lru_cache(maxsize=64)
def _target_keepout_mask(zone, grid_w, grid_h, world_xmin, world_ymin,
                         cell_w, cell_h, margin):
    """Build one immutable target mask per geometry/grid instead of per plan."""
    mask = np.zeros((int(grid_h), int(grid_w)), dtype=bool)
    for iy in range(int(grid_h)):
        wy = float(world_ymin) + (iy + 0.5) * float(cell_h)
        for ix in range(int(grid_w)):
            wx = float(world_xmin) + (ix + 0.5) * float(cell_w)
            mask[iy, ix] = zone.contains_world(wx, wy, margin=float(margin))
    mask.setflags(write=False)
    return mask


def _stamp_target_zone_keepout(field, target_zone, margin: float = 0.0) -> None:
    """Stamp a cached exact target-shape mask into an approach obstacle field."""
    zone = resolve_target_zone(target_zone, 0.8)
    mask = _target_keepout_mask(
        zone,
        int(field.grid_w), int(field.grid_h),
        float(field.world_xmin), float(field.world_ymin),
        float(field.cell_w), float(field.cell_h), float(margin),
    )
    np.logical_or(field.obstacles, mask, out=field.obstacles)


def _approach_grid_dim(env_radius: float) -> int:
    cell_size = max(float(ASTAR_APPROACH_CONFIG["cell_size"]), 1e-3)
    grid_dim = int(math.ceil((2.0 * float(env_radius)) / cell_size))
    if grid_dim % 2 == 0:
        grid_dim += 1
    return max(11, grid_dim)


def _build_approach_field(
    env_radius: float,
    pebble_centers: Sequence[Tuple[float, float]],
    target_zone_radius: float,
    block_target_zone: bool,
    rover_radius: float = None,
    target_zone=None,
):
    grid_dim = _approach_grid_dim(env_radius)
    field = build_obstacle_field(
        env_radius,
        grid_dim,
        grid_dim,
        pebble_centers,
        ASTAR_APPROACH_CONFIG["rover_radius"] if rover_radius is None else rover_radius,
        ASTAR_APPROACH_CONFIG["pebble_radius"],
        ASTAR_APPROACH_CONFIG["clearance"],
    )
    if block_target_zone:
        _stamp_target_zone_keepout(
            field,
            target_zone if target_zone is not None else target_zone_radius,
            margin=ASTAR_APPROACH_CONFIG["target_zone_guard_margin"],
        )
    return field


def _plan_approach_astar(
    env_radius: float,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    pebble_centers: Sequence[Tuple[float, float]],
    target_zone_radius: float,
    rover_radius: float = None,
    target_zone=None,
):
    full_field = _build_approach_field(
        env_radius,
        pebble_centers,
        target_zone_radius,
        block_target_zone=True,
        rover_radius=rover_radius,
        target_zone=target_zone,
    )
    raw_path = AStarGridPlanner(full_field).plan(start_xy, goal_xy)
    if raw_path is not None:
        return full_field, raw_path, list(pebble_centers), [], "full_target_keepout"

    if not ASTAR_APPROACH_CONFIG["allow_relaxation"]:
        return full_field, None, list(pebble_centers), [], "failed"

    inflated_radius = (
        (ASTAR_APPROACH_CONFIG["rover_radius"] if rover_radius is None else rover_radius)
        + ASTAR_APPROACH_CONFIG["pebble_radius"]
        + ASTAR_APPROACH_CONFIG["clearance"]
    )
    components = cluster_pebbles_by_distance(
        pebble_centers,
        ASTAR_APPROACH_CONFIG["relax_cluster_link_radius"],
    )
    max_component_size = max((len(comp) for comp in components), default=0)
    max_ignored = ASTAR_APPROACH_CONFIG["relax_max_ignored_cluster_size"]
    if max_ignored is None:
        max_min_cluster_size = max_component_size + 1
    else:
        max_min_cluster_size = min(max_component_size + 1, int(max_ignored) + 1)
    if not ASTAR_APPROACH_CONFIG["relax_progressive"]:
        max_min_cluster_size = int(ASTAR_APPROACH_CONFIG["relax_min_cluster_size"])

    last_field = full_field
    last_kept = list(pebble_centers)
    last_ignored = []

    print(
        "A* approach failed with full obstacles; trying relaxation "
        f"(clusters={len(components)}, inflated_radius={inflated_radius:.3f}m)."
    )

    for keep_threshold in range(
        int(ASTAR_APPROACH_CONFIG["relax_min_cluster_size"]),
        int(max_min_cluster_size) + 1,
    ):
        kept, ignored = split_pebbles_by_cluster_size(
            pebble_centers,
            components,
            min_cluster_size=keep_threshold,
        )
        field = _build_approach_field(
            env_radius,
            kept,
            target_zone_radius,
            block_target_zone=True,
            rover_radius=rover_radius,
            target_zone=target_zone,
        )
        raw_path = AStarGridPlanner(field).plan(start_xy, goal_xy)
        last_field = field
        last_kept = kept
        last_ignored = ignored
        if raw_path is not None:
            mode = f"relaxed_target_keepout_ignore_clusters_le_{keep_threshold - 1}"
            print(f"  A* approach relaxation succeeded: {mode}")
            return field, raw_path, kept, ignored, mode

    return last_field, None, last_kept, last_ignored, "failed"


def _smooth_approach(field, raw_path):
    if raw_path is None or len(raw_path) < 2:
        return None
    smoothed = smooth_path_collision_checked(
        field,
        raw_path,
        iterations=ASTAR_APPROACH_CONFIG["smoothing_iterations"],
        cut=ASTAR_APPROACH_CONFIG["smoothing_cut"],
    )
    trajectory = resample_polyline(
        smoothed,
        spacing=ASTAR_APPROACH_CONFIG["resample_spacing"],
    )
    if not polyline_is_free(field, trajectory):
        trajectory = resample_polyline(
            raw_path,
            spacing=ASTAR_APPROACH_CONFIG["resample_spacing"],
        )
    return [np.array(pt, dtype=float) for pt in trajectory]


def _append_polyline(base, extra, min_separation=0.02):
    out = list(base)
    for pt in extra:
        q = np.array(pt, dtype=float)
        if out and float(np.linalg.norm(q - np.array(out[-1], dtype=float))) < min_separation:
            continue
        out.append(q)
    return out


def _finalize_prebuilt_schedule_plan(
    agent_cfg: Dict,
    planner_field,
    trajectory_points: Sequence[Tuple[float, float]],
    planning_pebbles: Sequence[Tuple[float, float]],
    ignored_pebbles: Sequence[Tuple[float, float]],
    planning_mode: str,
):
    trajectory = resample_polyline(
        [np.array(pt, dtype=float) for pt in trajectory_points],
        spacing=ASTAR_APPROACH_CONFIG["resample_spacing"],
    )
    if len(trajectory) < 2:
        raise RuntimeError(f"{agent_cfg['id']}: hybrid trajectory has fewer than two points.")

    pts, segs, seg_lens, s_cum, total_L = sched.precompute_arc_length(trajectory)
    cell_path = sched.path_cells_from_raw_path(planner_field, trajectory)
    cell_points = sched.cell_points_from_path(planner_field, cell_path)
    cell_s_cum = sched.cell_arc_length(cell_points)
    cell_time_windows = sched.estimate_windows_for_cell_path(cell_points, 0.0)

    eta_lower = float(agent_cfg.get("eta_scale_lower", sched.ETA_TIME_SCALE_LOWER))
    eta_upper = float(agent_cfg.get("eta_scale_upper", sched.ETA_TIME_SCALE_UPPER))
    eta_nominal = 0.5 * (eta_lower + eta_upper)
    (
        T_lower_nodes,
        T_upper_nodes,
        T_nominal_nodes,
        v_nom_nodes,
        landmark_s,
        landmark_T_lower,
        landmark_T_upper,
        landmark_T_nominal,
    ) = sched.precompute_geometric_time_profile(
        pts,
        segs,
        seg_lens,
        s_cum,
        time_scale_lower=eta_lower,
        time_scale_upper=eta_upper,
        time_scale_nominal=eta_nominal,
    )

    first_vec = pts[min(1, len(pts) - 1)] - pts[0]
    start_yaw = 0.0
    if float(np.linalg.norm(first_vec)) > 1e-9:
        start_yaw = math.atan2(float(first_vec[1]), float(first_vec[0]))

    return {
        "raw_path": [np.array(pt, dtype=float) for pt in trajectory],
        "shortcut_path": [],
        "trajectory": [np.array(pt, dtype=float) for pt in trajectory],
        "path_base_name": "hybrid_astar_task_path",
        "planning_mode": planning_mode,
        "planning_pebbles": list(planning_pebbles),
        "ignored_pebbles": list(ignored_pebbles),
        "cell_path": cell_path,
        "cell_points": cell_points,
        "cell_s_cum": cell_s_cum,
        "cell_time_windows": cell_time_windows,
        "cell_w": float(planner_field.cell_w),
        "cell_h": float(planner_field.cell_h),
        "eta_scale_lower": eta_lower,
        "eta_scale_upper": eta_upper,
        "geom": {
            "pts": pts,
            "segs": segs,
            "seg_lens": seg_lens,
            "s_cum": s_cum,
            "total_L": total_L,
        },
        "start_yaw": start_yaw,
        "T_lower_nodes": T_lower_nodes,
        "T_upper_nodes": T_upper_nodes,
        "T_nominal_nodes": T_nominal_nodes,
        "v_nom_nodes": v_nom_nodes,
        "landmark_s": landmark_s,
        "landmark_T_lower": landmark_T_lower,
        "landmark_T_upper": landmark_T_upper,
        "landmark_T_nominal": landmark_T_nominal,
        "T_total_lower": float(T_lower_nodes[-1]),
        "T_total_upper": float(T_upper_nodes[-1]),
        "T_total_nominal": float(T_nominal_nodes[-1]),
    }


def _build_schedule_agent_from_points(
    agent_id: str,
    priority: float,
    color,
    trajectory_points: Sequence[Tuple[float, float]],
    env_radius: float,
    planning_pebbles: Sequence[Tuple[float, float]],
    ignored_pebbles: Sequence[Tuple[float, float]],
    planning_mode: str,
    allow_replan: bool,
    rover_radius: float = None,
):
    if len(trajectory_points) < 2:
        raise RuntimeError(f"{agent_id}: schedule trajectory must have at least two points.")

    planning_field = _build_approach_field(
        env_radius,
        planning_pebbles,
        target_zone_radius=0.0,
        block_target_zone=False,
        rover_radius=rover_radius,
    )
    start = np.array(trajectory_points[0], dtype=float)
    goal = np.array(trajectory_points[-1], dtype=float)
    agent_cfg = {
        "id": agent_id,
        "start": start,
        "goal": goal,
        "priority": float(priority),
        "allow_replan": bool(allow_replan),
        "eta_scale_lower": sched.ETA_TIME_SCALE_LOWER,
        "eta_scale_upper": sched.ETA_TIME_SCALE_UPPER,
        "color": color,
        "path_color": sched.path_color_from_rgba(color),
    }
    plan = _finalize_prebuilt_schedule_plan(
        agent_cfg,
        planning_field,
        trajectory_points,
        planning_pebbles,
        ignored_pebbles,
        planning_mode,
    )
    plan["timing"] = {"total_plan": 0.0}
    return {
        "id": agent_id,
        "start": start.copy(),
        "goal": goal.copy(),
        "priority": float(priority),
        "allow_replan": bool(allow_replan),
        "eta_scale_lower": sched.ETA_TIME_SCALE_LOWER,
        "eta_scale_upper": sched.ETA_TIME_SCALE_UPPER,
        "color": color,
        "path_color": sched.path_color_from_rgba(color),
        "hard_static_pebbles": list(planning_pebbles),
        **plan,
    }


def _build_hybrid_astar_schedule_agent(
    agent_id: str,
    priority: float,
    color,
    start_xy: Tuple[float, float],
    gate_xy: Tuple[float, float],
    push_world_pts: Sequence[Tuple[float, float]],
    env_radius: float,
    target_zone_radius: float,
    pebble_centers: Sequence[Tuple[float, float]],
    allow_replan: bool,
    rover_radius: float = None,
    target_zone=None,
):
    if len(push_world_pts) < 2:
        raise RuntimeError(f"{agent_id}: push path must have at least two points.")

    zone = resolve_target_zone(target_zone, target_zone_radius)
    start_clearance = zone.signed_distance_world(*start_xy)
    required_clearance = ASTAR_APPROACH_CONFIG["target_zone_guard_margin"]
    if start_clearance < required_clearance:
        raise TargetKeepoutBlocked(
            agent_id, start_clearance, required_clearance,
        )

    approach_field, raw_approach, planning_pebbles, ignored_pebbles, mode = (
        _plan_approach_astar(
            env_radius,
            start_xy,
            gate_xy,
            pebble_centers,
            target_zone_radius,
            rover_radius=rover_radius,
            target_zone=target_zone,
        )
    )

    if raw_approach is None:
        print(f"[A*] {agent_id}: approach A* failed; using direct fallback to gate.")
        approach_points = [np.array(start_xy, dtype=float), np.array(gate_xy, dtype=float)]
        mode = "direct_fallback"
    else:
        approach_points = _smooth_approach(approach_field, raw_approach)

    schedule_agent = _build_schedule_agent_from_points(
        agent_id,
        priority,
        color,
        approach_points,
        env_radius,
        planning_pebbles,
        ignored_pebbles,
        f"approach_{mode}",
        allow_replan,
    )
    schedule_agent["debug_approach_points"] = [np.array(pt, dtype=float) for pt in approach_points]
    schedule_agent["debug_connector_points"] = [
        np.array(gate_xy, dtype=float),
        np.array(push_world_pts[0], dtype=float),
    ]
    schedule_agent["debug_push_points"] = [np.array(pt, dtype=float) for pt in push_world_pts]
    return schedule_agent


def _init_scheduler_runtime(schedule_agent: Dict, agent: Dict, sim_time: float) -> None:
    schedule_agent["body"] = agent["body_id"]
    schedule_agent["left_joint"] = agent["left_joint"]
    schedule_agent["right_joint"] = agent["right_joint"]
    schedule_agent["state"] = _base.get_state(agent["body_id"])
    profile = agent.get("rover_profile")
    schedule_agent["tracking_offset"] = float(
        profile.shovel_offset if profile is not None else sched.SHOVEL_OFFSET
    )
    shovel_width = float(
        profile.shovel_width if profile is not None else 2.0 * sched.ENDPOINT_PASS_CROSS_TRACK_TOL
    )
    schedule_agent["endpoint_cross_track_tol"] = max(
        float(sched.GOAL_DONE_DIST_TOL), 0.5 * shovel_width
    )
    schedule_agent["endpoint_reacquiring"] = False
    schedule_agent["control"] = (0.0, 0.0)
    schedule_agent["s"] = 0.0
    schedule_agent["s_prev"] = 0.0
    schedule_agent["cell_idx"] = 0
    schedule_agent["d_path"] = 0.0
    schedule_agent["path_proj"] = schedule_agent["geom"]["pts"][0]
    schedule_agent["path_tangent"] = sched.tangent_at_s(schedule_agent["geom"], 0.0)
    schedule_agent["time_prev"] = None
    schedule_agent["v_s_ema"] = 0.0
    schedule_agent["speed_factor"] = 1.0
    schedule_agent["yield_mode"] = "normal"
    schedule_agent["hold_s"] = None
    schedule_agent["emergency_stop"] = False
    schedule_agent["emergency_blocker"] = None
    schedule_agent["emergency_reason"] = None
    schedule_agent["emergency_details"] = {}
    schedule_agent["last_emergency_print_time"] = -1e9
    schedule_agent["backoff_until"] = 0.0
    schedule_agent["backoff_from_id"] = None
    schedule_agent["backoff_from_xy"] = None
    schedule_agent["backoff_last_trigger_time"] = -1e9
    schedule_agent["backoff_reason"] = None
    schedule_agent["wait_until"] = 0.0
    schedule_agent["yield_winner_idx"] = None
    schedule_agent["winner_release_s"] = None
    schedule_agent["yield_blockers"] = []
    schedule_agent["last_replan_time"] = float(sim_time)
    schedule_agent["last_replan_request_time"] = float(sim_time)
    schedule_agent["replan_future"] = None
    schedule_agent["pending_replan"] = None
    schedule_agent["route_refresh_future"] = None
    schedule_agent["cell_refresh_future"] = None
    schedule_agent["last_route_refresh_time"] = float(sim_time)
    schedule_agent["last_cell_refresh_time"] = float(sim_time)
    schedule_agent["time_reserved_holds"] = list(schedule_agent.get("time_reserved_holds", []))
    schedule_agent["time_reserved_hold"] = False
    schedule_agent["clearing_winner_path"] = False
    schedule_agent["clear_winner_ids"] = []
    schedule_agent["plan_debug_items"] = []
    schedule_agent["replan_debug_items"] = []
    schedule_agent["done"] = False
    schedule_agent["actual_dist"] = 0.0
    schedule_agent["prev_xy"] = np.array(schedule_agent["state"][:2], dtype=float)


class MultiAStarScheduledHybridOrchestrator(_shared.SharedPlanningDeadlockOrchestrator):
    def __init__(
        self,
        *args,
        scheduler_replan_workers: int = 0,
        scheduler_allow_replans: bool = False,
        draw_scheduler_conflicts: bool = False,
        draw_execution_paths: bool = DRAW_EXECUTION_PATHS_DEFAULT,
        draw_planning_boundaries: bool = SHOW_3D_PLANNING_BOUNDARIES,
        push_extra_distance: float = DEFAULT_PUSH_EXTRA_DISTANCE,
        approach_replan_interval: float = DEFAULT_APPROACH_REPLAN_INTERVAL,
        navigation_outside_margin: float = DEFAULT_NAVIGATION_OUTSIDE_MARGIN,
        enable_idle_parking: bool = ENABLE_IDLE_PARKING,
        parking_no_task_grace: float = DEFAULT_PARKING_NO_TASK_GRACE,
        parking_retry_interval: float = DEFAULT_PARKING_RETRY_INTERVAL,
        parking_task_poll_interval: float = DEFAULT_PARKING_TASK_POLL_INTERVAL,
        parking_ring_offset: float = DEFAULT_PARKING_RING_OFFSET,
        parking_slot_count: int = DEFAULT_PARKING_SLOT_COUNT,
        cooperative_clear_after: float = DEFAULT_COOPERATIVE_CLEAR_AFTER,
        enable_fleet_deadlock_recovery: bool = ENABLE_FLEET_DEADLOCK_RECOVERY,
        fleet_deadlock_stuck_duration: float = DEFAULT_FLEET_DEADLOCK_STUCK_DURATION,
        fleet_deadlock_escape_distance: float = DEFAULT_FLEET_DEADLOCK_ESCAPE_DISTANCE,
        enable_priority_reservations: bool = ENABLE_PRIORITY_TRAJECTORY_RESERVATIONS,
        priority_reservation_horizon: float = DEFAULT_PRIORITY_RESERVATION_HORIZON,
        priority_reservation_margin: float = DEFAULT_PRIORITY_RESERVATION_MARGIN,
        priority_stalled_after: float = DEFAULT_PRIORITY_STALLED_AFTER,
        enable_congestion_reassignment: bool = ENABLE_CONGESTION_AWARE_REASSIGNMENT,
        congestion_window: float = DEFAULT_CONGESTION_WINDOW,
        congestion_min_gate_progress: float = DEFAULT_CONGESTION_MIN_GATE_PROGRESS,
        congestion_conflict_fraction: float = DEFAULT_CONGESTION_CONFLICT_FRACTION,
        congestion_failed_replans: int = DEFAULT_CONGESTION_FAILED_REPLANS,
        congestion_same_blocker_samples: int = DEFAULT_CONGESTION_SAME_BLOCKER_SAMPLES,
        congestion_cooldown: float = DEFAULT_CONGESTION_COOLDOWN,
        congestion_zone_radius: float = DEFAULT_CONGESTION_ZONE_RADIUS,
        congestion_rearm: float = DEFAULT_CONGESTION_REARM,
        auto_exit_on_completion: bool = False,
        max_sim_time: Optional[float] = None,
        max_no_delivery_time: Optional[float] = None,
        event_log_dir: Optional[str] = None,
        event_log_pose_interval: float = DEFAULT_EVENT_LOG_POSE_INTERVAL,
        enable_event_log: bool = ENABLE_EVENT_LOG,
        enable_benchmark_telemetry: bool = ENABLE_BENCHMARK_TELEMETRY,
        benchmark_capture_images: bool = BENCHMARK_CAPTURE_IMAGES,
        benchmark_capture_every_push: bool = BENCHMARK_CAPTURE_EVERY_PUSH,
        benchmark_save_snapshots: bool = BENCHMARK_SAVE_COMPRESSED_SNAPSHOTS,
        benchmark_observation_delay: float = BENCHMARK_POST_PUSH_OBSERVATION_DELAY,
        benchmark_corridor_margin: float = BENCHMARK_PUSH_CORRIDOR_MARGIN,
        benchmark_reference_highway_ratio: float = BENCHMARK_REFERENCE_HIGHWAY_RATIO,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.safety = ScheduledTaskPrioritySafety(self.safety.config)
        # The cooperative manager below owns reverse recovery in this runner.
        if hasattr(self.safety.config, "enable_reverse_recovery"):
            self.safety.config.enable_reverse_recovery = False
        sched.BACKOFF_ESCAPE_ENABLED = False
        self.navigation_outside_margin = max(0.0, float(navigation_outside_margin))
        self.cooperative_recovery = CooperativeRecoveryManager(
            CooperativeRecoveryConfig(
                navigation_outside_margin=self.navigation_outside_margin,
                cooperative_clear_after=max(0.5, float(cooperative_clear_after)),
            )
        )
        self.enable_fleet_deadlock_recovery = bool(enable_fleet_deadlock_recovery)
        self.fleet_deadlock_escape_distance = max(
            0.65, float(fleet_deadlock_escape_distance),
        )
        self.fleet_recovery = FleetDeadlockRecoveryManager(
            FleetDeadlockConfig(
                # Match the pairwise release envelope, which is stricter than
                # the reservation margin and closes the former dead band.
                trigger_clearance=0.24,
                release_clearance=max(0.36, float(priority_reservation_margin) + 0.12),
                stuck_duration=max(1.0, float(fleet_deadlock_stuck_duration)),
                release_hold_time=DEFAULT_FLEET_DEADLOCK_RELEASE_HOLD,
            )
        )
        self.enable_priority_reservations = bool(enable_priority_reservations)
        self.priority_reservations = PrioritizedTrajectoryCoordinator(
            PrioritizedReservationConfig(
                horizon_s=max(1.0, float(priority_reservation_horizon)),
                safety_margin=max(0.02, float(priority_reservation_margin)),
                stalled_after=max(0.25, float(priority_stalled_after)),
            )
        )
        self._priority_reservation_holds = set()
        self._priority_reservation_conflict_pairs = set()
        self._priority_reservation_conflict_indices = set()
        self._last_priority_reservation_signature = None
        self.congestion_config = CongestionConfig(
            enabled=bool(enable_congestion_reassignment),
            window_s=max(5.0, float(congestion_window)),
            minimum_task_age_s=max(2.0, 0.5 * float(congestion_window)),
            minimum_gate_progress_m=max(0.01, float(congestion_min_gate_progress)),
            conflict_fraction=max(0.0, min(1.0, float(congestion_conflict_fraction))),
            failed_replans=max(1, int(congestion_failed_replans)),
            same_blocker_samples=max(1, int(congestion_same_blocker_samples)),
            cooldown_s=max(1.0, float(congestion_cooldown)),
            exclusion_radius_m=max(0.25, float(congestion_zone_radius)),
            rearm_s=max(1.0, float(congestion_rearm)),
        )
        self.congestion_monitor = ApproachCongestionMonitor(self.congestion_config)
        self._congestion_zones = []
        self._congestion_staging_agents = set()
        self.enable_idle_parking = bool(enable_idle_parking)
        self.parking_no_task_grace = max(0.0, float(parking_no_task_grace))
        self.parking_retry_interval = max(0.25, float(parking_retry_interval))
        self.parking_task_poll_interval = max(0.50, float(parking_task_poll_interval))
        self.parking_ring_offset = max(0.10, float(parking_ring_offset))
        self.parking_slot_count = max(4, int(parking_slot_count))
        self._parking_slot_owners = {}
        self.scheduler_replan_workers = max(0, int(scheduler_replan_workers))
        self.scheduler_allow_replans = bool(scheduler_allow_replans)
        self.draw_scheduler_conflicts = bool(draw_scheduler_conflicts)
        self.draw_execution_paths = bool(draw_execution_paths)
        self.draw_planning_boundaries = bool(draw_planning_boundaries)
        self.push_extra_distance = max(0.0, float(push_extra_distance))
        self.approach_replan_interval = max(0.0, float(approach_replan_interval))
        self.replan_executor = None
        self._sim_time = 0.0
        self._scheduler_acc = 0.0
        self._last_scheduler_status_print = -10.0
        self._last_task_backoff_print = -10.0
        self._pending_execution_map_refresh = False
        self._pending_execution_map_refresh_agents = set()
        self._last_execution_map_refresh_print = -10.0
        self._consumed_cells_in_submitted_refresh = set()
        self._last_telemetry_time = -1e9
        self._last_interaction_signatures = {}
        self._last_recovery_signature = None
        self._last_recovery_result = None
        self._last_fleet_recovery_signature = None
        self._last_fleet_recovery_result = None
        self._previous_logged_positions = {}
        self._quarantined_agents = set()
        self._outside_nominal_agents = set()
        self._nominal_boundary_debug_items = []
        self._planning_boundary_debug_items = []
        self._unfinished_material_signature = None
        self._unfinished_material_refreshes = 0
        self.auto_exit_on_completion = bool(auto_exit_on_completion)
        self.max_sim_time = (
            None if max_sim_time is None or float(max_sim_time) <= 0.0
            else float(max_sim_time)
        )
        self.max_no_delivery_time = (
            None
            if max_no_delivery_time is None or float(max_no_delivery_time) <= 0.0
            else float(max_no_delivery_time)
        )
        self._last_delivered_count = None
        self._last_delivery_progress_time = 0.0
        self._run_outcome = None
        self.event_logger = None
        self.benchmark_telemetry = None
        self._benchmark_last_map_epoch = None
        self._benchmark_material_complete_logged = False
        self._benchmark_all_parked_logged = False
        if enable_event_log:
            try:
                self.event_logger = SimulationEventLogger(
                    event_log_dir or os.path.join(HERE, "simulation_logs"),
                    pose_interval=event_log_pose_interval,
                )
            except Exception as exc:
                print(f"[EVENT-LOG] Could not create event log: {exc}")
        if enable_benchmark_telemetry and self.event_logger is not None:
            try:
                self.benchmark_telemetry = BenchmarkTelemetry(
                    self.event_logger.log_dir,
                    self.event_logger.run_id,
                    event_callback=self._benchmark_event,
                    capture_images=benchmark_capture_images,
                    capture_push_images=benchmark_capture_every_push,
                    save_snapshot_data=benchmark_save_snapshots,
                    observation_delay_s=benchmark_observation_delay,
                    corridor_margin_m=benchmark_corridor_margin,
                    reference_highway_ratio=benchmark_reference_highway_ratio,
                    milestones=BENCHMARK_DELIVERY_MILESTONES,
                    image_dpi=BENCHMARK_IMAGE_DPI,
                )
                with self.event_logger.latest_path.open("a", encoding="utf-8") as latest:
                    latest.write(f"PUSHES_CSV={self.benchmark_telemetry.pushes_path}\n")
                    latest.write(f"SNAPSHOTS_CSV={self.benchmark_telemetry.snapshots_path}\n")
                    latest.write(f"RUN_SUMMARY_JSON={self.benchmark_telemetry.run_summary_path}\n")
                    latest.write(f"ARTIFACT_DIR={self.benchmark_telemetry.artifact_dir}\n")
            except Exception as exc:
                self.benchmark_telemetry = None
                print(f"[BENCHMARK] Could not create benchmark telemetry: {exc!r}")
                self.event_logger.event(
                    "BENCHMARK_INITIALIZATION_FAILED", self._sim_time,
                    error_type=type(exc).__name__, error=repr(exc),
                    requested_log_dir=str(self.event_logger.log_dir),
                )

    def _log_event(self, event_type, agent=None, **data):
        if self.event_logger is None:
            return
        agent_id = None if agent is None else str(agent.get("id"))
        self.event_logger.event(event_type, self._sim_time, agent_id, **data)

    def _benchmark_event(self, event_type, sim_time, **data):
        if self.event_logger is None:
            return
        agent_id = data.pop("agent_id", None)
        self.event_logger.event(event_type, sim_time, agent_id, **data)

    def _pebble_physics_states(self):
        """Stable per-body physics snapshot used for contact attribution."""
        states = {}
        for bullet_id in self.pebble_ids:
            try:
                position, _ = p.getBasePositionAndOrientation(bullet_id)
                velocity, _ = p.getBaseVelocity(bullet_id)
            except Exception:
                continue
            if float(position[2]) <= -0.5:
                continue
            instance = self.pebble_instances_by_id[bullet_id]
            logical_id = int(instance.index)
            x, y = float(position[0]), float(position[1])
            states[logical_id] = {
                "pebble_id": logical_id,
                "bullet_id": int(bullet_id),
                "profile": instance.profile_name,
                "material_mass": int(instance.material_mass),
                "visual_scale": float(instance.visual_scale),
                "position": [x, y, float(position[2])],
                "velocity": [float(velocity[0]), float(velocity[1]), float(velocity[2])],
                "speed_m_s": math.hypot(float(velocity[0]), float(velocity[1])),
                "in_target": bool(self.target_zone.contains_world(x, y)),
                "signed_target_distance_m": float(self.target_zone.signed_distance_world(x, y)),
            }
        return states

    @staticmethod
    def _shovel_link_indices(body_id):
        links = set()
        try:
            for index in range(p.getNumJoints(body_id)):
                info = p.getJointInfo(body_id, index)
                names = (
                    info[1].decode("utf-8", errors="ignore").lower(),
                    info[12].decode("utf-8", errors="ignore").lower(),
                )
                if any(token in name for name in names for token in ("shovel", "blade", "bucket")):
                    links.add(index)
        except Exception:
            pass
        return links

    def _record_benchmark_contacts(self):
        telemetry = self.benchmark_telemetry
        if telemetry is None or not telemetry.active_pushes:
            return
        logical_by_body = {
            int(body_id): int(instance.index)
            for body_id, instance in self.pebble_instances_by_id.items()
        }
        for agent in self.agents:
            agent_id = str(agent["id"])
            if agent_id not in telemetry.active_pushes:
                continue
            contacts = set()
            shovel_contacts = set()
            shovel_links = agent.setdefault(
                "benchmark_shovel_links", self._shovel_link_indices(agent["body_id"])
            )
            try:
                points = p.getContactPoints(bodyA=agent["body_id"])
            except Exception:
                points = ()
            for point in points:
                logical_id = logical_by_body.get(int(point[2]))
                if logical_id is None:
                    continue
                contacts.add(logical_id)
                if int(point[3]) in shovel_links:
                    shovel_contacts.add(logical_id)
            telemetry.record_contacts(agent_id, contacts, shovel_contacts)

    def _benchmark_rover_states(self):
        rows = []
        for agent in self.agents:
            state = agent.get("state_val")
            if state is None:
                state = _base.get_state(agent["body_id"])
            rows.append({
                "agent_id": str(agent["id"]),
                "rover_type": getattr(agent.get("rover_profile"), "name", "legacy"),
                "state": agent.get("state"), "phase": agent.get("astar_phase"),
                "pose": [float(value) for value in state], "color": agent.get("color"),
                "task_id": agent.get("event_task_id"),
            })
        return rows

    def _benchmark_active_paths(self):
        rows = []
        for agent in self.agents:
            selection = agent.get("selection") or {}
            points = selection.get("world_pts") or []
            if points:
                rows.append({
                    "agent_id": str(agent["id"]), "task_id": agent.get("event_task_id"),
                    "task_type": selection.get("path_type"), "source_cell": selection.get("cell"),
                    "points": [[float(point[0]), float(point[1])] for point in points],
                    "color": agent.get("color"),
                })
        return rows

    def _capture_benchmark_snapshot(self, reason, force_image=False):
        if self.benchmark_telemetry is None or self.env_2d is None:
            return None
        progress = self._get_material_progress()
        progress["material_value_mode"] = self.material_value_mode
        epoch = int(self.shared_map.epoch) if self.shared_map is not None else None
        polygon = list(self.target_zone.geometry.exterior.coords)
        return self.benchmark_telemetry.record_snapshot(
            reason, self._sim_time, self.env_2d, epoch, progress,
            self._pebble_physics_states(), self._benchmark_rover_states(),
            self._benchmark_active_paths(), dict(_planning_2d.PLANNING_HYPERPARAMETERS),
            float(getattr(self.coord_converter, "env_radius", self.env_radius)), polygon,
            force_image=force_image,
        )

    def _update_benchmark_lifecycle(self):
        telemetry = self.benchmark_telemetry
        if telemetry is not None:
            pebble_states = self._pebble_physics_states()
            completed = telemetry.observe_due(self._sim_time, pebble_states)
            if completed and telemetry.capture_push_images:
                self._capture_benchmark_snapshot("after_push_settled", force_image=True)
        progress = self._get_material_progress()
        delivered_count = int(progress.get("delivered_count", 0))
        if self._last_delivered_count is None or delivered_count > self._last_delivered_count:
            self._last_delivered_count = delivered_count
            self._last_delivery_progress_time = float(self._sim_time)
        total = int(progress.get("delivered_count", 0)) + int(progress.get("remaining_count", 0))
        fraction = float(progress.get("delivered_count", 0)) / total if total else 1.0
        if telemetry is not None:
            for reason in telemetry.milestone_reasons(fraction):
                self._capture_benchmark_snapshot(reason, force_image=True)
        if int(progress.get("remaining_count", 0)) == 0 and not self._benchmark_material_complete_logged:
            self._benchmark_material_complete_logged = True
            self._log_event("MISSION_MATERIAL_COMPLETE", material_progress=progress)
            self._capture_benchmark_snapshot("all_material_delivered", force_image=True)
        all_parked = bool(self.agents) and all(agent.get("state") == "PARKED" for agent in self.agents)
        if self._benchmark_material_complete_logged and all_parked and not self._benchmark_all_parked_logged:
            self._benchmark_all_parked_logged = True
            self._log_event("MISSION_COMPLETE_ALL_PARKED", material_progress=progress)
            self._capture_benchmark_snapshot("all_rovers_parked", force_image=True)
            if self.auto_exit_on_completion:
                self._run_outcome = "mission_complete"
                self.running = False
                print("[MISSION] All material delivered and all rovers parked; closing run.")
                return
        if (
            self._run_outcome is None
            and self.max_no_delivery_time is not None
            and not self._benchmark_material_complete_logged
            and self._sim_time - self._last_delivery_progress_time
            >= self.max_no_delivery_time
        ):
            self._run_outcome = "stalled_no_delivery_progress"
            self._log_event(
                "MISSION_STOPPED_NO_DELIVERY_PROGRESS",
                material_progress=progress,
                no_delivery_progress_s=(
                    self._sim_time - self._last_delivery_progress_time
                ),
                limit_s=self.max_no_delivery_time,
            )
            self.running = False
            print("[MISSION] No-delivery-progress limit reached; closing run.")
            return
        if (
            self._run_outcome is None
            and self.max_sim_time is not None
            and self._sim_time >= self.max_sim_time
        ):
            self._run_outcome = "max_sim_time"
            self._log_event(
                "MISSION_STOPPED_MAX_SIM_TIME",
                material_progress=progress,
                limit_s=self.max_sim_time,
            )
            self.running = False
            print("[MISSION] Maximum simulated time reached; closing run.")

    def _boundary_circle_points(self, radius, segments=96):
        radius = max(0.0, float(radius))
        points = [
            (
                radius * math.cos(2.0 * math.pi * index / float(segments)),
                radius * math.sin(2.0 * math.pi * index / float(segments)),
            )
            for index in range(int(segments))
        ]
        return points + points[:1]

    def _refresh_planning_envelope_visual(self, planning_radius):
        _remove_debug_items(self._nominal_boundary_debug_items)
        _remove_debug_items(self._planning_boundary_debug_items)
        self._nominal_boundary_debug_items = []
        self._planning_boundary_debug_items = []
        planning_radius = max(float(self.env_radius), float(planning_radius))
        if self.draw_planning_boundaries:
            self._nominal_boundary_debug_items = _draw_debug_polyline(
                self._boundary_circle_points(self.env_radius),
                (0.35, 0.35, 0.35),
                z=0.015,
                width=1.5,
            )
            self._planning_boundary_debug_items = _draw_debug_polyline(
                self._boundary_circle_points(planning_radius),
                (0.10, 0.85, 0.95),
                z=0.02,
                width=2.5,
            )
        self._log_event(
            "PLANNING_ENVELOPE_UPDATED",
            nominal_radius_m=float(self.env_radius),
            planning_radius_m=planning_radius,
            visible=bool(self.draw_planning_boundaries),
        )

    def initialize(self):
        super().initialize()
        self._refresh_planning_envelope_visual(
            float(getattr(self.coord_converter, "env_radius", self.env_radius))
        )
        for agent in self.agents:
            agent["no_task_since"] = None
            agent["next_task_allocation_time"] = 0.0
            agent["parking_slot_index"] = None
            agent["parking_goal"] = None
            agent["parking_probe_future"] = None
            agent["parking_probe_id"] = 0
            agent["next_parking_probe_time"] = 0.0
            agent["parking_debug_items"] = []
            agent["target_exit_extended_logged"] = False
            agent["target_exit_boundary"] = None
            agent["target_exit_direction"] = None
            agent["target_exit_goal"] = None
            agent["target_exit_path"] = []
            agent["priority_reserved_blocker_points"] = []
            agent["priority_reservation_reason"] = None
            agent["priority_replan_failures"] = 0
            agent["congestion_failed_replans_total"] = 0
            agent["congestion_task_started_at"] = 0.0
            agent["pending_target_exit_recovery"] = None
            agent["path_rejection_count"] = 0
            agent["group_escape_goal"] = None
            agent["group_escape_route"] = []
            agent["group_recovery_task_aborted"] = False
            agent["group_recovery_original_phase"] = None
            agent["group_recovery_map_refresh"] = False
        if self.scheduler_allow_replans and self.scheduler_replan_workers > 0:
            self.replan_executor = cf.ProcessPoolExecutor(
                max_workers=self.scheduler_replan_workers
            )
            try:
                warmups = [
                    self.replan_executor.submit(sched.planner_warmup)
                    for _ in range(self.scheduler_replan_workers)
                ]
                for future in warmups:
                    future.result(timeout=8.0)
            except Exception as exc:
                print(f"[SCHED] Replan worker warmup failed; continuing without workers: {exc}")
                self.replan_executor.shutdown(wait=False, cancel_futures=True)
                self.replan_executor = None
        print(
            "[SCHED] Multi-A* scheduled control ready "
            f"(replan_workers={self.scheduler_replan_workers if self.replan_executor else 0}, "
            f"replans={self.scheduler_allow_replans}, "
            f"push_extra={self.push_extra_distance:.2f}m, "
            f"approach_replan={self.approach_replan_interval:.1f}s, "
            f"outside_margin={self.navigation_outside_margin:.2f}m, "
            f"idle_parking={self.enable_idle_parking}, "
            f"parking_grace={self.parking_no_task_grace:.1f}s, "
            f"cooperative_clear={self.cooperative_recovery.config.cooperative_clear_after:.1f}s, "
            f"fleet_deadlock={self.enable_fleet_deadlock_recovery}, "
            f"fleet_stuck={self.fleet_recovery.config.stuck_duration:.1f}s, "
            f"priority_reservations={self.enable_priority_reservations}, "
            f"reservation_horizon={self.priority_reservations.config.horizon_s:.1f}s, "
            f"congestion_reassignment={self.congestion_config.enabled}, "
            f"congestion_window={self.congestion_config.window_s:.1f}s, "
            f"draw_paths={self.draw_execution_paths}, "
            f"boundary_rings={self.draw_planning_boundaries})"
        )
        if self.scheduler_allow_replans:
            print(
                "[SCHED] WARNING: scheduler replans are experimental in the hybrid "
                "pushing runner because the original replanner targets only the final "
                "goal and can bypass the selected push corridor."
            )
        if self.event_logger is not None:
            print(f"[EVENT-LOG] Detailed events: {self.event_logger.jsonl_path}")
            print(f"[EVENT-LOG] Task summary:    {self.event_logger.summary_path}")
            self._log_event(
                "RUN_CONFIG",
                rovers=[{
                    "agent_id": agent["id"],
                    "rover_type": getattr(agent.get("rover_profile"), "name", "legacy"),
                    "initial_pose": self.initial_robot_poses[int(agent["index"])],
                    "right_of_way_priority": getattr(agent.get("rover_profile"), "right_of_way_priority", None),
                    "task_policy": _policy_log_config(agent["rover_profile"].policy),
                } for agent in self.agents],
                planning_2d=dict(_planning_2d.PLANNING_HYPERPARAMETERS),
                material_value_mode=self.material_value_mode,
                pebble_distribution=self.pebble_distribution,
                pebble_deployment=self.pebble_deployment,
                pebble_summary=summarize_instances(self.pebble_instances),
                initial_pebble_states=self._pebble_physics_states(),
                initial_material_progress=self._get_material_progress(),
                env_radius=self.env_radius,
                shared_map_radius=float(self.shared_map.env_radius),
                planning_boundary_rings_visible=self.draw_planning_boundaries,
                navigation_outside_margin=self.navigation_outside_margin,
                idle_parking_enabled=self.enable_idle_parking,
                parking_no_task_grace=self.parking_no_task_grace,
                parking_task_poll_interval=self.parking_task_poll_interval,
                parking_ring_offset=self.parking_ring_offset,
                parking_slot_count=self.parking_slot_count,
                cooperative_clear_after=self.cooperative_recovery.config.cooperative_clear_after,
                fleet_deadlock_recovery=self.enable_fleet_deadlock_recovery,
                fleet_deadlock_stuck_duration_s=self.fleet_recovery.config.stuck_duration,
                fleet_deadlock_escape_distance_m=self.fleet_deadlock_escape_distance,
                fleet_deadlock_trigger_clearance_m=self.fleet_recovery.config.trigger_clearance,
                fleet_deadlock_release_clearance_m=self.fleet_recovery.config.release_clearance,
                priority_trajectory_reservations=self.enable_priority_reservations,
                priority_reservation_horizon_s=self.priority_reservations.config.horizon_s,
                priority_reservation_margin_m=self.priority_reservations.config.safety_margin,
                priority_stalled_after_s=self.priority_reservations.config.stalled_after,
                congestion_reassignment={
                    "enabled": self.congestion_config.enabled,
                    "window_s": self.congestion_config.window_s,
                    "minimum_task_age_s": self.congestion_config.minimum_task_age_s,
                    "minimum_gate_progress_m": self.congestion_config.minimum_gate_progress_m,
                    "conflict_fraction": self.congestion_config.conflict_fraction,
                    "failed_replans": self.congestion_config.failed_replans,
                    "same_blocker_samples": self.congestion_config.same_blocker_samples,
                    "cooldown_s": self.congestion_config.cooldown_s,
                    "exclusion_radius_m": self.congestion_config.exclusion_radius_m,
                    "rearm_s": self.congestion_config.rearm_s,
                },
                target_root_sources_only=self.target_root_sources_only,
                target_source_mode=self.target_source_mode,
                target_path_mode=self.target_path_mode,
                target_candidate_value_mode=self.target_candidate_value_mode,
                capacity_scoring_mode=_planning_overlay.CAPACITY_SCORING_MODE,
                comparison_label=self.comparison_label,
                runtime_mode=(
                    "headless_fast" if getattr(self, "fast_headless", False)
                    else "headless" if getattr(self, "headless", False)
                    else "gui"
                ),
                synchronous_planning=self.synchronous_planning,
                auto_exit_on_completion=self.auto_exit_on_completion,
                max_sim_time_s=self.max_sim_time,
                max_no_delivery_time_s=self.max_no_delivery_time,
                target_zone_radius=self.target_zone_radius,
                target_zone=self.target_zone.to_dict(),
                scenario_name=self.scenario_name,
                scenario=getattr(self, "scenario_config", None),
                pose_interval=self.event_logger.pose_interval,
                benchmark_telemetry=(
                    self.benchmark_telemetry.config_dict()
                    if self.benchmark_telemetry is not None else None
                ),
            )
        if self.benchmark_telemetry is not None:
            print(f"[BENCHMARK] Push metrics: {self.benchmark_telemetry.pushes_path}")
            print(f"[BENCHMARK] Snapshots:    {self.benchmark_telemetry.snapshots_path}")
            self._benchmark_last_map_epoch = int(self.shared_map.epoch)
            self._capture_benchmark_snapshot("initial_map", force_image=True)

    def run(self):
        try:
            super().run()
        except Exception:
            self._run_outcome = "error"
            raise
        finally:
            if self.replan_executor is not None:
                self.replan_executor.shutdown(wait=False, cancel_futures=True)
            final_progress = self._get_material_progress()
            if self.benchmark_telemetry is not None:
                # A stopped run still produces an honest run-level summary.
                self.benchmark_telemetry.close(
                    self._sim_time,
                    self._run_outcome or "simulation_stopped",
                    final_progress,
                )
            if self.event_logger is not None:
                self.event_logger.close(
                    self._sim_time, self._run_outcome or "simulation_stopped"
                )

    def _before_runtime_shutdown(self):
        """Capture the terminal environment while PyBullet is still connected."""
        if self.benchmark_telemetry is None or self._run_outcome == "mission_complete":
            return
        self._capture_benchmark_snapshot(
            f"final_{self._run_outcome or 'simulation_stopped'}",
            force_image=True,
        )

    def _setup_agent_selection(self, agent, cell, choice, path_info):
        """Install an allocated task and turn target-guard failures into recovery."""
        try:
            result = self._setup_agent_selection_impl(
                agent, cell, choice, path_info,
            )
        except TargetKeepoutBlocked as exc:
            agent["pending_target_exit_recovery"] = {
                "reason": "allocation_start_inside_target_keepout",
                "signed_clearance_m": exc.signed_clearance,
                "required_clearance_m": exc.required_clearance,
            }
            agent["path_rejection_count"] = int(
                agent.get("path_rejection_count", 0)
            ) + 1
            agent["next_task_allocation_time"] = self._sim_time + max(
                0.5, float(self.parking_retry_interval),
            )
            raise
        except Exception:
            # A malformed or temporarily unsafe path must not be resubmitted on
            # every 50 ms physics tick.  The shared allocator releases its
            # reservations after this exception; this timestamp throttles the
            # next attempt in the scheduled runner.
            agent["path_rejection_count"] = int(
                agent.get("path_rejection_count", 0)
            ) + 1
            agent["next_task_allocation_time"] = self._sim_time + max(
                0.5, float(self.parking_retry_interval),
            )
            raise
        agent["pending_target_exit_recovery"] = None
        agent["path_rejection_count"] = 0
        return result

    def _setup_agent_selection_impl(self, agent, cell, choice, path_info):
        self._leave_parking_for_task(agent)
        traj = path_info["path"]
        print(
            f"Agent {agent['id']} planned {choice} task path: "
            f"{len(traj)} grid waypoints, dist={path_info.get('distance', 0.0):.2f}"
        )

        self.visualizer.set_trajectory_preview(
            cell, choice, path_info,
            agent_id=agent["id"], agent_color=agent["color"],
        )

        profile = agent.get("rover_profile")
        overlay_world_path = path_info.get("world_path") or []
        if overlay_world_path:
            # This is a real profile-specific overlay A* path in world coordinates.
            world_pts = [np.array(point, dtype=float) for point in overlay_world_path]
            print(
                f"[OVERLAY] {agent['id']} shovel={profile.shovel_width:.3f}m "
                f"cell={path_info.get('overlay_cell_size', 0.0):.3f}m "
                f"cells={len(path_info.get('overlay_cells', []))}, "
                f"source_cells={len(path_info.get('source_canonical_cells', []))}, "
                f"quantity={path_info.get('expected_collected', 0.0):.1f}/"
                f"{path_info.get('capacity_quantity', 0.0):.0f} "
                f"{path_info.get('material_value_mode', 'count')}, "
                f"objects={path_info.get('expected_collected_objects', 0.0):.1f}, "
                f"mass={path_info.get('expected_collected_mass', 0.0):.1f}"
            )
        else:
            grid_wps = [(c.x, c.y) for c in traj]
            spline_pts, _, success = _base.smooth_path_with_spline(
                grid_wps,
                SPILLAGE_CONFIG["smoothing_factor"],
                SPILLAGE_CONFIG["num_points"],
            )
            if not success:
                spline_pts = grid_wps
            # Canonical grid indices must use the canonical shared-map converter.
            coord_converter = self.coord_converter
            world_pts = [coord_converter.convert_2d_to_3d(gx, gy) for gx, gy in spline_pts]

        planning_radius = max(
            float(self.env_radius),
            float(self.shared_map.env_radius) if self.shared_map is not None else float(self.env_radius),
        )
        if not world_path_is_sane(world_pts, planning_radius, tolerance=0.25):
            raise RuntimeError(
                f"{agent['id']}: rejected non-finite or out-of-envelope task path."
            )
        world_pts = self._resample(world_pts, SPILLAGE_CONFIG["target_spacing"])
        if len(world_pts) < 2:
            raise RuntimeError(f"{agent['id']}: selected task path is too short.")
        original_world_pts = [np.array(pt, dtype=float) for pt in world_pts]
        world_pts, push_extension_pts = _extend_polyline_end(
            original_world_pts,
            self.push_extra_distance,
        )

        S0 = np.array(original_world_pts[0], dtype=float)
        S1 = np.array(original_world_pts[min(5, len(original_world_pts) - 1)], dtype=float)
        v_path = S1 - S0
        v_norm = float(np.linalg.norm(v_path))
        if v_norm < 1e-9:
            v_path = np.array([1.0, 0.0], dtype=float)
        else:
            v_path = v_path / v_norm
        gate_xy, actual_gate_back, gate_clipped = bounded_approach_gate(
            S0,
            v_path,
            float(GATE_CONFIG["gate_back"]),
            planning_radius + self.navigation_outside_margin,
        )
        G = np.array(gate_xy, dtype=float)
        if gate_clipped:
            print(
                f"[PATH-GUARD] {agent['id']} approach gate shortened from "
                f"{float(GATE_CONFIG['gate_back']):.2f}m to {actual_gate_back:.2f}m "
                f"to avoid an outward/far-away goal."
            )

        state_now = _base.get_state(agent["body_id"])
        yaw = float(state_now[2])
        if self.phase1_tracking_point == "shovel":
            start_xy = (
                float(state_now[0]) + (float(profile.shovel_offset) if profile is not None else sched.SHOVEL_OFFSET) * math.cos(yaw),
                float(state_now[1]) + (float(profile.shovel_offset) if profile is not None else sched.SHOVEL_OFFSET) * math.sin(yaw),
            )
        else:
            start_xy = (float(state_now[0]), float(state_now[1]))

        live_pebbles = list(self._get_live_pebble_centers())
        approach_obstacles = live_pebbles + self._other_rover_obstacle_points(
            agent,
            include_active_paths=self.enable_priority_reservations,
            higher_priority_paths_only=True,
        )
        dynamic_env_r = self.calculate_dynamic_env_radius(live_pebbles)
        required_env_r = _required_env_radius(
            [start_xy, G] + list(world_pts),
            margin=0.0,
        )
        # Keep a common, non-shrinking A* frame for all active rovers. The
        # scheduler compares raw A* cell coordinates between rovers, so letting
        # one rover plan on a smaller dynamic grid can create nonsense conflict
        # cells and boundary-clamped starts after previous pushes.
        common_navigation_radius = max(
            float(self.env_radius),
            float(self.shared_map.env_radius) if self.shared_map is not None else float(self.env_radius),
        ) + self.navigation_outside_margin
        env_r = max(common_navigation_radius, dynamic_env_r, required_env_r)

        t0 = time.perf_counter()
        schedule_agent = _build_hybrid_astar_schedule_agent(
            agent_id=agent["id"],
            priority=float(profile.right_of_way_priority if profile is not None else 0.0) + 1e-3 * float(agent["index"]),
            color=agent["color"],
            start_xy=start_xy,
            gate_xy=_as_xy_tuple(G),
            push_world_pts=world_pts,
            env_radius=env_r,
            target_zone_radius=self.target_zone_radius,
            pebble_centers=approach_obstacles,
            allow_replan=self.scheduler_allow_replans,
            rover_radius=(float(profile.astar_radius) if profile is not None else None),
            target_zone=self.target_zone,
        )
        schedule_agent["timing"]["total_plan"] = time.perf_counter() - t0
        _init_scheduler_runtime(schedule_agent, agent, self._sim_time)

        agent["scheduler_agent"] = schedule_agent
        agent["astar_phase"] = "APPROACH"
        agent["hybrid_env_radius"] = env_r
        agent["push_world_pts"] = [np.array(pt, dtype=float) for pt in world_pts]
        agent["planning_pebbles"] = list(schedule_agent.get("planning_pebbles", approach_obstacles))
        agent["ignored_pebbles"] = list(schedule_agent.get("ignored_pebbles", []))
        agent["phase_transition_started_at"] = None
        agent["last_approach_replan_time"] = self._sim_time
        agent["approach_replan_requested_at"] = None
        agent["approach_replan_blockers"] = []
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        agent["priority_replan_failures"] = 0
        agent.setdefault("congestion_failed_replans_total", 0)
        agent["congestion_task_started_at"] = float(self._sim_time)
        agent["selection"] = {
            "cell": cell,
            "path_type": choice,
            "path_info": path_info,
            "world_pts": world_pts,
            "original_world_pts": original_world_pts,
            "push_extension_pts": push_extension_pts,
            "gate": (float(G[0]), float(G[1])),
            "path_start": (float(S0[0]), float(S0[1])),
            "overlay_cells": list(path_info.get("overlay_cells", [])),
            "overlay_cell_size": path_info.get("overlay_cell_size"),
            "shovel_width": path_info.get("shovel_width", profile.shovel_width if profile is not None else self.shovel_width),
        }

        if self.event_logger is not None:
            approach_path = self._scheduler_path_points(schedule_agent)
            push_metrics = _polyline_metrics(world_pts)
            approach_metrics = _polyline_metrics(approach_path)
            push_lower = push_metrics["length_m"] / max(0.05, float(sched.V_MAX))
            push_upper = push_metrics["length_m"] / max(0.05, 0.50 * float(sched.V_MAX))
            estimated_lower = float(schedule_agent["T_total_lower"]) + push_lower
            estimated_upper = float(schedule_agent["T_total_upper"]) + push_upper
            agent["event_task_id"] = self.event_logger.start_task(
                agent["id"], self._sim_time,
                rover_type=getattr(profile, "name", "legacy"),
                task_type=choice,
                plan_id=int(agent.get("plan_id", 0)),
                map_epoch=int(self.shared_map.epoch) if self.shared_map is not None else None,
                selected_task_tier=path_info.get("selected_task_tier"),
                selected_capacity_tier=path_info.get("selected_capacity_tier"),
                capacity_class=path_info.get("capacity_class"),
                capacity_effective_load_ratio=path_info.get(
                    "capacity_effective_load_ratio"
                ),
                collectable_objects_uncapped=path_info.get(
                    "collectable_objects_uncapped"
                ),
                collectable_material_mass_uncapped=path_info.get(
                    "collectable_material_mass_uncapped"
                ),
                task_policy_mode=path_info.get("policy_task_mode"),
                target_ready_fraction=path_info.get("target_ready_fraction"),
                remaining_outside_pebble_fraction=path_info.get(
                    "remaining_outside_pebble_fraction"
                ),
                endgame_target_only_active=path_info.get(
                    "policy_endgame_target_only_active"
                ),
                policy_fallback_order=path_info.get("policy_fallback_order"),
                expected_objects=path_info.get("expected_collected_objects", path_info.get("expected_collected")),
                expected_material_mass=path_info.get("expected_collected_mass"),
                expected_planning_quantity=path_info.get("expected_collected"),
                predicted_delivered=path_info.get(
                    "expected_delivered", path_info.get("total_objects_target")
                ),
                predicted_spillage=path_info.get("expected_spillage"),
                capacity_objects=path_info.get("capacity_objects"),
                capacity_material_mass=path_info.get("capacity_mass"),
                capacity_planning_quantity=path_info.get("capacity_quantity"),
                material_value_mode=path_info.get("material_value_mode", self.material_value_mode),
                allocation_score=path_info.get("allocation_score"),
                estimated_lower_s=estimated_lower,
                estimated_upper_s=estimated_upper,
                estimated_components={
                    "approach_lower_s": schedule_agent["T_total_lower"],
                    "approach_upper_s": schedule_agent["T_total_upper"],
                    "push_lower_s": push_lower,
                    "push_upper_s": push_upper,
                },
                allocated_waypoints=len(traj),
                approach_waypoints=approach_metrics["waypoints"],
                max_path_radius_m=max(push_metrics["max_radius_m"], approach_metrics["max_radius_m"]),
                allocated_grid_path=traj,
                allocated_world_path=original_world_pts,
                extended_push_path=world_pts,
                push_extension_path=push_extension_pts,
                approach_trajectory=approach_path,
                path_metrics={"push": push_metrics, "approach": approach_metrics},
                gate=G,
                scheduler_goal=schedule_agent.get("goal"),
                scheduler_planning_mode=schedule_agent.get("planning_mode"),
                scheduler_priority=schedule_agent.get("priority"),
                start_pose=state_now,
                source_canonical_cells=path_info.get("source_canonical_cells", []),
                impacted_cells=path_info.get("impacted_cells", {}),
                overlay_cells=path_info.get("overlay_cells", []),
                reserved_path_cells=agent.get("reserved_cells", set()),
                reserved_object_cells=agent.get("reserved_object_cells", set()),
                env_radius=env_r,
            )

        if self.draw_execution_paths:
            self._draw_agent_execution_paths(
                agent,
                schedule_agent,
                original_world_pts,
                push_extension_pts,
            )

        if self.draw_flow_field or self.flow_field_vis_mode == "always":
            try:
                sched.redraw_agent_plan(schedule_agent)
            except Exception:
                pass

        print(
            f"[A*] {agent['id']} scheduled APPROACH phase: "
            f"mode={schedule_agent['planning_mode']}, "
            f"approach={schedule_agent['geom']['total_L']:.2f}m, "
            f"T=[{schedule_agent['T_total_lower']:.2f},"
            f"{schedule_agent['T_total_upper']:.2f}]s, "
            f"env_r={env_r:.2f}m, "
            f"push_extra={self.push_extra_distance:.2f}m, "
            f"gate=({G[0]:.2f},{G[1]:.2f}), "
            f"push_end=({world_pts[-1][0]:.2f},{world_pts[-1][1]:.2f})"
        )

    def _draw_agent_execution_paths(
        self,
        agent,
        schedule_agent,
        original_push_pts,
        push_extension_pts,
    ):
        _remove_debug_items(agent.get("execution_debug_items", []))
        item_ids = []

        approach_pts = schedule_agent.get("debug_approach_points", [])
        connector_pts = schedule_agent.get("debug_connector_points", [])
        executed_pts = schedule_agent.get("trajectory", [])

        item_ids.extend(
            _draw_debug_polyline(
                executed_pts,
                color=[0.45, 0.45, 0.45],
                z=0.070 + 0.010 * int(agent["index"]),
                width=1.0,
            )
        )
        item_ids.extend(
            _draw_debug_polyline(
                approach_pts,
                color=[1.0, 0.85, 0.05],
                z=0.095 + 0.010 * int(agent["index"]),
                width=3.0,
            )
        )
        item_ids.extend(
            _draw_debug_polyline(
                connector_pts,
                color=[1.0, 0.35, 0.05],
                z=0.105 + 0.010 * int(agent["index"]),
                width=2.5,
            )
        )
        item_ids.extend(
            _draw_debug_polyline(
                original_push_pts,
                color=[0.05, 0.55, 1.0],
                z=0.115 + 0.010 * int(agent["index"]),
                width=3.0,
            )
        )
        item_ids.extend(
            _draw_debug_polyline(
                push_extension_pts,
                color=[0.0, 0.95, 0.25],
                z=0.125 + 0.010 * int(agent["index"]),
                width=4.0,
            )
        )

        try:
            if approach_pts:
                a0 = approach_pts[0]
                item_ids.append(
                    p.addUserDebugText(
                        f"{agent['id']} A*",
                        [float(a0[0]), float(a0[1]), 0.18],
                        [1.0, 0.75, 0.0],
                        textSize=0.9,
                        lifeTime=0.0,
                    )
                )
            if original_push_pts:
                p0 = original_push_pts[0]
                item_ids.append(
                    p.addUserDebugText(
                        f"{agent['id']} 2D push",
                        [float(p0[0]), float(p0[1]), 0.20],
                        [0.05, 0.45, 1.0],
                        textSize=0.9,
                        lifeTime=0.0,
                    )
                )
        except Exception:
            pass

        agent["execution_debug_items"] = item_ids

    def _tracking_point_xy(self, state, agent=None):
        yaw = float(state[2])
        profile = agent.get("rover_profile") if agent is not None else None
        if agent is not None and agent.get("astar_phase") == "PARKING":
            return np.array([float(state[0]), float(state[1])], dtype=float)
        if self.phase1_tracking_point == "shovel":
            return np.array(
                [
                    float(state[0]) + (float(profile.shovel_offset) if profile is not None else sched.SHOVEL_OFFSET) * math.cos(yaw),
                    float(state[1]) + (float(profile.shovel_offset) if profile is not None else sched.SHOVEL_OFFSET) * math.sin(yaw),
                ],
                dtype=float,
            )
        return np.array([float(state[0]), float(state[1])], dtype=float)

    def _turn_toward_push_path(self, agent, dt):
        state = _base.get_state(agent["body_id"])
        push_pts = [np.array(pt, dtype=float) for pt in agent.get("push_world_pts", [])]
        if len(push_pts) < 2:
            return True, (0.0, 0.0)

        tracking_xy = self._tracking_point_xy(state, agent)
        target = push_pts[0]
        if float(np.linalg.norm(target - tracking_xy)) < 0.12:
            target = push_pts[1]

        desired = target - tracking_xy
        if float(np.linalg.norm(desired)) < 1e-6:
            desired = push_pts[1] - push_pts[0]
        if float(np.linalg.norm(desired)) < 1e-6:
            return True, (0.0, 0.0)

        theta_des = math.atan2(float(desired[1]), float(desired[0]))
        e_theta = sched.wrap_angle(theta_des - float(state[2]))
        if abs(e_theta) <= math.radians(GATE_TURN_TOL_DEG):
            return True, (0.0, 0.0)

        w_cmd = max(-sched.W_MAX, min(sched.W_MAX, GATE_TURN_KP * e_theta))
        return False, (0.0, w_cmd)

    def _start_push_phase(self, agent):
        state_now = _base.get_state(agent["body_id"])
        tracking_xy = self._tracking_point_xy(state_now, agent)
        push_pts = [np.array(pt, dtype=float) for pt in agent.get("push_world_pts", [])]
        if len(push_pts) < 2:
            print(f"[A*] {agent['id']} has no valid push path after approach; syncing.")
            agent["state"] = "SYNC"
            return

        ext_segment = self._resample(
            [tuple(tracking_xy), tuple(push_pts[0])],
            SPILLAGE_CONFIG["target_spacing"],
        )
        push_trajectory = _append_polyline(ext_segment, push_pts)
        env_r = max(
            float(agent.get("hybrid_env_radius", self.env_radius)),
            _required_env_radius(push_trajectory, ASTAR_APPROACH_CONFIG["env_margin"]),
            self.env_radius,
        )

        profile = agent.get("rover_profile")
        push_priority = self._task_phase_priority(agent)
        push_schedule_agent = _build_schedule_agent_from_points(
            agent["id"],
            push_priority,
            agent["color"],
            push_trajectory,
            env_r,
            agent.get("planning_pebbles", self._get_live_pebble_centers()),
            agent.get("ignored_pebbles", []),
            "push_2d_corridor",
            self.scheduler_allow_replans,
        )
        previous = agent.get("scheduler_agent") or {}
        push_schedule_agent["debug_approach_points"] = previous.get("debug_approach_points", [])
        push_schedule_agent["debug_connector_points"] = [
            np.array(tracking_xy, dtype=float),
            np.array(push_pts[0], dtype=float),
        ]
        push_schedule_agent["debug_push_points"] = list(push_pts)
        _init_scheduler_runtime(push_schedule_agent, agent, self._sim_time)

        agent["scheduler_agent"] = push_schedule_agent
        agent["astar_phase"] = "PUSH"
        agent["phase_transition_started_at"] = None
        agent["approach_replan_requested_at"] = None
        agent["approach_replan_blockers"] = []
        if self.benchmark_telemetry is not None:
            self._capture_benchmark_snapshot(
                f"before_push_{agent['id']}",
                force_image=self.benchmark_telemetry.capture_push_images,
            )
            self.benchmark_telemetry.start_push(
                agent, self._sim_time,
                self._scheduler_path_points(push_schedule_agent),
                self._pebble_physics_states(),
            )
        if self.event_logger is not None:
            self.event_logger.phase(
                agent["id"], "PUSH", self._sim_time,
                start_pose=state_now,
                push_trajectory=self._scheduler_path_points(push_schedule_agent),
                scheduler_goal=push_schedule_agent.get("goal"),
                path_metrics=_polyline_metrics(self._scheduler_path_points(push_schedule_agent)),
            )

        if self.draw_execution_paths:
            selection = agent.get("selection", {})
            self._draw_agent_execution_paths(
                agent,
                push_schedule_agent,
                selection.get("original_world_pts", push_pts),
                selection.get("push_extension_pts", []),
            )

        print(
            f"[A*] {agent['id']} -> PUSH phase "
            f"path={push_schedule_agent['geom']['total_L']:.2f}m "
            f"end=({push_pts[-1][0]:.2f},{push_pts[-1][1]:.2f})"
        )

    def _update_phase_done(self, agent, schedule_agent):
        if schedule_agent.get("done"):
            return

        remaining = sched.remaining_path_length(schedule_agent)
        tracking_xy = self._tracking_point_xy(schedule_agent["state"], agent)
        goal = np.array(schedule_agent["goal"], dtype=float)
        dist_goal = float(np.linalg.norm(tracking_xy - goal))

        strict_done = (
            remaining <= sched.PATH_STOP_S
            and dist_goal <= sched.GOAL_DIST_TOL
        )
        near_path_end_done = (
            remaining <= sched.GOAL_REMAINING_S_TOL
            and dist_goal <= sched.GOAL_DONE_DIST_TOL
        )
        endpoint = sched.endpoint_completion_metrics(schedule_agent, tracking_xy)
        passed_end_done = (
            agent.get("astar_phase") == "PUSH"
            and remaining <= sched.PATH_STOP_S
            and endpoint["along"] >= 0.0
            and endpoint["cross_track"] <= endpoint["cross_track_tol"]
        )
        if strict_done or near_path_end_done or passed_end_done:
            schedule_agent["done"] = True
            schedule_agent["done_reason"] = (
                "endpoint_plane_passed" if passed_end_done else "goal_tolerance"
            )
            schedule_agent["control"] = (0.0, 0.0)
            schedule_agent["emergency_stop"] = False
            schedule_agent["emergency_blocker"] = None
            schedule_agent["done_remaining_s"] = remaining
            schedule_agent["done_dist_goal"] = dist_goal

    def _active_scheduler_agents(self):
        active = []
        for agent in self.agents:
            if agent.get("state") not in ("NAVIGATING", "PARKING"):
                continue
            if agent.get("astar_phase") not in ("APPROACH", "PUSH", "PARKING"):
                continue
            schedule_agent = agent.get("scheduler_agent")
            if schedule_agent is not None:
                # Keep the path scheduler and central collision policy on one priority.
                schedule_agent["priority"] = self._task_phase_priority(agent)
                active.append(schedule_agent)
        return active

    def _apply_agent_control(self, agent, v_cmd: float, w_cmd: float) -> None:
        _base.set_wheel_velocities(
            agent["body_id"],
            agent["left_joint"],
            agent["right_joint"],
            float(v_cmd),
            float(w_cmd),
            max_wheel_speed=sched.configured_max_wheel_speed(),
            max_torque=sched.configured_max_torque(),
        )

    def _stop_agent(self, agent):
        self._apply_agent_control(agent, 0.0, 0.0)

    def _scheduler_path_points(self, schedule_agent):
        if schedule_agent is None:
            return ()
        pts = schedule_agent.get("trajectory")
        if pts is None:
            geom = schedule_agent.get("geom", {})
            pts = geom.get("pts", [])
        return tuple((float(pt[0]), float(pt[1])) for pt in pts)

    def _target_exit_required_clearance(self, agent) -> float:
        profile = agent.get("rover_profile")
        rover_extent = (
            float(profile.astar_radius)
            if profile is not None else float(ASTAR_APPROACH_CONFIG["rover_radius"])
        )
        return rover_extent + TARGET_EXIT_CLEARANCE_EXTRA

    def _target_exit_footprint_clearances(self, agent, state=None):
        """Return target clearances for the base and forward shovel footprint.

        Approach A* follows the shovel point when configured to do so.  Target
        exit therefore cannot be declared complete from the base position
        alone: the exact failure seen in the log had a clear base and a shovel
        still inside the polygon keepout.
        """
        if state is None:
            state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        x, y, yaw = float(state[0]), float(state[1]), float(state[2])
        profile = agent.get("rover_profile")
        offset = float(
            profile.shovel_offset if profile is not None else sched.SHOVEL_OFFSET
        )
        width = float(
            profile.shovel_width if profile is not None else self.shovel_width
        )
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        lateral = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)
        base = np.array([x, y], dtype=float)
        shovel_center = base + offset * forward
        half_width = 0.5 * max(0.0, width)
        shovel_points = (
            shovel_center,
            shovel_center + half_width * lateral,
            shovel_center - half_width * lateral,
        )
        base_clearance = float(self.target_zone.signed_distance_world(x, y))
        shovel_clearances = tuple(
            float(self.target_zone.signed_distance_world(float(pt[0]), float(pt[1])))
            for pt in shovel_points
        )
        return base_clearance, shovel_clearances

    def _target_exit_clearance(self, agent, state=None) -> float:
        base_clearance, shovel_clearances = (
            self._target_exit_footprint_clearances(agent, state)
        )
        required = self._target_exit_required_clearance(agent)
        guard = float(ASTAR_APPROACH_CONFIG["target_zone_guard_margin"])
        # Preserve the historical base-clearance scale so callers can keep
        # comparing this value with _target_exit_required_clearance().  A
        # shovel point exactly at the A* guard maps to exactly `required`.
        adjusted_shovel = tuple(
            value + required - guard for value in shovel_clearances
        )
        return min((base_clearance,) + adjusted_shovel)

    def _plan_target_exit(self, agent, state=None):
        if state is None:
            state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        x, y = float(state[0]), float(state[1])
        base_clearance, _ = self._target_exit_footprint_clearances(agent, state)
        boundary = self.target_zone.closest_boundary_point_world(x, y)
        if base_clearance < 0.0:
            travel = np.array(
                [float(boundary[0]) - x, float(boundary[1]) - y],
                dtype=float,
            )
        else:
            travel = np.array(
                [x - float(boundary[0]), y - float(boundary[1])],
                dtype=float,
            )
        norm = float(np.linalg.norm(travel))
        if norm <= 1e-6:
            travel = np.array(
                self.target_zone.outward_direction_world(x, y),
                dtype=float,
            )
            norm = float(np.linalg.norm(travel))
        if norm <= 1e-6:
            yaw = float(state[2])
            travel = np.array([-math.cos(yaw), -math.sin(yaw)], dtype=float)
            norm = 1.0
        direction = travel / norm
        required = self._target_exit_required_clearance(agent)
        profile = agent.get("rover_profile")
        offset = float(
            profile.shovel_offset if profile is not None else sched.SHOVEL_OFFSET
        )
        width = float(
            profile.shovel_width if profile is not None else self.shovel_width
        )
        footprint_reach = max(
            required,
            math.hypot(offset, 0.5 * max(0.0, width))
            + float(ASTAR_APPROACH_CONFIG["target_zone_guard_margin"]),
        )
        goal = np.array(boundary, dtype=float) + (
            footprint_reach + TARGET_EXIT_GOAL_EXTRA
        ) * direction
        agent["target_exit_boundary"] = [
            float(boundary[0]), float(boundary[1]),
        ]
        agent["target_exit_direction"] = [
            float(direction[0]), float(direction[1]),
        ]
        agent["target_exit_goal"] = [float(goal[0]), float(goal[1])]
        agent["target_exit_path"] = [
            (x, y),
            (float(goal[0]), float(goal[1])),
        ]
        return boundary, direction, goal

    def _target_exit_control(self, agent):
        state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        yaw = float(state[2])
        direction = agent.get("target_exit_direction")
        if direction is None or len(direction) < 2:
            _, direction, _ = self._plan_target_exit(agent, state)
        else:
            direction = np.array(direction[:2], dtype=float)
            norm = float(np.linalg.norm(direction))
            if not math.isfinite(norm) or norm <= 1e-6:
                _, direction, _ = self._plan_target_exit(agent, state)
            else:
                direction = direction / norm
        travel_heading = math.atan2(float(direction[1]), float(direction[0]))
        desired_body_heading = math.atan2(
            math.sin(travel_heading + math.pi),
            math.cos(travel_heading + math.pi),
        )
        error = math.atan2(
            math.sin(desired_body_heading - yaw),
            math.cos(desired_body_heading - yaw),
        )
        turn = max(
            -TARGET_EXIT_TURN_LIMIT,
            min(TARGET_EXIT_TURN_LIMIT, TARGET_EXIT_TURN_GAIN * error),
        )
        if abs(error) > math.radians(55.0):
            return 0.0, turn
        return (
            TARGET_EXIT_REVERSE_SPEED * max(0.2, math.cos(error)),
            turn,
        )

    def _begin_target_exit(self, agent, minimum_reverse_time=0.0, reason="post_push"):
        agent["state"] = "ROLLBACK"
        agent["rollback_timer"] = max(0.0, float(minimum_reverse_time))
        agent["target_exit_extended_logged"] = False
        agent["pending_target_exit_recovery"] = None
        clearance = self._target_exit_clearance(agent)
        required = self._target_exit_required_clearance(agent)
        base_clearance, shovel_clearances = (
            self._target_exit_footprint_clearances(agent)
        )
        boundary, direction, goal = self._plan_target_exit(agent)
        released_episode = self.safety.release_winner_for_target_exit(
            int(agent["index"]), self._sim_time,
        )
        if released_episode is not None:
            self._log_event(
                "SAFETY_EPISODE_RELEASED_FOR_TARGET_EXIT",
                agent,
                previous_episode=released_episode,
            )
        self._log_event(
            "TARGET_EXIT_STARTED",
            agent,
            reason=reason,
            signed_clearance_m=clearance,
            required_clearance_m=required,
            base_clearance_m=base_clearance,
            shovel_clearances_m=shovel_clearances,
            boundary=[float(boundary[0]), float(boundary[1])],
            direction=[float(direction[0]), float(direction[1])],
            goal=[float(goal[0]), float(goal[1])],
            released_safety_episode=released_episode,
        )
        if reason in ("push_complete", "post_push") and self.benchmark_telemetry is not None:
            self.benchmark_telemetry.end_push(
                str(agent["id"]), self._sim_time, self._pebble_physics_states(),
            )
            if self.benchmark_telemetry.capture_push_images:
                self._capture_benchmark_snapshot(
                    f"after_push_immediate_{agent['id']}", force_image=True,
                )
        return clearance, required

    def _task_phase_priority(self, agent) -> float:
        phase = agent.get("astar_phase")
        state_name = agent.get("state")
        if state_name == "ROLLBACK":
            key = (
                "TARGET_EXIT"
                if self._target_exit_clearance(agent)
                < self._target_exit_required_clearance(agent)
                else "ROLLBACK"
            )
        else:
            key = phase or state_name
        base_priority = PROTECTED_PHASE_PRIORITIES.get(key, 60.0)
        profile = agent.get("rover_profile")
        raw_rover_priority = float(profile.right_of_way_priority) if profile is not None else 0.0
        rover_priority = max(-4.0, min(4.0, raw_rover_priority))
        return float(base_priority) + rover_priority + 1e-3 * float(agent.get("index", 0))

    def _release_safety_episode_for_conflict_owner(self, pair, reason):
        episode = getattr(self.safety, "episode", None)
        if episode is None or not getattr(episode, "active", False):
            return False
        winner_idx = getattr(episode, "winner_idx", None)
        yielder_indices = set(getattr(episode, "yielder_indices", set()))
        pair_set = {int(value) for value in pair}
        episode_indices = set(yielder_indices)
        if winner_idx is not None:
            episode_indices.add(int(winner_idx))
        if not pair_set.issubset(episode_indices):
            return False

        previous = {
            "winner_idx": winner_idx,
            "yielder_indices": sorted(int(value) for value in yielder_indices),
            "phase": getattr(getattr(episode, "phase", None), "value", str(getattr(episode, "phase", None))),
        }
        self.safety.episode = type(episode)()
        self.safety.rearm_until = self._sim_time + float(self.safety.config.rearm_delay)
        if hasattr(self.safety, "_active_deadlock"):
            self.safety._active_deadlock = False
        self._log_event(
            "SAFETY_EPISODE_RELEASED_FOR_CONFLICT_COORDINATOR",
            pair=sorted(pair_set),
            reason=reason,
            previous_episode=previous,
        )
        return True

    def _synchronize_conflict_ownership(self):
        recovery = getattr(self.cooperative_recovery, "episode", None)
        if recovery is None:
            return
        self._release_safety_episode_for_conflict_owner(
            (int(recovery.winner_idx), int(recovery.yielder_idx)),
            "cooperative_recovery_owns_pair",
        )

    def _apply_cooperative_recovery(self, contexts, controls):
        if (
            self.enable_fleet_deadlock_recovery
            and getattr(self.fleet_recovery, "episode", None) is not None
        ):
            # The fleet coordinator owns every pair in its connected component.
            return dict(controls)
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        snapshots = []
        for ctx in contexts:
            agent = agent_by_idx.get(int(ctx.idx))
            if agent is None:
                continue
            profile = agent.get("rover_profile")
            phase = "ROLLBACK" if agent.get("state") == "ROLLBACK" else str(agent.get("astar_phase") or agent.get("state"))
            snapshots.append(
                RecoveryAgent(
                    idx=int(ctx.idx),
                    agent_id=str(ctx.agent_id),
                    state=ctx.state,
                    active=bool(ctx.active),
                    phase=phase,
                    priority=float(ctx.priority),
                    collision_radius=float(ctx.collision_radius),
                    static_clearance=(float(profile.astar_radius) + 0.02 if profile is not None else 0.27),
                    path_points=tuple((float(p[0]), float(p[1])) for p in ctx.path_points),
                )
            )
        live_pebbles = self._get_live_pebble_centers()
        result = self.cooperative_recovery.update(
            snapshots,
            controls,
            pebbles=tuple((float(p[0]), float(p[1])) for p in live_pebbles),
            env_radius=max(float(self.env_radius), float(self.shared_map.env_radius) if self.shared_map is not None else float(self.env_radius)),
            target_radius=float(self.target_zone_radius),
            sim_time=float(self._sim_time),
            target_zone=self.target_zone,
        )
        if result.active_pair is not None:
            self._release_safety_episode_for_conflict_owner(
                result.active_pair,
                "cooperative_recovery_started",
            )
        for idx in result.replan_indices:
            agent = agent_by_idx.get(int(idx))
            if agent is None:
                continue
            if agent.get("state") == "PARKING":
                self._release_parking_slot(agent)
                _remove_debug_items(agent.get("parking_debug_items", []))
                agent["parking_debug_items"] = []
                agent["scheduler_agent"] = None
                agent["astar_phase"] = None
                agent["state"] = "IDLE"
                agent["no_task_since"] = self._sim_time - self.parking_no_task_grace
                agent["next_task_allocation_time"] = self._sim_time
                self._log_event(
                    "PARKING_REPLAN_AFTER_RECOVERY", agent,
                    recovery_pair=result.active_pair,
                )
                continue
            if (
                agent.get("state") == "NAVIGATING"
                and agent.get("astar_phase") == "APPROACH"
            ):
                agent["approach_replan_requested_at"] = self._sim_time
                pair = result.active_pair or ()
                agent["approach_replan_blockers"] = [value for value in pair if value != idx]
        for message in result.messages:
            print(message)
        recovery_episode = getattr(self.cooperative_recovery, "episode", None)
        recovery_diagnostics = {}
        if recovery_episode is not None:
            recovery_diagnostics = {
                "clear_attempt": int(recovery_episode.clear_attempt),
                "escalation_level": int(recovery_episode.escalation_level),
                "blocked_safe_stop": recovery_episode.phase == "BLOCKED_SAFE_STOP",
                "escape_targets": {
                    str(idx): [float(target[0]), float(target[1])]
                    for idx, target in recovery_episode.clear_targets.items()
                },
                "candidate_rejections": {
                    str(idx): dict(counts)
                    for idx, counts in recovery_episode.clear_rejections.items()
                },
            }
        signature = (tuple(result.active_pair) if result.active_pair else None, result.phase)
        if signature != self._last_recovery_signature:
            previous = self._last_recovery_signature
            if signature[0] is None:
                self._log_event(
                    "COLLISION_RECOVERY_ENDED",
                    previous_pair=previous[0] if previous else None,
                    previous_phase=previous[1] if previous else None,
                    messages=result.messages,
                )
            else:
                winner_idx, yielder_idx = signature[0]
                winner = agent_by_idx.get(int(winner_idx))
                yielder = agent_by_idx.get(int(yielder_idx))
                winner_state = winner.get("state_val") if winner is not None else None
                yielder_state = yielder.get("state_val") if yielder is not None else None
                separation = None
                if winner_state is not None and yielder_state is not None:
                    separation = math.hypot(
                        float(winner_state[0]) - float(yielder_state[0]),
                        float(winner_state[1]) - float(yielder_state[1]),
                    )
                self._log_event(
                    "COLLISION_RECOVERY_CHANGED",
                    pair=signature[0], phase=signature[1],
                    winner_id=winner.get("id") if winner is not None else None,
                    yielder_id=yielder.get("id") if yielder is not None else None,
                    winner_task_phase=(winner.get("astar_phase") or winner.get("state")) if winner is not None else None,
                    yielder_task_phase=(yielder.get("astar_phase") or yielder.get("state")) if yielder is not None else None,
                    winner_priority=self._task_phase_priority(winner) if winner is not None else None,
                    yielder_priority=self._task_phase_priority(yielder) if yielder is not None else None,
                    separation_m=separation,
                    controls={idx: result.controls.get(idx) for idx in signature[0]},
                    messages=result.messages,
                    recovery_diagnostics=recovery_diagnostics,
                )
            self._last_recovery_signature = signature
        elif result.messages:
            self._log_event(
                "COLLISION_RECOVERY_MESSAGE",
                pair=signature[0], phase=signature[1], messages=result.messages,
                recovery_diagnostics=recovery_diagnostics,
            )
        self._last_recovery_result = result
        return result.controls

    def _fleet_recovery_snapshots(self, contexts):
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        snapshots = []
        for ctx in contexts:
            agent = agent_by_idx.get(int(ctx.idx))
            if agent is None:
                continue
            state_name = str(agent.get("state") or "IDLE")
            if state_name == "ROLLBACK":
                phase = "TARGET_EXIT"
            elif state_name == "GROUP_RECOVERY":
                phase = "GROUP_ESCAPE"
            else:
                phase = str(agent.get("astar_phase") or state_name)
            schedule_agent = agent.get("scheduler_agent") or {}
            remaining = None
            if schedule_agent:
                try:
                    remaining = float(sched.remaining_path_length(schedule_agent))
                except Exception:
                    remaining = None
            selection = agent.get("selection") or {}
            path_info = selection.get("path_info") or {}
            expected_quantity = path_info.get(
                "expected_collected",
                path_info.get("expected_collected_mass", 0.0),
            )
            snapshots.append(
                FleetRecoveryAgent(
                    idx=int(ctx.idx),
                    agent_id=str(ctx.agent_id),
                    state=ctx.state,
                    # Reservation conflicts can freeze an APPROACH against an
                    # IDLE/PLANNING/PARKED rover before the local collision
                    # layer considers that rover active.  Promote both members
                    # into the slower fleet watchdog so sustained zero-motion
                    # holds are evacuated by the existing group recovery.
                    active=bool(
                        ctx.active
                        or int(ctx.idx)
                        in getattr(
                            self, "_priority_reservation_conflict_indices", set()
                        )
                    ),
                    phase=phase,
                    priority=float(ctx.priority),
                    collision_radius=float(ctx.collision_radius),
                    remaining_path_m=remaining,
                    expected_quantity=float(expected_quantity or 0.0),
                    path_progress_m=(
                        float(schedule_agent.get("s"))
                        if schedule_agent.get("s") is not None else None
                    ),
                )
            )
        return snapshots

    def _fleet_escape_segment_is_safe(self, mover, component, start, goal):
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
        shared_radius = (
            float(self.shared_map.env_radius)
            if self.shared_map is not None else float(self.env_radius)
        )
        navigation_limit = max(float(self.env_radius), shared_radius) + self.navigation_outside_margin
        if math.hypot(*goal) > navigation_limit - 0.03:
            return False

        zone = self.target_zone
        start_zone_clearance = zone.signed_distance_world(*start)
        profile = next(
            (
                agent.get("rover_profile") for agent in self.agents
                if int(agent["index"]) == int(mover.idx)
            ),
            None,
        )
        pebble_clearance = (
            float(profile.astar_radius) if profile is not None else 0.25
        ) + float(ASTAR_APPROACH_CONFIG["pebble_radius"]) + 0.01
        pebbles = tuple(
            (float(point[0]), float(point[1]))
            for point in self._get_live_pebble_centers()
        )
        pebble_start_distances = tuple(
            math.hypot(start[0] - px, start[1] - py) for px, py in pebbles
        )
        others = [agent for agent in component if int(agent.idx) != int(mover.idx)]
        rover_start_distances = {
            int(other.idx): math.hypot(
                start[0] - other.position[0], start[1] - other.position[1],
            )
            for other in others
        }

        for step in range(1, 25):
            ratio = float(step) / 24.0
            sample = (
                start[0] + ratio * (goal[0] - start[0]),
                start[1] + ratio * (goal[1] - start[1]),
            )
            if math.hypot(*sample) > navigation_limit - 0.02:
                return False
            sample_zone_clearance = zone.signed_distance_world(*sample)
            if start_zone_clearance >= 0.0:
                if sample_zone_clearance < 0.0:
                    return False
            elif sample_zone_clearance < start_zone_clearance - 0.02:
                # A rover already overlapping the target may escape, but must
                # never move deeper through delivered material.
                return False

            for (px, py), start_distance in zip(pebbles, pebble_start_distances):
                sample_distance = math.hypot(sample[0] - px, sample[1] - py)
                if start_distance >= pebble_clearance:
                    if sample_distance < pebble_clearance:
                        return False
                elif sample_distance < start_distance - 0.02:
                    return False

            for other in others:
                start_distance = rover_start_distances[int(other.idx)]
                sample_distance = math.hypot(
                    sample[0] - other.position[0], sample[1] - other.position[1],
                )
                if sample_distance < start_distance - 0.025:
                    return False

        if others:
            closest_start = min(rover_start_distances.values())
            closest_goal = min(
                math.hypot(goal[0] - other.position[0], goal[1] - other.position[1])
                for other in others
            )
            if closest_goal < closest_start + 0.22:
                return False
        return True

    def _plan_fleet_escape_route(self, mover, component, retain_push, attempt):
        start = mover.position
        others = [agent for agent in component if int(agent.idx) != int(mover.idx)]
        if not others:
            return None
        center = (
            sum(agent.position[0] for agent in others) / float(len(others)),
            sum(agent.position[1] for agent in others) / float(len(others)),
        )
        away = (start[0] - center[0], start[1] - center[1])
        norm = math.hypot(*away)
        if norm <= 1e-8:
            away_angle = float(mover.state[2])
        else:
            away_angle = math.atan2(away[1], away[0])

        distance = self.fleet_deadlock_escape_distance + 0.20 * min(3, int(attempt))
        if retain_push:
            # Travel approximately backwards so the shovel stays pointed at the
            # material and the selected push task can be resumed afterwards.
            base_angle = float(mover.state[2]) + math.pi
            offsets = (0, 15, -15, 30, -30)
        else:
            base_angle = away_angle
            offsets = (0, 25, -25, 50, -50, 75, -75, 105, -105, 140, -140, 180)

        candidates = []
        for offset in offsets:
            angle = base_angle + math.radians(float(offset))
            goal = (
                start[0] + distance * math.cos(angle),
                start[1] + distance * math.sin(angle),
            )
            if not self._fleet_escape_segment_is_safe(mover, component, start, goal):
                continue
            min_separation = min(
                math.hypot(goal[0] - other.position[0], goal[1] - other.position[1])
                for other in others
            )
            if retain_push:
                reverse = True
                heading_cost = abs(float(offset)) / 180.0
                mode = "push_task_preserving_rollback"
            else:
                forward_error = abs(_wrap_angle(angle - float(mover.state[2])))
                reverse_error = abs(_wrap_angle(angle + math.pi - float(mover.state[2])))
                reverse = reverse_error + math.radians(8.0) < forward_error
                heading_cost = min(forward_error, reverse_error) / math.pi
                mode = "group_escape_reverse" if reverse else "group_escape_forward"
            score = min_separation - 0.18 * heading_cost
            candidates.append((score, goal, reverse, mode))
        if not candidates:
            return None
        _, goal, reverse, mode = max(candidates, key=lambda item: item[0])
        return EscapeRoute(
            points=(start, (float(goal[0]), float(goal[1]))),
            reverse=bool(reverse),
            mode=str(mode),
        )

    def _abort_task_for_target_exit_recovery(self, agent, exc, reason):
        """Release an unexecuted approach and immediately clear the target."""
        original_phase = str(agent.get("astar_phase") or agent.get("state") or "IDLE")
        reserved_path = set(agent.get("reserved_cells", set()))
        reserved_objects = set(agent.get("reserved_object_cells", set()))
        with self._reservation_lock:
            self.global_reserved_cells.difference_update(reserved_path)
            self.reserved_object_cells.difference_update(reserved_objects)

        if self.event_logger is not None and agent.get("event_task_id") is not None:
            self.event_logger.finish_task(
                agent["id"], self._sim_time, "target_keepout_recovery",
                final_pose=agent.get("state_val"),
                consumed_object_cells=0,
                material_progress=self._get_material_progress(),
            )
            agent["event_task_id"] = None
        _remove_debug_items(agent.get("execution_debug_items", []))
        agent["execution_debug_items"] = []
        agent["scheduler_agent"] = None
        agent["selection"] = None
        agent["reserved_cells"] = set()
        agent["reserved_object_cells"] = set()
        agent["push_world_pts"] = []
        agent["planning_pebbles"] = []
        agent["ignored_pebbles"] = []
        agent["astar_phase"] = None
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        agent["priority_replan_failures"] = 0
        self.priority_reservations.clear_agent(int(agent["index"]))
        self._priority_reservation_holds.discard(int(agent["index"]))
        self._log_event(
            "TARGET_KEEP_OUT_ACTIVE_TASK_ABORTED",
            agent,
            original_phase=original_phase,
            trigger_reason=str(reason),
            signed_clearance_m=exc.signed_clearance,
            required_clearance_m=exc.required_clearance,
            released_path_cells=len(reserved_path),
            released_object_cells=len(reserved_objects),
        )
        self._begin_target_exit(
            agent,
            minimum_reverse_time=0.0,
            reason="approach_replan_inside_target_keepout",
        )

    def _abort_task_for_group_recovery(self, agent):
        if agent.get("group_recovery_task_aborted", False):
            return
        original_phase = str(agent.get("astar_phase") or agent.get("state") or "IDLE")
        pending_allocation = agent.get("future")
        if pending_allocation is not None and not pending_allocation.done():
            pending_allocation.cancel()
        agent["future"] = None
        # Invalidate a worker result that may already be completing while this
        # rover is handed to the group escape controller.
        agent["plan_id"] = int(agent.get("plan_id", 0)) + 1
        reserved_path = set(agent.get("reserved_cells", set()))
        reserved_objects = set(agent.get("reserved_object_cells", set()))
        with self._reservation_lock:
            self.global_reserved_cells.difference_update(reserved_path)
            self.reserved_object_cells.difference_update(reserved_objects)

        if self.event_logger is not None and agent.get("event_task_id") is not None:
            self.event_logger.finish_task(
                agent["id"], self._sim_time, "group_deadlock_reallocated",
                final_pose=agent.get("state_val"),
                consumed_object_cells=0,
                material_progress=self._get_material_progress(),
            )
            agent["event_task_id"] = None
        if agent.get("state") == "PARKING":
            self._release_parking_slot(agent)
        _remove_debug_items(agent.get("parking_debug_items", []))
        _remove_debug_items(agent.get("execution_debug_items", []))
        agent["parking_debug_items"] = []
        agent["execution_debug_items"] = []
        agent["scheduler_agent"] = None
        agent["selection"] = None
        agent["reserved_cells"] = set()
        agent["reserved_object_cells"] = set()
        agent["push_world_pts"] = []
        agent["planning_pebbles"] = []
        agent["ignored_pebbles"] = []
        agent["state"] = "GROUP_RECOVERY"
        agent["astar_phase"] = "GROUP_ESCAPE"
        agent["group_recovery_task_aborted"] = True
        agent["group_recovery_original_phase"] = original_phase
        agent["group_recovery_map_refresh"] = original_phase == "PUSH"
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        self.priority_reservations.clear_agent(int(agent["index"]))
        self._priority_reservation_holds.discard(int(agent["index"]))
        self._log_event(
            "GROUP_MEMBER_TASK_ABORTED", agent,
            original_phase=original_phase,
            released_path_cells=len(reserved_path),
            released_object_cells=len(reserved_objects),
            consumed_object_cells=0,
        )

    def _resume_after_group_recovery(self, agent):
        idx = int(agent["index"])
        aborted = bool(agent.get("group_recovery_task_aborted", False))
        original_phase = agent.get("group_recovery_original_phase")
        if aborted or agent.get("state") == "GROUP_RECOVERY":
            agent["state"] = "IDLE"
            agent["astar_phase"] = None
            agent["scheduler_agent"] = None
            agent["selection"] = None
            agent["no_task_since"] = None
            agent["next_task_allocation_time"] = self._sim_time
        else:
            schedule_agent = agent.get("scheduler_agent")
            if schedule_agent is not None:
                tracking_xy = self._tracking_point_xy(agent["state_val"], agent)
                sched.reproject_agent_progress(schedule_agent, tracking_xy)
            if agent.get("astar_phase") == "APPROACH":
                agent["approach_replan_requested_at"] = self._sim_time
        if agent.get("group_recovery_map_refresh", False):
            self._request_execution_map_refresh(agent["id"])
        agent["group_escape_goal"] = None
        agent["group_escape_route"] = []
        agent["group_recovery_task_aborted"] = False
        agent["group_recovery_original_phase"] = None
        agent["group_recovery_map_refresh"] = False
        self.priority_reservations.clear_agent(idx)

    def _release_lower_level_recovery_for_group(self, component):
        component = set(int(idx) for idx in component)
        recovery = getattr(self.cooperative_recovery, "episode", None)
        if recovery is not None and component.intersection({
            int(recovery.winner_idx), int(recovery.yielder_idx),
        }):
            self.cooperative_recovery.episode = None
        safety_episode = getattr(self.safety, "episode", None)
        if (
            safety_episode is not None
            and getattr(safety_episode, "active", False)
            and component.intersection(
                {int(safety_episode.winner_idx)}.union(
                    int(idx) for idx in safety_episode.yielder_indices
                )
            )
        ):
            self.safety.episode = type(safety_episode)()
            self.safety.rearm_until = self._sim_time + float(self.safety.config.rearm_delay)
            if hasattr(self.safety, "_active_deadlock"):
                self.safety._active_deadlock = False

    def _apply_fleet_deadlock_recovery(self, contexts, controls):
        if not self.enable_fleet_deadlock_recovery:
            return dict(controls)
        snapshots = self._fleet_recovery_snapshots(contexts)
        result = self.fleet_recovery.update(
            snapshots,
            controls,
            sim_time=float(self._sim_time),
            route_planner=self._plan_fleet_escape_route,
        )
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        if result.active_component:
            self._release_lower_level_recovery_for_group(result.active_component)
            for idx in result.active_component:
                self._priority_reservation_holds.discard(int(idx))
        for idx in result.abort_task_indices:
            agent = agent_by_idx.get(int(idx))
            if agent is not None:
                self._abort_task_for_group_recovery(agent)
        for idx in result.resume_indices:
            agent = agent_by_idx.get(int(idx))
            if agent is not None:
                self._resume_after_group_recovery(agent)

        episode = getattr(self.fleet_recovery, "episode", None)
        if episode is not None and episode.current_mover_idx is not None:
            mover = agent_by_idx.get(int(episode.current_mover_idx))
            route = episode.route
            if mover is not None:
                mover["group_escape_route"] = list(route.points) if route is not None else []
                mover["group_escape_goal"] = route.goal if route is not None else None

        for event in result.events:
            data = dict(event)
            event_type = data.pop("event")
            component = data.get("component")
            if component is not None:
                data["component_ids"] = [
                    agent_by_idx[idx]["id"] for idx in component if idx in agent_by_idx
                ]
            winner_idx = data.get("winner_idx")
            if winner_idx is not None and int(winner_idx) in agent_by_idx:
                data["winner_id"] = agent_by_idx[int(winner_idx)]["id"]
            event_agent = None
            agent_idx = data.get("agent_idx")
            if agent_idx is not None:
                event_agent = agent_by_idx.get(int(agent_idx))
            self._log_event(event_type, event_agent, **data)
            print(f"[GROUP-RECOVERY] {event_type}: {data}")

        signature = (
            tuple(result.active_component), result.winner_idx,
            result.mover_idx, result.phase,
        )
        if signature != self._last_fleet_recovery_signature:
            self._last_fleet_recovery_signature = signature
        self._last_fleet_recovery_result = result
        return result.controls

    def _protected_path_points(self, agent):
        if agent.get("state") == "GROUP_RECOVERY":
            pts = agent.get("group_escape_route", [])
            return tuple((float(pt[0]), float(pt[1])) for pt in pts)
        if agent.get("state") == "ROLLBACK":
            pts = agent.get("target_exit_path", [])
            return tuple((float(pt[0]), float(pt[1])) for pt in pts)
        phase = agent.get("astar_phase")
        if phase == "TURN_TO_PUSH":
            pts = agent.get("push_world_pts", [])
            return tuple((float(pt[0]), float(pt[1])) for pt in pts)
        if phase == "PUSH":
            return self._scheduler_path_points(agent.get("scheduler_agent"))
        return ()

    def _safety_goal_for_agent(self, agent, path_points, path_constrained):
        state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])

        if agent.get("state") == "ROLLBACK":
            goal = agent.get("target_exit_goal")
            if goal is not None and len(goal) >= 2:
                return float(goal[0]), float(goal[1])
            x, y, yaw = float(state[0]), float(state[1]), float(state[2])
            return x - 0.50 * math.cos(yaw), y - 0.50 * math.sin(yaw)

        if agent.get("state") == "GROUP_RECOVERY":
            goal = agent.get("group_escape_goal")
            if goal is not None and len(goal) >= 2:
                return float(goal[0]), float(goal[1])

        schedule_agent = agent.get("scheduler_agent")
        if schedule_agent is not None and schedule_agent.get("goal") is not None:
            goal = schedule_agent["goal"]
            return float(goal[0]), float(goal[1])

        push_pts = agent.get("push_world_pts", [])
        if agent.get("astar_phase") == "TURN_TO_PUSH" and push_pts:
            target_idx = 1 if len(push_pts) > 1 else 0
            target = push_pts[target_idx]
            return float(target[0]), float(target[1])

        if path_points:
            return path_points[-1] if path_constrained else path_points[0]

        selection = agent.get("selection")
        if selection and selection.get("gate") is not None:
            gx, gy = selection["gate"]
            return float(gx), float(gy)
        return None

    def _build_scheduled_safety_contexts(self):
        contexts = []
        for agent in self.agents:
            idx = int(agent["index"])
            state = agent.get("state_val")
            if state is None:
                state = _base.get_state(agent["body_id"])

            state_name = agent.get("state")
            phase = agent.get("astar_phase")
            schedule_agent = agent.get("scheduler_agent")
            active = (
                state_name in ("NAVIGATING", "ROLLBACK", "PARKING", "GROUP_RECOVERY")
                and idx not in self._quarantined_agents
            )
            protected_points = self._protected_path_points(agent)
            path_constrained = bool(
                active
                and (
                    (
                        state_name == "NAVIGATING"
                        and phase in ("PUSH", "TURN_TO_PUSH")
                    )
                    or state_name in ("ROLLBACK", "GROUP_RECOVERY")
                )
                and len(protected_points) >= 2
            )
            path_points = protected_points if path_constrained else ()
            goal = (
                self._safety_goal_for_agent(agent, path_points, path_constrained)
                if active else None
            )
            contexts.append(
                SafetyAgentContext(
                    idx=idx,
                    agent_id=agent["id"],
                    state=np.array(state, dtype=float, copy=True),
                    active=active,
                    path_constrained=path_constrained,
                    path_points=path_points,
                    goal=goal,
                    priority=self._task_phase_priority(agent),
                    collision_radius=(
                        float(agent["rover_profile"].collision_radius)
                        if agent.get("rover_profile") is not None else 0.46
                    ),
                )
            )

        contexts = self.safety.with_priorities(contexts)
        by_idx = {ctx.idx: ctx for ctx in contexts}
        for agent in self.agents:
            agent["priority"] = by_idx[int(agent["index"])].priority
        return contexts

    def _print_scheduled_safety_status(self, contexts, controls):
        episode = self.safety.episode
        if not episode.active or self._sim_time - self._last_safety_status_print < 1.0:
            return

        self._last_safety_status_print = self._sim_time
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        parts = []
        for ctx in contexts:
            agent = agent_by_idx.get(ctx.idx)
            phase = agent.get("astar_phase") if agent is not None else None
            mode_name = phase or (agent.get("state") if agent is not None else "IDLE")
            behavior = self.safety.behavior_for(ctx.idx, contexts)
            behavior_name = getattr(behavior, "value", str(behavior))
            v_cmd, w_cmd = controls.get(ctx.idx, (0.0, 0.0))
            role = "winner" if ctx.idx == episode.winner_idx else (
                "yielder" if ctx.idx in episode.yielder_indices else "free"
            )
            parts.append(
                f"{ctx.agent_id}:{role}/{mode_name}/{behavior_name} "
                f"v=({v_cmd:.2f},{w_cmd:.2f})"
            )
        print(f"[SAFETY] phase={episode.phase.value} | " + " | ".join(parts))

    def _approach_replan_blockers(self, agent):
        blockers = []
        requested = set(agent.get("approach_replan_blockers", []))
        requested_radii = []
        for other in self.agents:
            if other is agent:
                continue
            if (
                other.get("astar_phase") in ("PUSH", "TURN_TO_PUSH")
                or other.get("state") == "PARKED"
                or int(other["index"]) in requested
            ):
                state = other.get("state_val")
                if state is None:
                    state = _base.get_state(other["body_id"])
                profile = other.get("rover_profile")
                radius = (
                    float(profile.collision_radius)
                    if profile is not None else APPROACH_REPLAN_KEEP_OUT_RADIUS
                )
                if int(other["index"]) in requested:
                    requested_radii.append(radius)
                blockers.extend(
                    _rover_keepout_points(
                        (float(state[0]), float(state[1])),
                        radius=max(APPROACH_REPLAN_KEEP_OUT_RADIUS, radius),
                    )
                )
        reserved_radius = max(
            [APPROACH_REPLAN_KEEP_OUT_RADIUS] + requested_radii,
        )
        for point in agent.get("priority_reserved_blocker_points", []):
            if point is None or len(point) < 2:
                continue
            blockers.extend(
                _rover_keepout_points(
                    (float(point[0]), float(point[1])),
                    radius=reserved_radius,
                )
            )
        return blockers

    def _replan_approach_phase(self, agent, reason: str) -> bool:
        if agent.get("state") != "NAVIGATING" or agent.get("astar_phase") != "APPROACH":
            return False

        selection = agent.get("selection") or {}
        gate_xy = selection.get("gate")
        push_world_pts = agent.get("push_world_pts", [])
        if gate_xy is None or len(push_world_pts) < 2:
            return False

        state_now = _base.get_state(agent["body_id"])
        start_xy = tuple(self._tracking_point_xy(state_now, agent))
        live_pebbles = list(self._get_live_pebble_centers())
        blocker_points = self._approach_replan_blockers(agent)
        planning_obstacles = live_pebbles + blocker_points
        env_r = max(
            float(agent.get("hybrid_env_radius", self.env_radius)),
            self.calculate_dynamic_env_radius(live_pebbles),
            _required_env_radius([start_xy, gate_xy] + list(push_world_pts), 0.0),
            self.env_radius,
        )
        profile = agent.get("rover_profile")

        try:
            t0 = time.perf_counter()
            schedule_agent = _build_hybrid_astar_schedule_agent(
                agent_id=agent["id"],
                priority=float(profile.right_of_way_priority if profile is not None else 0.0) + 1e-3 * float(agent["index"]),
                color=agent["color"],
                start_xy=start_xy,
                gate_xy=_as_xy_tuple(gate_xy),
                push_world_pts=push_world_pts,
                env_radius=env_r,
                target_zone_radius=self.target_zone_radius,
                pebble_centers=planning_obstacles,
                allow_replan=self.scheduler_allow_replans,
                rover_radius=(float(profile.astar_radius) if profile is not None else None),
                target_zone=self.target_zone,
            )
        except TargetKeepoutBlocked as exc:
            self._abort_task_for_target_exit_recovery(agent, exc, reason)
            print(
                f"[TARGET-EXIT] {agent['id']} approach replan started inside "
                "the target keepout; task released and protected exit started."
            )
            return False
        except Exception as exc:
            agent["last_approach_replan_time"] = self._sim_time
            agent["priority_replan_failures"] = int(
                agent.get("priority_replan_failures", 0),
            ) + (1 if reason.startswith("priority-") else 0)
            if reason.startswith("priority-"):
                agent["congestion_failed_replans_total"] = int(
                    agent.get("congestion_failed_replans_total", 0)
                ) + 1
            self._log_event(
                "APPROACH_REPLAN_FAILED", agent,
                reason=reason, error=repr(exc), start=start_xy, gate=gate_xy,
                blocker_points=blocker_points, env_radius=env_r,
            )
            print(f"[A*] {agent['id']} approach replan failed ({reason}): {exc}")
            return False

        schedule_agent["timing"]["total_plan"] = time.perf_counter() - t0
        _init_scheduler_runtime(schedule_agent, agent, self._sim_time)
        agent["scheduler_agent"] = schedule_agent
        agent["hybrid_env_radius"] = env_r
        agent["planning_pebbles"] = live_pebbles
        agent["ignored_pebbles"] = []
        agent["last_approach_replan_time"] = self._sim_time
        agent["approach_replan_requested_at"] = None
        agent["approach_replan_blockers"] = []
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        agent["priority_replan_failures"] = 0

        if self.draw_execution_paths:
            self._draw_agent_execution_paths(
                agent,
                schedule_agent,
                selection.get("original_world_pts", push_world_pts),
                selection.get("push_extension_pts", []),
            )

        replanned_path = self._scheduler_path_points(schedule_agent)
        self._log_event(
            "APPROACH_REPLANNED", agent,
            reason=reason, start=start_xy, gate=gate_xy,
            blocker_points=blocker_points, env_radius=env_r,
            scheduler_goal=schedule_agent.get("goal"),
            trajectory=replanned_path,
            path_metrics=_polyline_metrics(replanned_path),
            estimated_lower_s=schedule_agent.get("T_total_lower"),
            estimated_upper_s=schedule_agent.get("T_total_upper"),
        )
        print(
            f"[A*] {agent['id']} replanned APPROACH ({reason}): "
            f"path={schedule_agent['geom']['total_L']:.2f}m, "
            f"blockers={len(blocker_points)}, env_r={env_r:.2f}m"
        )
        return True

    def _active_conflict_pairs(self):
        pairs = set()
        if self.enable_fleet_deadlock_recovery:
            pairs.update(self.fleet_recovery.owned_pairs())
        recovery = getattr(self.cooperative_recovery, "episode", None)
        if recovery is not None:
            pairs.add(tuple(sorted((
                int(recovery.winner_idx), int(recovery.yielder_idx),
            ))))

        safety_episode = getattr(self.safety, "episode", None)
        if safety_episode is not None and getattr(safety_episode, "active", False):
            winner_idx = getattr(safety_episode, "winner_idx", None)
            if winner_idx is not None:
                for yielder_idx in getattr(safety_episode, "yielder_indices", set()):
                    pairs.add(tuple(sorted((int(winner_idx), int(yielder_idx)))))
        return pairs

    def _priority_reservation_path(self, agent):
        if agent.get("state") == "GROUP_RECOVERY":
            points = agent.get("group_escape_route", [])
            if points:
                return tuple((float(point[0]), float(point[1])) for point in points)
        if agent.get("state") == "ROLLBACK":
            points = agent.get("target_exit_path", [])
            if points:
                return tuple((float(point[0]), float(point[1])) for point in points)
        return self._scheduler_path_points(agent.get("scheduler_agent"))

    def _update_prioritized_trajectory_reservations(self):
        self._priority_reservation_holds = set()
        self._priority_reservation_conflict_pairs = set()
        self._priority_reservation_conflict_indices = set()
        if not self.enable_priority_reservations:
            return

        snapshots = []
        for agent in self.agents:
            idx = int(agent["index"])
            if idx in self._quarantined_agents:
                continue
            state = agent.get("state_val")
            if state is None or len(state) < 4:
                continue
            state_name = str(agent.get("state") or "IDLE")
            active = state_name in {
                "NAVIGATING", "ROLLBACK", "PARKING", "PARKED", "PLANNING", "IDLE",
                "GROUP_RECOVERY",
            }
            if not active:
                continue
            profile = agent.get("rover_profile")
            phase = "TARGET_EXIT" if state_name == "ROLLBACK" else str(
                agent.get("astar_phase") or state_name
            )
            snapshots.append(
                ReservationAgent(
                    idx=idx,
                    agent_id=str(agent["id"]),
                    priority=float(self._task_phase_priority(agent)),
                    position=(float(state[0]), float(state[1])),
                    speed=float(state[3]),
                    collision_radius=(
                        float(profile.collision_radius)
                        if profile is not None else 0.46
                    ),
                    phase=phase,
                    active=True,
                    can_replan=(
                        state_name == "NAVIGATING"
                        and agent.get("astar_phase") == "APPROACH"
                    ),
                    path_points=self._priority_reservation_path(agent),
                    replan_failures=int(agent.get("priority_replan_failures", 0)),
                )
            )

        decision = self.priority_reservations.update(
            snapshots,
            self._sim_time,
            owned_pairs=self._active_conflict_pairs(),
        )
        self._priority_reservation_holds = set(decision.hold_indices)
        self._priority_reservation_conflict_pairs = {
            tuple(sorted((int(conflict.winner_idx), int(conflict.yielder_idx))))
            for conflict in decision.conflicts
        }
        self._priority_reservation_conflict_indices = {
            idx
            for pair in self._priority_reservation_conflict_pairs
            for idx in pair
        }
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        for replan_idx, blockers in decision.replan_blockers.items():
            agent = agent_by_idx.get(int(replan_idx))
            if agent is None:
                continue
            if not (
                agent.get("state") == "NAVIGATING"
                and agent.get("astar_phase") == "APPROACH"
            ):
                continue
            agent["approach_replan_requested_at"] = self._sim_time
            existing = set(agent.get("approach_replan_blockers", []))
            existing.update(int(value) for value in blockers)
            agent["approach_replan_blockers"] = sorted(existing)
            agent["priority_reserved_blocker_points"] = [
                (float(point[0]), float(point[1]))
                for point in decision.blocker_points.get(int(replan_idx), ())
            ]
            matching = [
                conflict for conflict in decision.conflicts
                if conflict.replan_idx == int(replan_idx)
            ]
            if matching:
                agent["priority_reservation_reason"] = matching[0].reason

        signature = tuple(sorted(
            (
                conflict.winner_idx,
                conflict.yielder_idx,
                conflict.replan_idx,
                conflict.reason,
            )
            for conflict in decision.conflicts
        ))
        if signature != self._last_priority_reservation_signature:
            if signature:
                self._log_event(
                    "PRIORITY_TRAJECTORY_CONFLICT",
                    conflicts=[{
                        "winner_id": agent_by_idx[conflict.winner_idx]["id"],
                        "yielder_id": agent_by_idx[conflict.yielder_idx]["id"],
                        "replan_id": (
                            agent_by_idx[conflict.replan_idx]["id"]
                            if conflict.replan_idx is not None else None
                        ),
                        "blocker_id": (
                            agent_by_idx[conflict.blocker_idx]["id"]
                            if conflict.blocker_idx is not None else None
                        ),
                        "time_to_conflict_s": conflict.time_to_conflict,
                        "predicted_distance_m": conflict.predicted_distance,
                        "required_distance_m": conflict.required_distance,
                        "yielder_stalled": conflict.yielder_stalled,
                        "reason": conflict.reason,
                    } for conflict in decision.conflicts],
                    hold_agents=[
                        agent_by_idx[idx]["id"]
                        for idx in sorted(decision.hold_indices)
                        if idx in agent_by_idx
                    ],
                )
                print(
                    "[RESERVATION] "
                    + " | ".join(
                        f"{agent_by_idx[conflict.winner_idx]['id']}>"
                        f"{agent_by_idx[conflict.yielder_idx]['id']} "
                        f"ttc={conflict.time_to_conflict:.1f}s "
                        f"action={conflict.reason}"
                        for conflict in decision.conflicts
                    )
                )
            elif self._last_priority_reservation_signature:
                self._log_event("PRIORITY_TRAJECTORY_CONFLICT_CLEARED")
            self._last_priority_reservation_signature = signature

    def _maybe_replan_approach_paths(self):
        owned_pairs = self._active_conflict_pairs()
        recovery_indices = {idx for pair in owned_pairs for idx in pair}
        for agent in self.agents:
            agent_index = int(agent.get("index", -1))
            if agent_index in self._quarantined_agents:
                continue
            if agent_index in recovery_indices:
                if not agent.get("recovery_replan_suppressed", False):
                    agent["recovery_replan_suppressed"] = True
                    self._log_event(
                        "APPROACH_REPLAN_SUPPRESSED",
                        agent,
                        reason=(
                            "fleet_deadlock_coordinator_owns_group"
                            if getattr(self.fleet_recovery, "episode", None) is not None
                            else "conflict_coordinator_owns_pair"
                        ),
                        recovery_pair=tuple(sorted(recovery_indices)),
                    )
                continue
            agent["recovery_replan_suppressed"] = False
            if agent.get("state") != "NAVIGATING" or agent.get("astar_phase") != "APPROACH":
                continue
            requested = agent.get("approach_replan_requested_at") is not None
            last_replan = float(agent.get("last_approach_replan_time", 0.0))
            if requested and self._sim_time - last_replan < APPROACH_REPLAN_MIN_GAP:
                continue
            interval_due = (
                self.approach_replan_interval > 0.0
                and self._sim_time - last_replan >= self.approach_replan_interval
            )
            if not requested and not interval_due:
                continue
            reason = (
                "priority-" + str(agent.get("priority_reservation_reason") or "yield")
                if requested else "periodic"
            )
            self._replan_approach_phase(agent, reason)

    def _request_execution_map_refresh(self, agent_id: str) -> None:
        self._pending_execution_map_refresh = True
        self._pending_execution_map_refresh_agents.add(str(agent_id))
        if self._sim_time - self._last_execution_map_refresh_print >= 1.0:
            self._last_execution_map_refresh_print = self._sim_time
            print(
                f"[MAP] 2D map refresh requested after executed path "
                f"({', '.join(sorted(self._pending_execution_map_refresh_agents))})."
            )

    def _poll_shared_map_result(self):
        fut = self.shared_map_future
        if fut is None or not fut.done():
            return

        self.shared_map_future = None
        try:
            snapshot = fut.result()
        except Exception as exc:
            print(f"[MAP] Shared 2D map build failed: {exc}")
            return

        self.shared_map = snapshot
        self._last_shared_map_built_clock = self._map_clock()
        self.env_2d = snapshot.env_2d
        self.coord_converter = snapshot.coord_converter

        with self._reservation_lock:
            covered_consumed = set(self._consumed_cells_in_submitted_refresh)
            if covered_consumed:
                self.consumed_object_cells.difference_update(covered_consumed)
            else:
                self.consumed_object_cells.clear()
        self._consumed_cells_in_submitted_refresh = set()

        try:
            self.visualizer.update_env(snapshot.env_2d)
            self._needs_redraw = True
        except Exception:
            pass
        self._refresh_planning_envelope_visual(snapshot.env_radius)

        print(
            f"[MAP] Shared 2D map ready "
            f"(epoch={snapshot.epoch}, objects={snapshot.pebbles_count}, "
            f"mass={snapshot.pebbles_material_mass:.0f}, mode={snapshot.material_value_mode})"
        )
        if self.benchmark_telemetry is not None:
            reason = "initial_map" if self._benchmark_last_map_epoch is None else "map_rebuild"
            self._benchmark_last_map_epoch = int(snapshot.epoch)
            self._capture_benchmark_snapshot(
                reason, force_image=(reason == "initial_map")
            )

    def _maybe_start_execution_map_refresh(self) -> None:
        self._poll_shared_map_result()
        if not self._pending_execution_map_refresh:
            return
        if self.map_executor is None:
            return
        if self.shared_map_future is not None and not self.shared_map_future.done():
            return

        now = self._map_clock()
        last_built = self._last_shared_map_built_clock
        wait_remaining = float(self.map_update_interval) - (now - last_built)
        if wait_remaining > 0.0:
            if self._sim_time - self._last_execution_map_refresh_print >= 1.0:
                self._last_execution_map_refresh_print = self._sim_time
                print(
                    "[MAP] Coalescing completed-path refresh "
                    f"({wait_remaining:.1f}s until min interval)."
                )
            return

        agents = ", ".join(sorted(self._pending_execution_map_refresh_agents))
        self._pending_execution_map_refresh = False
        self._pending_execution_map_refresh_agents.clear()
        with self._reservation_lock:
            self._consumed_cells_in_submitted_refresh = set(self.consumed_object_cells)
        self._maybe_start_shared_map_build(force=True)
        if self.shared_map_future is not None:
            print(f"[MAP] Submitted completed-path 2D refresh ({agents}).")
        else:
            self._consumed_cells_in_submitted_refresh = set()
            self._pending_execution_map_refresh = True

    def _other_rover_obstacle_points(
        self,
        agent,
        include_active_paths=False,
        higher_priority_paths_only=False,
    ):
        points = []
        requester_priority = self._task_phase_priority(agent)
        if agent.get("astar_phase") is None and agent.get("state") in (
            "PLANNING", "NAVIGATING",
        ):
            requester_priority = self._task_phase_priority(
                {**agent, "astar_phase": "APPROACH"},
            )
        for other in self.agents:
            if other is agent:
                continue
            state = other.get("state_val")
            if state is None:
                state = _base.get_state(other["body_id"])
            profile = other.get("rover_profile")
            radius = (
                float(profile.collision_radius)
                if profile is not None else APPROACH_REPLAN_KEEP_OUT_RADIUS
            )
            points.extend(
                _rover_keepout_points(
                    (float(state[0]), float(state[1])),
                    radius=max(APPROACH_REPLAN_KEEP_OUT_RADIUS, radius),
                )
            )
            path_has_priority = (
                not higher_priority_paths_only
                or self._task_phase_priority(other) < requester_priority
            )
            if (
                include_active_paths
                and path_has_priority
                and other.get("state") in ("NAVIGATING", "ROLLBACK", "PARKING")
            ):
                path_points = self._scheduler_path_points(other.get("scheduler_agent"))
                if other.get("state") == "ROLLBACK":
                    path_points = tuple(other.get("target_exit_path", ())) or path_points
                for a, b in zip(path_points[:-1], path_points[1:]):
                    ax, ay = float(a[0]), float(a[1])
                    bx, by = float(b[0]), float(b[1])
                    length = math.hypot(bx - ax, by - ay)
                    samples = max(1, int(math.ceil(length / 0.22)))
                    for step in range(samples + 1):
                        t = float(step) / float(samples)
                        points.append((ax + t * (bx - ax), ay + t * (by - ay)))
        return points

    def _parking_slot_candidates(self, agent):
        profile = agent.get("rover_profile")
        rover_radius = float(profile.astar_radius) if profile is not None else 0.25
        base_radius = max(
            float(self.env_radius),
            float(self.shared_map.env_radius) if self.shared_map is not None else float(self.env_radius),
        )
        navigation_limit = base_radius + self.navigation_outside_margin
        ring_radius = min(
            base_radius + self.parking_ring_offset,
            navigation_limit - rover_radius - 0.08,
        )
        if ring_radius <= self.target_zone.bounding_radius + rover_radius + 0.15:
            return []

        state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        current_angle = math.atan2(float(state[1]), float(state[0]))
        candidates = []
        for slot in range(self.parking_slot_count):
            owner = self._parking_slot_owners.get(slot)
            if owner is not None and owner != int(agent["index"]):
                continue
            angle = 2.0 * math.pi * float(slot) / float(self.parking_slot_count)
            goal = (ring_radius * math.cos(angle), ring_radius * math.sin(angle))
            if self.target_zone.signed_distance_world(*goal) < rover_radius + 0.10:
                continue
            angular_cost = abs(math.atan2(math.sin(angle - current_angle), math.cos(angle - current_angle)))
            candidates.append((angular_cost, slot, goal, navigation_limit))
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates

    def _start_parking(self, agent, force=False):
        if not self.enable_idle_parking and not force:
            return False
        state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        start_xy = (float(state[0]), float(state[1]))
        profile = agent.get("rover_profile")
        rover_radius = float(profile.astar_radius) if profile is not None else 0.25
        live_pebbles = list(self._get_live_pebble_centers())
        obstacle_points = live_pebbles + self._other_rover_obstacle_points(
            agent, include_active_paths=True,
        )

        parking_candidates = self._parking_slot_candidates(agent)
        parking_rejections = {
            "goal_blocked": 0,
            "astar_failed": 0,
            "degenerate_path": 0,
            "path_outside_navigation_limit": 0,
        }
        for _, slot, goal, navigation_limit in parking_candidates:
            if any(
                math.hypot(goal[0] - pnt[0], goal[1] - pnt[1])
                < rover_radius + ASTAR_APPROACH_CONFIG["pebble_radius"] + 0.10
                for pnt in obstacle_points
            ):
                parking_rejections["goal_blocked"] += 1
                continue
            env_r = max(
                float(navigation_limit),
                _required_env_radius(
                    [start_xy, goal],
                    0.0,
                ),
            )
            field = _build_approach_field(
                env_r,
                obstacle_points,
                self.target_zone_radius,
                block_target_zone=True,
                rover_radius=rover_radius,
                target_zone=self.target_zone,
            )
            raw_path = AStarGridPlanner(field).plan(start_xy, goal)
            if raw_path is None:
                parking_rejections["astar_failed"] += 1
                continue
            parking_path = _smooth_approach(field, raw_path)
            if parking_path is None or len(parking_path) < 2:
                parking_rejections["degenerate_path"] += 1
                continue
            if not world_path_is_sane(
                parking_path,
                float(navigation_limit),
                tolerance=ASTAR_APPROACH_CONFIG["env_margin"],
            ):
                parking_rejections["path_outside_navigation_limit"] += 1
                continue

            schedule_agent = _build_schedule_agent_from_points(
                agent["id"],
                self._task_phase_priority({**agent, "astar_phase": "PARKING"}),
                agent["color"],
                parking_path,
                env_r,
                obstacle_points,
                [],
                "parking_astar",
                False,
                rover_radius=rover_radius,
            )
            _init_scheduler_runtime(schedule_agent, agent, self._sim_time)
            # Parking is a base-position maneuver, independent of shovel geometry.
            schedule_agent["tracking_offset"] = 0.0
            schedule_agent["endpoint_cross_track_tol"] = max(
                0.08, float(sched.GOAL_DONE_DIST_TOL),
            )
            self._parking_slot_owners[int(slot)] = int(agent["index"])
            agent["parking_slot_index"] = int(slot)
            agent["parking_goal"] = tuple(float(v) for v in goal)
            agent["scheduler_agent"] = schedule_agent
            agent["hybrid_env_radius"] = env_r
            agent["astar_phase"] = "PARKING"
            agent["state"] = "PARKING"
            agent["next_parking_probe_time"] = (
                self._sim_time + self.parking_task_poll_interval
            )
            _remove_debug_items(agent.get("parking_debug_items", []))
            agent["parking_debug_items"] = (
                _draw_debug_polyline(
                    parking_path,
                    sched.path_color_from_rgba(agent["color"]),
                    z=0.11,
                    width=3.0,
                )
                if self.draw_execution_paths else []
            )
            self._log_event(
                "PARKING_STARTED",
                agent,
                slot=int(slot),
                goal=goal,
                route=self._scheduler_path_points(schedule_agent),
                route_length_m=float(schedule_agent["geom"]["total_L"]),
                priority=self._task_phase_priority(agent),
            )
            print(
                f"[PARK] {agent['id']} -> slot {slot} "
                f"goal=({goal[0]:.2f},{goal[1]:.2f}) "
                f"path={schedule_agent['geom']['total_L']:.2f}m"
            )
            return True

        self._log_event(
            "PARKING_DEFERRED",
            agent,
            reason="no_safe_astar_slot",
            start=start_xy,
            candidate_slots=len(parking_candidates),
            obstacle_points=len(obstacle_points),
            rejections=parking_rejections,
        )
        print(
            f"[PARK] {agent['id']} found no safe parking route; will retry. "
            f"rejections={parking_rejections}"
        )
        return False

    def _release_parking_slot(self, agent):
        slot = agent.get("parking_slot_index")
        if slot is not None and self._parking_slot_owners.get(int(slot)) == int(agent["index"]):
            self._parking_slot_owners.pop(int(slot), None)
        agent["parking_slot_index"] = None
        agent["parking_goal"] = None

    def _allocation_excluded_task_zones(self, agent):
        """Return live, rover-specific congestion exclusions for allocation."""
        if not self.congestion_config.enabled:
            return ()
        now = float(self._sim_time)
        self._congestion_zones = [
            zone for zone in self._congestion_zones
            if float(zone["expires_at"]) > now
        ]
        idx = int(agent.get("index", -1))
        return tuple(
            (
                float(zone["center"][0]),
                float(zone["center"][1]),
                float(zone["radius"]),
            )
            for zone in self._congestion_zones
            if int(zone["agent_idx"]) == idx
        )

    def _approach_congestion_observations(self, contexts):
        context_by_idx = {int(ctx.idx): ctx for ctx in contexts}
        observations = []
        fleet_component = set()
        fleet_result = self._last_fleet_recovery_result
        if fleet_result is not None:
            fleet_component = set(int(idx) for idx in fleet_result.active_component)
        recovery_episode = getattr(self.cooperative_recovery, "episode", None)
        recovery_members = set()
        if recovery_episode is not None:
            recovery_members = {
                int(recovery_episode.winner_idx), int(recovery_episode.yielder_idx),
            }
        safety_episode = getattr(self.safety, "episode", None)
        safety_members = set()
        if safety_episode is not None and getattr(safety_episode, "active", False):
            safety_members = {int(safety_episode.winner_idx)}.union(
                int(idx) for idx in safety_episode.yielder_indices
            )

        for agent in self.agents:
            if agent.get("state") != "NAVIGATING" or agent.get("astar_phase") != "APPROACH":
                continue
            idx = int(agent["index"])
            state = agent.get("state_val")
            selection = agent.get("selection") or {}
            gate = selection.get("gate")
            if state is None or gate is None:
                continue
            tracking = self._tracking_point_xy(state, agent)
            gate_distance = math.hypot(
                float(tracking[0]) - float(gate[0]),
                float(tracking[1]) - float(gate[1]),
            )
            blockers = set(int(value) for value in agent.get("approach_replan_blockers", ()))
            for pair in self._priority_reservation_conflict_pairs:
                if idx in pair:
                    blockers.update(int(value) for value in pair if int(value) != idx)
            conflicted = bool(
                idx in self._priority_reservation_holds
                or blockers
                or idx in recovery_members
                or idx in safety_members
                or idx in fleet_component
            )
            path_info = selection.get("path_info") or {}
            observations.append(ApproachObservation(
                idx=idx,
                agent_id=str(agent["id"]),
                sim_time=float(self._sim_time),
                task_id=agent.get("event_task_id"),
                task_started_at=float(agent.get("congestion_task_started_at", self._sim_time)),
                position=(float(state[0]), float(state[1])),
                gate=(float(gate[0]), float(gate[1])),
                gate_distance_m=float(gate_distance),
                conflicted=conflicted,
                blocker_indices=tuple(sorted(blockers.union(
                    (recovery_members | safety_members | fleet_component) - {idx}
                ))),
                failed_replans_total=int(agent.get("congestion_failed_replans_total", 0)),
                expected_load_ratio=float(path_info.get("capacity_effective_load_ratio", 0.0) or 0.0),
                task_type=str(selection.get("path_type") or "unknown"),
                priority=float(self._task_phase_priority(agent)),
            ))
        return observations

    def _abort_congested_approach(self, agent, decision):
        """Cancel one approach, release its claims, and stage it before retry."""
        if agent.get("state") != "NAVIGATING" or agent.get("astar_phase") != "APPROACH":
            return False
        original_task_id = agent.get("event_task_id")
        original_selection = agent.get("selection") or {}
        original_path_info = original_selection.get("path_info") or {}
        reserved_path = set(agent.get("reserved_cells", set()))
        reserved_objects = set(agent.get("reserved_object_cells", set()))
        with self._reservation_lock:
            self.global_reserved_cells.difference_update(reserved_path)
            self.reserved_object_cells.difference_update(reserved_objects)
        if self.event_logger is not None and original_task_id is not None:
            self.event_logger.finish_task(
                agent["id"], self._sim_time, "congestion_reallocated",
                final_pose=agent.get("state_val"),
                consumed_object_cells=0,
                material_progress=self._get_material_progress(),
            )
            agent["event_task_id"] = None
        _remove_debug_items(agent.get("execution_debug_items", []))
        agent["execution_debug_items"] = []
        agent["scheduler_agent"] = None
        agent["selection"] = None
        agent["reserved_cells"] = set()
        agent["reserved_object_cells"] = set()
        agent["push_world_pts"] = []
        agent["planning_pebbles"] = []
        agent["ignored_pebbles"] = []
        agent["state"] = "IDLE"
        agent["astar_phase"] = None
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        agent["approach_replan_blockers"] = []
        agent["approach_replan_requested_at"] = None
        self.priority_reservations.clear_agent(int(agent["index"]))
        self._priority_reservation_holds.discard(int(agent["index"]))

        expires_at = float(self._sim_time) + self.congestion_config.cooldown_s
        zone = {
            "agent_idx": int(agent["index"]),
            "center": tuple(float(value) for value in decision.task_zone),
            "radius": float(self.congestion_config.exclusion_radius_m),
            "expires_at": expires_at,
            "task_id": original_task_id,
        }
        self._congestion_zones.append(zone)
        self._congestion_staging_agents.add(int(agent["index"]))
        self._log_event(
            "CONGESTION_TASK_REALLOCATED", agent,
            original_task_id=original_task_id,
            original_task_type=original_selection.get("path_type"),
            expected_load_ratio=original_path_info.get("capacity_effective_load_ratio"),
            member_indices=list(decision.member_indices),
            blocker_indices=list(decision.blocker_indices),
            dominant_blocker_idx=decision.dominant_blocker_idx,
            gate_progress_m=decision.gate_progress_m,
            conflict_fraction=decision.conflict_fraction,
            failed_replans=decision.failed_replans,
            commitment=list(decision.commitment),
            conflict_zone=list(decision.conflict_zone),
            excluded_task_zone={
                "center": list(decision.task_zone),
                "radius_m": self.congestion_config.exclusion_radius_m,
                "expires_at": expires_at,
            },
            diagnostics=decision.diagnostics,
            released_path_cells=len(reserved_path),
            released_object_cells=len(reserved_objects),
        )
        staged = self._start_parking(agent, force=True)
        if staged:
            self._log_event(
                "CONGESTION_STAGING_STARTED", agent,
                goal=agent.get("parking_goal"),
                excluded_until=expires_at,
            )
        else:
            agent["next_task_allocation_time"] = min(
                expires_at,
                float(self._sim_time) + max(1.0, self.parking_retry_interval),
            )
            self._log_event(
                "CONGESTION_STAGING_DEFERRED", agent,
                excluded_until=expires_at,
            )
        print(
            f"[CONGESTION] {agent['id']} released {original_selection.get('path_type')} "
            f"task after {decision.gate_progress_m:.2f}m gate progress and "
            f"{decision.conflict_fraction:.0%} conflicted samples; staging before reassignment."
        )
        return True

    def _update_congestion_reassignment(self, contexts):
        if not self.congestion_config.enabled:
            return None
        if getattr(self.fleet_recovery, "episode", None) is not None:
            return None
        decision = self.congestion_monitor.update(
            self._approach_congestion_observations(contexts), self._sim_time,
        )
        if decision is None:
            return None
        agent = next(
            (item for item in self.agents if int(item["index"]) == decision.mover_idx),
            None,
        )
        if agent is not None:
            if self._abort_congested_approach(agent, decision):
                self._release_lower_level_recovery_for_group(decision.member_indices)
                return int(agent["index"])
        return None

    def _submit_parking_task_probe(self, agent):
        future = agent.get("parking_probe_future")
        if future is not None and not future.done():
            return
        self._poll_shared_map_result()
        if self.shared_map is None:
            return
        agent["parking_probe_id"] = int(agent.get("parking_probe_id", 0)) + 1
        probe_id = int(agent["parking_probe_id"])
        state = agent.get("state_val")
        if state is None:
            state = _base.get_state(agent["body_id"])
        reserved_path, reserved_objects, consumed_objects = self._reservation_snapshot()
        profile = agent.get("rover_profile")
        request = _shared.PathAllocationRequest(
            agent_id=agent["id"],
            plan_id=probe_id,
            map_epoch=self.shared_map.epoch,
            agent_pose_xy=(float(state[0]), float(state[1])),
            reserved_path_cells=reserved_path,
            reserved_object_cells=reserved_objects,
            consumed_object_cells=consumed_objects,
            rover_type=profile,
            shovel_width=(
                float(profile.shovel_width) if profile is not None else self.shovel_width
            ),
            overlay_cell_size=(
                float(profile.overlay_cell_size) if profile is not None else 0.0
            ),
            reservation_radius=(
                float(profile.reservation_radius)
                if profile is not None else self.shovel_width / 2.0 + 0.03
            ),
            target_root_sources_only=self.target_root_sources_only,
            target_source_mode=self._effective_target_source_mode(profile),
            previous_task_preference=agent.get("task_preference"),
            excluded_task_zones=tuple(self._allocation_excluded_task_zones(agent)),
        )
        if self.synchronous_planning:
            agent["parking_probe_future"] = cf.Future()
            try:
                agent["parking_probe_future"].set_result(
                    _shared.allocate_path_job(self.shared_map, request)
                )
            except Exception as exc:
                agent["parking_probe_future"].set_exception(exc)
        else:
            agent["parking_probe_future"] = self.executor.submit(
                _shared.allocate_path_job,
                self.shared_map,
                request,
            )
        agent["next_parking_probe_time"] = (
            self._sim_time + self.parking_task_poll_interval
        )

    def _poll_parking_task_probes(self):
        for agent in self.agents:
            future = agent.get("parking_probe_future")
            if future is None or not future.done():
                continue
            agent["parking_probe_future"] = None
            try:
                result = future.result()
            except Exception as exc:
                self._log_event(
                    "PARKING_TASK_PROBE_FAILED", agent, error=repr(exc),
                )
                continue
            current_epoch = self.shared_map.epoch if self.shared_map is not None else None
            if (
                int(agent.get("index", -1)) in self._congestion_staging_agents
                and self._allocation_excluded_task_zones(agent)
            ):
                # A congestion mover must first reach a staging slot.  Probing
                # continues in the background, but an immediately available
                # task must not interrupt the evacuation maneuver.
                self._log_event(
                    "CONGESTION_TASK_PROBE_DEFERRED", agent,
                    reason="staging_or_cooldown_active",
                )
                continue
            if result.map_epoch == current_epoch:
                preference = (getattr(result, "diagnostics", {}) or {}).get(
                    "task_preference"
                )
                if preference in ("target", "highway"):
                    agent["task_preference"] = preference
            task_available = bool(
                result.error is None
                and current_epoch is not None
                and result.map_epoch == current_epoch
                and result.best_cell is not None
                and result.best_path_info is not None
            )
            if not task_available:
                continue
            previous_state = agent.get("state")
            self._leave_parking_for_task(agent)
            agent["state"] = "IDLE"
            self._log_event(
                "PARKING_INTERRUPTED_FOR_TASK",
                agent,
                previous_state=previous_state,
                task_type=result.best_choice,
                source_cell=(result.best_cell.x, result.best_cell.y),
            )
            print(
                f"[PARK] {agent['id']} leaving {previous_state}; "
                f"a {result.best_choice} task is available."
            )

    def _leave_parking_for_task(self, agent):
        self._release_parking_slot(agent)
        _remove_debug_items(agent.get("parking_debug_items", []))
        agent["parking_debug_items"] = []
        if agent.get("state") in ("PARKING", "PARKED"):
            agent["scheduler_agent"] = None
            agent["astar_phase"] = None
        probe = agent.get("parking_probe_future")
        if probe is not None and not probe.done():
            probe.cancel()
        agent["parking_probe_future"] = None
        agent["no_task_since"] = None
        agent["next_task_allocation_time"] = self._sim_time
        self._congestion_staging_agents.discard(int(agent.get("index", -1)))

    def _on_no_valid_task(self, agent, result):
        now = float(self._sim_time)
        progress = self._get_material_progress()
        remaining_count = int(progress.get("remaining_count", 0))
        remaining_mass = float(progress.get("remaining_material_mass", 0.0))
        target_zone = getattr(self, "target_zone", None)
        remaining_positions = []
        if target_zone is not None:
            for point in self._get_live_pebble_centers():
                xy = (float(point[0]), float(point[1]))
                if not target_zone.contains_world(xy[0], xy[1]):
                    remaining_positions.append(
                        [round(xy[0], 4), round(xy[1], 4)]
                    )
        material_signature = (remaining_count, round(remaining_mass, 6))
        if material_signature != self._unfinished_material_signature:
            self._unfinished_material_signature = material_signature
            self._unfinished_material_refreshes = 0

        diagnostics = dict(getattr(result, "diagnostics", {}) or {})
        can_force_refresh = (
            remaining_count > 0
            and self._unfinished_material_refreshes
            < UNFINISHED_MATERIAL_REFRESH_LIMIT
            and not self._pending_execution_map_refresh
            and (
                self.shared_map_future is None
                or self.shared_map_future.done()
            )
        )
        if can_force_refresh:
            self._unfinished_material_refreshes += 1
            agent["state"] = "IDLE"
            agent["no_task_since"] = None
            agent["next_task_allocation_time"] = (
                now + max(0.5, self.parking_retry_interval)
            )
            self._request_execution_map_refresh(
                f"{agent['id']}-unfinished-material"
            )
            self._log_event(
                "UNFINISHED_MATERIAL_MAP_REFRESH",
                agent,
                remaining_count=remaining_count,
                remaining_material_mass=remaining_mass,
                remaining_positions=remaining_positions,
                refresh_attempt=self._unfinished_material_refreshes,
                allocator_diagnostics=diagnostics,
            )
            print(
                f"[MAP-RECOVERY] {remaining_count} pebbles remain outside the "
                f"target but {agent['id']} found no task; rebuilding the live "
                f"expanded map (attempt {self._unfinished_material_refreshes}/"
                f"{UNFINISHED_MATERIAL_REFRESH_LIMIT}). diagnostics={diagnostics}"
            )
            return

        if diagnostics:
            print(
                f"[ALLOC] {agent['id']} no-task diagnostics: {diagnostics}"
            )
        if agent.get("no_task_since") is None:
            agent["no_task_since"] = now
        elapsed = now - float(agent["no_task_since"])
        if not self.enable_idle_parking or elapsed < self.parking_no_task_grace:
            agent["state"] = "IDLE"
            remaining = max(0.0, self.parking_no_task_grace - elapsed)
            agent["next_task_allocation_time"] = now + min(
                self.parking_retry_interval,
                max(0.25, remaining),
            )
            return
        if not self._start_parking(agent):
            agent["state"] = "IDLE"
            agent["next_task_allocation_time"] = now + self.parking_retry_interval

    def _handle_auto_mode(self):
        self._maybe_start_execution_map_refresh()
        self._poll_parking_task_probes()
        for agent in self.agents:
            state_name = agent["state"]
            agent_index = int(agent.get("index", -1))
            if state_name == "IDLE" and agent_index in self._congestion_staging_agents:
                if self._allocation_excluded_task_zones(agent):
                    if self._sim_time >= float(agent.get("next_task_allocation_time", 0.0)):
                        staged = self._start_parking(agent, force=True)
                        if not staged:
                            agent["next_task_allocation_time"] = (
                                self._sim_time + max(1.0, self.parking_retry_interval)
                            )
                    continue
                self._congestion_staging_agents.discard(agent_index)
            pending_exit = agent.get("pending_target_exit_recovery")
            if (
                state_name == "IDLE"
                and self._target_exit_clearance(agent)
                < self._target_exit_required_clearance(agent)
            ):
                reason = (
                    str(pending_exit.get("reason"))
                    if isinstance(pending_exit, dict)
                    else "idle_inside_target_guard"
                )
                if isinstance(pending_exit, dict):
                    self._log_event(
                        "TARGET_KEEP_OUT_REJECTION_RECOVERY",
                        agent,
                        **pending_exit,
                        rejection_count=int(agent.get("path_rejection_count", 0)),
                    )
                self._begin_target_exit(
                    agent,
                    minimum_reverse_time=0.0,
                    reason=reason,
                )
                continue
            if state_name == "IDLE" and pending_exit is not None:
                # The rover may have been moved externally between rejection
                # and this control tick.  Do not retain a stale recovery latch.
                agent["pending_target_exit_recovery"] = None
            if (
                state_name == "IDLE"
                and self._sim_time >= float(agent.get("next_task_allocation_time", 0.0))
            ):
                self._start_planning_for_agent(agent)
            elif (
                state_name in ("PARKING", "PARKED")
                and self._sim_time >= float(agent.get("next_parking_probe_time", 0.0))
            ):
                self._submit_parking_task_probe(agent)


    def _print_scheduler_status(self, active):
        if self._sim_time - self._last_scheduler_status_print < 1.0:
            return
        self._last_scheduler_status_print = self._sim_time
        if not active:
            return
        parts = []
        for ag in active:
            remaining = sched.remaining_path_length(ag)
            mode = ag.get("yield_mode", "normal")
            estop = f"/ESTOP:{ag.get('emergency_blocker')}" if ag.get("emergency_stop") else ""
            hold = "/reserved-slow" if ag.get("time_reserved_hold") else ""
            parts.append(
                f"{ag['id']}:{mode}{estop}{hold} "
                f"s={ag.get('s', 0.0):.2f} rem={remaining:.2f}"
            )
        print("[SCHED] " + " | ".join(parts))

    def _reset_interactions_after_reposition(self, agent, previous_xy, current_xy, jump_m):
        """Accept a finite mouse/GUI reposition and rebuild pose-dependent state."""
        idx = int(agent["index"])
        schedule_agent = agent.get("scheduler_agent")
        progress_before = None
        progress_after = None
        if schedule_agent is not None:
            progress_before = float(schedule_agent.get("s", 0.0))
            schedule_agent["state"] = np.array(agent["state_val"], dtype=float, copy=True)
            schedule_agent["prev_xy"] = np.array(current_xy, dtype=float)
            tracking_xy = self._tracking_point_xy(agent["state_val"], agent)
            progress_after = sched.reproject_agent_progress(schedule_agent, tracking_xy)
            if agent.get("astar_phase") == "APPROACH":
                agent["approach_replan_requested_at"] = self._sim_time

        episode = getattr(self.safety, "episode", None)
        safety_reset = bool(
            episode is not None
            and getattr(episode, "active", False)
            and (
                getattr(episode, "winner_idx", None) == idx
                or idx in getattr(episode, "yielder_indices", set())
            )
        )
        if safety_reset:
            self.safety.episode = type(episode)()
            self.safety.rearm_until = self._sim_time + float(self.safety.config.rearm_delay)
            if hasattr(self.safety, "_active_deadlock"):
                self.safety._active_deadlock = False

        recovery = getattr(self.cooperative_recovery, "episode", None)
        recovery_reset = bool(
            recovery is not None
            and idx in (getattr(recovery, "winner_idx", None), getattr(recovery, "yielder_idx", None))
        )
        if recovery_reset:
            self.cooperative_recovery.episode = None

        fleet_episode = getattr(self.fleet_recovery, "episode", None)
        fleet_recovery_reset = bool(
            fleet_episode is not None
            and idx in getattr(fleet_episode, "component", ())
        )
        if fleet_recovery_reset:
            affected = tuple(fleet_episode.component)
            self.fleet_recovery.episode = None
            for affected_idx in affected:
                affected_agent = next(
                    (
                        value for value in self.agents
                        if int(value["index"]) == int(affected_idx)
                    ),
                    None,
                )
                if affected_agent is not None:
                    self._resume_after_group_recovery(affected_agent)

        self.priority_reservations.clear_agent(idx)
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        self._priority_reservation_holds.discard(idx)

        self._last_interaction_signatures.clear()
        self._log_event(
            "ROVER_EXTERNAL_REPOSITION",
            agent,
            previous_pose_xy=previous_xy,
            pose=agent.get("state_val"),
            jump_m=jump_m,
            scheduler_progress_before_m=progress_before,
            scheduler_progress_after_m=progress_after,
            safety_episode_reset=safety_reset,
            recovery_episode_reset=recovery_reset,
            fleet_recovery_episode_reset=fleet_recovery_reset,
        )
        print(
            f"[REPOSITION] {agent['id']} moved {jump_m:.2f}m externally; "
            "progress reprojected and stale interaction state cleared."
        )

    def _check_runtime_anomalies(self):
        for agent in self.agents:
            idx = int(agent["index"])
            state = agent.get("state_val")
            if state is None:
                continue
            values = [float(value) for value in state]
            reasons = []
            if not all(math.isfinite(value) for value in values):
                reasons.append("non_finite_pose")
                radial = float("inf")
            else:
                radial = math.hypot(values[0], values[1])
                outside_nominal = radial > float(self.env_radius) + OUT_OF_BOUNDS_MARGIN
                if outside_nominal and idx not in self._outside_nominal_agents:
                    self._outside_nominal_agents.add(idx)
                    self._log_event(
                        "ROVER_OUTSIDE_NOMINAL_ARENA",
                        agent,
                        pose=state,
                        radial_distance_m=radial,
                        nominal_env_radius_m=self.env_radius,
                        action="logged_only",
                    )
                    print(
                        f"[ARENA] {agent['id']} is outside the nominal {self.env_radius:.2f}m "
                        f"map radius (r={radial:.2f}m); motion remains enabled."
                    )
                elif not outside_nominal and idx in self._outside_nominal_agents:
                    self._outside_nominal_agents.remove(idx)
                    self._log_event(
                        "ROVER_RETURNED_TO_NOMINAL_ARENA",
                        agent,
                        pose=state,
                        radial_distance_m=radial,
                    )

                previous = self._previous_logged_positions.get(idx)
                current = (values[0], values[1])
                if previous is not None:
                    jump = math.hypot(current[0] - previous[0], current[1] - previous[1])
                    if jump > 0.45:
                        self._reset_interactions_after_reposition(agent, previous, current, jump)
                self._previous_logged_positions[idx] = current

            schedule_agent = agent.get("scheduler_agent")
            trajectory = self._scheduler_path_points(schedule_agent)
            path_metrics = _polyline_metrics(trajectory)
            trajectory_finite = all(
                math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
                for point in trajectory
            )
            if not trajectory_finite:
                reasons.append("non_finite_scheduler_path")
            else:
                planned_radius = float(agent.get("hybrid_env_radius", self.env_radius))
                if path_metrics["max_radius_m"] > planned_radius + PATH_ANOMALY_MARGIN:
                    reasons.append("scheduler_path_outside_planned_envelope")

            if reasons and idx not in self._quarantined_agents:
                self._quarantined_agents.add(idx)
                self._log_event(
                    "ROVER_QUARANTINED", agent,
                    reasons=reasons,
                    pose=state,
                    radial_distance_m=radial,
                    configured_env_radius_m=self.env_radius,
                    rover_state=agent.get("state"),
                    task_phase=agent.get("astar_phase"),
                    task_id=agent.get("event_task_id"),
                    scheduler_goal=schedule_agent.get("goal") if schedule_agent is not None else None,
                    scheduler_path=trajectory,
                    scheduler_path_metrics=path_metrics,
                    selection=agent.get("selection"),
                )
                print(
                    f"[PATH-ANOMALY] {agent['id']} stopped for diagnostics: "
                    f"{','.join(reasons)}. See the event log."
                )
        return set(self._quarantined_agents)

    def _record_runtime_telemetry(self, contexts, nominal_controls, final_controls):
        if self.event_logger is None:
            return
        context_by_idx = {int(ctx.idx): ctx for ctx in contexts}
        safety_episode = self.safety.episode
        recovery_pair = (
            tuple(self._last_recovery_result.active_pair)
            if self._last_recovery_result is not None and self._last_recovery_result.active_pair
            else ()
        )
        recovery_phase = self._last_recovery_result.phase if self._last_recovery_result is not None else None
        fleet_result = self._last_fleet_recovery_result
        fleet_component = (
            tuple(fleet_result.active_component)
            if fleet_result is not None else ()
        )
        fleet_phase = fleet_result.phase if fleet_result is not None else None
        pairwise = []
        for left_index, left in enumerate(self.agents):
            left_state = left.get("state_val")
            if left_state is None:
                continue
            for right in self.agents[left_index + 1:]:
                right_state = right.get("state_val")
                if right_state is None:
                    continue
                pairwise.append({
                    "agents": [left["id"], right["id"]],
                    "distance_m": math.hypot(
                        float(left_state[0]) - float(right_state[0]),
                        float(left_state[1]) - float(right_state[1]),
                    ),
                })

        for agent in self.agents:
            idx = int(agent["index"])
            ctx = context_by_idx.get(idx)
            schedule_agent = agent.get("scheduler_agent") or {}
            yield_mode = schedule_agent.get("yield_mode", "normal")
            emergency_blocker = schedule_agent.get("emergency_blocker") if schedule_agent.get("emergency_stop") else None
            safety_role = "free"
            safety_phase = None
            if safety_episode.active:
                safety_phase = getattr(safety_episode.phase, "value", str(safety_episode.phase))
                if idx == safety_episode.winner_idx:
                    safety_role = "winner"
                elif idx in safety_episode.yielder_indices:
                    safety_role = "yielder"
            recovery_role = "free"
            if recovery_pair:
                recovery_role = "winner" if idx == recovery_pair[0] else ("yielder" if idx == recovery_pair[1] else "free")
            fleet_role = "free"
            if idx in fleet_component:
                if fleet_result is not None and idx == fleet_result.winner_idx:
                    fleet_role = "winner"
                elif fleet_result is not None and idx == fleet_result.mover_idx:
                    fleet_role = "evacuating"
                else:
                    fleet_role = "held_member"
            behavior_name = None
            if ctx is not None:
                try:
                    behavior = self.safety.behavior_for(idx, contexts)
                    behavior_name = getattr(behavior, "value", str(behavior))
                except Exception:
                    behavior_name = None
            signature = (
                agent.get("state"), agent.get("astar_phase"), yield_mode,
                emergency_blocker, safety_phase, safety_role, behavior_name,
                recovery_phase, recovery_role, idx in self._quarantined_agents,
                fleet_phase, fleet_role,
                bool(schedule_agent.get("endpoint_reacquiring", False)),
            )
            if self._last_interaction_signatures.get(idx) != signature:
                self._last_interaction_signatures[idx] = signature
                self._log_event(
                    "CONTROL_INTERACTION_CHANGED", agent,
                    rover_state=agent.get("state"), task_phase=agent.get("astar_phase"),
                    task_id=agent.get("event_task_id"),
                    scheduler_yield_mode=yield_mode,
                    emergency_blocker=emergency_blocker,
                    safety_phase=safety_phase, safety_role=safety_role,
                    safety_behavior=behavior_name,
                    recovery_phase=recovery_phase, recovery_role=recovery_role,
                    fleet_recovery_phase=fleet_phase, fleet_recovery_role=fleet_role,
                    fleet_recovery_component=list(fleet_component),
                    priority=float(ctx.priority) if ctx is not None else None,
                    nominal_control=nominal_controls.get(idx),
                    final_control=final_controls.get(idx),
                    endpoint_reacquiring=bool(schedule_agent.get("endpoint_reacquiring", False)),
                    quarantined=idx in self._quarantined_agents,
                )

        if self._sim_time - self._last_telemetry_time < self.event_logger.pose_interval:
            return
        self._last_telemetry_time = self._sim_time
        self._log_event("PAIRWISE_PROXIMITY", pairs=pairwise)
        self._log_event(
            "MATERIAL_PROGRESS",
            material_value_mode=self.material_value_mode,
            **self._get_material_progress(),
        )
        for agent in self.agents:
            idx = int(agent["index"])
            state = agent.get("state_val")
            if state is None:
                continue
            schedule_agent = agent.get("scheduler_agent") or {}
            nearest = None
            for pair in pairwise:
                if agent["id"] in pair["agents"] and (nearest is None or pair["distance_m"] < nearest["distance_m"]):
                    other = pair["agents"][1] if pair["agents"][0] == agent["id"] else pair["agents"][0]
                    nearest = {"agent_id": other, "distance_m": pair["distance_m"]}
            remaining = None
            target_exit_clearance = None
            target_exit_required = None
            if agent.get("state") == "ROLLBACK":
                target_exit_clearance = self._target_exit_clearance(agent, state)
                target_exit_required = self._target_exit_required_clearance(agent)
            if schedule_agent:
                try:
                    remaining = sched.remaining_path_length(schedule_agent)
                except Exception:
                    remaining = None
            self._log_event(
                "ROVER_STATE", agent,
                task_id=agent.get("event_task_id"),
                rover_type=getattr(agent.get("rover_profile"), "name", "legacy"),
                rover_state=agent.get("state"), task_phase=agent.get("astar_phase"),
                pose=state,
                radial_distance_m=math.hypot(float(state[0]), float(state[1])),
                nominal_control=nominal_controls.get(idx),
                final_control=final_controls.get(idx),
                scheduler_goal=schedule_agent.get("goal"),
                scheduler_progress_m=schedule_agent.get("s"),
                scheduler_remaining_m=remaining,
                endpoint_reacquiring=bool(schedule_agent.get("endpoint_reacquiring", False)),
                endpoint_distance_m=schedule_agent.get("endpoint_distance"),
                endpoint_along_m=schedule_agent.get("endpoint_along"),
                endpoint_cross_track_m=schedule_agent.get("endpoint_cross_track"),
                scheduler_yield_mode=schedule_agent.get("yield_mode", "normal"),
                emergency_stop=bool(schedule_agent.get("emergency_stop", False)),
                emergency_blocker=schedule_agent.get("emergency_blocker"),
                priority=agent.get("priority"),
                nearest_rover=nearest,
                target_exit_clearance_m=target_exit_clearance,
                target_exit_required_clearance_m=target_exit_required,
                target_exit_goal=agent.get("target_exit_goal"),
                group_escape_goal=agent.get("group_escape_goal"),
                group_recovery_phase=fleet_phase if idx in fleet_component else None,
                group_recovery_role=(
                    "winner" if fleet_result is not None and idx == fleet_result.winner_idx
                    else "evacuating" if fleet_result is not None and idx == fleet_result.mover_idx
                    else "held_member" if idx in fleet_component else None
                ),
                quarantined=idx in self._quarantined_agents,
            )

    def _step_agents(self, dt):
        self._sim_time += float(dt)
        self._scheduler_acc += float(dt)
        self._snapshot_pebbles_xy()

        for agent in self.agents:
            agent["state_val"] = _base.get_state(agent["body_id"])
        self._update_benchmark_lifecycle()

        quarantined = self._check_runtime_anomalies()
        self._update_prioritized_trajectory_reservations()
        self._maybe_replan_approach_paths()

        active = self._active_scheduler_agents()
        now = self._sim_time
        for schedule_agent in active:
            sched.update_agent_state_and_progress(schedule_agent, now)

        if active and self._scheduler_acc >= sched.SCHEDULER_DT:
            self._scheduler_acc = 0.0
            # The scheduler's original background replans are goal-directed:
            # current pose -> final goal. That is correct for point-to-point demos
            # but wrong for earth-moving tasks, where the selected push corridor is
            # the actual work. Keep replans disabled unless explicitly requested.
            scheduler_executor = (
                self.replan_executor
                if self.scheduler_allow_replans else None
            )
            sched.run_scheduler(
                active,
                self.env_radius,
                self._sim_time,
                scheduler_executor,
                draw_conflicts=self.draw_scheduler_conflicts,
            )

        if active:
            sched.update_emergency_stops(active, self._sim_time)
            sched.update_close_pair_backoff(active, self._sim_time)

        schedule_by_id = {ag["id"]: ag for ag in active}
        nominal_controls: Dict[int, Tuple[float, float]] = {}

        for agent in self.agents:
            idx = int(agent["index"])
            state_name = agent["state"]
            if state_name in ("IDLE", "PLANNING"):
                nominal_controls[idx] = (0.0, 0.0)
                continue

            if state_name == "PARKED":
                nominal_controls[idx] = (0.0, 0.0)
                continue

            if state_name == "PARKING":
                schedule_agent = schedule_by_id.get(agent["id"])
                if schedule_agent is None:
                    nominal_controls[idx] = (0.0, 0.0)
                    self._release_parking_slot(agent)
                    agent["scheduler_agent"] = None
                    agent["astar_phase"] = None
                    agent["state"] = "IDLE"
                    agent["next_task_allocation_time"] = (
                        self._sim_time + self.parking_retry_interval
                    )
                    continue
                self._update_phase_done(agent, schedule_agent)
                if schedule_agent.get("done"):
                    nominal_controls[idx] = (0.0, 0.0)
                    agent["state"] = "PARKED"
                    agent["scheduler_agent"] = None
                    agent["astar_phase"] = "PARKED"
                    _remove_debug_items(agent.get("parking_debug_items", []))
                    agent["parking_debug_items"] = []
                    self._log_event(
                        "PARKING_REACHED",
                        agent,
                        slot=agent.get("parking_slot_index"),
                        goal=agent.get("parking_goal"),
                        pose=agent.get("state_val"),
                    )
                    print(
                        f"[PARK] {agent['id']} parked in slot "
                        f"{agent.get('parking_slot_index')}."
                    )
                    continue
                v_cmd, w_cmd = sched.compute_path_control(
                    schedule_agent, self._sim_time,
                )
                schedule_agent["control"] = (v_cmd, w_cmd)
                nominal_controls[idx] = (v_cmd, w_cmd)
                continue

            if state_name == "NAVIGATING":
                phase = agent.get("astar_phase", "PUSH")
                if phase == "TURN_TO_PUSH":
                    turn_done, turn_control = self._turn_toward_push_path(agent, dt)
                    nominal_controls[idx] = turn_control
                    if turn_done:
                        nominal_controls[idx] = (0.0, 0.0)
                        self._start_push_phase(agent)
                    continue

                schedule_agent = schedule_by_id.get(agent["id"])
                if schedule_agent is None:
                    nominal_controls[idx] = (0.0, 0.0)
                    agent["state"] = "SYNC"
                    continue

                self._update_phase_done(agent, schedule_agent)
                if schedule_agent.get("done"):
                    if phase == "APPROACH":
                        nominal_controls[idx] = (0.0, 0.0)
                        agent["astar_phase"] = "TURN_TO_PUSH"
                        agent["phase_transition_started_at"] = self._sim_time
                        if self.event_logger is not None:
                            self.event_logger.phase(
                                agent["id"], "TURN_TO_PUSH", self._sim_time,
                                pose=agent.get("state_val"),
                                remaining_m=schedule_agent.get("done_remaining_s"),
                                tracking_distance_m=schedule_agent.get("done_dist_goal"),
                            )
                        print(
                            f"[A*] {agent['id']} reached gate; stopping and "
                            "turning toward 2D push path "
                            f"(remaining={schedule_agent.get('done_remaining_s', 0.0):.3f}m, "
                            f"tracking_dist={schedule_agent.get('done_dist_goal', 0.0):.3f}m)."
                        )
                        continue

                    nominal_controls[idx] = (0.0, 0.0)
                    print(
                        f"  Agent {agent['id']} A* scheduled path complete. Rolling back "
                        f"(remaining={schedule_agent.get('done_remaining_s', 0.0):.3f}m, "
                        f"tracking_dist={schedule_agent.get('done_dist_goal', 0.0):.3f}m)."
                    )
                    self._begin_target_exit(
                        agent,
                        minimum_reverse_time=ROLLBACK_AFTER_PUSH_SEC,
                        reason="push_complete",
                    )
                    if self.event_logger is not None:
                        self.event_logger.phase(
                            agent["id"], "ROLLBACK", self._sim_time,
                            pose=agent.get("state_val"),
                            remaining_m=schedule_agent.get("done_remaining_s"),
                            tracking_distance_m=schedule_agent.get("done_dist_goal"),
                        )
                    continue

                v_cmd, w_cmd = sched.compute_path_control(schedule_agent, self._sim_time)
                schedule_agent["control"] = (v_cmd, w_cmd)
                nominal_controls[idx] = (v_cmd, w_cmd)
                continue

            if state_name == "ROLLBACK":
                agent["rollback_timer"] = max(
                    0.0, float(agent.get("rollback_timer", 0.0)) - dt,
                )
                clearance = self._target_exit_clearance(agent)
                required = self._target_exit_required_clearance(agent)
                base_clearance, shovel_clearances = (
                    self._target_exit_footprint_clearances(agent)
                )
                target_exit_needed = clearance < required
                if target_exit_needed:
                    nominal_controls[idx] = self._target_exit_control(agent)
                    if (
                        agent["rollback_timer"] <= 0.0
                        and not agent.get("target_exit_extended_logged", False)
                    ):
                        agent["target_exit_extended_logged"] = True
                        self._log_event(
                            "TARGET_EXIT_EXTENDED",
                            agent,
                            signed_clearance_m=clearance,
                            required_clearance_m=required,
                            base_clearance_m=base_clearance,
                            shovel_clearances_m=shovel_clearances,
                            direction=agent.get("target_exit_direction"),
                            goal=agent.get("target_exit_goal"),
                        )
                        print(
                            f"[TARGET-EXIT] {agent['id']} remains inside/near the "
                            f"target ({clearance:.2f}m < {required:.2f}m); "
                            "continuing protected reverse exit."
                        )
                elif agent["rollback_timer"] > 0.0:
                    nominal_controls[idx] = (ROLLBACK_SPEED, 0.0)
                else:
                    nominal_controls[idx] = (0.0, 0.0)
                    agent["state"] = "SYNC"
                    self._log_event(
                        "TARGET_EXIT_COMPLETED",
                        agent,
                        signed_clearance_m=clearance,
                        required_clearance_m=required,
                        base_clearance_m=base_clearance,
                        shovel_clearances_m=shovel_clearances,
                    )
                    if self.event_logger is not None:
                        self.event_logger.phase(agent["id"], "SYNC", self._sim_time, pose=agent.get("state_val"))
                continue

            if state_name == "SYNC":
                nominal_controls[idx] = (0.0, 0.0)
                self._step_sync(agent)
                continue

            nominal_controls[idx] = (0.0, 0.0)

        for schedule_agent in active:
            schedule_agent["time_reserved_hold"] = False
        for idx in self._priority_reservation_holds:
            agent = next(
                (value for value in self.agents if int(value["index"]) == int(idx)),
                None,
            )
            if agent is None or agent.get("astar_phase") != "APPROACH":
                continue
            nominal_controls[int(idx)] = (0.0, 0.0)
            schedule_agent = agent.get("scheduler_agent")
            if schedule_agent is not None:
                schedule_agent["time_reserved_hold"] = True

        self._synchronize_conflict_ownership()
        contexts = self._build_scheduled_safety_contexts()
        congestion_mover = self._update_congestion_reassignment(contexts)
        if congestion_mover is not None:
            nominal_controls[int(congestion_mover)] = (0.0, 0.0)
        # Rebuild contexts if an approach was cancelled and replaced by a
        # staging route during this tick.
        contexts = self._build_scheduled_safety_contexts()
        self.safety.update(contexts, self._sim_time)
        final_controls = self.safety.filter_controls(contexts, nominal_controls)
        final_controls = self._apply_cooperative_recovery(contexts, final_controls)
        final_controls = self._apply_fleet_deadlock_recovery(contexts, final_controls)
        for agent in self.agents:
            idx = int(agent["index"])
            control = final_controls.get(idx, (0.0, 0.0))
            if not all(math.isfinite(float(value)) for value in control):
                if idx not in self._quarantined_agents:
                    self._quarantined_agents.add(idx)
                    self._log_event(
                        "ROVER_QUARANTINED", agent,
                        reasons=["non_finite_control"], pose=agent.get("state_val"),
                        nominal_control=nominal_controls.get(idx), final_control=control,
                    )
                    print(f"[PATH-ANOMALY] {agent['id']} stopped for non-finite control. See the event log.")
                quarantined.add(idx)
        for idx in set(quarantined).union(self._quarantined_agents):
            final_controls[int(idx)] = (0.0, 0.0)
        self._record_benchmark_contacts()
        self._record_runtime_telemetry(contexts, nominal_controls, final_controls)
        self._print_scheduled_safety_status(contexts, final_controls)
        for agent in self.agents:
            idx = int(agent["index"])
            v_cmd, w_cmd = final_controls.get(idx, (0.0, 0.0))
            self._apply_agent_control(agent, v_cmd, w_cmd)

        self._print_scheduler_status(active)

    def _step_sync(self, agent):
        completed_reserved_objects = set(agent.get("reserved_object_cells", set()))
        if self.event_logger is not None and agent.get("event_task_id") is not None:
            state_now = agent.get("state_val")
            if state_now is None:
                state_now = _base.get_state(agent["body_id"])
            self.event_logger.finish_task(
                agent["id"], self._sim_time, "completed",
                final_pose=state_now,
                consumed_object_cells=len(completed_reserved_objects),
                material_progress=self._get_material_progress(),
            )
            agent["event_task_id"] = None
        agent["scheduler_agent"] = None
        agent["astar_phase"] = None
        agent["push_world_pts"] = []
        agent["planning_pebbles"] = []
        agent["ignored_pebbles"] = []
        agent["target_exit_boundary"] = None
        agent["target_exit_direction"] = None
        agent["target_exit_goal"] = None
        agent["target_exit_path"] = []
        agent["priority_reserved_blocker_points"] = []
        agent["priority_reservation_reason"] = None
        agent["priority_replan_failures"] = 0
        agent["pending_target_exit_recovery"] = None
        agent["path_rejection_count"] = 0
        agent["group_escape_goal"] = None
        agent["group_escape_route"] = []
        agent["group_recovery_task_aborted"] = False
        agent["group_recovery_original_phase"] = None
        agent["group_recovery_map_refresh"] = False
        self.priority_reservations.clear_agent(int(agent["index"]))
        super()._step_sync(agent)
        agent["no_task_since"] = None
        agent["next_task_allocation_time"] = self._sim_time
        if completed_reserved_objects:
            self._request_execution_map_refresh(agent["id"])


MultiAgentHybridOrchestrator = MultiAStarScheduledHybridOrchestrator


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run multi-rover shared-planning A* scheduled hybrid orchestrator."
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default=SCENARIO_NAME,
        help="Named reproducible scenario. Change DEFAULT_SCENARIO_NAME for IDE Play.",
    )
    parser.add_argument(
        "--rovers",
        type=int,
        choices=sorted(ROVER_POSE_PRESETS),
        default=3,
        help="Number of rovers to spawn.",
    )
    parser.add_argument("--pebbles", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--pebble-deployment",
        choices=tuple(sorted(_base.PEBBLE_DEPLOYMENTS)),
        default="uniform_area",
        help="Deterministic initial spatial distribution family.",
    )
    parser.add_argument(
        "--runtime-mode",
        choices=("gui", "headless", "headless_fast"),
        default="gui",
        help=(
            "gui is the interactive desktop runner; headless keeps real-time "
            "fixed stepping without windows; headless_fast advances the same "
            "240 Hz physics/20 Hz control clock as quickly as the CPU permits."
        ),
    )
    parser.add_argument(
        "--material-mode",
        choices=sorted(MATERIAL_VALUE_MODES),
        default=None,
        help="Use physical pebble count or logical material mass in the heatmap.",
    )
    parser.add_argument(
        "--uniform-small-pebbles",
        action="store_true",
        help=(
            "Override the scenario material distribution with uniform small "
            "pebbles; useful for controlled direct-path comparisons."
        ),
    )
    parser.add_argument("--rover-profiles", default=None, help=f"One profile for all rovers or comma-separated profiles. Available: {','.join(sorted(ROVER_PROFILES))}")
    parser.add_argument(
        "--map-interval",
        type=float,
        default=DEFAULT_MAP_UPDATE_INTERVAL,
        help=(
            "Minimum seconds between completed-path 2D map refreshes. "
            "Refreshes are event-driven by completed 2D push paths."
        ),
    )
    parser.add_argument(
        "--flow-field-vis",
        choices=("never", "ask", "always"),
        default=FLOW_FIELD_VIS_MODE,
        help="3D route visualization mode.",
    )
    parser.add_argument(
        "--phase1-tracking-point",
        choices=("base", "shovel"),
        default="shovel",
    )
    parser.add_argument(
        "--scheduler-replan-workers",
        type=int,
        default=0,
        help="Background workers for experimental scheduler replans.",
    )
    parser.add_argument(
        "--no-scheduler-replans",
        action="store_true",
        help="Compatibility flag: replans are already disabled by default.",
    )
    parser.add_argument(
        "--experimental-scheduler-replans",
        action="store_true",
        help=(
            "Enable the original scheduler's goal-directed replans. "
            "This may bypass selected push corridors."
        ),
    )
    parser.add_argument(
        "--draw-conflicts",
        action="store_true",
        default=SHOW_3D_CONFLICT_MARKERS,
        help="Draw scheduler conflict markers in PyBullet.",
    )
    parser.add_argument(
        "--no-draw-conflicts",
        dest="draw_conflicts",
        action="store_false",
        help="Hide scheduler conflict markers in PyBullet.",
    )
    parser.add_argument(
        "--show-planning-boundaries",
        dest="draw_planning_boundaries",
        action="store_true",
        default=SHOW_3D_PLANNING_BOUNDARIES,
        help="Draw nominal and expanded planning-boundary rings in PyBullet.",
    )
    parser.add_argument(
        "--hide-planning-boundaries",
        dest="draw_planning_boundaries",
        action="store_false",
        help="Do not draw the nominal/expanded planning-boundary rings.",
    )
    parser.add_argument(
        "--no-draw-execution-paths",
        dest="no_draw_execution_paths",
        action="store_true",
        default=not SHOW_3D_PATHS,
        help="Hide split 3D path debug lines for A* approach and 2D push paths.",
    )
    parser.add_argument(
        "--draw-execution-paths",
        dest="no_draw_execution_paths",
        action="store_false",
        help="Show split 3D path debug lines for A* approach and 2D push paths.",
    )
    parser.add_argument(
        "--hide-3d-paths",
        dest="no_draw_execution_paths",
        action="store_true",
        help="Alias for --no-draw-execution-paths.",
    )
    parser.add_argument(
        "--push-extra-distance",
        type=float,
        default=DEFAULT_PUSH_EXTRA_DISTANCE,
        help="Extra meters appended to the end of each 2D push path.",
    )
    parser.add_argument(
        "--approach-replan-interval",
        type=float,
        default=DEFAULT_APPROACH_REPLAN_INTERVAL,
        help=(
            "Seconds between approach-only A* replans. Set 0 to disable periodic "
            "approach replanning; priority emergency yields still request replans."
        ),
    )
    parser.add_argument(
        "--navigation-outside-margin",
        type=float,
        default=DEFAULT_NAVIGATION_OUTSIDE_MARGIN,
        help=(
            "Meters outside the material-map radius available for approach, "
            "turning, detours, and cooperative collision recovery."
        ),
    )
    parser.add_argument(
        "--no-idle-parking",
        dest="idle_parking",
        action="store_false",
        default=ENABLE_IDLE_PARKING,
        help="Keep no-task rovers stationary instead of sending them to parking slots.",
    )
    parser.add_argument(
        "--parking-no-task-grace",
        type=float,
        default=DEFAULT_PARKING_NO_TASK_GRACE,
        help="Seconds with no valid task before a rover starts parking.",
    )
    parser.add_argument(
        "--parking-task-poll-interval",
        type=float,
        default=DEFAULT_PARKING_TASK_POLL_INTERVAL,
        help="Seconds between background task checks while parking or parked.",
    )
    parser.add_argument(
        "--parking-ring-offset",
        type=float,
        default=DEFAULT_PARKING_RING_OFFSET,
        help="Desired parking-ring offset outside the material-map radius.",
    )
    parser.add_argument(
        "--parking-slot-count",
        type=int,
        default=DEFAULT_PARKING_SLOT_COUNT,
        help="Number of reservable positions on the parking ring.",
    )
    parser.add_argument(
        "--cooperative-clear-after",
        type=float,
        default=DEFAULT_COOPERATIVE_CLEAR_AFTER,
        help="Seconds in a failed close-pair recovery before both rovers clear.",
    )
    parser.add_argument(
        "--fleet-deadlock-recovery",
        dest="fleet_deadlock_recovery",
        action="store_true",
        default=ENABLE_FLEET_DEADLOCK_RECOVERY,
        help="Enable connected-group deadlock evacuation above pairwise recovery.",
    )
    parser.add_argument(
        "--no-fleet-deadlock-recovery",
        dest="fleet_deadlock_recovery",
        action="store_false",
        help="Disable connected-group deadlock evacuation.",
    )
    parser.add_argument(
        "--fleet-deadlock-stuck-duration",
        type=float,
        default=DEFAULT_FLEET_DEADLOCK_STUCK_DURATION,
        help="Seconds without measured group progress before fleet recovery starts.",
    )
    parser.add_argument(
        "--fleet-deadlock-escape-distance",
        type=float,
        default=DEFAULT_FLEET_DEADLOCK_ESCAPE_DISTANCE,
        help="Base distance for verified group-evacuation staging moves.",
    )
    parser.add_argument(
        "--priority-reservations",
        dest="priority_reservations",
        action="store_true",
        default=ENABLE_PRIORITY_TRAJECTORY_RESERVATIONS,
        help="Plan approach paths in priority order using predicted rover trajectories.",
    )
    parser.add_argument(
        "--no-priority-reservations",
        dest="priority_reservations",
        action="store_false",
        help="Disable priority-aware trajectory reservations.",
    )
    parser.add_argument(
        "--priority-reservation-horizon",
        type=float,
        default=DEFAULT_PRIORITY_RESERVATION_HORIZON,
        help="Seconds of future rover motion reserved during approach planning.",
    )
    parser.add_argument(
        "--priority-reservation-margin",
        type=float,
        default=DEFAULT_PRIORITY_RESERVATION_MARGIN,
        help="Extra meters added around the two profile collision radii.",
    )
    parser.add_argument(
        "--priority-stalled-after",
        type=float,
        default=DEFAULT_PRIORITY_STALLED_AFTER,
        help="Seconds without motion before a yielder becomes a fixed obstacle.",
    )
    parser.add_argument(
        "--congestion-reassignment",
        dest="congestion_reassignment",
        action="store_true",
        default=ENABLE_CONGESTION_AWARE_REASSIGNMENT,
        help="Enable task cancellation, staging, and temporary work-zone exclusion after persistent approach congestion.",
    )
    parser.add_argument(
        "--no-congestion-reassignment",
        dest="congestion_reassignment",
        action="store_false",
        help="Keep the existing recovery-only behavior (default).",
    )
    parser.add_argument("--congestion-window", type=float, default=DEFAULT_CONGESTION_WINDOW)
    parser.add_argument("--congestion-min-gate-progress", type=float, default=DEFAULT_CONGESTION_MIN_GATE_PROGRESS)
    parser.add_argument("--congestion-conflict-fraction", type=float, default=DEFAULT_CONGESTION_CONFLICT_FRACTION)
    parser.add_argument("--congestion-failed-replans", type=int, default=DEFAULT_CONGESTION_FAILED_REPLANS)
    parser.add_argument("--congestion-same-blocker-samples", type=int, default=DEFAULT_CONGESTION_SAME_BLOCKER_SAMPLES)
    parser.add_argument("--congestion-cooldown", type=float, default=DEFAULT_CONGESTION_COOLDOWN)
    parser.add_argument("--congestion-zone-radius", type=float, default=DEFAULT_CONGESTION_ZONE_RADIUS)
    parser.add_argument("--congestion-rearm", type=float, default=DEFAULT_CONGESTION_REARM)
    parser.add_argument(
        "--target-root-sources-only",
        dest="target_root_sources_only",
        action="store_true",
        default=TARGET_ROOT_SOURCES_ONLY,
        help="Allow direct target tasks only from upstream/root source cells.",
    )
    parser.add_argument(
        "--allow-non-root-target-sources",
        dest="target_root_sources_only",
        action="store_false",
        help="Allow any valid source cell for direct target tasks.",
    )
    parser.add_argument(
        "--target-source-mode",
        choices=("all", "root", "convex_hull"),
        default=TARGET_SOURCE_MODE,
        help=(
            "Source eligibility for direct target tasks: all valid sources, "
            "legacy push-graph roots, or live geometric convex-hull sources. "
            "When omitted, the legacy root/profile setting is preserved."
        ),
    )
    parser.add_argument(
        "--target-path-mode",
        choices=("material_aware", "straight_nearest"),
        default=DIRECT_TARGET_PATH_MODE,
        help=(
            "material_aware uses the existing heat/fabric overlay; "
            "straight_nearest pushes directly to the nearest target boundary."
        ),
    )
    parser.add_argument(
        "--target-candidate-value-mode",
        choices=("material_aware", "source_only"),
        default=TARGET_CANDIDATE_VALUE_MODE,
        help=(
            "material_aware values all corridor material; source_only values "
            "only the selected source while still logging actual collection."
        ),
    )
    parser.add_argument(
        "--capacity-scoring-mode",
        choices=tuple(sorted(_planning_overlay.CAPACITY_SCORING_MODES)),
        default=CAPACITY_SCORING_MODE,
        help=(
            "profile_cap clips predicted corridor value to nominal rover "
            "capacity; uncapped_corridor leaves capacity out of task value."
        ),
    )
    parser.add_argument(
        "--comparison-label",
        default=COMPARISON_LABEL,
        help="Optional experiment label recorded in RUN_CONFIG and run summary metadata.",
    )
    parser.add_argument(
        "--auto-exit-on-completion",
        action="store_true",
        help=(
            "Close PyBullet/Pygame automatically only after all material is in "
            "the target and every rover has reached PARKED."
        ),
    )
    parser.add_argument(
        "--max-sim-time",
        type=float,
        default=None,
        help="Optional simulated-time limit in seconds; non-positive disables it.",
    )
    parser.add_argument(
        "--max-no-delivery-time",
        type=float,
        default=None,
        help=(
            "Optional simulated seconds without an increase in delivered "
            "pebbles before marking the run stalled; non-positive disables it."
        ),
    )
    parser.add_argument(
        "--v-max",
        type=float,
        default=sched.V_MAX,
        help="Maximum forward speed command in m/s.",
    )
    parser.add_argument(
        "--w-max",
        type=float,
        default=sched.W_MAX,
        help="Maximum yaw-rate command in rad/s.",
    )
    parser.add_argument(
        "--path-stop-s",
        type=float,
        default=DEFAULT_PATH_STOP_S,
        help="Remaining arc length threshold for stopping at a path endpoint.",
    )
    parser.add_argument(
        "--goal-dist-tol",
        type=float,
        default=DEFAULT_GOAL_DIST_TOL,
        help="Tracking-point distance tolerance for endpoint completion.",
    )
    parser.add_argument(
        "--goal-relaxed-remaining-s",
        type=float,
        default=DEFAULT_GOAL_REMAINING_S_TOL,
        help="Relaxed remaining-arc threshold for path completion.",
    )
    parser.add_argument(
        "--goal-relaxed-dist-tol",
        type=float,
        default=DEFAULT_GOAL_DONE_DIST_TOL,
        help="Relaxed tracking-point distance tolerance for path completion.",
    )
    parser.add_argument(
        "--event-log-dir",
        default=None,
        help="Directory for JSONL telemetry and CSV task summaries.",
    )
    parser.add_argument(
        "--event-log-pose-interval",
        type=float,
        default=DEFAULT_EVENT_LOG_POSE_INTERVAL,
        help="Seconds between detailed rover-state samples.",
    )
    parser.add_argument(
        "--no-event-log",
        action="store_true",
        help="Disable automatic structured simulation logging.",
    )
    parser.add_argument(
        "--no-benchmark-telemetry", action="store_true",
        help="Disable per-push physics attribution, snapshots, and run summaries.",
    )
    parser.add_argument(
        "--no-benchmark-images", action="store_true",
        help="Keep benchmark CSV/JSON data but do not save standardized 2D PNGs.",
    )
    parser.add_argument(
        "--no-benchmark-push-images", action="store_true",
        help="Capture milestone/map images but skip before/after-push PNGs.",
    )
    parser.add_argument(
        "--benchmark-observation-delay", type=float,
        default=BENCHMARK_POST_PUSH_OBSERVATION_DELAY,
        help="Simulated seconds after a push before measuring retained delivery/spillage.",
    )
    parser.add_argument(
        "--benchmark-corridor-margin", type=float,
        default=BENCHMARK_PUSH_CORRIDOR_MARGIN,
        help="Extra meters around half the shovel width for lateral-spillage classification.",
    )
    parser.add_argument(
        "--benchmark-reference-highway-ratio", type=float,
        default=BENCHMARK_REFERENCE_HIGHWAY_RATIO,
        help="Fixed normalized heat threshold used only for cross-run highway evaluation.",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    _planning_overlay.CAPACITY_SCORING_MODE = (
        _planning_overlay.normalize_capacity_scoring_mode(args.capacity_scoring_mode)
    )
    planning_2d_config = _configure_2d_planner()
    sched.V_MAX = max(0.05, float(args.v_max))
    scenario = get_scenario(args.scenario)
    if TARGET_ZONE_CENTER is not None:
        scenario = scenario.with_target_center(TARGET_ZONE_CENTER)
    pebble_count = scenario.num_pebbles if args.pebbles is None else args.pebbles
    random_seed = scenario.random_seed if args.seed is None else args.seed
    material_mode = scenario.material_mode if args.material_mode is None else args.material_mode
    if args.rover_profiles is None:
        scenario_profiles = [
            name.strip() for name in scenario.rover_profiles.split(",") if name.strip()
        ]
        rover_profile_names = ",".join(
            scenario_profiles[index % len(scenario_profiles)] for index in range(args.rovers))
    else:
        rover_profile_names = args.rover_profiles
    pebble_distribution = (
        {"small": 1.0, "medium": 0.0, "large": 0.0}
        if args.uniform_small_pebbles
        else dict(scenario.pebble_distribution)
    )
    sched.W_MAX = max(0.10, float(args.w_max))
    sched.PATH_STOP_S = max(0.005, float(args.path_stop_s))
    sched.GOAL_DIST_TOL = max(0.005, float(args.goal_dist_tol))
    sched.GOAL_REMAINING_S_TOL = max(sched.PATH_STOP_S, float(args.goal_relaxed_remaining_s))
    sched.GOAL_DONE_DIST_TOL = max(sched.GOAL_DIST_TOL, float(args.goal_relaxed_dist_tol))
    SIMULATION_CONFIG["use_spillage_model"] = True
    SIMULATION_CONFIG["visualize_potential"] = False

    initial_robot_poses = ROVER_POSE_PRESETS[args.rovers]
    rover_profiles = _resolved_profiles_with_overrides(
        rover_profile_names, args.rovers
    )
    rover_policy_configs = [
        {"rover_type": profile.name, **_policy_log_config(profile.policy)}
        for profile in rover_profiles
    ]
    print(
        "Config: "
        f"Rovers={args.rovers}, "
        f"Profiles={','.join(profile.name for profile in rover_profiles)}, "
        f"Scenario={scenario.name}, Target={scenario.target_zone.to_dict()}, "
        f"PebbleMaterialMode={material_mode}, "
        f"PebbleDistribution={pebble_distribution}, "
        f"PebbleDeployment={args.pebble_deployment}, "
        f"TaskPolicies={rover_policy_configs}, "
        f"Planning2D={planning_2d_config}, "
        f"SharedMapInterval={args.map_interval:.1f}s, "
        f"Spillage2D=True, "
        f"3D_Vis={args.flow_field_vis}, "
        f"Phase1Track={args.phase1_tracking_point}, "
        "Motion=A*_scheduled, "
        f"SchedulerReplans={args.experimental_scheduler_replans and not args.no_scheduler_replans}, "
        f"PushExtra={args.push_extra_distance:.2f}m, "
        f"ApproachReplan={args.approach_replan_interval:.1f}s, "
        f"NavigationOutsideMargin={args.navigation_outside_margin:.2f}m, "
        f"IdleParking={args.idle_parking}, "
        f"ParkingGrace={args.parking_no_task_grace:.1f}s, "
        f"ParkingRingOffset={args.parking_ring_offset:.2f}m, "
        f"CooperativeClearAfter={args.cooperative_clear_after:.1f}s, "
        f"FleetDeadlockRecovery={args.fleet_deadlock_recovery}, "
        f"FleetStuck={args.fleet_deadlock_stuck_duration:.1f}s, "
        f"FleetEscape={args.fleet_deadlock_escape_distance:.2f}m, "
        f"PriorityReservations={args.priority_reservations}, "
        f"ReservationHorizon={args.priority_reservation_horizon:.1f}s, "
        f"CongestionReassignment={args.congestion_reassignment}, "
        f"CongestionWindow={args.congestion_window:.1f}s, "
        f"CongestionCooldown={args.congestion_cooldown:.1f}s, "
        f"TargetRootOnly={args.target_root_sources_only if args.target_root_sources_only is not None else 'profile'}, "
        f"TargetSourceMode={args.target_source_mode or 'legacy/profile'}, "
        f"TargetPathMode={args.target_path_mode}, "
        f"TargetCandidateValue={args.target_candidate_value_mode}, "
        f"CapacityScoring={args.capacity_scoring_mode}, "
        f"Comparison={args.comparison_label or 'none'}, "
        f"RuntimeMode={args.runtime_mode}, "
        f"AutoExit={args.auto_exit_on_completion}, "
        f"MaxSimTime={args.max_sim_time}, "
        f"MaxNoDelivery={args.max_no_delivery_time}, "
        f"DrawPaths={not args.no_draw_execution_paths}, "
        f"BoundaryRings={args.draw_planning_boundaries}, "
        f"Vmax={sched.V_MAX:.2f}m/s, "
        f"Wmax={sched.W_MAX:.2f}rad/s, "
        f"StopS={sched.PATH_STOP_S:.2f}m, "
        f"GoalTol={sched.GOAL_DIST_TOL:.2f}m, "
        f"EventLog={not args.no_event_log}"
        f", Benchmark={not args.no_benchmark_telemetry}, "
        f"BenchmarkImages={not args.no_benchmark_images}"
    )

    orch = MultiAStarScheduledHybridOrchestrator(
        env_radius=scenario.env_radius,
        target_zone_radius=scenario.target_zone.bounding_radius,
        target_zone=scenario.target_zone,
        scenario_name=scenario.name,
        num_pebbles=pebble_count,
        random_seed=random_seed,
        headless=args.runtime_mode != "gui",
        fast_headless=args.runtime_mode == "headless_fast",
        initial_robot_poses=initial_robot_poses,
        shovel_width=0.22,
        rover_profiles=rover_profiles,
        material_value_mode=material_mode,
        pebble_distribution=pebble_distribution,
        pebble_deployment=args.pebble_deployment,
        auto_mode=True,
        phase1_tracking_point=args.phase1_tracking_point,
        map_update_interval=args.map_interval,
        scheduler_replan_workers=args.scheduler_replan_workers,
        scheduler_allow_replans=(
            args.experimental_scheduler_replans and not args.no_scheduler_replans
        ),
        draw_scheduler_conflicts=args.draw_conflicts,
        draw_execution_paths=not args.no_draw_execution_paths,
        draw_planning_boundaries=args.draw_planning_boundaries,
        push_extra_distance=args.push_extra_distance,
        approach_replan_interval=args.approach_replan_interval,
        navigation_outside_margin=args.navigation_outside_margin,
        enable_idle_parking=args.idle_parking,
        parking_no_task_grace=args.parking_no_task_grace,
        parking_task_poll_interval=args.parking_task_poll_interval,
        parking_ring_offset=args.parking_ring_offset,
        parking_slot_count=args.parking_slot_count,
        cooperative_clear_after=args.cooperative_clear_after,
        enable_fleet_deadlock_recovery=args.fleet_deadlock_recovery,
        fleet_deadlock_stuck_duration=args.fleet_deadlock_stuck_duration,
        fleet_deadlock_escape_distance=args.fleet_deadlock_escape_distance,
        enable_priority_reservations=args.priority_reservations,
        priority_reservation_horizon=args.priority_reservation_horizon,
        priority_reservation_margin=args.priority_reservation_margin,
        priority_stalled_after=args.priority_stalled_after,
        enable_congestion_reassignment=args.congestion_reassignment,
        congestion_window=args.congestion_window,
        congestion_min_gate_progress=args.congestion_min_gate_progress,
        congestion_conflict_fraction=args.congestion_conflict_fraction,
        congestion_failed_replans=args.congestion_failed_replans,
        congestion_same_blocker_samples=args.congestion_same_blocker_samples,
        congestion_cooldown=args.congestion_cooldown,
        congestion_zone_radius=args.congestion_zone_radius,
        congestion_rearm=args.congestion_rearm,
        target_root_sources_only=args.target_root_sources_only,
        target_source_mode=args.target_source_mode,
        target_path_mode=args.target_path_mode,
        target_candidate_value_mode=args.target_candidate_value_mode,
        comparison_label=args.comparison_label,
        synchronous_planning=args.runtime_mode == "headless_fast",
        auto_exit_on_completion=args.auto_exit_on_completion,
        max_sim_time=args.max_sim_time,
        max_no_delivery_time=args.max_no_delivery_time,
        event_log_dir=args.event_log_dir,
        event_log_pose_interval=args.event_log_pose_interval,
        enable_event_log=not args.no_event_log,
        enable_benchmark_telemetry=not args.no_benchmark_telemetry,
        benchmark_capture_images=not args.no_benchmark_images,
        benchmark_capture_every_push=not args.no_benchmark_push_images,
        benchmark_observation_delay=args.benchmark_observation_delay,
        benchmark_corridor_margin=args.benchmark_corridor_margin,
        benchmark_reference_highway_ratio=args.benchmark_reference_highway_ratio,
    )
    orch.scenario_config = scenario.to_dict()
    orch.scenario_config["comparison"] = {
        "label": args.comparison_label,
        "target_path_mode": args.target_path_mode,
        "target_candidate_value_mode": args.target_candidate_value_mode,
        "capacity_scoring_mode": args.capacity_scoring_mode,
        "target_root_sources_only": args.target_root_sources_only,
        "target_source_mode": args.target_source_mode,
        "target_visibility_mode": TARGET_VISIBILITY_MODE,
        "target_visibility_angle": TARGET_VISIBILITY_ANGLE,
    }
    orch.flow_field_vis_mode = args.flow_field_vis
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()




































