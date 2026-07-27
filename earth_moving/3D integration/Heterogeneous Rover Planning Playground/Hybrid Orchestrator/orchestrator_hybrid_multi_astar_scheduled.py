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

import orchestrator_hybrid_multi_shared_safe as _shared
import multi_astar_priority_scheduling3 as sched
from rover_profiles import ROVER_PROFILES, resolve_profiles
from rover_planning_overlay import bounded_approach_gate, world_path_is_sane
from multi_agent_collision_safety import SafetyAgentContext
from cooperative_collision_recovery import CooperativeRecoveryManager, RecoveryAgent
from simulation_event_logger import SimulationEventLogger
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

ROLLBACK_AFTER_PUSH_SEC = 1.0
ROLLBACK_SPEED = -0.5

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
SCENARIO_NAME = l_shape_target
# None keeps the scenario's own center. Set (x, y) to move any selected shape.
# Example: TARGET_ZONE_CENTER = (1.25, 0.65)
TARGET_ZONE_CENTER = (1.25, 0.65)
SHOW_3D_PATHS = False              # A* approach, connector, 2D push, and extension lines.
SHOW_3D_CONFLICT_MARKERS = False   # Scheduler conflict/debug markers.
FLOW_FIELD_VIS_MODE = "never"      # "never", "ask", or "always".
DEFAULT_PUSH_EXTRA_DISTANCE = 0.25 # Extra meters at the end of each 2D push path.
DEFAULT_MAP_UPDATE_INTERVAL = 12.0 # Minimum seconds between completed-path 2D map rebuilds.
DEFAULT_APPROACH_REPLAN_INTERVAL = 2.5 # Seconds between approach-only A* replans.
ENABLE_EVENT_LOG = True             # Automatic JSONL telemetry + CSV task summary.
DEFAULT_EVENT_LOG_POSE_INTERVAL = 0.50
PEBBLE_MATERIAL_MODE = DEFAULT_MATERIAL_VALUE_MODE  # "mass" or "count" thesis comparison.
PEBBLE_DISTRIBUTION = dict(DEFAULT_PEBBLE_DISTRIBUTION)
OUT_OF_BOUNDS_MARGIN = 0.75         # Quarantine a rover beyond the configured arena.
PATH_ANOMALY_MARGIN = 1.00          # Reject a controller path outside the arena envelope.
DEFAULT_ROVER_PROFILES = "small,large,small" # Used when clicking Play with the default 3 rovers.
# ==============================================================

DRAW_EXECUTION_PATHS_DEFAULT = SHOW_3D_PATHS
GATE_TURN_TOL_DEG = 8.0
GATE_TURN_KP = 5.0
DEFAULT_PATH_STOP_S = 0.05
DEFAULT_GOAL_DIST_TOL = 0.06
DEFAULT_GOAL_REMAINING_S_TOL = 0.10
DEFAULT_GOAL_DONE_DIST_TOL = 0.10
APPROACH_REPLAN_MIN_GAP = 0.75
PROTECTED_PHASE_PRIORITIES = {
    # Wide gaps guarantee task phase dominates any rover-type tie-break bias.
    "PUSH": 0.0,
    "TURN_TO_PUSH": 10.0,
    "APPROACH": 20.0,
    "ROLLBACK": 30.0,
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


def _as_xy_tuple(point) -> Tuple[float, float]:
    return float(point[0]), float(point[1])


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
        push_extra_distance: float = DEFAULT_PUSH_EXTRA_DISTANCE,
        approach_replan_interval: float = DEFAULT_APPROACH_REPLAN_INTERVAL,
        event_log_dir: Optional[str] = None,
        event_log_pose_interval: float = DEFAULT_EVENT_LOG_POSE_INTERVAL,
        enable_event_log: bool = ENABLE_EVENT_LOG,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.safety = ScheduledTaskPrioritySafety(self.safety.config)
        # The cooperative manager below owns reverse recovery in this runner.
        if hasattr(self.safety.config, "enable_reverse_recovery"):
            self.safety.config.enable_reverse_recovery = False
        sched.BACKOFF_ESCAPE_ENABLED = False
        self.cooperative_recovery = CooperativeRecoveryManager()
        self.scheduler_replan_workers = max(0, int(scheduler_replan_workers))
        self.scheduler_allow_replans = bool(scheduler_allow_replans)
        self.draw_scheduler_conflicts = bool(draw_scheduler_conflicts)
        self.draw_execution_paths = bool(draw_execution_paths)
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
        self._previous_logged_positions = {}
        self._quarantined_agents = set()
        self._outside_nominal_agents = set()
        self.event_logger = None
        if enable_event_log:
            try:
                self.event_logger = SimulationEventLogger(
                    event_log_dir or os.path.join(HERE, "simulation_logs"),
                    pose_interval=event_log_pose_interval,
                )
            except Exception as exc:
                print(f"[EVENT-LOG] Could not create event log: {exc}")

    def _log_event(self, event_type, agent=None, **data):
        if self.event_logger is None:
            return
        agent_id = None if agent is None else str(agent.get("id"))
        self.event_logger.event(event_type, self._sim_time, agent_id, **data)

    def initialize(self):
        super().initialize()
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
            f"draw_paths={self.draw_execution_paths})"
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
                } for agent in self.agents],
                material_value_mode=self.material_value_mode,
                pebble_distribution=self.pebble_distribution,
                pebble_summary=summarize_instances(self.pebble_instances),
                initial_material_progress=self._get_material_progress(),
                env_radius=self.env_radius,
                target_zone_radius=self.target_zone_radius,
                target_zone=self.target_zone.to_dict(),
                scenario_name=self.scenario_name,
                scenario=getattr(self, "scenario_config", None),
                pose_interval=self.event_logger.pose_interval,
            )

    def run(self):
        try:
            super().run()
        finally:
            if self.replan_executor is not None:
                self.replan_executor.shutdown(wait=False, cancel_futures=True)
            if self.event_logger is not None:
                self.event_logger.close(self._sim_time)

    def _setup_agent_selection(self, agent, cell, choice, path_info):
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
            planning_radius + 0.15,
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

        live_pebbles = self._get_live_pebble_centers()
        dynamic_env_r = self.calculate_dynamic_env_radius(live_pebbles)
        required_env_r = _required_env_radius(
            [start_xy, G] + list(world_pts),
            margin=ASTAR_APPROACH_CONFIG["env_margin"],
        )
        # Keep a common, non-shrinking A* frame for all active rovers. The
        # scheduler compares raw A* cell coordinates between rovers, so letting
        # one rover plan on a smaller dynamic grid can create nonsense conflict
        # cells and boundary-clamped starts after previous pushes.
        env_r = max(self.env_radius, dynamic_env_r, required_env_r)

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
            pebble_centers=live_pebbles,
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
        agent["planning_pebbles"] = list(schedule_agent.get("planning_pebbles", live_pebbles))
        agent["ignored_pebbles"] = list(schedule_agent.get("ignored_pebbles", []))
        agent["phase_transition_started_at"] = None
        agent["last_approach_replan_time"] = self._sim_time
        agent["approach_replan_requested_at"] = None
        agent["approach_replan_blockers"] = []
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
                policy_fallback_order=path_info.get("policy_fallback_order"),
                expected_objects=path_info.get("expected_collected_objects", path_info.get("expected_collected")),
                expected_material_mass=path_info.get("expected_collected_mass"),
                expected_planning_quantity=path_info.get("expected_collected"),
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
            if agent.get("state") != "NAVIGATING":
                continue
            if agent.get("astar_phase") not in ("APPROACH", "PUSH"):
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

    def _task_phase_priority(self, agent) -> float:
        phase = agent.get("astar_phase")
        state_name = agent.get("state")
        key = "ROLLBACK" if state_name == "ROLLBACK" else phase
        base_priority = PROTECTED_PHASE_PRIORITIES.get(key, 2.0)
        profile = agent.get("rover_profile")
        raw_rover_priority = float(profile.right_of_way_priority) if profile is not None else 0.0
        rover_priority = max(-4.0, min(4.0, raw_rover_priority))
        return float(base_priority) + rover_priority + 1e-3 * float(agent.get("index", 0))

    def _apply_cooperative_recovery(self, contexts, controls):
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
        for idx in result.replan_indices:
            agent = agent_by_idx.get(int(idx))
            if agent is None or agent.get("state") != "NAVIGATING":
                continue
            if agent.get("astar_phase") == "APPROACH":
                agent["approach_replan_requested_at"] = self._sim_time
                pair = result.active_pair or ()
                agent["approach_replan_blockers"] = [value for value in pair if value != idx]
        for message in result.messages:
            print(message)
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
                )
            self._last_recovery_signature = signature
        elif result.messages:
            self._log_event(
                "COLLISION_RECOVERY_MESSAGE",
                pair=signature[0], phase=signature[1], messages=result.messages,
            )
        self._last_recovery_result = result
        return result.controls
    def _protected_path_points(self, agent):
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
            x, y, yaw = float(state[0]), float(state[1]), float(state[2])
            return x - 0.50 * math.cos(yaw), y - 0.50 * math.sin(yaw)

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
                state_name in ("NAVIGATING", "ROLLBACK")
                and idx not in self._quarantined_agents
            )
            protected_points = self._protected_path_points(agent)
            path_constrained = bool(
                active
                and state_name == "NAVIGATING"
                and phase in ("PUSH", "TURN_TO_PUSH")
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
        for other in self.agents:
            if other is agent:
                continue
            if other.get("astar_phase") in ("PUSH", "TURN_TO_PUSH") or int(other["index"]) in requested:
                state = other.get("state_val")
                if state is None:
                    state = _base.get_state(other["body_id"])
                blockers.extend(_rover_keepout_points((float(state[0]), float(state[1]))))
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
            _required_env_radius([start_xy, gate_xy] + list(push_world_pts), ASTAR_APPROACH_CONFIG["env_margin"]),
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
        except Exception as exc:
            agent["last_approach_replan_time"] = self._sim_time
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

    def _maybe_replan_approach_paths(self):
        for agent in self.agents:
            if int(agent.get("index", -1)) in self._quarantined_agents:
                continue
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
            reason = "priority-yield" if requested else "periodic"
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

        print(
            f"[MAP] Shared 2D map ready "
            f"(epoch={snapshot.epoch}, objects={snapshot.pebbles_count}, "
            f"mass={snapshot.pebbles_material_mass:.0f}, mode={snapshot.material_value_mode})"
        )

    def _maybe_start_execution_map_refresh(self) -> None:
        self._poll_shared_map_result()
        if not self._pending_execution_map_refresh:
            return
        if self.map_executor is None:
            return
        if self.shared_map_future is not None and not self.shared_map_future.done():
            return

        now = time.time()
        last_built = self.shared_map.built_wall_time if self.shared_map is not None else 0.0
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
        if self.shared_map_future is not None and not self.shared_map_future.done():
            print(f"[MAP] Submitted completed-path 2D refresh ({agents}).")
        else:
            self._consumed_cells_in_submitted_refresh = set()
            self._pending_execution_map_refresh = True

    def _handle_auto_mode(self):
        self._maybe_start_execution_map_refresh()
        for agent in self.agents:
            if agent["state"] == "IDLE":
                self._start_planning_for_agent(agent)


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
                quarantined=idx in self._quarantined_agents,
            )

    def _step_agents(self, dt):
        self._sim_time += float(dt)
        self._scheduler_acc += float(dt)
        self._snapshot_pebbles_xy()

        for agent in self.agents:
            agent["state_val"] = _base.get_state(agent["body_id"])

        quarantined = self._check_runtime_anomalies()
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
                    agent["state"] = "ROLLBACK"
                    agent["rollback_timer"] = ROLLBACK_AFTER_PUSH_SEC
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
                agent["rollback_timer"] -= dt
                if agent["rollback_timer"] > 0:
                    nominal_controls[idx] = (ROLLBACK_SPEED, 0.0)
                else:
                    nominal_controls[idx] = (0.0, 0.0)
                    agent["state"] = "SYNC"
                    if self.event_logger is not None:
                        self.event_logger.phase(agent["id"], "SYNC", self._sim_time, pose=agent.get("state_val"))
                continue

            if state_name == "SYNC":
                nominal_controls[idx] = (0.0, 0.0)
                self._step_sync(agent)
                continue

            nominal_controls[idx] = (0.0, 0.0)

        contexts = self._build_scheduled_safety_contexts()
        self.safety.update(contexts, self._sim_time)
        final_controls = self.safety.filter_controls(contexts, nominal_controls)
        final_controls = self._apply_cooperative_recovery(contexts, final_controls)
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
        super()._step_sync(agent)
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
        "--material-mode",
        choices=sorted(MATERIAL_VALUE_MODES),
        default=None,
        help="Use physical pebble count or logical material mass in the heatmap.",
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
    return parser.parse_args()


def main():
    args = _parse_args()
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
    pebble_distribution = dict(scenario.pebble_distribution)
    sched.W_MAX = max(0.10, float(args.w_max))
    sched.PATH_STOP_S = max(0.005, float(args.path_stop_s))
    sched.GOAL_DIST_TOL = max(0.005, float(args.goal_dist_tol))
    sched.GOAL_REMAINING_S_TOL = max(sched.PATH_STOP_S, float(args.goal_relaxed_remaining_s))
    sched.GOAL_DONE_DIST_TOL = max(sched.GOAL_DIST_TOL, float(args.goal_relaxed_dist_tol))
    SIMULATION_CONFIG["use_spillage_model"] = True
    SIMULATION_CONFIG["visualize_potential"] = False

    initial_robot_poses = ROVER_POSE_PRESETS[args.rovers]
    rover_profiles = resolve_profiles(rover_profile_names, args.rovers)
    print(
        "Config: "
        f"Rovers={args.rovers}, "
        f"Profiles={','.join(profile.name for profile in rover_profiles)}, "
        f"Scenario={scenario.name}, Target={scenario.target_zone.to_dict()}, "
        f"PebbleMaterialMode={material_mode}, "
        f"PebbleDistribution={pebble_distribution}, "
        f"SharedMapInterval={args.map_interval:.1f}s, "
        f"Spillage2D=True, "
        f"3D_Vis={args.flow_field_vis}, "
        f"Phase1Track={args.phase1_tracking_point}, "
        "Motion=A*_scheduled, "
        f"SchedulerReplans={args.experimental_scheduler_replans and not args.no_scheduler_replans}, "
        f"PushExtra={args.push_extra_distance:.2f}m, "
        f"ApproachReplan={args.approach_replan_interval:.1f}s, "
        f"DrawPaths={not args.no_draw_execution_paths}, "
        f"Vmax={sched.V_MAX:.2f}m/s, "
        f"Wmax={sched.W_MAX:.2f}rad/s, "
        f"StopS={sched.PATH_STOP_S:.2f}m, "
        f"GoalTol={sched.GOAL_DIST_TOL:.2f}m, "
        f"EventLog={not args.no_event_log}"
    )

    orch = MultiAStarScheduledHybridOrchestrator(
        env_radius=scenario.env_radius,
        target_zone_radius=scenario.target_zone.bounding_radius,
        target_zone=scenario.target_zone,
        scenario_name=scenario.name,
        num_pebbles=pebble_count,
        random_seed=random_seed,
        initial_robot_poses=initial_robot_poses,
        shovel_width=0.22,
        rover_profiles=rover_profiles,
        material_value_mode=material_mode,
        pebble_distribution=pebble_distribution,
        auto_mode=True,
        phase1_tracking_point=args.phase1_tracking_point,
        map_update_interval=args.map_interval,
        scheduler_replan_workers=args.scheduler_replan_workers,
        scheduler_allow_replans=(
            args.experimental_scheduler_replans and not args.no_scheduler_replans
        ),
        draw_scheduler_conflicts=args.draw_conflicts,
        draw_execution_paths=not args.no_draw_execution_paths,
        push_extra_distance=args.push_extra_distance,
        approach_replan_interval=args.approach_replan_interval,
        event_log_dir=args.event_log_dir,
        event_log_pose_interval=args.event_log_pose_interval,
        enable_event_log=not args.no_event_log,
    )
    orch.scenario_config = scenario.to_dict()
    orch.flow_field_vis_mode = args.flow_field_vis
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()




































