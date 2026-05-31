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
from multi_agent_collision_safety import SafetyAgentContext
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
SHOW_3D_PATHS = False              # A* approach, connector, 2D push, and extension lines.
SHOW_3D_CONFLICT_MARKERS = False   # Scheduler conflict/debug markers.
FLOW_FIELD_VIS_MODE = "never"      # "never", "ask", or "always".
DEFAULT_PUSH_EXTRA_DISTANCE = 0.25 # Extra meters at the end of each 2D push path.
DEFAULT_MAP_UPDATE_INTERVAL = 12.0 # Minimum seconds between completed-path 2D map rebuilds.
DEFAULT_APPROACH_REPLAN_INTERVAL = 2.5 # Seconds between approach-only A* replans.
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
    "PUSH": 0.0,
    "TURN_TO_PUSH": 0.10,
    "APPROACH": 1.0,
    "ROLLBACK": 1.5,
}
APPROACH_REPLAN_KEEP_OUT_RADIUS = 0.32
APPROACH_REPLAN_KEEP_OUT_SAMPLES = 10
TASK_BACKOFF_SPEED = 0.24
TASK_BACKOFF_TURN_KP = 3.5
TASK_BACKOFF_CLEAR_DISTANCE = 0.82


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
        max_radius = max(max_radius, math.hypot(float(point[0]), float(point[1])))
    return max_radius + float(margin)


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


def _stamp_target_zone_keepout(field, keepout_radius: float) -> None:
    if keepout_radius <= 0.0:
        return
    c0x, c0y = field.world_to_cell(0.0, 0.0)
    steps_x = int(math.ceil(keepout_radius / max(field.cell_w, 1e-9)))
    steps_y = int(math.ceil(keepout_radius / max(field.cell_h, 1e-9)))
    for dy in range(-steps_y, steps_y + 1):
        for dx in range(-steps_x, steps_x + 1):
            ix, iy = c0x + dx, c0y + dy
            if 0 <= ix < field.grid_w and 0 <= iy < field.grid_h:
                wx, wy = field.cell_to_world_center(ix, iy)
                if math.hypot(wx, wy) <= keepout_radius:
                    field.obstacles[iy, ix] = True


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
):
    grid_dim = _approach_grid_dim(env_radius)
    field = build_obstacle_field(
        env_radius,
        grid_dim,
        grid_dim,
        pebble_centers,
        ASTAR_APPROACH_CONFIG["rover_radius"],
        ASTAR_APPROACH_CONFIG["pebble_radius"],
        ASTAR_APPROACH_CONFIG["clearance"],
    )
    if block_target_zone:
        _stamp_target_zone_keepout(
            field,
            target_zone_radius + ASTAR_APPROACH_CONFIG["target_zone_guard_margin"],
        )
    return field


def _plan_approach_astar(
    env_radius: float,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    pebble_centers: Sequence[Tuple[float, float]],
    target_zone_radius: float,
):
    full_field = _build_approach_field(
        env_radius,
        pebble_centers,
        target_zone_radius,
        block_target_zone=True,
    )
    raw_path = AStarGridPlanner(full_field).plan(start_xy, goal_xy)
    if raw_path is not None:
        return full_field, raw_path, list(pebble_centers), [], "full_target_keepout"

    if not ASTAR_APPROACH_CONFIG["allow_relaxation"]:
        return full_field, None, list(pebble_centers), [], "failed"

    inflated_radius = (
        ASTAR_APPROACH_CONFIG["rover_radius"]
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
):
    if len(trajectory_points) < 2:
        raise RuntimeError(f"{agent_id}: schedule trajectory must have at least two points.")

    planning_field = _build_approach_field(
        env_radius,
        planning_pebbles,
        target_zone_radius=0.0,
        block_target_zone=False,
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
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.safety = ScheduledTaskPrioritySafety(self.safety.config)
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

    def run(self):
        try:
            super().run()
        finally:
            if self.replan_executor is not None:
                self.replan_executor.shutdown(wait=False, cancel_futures=True)

    def _setup_agent_selection(self, agent, cell, choice, path_info):
        traj = path_info["path"]
        print(
            f"Agent {agent['id']} planned {choice} task path: "
            f"{len(traj)} grid waypoints, dist={path_info.get('distance', 0.0):.2f}"
        )

        self.visualizer.set_trajectory_preview(cell, choice, path_info)

        grid_wps = [(c.x, c.y) for c in traj]
        spline_pts, _, success = _base.smooth_path_with_spline(
            grid_wps,
            SPILLAGE_CONFIG["smoothing_factor"],
            SPILLAGE_CONFIG["num_points"],
        )
        if not success:
            spline_pts = grid_wps

        coord_converter = agent.get("coord_converter", self.coord_converter)
        world_pts = [
            coord_converter.convert_2d_to_3d(gx, gy)
            for gx, gy in spline_pts
        ]
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
        G = S0 - float(GATE_CONFIG["gate_back"]) * v_path

        state_now = _base.get_state(agent["body_id"])
        yaw = float(state_now[2])
        if self.phase1_tracking_point == "shovel":
            start_xy = (
                float(state_now[0]) + sched.SHOVEL_OFFSET * math.cos(yaw),
                float(state_now[1]) + sched.SHOVEL_OFFSET * math.sin(yaw),
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
            priority=float(agent["index"]),
            color=agent["color"],
            start_xy=start_xy,
            gate_xy=_as_xy_tuple(G),
            push_world_pts=world_pts,
            env_radius=env_r,
            target_zone_radius=self.target_zone_radius,
            pebble_centers=live_pebbles,
            allow_replan=self.scheduler_allow_replans,
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
        }

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

    def _tracking_point_xy(self, state):
        yaw = float(state[2])
        if self.phase1_tracking_point == "shovel":
            return np.array(
                [
                    float(state[0]) + sched.SHOVEL_OFFSET * math.cos(yaw),
                    float(state[1]) + sched.SHOVEL_OFFSET * math.sin(yaw),
                ],
                dtype=float,
            )
        return np.array([float(state[0]), float(state[1])], dtype=float)

    def _turn_toward_push_path(self, agent, dt):
        state = _base.get_state(agent["body_id"])
        push_pts = [np.array(pt, dtype=float) for pt in agent.get("push_world_pts", [])]
        if len(push_pts) < 2:
            return True, (0.0, 0.0)

        tracking_xy = self._tracking_point_xy(state)
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
        tracking_xy = self._tracking_point_xy(state_now)
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

        push_schedule_agent = _build_schedule_agent_from_points(
            agent["id"],
            float(agent["index"]),
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

    def _update_phase_done(self, schedule_agent):
        if schedule_agent.get("done"):
            return

        remaining = sched.remaining_path_length(schedule_agent)
        tracking_xy = self._tracking_point_xy(schedule_agent["state"])
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
        if strict_done or near_path_end_done:
            schedule_agent["done"] = True
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
        return float(base_priority) + 1e-3 * float(agent.get("index", 0))

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
            active = state_name in ("NAVIGATING", "ROLLBACK")
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
        start_xy = tuple(self._tracking_point_xy(state_now))
        live_pebbles = list(self._get_live_pebble_centers())
        blocker_points = self._approach_replan_blockers(agent)
        planning_obstacles = live_pebbles + blocker_points
        env_r = max(
            float(agent.get("hybrid_env_radius", self.env_radius)),
            self.calculate_dynamic_env_radius(live_pebbles),
            _required_env_radius([start_xy, gate_xy] + list(push_world_pts), ASTAR_APPROACH_CONFIG["env_margin"]),
            self.env_radius,
        )

        try:
            t0 = time.perf_counter()
            schedule_agent = _build_hybrid_astar_schedule_agent(
                agent_id=agent["id"],
                priority=float(agent["index"]),
                color=agent["color"],
                start_xy=start_xy,
                gate_xy=_as_xy_tuple(gate_xy),
                push_world_pts=push_world_pts,
                env_radius=env_r,
                target_zone_radius=self.target_zone_radius,
                pebble_centers=planning_obstacles,
                allow_replan=self.scheduler_allow_replans,
            )
        except Exception as exc:
            agent["last_approach_replan_time"] = self._sim_time
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

        print(
            f"[A*] {agent['id']} replanned APPROACH ({reason}): "
            f"path={schedule_agent['geom']['total_L']:.2f}m, "
            f"blockers={len(blocker_points)}, env_r={env_r:.2f}m"
        )
        return True

    def _maybe_replan_approach_paths(self):
        for agent in self.agents:
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

    def _task_backoff_control(self, yielder_ctx, winner_ctx):
        y_pos = np.array(yielder_ctx.state[:2], dtype=float)
        w_pos = np.array(winner_ctx.state[:2], dtype=float)
        away = y_pos - w_pos
        away_norm = float(np.linalg.norm(away))
        if away_norm < 1e-6:
            away = np.array([1.0, 0.0], dtype=float)
            away_norm = 1.0
        away /= away_norm

        yaw = float(yielder_ctx.state[2])
        reverse_dir = np.array([-math.cos(yaw), -math.sin(yaw)], dtype=float)
        if float(np.dot(reverse_dir, away)) > 0.25:
            return -min(TASK_BACKOFF_SPEED, 0.45 * sched.V_MAX), 0.0

        desired_yaw = math.atan2(float(w_pos[1] - y_pos[1]), float(w_pos[0] - y_pos[0]))
        e_theta = sched.wrap_angle(desired_yaw - yaw)
        w_cmd = max(-sched.W_MAX, min(sched.W_MAX, TASK_BACKOFF_TURN_KP * e_theta))
        return 0.0, w_cmd

    def _apply_task_priority_backoff(self, contexts, controls):
        by_idx = {ctx.idx: ctx for ctx in contexts}
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        out = dict(controls)
        emergency_dist = float(self.safety.config.emergency_stop_distance)
        handled = set()

        for a in contexts:
            if not a.active:
                continue
            for b in contexts:
                if not b.active or b.idx <= a.idx:
                    continue
                d = float(np.linalg.norm(np.array(a.state[:2], dtype=float) - np.array(b.state[:2], dtype=float)))
                if d >= emergency_dist:
                    continue
                if abs(float(a.priority) - float(b.priority)) < 0.01:
                    continue
                winner, yielder = (a, b) if float(a.priority) < float(b.priority) else (b, a)
                winner_agent = agent_by_idx.get(winner.idx)
                yielder_agent = agent_by_idx.get(yielder.idx)
                if winner_agent is None or yielder_agent is None:
                    continue
                if winner_agent.get("astar_phase") not in ("PUSH", "TURN_TO_PUSH"):
                    continue

                out[winner.idx] = (0.0, 0.0)
                out[yielder.idx] = self._task_backoff_control(yielder, winner)
                yielder_agent["approach_replan_requested_at"] = self._sim_time
                yielder_agent["approach_replan_blockers"] = [winner.idx]
                handled.add((winner.agent_id, yielder.agent_id, d))

        if handled and self._sim_time - self._last_task_backoff_print >= 0.75:
            self._last_task_backoff_print = self._sim_time
            parts = [
                f"{winner}->{yielder} d={d:.3f}m"
                for winner, yielder, d in sorted(handled)
            ]
            print("[SAFETY] Task-priority backoff: " + " | ".join(parts))
        return out

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
            f"(epoch={snapshot.epoch}, objects={snapshot.pebbles_count})"
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

    def _step_agents(self, dt):
        self._sim_time += float(dt)
        self._scheduler_acc += float(dt)
        self._snapshot_pebbles_xy()

        for agent in self.agents:
            agent["state_val"] = _base.get_state(agent["body_id"])

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

                self._update_phase_done(schedule_agent)
                if schedule_agent.get("done"):
                    if phase == "APPROACH":
                        nominal_controls[idx] = (0.0, 0.0)
                        agent["astar_phase"] = "TURN_TO_PUSH"
                        agent["phase_transition_started_at"] = self._sim_time
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
                continue

            if state_name == "SYNC":
                nominal_controls[idx] = (0.0, 0.0)
                self._step_sync(agent)
                continue

            nominal_controls[idx] = (0.0, 0.0)

        contexts = self._build_scheduled_safety_contexts()
        self.safety.update(contexts, self._sim_time)
        final_controls = self.safety.filter_controls(contexts, nominal_controls)
        final_controls = self._apply_task_priority_backoff(contexts, final_controls)
        self._print_scheduled_safety_status(contexts, final_controls)
        for agent in self.agents:
            idx = int(agent["index"])
            v_cmd, w_cmd = final_controls.get(idx, (0.0, 0.0))
            self._apply_agent_control(agent, v_cmd, w_cmd)

        self._print_scheduler_status(active)

    def _step_sync(self, agent):
        completed_reserved_objects = set(agent.get("reserved_object_cells", set()))
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
        "--rovers",
        type=int,
        choices=sorted(ROVER_POSE_PRESETS),
        default=3,
        help="Number of rovers to spawn.",
    )
    parser.add_argument("--pebbles", type=int, default=50)
    parser.add_argument("--seed", type=int, default=41)
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
    return parser.parse_args()


def main():
    args = _parse_args()
    sched.V_MAX = max(0.05, float(args.v_max))
    sched.W_MAX = max(0.10, float(args.w_max))
    sched.PATH_STOP_S = max(0.005, float(args.path_stop_s))
    sched.GOAL_DIST_TOL = max(0.005, float(args.goal_dist_tol))
    sched.GOAL_REMAINING_S_TOL = max(sched.PATH_STOP_S, float(args.goal_relaxed_remaining_s))
    sched.GOAL_DONE_DIST_TOL = max(sched.GOAL_DIST_TOL, float(args.goal_relaxed_dist_tol))
    SIMULATION_CONFIG["use_spillage_model"] = True
    SIMULATION_CONFIG["visualize_potential"] = False

    initial_robot_poses = ROVER_POSE_PRESETS[args.rovers]
    print(
        "Config: "
        f"Rovers={args.rovers}, "
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
        f"GoalTol={sched.GOAL_DIST_TOL:.2f}m"
    )

    orch = MultiAStarScheduledHybridOrchestrator(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=args.pebbles,
        random_seed=args.seed,
        initial_robot_poses=initial_robot_poses,
        shovel_width=0.22,
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
    )
    orch.flow_field_vis_mode = args.flow_field_vis
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()
