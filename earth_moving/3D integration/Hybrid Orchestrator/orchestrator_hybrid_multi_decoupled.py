"""
orchestrator_hybrid_multi_decoupled.py

Multi-agent hybrid orchestrator with **per-agent asynchronous planning**.

Key properties:
- PyBullet physics stays on the main thread (240Hz).
- Control loop runs at 20Hz.
- Each agent owns its own planning Future + env_2d cache (optional) + plan_id.
- Planning runs in a background executor and never calls PyBullet.
- Main thread snapshots world state (pebbles + agent pose) and submits jobs.
- Results are applied on the main thread only, with plan_id versioning to avoid races.

NOTE:
- This file assumes you already have:
  - main.py exposing run_2d_env, compute_2d_env
  - coordinate_converter.py (CoordinateConverter)
  - spillage_model.py (smooth_path_with_spline)
  - navigation_manager.py (NavigationManager)
  - 2_wheel_rover.urdf, pebbles.urdf in same folder
"""

import os
import sys
import time
import math
import numpy as np
import pygame
import pybullet as p
import pybullet_data
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List, Set
import concurrent.futures as cf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import run_2d_env, compute_2d_env
from coordinate_converter import CoordinateConverter
from spillage_model import smooth_path_with_spline
from navigation_manager import NavigationManager

# ===================== CONFIG =====================
SIMULATION_CONFIG = {
    "use_spillage_model": True,
    "visualize_potential": True,  # WARNING: if this shows matplotlib windows it can still block
    "show_grid": True,
}

SPILLAGE_CONFIG = {
    "smoothing_factor": 0.5,
    "num_points": 1000,
    "target_spacing": 0.03,
}

GATE_CONFIG = {
    "gate_back": 0.50,
    "approach_tol": 0.15,
}

PLANNING_CONFIG = {
    # Use processes for true parallelism (CPU heavy). If env_2d is not picklable, set False.
    "use_process_pool": False,
    "max_workers": 2,

    # If True, each agent will keep its own env_2d and reuse it until it replans.
    # (We still recompute env_2d during planning; this flag just controls storage.)
    "store_env_per_agent": True,

    # If True, selection will consider global cell reservations (coordination).
    "use_global_reservations": True,

    # If False, use initial env_radius always; if True, we can recompute radius at snapshot time.
    "dynamic_env_radius": False,
}


# ===================== Helpers =====================
def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def get_state(body_id) -> np.ndarray:
    """[x, y, yaw, v_forward, w]"""
    pos, orn = p.getBasePositionAndOrientation(body_id)
    x, y = pos[0], pos[1]
    yaw = p.getEulerFromQuaternion(orn)[2]
    lin_vel, ang_vel = p.getBaseVelocity(body_id)
    vx, vy = lin_vel[0], lin_vel[1]
    v_fwd = math.cos(yaw) * vx + math.sin(yaw) * vy
    w = ang_vel[2]
    return np.array([x, y, yaw, v_fwd, w], dtype=float)


def set_wheel_velocities(body_id, left_joint, right_joint, v_cmd, w_cmd,
                         wheel_radius=0.07, track_width=0.20,
                         max_wheel_speed=20.0, max_torque=5.0):
    vL = v_cmd - w_cmd * track_width / 2.0
    vR = v_cmd + w_cmd * track_width / 2.0
    wL = -vL / wheel_radius
    wR = -vR / wheel_radius
    wL = max(min(wL, max_wheel_speed), -max_wheel_speed)
    wR = max(min(wR, max_wheel_speed), -max_wheel_speed)
    p.setJointMotorControl2(body_id, left_joint, p.VELOCITY_CONTROL,
                            targetVelocity=wL, force=max_torque)
    p.setJointMotorControl2(body_id, right_joint, p.VELOCITY_CONTROL,
                            targetVelocity=wR, force=max_torque)


def find_wheel_joints(body_id):
    left = right = None
    for j in range(p.getNumJoints(body_id)):
        name = p.getJointInfo(body_id, j)[1].decode("utf-8")
        if name == "base_to_lwheel":
            left = j
        elif name == "base_to_rwheel":
            right = j
    return left, right


# ===================== Planning data contracts =====================
@dataclass(frozen=True)
class WorldSnapshot:
    env_radius: float
    target_zone_radius: float
    shovel_width: float
    agent_id: str
    agent_pose_xy: Tuple[float, float]
    pebbles_xy: np.ndarray                    # (N,2)
    global_reserved_cells: Tuple[Tuple[int, int], ...]
    plan_id: int


@dataclass
class PlanResult:
    agent_id: str
    plan_id: int
    env_radius: float
    env_2d: Any
    best_cell: Any
    best_choice: str
    best_path_info: Optional[Dict[str, Any]]
    best_reserved: Set[Tuple[int, int]]
    error: Optional[str] = None


def _pick_best_path_for_agent(env_2d, coord_converter: CoordinateConverter,
                             rover_pos_xy: Tuple[float, float],
                             reserved_global: Set[Tuple[int, int]]) -> Tuple[Any, str, Optional[Dict], Set[Tuple[int, int]]]:
    """
    Mirrors your existing auto-mode selection:
    - iterate env_2d.cells_with_objects
    - take env_2d.get_path_for_preview(cell, "target")[0]
    - score objects - 0.5*distance(rover->start)
    - reject if required_cells overlaps reserved_global
    """
    best_cell = None
    best_score = -float("inf")
    best_path_info = None
    best_choice = "target"
    best_reserved = set()

    for cell in getattr(env_2d, "cells_with_objects", []):
        paths = env_2d.get_path_for_preview(cell, "target")
        if not paths:
            continue
        path_info = paths[0]
        objects = path_info.get("objects", 0)
        if objects <= 0:
            continue

        traj = path_info.get("path", [])
        if not traj:
            continue

        required_cells = {(c.x, c.y) for c in traj}
        impacted = path_info.get("impacted_cells", {})
        required_cells.update(impacted.keys())

        if reserved_global and required_cells.intersection(reserved_global):
            continue

        start_c = traj[0]
        start_wx, start_wy = coord_converter.convert_2d_to_3d(start_c.x, start_c.y)
        dist_to_start = math.hypot(rover_pos_xy[0] - start_wx, rover_pos_xy[1] - start_wy)

        score = objects - 0.5 * dist_to_start
        if score > best_score:
            best_score = score
            best_cell = cell
            best_path_info = path_info
            best_reserved = required_cells

    return best_cell, best_choice, best_path_info, best_reserved


def plan_for_agent(snapshot: WorldSnapshot) -> PlanResult:
    """
    Worker entrypoint.
    MUST NOT call PyBullet.
    """
    try:
        pebbles_3d = [(float(x), float(y), 0.01) for x, y in snapshot.pebbles_xy]
        env_2d = compute_2d_env(
            snapshot.env_radius,
            snapshot.target_zone_radius,
            snapshot.shovel_width,
            pebbles_3d,
            manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG["use_spillage_model"],
            visualize_potential=SIMULATION_CONFIG["visualize_potential"],
        )

        coord_converter = CoordinateConverter(snapshot.env_radius, snapshot.target_zone_radius, snapshot.shovel_width)

        reserved = set(snapshot.global_reserved_cells) if snapshot.global_reserved_cells else set()
        best_cell, best_choice, best_path_info, best_reserved = _pick_best_path_for_agent(
            env_2d, coord_converter, snapshot.agent_pose_xy, reserved
        )

        return PlanResult(
            agent_id=snapshot.agent_id,
            plan_id=snapshot.plan_id,
            env_radius=snapshot.env_radius,
            env_2d=env_2d,
            best_cell=best_cell,
            best_choice=best_choice,
            best_path_info=best_path_info,
            best_reserved=best_reserved,
            error=None,
        )
    except Exception as e:
        return PlanResult(
            agent_id=snapshot.agent_id,
            plan_id=snapshot.plan_id,
            env_radius=snapshot.env_radius,
            env_2d=None,
            best_cell=None,
            best_choice="target",
            best_path_info=None,
            best_reserved=set(),
            error=str(e),
        )


# ===================== Main Orchestrator =====================
class MultiAgentHybridOrchestratorDecoupled:
    def __init__(self, env_radius=3.0, target_zone_radius=0.8, num_pebbles=50,
                 random_seed=41, initial_robot_poses=None,
                 shovel_width=0.22, auto_mode=True):
        self.env_radius = float(env_radius)
        self.target_zone_radius = float(target_zone_radius)
        self.num_pebbles = int(num_pebbles)
        self.random_seed = int(random_seed)
        self.shovel_width = float(shovel_width)
        self.auto_mode = bool(auto_mode)

        self.initial_robot_poses = initial_robot_poses or [(2.0, 2.0, -math.pi / 2)]
        self.num_rovers = len(self.initial_robot_poses)

        # global coordination only (not global env_2d)
        self.global_reserved_cells: Set[Tuple[int, int]] = set()

        # PyBullet objects
        self.agents: List[Dict[str, Any]] = []
        self.pebble_ids: List[int] = []
        self.pebble_centers: List[Tuple[float, float]] = []

        # 2D env + visualizer for UI only (we keep one, updated from agent 0's latest plan)
        self.visualizer = None
        self.coord_converter = CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width)

        # executor
        self.executor = None

        # UI
        self.running = False
        self._needs_redraw = True

        # visualization mode (optional)
        self.flow_field_vis_mode = "never"  # "ask" "always" "never"

    def initialize(self):
        print("=" * 60)
        print("Initializing Multi-Agent Hybrid Orchestrator (Decoupled Planning)")
        print("=" * 60)

        # Executor
        if PLANNING_CONFIG["use_process_pool"]:
            self.executor = cf.ProcessPoolExecutor(max_workers=PLANNING_CONFIG["max_workers"])
        else:
            self.executor = cf.ThreadPoolExecutor(max_workers=PLANNING_CONFIG["max_workers"])

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(5.0, 45, -60, [0, 0, 0])
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)

        # Ground
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Target zone visualization
        tz_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.target_zone_radius,
            length=0.01,
            rgbaColor=[1, 0, 0, 0.3],
        )
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=tz_vis, basePosition=[0, 0, 0.005])

        # Spawn rovers
        here = os.path.dirname(os.path.abspath(__file__))
        rover_urdf = os.path.join(here, "2_wheel_rover.urdf")

        agent_colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
            (0.1, 1.0, 0.5, 1.0),
            (1.0, 0.1, 0.5, 1.0),
            (0.5, 0.1, 1.0, 1.0),
        ]

        for i, (x, y, yaw) in enumerate(self.initial_robot_poses):
            body_id = p.loadURDF(rover_urdf, [x, y, 0.02], yaw_to_quat(yaw))
            left_j, right_j = find_wheel_joints(body_id)

            p.changeDynamics(body_id, -1, lateralFriction=0.8)
            for j in (left_j, right_j):
                p.changeDynamics(body_id, j, lateralFriction=1.0, rollingFriction=0.0, spinningFriction=0.0)
                p.setJointMotorControl2(body_id, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)

            p.changeVisualShape(body_id, -1, rgbaColor=agent_colors[i % len(agent_colors)])

            grid_dim = int(math.ceil(2 * self.env_radius / 0.15))
            if grid_dim % 2 == 0:
                grid_dim += 1

            nav = NavigationManager(
                world_bounds=(-self.env_radius, self.env_radius, -self.env_radius, self.env_radius),
                grid_size=(grid_dim, grid_dim),
                rover_radius=0.25,
                pebble_radius=0.05,
            )

            self.agents.append({
                "id": f"R{i}",
                "index": i,
                "body_id": body_id,
                "left_joint": left_j,
                "right_joint": right_j,
                "color": agent_colors[i % len(agent_colors)],

                # state machine
                "state": "IDLE",  # IDLE, NAVIGATING, ROLLBACK, PLANNING

                # navigation
                "navigator": nav,
                "selection": None,

                # coordination
                "reserved_cells": set(),

                # planning async
                "plan_id": 0,
                "future": None,
                "pending_result": None,

                # per-agent 2D cache (optional)
                "env_2d": None,
                "coord_converter": CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width),
            })
            print(f"  Rover {i} at ({x:.2f}, {y:.2f})")

        # Spawn pebbles
        pebble_urdf = os.path.join(here, "pebbles.urdf")
        np.random.seed(self.random_seed)

        for _ in range(self.num_pebbles):
            r_min = self.target_zone_radius
            r_max = self.env_radius
            r = math.sqrt(np.random.rand() * (r_max ** 2 - r_min ** 2) + r_min ** 2)
            phi = 2 * math.pi * np.random.rand()
            px, py = r * math.cos(phi), r * math.sin(phi)
            bid = p.loadURDF(pebble_urdf, [px, py, 0.01], useFixedBase=False)
            p.changeDynamics(bid, -1, lateralFriction=0.8)
            self.pebble_ids.append(bid)
            self.pebble_centers.append((px, py))

        print(f"  {len(self.pebble_centers)} pebbles spawned")

        # Create ONE visualizer env for UI (optional): reuse your run_2d_env just once
        pebbles_3d = [(x, y, 0.01) for x, y in self.pebble_centers]
        env_2d, self.visualizer = run_2d_env(
            self.env_radius, self.target_zone_radius, self.shovel_width,
            pebbles_3d,
            manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG["use_spillage_model"],
            visualize_potential=SIMULATION_CONFIG["visualize_potential"],
        )
        # Store it only for visualization
        self.visualizer.update_env(env_2d)
        self._needs_redraw = True

        print("\n[READY] Auto-mode =", self.auto_mode)
        print("=" * 60)

    # ------------- Main loop -------------
    def run(self):
        self.running = True
        sim_dt = 1 / 240.0

        last_time = time.time()
        physics_acc = 0.0
        control_acc = 0.0
        display_acc = 0.0

        print("\nStarting main execution loop...")

        while self.running and p.isConnected():
            now = time.time()
            dt_real = min(now - last_time, 0.1)
            last_time = now

            physics_acc += dt_real
            control_acc += dt_real
            display_acc += dt_real

            # (1) Physics 240Hz
            while physics_acc >= sim_dt:
                p.stepSimulation()
                physics_acc -= sim_dt

            # (2) Control 20Hz
            if control_acc >= 0.05:
                self._step_agents(0.05)
                self._poll_planning_results()
                control_acc = control_acc % 0.05

            # (3) Display / events ~15Hz
            if display_acc >= 1 / 15.0:
                self._handle_events()
                if self.auto_mode:
                    self._auto_allocate_idle_agents()
                if self._needs_redraw:
                    self._needs_redraw = False
                    self._update_visualizer()
                display_acc = 0.0

        self.shutdown()

    def shutdown(self):
        self.running = False
        try:
            if self.executor:
                self.executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        try:
            if p.isConnected():
                p.disconnect()
        except Exception:
            pass

    # ------------- World snapshots (MAIN THREAD ONLY) -------------
    def _snapshot_pebbles_xy(self) -> np.ndarray:
        # Query positions on main thread
        pts = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            if pos[2] > -0.5:
                pts.append((float(pos[0]), float(pos[1])))
        self.pebble_centers = pts
        if len(pts) == 0:
            return np.zeros((0, 2), dtype=float)
        return np.array(pts, dtype=float)

    def _calc_env_radius(self) -> float:
        if not PLANNING_CONFIG["dynamic_env_radius"]:
            return self.env_radius
        # Optional: simple dynamic radius based on farthest pebble
        if not self.pebble_centers:
            return self.env_radius
        max_r = max(math.hypot(x, y) for x, y in self.pebble_centers)
        return float(max(self.target_zone_radius + 0.5, min(max_r + 0.6, self.env_radius)))

    # ------------- Planning submission / receive -------------
    def _start_planning_for_agent(self, agent: Dict[str, Any]):
        # Do not start if one is running
        fut = agent.get("future", None)
        if fut is not None and not fut.done():
            return

        pebbles_xy = self._snapshot_pebbles_xy()
        env_r = self._calc_env_radius()

        # versioning
        agent["plan_id"] += 1
        pid = agent["plan_id"]

        st = get_state(agent["body_id"])
        rover_xy = (float(st[0]), float(st[1]))

        reserved = tuple(self.global_reserved_cells) if PLANNING_CONFIG["use_global_reservations"] else tuple()

        snap = WorldSnapshot(
            env_radius=env_r,
            target_zone_radius=self.target_zone_radius,
            shovel_width=self.shovel_width,
            agent_id=agent["id"],
            agent_pose_xy=rover_xy,
            pebbles_xy=pebbles_xy,
            global_reserved_cells=reserved,
            plan_id=pid,
        )

        agent["state"] = "PLANNING"
        agent["pending_result"] = None

        agent["future"] = self.executor.submit(plan_for_agent, snap)
        print(f"[PLAN] Submitted plan for {agent['id']} (plan_id={pid}, pebbles={len(pebbles_xy)})")

    def _poll_planning_results(self):
        # Main thread: apply completed plans
        for agent in self.agents:
            fut = agent.get("future", None)
            if fut is None or not fut.done():
                continue

            try:
                res: PlanResult = fut.result()
            except Exception as e:
                print(f"[PLAN] {agent['id']} future crashed: {e}")
                agent["future"] = None
                agent["state"] = "IDLE"
                continue

            agent["future"] = None

            # stale result?
            if res.plan_id != agent["plan_id"]:
                print(f"[PLAN] Discard stale plan for {agent['id']} (got {res.plan_id}, want {agent['plan_id']})")
                continue

            if res.error is not None:
                print(f"[PLAN] {agent['id']} planning error: {res.error}")
                agent["state"] = "IDLE"
                continue

            if res.best_cell is None or res.best_path_info is None:
                # no valid plan found
                print(f"[PLAN] {agent['id']} found no valid path. Returning to IDLE.")
                agent["state"] = "IDLE"
                continue

            # Apply reservations (main thread only)
            if PLANNING_CONFIG["use_global_reservations"]:
                self.global_reserved_cells.update(res.best_reserved)
                agent["reserved_cells"] = set(res.best_reserved)

            # Optionally store agent-private env_2d
            if PLANNING_CONFIG["store_env_per_agent"]:
                agent["env_2d"] = res.env_2d
                agent["coord_converter"] = CoordinateConverter(res.env_radius, self.target_zone_radius, self.shovel_width)

            # Update UI visualizer with the latest env from whoever planned (optional)
            try:
                self.visualizer.update_env(res.env_2d)
            except Exception:
                pass

            # Setup the navigation path (main thread)
            self._setup_agent_selection(
                agent,
                coord_converter=CoordinateConverter(res.env_radius, self.target_zone_radius, self.shovel_width),
                cell=res.best_cell,
                choice=res.best_choice,
                path_info=res.best_path_info
            )
            agent["state"] = "NAVIGATING"
            self._needs_redraw = True

            print(f"[PLAN] {agent['id']} -> NAVIGATING (cell=({res.best_cell.x},{res.best_cell.y}))")

    # ------------- Auto allocation (IDLE -> PLANNING) -------------
    def _auto_allocate_idle_agents(self):
        # Start planning for any agent that is IDLE and has no running plan
        for agent in self.agents:
            if agent["state"] == "IDLE":
                self._start_planning_for_agent(agent)

    # ------------- Agent stepping -------------
    def _step_agents(self, dt: float):
        # cache states
        for agent in self.agents:
            agent["state_val"] = get_state(agent["body_id"])

        for agent in self.agents:
            if agent["state"] == "IDLE":
                # stop wheels
                set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], 0.0, 0.0)
                continue

            if agent["state"] == "PLANNING":
                # keep still, but physics keeps running for others
                set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], 0.0, 0.0)
                continue

            if agent["state"] == "NAVIGATING":
                other_agents = [a for a in self.agents if a["id"] != agent["id"]]

                # ORCA others format
                orca_others = []
                for oa in other_agents:
                    orca_others.append({
                        "id": oa["id"],
                        "state": oa["state_val"],
                        "shape": {"circles": [(0.0, 0.0, 0.35)]},
                    })

                nav_agent_dict = {
                    "id": agent["id"],
                    "state": agent["state_val"],
                    "priority": 0.0,
                    "shape": {"circles": [(0.0, 0.0, 0.35)]},
                }

                agent["navigator"].set_agent(nav_agent_dict)
                agent["navigator"].update_pebbles(self.pebble_centers)
                status = agent["navigator"].step(dt, orca_others)

                set_wheel_velocities(
                    agent["body_id"], agent["left_joint"], agent["right_joint"],
                    status.v_cmd, status.w_cmd
                )

                if status.completed:
                    print(f"[DONE] {agent['id']} path complete -> ROLLBACK")
                    agent["state"] = "ROLLBACK"
                    agent["rollback_timer"] = 0.5

            elif agent["state"] == "ROLLBACK":
                self._step_rollback(agent, dt)

    def _step_rollback(self, agent: Dict[str, Any], dt: float):
        # Simple timed reverse (same spirit as your existing)
        agent["rollback_timer"] -= dt
        if agent["rollback_timer"] > 0.0:
            set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], -0.3, 0.0)
        else:
            set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], 0.0, 0.0)

            # Release reservations when finished
            if PLANNING_CONFIG["use_global_reservations"]:
                self.global_reserved_cells.difference_update(agent["reserved_cells"])
                agent["reserved_cells"] = set()

            agent["selection"] = None
            agent["state"] = "IDLE"
            self._needs_redraw = True

    # ------------- Path setup (same core as your multi version, but parameterized) -------------
    def _setup_agent_selection(self, agent: Dict[str, Any],
                              coord_converter: CoordinateConverter,
                              cell: Any, choice: str, path_info: Dict[str, Any]):
        traj = path_info["path"]
        print(f"[SETUP] {agent['id']} planned {choice} path: {len(traj)} waypoints, dist={path_info.get('distance', 0.0):.2f}")

        # Visual preview (optional)
        try:
            self.visualizer.set_trajectory_preview(cell, choice, path_info)
        except Exception:
            pass

        # Smooth trajectory in grid coords
        grid_wps = [(c.x, c.y) for c in traj]
        spline_pts, _, success = smooth_path_with_spline(
            grid_wps, SPILLAGE_CONFIG["smoothing_factor"], SPILLAGE_CONFIG["num_points"]
        )
        if not success:
            spline_pts = grid_wps

        # Convert to world
        world_pts = []
        for gx, gy in spline_pts:
            wx, wy = coord_converter.convert_2d_to_3d(gx, gy)
            world_pts.append((wx, wy))

        world_pts = self._resample(world_pts, SPILLAGE_CONFIG["target_spacing"])

        # Gate point behind path start
        S0 = np.array(world_pts[0], dtype=float)
        S1 = np.array(world_pts[min(5, len(world_pts) - 1)], dtype=float)
        v_path = S1 - S0
        v_path = v_path / (np.linalg.norm(v_path) + 1e-9)
        G = S0 - GATE_CONFIG["gate_back"] * v_path

        agent["selection"] = {
            "cell": cell,
            "path_type": choice,
            "path_info": path_info,
            "world_pts": world_pts,
            "gate": (float(G[0]), float(G[1])),
            "path_start": (float(S0[0]), float(S0[1])),
        }

        # Start navigation through NavigationManager
        state_now = get_state(agent["body_id"])
        nav_agent_dict = {
            "id": agent["id"],
            "state": state_now,
            "priority": 0.0,
            "shape": {"circles": [(0.0, 0.0, 0.35)]},
        }
        agent["navigator"].set_agent(nav_agent_dict)

        full_path = [(float(G[0]), float(G[1]))] + world_pts
        agent["navigator"].start_navigation(full_path, self.pebble_centers)

    @staticmethod
    def _resample(points: List[Tuple[float, float]], spacing: float) -> List[Tuple[float, float]]:
        if len(points) < 2:
            return points[:]
        out = [points[0]]
        acc = 0.0
        for i in range(1, len(points)):
            x0, y0 = out[-1]
            x1, y1 = points[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                continue
            dx = (x1 - x0) / seg
            dy = (y1 - y0) / seg
            while acc + seg >= spacing:
                t = (spacing - acc)
                nx = x0 + dx * t
                ny = y0 + dy * t
                out.append((nx, ny))
                seg -= t
                x0, y0 = nx, ny
                acc = 0.0
            acc += seg
        return out

    # ------------- UI / drawing (minimal, reusing your visualizer) -------------
    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

    def _update_visualizer(self):
        # Minimal redraw: rely on your visualizer implementation
        try:
            self.visualizer.screen.fill((255, 255, 255))
            self.visualizer.draw_grid()
            self.visualizer.draw_target_zone()
            self.visualizer.draw_heat_map()
            self.visualizer.draw_objects()
            self.visualizer.draw()
        except Exception:
            pass


def main():
    # Recommend: if your 2D code can open matplotlib windows, keep this False in auto mode.
    SIMULATION_CONFIG["visualize_potential"] = False

    orch = MultiAgentHybridOrchestratorDecoupled(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=50,
        random_seed=41,
        initial_robot_poses=[(2.0, 2.0, -math.pi/2), (-2.0, -2.0, math.pi/2)],
        shovel_width=0.22,
        auto_mode=True,
    )
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()