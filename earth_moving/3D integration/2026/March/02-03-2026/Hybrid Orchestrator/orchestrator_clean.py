"""
orchestrator_clean.py - Clean Multi-Agent Hybrid Orchestrator

Architecture:
  - Cached env_2d for INSTANT path selection (no multi-second planning freeze)
  - Background thread refreshes env_2d ONLY after path execution changes the world
  - One threading.Thread + queue.Queue for env refresh (not per-agent)
  - All PyBullet / pygame / NavigationManager calls on main thread only
  - Clean 4-state machine: IDLE -> PLANNING -> NAVIGATING -> ROLLBACK -> IDLE
  - Single _release_reservations() called on every exit from active states
  - Overlap re-check when consuming planning results
  - set_agent() called EVERY tick (matching working orchestrators exactly)
  - visualize_potential=False always in background planning
  - state_val initialized to None; ORCA others list skips agents with state_val is None
  - Correct visualizer API (draw_grid, draw_agents, etc. — NOT the nonexistent draw())

Fixes over orchestrator_hybrid_multi.py:
  1. No race condition on shared env_2d (atomic swap from background thread)
  2. state_val always initialized (no KeyError in ORCA formatting)
  3. No WAITING_FOR_MAP deadlock (clean IDLE->PLANNING->NAVIGATING->ROLLBACK)
  4. visualize_potential=False always (no matplotlib blocking)
  5. Correct visualizer API calls

Fixes over orchestrator_hybrid_multi_decoupled.py:
  1. Reserved cells always cleared on error/failure paths
  2. Correct visualizer API (draw_elements components, not draw())
  3. No complex plan_id versioning — simple cached env + background refresh
"""

import os
import sys
import time
import math
import enum
import queue
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import pygame
import pybullet as p
import pybullet_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import run_2d_env, compute_2d_env
from coordinate_converter import CoordinateConverter
from spillage_model import smooth_path_with_spline
from navigation_manager import NavigationManager


# ────────────────────────── Configuration ──────────────────────────

@dataclass(frozen=True)
class SimConfig:
    """Immutable simulation hyperparameters."""
    env_radius: float = 3.0
    target_zone_radius: float = 0.8
    num_pebbles: int = 50
    random_seed: int = 41
    shovel_width: float = 0.22

    # Robot spawn poses: list of (x, y, yaw)
    initial_robot_poses: Tuple[Tuple[float, float, float], ...] = (
        (2.0, 2.0, -math.pi / 2),
        (-2.0, -2.0, math.pi / 2),
    )

    # Spillage / path smoothing
    smoothing_factor: float = 0.5
    num_spline_points: int = 1000
    target_spacing: float = 0.03

    # Gate approach
    gate_back: float = 0.50

    # Rollback
    rollback_duration: float = 0.5
    rollback_speed: float = -0.3

    # Physics
    physics_hz: float = 240.0
    control_hz: float = 20.0
    display_hz: float = 15.0

    # ORCA / rover geometry
    rover_radius: float = 0.25
    orca_shape_radius: float = 0.35
    pebble_radius: float = 0.05
    nav_grid_cell_size: float = 0.15

    # Planning
    use_spillage_model: bool = True


# ────────────────────────── State Machine ──────────────────────────

class AgentState(enum.Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"       # only used when cached env has no valid path
    NAVIGATING = "NAVIGATING"
    ROLLBACK = "ROLLBACK"


# ────────────────────────── Data Contracts ──────────────────────────

@dataclass(frozen=True)
class PlanningSnapshot:
    """Immutable snapshot passed to background planning thread."""
    env_radius: float
    target_zone_radius: float
    shovel_width: float
    use_spillage_model: bool
    pebbles_3d: Tuple[Tuple[float, float, float], ...]
    agent_pose_xy: Tuple[float, float]
    reserved_cells_snapshot: frozenset  # frozenset of (int, int)


@dataclass
class PlanResult:
    """Result from path selection (fast or background)."""
    best_cell: Any = None
    best_choice: str = "target"
    best_path_info: Optional[Dict[str, Any]] = None
    best_reserved: Set[Tuple[int, int]] = field(default_factory=set)
    env_2d: Any = None
    coord_converter: Optional[CoordinateConverter] = None
    error: Optional[str] = None


@dataclass
class AgentRecord:
    """Per-agent mutable state, only mutated on main thread."""
    agent_id: str
    index: int
    body_id: int
    left_joint: int
    right_joint: int
    color: Tuple[float, float, float, float]
    navigator: NavigationManager
    state: AgentState = AgentState.IDLE
    state_val: Optional[np.ndarray] = None  # [x, y, yaw, v_fwd, w]
    selection: Optional[Dict[str, Any]] = None
    reserved_cells: Set[Tuple[int, int]] = field(default_factory=set)
    rollback_timer: float = 0.0

    # Background planning thread + result queue (fallback only)
    _thread: Optional[threading.Thread] = field(default=None, repr=False)
    _result_queue: queue.Queue = field(default_factory=queue.Queue, repr=False)


# ────────────────────────── Helper Functions ──────────────────────────

def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def get_state(body_id: int) -> np.ndarray:
    """Return [x, y, yaw, v_forward, w] from PyBullet."""
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


# ────────────────────────── Path Selection (no PyBullet) ──────────────────────────

def _pick_best_path(env_2d, coord_converter: CoordinateConverter,
                    rover_xy: Tuple[float, float],
                    reserved: Set[Tuple[int, int]]):
    """
    Find best non-overlapping target path from env_2d.
    Score = objects - 0.5 * distance(rover -> path start).
    Returns (best_cell, "target", best_path_info, best_reserved) or all-None.
    """
    best_cell = None
    best_score = -float("inf")
    best_path_info = None
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

        if reserved and required_cells & reserved:
            continue

        start_c = traj[0]
        start_wx, start_wy = coord_converter.convert_2d_to_3d(start_c.x, start_c.y)
        dist = math.hypot(rover_xy[0] - start_wx, rover_xy[1] - start_wy)

        score = objects - 0.5 * dist
        if score > best_score:
            best_score = score
            best_cell = cell
            best_path_info = path_info
            best_reserved = required_cells

    return best_cell, "target", best_path_info, best_reserved


# ────────────────────────── Background Env Refresh Worker ──────────────────────────

def _env_refresh_worker(cfg: SimConfig, pebbles_3d: list, result_queue: queue.Queue):
    """
    Background thread: recompute env_2d from current pebble positions.
    MUST NOT call PyBullet or pygame.
    """
    try:
        env_2d = compute_2d_env(
            cfg.env_radius,
            cfg.target_zone_radius,
            cfg.shovel_width,
            pebbles_3d,
            manual_mode=False,
            use_spillage_model=cfg.use_spillage_model,
            visualize_potential=False,  # NEVER show matplotlib in background
        )
        cc = CoordinateConverter(cfg.env_radius, cfg.target_zone_radius, cfg.shovel_width)
        result_queue.put((env_2d, cc))
    except Exception as e:
        print(f"[REFRESH] Background env refresh failed: {e}")
        result_queue.put(None)


def _plan_worker(snapshot: PlanningSnapshot, result_queue: queue.Queue):
    """
    Background thread: compute fresh env_2d + find best path.
    Fallback for when cached env has no valid paths.
    MUST NOT call PyBullet or pygame.
    """
    try:
        pebbles_3d_list = list(snapshot.pebbles_3d)

        env_2d = compute_2d_env(
            snapshot.env_radius,
            snapshot.target_zone_radius,
            snapshot.shovel_width,
            pebbles_3d_list,
            manual_mode=False,
            use_spillage_model=snapshot.use_spillage_model,
            visualize_potential=False,
        )

        coord_converter = CoordinateConverter(
            snapshot.env_radius,
            snapshot.target_zone_radius,
            snapshot.shovel_width,
        )

        reserved = set(snapshot.reserved_cells_snapshot)
        best_cell, best_choice, best_path_info, best_reserved = _pick_best_path(
            env_2d, coord_converter, snapshot.agent_pose_xy, reserved
        )

        result_queue.put(PlanResult(
            best_cell=best_cell,
            best_choice=best_choice,
            best_path_info=best_path_info,
            best_reserved=best_reserved,
            env_2d=env_2d,
            coord_converter=coord_converter,
        ))

    except Exception as e:
        result_queue.put(PlanResult(error=str(e)))


# ────────────────────────── Orchestrator ──────────────────────────

class CleanMultiAgentOrchestrator:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg

        # Global coordination
        self._reserved_cells: Set[Tuple[int, int]] = set()

        # Agent records (populated in initialize)
        self.agents: List[AgentRecord] = []

        # PyBullet pebble tracking
        self.pebble_ids: List[int] = []
        self.pebble_centers: List[Tuple[float, float]] = []

        # Visualizer (created in initialize via run_2d_env)
        self.visualizer = None
        self.coord_converter = CoordinateConverter(
            cfg.env_radius, cfg.target_zone_radius, cfg.shovel_width
        )

        # Cached env_2d for INSTANT path selection (the key to smooth movement)
        self._env_2d = None
        self._env_cc = None  # CoordinateConverter matching _env_2d

        # Background env refresh (runs after rollback to update _env_2d)
        self._refresh_thread: Optional[threading.Thread] = None
        self._refresh_queue: queue.Queue = queue.Queue()

        self.running = False

    # ─────────── Initialization ───────────

    def initialize(self):
        print("=" * 60)
        print("Initializing Clean Multi-Agent Orchestrator")
        print("=" * 60)
        cfg = self.cfg

        # PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(5.0, 45, -60, [0, 0, 0])
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / cfg.physics_hz)

        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Target zone visual
        tz_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=cfg.target_zone_radius,
            length=0.01,
            rgbaColor=[1, 0, 0, 0.3],
        )
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=tz_vis,
                          basePosition=[0, 0, 0.005])

        # Rovers
        here = os.path.dirname(os.path.abspath(__file__))
        rover_urdf = os.path.join(here, "2_wheel_rover.urdf")

        agent_colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
            (0.1, 1.0, 0.5, 1.0),
            (1.0, 0.1, 0.5, 1.0),
            (0.5, 0.1, 1.0, 1.0),
        ]

        grid_dim = int(math.ceil(2 * cfg.env_radius / cfg.nav_grid_cell_size))
        if grid_dim % 2 == 0:
            grid_dim += 1

        for i, (x, y, yaw) in enumerate(cfg.initial_robot_poses):
            body_id = p.loadURDF(rover_urdf, [x, y, 0.02], yaw_to_quat(yaw))
            left_j, right_j = find_wheel_joints(body_id)

            p.changeDynamics(body_id, -1, lateralFriction=0.8)
            for j in (left_j, right_j):
                p.changeDynamics(body_id, j, lateralFriction=1.0,
                                 rollingFriction=0.0, spinningFriction=0.0)
                p.setJointMotorControl2(body_id, j, p.VELOCITY_CONTROL,
                                        targetVelocity=0.0, force=0.0)

            color = agent_colors[i % len(agent_colors)]
            p.changeVisualShape(body_id, -1, rgbaColor=color)

            nav = NavigationManager(
                world_bounds=(-cfg.env_radius, cfg.env_radius,
                              -cfg.env_radius, cfg.env_radius),
                grid_size=(grid_dim, grid_dim),
                rover_radius=cfg.rover_radius,
                pebble_radius=cfg.pebble_radius,
            )

            self.agents.append(AgentRecord(
                agent_id=f"R{i}",
                index=i,
                body_id=body_id,
                left_joint=left_j,
                right_joint=right_j,
                color=color,
                navigator=nav,
            ))
            print(f"  Rover {i} at ({x:.2f}, {y:.2f})")

        # Pebbles
        pebble_urdf = os.path.join(here, "pebbles.urdf")
        np.random.seed(cfg.random_seed)

        for _ in range(cfg.num_pebbles):
            r_min = cfg.target_zone_radius
            r_max = cfg.env_radius
            r = math.sqrt(np.random.rand() * (r_max ** 2 - r_min ** 2) + r_min ** 2)
            phi = 2 * math.pi * np.random.rand()
            px, py = r * math.cos(phi), r * math.sin(phi)
            bid = p.loadURDF(pebble_urdf, [px, py, 0.01], useFixedBase=False)
            p.changeDynamics(bid, -1, lateralFriction=0.8)
            self.pebble_ids.append(bid)
            self.pebble_centers.append((px, py))

        print(f"  {len(self.pebble_centers)} pebbles spawned")

        # Initial 2D env + visualizer (main thread, once)
        pebbles_3d = [(x, y, 0.01) for x, y in self.pebble_centers]
        env_2d, self.visualizer = run_2d_env(
            cfg.env_radius, cfg.target_zone_radius, cfg.shovel_width,
            pebbles_3d,
            manual_mode=False,
            use_spillage_model=cfg.use_spillage_model,
            visualize_potential=False,
        )
        self.visualizer.update_env(env_2d)

        # Cache env_2d for instant path selection
        self._env_2d = env_2d
        self._env_cc = CoordinateConverter(cfg.env_radius, cfg.target_zone_radius, cfg.shovel_width)

        print("\n[READY] Clean orchestrator initialized")
        print("=" * 60)

    # ─────────── Main Loop ───────────

    def run(self):
        self.running = True
        cfg = self.cfg
        sim_dt = 1.0 / cfg.physics_hz
        control_period = 1.0 / cfg.control_hz
        display_period = 1.0 / cfg.display_hz

        last_time = time.time()
        physics_acc = 0.0
        control_acc = 0.0
        display_acc = 0.0

        print("\nStarting main loop...")

        try:
            while self.running and p.isConnected():
                now = time.time()
                dt_real = min(now - last_time, 0.1)
                last_time = now

                physics_acc += dt_real
                control_acc += dt_real
                display_acc += dt_real

                # (1) Physics — 240 Hz, never blocked
                while physics_acc >= sim_dt:
                    p.stepSimulation()
                    physics_acc -= sim_dt

                # (2) Control — 20 Hz
                if control_acc >= control_period:
                    self._control_tick(control_period)
                    control_acc %= control_period

                # (3) Display / events — 15 Hz
                if display_acc >= display_period:
                    self._handle_pygame_events()
                    self._apply_pending_env_refresh()
                    self._dispatch_idle_agents()
                    self._consume_planning_results()
                    self._redraw()
                    display_acc %= display_period

                time.sleep(0.001)
        finally:
            self._shutdown()

    def _shutdown(self):
        self.running = False
        # Join all planning threads
        for ag in self.agents:
            t = ag._thread
            if t is not None and t.is_alive():
                t.join(timeout=5.0)
        if self._refresh_thread is not None and self._refresh_thread.is_alive():
            self._refresh_thread.join(timeout=5.0)
        try:
            pygame.quit()
        except Exception:
            pass
        try:
            if p.isConnected():
                p.disconnect()
        except Exception:
            pass

    # ─────────── Pebble Snapshots (main thread) ───────────

    def _refresh_pebble_centers(self):
        """Update self.pebble_centers from PyBullet. Main thread only."""
        pts = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            if pos[2] > -0.5:
                pts.append((float(pos[0]), float(pos[1])))
        self.pebble_centers = pts

    def _snapshot_pebbles_3d(self) -> List[Tuple[float, float, float]]:
        """Query PyBullet for current pebble positions (x, y, z). Also updates pebble_centers."""
        pts_3d = []
        pts_2d = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            if pos[2] > -0.5:
                pts_3d.append((float(pos[0]), float(pos[1]), float(pos[2])))
                pts_2d.append((float(pos[0]), float(pos[1])))
        self.pebble_centers = pts_2d
        return pts_3d

    # ─────────── Reservation Helpers ───────────

    def _reserve_cells(self, ag: AgentRecord, cells: Set[Tuple[int, int]]):
        ag.reserved_cells = set(cells)
        self._reserved_cells.update(cells)

    def _release_reservations(self, ag: AgentRecord):
        """Idempotent: remove agent's reserved cells from global set."""
        if ag.reserved_cells:
            self._reserved_cells.difference_update(ag.reserved_cells)
            ag.reserved_cells = set()

    # ─────────── Background Env Refresh ───────────

    def _trigger_env_refresh(self):
        """Start background recomputation of env_2d after a path execution."""
        if self._refresh_thread is not None and self._refresh_thread.is_alive():
            return  # already refreshing

        pebbles_3d = self._snapshot_pebbles_3d()
        # Drain any stale results
        while not self._refresh_queue.empty():
            try:
                self._refresh_queue.get_nowait()
            except queue.Empty:
                break

        self._refresh_thread = threading.Thread(
            target=_env_refresh_worker,
            args=(self.cfg, pebbles_3d, self._refresh_queue),
            daemon=True,
        )
        self._refresh_thread.start()
        print("[REFRESH] Background env recomputation started")

    def _apply_pending_env_refresh(self):
        """Check for completed env refresh and swap atomically. Main thread only."""
        try:
            result = self._refresh_queue.get_nowait()
        except queue.Empty:
            return

        if result is None:
            print("[REFRESH] Background env refresh returned None (error)")
            return

        env_2d, cc = result
        self._env_2d = env_2d
        self._env_cc = cc
        try:
            self.visualizer.update_env(env_2d)
        except Exception:
            pass
        print("[REFRESH] Cached env_2d updated from background")

    # ─────────── Fast Path Selection (main thread, instant) ───────────

    def _find_path_fast(self, ag: AgentRecord) -> Optional[PlanResult]:
        """
        Find best path using cached _env_2d. Runs on main thread, instant.
        Returns PlanResult or None if no valid path found.
        """
        if self._env_2d is None or self._env_cc is None:
            return None

        st = get_state(ag.body_id)
        rover_xy = (float(st[0]), float(st[1]))

        best_cell, best_choice, best_path_info, best_reserved = _pick_best_path(
            self._env_2d, self._env_cc, rover_xy, self._reserved_cells
        )

        if best_cell is None or best_path_info is None:
            return None

        return PlanResult(
            best_cell=best_cell,
            best_choice=best_choice,
            best_path_info=best_path_info,
            best_reserved=best_reserved,
            env_2d=self._env_2d,
            coord_converter=self._env_cc,
        )

    # ─────────── IDLE -> NAVIGATING (fast) or PLANNING (fallback) ───────────

    def _dispatch_idle_agents(self):
        """
        Try INSTANT path selection from cached env_2d first.
        Only fall back to slow background planning if cache has no valid paths.
        """
        cfg = self.cfg
        for ag in self.agents:
            if ag.state != AgentState.IDLE:
                continue

            # --- Fast path: instant selection from cached env_2d ---
            result = self._find_path_fast(ag)
            if result is not None:
                # Overlap re-check
                if result.best_reserved & self._reserved_cells:
                    continue  # another agent just reserved these cells, try next tick

                # Reserve and setup navigation immediately
                self._reserve_cells(ag, result.best_reserved)
                ok = self._setup_navigation(ag, result)
                if ok:
                    ag.state = AgentState.NAVIGATING
                    print(f"[FAST] {ag.agent_id} -> NAVIGATING "
                          f"(cell=({result.best_cell.x},{result.best_cell.y}))")
                else:
                    self._release_reservations(ag)
                    print(f"[FAST] {ag.agent_id} navigation setup failed")
                continue

            # --- Slow path: background planning with fresh env_2d ---
            # Only if no thread is running for this agent
            if ag._thread is not None and ag._thread.is_alive():
                continue

            pebbles_3d = self._snapshot_pebbles_3d()
            st = get_state(ag.body_id)

            snapshot = PlanningSnapshot(
                env_radius=cfg.env_radius,
                target_zone_radius=cfg.target_zone_radius,
                shovel_width=cfg.shovel_width,
                use_spillage_model=cfg.use_spillage_model,
                pebbles_3d=tuple(pebbles_3d),
                agent_pose_xy=(float(st[0]), float(st[1])),
                reserved_cells_snapshot=frozenset(self._reserved_cells),
            )

            # Clear stale results
            while not ag._result_queue.empty():
                try:
                    ag._result_queue.get_nowait()
                except queue.Empty:
                    break

            ag.state = AgentState.PLANNING
            ag._thread = threading.Thread(
                target=_plan_worker,
                args=(snapshot, ag._result_queue),
                daemon=True,
            )
            ag._thread.start()
            print(f"[PLAN] {ag.agent_id} -> PLANNING (no path in cache, recomputing env)")

    # ─────────── PLANNING -> NAVIGATING or IDLE ───────────

    def _consume_planning_results(self):
        for ag in self.agents:
            if ag.state != AgentState.PLANNING:
                continue

            try:
                result: PlanResult = ag._result_queue.get_nowait()
            except queue.Empty:
                continue

            ag._thread = None

            if result.error is not None:
                print(f"[PLAN] {ag.agent_id} error: {result.error}")
                ag.state = AgentState.IDLE
                continue

            if result.best_cell is None or result.best_path_info is None:
                print(f"[PLAN] {ag.agent_id} found no valid path -> IDLE")
                ag.state = AgentState.IDLE
                continue

            # Overlap re-check
            if result.best_reserved & self._reserved_cells:
                print(f"[PLAN] {ag.agent_id} overlap detected -> IDLE")
                ag.state = AgentState.IDLE
                continue

            # Update cached env from the fresh computation
            self._env_2d = result.env_2d
            self._env_cc = result.coord_converter
            try:
                self.visualizer.update_env(result.env_2d)
            except Exception:
                pass

            # Reserve and setup navigation
            self._reserve_cells(ag, result.best_reserved)
            ok = self._setup_navigation(ag, result)
            if ok:
                ag.state = AgentState.NAVIGATING
                print(f"[PLAN] {ag.agent_id} -> NAVIGATING "
                      f"(cell=({result.best_cell.x},{result.best_cell.y}))")
            else:
                self._release_reservations(ag)
                ag.state = AgentState.IDLE
                print(f"[PLAN] {ag.agent_id} navigation setup failed -> IDLE")

    # ─────────── Navigation Setup ───────────

    def _setup_navigation(self, ag: AgentRecord, result: PlanResult) -> bool:
        """Smooth path, compute gate, start navigation. Returns True on success."""
        cfg = self.cfg
        path_info = result.best_path_info
        traj = path_info["path"]
        cc = result.coord_converter

        print(f"[SETUP] {ag.agent_id} target path: "
              f"{len(traj)} waypoints, dist={path_info.get('distance', 0.0):.2f}")

        # Optional: show preview on visualizer
        try:
            self.visualizer.set_trajectory_preview(
                result.best_cell, result.best_choice, path_info
            )
        except Exception:
            pass

        # Smooth trajectory in grid coordinates
        grid_wps = [(c.x, c.y) for c in traj]
        spline_pts, _, success = smooth_path_with_spline(
            grid_wps, cfg.smoothing_factor, cfg.num_spline_points
        )
        if not success:
            spline_pts = grid_wps

        # Convert to world coordinates
        world_pts = [cc.convert_2d_to_3d(gx, gy) for gx, gy in spline_pts]
        world_pts = self._resample(world_pts, cfg.target_spacing)

        if len(world_pts) < 2:
            return False

        # Compute gate point behind path start
        S0 = np.array(world_pts[0], dtype=float)
        S1 = np.array(world_pts[min(5, len(world_pts) - 1)], dtype=float)
        v_path = S1 - S0
        v_path = v_path / (np.linalg.norm(v_path) + 1e-9)
        G = S0 - cfg.gate_back * v_path

        ag.selection = {
            "cell": result.best_cell,
            "path_type": result.best_choice,
            "path_info": path_info,
            "world_pts": world_pts,
            "gate": (float(G[0]), float(G[1])),
            "path_start": (float(S0[0]), float(S0[1])),
        }

        # Build nav agent dict and start navigation
        state_now = get_state(ag.body_id)
        nav_agent_dict = {
            "id": ag.agent_id,
            "state": state_now,
            "priority": 0.0,
            "shape": {"circles": [(0.0, 0.0, cfg.orca_shape_radius)]},
        }
        ag.navigator.set_agent(nav_agent_dict)

        full_path = [(float(G[0]), float(G[1]))] + list(world_pts)
        return ag.navigator.start_navigation(full_path, self.pebble_centers)

    # ─────────── Control Tick (20 Hz) ───────────

    def _control_tick(self, dt: float):
        # Refresh pebble positions every control tick
        self._refresh_pebble_centers()

        # Update state_val for ALL agents unconditionally
        for ag in self.agents:
            ag.state_val = get_state(ag.body_id)

        for ag in self.agents:
            if ag.state == AgentState.IDLE or ag.state == AgentState.PLANNING:
                set_wheel_velocities(ag.body_id, ag.left_joint, ag.right_joint,
                                     0.0, 0.0)
            elif ag.state == AgentState.NAVIGATING:
                self._step_navigating(ag, dt)
            elif ag.state == AgentState.ROLLBACK:
                self._step_rollback(ag, dt)

    def _step_navigating(self, ag: AgentRecord, dt: float):
        cfg = self.cfg

        # Build ORCA others list — skip agents with state_val is None
        orca_others = []
        for other in self.agents:
            if other.agent_id == ag.agent_id:
                continue
            if other.state_val is None:
                continue
            orca_others.append({
                "id": other.agent_id,
                "state": other.state_val,
                "shape": {"circles": [(0.0, 0.0, cfg.orca_shape_radius)]},
            })

        # Build fresh agent dict every tick (matches working orchestrators exactly)
        nav_agent_dict = {
            "id": ag.agent_id,
            "state": ag.state_val,
            "priority": 0.0,
            "shape": {"circles": [(0.0, 0.0, cfg.orca_shape_radius)]},
        }
        ag.navigator.set_agent(nav_agent_dict)
        ag.navigator.update_pebbles(self.pebble_centers)

        status = ag.navigator.step(dt, orca_others)

        set_wheel_velocities(ag.body_id, ag.left_joint, ag.right_joint,
                             status.v_cmd, status.w_cmd)

        if status.completed:
            print(f"[DONE] {ag.agent_id} path complete -> ROLLBACK")
            ag.state = AgentState.ROLLBACK
            ag.rollback_timer = cfg.rollback_duration

    def _step_rollback(self, ag: AgentRecord, dt: float):
        cfg = self.cfg
        ag.rollback_timer -= dt
        if ag.rollback_timer > 0.0:
            set_wheel_velocities(ag.body_id, ag.left_joint, ag.right_joint,
                                 cfg.rollback_speed, 0.0)
        else:
            set_wheel_velocities(ag.body_id, ag.left_joint, ag.right_joint,
                                 0.0, 0.0)
            self._release_reservations(ag)
            ag.selection = None
            ag.state = AgentState.IDLE
            print(f"[DONE] {ag.agent_id} -> IDLE")

            # Trigger background env refresh so next dispatch uses fresh data
            self._trigger_env_refresh()

    # ─────────── Pygame Events ───────────

    def _handle_pygame_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

    # ─────────── Redraw (15 Hz) ───────────

    def _redraw(self):
        """Correct visualizer API: individual draw_* calls, then flip."""

        # Build agent visualization data
        class _DummyAgent:
            def __init__(self, pos):
                self.position = pos

        vis_agents = []
        vis_paths = {}

        for ag in self.agents:
            if ag.state_val is not None:
                x, y = ag.state_val[0], ag.state_val[1]
            else:
                st = get_state(ag.body_id)
                x, y = st[0], st[1]
            cx, cy = self.coord_converter.convert_3d_to_2d(x, y)
            vis_agents.append(_DummyAgent((cx, cy)))

            if ag.selection:
                path_cells = ag.selection["path_info"]["path"]
                vis_paths[ag.index] = [(c.x, c.y) for c in path_cells]

        self.visualizer.update_agents(vis_agents)
        self.visualizer.update_agent_paths(vis_paths)

        # Draw
        self.visualizer.screen.fill((255, 255, 255))
        self.visualizer.draw_grid()
        self.visualizer.draw_target_zone()
        self.visualizer.draw_heat_map()
        self.visualizer.draw_objects()
        self.visualizer.draw_agents()
        self.visualizer.draw_agent_paths()
        pygame.display.flip()

    # ─────────── Path Resampling ───────────

    @staticmethod
    def _resample(points: List[Tuple[float, float]],
                  spacing: float) -> List[Tuple[float, float]]:
        if len(points) < 2:
            return list(points)
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
                t = spacing - acc
                nx = x0 + dx * t
                ny = y0 + dy * t
                out.append((nx, ny))
                seg -= t
                x0, y0 = nx, ny
                acc = 0.0
            acc += seg
        return out


# ────────────────────────── Entry Point ──────────────────────────

def main():
    cfg = SimConfig()
    orch = CleanMultiAgentOrchestrator(cfg)
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()
