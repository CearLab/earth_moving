"""
orchestrator_hybrid.py - Main Hybrid Orchestrator

Two-phase trajectory execution:
1. Flow field navigation to approach point (gate) behind path start
2. Path tracking along the smooth trajectory

All execution logic is INLINE to avoid cross-file overhead.
"""

import os
import sys
import time
import math
import heapq
import numpy as np
import pygame
import pybullet as p
import pybullet_data
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List, Set
import concurrent.futures as cf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import run_2d_env, compute_2d_env
from coordinate_converter import CoordinateConverter
from spillage_model import smooth_path_with_spline
from navigation_manager import NavigationManager, NavigationMode

# ===================== CONFIG =====================
SIMULATION_CONFIG = {
    'use_spillage_model': True,
    'visualize_potential': True,
    'show_grid': True,
}

SPILLAGE_CONFIG = {
    'smoothing_factor': 0.5,
    'num_points': 1000,
    'target_spacing': 0.03,
}

# Gate approach config
GATE_CONFIG = {
    'gate_back': 0.50,  # distance behind path start
    'approach_tol': 0.15,  # arrival tolerance
}

# ===================== INLINE HELPER FUNCTIONS =====================

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def get_state(body_id):
    """Get robot state: [x, y, yaw, v_forward, w]"""
    pos, orn = p.getBasePositionAndOrientation(body_id)
    x, y = pos[0], pos[1]
    yaw = p.getEulerFromQuaternion(orn)[2]
    lin_vel, ang_vel = p.getBaseVelocity(body_id)
    vx, vy = lin_vel[0], lin_vel[1]
    v_fwd = math.cos(yaw) * vx + math.sin(yaw) * vy
    w = ang_vel[2]
    return np.array([x, y, yaw, v_fwd, w])


def set_wheel_velocities(body_id, left_joint, right_joint, v_cmd, w_cmd,
                         wheel_radius=0.07, track_width=0.20,
                         max_wheel_speed=20.0, max_torque=5.0):
    """Set differential drive wheel velocities."""
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
    """Find wheel joint indices."""
    left = right = None
    for j in range(p.getNumJoints(body_id)):
        name = p.getJointInfo(body_id, j)[1].decode("utf-8")
        if name == "base_to_lwheel":
            left = j
        elif name == "base_to_rwheel":
            right = j
    return left, right

@dataclass(frozen=True)
class WorldSnapshot:
    env_radius: float
    target_zone_radius: float
    shovel_width: float
    agent_id: str
    agent_pose_xy: Tuple[float, float]
    pebbles_xy: np.ndarray
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

def _pick_best_path_for_agent(env_2d, coord_converter, rover_pos_xy, reserved_global):
    best_cell = None
    best_score = -float("inf")
    best_path_info = None
    best_choice = "target"
    best_reserved = set()

    for cell in getattr(env_2d, "cells_with_objects", []):
        paths = env_2d.get_path_for_preview(cell, "target")
        if not paths: continue
        path_info = paths[0]
        objects = path_info.get("objects", 0)
        if objects <= 0: continue

        traj = path_info.get("path", [])
        if not traj: continue

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

# ===================== MAIN ORCHESTRATOR =====================

class MultiAgentHybridOrchestrator:
    def __init__(self, env_radius=3.0, target_zone_radius=0.3, num_pebbles=50,
                 random_seed=41, initial_robot_poses=None,
                 shovel_width=0.22, auto_mode=False):
        self.env_radius = env_radius
        self.target_zone_radius = target_zone_radius
        self.num_pebbles = num_pebbles
        self.random_seed = random_seed
        self.shovel_width = shovel_width
        self.auto_mode = auto_mode
        self.initial_robot_poses = initial_robot_poses or [(2.0, 2.0, -math.pi/2)]
        self.num_rovers = len(self.initial_robot_poses)

        # Visualization Config
        # "ask", "always", "never"
        self.flow_field_vis_mode = "never" 

        # PyBullet state
        self.agents = []
        self.pebble_centers = []
        self.draw_flow_field = False
        self.global_reserved_cells = set() # Global set of reserved cells

        # Components
        self.coord_converter = None
        self.env_2d = None
        self.visualizer = None

        self.running = False
        self.executor = None
        self.executing_agents = {} # Tracks active agents executing trajectories

    def initialize(self):
        print("=" * 60)
        print("Initializing Multi-Agent Hybrid Orchestrator")
        print("=" * 60)

        # PyBullet setup
        print("\n[1/3] Setting up PyBullet...")
        self.executor = cf.ThreadPoolExecutor(max_workers=self.num_rovers)
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(5.0, 45, -60, [0, 0, 0])
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)

        # Ground plane
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Target zone
        tz_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=self.target_zone_radius,
                                     length=0.01, rgbaColor=[1, 0, 0, 0.3])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=tz_vis, basePosition=[0, 0, 0.005])

        # Rovers
        here = os.path.dirname(os.path.abspath(__file__))
        rover_urdf = os.path.join(here, "2_wheel_rover.urdf")
        
        agent_colors = [
            (0.1, 0.5, 1.0, 1.0),  # Blue
            (1.0, 0.5, 0.1, 1.0),  # Orange
            (0.1, 1.0, 0.5, 1.0),  # Green
            (1.0, 0.1, 0.5, 1.0),  # Pink
            (0.5, 0.1, 1.0, 1.0),  # Purple
        ]
        
        for i, (x, y, yaw) in enumerate(self.initial_robot_poses):
            body_id = p.loadURDF(rover_urdf, [x, y, 0.02], yaw_to_quat(yaw))
            left_j, right_j = find_wheel_joints(body_id)
            
            p.changeDynamics(body_id, -1, lateralFriction=0.8)
            for j in (left_j, right_j):
                p.changeDynamics(body_id, j, lateralFriction=1.0, rollingFriction=0.0, spinningFriction=0.0)
                p.setJointMotorControl2(body_id, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
            
            color = agent_colors[i % len(agent_colors)]
            p.changeVisualShape(body_id, -1, rgbaColor=color)
            
            grid_dim = int(math.ceil(2 * self.env_radius / 0.15))
            if grid_dim % 2 == 0: grid_dim += 1
            nav = NavigationManager(
                world_bounds=(-self.env_radius, self.env_radius, -self.env_radius, self.env_radius),
                grid_size=(grid_dim, grid_dim),
                rover_radius=0.25,
                pebble_radius=0.05
            )

            self.agents.append({
                "id": f"R{i}",
                "index": i,
                "body_id": body_id,
                "left_joint": left_j,
                "right_joint": right_j,
                "color": color,
                "state": "IDLE", # IDLE, PLANNING, NAVIGATING, ROLLBACK, SYNC
                "reserved_cells": set(),
                "navigator": nav,
                "plan_id": 0,
                "future": None,
                "env_2d": None,
                "coord_converter": CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width),
            })
            print(f"  Rover {i} at ({x:.2f}, {y:.2f})")

        # Pebbles
        pebble_urdf = os.path.join(here, "pebbles.urdf")
        np.random.seed(self.random_seed)
        self.pebble_ids = []  # Store IDs to query positions later
        for _ in range(self.num_pebbles):
            r_min = self.target_zone_radius
            r_max = self.env_radius
            r = math.sqrt(np.random.rand() * (r_max**2 - r_min**2) + r_min**2)
            phi = 2 * math.pi * np.random.rand()
            px, py = r * math.cos(phi), r * math.sin(phi)
            bid = p.loadURDF(pebble_urdf, [px, py, 0.01], useFixedBase=False)
            p.changeDynamics(bid, -1, lateralFriction=0.8)
            self.pebble_centers.append((px, py))
            self.pebble_ids.append(bid)

        print(f"  {len(self.pebble_centers)} pebbles spawned")

        # Coordinate converter
        print("\n[2/3] Setting up 2D environment...")
        self.coord_converter = CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width)

        # 2D environment
        pebbles_3d = [(x, y, 0.01) for x, y in self.pebble_centers]
        self.env_2d, self.visualizer = run_2d_env(
            self.env_radius, self.target_zone_radius, self.shovel_width,
            pebbles_3d, manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
            visualize_potential=SIMULATION_CONFIG['visualize_potential'])
        
        print("\n[3/3] Ready!")
        print("=" * 60)
        print("Controls: Click=Select (Agent 0), ENTER=Execute, ESC=Cancel/Quit")
        print("=" * 60 + "\n")

    def run(self):
        self.running = True
        sim_dt = 1 / 240.0
        
        last_time = time.time()
        physics_acc = 0.0
        display_acc = 0.0
        control_acc = 0.0

        print("\nStarting main execution loop...")

        while self.running and p.isConnected():
            now = time.time()
            # Cap dt_real to avoid spiral of death if PyGame freezes for a moment
            dt_real = min(now - last_time, 0.1) 
            last_time = now
            
            physics_acc += dt_real
            control_acc += dt_real
            display_acc += dt_real
            
            # 1. Physics Loop (240Hz)
            while physics_acc >= sim_dt:
                p.stepSimulation()
                physics_acc -= sim_dt
            
            # 2. Control Loop (20Hz)
            if control_acc >= 0.05: 
                self._step_agents(0.05)
                self._poll_planning_results()
                # Keep remainder rather than clearing to 0.0 to prevent drifting
                control_acc = control_acc % 0.05
                
            # 3. Display / Auto-Mode Loop (15Hz is enough for UI and fast enough for planning)
            if display_acc >= 1/15.0:
                self._handle_events()
                
                if self.auto_mode:
                    self._handle_auto_mode()
                
                if getattr(self, '_needs_redraw', True):
                    self._needs_redraw = False
                    self._update_visualizer()
                    
                    self.visualizer.screen.fill((255, 255, 255))
                    self.visualizer.draw_grid()
                    self.visualizer.draw_target_zone()
                    self.visualizer.draw_heat_map()
                    self.visualizer.draw_objects()
                    self.visualizer.draw_agent_paths()
                    pygame.display.flip()
                
                display_acc = display_acc % (1/15.0)
                
            time.sleep(0.001)

        pygame.quit()
        if self.executor:
            self.executor.shutdown(wait=False, cancel_futures=True)
        p.disconnect()

    def _update_visualizer(self):
        """Passes 3D multi-agent state down to PyGame for dual path rendering"""
        class DummyAgentVis:
            def __init__(self, pos):
                self.position = pos
                
        vis_agents = []
        vis_paths = {}
        
        for i, agent in enumerate(self.agents):
            x, y, yaw, _, _ = get_state(agent["body_id"])
            
            coord_converter = agent.get('coord_converter', self.coord_converter)
            cx, cy = coord_converter.convert_3d_to_2d(x, y)
            
            vis_agents.append(DummyAgentVis((cx, cy)))
            
            # Extract planned paths if any
            if agent.get('selection'):
                path_info = agent['selection']['path_info']['path']
                vis_paths[i] = [(c.x, c.y) for c in path_info]
                
        self.visualizer.update_agents(vis_agents)
        self.visualizer.update_agent_paths(vis_paths)


    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self._handle_click(event.pos)

    def _poll_planning_results(self):
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

            if res.plan_id != agent["plan_id"]:
                continue

            if res.error is not None:
                print(f"[PLAN] {agent['id']} planning error: {res.error}")
                agent["state"] = "IDLE"
                continue

            if res.best_cell is None or res.best_path_info is None:
                agent["state"] = "IDLE"
                continue

            self.global_reserved_cells.update(res.best_reserved)
            agent["reserved_cells"] = set(res.best_reserved)

            agent["env_2d"] = res.env_2d
            agent["coord_converter"] = CoordinateConverter(res.env_radius, self.target_zone_radius, self.shovel_width)

            try:
                self.visualizer.update_env(res.env_2d)
                self.coord_converter = agent["coord_converter"]
            except Exception:
                pass

            self._setup_agent_selection(
                agent,
                cell=res.best_cell,
                choice=res.best_choice,
                path_info=res.best_path_info
            )
            agent["state"] = "NAVIGATING"
            self._needs_redraw = True

            print(f"[PLAN] {agent['id']} -> NAVIGATING (cell=({res.best_cell.x},{res.best_cell.y}))")

    def calculate_dynamic_env_radius(self, safety_margin=0.5):
        """
        Calculate environment radius based on the farthest object from center.
        Adapted from pybullet_integration.py
        """
        max_distance = 0.0
        
        # Iterate over stored pebble IDs
        for obj_id in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(obj_id)
            distance = math.hypot(pos[0], pos[1])
            max_distance = max(max_distance, distance)

        # Add safety margin
        dynamic_radius = max_distance + safety_margin

        # Ensure minimum radius (at least target zone + small buffer)
        min_radius = self.target_zone_radius + 0.1
        dynamic_radius = max(dynamic_radius, min_radius)

        print(f"\n(MEASUREMENT) Dynamic Environment Radius Calculation:")
        print(f"  - Farthest object: {max_distance:.3f}m")
        print(f"  - New radius: {dynamic_radius:.3f}m")

        return dynamic_radius

    def _snapshot_pebbles_xy(self) -> np.ndarray:
        pts = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            if pos[2] > -0.5:
                pts.append((float(pos[0]), float(pos[1])))
        if len(pts) == 0: return np.zeros((0, 2), dtype=float)
        return np.array(pts, dtype=float)

    def _start_planning_for_agent(self, agent):
        fut = agent.get("future", None)
        if fut is not None and not fut.done():
            return

        pebbles_xy = self._snapshot_pebbles_xy()
        env_r = self.calculate_dynamic_env_radius()

        agent["plan_id"] += 1
        pid = agent["plan_id"]

        st = get_state(agent["body_id"])
        rover_xy = (float(st[0]), float(st[1]))

        reserved = tuple(self.global_reserved_cells)

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
        agent["future"] = self.executor.submit(plan_for_agent, snap)
        print(f"[PLAN] Submitted plan for {agent['id']} (plan_id={pid})")

    def _handle_auto_mode(self):
        """Auto allocate non-overlapping trajectories to idle agents."""
        for agent in self.agents:
            if agent['index'] != 0:
                continue # Freeze agent 1 for testing
            if agent['state'] == "IDLE":
                self._start_planning_for_agent(agent)

    def _handle_click(self, pos):
        if self.auto_mode:
            return # Ignore clicks during auto mode evaluation
            
        cell = self.visualizer.handle_click_event(pos)
        if not cell:
            return

        print(f"\nClicked ({cell.x}, {cell.y}) with {cell.num_objects} objects")

        choice = "target"
        paths = self.env_2d.get_path_for_preview(cell, choice)
        if not paths:
            print(f"No {choice} path found")
            return

        print("Manual click pathing not fully supported in Multi-Agent mode yet.")
    def _setup_agent_selection(self, agent, cell, choice, path_info):
        """Setup execution logic and visual debug for an agent's intended path"""
        traj = path_info['path']
        print(f"Agent {agent['id']} planned {choice} path: {len(traj)} waypoints, dist={path_info['distance']:.2f}")

        # Update global visualizer (optional depending on how we want to handle multiple path visualization)
        self.visualizer.set_trajectory_preview(cell, choice, path_info)

        # Generate smooth trajectory
        grid_wps = [(c.x, c.y) for c in traj]
        spline_pts, _, success = smooth_path_with_spline(grid_wps, SPILLAGE_CONFIG['smoothing_factor'], SPILLAGE_CONFIG['num_points'])

        if not success:
            spline_pts = grid_wps

        # Convert to world
        world_pts = []
        for gx, gy in spline_pts:
            wx, wy = agent['coord_converter'].convert_2d_to_3d(gx, gy)
            world_pts.append((wx, wy))

        # Resample
        world_pts = self._resample(world_pts, SPILLAGE_CONFIG['target_spacing'])

        # Compute gate point (behind path start)
        S0 = np.array(world_pts[0])
        S1 = np.array(world_pts[min(5, len(world_pts)-1)])
        v_path = S1 - S0
        v_path = v_path / (np.linalg.norm(v_path) + 1e-9)
        G = S0 - GATE_CONFIG['gate_back'] * v_path

        # Visualize
        if self.draw_flow_field or self.flow_field_vis_mode == "always":
            p.removeAllUserDebugItems()
            self._redraw_grid()
    
            # Blue trajectory
            for i in range(len(world_pts) - 1):
                p.addUserDebugLine([world_pts[i][0], world_pts[i][1], 0.02],
                                   [world_pts[i+1][0], world_pts[i+1][1], 0.02], [0, 0, 1], 2.0)
    
            # Green sphere at S0 (path start)
            s0_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=[0, 1, 0, 1])
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=s0_vis, basePosition=[S0[0], S0[1], 0.05])
    
            # Red sphere at G (gate/approach point)
            g_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.06, rgbaColor=[1, 0, 0, 1])
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=g_vis, basePosition=[G[0], G[1], 0.05])
    
            # Red line from G to S0
            p.addUserDebugLine([G[0], G[1], 0.03], [S0[0], S0[1], 0.03], [1, 0, 0], 2.0)
    
            p.addUserDebugText("S0", [S0[0], S0[1], 0.12], [0, 0.5, 0], 1.0)
            p.addUserDebugText("G", [G[0], G[1], 0.10], [0.5, 0, 0], 1.0)

        # Store in agent dict
        agent['selection'] = {
            'cell': cell,
            'path_type': choice,
            'path_info': path_info,
            'world_pts': world_pts,
            'gate': (G[0], G[1]),
            'path_start': (S0[0], S0[1]),
        }
        
        # Start navigation via NavigationManager
        state_now = get_state(agent["body_id"])
        nav_agent_dict = {
            "id": agent["id"],
            "state": state_now,
            "priority": 0.0,
            "shape": {"circles": [(0.0, 0.0, 0.35)]}
        }
        agent['navigator'].set_agent(nav_agent_dict)
        
        full_path = [(G[0], G[1])] + world_pts
        agent['navigator'].start_navigation(full_path, self.pebble_centers)

        print(f"Gate G: ({G[0]:.2f}, {G[1]:.2f}), Path start S0: ({S0[0]:.2f}, {S0[1]:.2f})")

    def _resample(self, pts, spacing):
        if len(pts) < 2:
            return pts
        dists = [0.0]
        for i in range(1, len(pts)):
            dists.append(dists[-1] + math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]))
        total = dists[-1]
        if total < spacing:
            return pts
        n = max(2, int(total / spacing) + 1)
        out = []
        for i in range(n):
            s = i * total / (n - 1)
            j = 0
            while j < len(dists) - 1 and dists[j+1] < s:
                j += 1
            if j >= len(pts) - 1:
                out.append(pts[-1])
            else:
                t = (s - dists[j]) / (dists[j+1] - dists[j] + 1e-9)
                x = pts[j][0] + t * (pts[j+1][0] - pts[j][0])
                y = pts[j][1] + t * (pts[j+1][1] - pts[j][1])
                out.append((x, y))
        return out

    def _step_agents(self, dt):
        """Tick all agents' state machines."""
        # 1. Update all agent states
        for agent in self.agents:
            state_now = get_state(agent["body_id"])
            agent["state_val"] = state_now
            
        # 2. Step active agents
        for agent in self.agents:
            if agent['state'] == "IDLE":
                continue
            elif agent['state'] == "NAVIGATING":
                other_agents = [a for a in self.agents if a['id'] != agent['id']]
                
                # Format other_agents for ORCA 
                orca_others = []
                for oa in other_agents:
                    orca_others.append({
                        "id": oa["id"],
                        "state": oa["state_val"],
                        "shape": {"circles": [(0.0, 0.0, 0.35)]}
                    })
                
                # Agent state format expected by NavigationManager
                nav_agent_dict = {
                    "id": agent["id"],
                    "state": agent["state_val"],
                    "priority": 0.0,  # Usually NavigationManager handles setting this via sailing_priority
                    "shape": {"circles": [(0.0, 0.0, 0.35)]}
                }
                
                agent['navigator'].set_agent(nav_agent_dict) 
                agent['navigator'].update_pebbles(self.pebble_centers)
                status = agent['navigator'].step(dt, orca_others)
                
                set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], status.v_cmd, status.w_cmd)
                
                if status.completed:
                    print(f"  Agent {agent['id']} path complete. Rolling back.")
                    agent['state'] = "ROLLBACK"
                    agent['rollback_timer'] = 0.5
            elif agent['state'] == "ROLLBACK":
                self._step_rollback(agent, dt)
            elif agent['state'] == "SYNC":
                self._step_sync(agent)

    def _step_rollback(self, agent, dt):
        agent['rollback_timer'] -= dt
        if agent['rollback_timer'] > 0:
            set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], -0.5, 0.0)
        else:
            set_wheel_velocities(agent["body_id"], agent["left_joint"], agent["right_joint"], 0, 0)
            agent['state'] = "SYNC"

    def _step_sync(self, agent):
        print(f"\n[SYNC] Agent {agent['id']} reached target. Going IDLE.")
        
        # Release reserved cells
        self.global_reserved_cells.difference_update(agent['reserved_cells'])
        agent['reserved_cells'] = set()
        agent['state'] = "IDLE"
        agent['selection'] = None
        
        self.visualizer.clear_trajectory_preview()
        self._needs_redraw = True
def main():
    # --- Configuration ---
    # 1. Use Spillage Model in 2D Algorithm? (True/False)
    USE_SPILLAGE_IN_2D = True
    
    # 2. Show "Spillage Visualization for All Paths" figure? (True/False)
    # Recommended False during AUTO_MODE to avoid blocking
    SHOW_SPILLAGE_PLOT = False
    
    # 3. Flow Field Visualization in 3D: "ask", "always", "never"
    # Recommended "never" or "always" during AUTO_MODE
    FLOW_FIELD_VIS_3D = "never"
    
    # 4. Automate rover path selection and execution
    AUTO_MODE = True
    
    # Apply to Global Config
    SIMULATION_CONFIG['use_spillage_model'] = USE_SPILLAGE_IN_2D
    SIMULATION_CONFIG['visualize_potential'] = SHOW_SPILLAGE_PLOT
    
    print(f"Config: Spillage2D={USE_SPILLAGE_IN_2D}, ShowPlot={SHOW_SPILLAGE_PLOT}, 3D_Vis={FLOW_FIELD_VIS_3D}, Auto={AUTO_MODE}")

    orch = MultiAgentHybridOrchestrator(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=50,
        random_seed=41,
        initial_robot_poses=[(2.0, 2.0, -math.pi/2), (-2.0, -2.0, math.pi/2)],
        shovel_width=0.22,
        auto_mode=AUTO_MODE)
    
    orch.flow_field_vis_mode = FLOW_FIELD_VIS_3D
    
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()

