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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import run_2d_env
from coordinate_converter import CoordinateConverter
from spillage_model import smooth_path_with_spline
from flowfield_base import FlowFieldController, FlowField2D

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


# ===================== FLOW FIELD FOR NAVIGATION =====================



class PathGuidanceField2D(FlowField2D):
    """
    Vector-field-based path following on the same 2D grid as FlowField2D.

    For each grid cell:
      - Find closest point on the reference path and its tangent t_hat.
      - Compute signed lateral error e_n to the path (using left normal n_hat).
      - Direction is v = k_t * t_hat - k_n * e_n * n_hat, then normalized.
    """

    def __init__(self,
                 world_xmin=-1.2,
                 world_xmax=+1.2,
                 world_ymin=-1.2,
                 world_ymax=+1.2,
                 grid_w=81,
                 grid_h=81,
                 rover_radius=0.25,
                 pebble_radius=0.05,
                 # clearance=0.02,  <-- REMOVED because base class doesn't accept it
                 k_t=1.0,
                 k_n=30.0,
                 band_radius=1.0):
        super().__init__(
            world_xmin=world_xmin,
            world_xmax=world_xmax,
            world_ymin=world_ymin,
            world_ymax=world_ymax,
            grid_w=grid_w,
            grid_h=grid_h,
            rover_radius=rover_radius,
            pebble_radius=pebble_radius)
            # clearance=clearance) <-- REMOVED
        self.k_t = float(k_t)
        self.k_n = float(k_n)
        self.band_radius = float(band_radius)
        self.path_points = None

    # ---------- geometry helpers ----------

    def _closest_point_on_path(self, px, py):
        """
        Return (closest_point_on_path, unit_tangent_there).
        """
        assert self.path_points is not None and len(self.path_points) >= 2

        p_world = np.array([px, py], dtype=float)
        best_dist2 = float("inf")
        best_closest = None
        best_tangent = None

        for i in range(len(self.path_points) - 1):
            a = self.path_points[i]
            b = self.path_points[i + 1]
            ab = b - a
            ab_len2 = float(np.dot(ab, ab))
            if ab_len2 < 1e-9:
                continue

            t = float(np.dot(p_world - a, ab) / ab_len2)
            t = max(0.0, min(1.0, t))
            closest = a + t * ab
            diff = p_world - closest
            d2 = float(np.dot(diff, diff))
            if d2 < best_dist2:
                best_dist2 = d2
                best_closest = closest
                ab_len = math.sqrt(ab_len2)
                t_hat = ab / (ab_len + 1e-9)
                best_tangent = t_hat

        if best_closest is None:
            a = self.path_points[0]
            b = self.path_points[1]
            ab = b - a
            ab_len = float(np.linalg.norm(ab))
            t_hat = ab / (ab_len + 1e-9)
            return a, t_hat

        return best_closest, best_tangent

    # ---------- build path-based vector field ----------

    def compute_path_direction_field(self, path_points):
        """
        Compute direction vectors only in a corridor (band_radius) around the path.
        """
        self.path_points = np.array(path_points, dtype=float)
        gh, gw = self.grid_h, self.grid_w
        self.dir_field[:, :, :] = 0.0

        cell_size = min(self.cell_w, self.cell_h)
        band_cells = int(math.ceil(self.band_radius / cell_size))
        band_cells = max(band_cells, 1)

        mask = np.zeros((gh, gw), dtype=bool)

        # Mark band cells
        for (px, py) in self.path_points:
            ix, iy = self.world_to_cell(px, py)
            for dy in range(-band_cells, band_cells + 1):
                yy = iy + dy
                if yy < 0 or yy >= gh:
                    continue
                for dx in range(-band_cells, band_cells + 1):
                    xx = ix + dx
                    if xx < 0 or xx >= gw:
                        continue
                    if dx * dx + dy * dy <= band_cells * band_cells:
                        mask[yy, xx] = True

        ys, xs = np.where(mask)
        for iy, ix in zip(ys, xs):
            if self.obstacles[iy, ix]:
                continue

            cx, cy = self.cell_to_world_center(ix, iy)
            path_pt, t_hat = self._closest_point_on_path(cx, cy)

            n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
            e_vec = np.array([cx, cy], dtype=float) - path_pt
            e_n = float(np.dot(e_vec, n_hat))

            v = self.k_t * t_hat - self.k_n * e_n * n_hat
            mag = float(np.linalg.norm(v))
            if mag < 1e-6:
                continue
            v /= mag

            self.dir_field[iy, ix, 0] = v[0]
            self.dir_field[iy, ix, 1] = v[1]

    def rebuild_for_path(self, path_points, pebble_centers):
        """
        Stamp obstacles from pebbles, then build the banded path-guidance field.
        """
        self.clear_obstacles()
        self.stamp_pebbles(pebble_centers)
        self.compute_path_direction_field(path_points)


# ===================== ARC LENGTH HELPERS =====================

def precompute_arc_length(path_points):
    pts = np.array(path_points, dtype=float)
    if len(pts) < 2:
        return pts, [], [], [0.0], 0.0
    segs = pts[1:] - pts[:-1]
    seg_lens = np.linalg.norm(segs, axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_L = s_cum[-1]
    return pts, segs, seg_lens, s_cum, total_L


def project_point_to_path_s(p_world, pts, segs, seg_lens, s_cum):
    best_d2 = float("inf")
    best_s = 0.0

    for i in range(len(segs)):
        a = pts[i]
        ab = segs[i]
        L2 = seg_lens[i] ** 2
        if L2 < 1e-12:
            continue

        t = np.dot(p_world - a, ab) / L2
        t = np.clip(t, 0.0, 1.0)
        proj = a + t * ab

        d2 = np.dot(p_world - proj, p_world - proj)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s_cum[i] + t * seg_lens[i]

    return best_s, math.sqrt(best_d2)



# ===================== MAIN ORCHESTRATOR =====================

class HybridOrchestrator:
    def __init__(self, env_radius=3.0, target_zone_radius=0.3, num_pebbles=50,
                 random_seed=41, initial_robot_pose=(2.0, 2.0, -math.pi/2),
                 shovel_width=0.22, auto_mode=False):
        self.env_radius = env_radius
        self.target_zone_radius = target_zone_radius
        self.num_pebbles = num_pebbles
        self.random_seed = random_seed
        self.initial_robot_pose = initial_robot_pose
        self.shovel_width = shovel_width
        self.auto_mode = auto_mode

        # Visualization Config
        # "ask", "always", "never"
        self.flow_field_vis_mode = "ask" 

        # PyBullet state
        self.body_id = None
        self.left_joint = None
        self.right_joint = None
        self.left_joint = None
        self.right_joint = None
        self.pebble_centers = []
        self.draw_flow_field = False

        # Components
        self.coord_converter = None
        self.env_2d = None
        self.visualizer = None

        self.running = False
        self.current_selection = None

    def initialize(self):
        print("=" * 60)
        print("Initializing Hybrid Orchestrator")
        print("=" * 60)

        # PyBullet setup
        print("\n[1/3] Setting up PyBullet...")
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

        # Rover
        here = os.path.dirname(os.path.abspath(__file__))
        rover_urdf = os.path.join(here, "2_wheel_rover.urdf")
        x, y, yaw = self.initial_robot_pose
        self.body_id = p.loadURDF(rover_urdf, [x, y, 0.02], yaw_to_quat(yaw))
        self.left_joint, self.right_joint = find_wheel_joints(self.body_id)

        p.changeDynamics(self.body_id, -1, lateralFriction=0.8)
        for j in (self.left_joint, self.right_joint):
            p.changeDynamics(self.body_id, j, lateralFriction=1.0, rollingFriction=0.0, spinningFriction=0.0)
            p.setJointMotorControl2(self.body_id, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)

        # Pebbles
        pebble_urdf = os.path.join(here, "pebbles.urdf")
        np.random.seed(self.random_seed)
        self.pebble_ids = []  # Store IDs to query positions later
        for _ in range(self.num_pebbles):
            # random pos in annulus [target_zone_radius, env_radius]
            # r = sqrt(U * (Rmax^2 - Rmin^2) + Rmin^2) for uniform area
            r_min = self.target_zone_radius
            r_max = self.env_radius
            r = math.sqrt(np.random.rand() * (r_max**2 - r_min**2) + r_min**2)
            phi = 2 * math.pi * np.random.rand()
            px, py = r * math.cos(phi), r * math.sin(phi)
            bid = p.loadURDF(pebble_urdf, [px, py, 0.01], useFixedBase=False)
            p.changeDynamics(bid, -1, lateralFriction=0.8)
            self.pebble_centers.append((px, py))
            self.pebble_ids.append(bid)

        print(f"  Rover at ({x:.2f}, {y:.2f}), {len(self.pebble_centers)} pebbles")

        # Coordinate converter
        print("\n[2/3] Setting up 2D environment...")
        self.coord_converter = CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width)

        # Grid overlay
        gs = self.coord_converter.grid_size
        cs = self.coord_converter.cell_size
        for i in range(gs + 1):
            x = -self.env_radius + i * cs
            p.addUserDebugLine([x, -self.env_radius, 0.01], [x, self.env_radius, 0.01], [0.5,0.5,0.5], 1.0)
            y = -self.env_radius + i * cs
            p.addUserDebugLine([-self.env_radius, y, 0.01], [self.env_radius, y, 0.01], [0.5,0.5,0.5], 1.0)

        # 2D environment
        pebbles_3d = [(x, y, 0.01) for x, y in self.pebble_centers]
        self.env_2d, self.visualizer = run_2d_env(
            self.env_radius, self.target_zone_radius, self.shovel_width,
            pebbles_3d, manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
            visualize_potential=SIMULATION_CONFIG['visualize_potential'])

        # Removed initial prompt, moving to execute logic
        # val = input("Enable flow field visualization for navigation? (y/N): ").strip().lower()
        # self.draw_flow_field = (val == 'y')
        
        print("\n[3/3] Ready!")
        print("=" * 60)
        print("Controls: Click=Select, ENTER=Execute, ESC=Cancel/Quit")
        print("=" * 60 + "\n")

    def run(self):
        self.running = True
        sim_dt = 1/240
        display_acc = 0.0

        while self.running and p.isConnected():
            p.stepSimulation()
            time.sleep(sim_dt)
            display_acc += sim_dt

            if display_acc >= 1/60:
                display_acc = 0.0
                self._handle_events()
                
                if self.auto_mode and self.current_selection is None:
                    self._handle_auto_mode()
                
                self.visualizer.screen.fill((255, 255, 255))
                self.visualizer.draw_elements()
                pygame.display.flip()

        pygame.quit()
        p.disconnect()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    if self.current_selection:
                        self.current_selection = None
                        self.visualizer.clear_trajectory_preview()
                        p.removeAllUserDebugItems()
                        self._redraw_grid()
                    else:
                        self.running = False
                elif event.key == pygame.K_RETURN and self.current_selection:
                    self._execute_trajectory()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self._handle_click(event.pos)

    def _redraw_grid(self):
        gs = self.coord_converter.grid_size
        cs = self.coord_converter.cell_size
        for i in range(gs + 1):
            x = -self.env_radius + i * cs
            p.addUserDebugLine([x, -self.env_radius, 0.01], [x, self.env_radius, 0.01], [0.5,0.5,0.5], 1.0)
            y = -self.env_radius + i * cs
            p.addUserDebugLine([-self.env_radius, y, 0.01], [self.env_radius, y, 0.01], [0.5,0.5,0.5], 1.0)

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

    def _handle_auto_mode(self):
        best_cell = None
        best_score = -float('inf')
        best_path_info = None
        best_choice = "target"

        state = get_state(self.body_id)
        rover_pos = (float(state[0]), float(state[1]))

        for cell in self.env_2d.cells_with_objects:
            # We focus on target paths for automation, but highway could be added
            paths = self.env_2d.get_path_for_preview(cell, "target")
            if not paths:
                continue
            path_info = paths[0]
            objects = path_info.get('objects', 0)
            if objects <= 0:
                continue
            
            traj = path_info.get('path', [])
            if not traj:
                continue
                
            start_c = traj[0]
            start_wx, start_wy = self.coord_converter.convert_2d_to_3d(start_c.x, start_c.y)
            dist_to_start = math.hypot(rover_pos[0] - start_wx, rover_pos[1] - start_wy)
            
            # Score formula: maximize objects, minimize distance from rover to start
            # Adjust distance penalty factor as needed (e.g. 0.5 means 1m travel costs 0.5 objects)
            score = objects - 0.5 * dist_to_start
            
            if score > best_score:
                best_score = score
                best_cell = cell
                best_path_info = path_info

        if best_cell:
            print(f"\n[AUTO MODE] Selected cell ({best_cell.x}, {best_cell.y}) "
                  f"with {best_path_info['objects']} objects. Score: {best_score:.2f}")
            
            self._setup_selection(best_cell, best_choice, best_path_info)
            
            # Preview before executing
            self.visualizer.screen.fill((255, 255, 255))
            self.visualizer.draw_elements()
            pygame.display.flip()
            time.sleep(1.0) # Show for 1 second before moving
            
            self._execute_trajectory()
        else:
            print("\n[AUTO MODE] No valid paths found bringing objects to target zone.")
            self.auto_mode = False

    def _handle_click(self, pos):
        if self.auto_mode:
            return # Ignore clicks during auto mode evaluation
            
        cell = self.visualizer.handle_click_event(pos)
        if not cell:
            return

        print(f"\nClicked ({cell.x}, {cell.y}) with {cell.num_objects} objects")

        choice = "target"
        if self.current_selection and self.current_selection['cell'].x == cell.x and self.current_selection['cell'].y == cell.y:
            choice = "highway" if self.current_selection['path_type'] == "target" else "target"

        paths = self.env_2d.get_path_for_preview(cell, choice)
        if not paths:
            print(f"No {choice} path found")
            return

        path_info = paths[0]
        self._setup_selection(cell, choice, path_info)
        
    def _setup_selection(self, cell, choice, path_info):
        traj = path_info['path']
        print(f"Planned {choice} path: {len(traj)} waypoints, dist={path_info['distance']:.2f}")

        self.visualizer.set_trajectory_preview(cell, choice, path_info)

        # Generate smooth trajectory
        grid_wps = [(c.x, c.y) for c in traj]
        spline_pts, _, success = smooth_path_with_spline(grid_wps, SPILLAGE_CONFIG['smoothing_factor'], SPILLAGE_CONFIG['num_points'])

        if not success:
            spline_pts = grid_wps

        # Convert to world
        world_pts = []
        for gx, gy in spline_pts:
            wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
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

        self.current_selection = {
            'cell': cell,
            'path_type': choice,
            'path_info': path_info,
            'world_pts': world_pts,
            'gate': (G[0], G[1]),
            'path_start': (S0[0], S0[1]),
        }

        print(f"Gate G: ({G[0]:.2f}, {G[1]:.2f}), Path start S0: ({S0[0]:.2f}, {S0[1]:.2f})")
        if not self.auto_mode:
            print("Press ENTER to execute, ESC to cancel")

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

    def _execute_trajectory(self):
        if not self.current_selection:
            return

        gate = self.current_selection['gate']
        path_start = self.current_selection['path_start']
        world_pts = self.current_selection['world_pts']

        print("\n" + "=" * 50)
        print("EXECUTION SETUP")
        
        # Flow Field Visualization Logic based on self.flow_field_vis_mode
        if self.flow_field_vis_mode == "always":
            self.draw_flow_field = True
            print("Flow field visualization: ENABLED (Always)")
        elif self.flow_field_vis_mode == "never":
            self.draw_flow_field = False
            print("Flow field visualization: DISABLED (Never)")
        else:
            # Default to "ask"
            val = input("Enable flow field visualization for this movement? (y/N): ").strip().lower()
            self.draw_flow_field = (val == 'y')
            
        print(f"Visualization: {self.draw_flow_field}")
        print("=" * 50)

        print("\n" + "=" * 50)
        print("PHASE 1: Navigate to gate point G")
        print("=" * 50)
        if self.draw_flow_field:
             p.removeAllUserDebugItems()
             self._redraw_grid()
        self._navigate_to_point(gate)

        # Build extended path: gate -> path_start -> trajectory
        # The path-following field will pull the rover from gate through S0
        # and along the trajectory in one continuous motion.
        state_now = get_state(self.body_id)
        rover_pos = (float(state_now[0]), float(state_now[1]))

        # Extension segment: current rover position -> path start
        ext_segment = self._resample([rover_pos, path_start], SPILLAGE_CONFIG['target_spacing'])

        # Full path: extension + actual trajectory (skip duplicate at junction)
        full_path = list(ext_segment) + list(world_pts)

        print("\n" + "=" * 50)
        print("PHASE 2: Follow trajectory (approach + path combined)")
        print("=" * 50)
        if self.draw_flow_field:
             p.removeAllUserDebugItems()
             self._redraw_grid()
        self._follow_path(full_path)

        # Phase 2.5: Reverse (Rollback) to clear space
        print("\n" + "=" * 50)
        print("PHASE 2.5: Reverse Motion")
        print("=" * 50)
        # Reverse to clear space
        # --- CONFIGURABLE REVERSE DISTANCE ---
        # The distance reversed is roughly t_reverse * abs(v_reverse).
        # To change how far it moves back, alter the 't_reverse' value below (time in seconds).
        t_reverse = 1 
        v_reverse = -0.5
        t_start = time.time()
        while time.time() - t_start < t_reverse:
             set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, v_reverse, 0.0)
             p.stepSimulation()
             time.sleep(1./240.)
        # Stop
        set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, 0.0, 0.0)
        for _ in range(20): 
            p.stepSimulation()
            time.sleep(1./240.)

        # Phase 3: Update and Synchronize
        print("\n" + "=" * 50)
        print("PHASE 3: Update 2D World State")
        print("=" * 50)
        
        # 1. Update pebble positions
        new_pebble_centers = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            new_pebble_centers.append((pos[0], pos[1]))
        self.pebble_centers = new_pebble_centers

        # 2. Dynamic Resizing
        new_radius = self.calculate_dynamic_env_radius()
        self.env_radius = new_radius
        
        # 3. Recreate 2D Environment completely
        print("(SYNC) Recreating 2D environment with updated state...")
        
        # Re-init coordinate converter to match new radius
        self.coord_converter = CoordinateConverter(self.env_radius, self.target_zone_radius, self.shovel_width)
        self._redraw_grid() # Update 3D grid lines too
        
        pebbles_3d = [(x, y, 0.01) for x, y in self.pebble_centers]
        
        # Close old pygame window if needed? No, run_2d_env might create a new one or re-use?
        # run_2d_env creates a new SimulationVisualizer which inits pygame.
        # Ideally we should close the old one if it spawned a window, but pygame.quit() is global.
        # We'll just overwrite self.env_2d and self.visualizer.
        
        self.env_2d, self.visualizer = run_2d_env(
            self.env_radius, self.target_zone_radius, self.shovel_width,
            pebbles_3d, manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
            visualize_potential=SIMULATION_CONFIG['visualize_potential'])

        self.visualizer.clear_trajectory_preview()
        self.current_selection = None
        print("\nTrajectory complete! Environment synchronized.")

    def _navigate_to_point(self, goal):
        #"\"\"Navigate to a point using flow field - ALL INLINE.\"\"\"
        gx, gy = goal

        # Dynamic grid size based on environment radius
        # Keep resolution ~0.15m per cell
        cell_size = 0.15
        grid_dim = int(math.ceil(2 * self.env_radius / cell_size))
        # Ensure odd grid size for center alignment
        if grid_dim % 2 == 0: grid_dim += 1
        
        # Build flow field
        ff = FlowField2D(-self.env_radius, self.env_radius,
                         -self.env_radius, self.env_radius,
                         grid_w=grid_dim, grid_h=grid_dim)
        
        # Custom FlowField build sequence to include Target Zone
        gx_cell, gy_cell = ff.world_to_cell(gx, gy)
        
        # 1. Clear & Stamp pebbles
        ff.clear_obstacles()
        ff.stamp_pebbles(self.pebble_centers)
        
        # 2. Stamp Target Zone (CRITICAL: Must be done BEFORE compute_distance_field)
        # Target zone is at (0,0)
        c0x, c0y = ff.world_to_cell(0, 0)
        # Use a slightly larger radius to ensures the obstacles cover the zone fully
        tz_r = self.target_zone_radius + 0.1 
        
        # Scan bounding box cells
        steps_x = int(math.ceil(tz_r / ff.cell_w))
        steps_y = int(math.ceil(tz_r / ff.cell_h))

        for dy in range(-steps_y, steps_y + 1):
            for dx in range(-steps_x, steps_x + 1):
                ix, iy = c0x + dx, c0y + dy
                if 0 <= ix < ff.grid_w and 0 <= iy < ff.grid_h:
                    wx, wy = ff.cell_to_world_center(ix, iy)
                    if math.hypot(wx, wy) <= tz_r:
                        ff.obstacles[iy, ix] = True
                        
        # 3. Compute fields respecting ALL obstacles
        ff.compute_distance_field((gx_cell, gy_cell))
        ff.compute_direction_field()

        if self.draw_flow_field:
            print("  Drawing flow field...")
            # Clear old debug lines first? Assuming p.removeAllUserDebugItems() handles it in execute_trajectory.
            # But the user said "you are not clear the vector field". 
            # p.removeAllUserDebugItems() wipes the grid lines too. 
            # We will rely on _execute_trajectory calling removeAllUserDebugItems() between phases.
            # For phase 1 & 2, we are here.
            ff.draw_debug(scale=0.15, life_time=0.0)

        sim_dt = 1/240
        acc = 0.0
        v_max = 1.5
        w_max = 10.0
        k_theta = 10.0
        stop_dist = GATE_CONFIG['approach_tol']

        t_start = time.time()
        last_print = t_start
        active_fallback_cell = None

        while p.isConnected():
            # === PHYSICS STEP ===
            p.stepSimulation()
            time.sleep(sim_dt)
            acc += sim_dt

            # === CONTROL AT 20 Hz ===
            if acc >= 0.05:
                acc = 0.0

                state = get_state(self.body_id)
                x, y, yaw, v_fwd, w = state

                dist = math.hypot(gx - x, gy - y)

                # Check arrival
                if dist < stop_dist:
                    set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, 0, 0)
                    print(f"  Arrived at ({gx:.2f}, {gy:.2f}), dist={dist:.3f}m")
                    break

                # Flow field direction
                vx, vy = ff.get_direction_world(x, y)
                
                # Check for dead zone or obstacle (zero vector)
                if abs(vx) < 1e-3 and abs(vy) < 1e-3:
                    # Clear fallback cell if we actually got close to it and we're *still* on a dead cell
                    # This prevents the rover orbiting a fallback cell it already reached
                    if active_fallback_cell is not None:
                        tx_check, ty_check = ff.cell_to_world_center(active_fallback_cell[0], active_fallback_cell[1])
                        dist_to_fallback = math.hypot(tx_check - x, ty_check - y)
                        if dist_to_fallback < 0.2: # Increased threshold to 0.2m to prevent orbiting the exact center
                            active_fallback_cell = None
                    # User Request: Refined logic.
                    # 1. Check if straight path to goal (gx, gy) intersects Target Zone?
                    # 2. If YES (or forced by config): Use BFS to find nearest valid cell (avoid crossing zone).
                    # 3. If NO (and config allows): Go straight to goal (gx, gy).
                    
                    # --- Configurable Fallback Mode ---
                    ALWAYS_USE_BFS_FALLBACK = True # Set to False to use the intersection check
                    
                    if ALWAYS_USE_BFS_FALLBACK:
                        use_bfs = True
                    else:
                        # --- Intersection Check (Line Segment vs Circle) ---
                        # Segment P1(x,y) -> P2(gx,gy). Circle (0,0, radius=target_zone_radius)
                        # Vector d = P2 - P1. Vector f = P1 - C = P1.
                        dx_seg = gx - x
                        dy_seg = gy - y
                        a = dx_seg**2 + dy_seg**2
                        b = 2 * (x * dx_seg + y * dy_seg)
                        c = (x**2 + y**2) - self.target_zone_radius**2
                        
                        intersects = False
                        if a > 1e-9:
                            delta = b**2 - 4*a*c
                            if delta >= 0:
                                sqrt_delta = math.sqrt(delta)
                                t1 = (-b - sqrt_delta) / (2*a)
                                t2 = (-b + sqrt_delta) / (2*a)
                                if (0 <= t1 <= 1) or (0 <= t2 <= 1):
                                    intersects = True
    
                        use_bfs = intersects
                    
                    if not use_bfs:
                        # Safe to go straight (for now)
                        # NOTE: This does NOT mean we go all the way to goal blindly.
                        # In the next control loop iteration (0.05s later), we check ff.get_direction_world again.
                        # If this motion brings us into a valid field cell, we resume flow-field navigation immediately.
                        vx, vy = gx - x, gy - y
                    else:
                        # Must avoid zone -> Find nearest valid flow cell
                        cx, cy = ff.world_to_cell(x, y)
                        found_valid = False
                        target_valid_cell = None
                        
                        if active_fallback_cell is not None:
                            target_valid_cell = active_fallback_cell
                            found_valid = True
                        else:
                            # BFS to find nearest cell with non-zero flow
                            queue = [(cx, cy)]
                        visited = set([(cx, cy)])
                        
                        # Limit search depth to avoid hanging
                        max_search_steps = 100 
                        steps_count = 0
                        
                        import collections
                        bfs_q = collections.deque([(cx, cy)])
                        
                        while bfs_q and steps_count < max_search_steps:
                            cur_x, cur_y = bfs_q.popleft()
                            steps_count += 1
                            
                            # Check if this cell has valid flow
                            # We need to access dir_field directly
                            if 0 <= cur_x < ff.grid_w and 0 <= cur_y < ff.grid_h:
                                dvx = ff.dir_field[cur_y, cur_x, 0]
                                dvy = ff.dir_field[cur_y, cur_x, 1]
                                if abs(dvx) > 1e-3 or abs(dvy) > 1e-3:
                                    target_valid_cell = (cur_x, cur_y)
                                    found_valid = True
                                    break
                            
                            # Add neighbors
                            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                                nx, ny = cur_x + dx, cur_y + dy
                                if (nx, ny) not in visited:
                                    if 0 <= nx < ff.grid_w and 0 <= ny < ff.grid_h:
                                        visited.add((nx, ny))
                                        bfs_q.append((nx, ny))
                        
                        if found_valid and target_valid_cell:
                            # Save it so we stick to this cell until we hit a valid flow region
                            active_fallback_cell = target_valid_cell
                            
                            # Steer towards the center of that valid cell
                            tx, ty = ff.cell_to_world_center(target_valid_cell[0], target_valid_cell[1])
                            vx, vy = tx - x, ty - y
                            
                            # Prevent atan2 bearing jitter when getting close to the cell center.
                            # When the target cell is close (e_theta is computed from tiny vectors), tiny x,y
                            # movements cause huge atan2 angle swings (chattering)
                            mag_temp = math.hypot(vx, vy)
                            
                            if mag_temp < 0.35: # Close range threshold
                                # Pull the valid flow vector from the target cell
                                dvx = ff.dir_field[target_valid_cell[1], target_valid_cell[0], 0]
                                dvy = ff.dir_field[target_valid_cell[1], target_valid_cell[0], 1]
                                
                                # Extrapolate a synthetic lookahead target L meters *past* the cell center
                                # This creates a stable orientation vector rather than aiming at a near point
                                L = 0.6
                                if abs(dvx) > 1e-3 or abs(dvy) > 1e-3:
                                    t_look_x = tx + dvx * L
                                    t_look_y = ty + dvy * L
                                    vx, vy = t_look_x - x, t_look_y - y
                                    
                                if mag_temp < 0.1:
                                    # We are officially perfectly on the cell. Release the lock to resume normal flow.
                                    active_fallback_cell = None
                            
                            # This flag tells the downstream logic we are in fallback mode
                            found_valid = True

                        else:
                            # Fallback if nothing found: Go to goal
                            vx, vy = gx - x, gy - y
                    
                    mag = math.hypot(vx, vy)
                    vx, vy = vx / (mag + 1e-9), vy / (mag + 1e-9)
                else:
                    # We are on a valid flow field cell! Clear any active fallback cell.
                    active_fallback_cell = None

                theta_des = math.atan2(vy, vx)
                e_theta = wrap_angle(theta_des - yaw)

                # We allow it to decelerate naturally using the cos^4 alignment curve
                is_fallback_mode = (found_valid if 'found_valid' in locals() else False)
                
                # Tuned for curves: penalize v_cmd heavily if not aligned to allow w_cmd to dominate
                align = max(0.0, math.cos(e_theta))
                
                # If we are in fallback mode and not fully aligned, slow down the base speed aggressively
                # but don't hard-cutoff to 0.0, to prevent pulsing.
                base_v_max = v_max * 0.5 if (is_fallback_mode and abs(e_theta) > math.radians(10)) else v_max
                
                # Use power 4 to aggressively slow down on curves -> allows differential turn naturally
                v_cmd = base_v_max * (align ** 4) * min(1.0, dist / 0.4)
                
                if is_fallback_mode:
                    k_d = 2.0
                    w_cmd = k_theta * e_theta - k_d * w
                else:
                    w_cmd = k_theta * e_theta
                
                # Saturation limit
                w_cmd = max(-w_max, min(w_max, w_cmd))
                
                # PREVENT DISCRETE-TIME OVERSHOOT (Fixes the +/- 10 rad/s limit cycle)
                # Limits turning speed so it never rotates past e_theta in a single 0.05s control tick
                w_cmd = math.copysign(min(abs(w_cmd), abs(e_theta) / 0.05), w_cmd)
                    
                # Smooth deadband to prevent micro-fluctuations
                if abs(e_theta) < math.radians(5): # Increased to 5 degrees
                    w_cmd = 0.0

                set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, v_cmd, w_cmd)

                # Status
                now = time.time()
                if now - last_print > 1.0:
                    last_print = now
                    print(f"  pos=({x:.2f},{y:.2f}) dist={dist:.2f}m v={v_cmd:.2f} w={w_cmd:.2f}")

                # Process PyGame events (just to keep window responsive / check for QUIT)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                        return

    def _follow_path(self, pts):
        """Follow path using pure FlowField logic + Arc Length progress (Reference Implementation)."""
        if len(pts) < 2:
            return

        pts_array = np.array(pts, dtype=float)

        # Tracking offset: set to 0.0 to track by rover center
        SHOVEL_OFFSET = 0.17

        # --- Build LOCAL bounds around the path (+ current rover position) ---
        state0 = get_state(self.body_id)
        x0, y0 = float(state0[0]), float(state0[1])

        all_xs = list(pts_array[:, 0]) + [x0]
        all_ys = list(pts_array[:, 1]) + [y0]

        path_margin = 1.0
        world_xmin = min(all_xs) - path_margin
        world_xmax = max(all_xs) + path_margin
        world_ymin = min(all_ys) - path_margin
        world_ymax = max(all_ys) + path_margin

        # Grid resolution: ~0.15 m/cell (same as reference)
        desired_cell = 0.15
        grid_w = int(math.ceil((world_xmax - world_xmin) / desired_cell)) + 1
        grid_h = int(math.ceil((world_ymax - world_ymin) / desired_cell)) + 1

        print(f"  Path: {len(pts)} pts, bounds x[{world_xmin:.2f},{world_xmax:.2f}] "
              f"y[{world_ymin:.2f},{world_ymax:.2f}] grid={grid_w}x{grid_h}")

        # --- Build path guidance field ---
        pgf = PathGuidanceField2D(
            world_xmin, world_xmax,
            world_ymin, world_ymax,
            grid_w=grid_w, grid_h=grid_h,
            rover_radius=0.25, pebble_radius=0.05,
            # clearance=0.02,
            k_t=2.0,
            k_n=10.0,
            band_radius=0.5)
        # Ignore pebbles during path following (PUSH behaviour)
        pgf.rebuild_for_path(pts_array, [])

        if self.draw_flow_field:
            print("  Drawing path guidance field...")
            pgf.draw_debug(scale=0.15, life_time=0.0)
            
            # Also draw the reference path
            print("  Drawing reference path...")
            for i in range(len(pts_array) - 1):
                p0 = pts_array[i]
                p1 = pts_array[i+1]
                p.addUserDebugLine([p0[0], p0[1], 0.05], [p1[0], p1[1], 0.05], [0, 1, 1], 2.0, lifeTime=0.0)

        # --- Controller ---
        controller = FlowFieldController(
            v_max=1.0,
            w_max=6.0,
            k_theta=6.0,
            turn_in_place_angle_deg=50.0,
            static_speed_threshold=0.03,
            w_turn_in_place=25.0,
            stop_dist=0.15)

        # --- Precompute Arc Length ---
        path_pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(pts_array)
        
        goal = np.array(pts_array[-1], dtype=float)

        # Simulation / Control Loop
        sim_dt = 1/240
        acc = 0.0
        last_print = time.time()
        
        # Stopping logic variables
        # S_TOL reduced to 0.02 to ensure shovel reaches the very end (pushing objects deep)
        S_TOL = 0.02 
        V_TOL = 0.03
        STOP_HOLD_TIME = 0.4
        low_speed_acc = 0.0
        
        s_prev = 0.0
        t_prev = time.time()
        v_s_ema = 0.0
        ema_alpha = 0.15

        while p.isConnected():
            p.stepSimulation()
            time.sleep(sim_dt)
            acc += sim_dt

            if acc < 0.05:
                continue
            acc = 0.0

            state = get_state(self.body_id)
            x, y, yaw, v_fwd, w = state

            # Tracking point (shovel or center depending on SHOVEL_OFFSET)
            x_s = x + SHOVEL_OFFSET * math.cos(yaw)
            y_s = y + SHOVEL_OFFSET * math.sin(yaw)
            
            # --- Progress Calculation ---
            p_shovel = np.array([x_s, y_s], dtype=float)
            s_now, d_now = project_point_to_path_s(p_shovel, path_pts, segs, seg_lens, s_cum)
            
            # Estimate speed along path (v_s)
            now = time.time()
            dt_real = max(1e-6, now - t_prev)
            v_s = (s_now - s_prev) / dt_real
            v_s_ema = (1.0 - ema_alpha) * v_s_ema + ema_alpha * v_s
            s_prev = s_now
            t_prev = now

            # --- Control (NO ADAPTER, PURE FIELD) ---
            tracking_state = np.array([x_s, y_s, yaw, v_fwd, w], dtype=float)
            v_cmd, w_cmd = controller.compute_control(tracking_state, goal, pgf)

            # --- OVERRIDE: FORCE PUSHING NEAR END ---
            # The controller naturally slows down when dist_goal < 0.4m.
            # But for earth moving, we want to push HARD until the very end (S_TOL).
            # If we are aligned and near the end, force v_cmd to be at least 0.4 m/s.
            if s_now > total_L - 0.5:
                # Check alignment to avoid speeding up while turning
                if abs(w_cmd) < 1.0: 
                    v_cmd = max(v_cmd, 0.4)

            set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, v_cmd, w_cmd)

            # --- Accurate Stopping Logic ---
            remaining_end = max(0.0, total_L - s_now)
            
            if remaining_end < S_TOL:
                # Immediate stop if within tolerance (aggressive pushing)
                set_wheel_velocities(self.body_id, self.left_joint, self.right_joint, 0, 0)
                print(f"  Path complete (Arc Length). rem={remaining_end:.3f}m, v_s={v_s_ema:.3f}")
                break

            # Status
            if now - last_print > 1.0:
                last_print = now
                print(f"  s={s_now:.2f}/{total_L:.2f} rem={remaining_end:.2f}m "
                      f"v_cmd={v_cmd:.2f} w_cmd={w_cmd:.2f}")

            # Keep pygame responsive
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    return


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

    orch = HybridOrchestrator(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=50,
        random_seed=41,
        initial_robot_pose=(2.0, 2.0, -math.pi/2),
        shovel_width=0.22,
        auto_mode=AUTO_MODE)
    
    orch.flow_field_vis_mode = FLOW_FIELD_VIS_3D
    
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()
