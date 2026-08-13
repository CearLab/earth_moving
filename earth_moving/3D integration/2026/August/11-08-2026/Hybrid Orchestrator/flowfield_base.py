"""
flowfield_base.py - Shared FlowField2D class for Hybrid Orchestrator

This module provides the base Dijkstra-based flow field used by both:
- ORCANavigator (for navigation to path start)
- PathTracker (as base class for PathGuidanceField2D)
"""

import math
import heapq
import numpy as np

try:
    import pybullet as p
    HAS_PYBULLET = True
except ImportError:
    HAS_PYBULLET = False


class FlowField2D:
    """
    Goal-based flow field on a 2D grid over the XY plane.

    - World region: [world_xmin, world_xmax] x [world_ymin, world_ymax]
    - Dijkstra from goal cell => distance field
    - Direction = -grad(distance), normalized
    - Obstacles are a boolean grid filled from pebble positions.
    """

    def __init__(self,
                 world_xmin=-1.2,
                 world_xmax=+1.2,
                 world_ymin=-1.2,
                 world_ymax=+1.2,
                 grid_w=41,
                 grid_h=41,
                 rover_radius=0.25,
                 pebble_radius=0.05,
                 clearance=0.02,
                 clearance_bias_gain=0.0):
        self.world_xmin = world_xmin
        self.world_xmax = world_xmax
        self.world_ymin = world_ymin
        self.world_ymax = world_ymax
        self.grid_w = grid_w
        self.grid_h = grid_h

        self.cell_w = (world_xmax - world_xmin) / grid_w
        self.cell_h = (world_ymax - world_ymin) / grid_h

        # Size assumptions (meters)
        self.rover_radius = rover_radius
        self.pebble_radius = pebble_radius
        self.clearance = clearance
        self.clearance_bias_gain = float(clearance_bias_gain)

        # Grids
        self.obstacles = np.zeros((grid_h, grid_w), dtype=bool)
        self.dist = np.full((grid_h, grid_w), np.inf, dtype=float)
        self.dir_field = np.zeros((grid_h, grid_w, 2), dtype=float)
        self.clearance_field = np.zeros((grid_h, grid_w), dtype=float)

        self.goal_cell = None

    # ------------ coordinate transforms ------------

    def world_to_cell(self, x, y):
        ix = int((x - self.world_xmin) / self.cell_w)
        iy = int((y - self.world_ymin) / self.cell_h)
        ix = max(0, min(self.grid_w - 1, ix))
        iy = max(0, min(self.grid_h - 1, iy))
        return ix, iy

    def cell_to_world_center(self, ix, iy):
        x = self.world_xmin + (ix + 0.5) * self.cell_w
        y = self.world_ymin + (iy + 0.5) * self.cell_h
        return x, y

    # ------------ obstacle stamping from pebbles ------------

    def clear_obstacles(self):
        self.obstacles[:, :] = False
        self.clearance_field[:, :] = 0.0

    def stamp_pebbles(self, pebble_centers):
        """
        pebble_centers: list of (x, y) in world coordinates.

        Mark as blocked any cell whose center is within:
            rover_radius + pebble_radius + clearance
        of a pebble center. That way the rover disc will not fit there.
        """
        gh, gw = self.grid_h, self.grid_w

        # Effective forbidden radius around each pebble
        R_block = self.rover_radius + self.pebble_radius + self.clearance

        for (px, py) in pebble_centers:
            # Skip pebbles outside the grid
            if not (self.world_xmin <= px <= self.world_xmax and
                    self.world_ymin <= py <= self.world_ymax):
                continue

            cx, cy = self.world_to_cell(px, py)

            # How many cells to check around the pebble
            max_cells_x = int(math.ceil(R_block / self.cell_w))
            max_cells_y = int(math.ceil(R_block / self.cell_h))

            for dy in range(-max_cells_y, max_cells_y + 1):
                for dx in range(-max_cells_x, max_cells_x + 1):
                    ix = cx + dx
                    iy = cy + dy
                    if ix < 0 or ix >= gw or iy < 0 or iy >= gh:
                        continue
                    wx, wy = self.cell_to_world_center(ix, iy)
                    if math.hypot(wx - px, wy - py) <= R_block:
                        self.obstacles[iy, ix] = True

    # ------------ Dijkstra distance field ------------

    def compute_distance_field(self, goal_cell):
        self.goal_cell = goal_cell
        gh, gw = self.grid_h, self.grid_w
        gx, gy = goal_cell

        self.dist[:, :] = np.inf

        if self.obstacles[gy, gx]:
            print("[FlowField2D] WARNING: goal is inside an obstacle cell.")
            return

        self.dist[gy, gx] = 0.0
        pq = []
        heapq.heappush(pq, (0.0, gx, gy))

        # 8-connected grid
        neighbor_offsets = [
            (-1, 0), (1, 0),
            (0, -1), (0, 1),
            (-1, -1), (-1, 1),
            (1, -1), (1, 1),
        ]

        while pq:
            d_cur, x, y = heapq.heappop(pq)
            if d_cur > self.dist[y, x] + 1e-9:
                continue

            for dx, dy in neighbor_offsets:
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= gw or ny < 0 or ny >= gh:
                    continue
                if self.obstacles[ny, nx]:
                    continue

                step = math.hypot(dx, dy)
                nd = d_cur + step
                if nd < self.dist[ny, nx]:
                    self.dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))

    # ------------ obstacle clearance / safety field ------------

    def compute_clearance_field(self):
        gh, gw = self.grid_h, self.grid_w
        self.clearance_field[:, :] = 0.0

        obstacle_cells = np.argwhere(self.obstacles)
        if len(obstacle_cells) == 0:
            max_span = math.hypot(self.world_xmax - self.world_xmin,
                                  self.world_ymax - self.world_ymin)
            self.clearance_field[:, :] = max_span
            return

        obstacle_points = np.array(
            [self.cell_to_world_center(int(ix), int(iy)) for iy, ix in obstacle_cells],
            dtype=float)
        cell_margin = 0.5 * math.hypot(self.cell_w, self.cell_h)

        for y in range(gh):
            for x in range(gw):
                if self.obstacles[y, x]:
                    continue

                wx, wy = self.cell_to_world_center(x, y)
                deltas = obstacle_points - np.array([wx, wy], dtype=float)
                dists = np.linalg.norm(deltas, axis=1)
                if len(dists) == 0:
                    continue
                self.clearance_field[y, x] = max(0.0, float(np.min(dists)) - cell_margin)

    # ------------ direction field: -grad(dist) ------------

    def compute_direction_field(self):
        gh, gw = self.grid_h, self.grid_w
        self.dir_field[:, :, :] = 0.0

        neighbor_offsets = [
            (-1, 0), (1, 0),
            (0, -1), (0, 1),
            (-1, -1), (-1, 1),
            (1, -1), (1, 1),
        ]

        for y in range(gh):
            for x in range(gw):
                if self.obstacles[y, x]:
                    continue
                d_cur = self.dist[y, x]
                if not math.isfinite(d_cur):
                    continue

                grad_x = 0.0
                grad_y = 0.0
                safe_x = 0.0
                safe_y = 0.0

                for dx, dy in neighbor_offsets:
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or nx >= gw or ny < 0 or ny >= gh:
                        continue
                    d_n = self.dist[ny, nx]
                    if not math.isfinite(d_n):
                        continue

                    diff = d_n - d_cur
                    grad_x += diff * dx
                    grad_y += diff * dy

                    c_n = self.clearance_field[ny, nx]
                    c_cur = self.clearance_field[y, x]
                    diff_safe = c_n - c_cur
                    safe_x += diff_safe * dx
                    safe_y += diff_safe * dy

                vx = -grad_x
                vy = -grad_y
                safe_mag = math.hypot(safe_x, safe_y)
                if self.clearance_bias_gain > 1e-9 and safe_mag > 1e-9:
                    vx += self.clearance_bias_gain * (safe_x / safe_mag)
                    vy += self.clearance_bias_gain * (safe_y / safe_mag)
                mag = math.hypot(vx, vy)
                if mag < 1e-6:
                    continue
                self.dir_field[y, x, 0] = vx / mag
                self.dir_field[y, x, 1] = vy / mag

    # ------------ public API ------------

    def rebuild(self, goal_world, pebble_centers):
        """
        goal_world: (gx, gy) in world coords
        pebble_centers: list[(x,y)] in world coords
        """
        self.clear_obstacles()
        self.stamp_pebbles(pebble_centers)
        gx_cell, gy_cell = self.world_to_cell(goal_world[0], goal_world[1])
        self.compute_distance_field((gx_cell, gy_cell))
        self.compute_clearance_field()
        self.compute_direction_field()

    def get_direction_world(self, x_world, y_world):
        # Bilinear interpolation avoids abrupt direction flips when the robot
        # crosses a cell boundary near obstacles or narrow passages.
        fx = (x_world - self.world_xmin) / self.cell_w - 0.5
        fy = (y_world - self.world_ymin) / self.cell_h - 0.5

        fx = min(max(fx, 0.0), self.grid_w - 1.0)
        fy = min(max(fy, 0.0), self.grid_h - 1.0)

        x0 = int(math.floor(fx))
        y0 = int(math.floor(fy))
        x1 = min(x0 + 1, self.grid_w - 1)
        y1 = min(y0 + 1, self.grid_h - 1)

        tx = fx - x0
        ty = fy - y0

        samples = [
            (x0, y0, (1.0 - tx) * (1.0 - ty)),
            (x1, y0, tx * (1.0 - ty)),
            (x0, y1, (1.0 - tx) * ty),
            (x1, y1, tx * ty),
        ]

        acc = np.zeros(2, dtype=float)
        total_weight = 0.0
        best_vec = np.zeros(2, dtype=float)
        best_weight = -1.0

        for ix, iy, weight in samples:
            vx = self.dir_field[iy, ix, 0]
            vy = self.dir_field[iy, ix, 1]
            mag = math.hypot(vx, vy)
            if mag < 1e-6:
                continue

            acc[0] += weight * vx
            acc[1] += weight * vy
            total_weight += weight

            if weight > best_weight:
                best_weight = weight
                best_vec[0] = vx
                best_vec[1] = vy

        if total_weight < 1e-9:
            return np.zeros(2, dtype=float)

        mag = math.hypot(acc[0], acc[1])
        if mag < 1e-6:
            return best_vec

        return acc / mag

    def draw_debug(self, scale=0.2, life_time=0.0):
        """Draw debug visualization in PyBullet."""
        if not HAS_PYBULLET:
            return

        gh, gw = self.grid_h, self.grid_w
        for y in range(gh):
            for x in range(gw):
                if self.obstacles[y, x]:
                    # Draw a small red cross on obstacle cells
                    cx, cy = self.cell_to_world_center(x, y)
                    p.addUserDebugLine([cx - 0.01, cy, 0.02],
                                       [cx + 0.01, cy, 0.02],
                                       [1, 0, 0],
                                       lifeTime=life_time)
                    p.addUserDebugLine([cx, cy - 0.01, 0.02],
                                       [cx, cy + 0.01, 0.02],
                                       [1, 0, 0],
                                       lifeTime=life_time)
                    continue
                vx = self.dir_field[y, x, 0]
                vy = self.dir_field[y, x, 1]
                if abs(vx) < 1e-3 and abs(vy) < 1e-3:
                    continue
                cx, cy = self.cell_to_world_center(x, y)
                start = [cx, cy, 0.02]
                end = [cx + vx * scale, cy + vy * scale, 0.02]
                p.addUserDebugLine(start, end, [0, 0, 1], lifeTime=life_time)


class FlowFieldController:
    """
    Simple flow-field controller for unicyclic (differential-drive) robots.

    Computes (v_cmd, w_cmd) to follow the flow field direction toward a goal.
    """

    def __init__(self,
                 v_max=0.8,
                 w_max=3.0,
                 k_theta=3.0,
                 turn_in_place_angle_deg=50.0,
                 static_speed_threshold=0.03,
                 w_turn_in_place=5.0,
                 stop_dist=0.15):
        """
        v_max: max forward speed
        w_max: max angular speed (for normal tracking)
        k_theta: heading P-gain
        turn_in_place_angle_deg: above this |heading error| (deg),
                                 and when nearly static, we spin in place.
        static_speed_threshold: |v_forward| below this is considered "static".
        w_turn_in_place: angular speed used for turn-in-place (rad/s).
        stop_dist: distance to goal under which we consider "reached".
        """
        self.v_max = v_max
        self.w_max = w_max
        self.k_theta = k_theta
        self.turn_in_place_angle = math.radians(turn_in_place_angle_deg)
        self.static_speed_threshold = static_speed_threshold
        self.w_turn_in_place = w_turn_in_place
        self.stop_dist = stop_dist

    def compute_control(self, state, goal_world, flow_field):
        """
        Compute control commands based on flow field.

        Args:
            state: [x, y, yaw, v_fwd, w]
            goal_world: (gx, gy) in world coordinates
            flow_field: FlowField2D instance

        Returns:
            (v_cmd, w_cmd) tuple
        """
        x, y, yaw, v_fwd, w = state
        gx, gy = goal_world

        dg = np.array([gx - x, gy - y], dtype=float)
        dist_goal = np.linalg.norm(dg)

        # Stop near the goal
        if dist_goal < self.stop_dist:
            return 0.0, 0.0

        # Preferred direction from flow field
        v_dir = flow_field.get_direction_world(x, y)
        mag_dir = np.linalg.norm(v_dir)
        if mag_dir < 1e-3:
            # "dead zone" in the field: go directly toward goal
            v_dir = dg / (dist_goal + 1e-9)
        else:
            v_dir /= mag_dir

        theta_des = math.atan2(v_dir[1], v_dir[0])
        e_theta = wrap_angle(theta_des - yaw)

        # Turn-in-place logic
        if abs(v_fwd) < self.static_speed_threshold and abs(e_theta) > self.turn_in_place_angle:
            w_cmd = math.copysign(self.w_turn_in_place, e_theta)
            return 0.0, w_cmd

        # Normal steering
        w_cmd = self.k_theta * e_theta
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))

        align = max(0.0, math.cos(e_theta))  # 1 when aligned, 0 when opposite
        v_base = self.v_max * align
        dist_factor = min(1.0, dist_goal / 0.4)  # slow near goal
        v_cmd = v_base * dist_factor

        return float(v_cmd), float(w_cmd)


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def estimate_eta_along_field(flow_field,
                             start_world,
                             goal_world,
                             v_trans=0.8,
                             w_turn=5.0,
                             step_fraction=0.7,
                             max_steps=8000,
                             goal_radius=0.08):
    """
    Integrate along flow-field streamlines to estimate time from start_world -> goal_world.

    Returns TWO times:
      - t_lower: optimistic (translation only, assuming always aligned)
      - t_upper: conservative (translation + "turn in place" time for direction changes)

    Args:
        flow_field: FlowField2D instance
        start_world: (sx, sy) start position
        goal_world: (gx, gy) goal position
        v_trans: nominal translational speed
        w_turn: angular speed for turn-in-place
        step_fraction: fraction of cell size for integration step
        max_steps: maximum integration steps
        goal_radius: arrival threshold

    Returns:
        (t_lower, t_upper) or (None, None) if unreachable
    """
    sx, sy = start_world
    gx, gy = goal_world

    x = float(sx)
    y = float(sy)

    # Step length along the path
    h = step_fraction * min(flow_field.cell_w, flow_field.cell_h)

    t_lower = 0.0  # translational time only
    t_upper = 0.0  # translational + turning time
    prev_dir = None

    for _ in range(max_steps):
        dxg = gx - x
        dyg = gy - y
        dist_goal = math.hypot(dxg, dyg)
        if dist_goal <= goal_radius:
            return t_lower, t_upper

        # Preferred direction from field
        v_dir = flow_field.get_direction_world(x, y)
        mag = float(np.linalg.norm(v_dir))

        if mag < 1e-6:
            # Dead zone in the field: fall back to straight-to-goal
            if dist_goal < 1e-6:
                return t_lower, t_upper
            v_dir = np.array([dxg / dist_goal, dyg / dist_goal], dtype=float)
        else:
            v_dir /= mag

        # Direction change vs previous
        if prev_dir is None:
            dtheta = 0.0
        else:
            dot = float(np.clip(prev_dir.dot(v_dir), -1.0, 1.0))
            dtheta = math.acos(dot)

        # Translational time
        v_trans_eff = max(v_trans, 1e-3)
        dt_trans = h / v_trans_eff

        # Turning time upper bound
        w_turn_eff = max(w_turn, 1e-3)
        dt_turn = abs(dtheta) / w_turn_eff

        t_lower += dt_trans
        t_upper += (dt_trans + dt_turn)

        # Euler step along direction
        x += v_dir[0] * h
        y += v_dir[1] * h
        prev_dir = v_dir

    # Did not reach goal within max_steps
    return None, None
