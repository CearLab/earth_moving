"""
orca_navigator.py - ORCA Navigation for Hybrid Orchestrator

This module provides:
- FlowFieldORCAPlanner: ORCA planner with flow-field preferred velocity
- Sailing-based priority for multi-agent coordination
- Collision avoidance while navigating to path start
"""

import collections
import math
import numpy as np

from flowfield_base import FlowField2D, wrap_angle


# =========================================================
#                 GLOBAL SETTINGS
# =========================================================

# ORCA neighbor radius (meters)
NEAR_RADIUS = 4.0

# Radius for building conflict clusters
CLUSTER_RADIUS = 2.5

# Global "wind" direction for sailing-based priority (radians)
WIND_DIR = math.radians(60.0)

NAV_FIELD_CONFIG = {
    "cell_size": 0.12,
    "rover_radius": 0.18,
    "clearance": 0.0,
    "clearance_bias_gain": 0.35,
}

PHASE1_CONFIG = {
    "dead_zone_reach_tol": 0.08,
    "dead_zone_heading_tol_deg": 6.0,
    "dead_zone_v_max": 0.55,
    "dead_zone_w_max": 7.5,
    "required_safe_layers": 1,
    "preferred_clearance_layers": 3,
    "dead_zone_preferred_clearance": 0.24,
    "dead_zone_min_clearance": 0.10,
    "dead_zone_stuck_window": 1.0,
    "dead_zone_stuck_bbox_size": 0.16,
    "dead_zone_stuck_min_progress": 0.08,
    "dead_zone_stuck_min_forward_speed": 0.10,
    "dead_zone_dwell_trigger": 0.80,
}

SHOVEL_TRACK_OFFSET = 0.17


# =========================================================
#          BASE ORCA PLANNER
# =========================================================

class ORCAPlanner:
    """
    ORCA-style planner using sampling + TTC collision checking.
    Supports priority and blind agent awareness.
    """

    def __init__(self,
                 tau=5.0,
                 v_pref=1.0,
                 v_max=1.6,
                 w_max=6.0,
                 k_theta=6.0,
                 turn_in_place_deg=50.0,
                 n_speed=6,
                 n_angle=24):
        """
        Initialize ORCA planner.

        Args:
            tau: collision horizon (seconds)
            v_pref: preferred speed toward goal
            v_max: maximum translational speed
            w_max: maximum angular speed
            k_theta: heading P-gain
            turn_in_place_deg: angle threshold for turn-in-place
            n_speed: radial samples in velocity space
            n_angle: angular samples in velocity space
        """
        self.tau = tau
        self.v_pref = v_pref
        self.v_max = v_max
        self.w_max = w_max
        self.k_theta = k_theta
        self.turn_in_place = math.radians(turn_in_place_deg)
        self.n_speed = n_speed
        self.n_angle = n_angle

    @staticmethod
    def _shape_radius(shape):
        """Approximate rover shape as a single disc."""
        return max(math.hypot(cx, cy) + r for (cx, cy, r) in shape["circles"])

    @staticmethod
    def _ttc_collision(p_ij, v_rel, r_sum, tau):
        """
        Check if there will be a collision within tau seconds.

        Args:
            p_ij: relative position (neighbor - self)
            v_rel: relative velocity (neighbor velocity - candidate velocity)
            r_sum: sum of radii
            tau: time horizon

        Returns:
            True if collision predicted
        """
        r_eff = r_sum * 1.05  # small safety margin

        dist0 = np.linalg.norm(p_ij)
        if dist0 < r_eff:
            return True  # already overlapping

        v2 = np.dot(v_rel, v_rel)
        if v2 < 1e-8:
            return False  # no relative motion

        t_star = -np.dot(p_ij, v_rel) / v2

        if t_star < 0.0:
            d_min = dist0
        elif t_star > tau:
            d_min = np.linalg.norm(p_ij + v_rel * tau)
        else:
            d_min = np.linalg.norm(p_ij + v_rel * t_star)

        return d_min < r_eff

    def _project_to_unicycle(self, ego, v_new, is_fallback_mode=False, dt=0.05, dist_to_goal=None):
        """
        Map 2D velocity vector to (v, w) commands for differential drive.
        """
        px, py, th, v_fwd, w = ego["state"]
        speed = np.linalg.norm(v_new)
        if speed < 1e-4:
            return 0.0, 0.0

        heading = math.atan2(v_new[1], v_new[0])
        e_th = wrap_angle(heading - th)

        if is_fallback_mode:
            heading_tol = math.radians(PHASE1_CONFIG["dead_zone_heading_tol_deg"])
            if abs(e_th) > heading_tol:
                w_cmd = math.copysign(
                    min(
                        PHASE1_CONFIG["dead_zone_w_max"],
                        max(3.0, 10.0 * abs(e_th)),
                    ),
                    e_th,
                )
                return 0.0, float(w_cmd)

        align = max(0.0, math.cos(e_th))
        
        # If we are in fallback mode and not fully aligned, slow down the base speed aggressively
        # but don't hard-cutoff to 0.0, to prevent pulsing.
        base_v_max = min(speed, self.v_max)
        if is_fallback_mode and abs(e_th) > math.radians(10):
            base_v_max *= 0.5
            
        # Tuned for curves: penalize v_cmd heavily if not aligned to allow w_cmd to dominate
        dist_factor = 1.0
        if dist_to_goal is not None:
             dist_factor = min(1.0, dist_to_goal / 0.4)
             
        v_cmd = base_v_max * (align ** 4) * dist_factor

        if is_fallback_mode:
            k_d = 2.0
            w_cmd = self.k_theta * e_th - k_d * w
        else:
            w_cmd = self.k_theta * e_th

        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))
        
        # PREVENT DISCRETE-TIME OVERSHOOT
        w_cmd = math.copysign(min(abs(w_cmd), abs(e_th) / dt), w_cmd)

        return v_cmd, w_cmd

    def _sample_candidates(self, v_pref_vec):
        """Sample velocities in a disc of radius v_max."""
        cand = [np.array([0.0, 0.0], dtype=float), v_pref_vec]
        speeds = np.linspace(0.0, self.v_max, self.n_speed)
        angles = np.linspace(-math.pi, math.pi, self.n_angle, endpoint=False)
        for s in speeds:
            for ang in angles:
                vx = s * math.cos(ang)
                vy = s * math.sin(ang)
                cand.append(np.array([vx, vy], dtype=float))
        return cand

    def plan(self, ego, goal, neighbors, shape):
        """
        Plan collision-free velocity.

        Args:
            ego: agent dict with "state", "priority"
            goal: np.array([gx, gy])
            neighbors: list of neighbor dicts
            shape: ego["shape"]

        Returns:
            (v_cmd, w_cmd) tuple
        """
        px, py, th, v_fwd, w = ego["state"]
        p_i = np.array([px, py], dtype=float)
        g = np.array(goal, dtype=float)
        dir_vec = g - p_i
        dist_goal = np.linalg.norm(dir_vec)

        if dist_goal > 1e-6:
            dir_unit = dir_vec / dist_goal
        else:
            dir_unit = np.zeros(2)

        # Preferred velocity towards goal (slow down near goal)
        s_pref = self.v_pref
        if dist_goal < 0.5:
            s_pref *= dist_goal / 0.5
        s_pref = min(self.v_max, max(0.0, s_pref))
        v_pref_vec = s_pref * dir_unit

        if not neighbors:
            v_cmd, w_cmd = self._project_to_unicycle(ego, v_pref_vec, dist_to_goal=dist_goal)
            return float(v_cmd), float(w_cmd)

        # Precompute neighbor info for TTC checks
        r_i = self._shape_radius(shape)
        pr_i = ego.get("priority", 0.0)

        neigh_info = []
        for nb in neighbors:
            pxj, pyj, thj, vj_fwd, wj = nb["state"]
            p_j = np.array([pxj, pyj], dtype=float)
            vx_j = math.cos(thj) * vj_fwd
            vy_j = math.sin(thj) * vj_fwd
            v_j = np.array([vx_j, vy_j], dtype=float)

            r_j = self._shape_radius(nb["shape"])
            r_sum = r_i + r_j

            pr_j = nb.get("priority", 0.0)
            is_blind = nb.get("is_blind", False)

            # Smaller numeric priority == higher right-of-way
            if is_blind or pr_j < pr_i:
                must_avoid = True
            elif pr_j > pr_i:
                must_avoid = False
            else:
                must_avoid = True  # symmetric case

            neigh_info.append({
                "p_ij": p_j - p_i,
                "v_j": v_j,
                "r_sum": r_sum,
                "must_avoid": must_avoid,
            })

        candidates = self._sample_candidates(v_pref_vec)

        best_v = v_pref_vec
        best_cost = float("inf")

        for v in candidates:
            if np.linalg.norm(v) > self.v_max + 1e-6:
                continue

            colliding = False
            for info in neigh_info:
                if not info["must_avoid"]:
                    continue
                p_ij = info["p_ij"]
                v_rel = info["v_j"] - v
                if self._ttc_collision(p_ij, v_rel, info["r_sum"], self.tau):
                    colliding = True
                    break

            if colliding:
                continue

            cost = np.linalg.norm(v - v_pref_vec)
            if cost < best_cost:
                best_cost = cost
                best_v = v

        if best_cost == float("inf"):
            best_v = np.zeros(2, dtype=float)

        v_cmd, w_cmd = self._project_to_unicycle(ego, best_v, dist_to_goal=dist_goal)
        return float(v_cmd), float(w_cmd)


# =========================================================
#          ORCA WITH FLOW FIELD PREFERRED VELOCITY
# =========================================================

class FlowFieldORCAPlanner(ORCAPlanner):
    """
    ORCA planner that uses an externally supplied preferred velocity
    vector v_pref_vec (world frame) from a flow field.
    """

    def _plan_core(self, ego, p_i, neighbors, shape, v_pref_vec, is_fallback_mode=False, dt=0.05, dist_to_goal=None):
        if not neighbors:
            v_cmd, w_cmd = self._project_to_unicycle(ego, v_pref_vec, is_fallback_mode, dt, dist_to_goal)
            return float(v_cmd), float(w_cmd)

        r_i = self._shape_radius(shape)
        pr_i = ego.get("priority", 0.0)

        neigh_info = []
        for nb in neighbors:
            pxj, pyj, thj, vj_fwd, wj = nb["state"]
            p_j = np.array([pxj, pyj], dtype=float)

            vx_j = math.cos(thj) * vj_fwd
            vy_j = math.sin(thj) * vj_fwd
            v_j = np.array([vx_j, vy_j], dtype=float)

            r_j = self._shape_radius(nb["shape"])
            r_sum = r_i + r_j

            pr_j = nb.get("priority", 0.0)
            is_blind = nb.get("is_blind", False)
            force_avoid = nb.get("force_avoid", False)

            if force_avoid or is_blind or pr_j < pr_i:
                must_avoid = True
            elif pr_j > pr_i:
                must_avoid = False
            else:
                must_avoid = True

            neigh_info.append({
                "p_ij": p_j - p_i,
                "v_j": v_j,
                "r_sum": r_sum,
                "must_avoid": must_avoid,
            })

        candidates = self._sample_candidates(v_pref_vec)

        best_v = v_pref_vec
        best_cost = float("inf")

        for v in candidates:
            if np.linalg.norm(v) > self.v_max + 1e-6:
                continue

            colliding = False
            for info in neigh_info:
                if not info["must_avoid"]:
                    continue
                p_ij = info["p_ij"]
                v_rel = info["v_j"] - v
                if self._ttc_collision(p_ij, v_rel, info["r_sum"], self.tau):
                    colliding = True
                    break

            if colliding:
                continue

            cost = np.linalg.norm(v - v_pref_vec)
            if cost < best_cost:
                best_cost = cost
                best_v = v

        if best_cost == float("inf"):
            best_v = np.zeros(2, dtype=float)

        v_cmd, w_cmd = self._project_to_unicycle(ego, best_v, is_fallback_mode, dt, dist_to_goal)
        return float(v_cmd), float(w_cmd)

    def plan_with_pref(self, ego, v_pref_vec, neighbors, shape, is_fallback_mode=False, dt=0.05, dist_to_goal=None):
        """Plan using externally supplied preferred velocity."""
        px, py, th, v_fwd, w = ego["state"]
        p_i = np.array([px, py], dtype=float)
        return self._plan_core(ego, p_i, neighbors, shape, v_pref_vec, is_fallback_mode, dt, dist_to_goal)


# =========================================================
#                 SAILING PRIORITY
# =========================================================

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def sailing_priority(yaw, wind_dir=WIND_DIR):
    """
    Compute priority based on heading vs global 'wind direction'.
    Smaller number => higher right-of-way.
    """
    rel = wrap_to_pi(yaw - wind_dir)
    return 0.5 - 0.5 * math.sin(rel)


# =========================================================
#                 ORCA NAVIGATOR
# =========================================================

class ORCANavigator:
    """
    High-level ORCA navigation controller.

    Uses FlowField2D for preferred direction and FlowFieldORCAPlanner
    for collision avoidance when navigating to a goal position.
    """

    def __init__(self,
                 world_bounds=(-3.5, 3.5, -3.5, 3.5),
                 grid_size=(81, 81),
                 tau=5.0,
                 v_max=1.5,
                 rover_radius=0.18,
                 pebble_radius=0.05,
                 arrival_threshold=0.15,
                 target_zone_radius=0.3,
                 phase1_tracking_point="shovel",
                 shovel_offset=SHOVEL_TRACK_OFFSET):
        """
        Initialize ORCA navigator.

        Args:
            world_bounds: (xmin, xmax, ymin, ymax)
            grid_size: (grid_w, grid_h)
            tau: ORCA time horizon
            v_max: maximum velocity
            rover_radius: rover safety radius
            pebble_radius: pebble radius
            arrival_threshold: distance to goal considered "arrived"
        """
        self.world_bounds = world_bounds
        self.grid_size = grid_size
        self.tau = tau
        self.v_max = v_max
        self.rover_radius = rover_radius
        self.pebble_radius = pebble_radius
        self.arrival_threshold = arrival_threshold
        self.target_zone_radius = float(target_zone_radius)
        tracking_point = str(phase1_tracking_point).strip().lower()
        if tracking_point not in ("base", "shovel"):
            raise ValueError("phase1_tracking_point must be 'base' or 'shovel'")
        self.phase1_tracking_point = tracking_point
        self.shovel_offset = float(shovel_offset)

        # State
        self.field = None
        self.planner = None
        self.goal = None
        self.active_fallback_cell = None
        self.last_flow_dir = None
        self.progress_history = collections.deque()
        self.nav_elapsed = 0.0
        self.dead_zone_dwell = 0.0
        self.target_zone_block_radius = self.target_zone_radius + 0.1
        self.target_zone_guard_radius = self.target_zone_block_radius + 0.05

    def _reset_recovery_state(self):
        self.active_fallback_cell = None
        self.last_flow_dir = None
        self.progress_history.clear()
        self.nav_elapsed = 0.0
        self.dead_zone_dwell = 0.0

    def get_tracking_position(self, state):
        x_b = float(state[0])
        y_b = float(state[1])
        yaw_b = float(state[2])
        if self.phase1_tracking_point == "base":
            return x_b, y_b
        return (
            x_b + self.shovel_offset * math.cos(yaw_b),
            y_b + self.shovel_offset * math.sin(yaw_b),
        )

    def _cell_has_flow_index(self, ix, iy):
        if ix < 0 or ix >= self.field.grid_w or iy < 0 or iy >= self.field.grid_h:
            return False
        dvx = self.field.dir_field[iy, ix, 0]
        dvy = self.field.dir_field[iy, ix, 1]
        return abs(dvx) > 1e-3 or abs(dvy) > 1e-3

    def _current_cell_has_flow(self, x_pos, y_pos):
        cx, cy = self.field.world_to_cell(x_pos, y_pos)
        return self._cell_has_flow_index(cx, cy)

    def _cell_clearance_m(self, ix, iy):
        if ix < 0 or ix >= self.field.grid_w or iy < 0 or iy >= self.field.grid_h:
            return 0.0
        return float(self.field.clearance_field[iy, ix])

    def _valid_flow_clearance_layers(self, ix, iy, max_layers):
        best_layers = 0
        for layers in range(1, max_layers + 1):
            ok = True
            for dy in range(-layers, layers + 1):
                for dx in range(-layers, layers + 1):
                    nx, ny = ix + dx, iy + dy
                    if not self._cell_has_flow_index(nx, ny):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                best_layers = layers
            else:
                break
        return best_layers

    def _line_crosses_target_zone(self, x0, y0, x1, y1, radius):
        dx = x1 - x0
        dy = y1 - y0
        a = dx * dx + dy * dy
        if a < 1e-9:
            return math.hypot(x0, y0) <= radius
        b = 2.0 * (x0 * dx + y0 * dy)
        c = (x0 * x0 + y0 * y0) - radius * radius
        delta = b * b - 4.0 * a * c
        if delta < 0.0:
            return False
        sqrt_delta = math.sqrt(delta)
        t1 = (-b - sqrt_delta) / (2.0 * a)
        t2 = (-b + sqrt_delta) / (2.0 * a)
        return (0.0 <= t1 <= 1.0) or (0.0 <= t2 <= 1.0)

    def _fallback_vector_from_cell(self, target_cell, x_pos, y_pos):
        tx, ty = self.field.cell_to_world_center(target_cell[0], target_cell[1])
        vx = tx - x_pos
        vy = ty - y_pos
        if math.hypot(vx, vy) < 0.35:
            dvx = self.field.dir_field[target_cell[1], target_cell[0], 0]
            dvy = self.field.dir_field[target_cell[1], target_cell[0], 1]
            if abs(dvx) > 1e-3 or abs(dvy) > 1e-3:
                lookahead = 0.6
                vx = (tx + dvx * lookahead) - x_pos
                vy = (ty + dvy * lookahead) - y_pos
        return vx, vy

    def _find_closest_valid_flow_cell(self, x_pos, y_pos):
        cx, cy = self.field.world_to_cell(x_pos, y_pos)
        visited = {(cx, cy)}
        bfs_q = collections.deque([(cx, cy)])
        max_search_steps = self.field.grid_w * self.field.grid_h
        required_safe_layers = PHASE1_CONFIG["required_safe_layers"]
        preferred_layers = PHASE1_CONFIG["preferred_clearance_layers"]
        preferred_clearance = PHASE1_CONFIG["dead_zone_preferred_clearance"]
        min_clearance = PHASE1_CONFIG["dead_zone_min_clearance"]
        gx, gy = self.goal
        best_candidate = None
        best_priority = None
        steps_count = 0

        while bfs_q and steps_count < max_search_steps:
            cur_x, cur_y = bfs_q.popleft()
            steps_count += 1

            if self._cell_has_flow_index(cur_x, cur_y):
                tx, ty = self.field.cell_to_world_center(cur_x, cur_y)
                clearance_layers = self._valid_flow_clearance_layers(cur_x, cur_y, preferred_layers)
                clearance_m = self._cell_clearance_m(cur_x, cur_y)
                crosses_zone = self._line_crosses_target_zone(
                    x_pos, y_pos, tx, ty, self.target_zone_block_radius
                )
                world_dist = math.hypot(tx - x_pos, ty - y_pos)
                goal_dist = math.hypot(gx - tx, gy - ty)

                if clearance_m >= preferred_clearance:
                    safety_rank = 0
                elif clearance_m >= min_clearance:
                    safety_rank = 1
                else:
                    safety_rank = 2

                if clearance_layers >= required_safe_layers:
                    layer_rank = 0
                elif clearance_layers > 0:
                    layer_rank = 1
                else:
                    layer_rank = 2

                priority = (
                    1 if crosses_zone else 0,
                    layer_rank,
                    safety_rank,
                    world_dist,
                    goal_dist,
                )
                if best_priority is None or priority < best_priority:
                    best_priority = priority
                    best_candidate = (cur_x, cur_y)

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = cur_x + dx, cur_y + dy
                if (nx, ny) not in visited and 0 <= nx < self.field.grid_w and 0 <= ny < self.field.grid_h:
                    visited.add((nx, ny))
                    bfs_q.append((nx, ny))

        return best_candidate

    def build_field(self, goal_world, pebble_centers):
        """
        Build flow field for navigation to goal.

        Args:
            goal_world: (x, y) goal position
            pebble_centers: list of (x, y) pebble positions
        """
        self.goal = np.array(goal_world, dtype=float)
        self._reset_recovery_state()

        # Build flow field
        xmin, xmax, ymin, ymax = self.world_bounds
        grid_w = int(math.ceil((xmax - xmin) / NAV_FIELD_CONFIG["cell_size"]))
        grid_h = int(math.ceil((ymax - ymin) / NAV_FIELD_CONFIG["cell_size"]))
        if grid_w % 2 == 0:
            grid_w += 1
        if grid_h % 2 == 0:
            grid_h += 1

        self.field = FlowField2D(
            world_xmin=xmin,
            world_xmax=xmax,
            world_ymin=ymin,
            world_ymax=ymax,
            grid_w=grid_w,
            grid_h=grid_h,
            rover_radius=NAV_FIELD_CONFIG["rover_radius"],
            pebble_radius=self.pebble_radius,
            clearance=NAV_FIELD_CONFIG["clearance"],
            clearance_bias_gain=NAV_FIELD_CONFIG["clearance_bias_gain"],
        )

        gx_cell, gy_cell = self.field.world_to_cell(goal_world[0], goal_world[1])
        self.field.clear_obstacles()
        self.field.stamp_pebbles(pebble_centers)

        c0x, c0y = self.field.world_to_cell(0.0, 0.0)
        steps_x = int(math.ceil(self.target_zone_block_radius / self.field.cell_w))
        steps_y = int(math.ceil(self.target_zone_block_radius / self.field.cell_h))
        for dy in range(-steps_y, steps_y + 1):
            for dx in range(-steps_x, steps_x + 1):
                ix, iy = c0x + dx, c0y + dy
                if 0 <= ix < self.field.grid_w and 0 <= iy < self.field.grid_h:
                    wx, wy = self.field.cell_to_world_center(ix, iy)
                    if math.hypot(wx, wy) <= self.target_zone_block_radius:
                        self.field.obstacles[iy, ix] = True

        self.field.compute_distance_field((gx_cell, gy_cell))
        self.field.compute_clearance_field()
        self.field.compute_direction_field()

        # Build planner
        self.planner = FlowFieldORCAPlanner(
            tau=self.tau,
            v_pref=1.0,
            v_max=self.v_max,
            w_max=10.0,
            k_theta=10.0,
            turn_in_place_deg=50.0,
            n_speed=6,
            n_angle=24,
        )

    def get_preferred_velocity(self, state, dt=0.05):
        """
        Compute flow-field preferred velocity at current position.

        Args:
            state: [x, y, yaw, v_fwd, w]

        Returns:
            tuple: (np.array([vx, vy]) preferred velocity, bool is_fallback_mode)
        """
        if self.field is None or self.goal is None:
            return np.zeros(2, dtype=float), False

        x, y, th, v_fwd, w = state
        x_nav, y_nav = self.get_tracking_position(state)
        gx, gy = self.goal

        dg = np.array([gx - x_nav, gy - y_nav], dtype=float)
        dist_goal = np.linalg.norm(dg)
        self.nav_elapsed += dt
        self.progress_history.append((self.nav_elapsed, x_nav, y_nav, dist_goal))
        while self.progress_history and self.nav_elapsed - self.progress_history[0][0] > PHASE1_CONFIG["dead_zone_stuck_window"]:
            self.progress_history.popleft()

        has_flow_here = self._current_cell_has_flow(x_nav, y_nav)
        if has_flow_here:
            flow_vec = self.field.get_direction_world(x_nav, y_nav)
            flow_mag = math.hypot(flow_vec[0], flow_vec[1])
            if flow_mag > 1e-6:
                self.last_flow_dir = (flow_vec[0] / flow_mag, flow_vec[1] / flow_mag)
            self.dead_zone_dwell = 0.0
        else:
            self.dead_zone_dwell += dt

        progress_delta = 0.0
        span_x = 0.0
        span_y = 0.0
        if self.progress_history:
            progress_delta = self.progress_history[0][3] - dist_goal
            xs = [pt[1] for pt in self.progress_history]
            ys = [pt[2] for pt in self.progress_history]
            span_x = max(xs) - min(xs)
            span_y = max(ys) - min(ys)

        is_stuck_in_dead_zone = (
            not has_flow_here and
            self.dead_zone_dwell >= PHASE1_CONFIG["dead_zone_dwell_trigger"] and
            span_x <= PHASE1_CONFIG["dead_zone_stuck_bbox_size"] and
            span_y <= PHASE1_CONFIG["dead_zone_stuck_bbox_size"] and
            progress_delta <= PHASE1_CONFIG["dead_zone_stuck_min_progress"] and
            abs(v_fwd) <= PHASE1_CONFIG["dead_zone_stuck_min_forward_speed"]
        )

        if has_flow_here and self.active_fallback_cell is not None:
            safe_layers_here = self._valid_flow_clearance_layers(
                *self.field.world_to_cell(x_nav, y_nav),
                PHASE1_CONFIG["preferred_clearance_layers"],
            )
            if safe_layers_here >= PHASE1_CONFIG["required_safe_layers"]:
                self.active_fallback_cell = None

        if is_stuck_in_dead_zone and self.active_fallback_cell is None:
            self.active_fallback_cell = self._find_closest_valid_flow_cell(x_nav, y_nav)

        is_fallback_mode = False
        speed_pref = self.v_max * min(1.0, dist_goal / 0.4)

        if self.active_fallback_cell is not None:
            tx, ty = self.field.cell_to_world_center(*self.active_fallback_cell)
            dist_to_fallback = math.hypot(tx - x_nav, ty - y_nav)
            if dist_to_fallback < PHASE1_CONFIG["dead_zone_reach_tol"]:
                vx, vy = self._fallback_vector_from_cell(self.active_fallback_cell, x_nav, y_nav)
                if math.hypot(vx, vy) < 1e-6:
                    vx = tx - x_nav
                    vy = ty - y_nav
            else:
                vx = tx - x_nav
                vy = ty - y_nav
            speed_pref = min(
                PHASE1_CONFIG["dead_zone_v_max"],
                max(0.18, 1.2 * math.hypot(vx, vy)),
            )
            is_fallback_mode = True
        elif has_flow_here:
            vx, vy = self.field.get_direction_world(x_nav, y_nav)
        elif self.last_flow_dir is not None:
            vx, vy = self.last_flow_dir
        else:
            vx = gx - x_nav
            vy = gy - y_nav

        radial_dist = math.hypot(x_nav, y_nav)
        if radial_dist < self.target_zone_guard_radius:
            if radial_dist > 1e-6:
                vx = x_nav / radial_dist
                vy = y_nav / radial_dist
            else:
                vx = math.cos(th)
                vy = math.sin(th)
            speed_pref = min(self.v_max, max(0.2, self.v_max * min(1.0, dist_goal / 0.4)))
            is_fallback_mode = True

        mag_dir = math.hypot(vx, vy)
        if mag_dir > 1e-6:
            d_pref = np.array([vx / mag_dir, vy / mag_dir], dtype=float)
        else:
            d_pref = np.zeros(2, dtype=float)

        return speed_pref * d_pref, is_fallback_mode

    def compute_control(self, agent, other_agents=None, dt=0.05):
        """
        Compute control commands with ORCA collision avoidance.

        Args:
            agent: agent dict with "state", "shape", "priority"
            other_agents: list of other agent dicts (for multi-agent)

        Returns:
            dict with:
                - v_cmd, w_cmd: control commands
                - dist_to_goal: distance to goal
                - arrived: True if at goal
        """
        if self.field is None or self.planner is None or self.goal is None:
            return {
                "v_cmd": 0.0,
                "w_cmd": 0.0,
                "dist_to_goal": float("inf"),
                "arrived": False,
            }

        state = agent["state"]
        x, y = state[0], state[1]
        x_nav, y_nav = self.get_tracking_position(state)
        dist_to_goal = np.linalg.norm(self.goal - np.array([x_nav, y_nav]))

        # Check if arrived
        if dist_to_goal < self.arrival_threshold:
            return {
                "v_cmd": 0.0,
                "w_cmd": 0.0,
                "dist_to_goal": dist_to_goal,
                "arrived": True,
            }

        # Compute preferred velocity from flow field
        v_pref_vec, is_fallback_mode = self.get_preferred_velocity(state, dt=dt)

        # Build neighbors list
        neighbors = []
        if other_agents:
            for other in other_agents:
                dx = other["state"][0] - x
                dy = other["state"][1] - y
                if dx * dx + dy * dy < NEAR_RADIUS ** 2:
                    neighbors.append({
                        "id": other.get("id", "unknown"),
                        "priority": other.get("priority", 0.5),
                        "is_blind": other.get("is_blind", False),
                        "force_avoid": other.get("force_avoid", False),
                        "state": other["state"].copy(),
                        "shape": other["shape"],
                    })

        # Compute ORCA control
        v_cmd, w_cmd = self.planner.plan_with_pref(
            ego=agent,
            v_pref_vec=v_pref_vec,
            neighbors=neighbors,
            shape=agent["shape"],
            is_fallback_mode=is_fallback_mode,
            dt=dt,
            dist_to_goal=dist_to_goal
        )

        return {
            "v_cmd": v_cmd,
            "w_cmd": w_cmd,
            "dist_to_goal": dist_to_goal,
            "arrived": False,
        }

    def draw_debug(self, scale=0.2, life_time=0.0):
        """Draw debug visualization in PyBullet."""
        if self.field is not None:
            self.field.draw_debug(scale=scale, life_time=life_time)
