"""
orca_navigator.py - ORCA Navigation for Hybrid Orchestrator

This module provides:
- FlowFieldORCAPlanner: ORCA planner with flow-field preferred velocity
- Sailing-based priority for multi-agent coordination
- Collision avoidance while navigating to path start
"""

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
                 v_max=1.6,
                 rover_radius=0.15,
                 pebble_radius=0.05,
                 arrival_threshold=0.2):
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

        # State
        self.field = None
        self.planner = None
        self.goal = None
        self.active_fallback_cell = None

    def build_field(self, goal_world, pebble_centers):
        """
        Build flow field for navigation to goal.

        Args:
            goal_world: (x, y) goal position
            pebble_centers: list of (x, y) pebble positions
        """
        self.goal = np.array(goal_world, dtype=float)

        # Build flow field
        xmin, xmax, ymin, ymax = self.world_bounds
        self.field = FlowField2D(
            world_xmin=xmin,
            world_xmax=xmax,
            world_ymin=ymin,
            world_ymax=ymax,
            grid_w=self.grid_size[0],
            grid_h=self.grid_size[1],
            rover_radius=self.rover_radius,
            pebble_radius=self.pebble_radius,
            clearance=0.01,
        )
        self.field.rebuild(goal_world, pebble_centers)

        # Build planner
        self.planner = FlowFieldORCAPlanner(
            tau=self.tau,
            v_pref=1.0,
            v_max=self.v_max,
            w_max=6.0,
            k_theta=6.0,
            turn_in_place_deg=50.0,
            n_speed=6,
            n_angle=24,
        )

    def get_preferred_velocity(self, state):
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
        gx, gy = self.goal

        dg = np.array([gx - x, gy - y], dtype=float)
        dist_goal = np.linalg.norm(dg)

        vx, vy = self.field.get_direction_world(x, y)
        mag_dir = math.hypot(vx, vy)
        
        is_fallback_mode = False

        if mag_dir < 1e-3 or self.active_fallback_cell is not None:
            is_fallback_mode = True
            if self.active_fallback_cell is not None:
                tx_check, ty_check = self.field.cell_to_world_center(*self.active_fallback_cell)
                if math.hypot(tx_check - x, ty_check - y) < 0.2:
                    self.active_fallback_cell = None
                    
            cx, cy = self.field.world_to_cell(x, y)
            found_valid = False
            target_valid_cell = None
            
            if self.active_fallback_cell is not None:
                target_valid_cell = self.active_fallback_cell
                found_valid = True
            else:
                import collections
                bfs_q = collections.deque([(cx, cy)])
                visited = set([(cx, cy)])
                steps_count = 0
                max_search_steps = 100
                
                while bfs_q and steps_count < max_search_steps:
                    cur_x, cur_y = bfs_q.popleft()
                    steps_count += 1
                    
                    if 0 <= cur_x < self.field.grid_w and 0 <= cur_y < self.field.grid_h:
                        dvx = self.field.dir_field[cur_y, cur_x, 0]
                        dvy = self.field.dir_field[cur_y, cur_x, 1]
                        if abs(dvx) > 1e-3 or abs(dvy) > 1e-3:
                            target_valid_cell = (cur_x, cur_y)
                            found_valid = True
                            break
                            
                    for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                        nx, ny = cur_x + dx, cur_y + dy
                        if (nx, ny) not in visited:
                            if 0 <= nx < self.field.grid_w and 0 <= ny < self.field.grid_h:
                                visited.add((nx, ny))
                                bfs_q.append((nx, ny))
                                
            if found_valid and target_valid_cell:
                self.active_fallback_cell = target_valid_cell
                tx, ty = self.field.cell_to_world_center(*target_valid_cell)
                vx, vy = tx - x, ty - y
                
                mag_temp = math.hypot(vx, vy)
                if mag_temp < 0.35:
                    dvx = self.field.dir_field[target_valid_cell[1], target_valid_cell[0], 0]
                    dvy = self.field.dir_field[target_valid_cell[1], target_valid_cell[0], 1]
                    L = 0.6
                    if abs(dvx) > 1e-3 or abs(dvy) > 1e-3:
                        vx, vy = (tx + dvx * L) - x, (ty + dvy * L) - y
                    if mag_temp < 0.1:
                        self.active_fallback_cell = None
            else:
                vx, vy = gx - x, gy - y
                is_fallback_mode = False
                
            mag_dir = math.hypot(vx, vy)

        if mag_dir > 1e-6:
            d_pref = np.array([vx / mag_dir, vy / mag_dir], dtype=float)
        else:
            d_pref = np.zeros(2, dtype=float)

        dist_factor = min(1.0, dist_goal / 0.4)
        s_pref = self.v_max * dist_factor
        return s_pref * d_pref, is_fallback_mode

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
        dist_to_goal = np.linalg.norm(self.goal - np.array([x, y]))

        # Check if arrived
        if dist_to_goal < self.arrival_threshold:
            return {
                "v_cmd": 0.0,
                "w_cmd": 0.0,
                "dist_to_goal": dist_to_goal,
                "arrived": True,
            }

        # Compute preferred velocity from flow field
        v_pref_vec, is_fallback_mode = self.get_preferred_velocity(state)

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
