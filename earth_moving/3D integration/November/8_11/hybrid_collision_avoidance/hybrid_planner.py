"""
Hybrid Collision Avoidance Planner

Hierarchical two-layer approach:
  [Layer 1] ORCA Planner: Handles rover-to-rover collision avoidance (hard constraint)
  [Layer 2] APF Navigation: Handles static obstacle avoidance (soft constraint)
            using velocity from Layer 1 as speed constraint

This approach ensures:
- Safe multi-rover coordination (ORCA handles negotiation)
- Smooth navigation around pebbles (APF provides guidance)
- Natural pushing behavior (APF soft constraint allows contact)
- Clear priority hierarchy (rovers > pebbles)

ORCA implementation copied from proven working code in ORCA/nh_ORCA.py
"""

import numpy as np
import math
from typing import List, Dict, Tuple, Optional


# ============================================================================
#  LAYER 1: ORCA PLANNER (Rover-to-Rover Collision Avoidance)
#  Copied from proven implementation in ORCA/nh_ORCA.py
# ============================================================================

class ORCAPlanner:
    """
    ORCA-style priority-aware collision avoidance for multi-rover coordination.

    Outputs: (v, omega) safe velocity command considering only other rovers.
    This becomes the speed constraint for Layer 2 (APF).
    """

    def __init__(self,
                 tau=8.0,              # prediction horizon (s)
                 v_pref=0.8,           # preferred speed toward goal
                 v_max=1.0,            # max translational speed
                 w_max=3.0,            # max angular speed
                 k_theta=3.0,          # heading feedback gain
                 turn_in_place_deg=45.0,
                 n_speed=6,            # velocity space samples (radial)
                 n_angle=20):          # velocity space samples (angular)
        self.tau = tau
        self.v_pref = v_pref
        self.v_max = v_max
        self.w_max = w_max
        self.k_theta = k_theta
        self.turn_in_place = math.radians(turn_in_place_deg)
        self.n_speed = n_speed
        self.n_angle = n_angle

    @staticmethod
    def _shape_radius(shape: Dict) -> float:
        """Approximate rover shape (two circles) as single disc."""
        return max(math.hypot(cx, cy) + r for (cx, cy, r) in shape["circles"])

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap angle to [-π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _ttc_collision(p_ij: np.ndarray, v_rel: np.ndarray, r_sum: float, tau: float) -> bool:
        """
        Check time-to-closest-approach collision.
        COPIED FROM PROVEN ORCA IMPLEMENTATION.

        Args:
            p_ij: Relative position (other - self)
            v_rel: Relative velocity (other velocity - candidate velocity)
            r_sum: Sum of radii
            tau: Time horizon

        Returns:
            True if collision predicted within horizon
        """
        r_eff = r_sum * 1.05  # 5% safety margin

        dist0 = np.linalg.norm(p_ij)
        if dist0 < r_eff:
            return True  # Already overlapping

        v2 = np.dot(v_rel, v_rel)
        if v2 < 1e-8:
            return False  # No relative motion

        # Time of closest approach
        t_star = -np.dot(p_ij, v_rel) / v2

        if t_star < 0.0:
            d_min = dist0
        elif t_star > tau:
            d_min = np.linalg.norm(p_ij + v_rel * tau)
        else:
            d_min = np.linalg.norm(p_ij + v_rel * t_star)

        return d_min < r_eff

    def _project_to_unicycle(self, ego: Dict, v_new: np.ndarray) -> Tuple[float, float]:
        """Map 2D velocity to (v, ω) unicycle commands."""
        px, py, th, v_fwd, w = ego["state"]
        speed = np.linalg.norm(v_new)

        if speed < 1e-4:
            return 0.0, 0.0

        heading = math.atan2(v_new[1], v_new[0])
        e_th = self._wrap_angle(heading - th)

        # Turn in place if heading error is large
        if abs(e_th) > self.turn_in_place:
            v_cmd = 0.0
        else:
            v_cmd = min(speed, self.v_max)

        w_cmd = self.k_theta * e_th
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))

        return v_cmd, w_cmd

    def _sample_candidates(self, v_pref_vec: np.ndarray) -> List[np.ndarray]:
        """Sample velocities in disc of radius v_max."""
        cand = [np.array([0.0, 0.0], dtype=float), v_pref_vec]
        speeds = np.linspace(0.0, self.v_max, self.n_speed)
        angles = np.linspace(-math.pi, math.pi, self.n_angle, endpoint=False)

        for s in speeds:
            for ang in angles:
                vx = s * math.cos(ang)
                vy = s * math.sin(ang)
                cand.append(np.array([vx, vy], dtype=float))

        return cand

    def plan(self, ego: Dict, goal: np.ndarray,
             neighbors: List[Dict], shape: Dict) -> Tuple[float, float]:
        """
        ORCA planning considering only rover-to-rover collision avoidance.
        COPIED FROM PROVEN IMPLEMENTATION IN ORCA/nh_ORCA.py

        Args:
            ego: Agent dict with "state", "priority"
            goal: Goal position [gx, gy]
            neighbors: List of neighboring rover dicts
            shape: Ego rover shape

        Returns:
            (v, omega): Safe velocity commands (considering only rovers)
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

        # Preferred velocity toward goal (slow down near goal)
        s_pref = self.v_pref
        if dist_goal < 0.5:
            s_pref *= dist_goal / 0.5
        s_pref = min(self.v_max, max(0.0, s_pref))
        v_pref_vec = s_pref * dir_unit

        # If no neighbors: just go to preferred velocity
        if not neighbors:
            v_cmd, w_cmd = self._project_to_unicycle(ego, v_pref_vec)
            return float(v_cmd), float(w_cmd)

        # Precompute neighbor info for TTC checks
        r_i = self._shape_radius(shape)
        pr_i = ego.get("priority", 0)

        neigh_info = []
        for nb in neighbors:
            pxj, pyj, thj, vj_fwd, wj = nb["state"]
            p_j = np.array([pxj, pyj], dtype=float)
            vx_j = math.cos(thj) * vj_fwd
            vy_j = math.sin(thj) * vj_fwd
            v_j = np.array([vx_j, vy_j], dtype=float)

            r_j = self._shape_radius(nb["shape"])
            r_sum = r_i + r_j

            pr_j = nb.get("priority", 0)
            is_blind = nb.get("is_blind", False)

            # Responsibility logic (same as original ORCA)
            if is_blind or pr_j < pr_i:
                must_avoid = True
            elif pr_j > pr_i:
                must_avoid = False
            else:
                must_avoid = True  # Equal priority

            neigh_info.append({
                "p_ij": p_j - p_i,
                "v_j": v_j,
                "r_sum": r_sum,
                "must_avoid": must_avoid,
            })

        # Sample and evaluate candidates
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

            # Cost = distance to preferred velocity
            cost = np.linalg.norm(v - v_pref_vec)
            if cost < best_cost:
                best_cost = cost
                best_v = v

        # Fallback to stop if no collision-free candidate
        if best_cost == float("inf"):
            best_v = np.zeros(2, dtype=float)

        v_cmd, w_cmd = self._project_to_unicycle(ego, best_v)
        return float(v_cmd), float(w_cmd)


# ============================================================================
#  LAYER 2: APF NAVIGATION (Static Obstacle Avoidance)
# ============================================================================

class APFNavigator:
    """
    Artificial Potential Field navigation for smooth obstacle avoidance.

    Uses velocity from ORCA as speed constraint (v_max_constraint).
    Applies APF to smoothly navigate around static pebbles.
    """

    def __init__(self,
                 k_att=1.5,            # Attractive gain
                 k_rep=0.08,           # Repulsive gain
                 d0=0.55,              # Repulsion cutoff (m)
                 v_max=1.2,            # Maximum velocity
                 w_max=3.5,            # Maximum angular velocity
                 k_w=3.0):             # Angular feedback gain
        self.k_att = k_att
        self.k_rep = k_rep
        self.d0 = d0
        self.v_max = v_max
        self.w_max = w_max
        self.k_w = k_w

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap angle to [-π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    def _attractive_force(self, p: np.ndarray, g: np.ndarray) -> np.ndarray:
        """Compute attractive force toward goal."""
        r = g - p
        d = np.linalg.norm(r)
        eps = 1e-9

        if d < eps:
            return np.zeros(2)

        r_hat = r / d
        # Simple linear attraction
        F_mag = self.k_att * d
        return F_mag * r_hat

    def _repulsive_force(self, p: np.ndarray,
                        obstacles: List[Tuple[float, float, float]]) -> np.ndarray:
        """
        Compute repulsive force from static obstacles.

        Args:
            p: Current position [x, y]
            obstacles: List of (cx, cy, radius) tuples

        Returns:
            Total repulsive force
        """
        F_total = np.zeros(2)
        eps = 1e-9

        for cx, cy, R in obstacles:
            r = p - np.array([cx, cy])
            d_center = np.linalg.norm(r)

            if d_center < eps:
                continue

            # Distance to boundary
            d = d_center - R

            # Only apply repulsion within cutoff distance
            if d <= self.d0 and d > eps:
                r_hat = r / d_center

                # Repulsive force magnitude
                term = (1.0 / d) - (1.0 / self.d0)
                F_mag = self.k_rep * term / (d * d + eps)

                F_total += F_mag * r_hat

        return F_total

    def compute(self,
                position: np.ndarray,
                heading: float,
                goal: np.ndarray,
                obstacles: List[Tuple[float, float, float]],
                v_max_constraint: Optional[float] = None) -> Tuple[float, float]:
        """
        Compute APF-based (v, ω) commands with speed constraint from ORCA.

        Args:
            position: Current position [x, y]
            heading: Current heading (radians)
            goal: Goal position [gx, gy]
            obstacles: List of (cx, cy, radius) pebbles
            v_max_constraint: Maximum velocity from ORCA layer (if None, use self.v_max)

        Returns:
            (v, omega): Control commands
        """
        p = np.array(position, dtype=float)
        g = np.array(goal, dtype=float)

        # Compute forces
        F_att = self._attractive_force(p, g)
        F_rep = self._repulsive_force(p, obstacles)
        F_total = F_att + F_rep

        # Desired heading from force
        F_mag = np.linalg.norm(F_total)
        eps = 1e-9

        if F_mag < eps:
            return 0.0, 0.0

        psi_d = math.atan2(F_total[1], F_total[0])

        # Heading error
        e = self._wrap_angle(psi_d - heading)

        # Angular velocity with saturation
        omega = self.k_w * e
        omega = max(-self.w_max, min(self.w_max, omega))

        # Forward velocity with speed constraint from ORCA
        v_max_allowed = v_max_constraint if v_max_constraint is not None else self.v_max

        # Speed reduction based on heading error
        v = v_max_allowed * math.cos(e)
        v = max(0.0, min(v, v_max_allowed))

        return float(v), float(omega)


# ============================================================================
#  HYBRID COORDINATOR: Combines both layers
# ============================================================================

class HybridCollisionAvoidance:
    """
    Hybrid planner that combines ORCA (rover-rover) and APF (obstacle avoidance).

    Pipeline:
      [Layer 1] ORCA: rover neighbors → v_safe
      [Layer 2] APF: v_safe + static obstacles → final (v, ω)
    """

    def __init__(self,
                 orca_params: Dict = None,
                 apf_params: Dict = None):
        """
        Initialize hybrid planner.

        Args:
            orca_params: Dict with ORCA parameters (optional)
            apf_params: Dict with APF parameters (optional)
        """
        # Initialize ORCA layer
        orca_params = orca_params or {}
        self.orca = ORCAPlanner(
            tau=orca_params.get("tau", 8.0),
            v_pref=orca_params.get("v_pref", 0.8),
            v_max=orca_params.get("v_max", 1.0),
            w_max=orca_params.get("w_max", 3.0),
            k_theta=orca_params.get("k_theta", 3.0),
            turn_in_place_deg=orca_params.get("turn_in_place_deg", 45.0),
            n_speed=orca_params.get("n_speed", 6),
            n_angle=orca_params.get("n_angle", 20),
        )

        # Initialize APF layer
        apf_params = apf_params or {}
        self.apf = APFNavigator(
            k_att=apf_params.get("k_att", 1.5),
            k_rep=apf_params.get("k_rep", 0.08),
            d0=apf_params.get("d0", 0.55),
            v_max=apf_params.get("v_max", 1.2),
            w_max=apf_params.get("w_max", 3.5),
            k_w=apf_params.get("k_w", 3.0),
        )

    def plan(self,
             ego: Dict,
             goal: np.ndarray,
             rover_neighbors: List[Dict],
             rover_shape: Dict,
             static_obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float]:
        """
        Compute hybrid control command.

        Current implementation: ORCA only (proven working).

        Note: APF layer for static obstacle avoidance is implemented below
        but requires careful coupling to maintain ORCA rover-rover safety.
        Future work: Properly integrate APF heading modulation without breaking ORCA.

        Pipeline:
          1. ORCA: Compute safe velocity considering only rovers (ACTIVE)
          2. APF: Navigate around static obstacles (AVAILABLE BUT NOT YET INTEGRATED)

        Args:
            ego: Agent dict with "state", "priority", "shape"
            goal: Goal position [gx, gy]
            rover_neighbors: List of neighboring rovers
            rover_shape: Ego rover shape dict
            static_obstacles: List of (cx, cy, r) pebbles (currently unused)

        Returns:
            (v, omega): Final control command from ORCA layer
        """
        # ---- LAYER 1: ORCA (Rover-to-Rover) ----
        # ORCA handles all collision avoidance with other rovers
        v_orca, w_orca = self.orca.plan(ego, goal, rover_neighbors, rover_shape)
        return float(v_orca), float(w_orca)

        # ---- LAYER 2: APF (Static Obstacle Avoidance) ---- [NOT YET INTEGRATED]
        # The code below implements APF heading modulation but requires
        # proper decoupling from ORCA to work correctly.
        # Uncommenting will break rover-rover collision avoidance.
        #
        # if static_obstacles:
        #     px, py, th, _, _ = ego["state"]
        #     position = np.array([px, py], dtype=float)
        #     heading = float(th)
        #
        #     F_att = self.apf._attractive_force(position, goal)
        #     F_rep = self.apf._repulsive_force(position, static_obstacles)
        #     F_total = F_att + F_rep
        #
        #     F_mag = np.linalg.norm(F_total)
        #     if F_mag > 1e-9:
        #         psi_d = math.atan2(F_total[1], F_total[0])
        #     else:
        #         psi_d = heading
        #
        #     e = self.apf._wrap_angle(psi_d - heading)
        #     w_new = self.apf.k_w * e
        #     w_new = max(-self.apf.w_max, min(self.apf.w_max, w_new))
        #
        #     return float(v_orca), float(w_new)
