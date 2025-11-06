"""
Potential field calculations for APF navigation.

Implements attractive (goal), repulsive (static/dynamic obstacles),
and tangential bias forces.
"""

import numpy as np
import math
from typing import List, Tuple
from .models import RoverParams, Circle


def goal_force(p: np.ndarray, g: np.ndarray, prm: RoverParams) -> np.ndarray:
    """
    Compute attractive force toward goal using conic-parabolic hybrid.

    Reduces orbiting near goal by switching from quadratic to linear potential.

    Args:
        p: Current position [x, y]
        g: Goal position [x, y]
        prm: Rover parameters

    Returns:
        Force vector [Fx, Fy] pointing toward goal
    """
    r = g - p
    d = np.linalg.norm(r)
    eps = 1e-9

    if d < eps:
        return np.zeros(2)

    r_hat = r / d

    if d <= prm.d_switch:
        # Parabolic region: F = k_att * d * r_hat
        F_mag = prm.k_att * d
    else:
        # Conic region: F = k_att * d_switch * r_hat
        F_mag = prm.k_att * prm.d_switch

    return F_mag * r_hat


def static_repulsion(p: np.ndarray, obstacles: List[Circle], prm: RoverParams,
                     goal: np.ndarray = None) -> np.ndarray:
    """
    Compute repulsive force from static circular obstacles.

    Uses boundary distance (not center distance) for accurate collision avoidance.

    Optional goal-proximity override: When rover is closer to goal than to any obstacle,
    repulsion is disabled to allow final approach even with nearby obstacles.

    Args:
        p: Current position [x, y]
        obstacles: List of (cx, cy, radius) tuples
        prm: Rover parameters
        goal: Optional goal position [x, y] for goal-proximity override

    Returns:
        Total repulsive force vector [Fx, Fy]
    """
    F_total = np.zeros(2)
    eps = 1e-9

    # Goal-proximity override: if closer to goal than to any obstacle, disable repulsion
    if goal is not None:
        dist_to_goal = np.linalg.norm(goal - p)

        # Find minimum distance to any obstacle surface
        min_obstacle_dist = float('inf')
        for cx, cy, R in obstacles:
            r = p - np.array([cx, cy])
            d_center = np.linalg.norm(r)
            d_surface = d_center - R
            min_obstacle_dist = min(min_obstacle_dist, d_surface)

        # If closer to goal than to nearest obstacle, disable repulsion
        if dist_to_goal < min_obstacle_dist:
            return np.zeros(2)

    for cx, cy, R in obstacles:
        r = p - np.array([cx, cy])
        d_center = np.linalg.norm(r)

        if d_center < eps:
            continue

        # Distance to boundary surface
        d = d_center - R

        # Only apply repulsion within cutoff distance
        if d <= prm.d0 and d > eps:
            r_hat = r / d_center

            # Repulsive potential: U = 0.5 * k_rep * (1/d - 1/d0)^2
            # Force: F = k_rep * (1/d - 1/d0) * (1/d^2) * r_hat
            term = (1.0 / d) - (1.0 / prm.d0)
            F_mag = prm.k_rep * term / (d * d + eps)

            F_total += F_mag * r_hat

    return F_total


def dynamic_repulsion(
    p: np.ndarray,
    self_heading: float,
    self_vmag: float,
    neighbors: List[Tuple[np.ndarray, np.ndarray]],  # (p_j, v_j)
    prm: RoverParams
) -> np.ndarray:
    """
    Compute approach-aware repulsion from dynamic obstacles (other rovers).

    Strengthens repulsion when agents are closing and nearby using sigmoid weighting.

    Args:
        p: Current position [x, y]
        self_heading: Current heading angle (radians)
        self_vmag: Current speed magnitude
        neighbors: List of (position, velocity) tuples for other rovers
        prm: Rover parameters

    Returns:
        Total dynamic repulsive force vector [Fx, Fy]
    """
    F_total = np.zeros(2)
    eps = 1e-9

    # Self velocity vector
    v_self = self_vmag * np.array([math.cos(self_heading), math.sin(self_heading)])

    for p_j, v_j in neighbors:
        r = p_j - p
        d = np.linalg.norm(r)

        if d < eps or d > prm.dyn_range:
            continue

        r_hat = r / d

        # Relative velocity (how fast neighbor is approaching)
        v_rel = v_j - v_self

        # Closing speed (positive when approaching)
        c = max(0.0, np.dot(r_hat, v_rel))

        # Sigmoid weight: emphasizes close + approaching scenarios
        # w = sigmoid(alpha * (d_safe + beta * c - d))
        z = prm.alpha * (prm.d_safe + prm.beta * c - d)
        w_dyn = 1.0 / (1.0 + math.exp(-z))

        # Repulsive force magnitude
        F_mag = prm.k_dyn * w_dyn / ((d + eps) ** 2)

        F_total += F_mag * r_hat

    return F_total


def tangential_bias(p: np.ndarray, neighbor_p: np.ndarray, eta: float) -> np.ndarray:
    """
    Compute tangential bias to break head-on symmetry.

    Applies 90° rotated force to help resolve deadlocks in narrow passages.

    Args:
        p: Current position [x, y]
        neighbor_p: Neighbor position [x, y]
        eta: Bias strength coefficient

    Returns:
        Tangential force vector [Fx, Fy]
    """
    r = neighbor_p - p
    d = np.linalg.norm(r)
    eps = 1e-9

    if d < eps:
        return np.zeros(2)

    r_hat = r / d

    # 90-degree rotation (left-normal)
    F_bias = eta * np.array([-r_hat[1], r_hat[0]])

    return F_bias
