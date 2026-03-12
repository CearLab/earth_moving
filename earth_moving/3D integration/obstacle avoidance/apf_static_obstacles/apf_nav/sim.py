"""
Simulation utilities for APF navigation.

Includes unicycle integration, distance calculation, and complete step function.
"""

import numpy as np
import math
from typing import List, Tuple
from .models import RoverState, RoverParams, Circle
from .fields import goal_force, static_repulsion, dynamic_repulsion, tangential_bias
from .control import force_to_commands, safety_override


def integrate_unicycle(x: float, y: float, psi: float, v: float, w: float, dt: float) -> Tuple[float, float, float]:
    """
    Exact constant-input integration for unicycle model.

    Integrates:
        dx/dt = v * cos(psi)
        dy/dt = v * sin(psi)
        dpsi/dt = w

    Args:
        x, y: Current position
        psi: Current heading (radians)
        v: Forward velocity
        w: Angular velocity
        dt: Time step

    Returns:
        (x_new, y_new, psi_new): Updated state
    """
    eps = 1e-9

    if abs(w) < eps:
        # Straight line motion
        x_new = x + v * math.cos(psi) * dt
        y_new = y + v * math.sin(psi) * dt
        psi_new = psi
    else:
        # Circular arc motion
        # Instantaneous center of curvature (ICC)
        R = v / w

        # Rotate heading
        psi_new = psi + w * dt

        # Position update via rotation around ICC
        dx = R * (math.sin(psi_new) - math.sin(psi))
        dy = R * (-math.cos(psi_new) + math.cos(psi))

        x_new = x + dx
        y_new = y + dy

    return x_new, y_new, psi_new


def nearest_distance(
    p: np.ndarray,
    obstacles: List[Circle],
    neighbors_p: List[np.ndarray],
    self_radius: float
) -> float:
    """
    Compute minimum distance to any obstacle or neighbor surface.

    Args:
        p: Current position [x, y]
        obstacles: List of static circular obstacles (cx, cy, R)
        neighbors_p: List of neighbor positions [x, y]
        self_radius: Radius of current rover

    Returns:
        Minimum surface-to-surface distance
    """
    d_min = float('inf')

    # Distance to static obstacles (boundary)
    for cx, cy, R in obstacles:
        d_center = np.linalg.norm(p - np.array([cx, cy]))
        d_surface = d_center - R
        d_min = min(d_min, d_surface)

    # Distance to neighbors (surface-to-surface)
    for p_j in neighbors_p:
        d_center = np.linalg.norm(p - p_j)
        d_surface = d_center - 2 * self_radius  # Both have radius
        d_min = min(d_min, d_surface)

    return max(0.0, d_min)  # Clamp to non-negative


def step_rovers(
    states: List[RoverState],
    goals: List[np.ndarray],
    obstacles: List[Circle],
    prm: RoverParams,
    dt: float
) -> List[RoverState]:
    """
    Complete APF step: compute fields, generate commands, integrate dynamics.

    Args:
        states: List of current rover states
        goals: List of goal positions (one per rover)
        obstacles: List of static circular obstacles
        prm: Rover parameters (same for all rovers)
        dt: Time step

    Returns:
        List of updated rover states
    """
    n_rovers = len(states)
    new_states = []

    for i, state in enumerate(states):
        # Current state
        p = np.array([state.x, state.y])
        psi = state.psi
        v_mag = np.linalg.norm(state.v_est)

        goal = goals[i]

        # Collect neighbor info (all other rovers)
        neighbors = []
        neighbors_p = []
        for j, other_state in enumerate(states):
            if i != j:
                p_j = np.array([other_state.x, other_state.y])
                v_j = other_state.v_est
                neighbors.append((p_j, v_j))
                neighbors_p.append(p_j)

        # Build total force field
        F_goal = goal_force(p, goal, prm)
        F_static = static_repulsion(p, obstacles, prm)
        F_dynamic = dynamic_repulsion(p, psi, v_mag, neighbors, prm)

        # Optional tangential bias (for head-on scenarios)
        F_bias = np.zeros(2)
        if prm.bias_eta > 0 and len(neighbors) > 0:
            # Apply bias to nearest neighbor
            nearest_idx = np.argmin([np.linalg.norm(p_j - p) for p_j in neighbors_p])
            F_bias = tangential_bias(p, neighbors_p[nearest_idx], prm.bias_eta)

        F_total = F_goal + F_static + F_dynamic + F_bias

        # Compute minimum distance for speed gating
        d_min = nearest_distance(p, obstacles, neighbors_p, state.radius)

        # Map force to commands
        v, omega = force_to_commands(p, psi, F_total, d_min, prm)

        # Safety override
        # Check if closing with nearest obstacle
        closing = False
        if len(neighbors_p) > 0:
            nearest_neighbor = neighbors_p[np.argmin([np.linalg.norm(p_j - p) for p_j in neighbors_p])]
            r_to_nearest = nearest_neighbor - p
            v_self = state.v_est
            closing = np.dot(r_to_nearest, v_self) > 0

        # Heading error to goal
        goal_dir = goal - p
        psi_goal = math.atan2(goal_dir[1], goal_dir[0])
        e_heading = psi_goal - psi

        v, omega = safety_override(v, omega, e_heading, d_min, closing, prm)

        # Integrate dynamics
        x_new, y_new, psi_new = integrate_unicycle(state.x, state.y, psi, v, omega, dt)

        # Estimate velocity for next step
        v_est_new = np.array([v * math.cos(psi_new), v * math.sin(psi_new)])

        # Create new state
        new_state = RoverState(
            x=x_new,
            y=y_new,
            psi=psi_new,
            v_est=v_est_new,
            radius=state.radius
        )

        new_states.append(new_state)

    return new_states
