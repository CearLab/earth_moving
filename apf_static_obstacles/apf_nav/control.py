"""
Control layer: Maps potential field forces to unicycle commands (v, ω).

Includes speed gating and safety overrides.
"""

import numpy as np
import math
from typing import Tuple
from .models import RoverParams


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def force_to_commands(
    p: np.ndarray,
    psi: float,
    F: np.ndarray,
    dmin: float,
    prm: RoverParams
) -> Tuple[float, float]:
    """
    Map resultant field force to (v, ω) commands with speed gating.

    Args:
        p: Current position [x, y]
        psi: Current heading (radians)
        F: Total force vector [Fx, Fy]
        dmin: Minimum distance to any obstacle
        prm: Rover parameters

    Returns:
        (v, omega): Linear and angular velocity commands
    """
    eps = 1e-9

    # Desired heading from force direction
    F_mag = np.linalg.norm(F)
    if F_mag < eps:
        # No force → stop
        return 0.0, 0.0

    psi_d = math.atan2(F[1], F[0])

    # Heading error
    e = wrap_angle(psi_d - psi)

    # Base angular velocity
    omega_nom = prm.k_w * e

    # Speed gating: reduce speed based on proximity and turn rate
    # Proximity scaling
    if dmin < prm.d_stop:
        s_prox = 0.0
    elif dmin < prm.d_slow:
        s_prox = (dmin - prm.d_stop) / (prm.d_slow - prm.d_stop + eps)
    else:
        s_prox = 1.0

    # Turn rate scaling
    s_turn = min(1.0, prm.w_slow / (abs(omega_nom) + eps))

    # Combined speed gate
    s_v = s_prox * s_turn

    # Forward velocity with cosine gating (reduce speed when turning)
    v = prm.v0 * math.cos(e) * s_v
    v = np.clip(v, 0.0, prm.v_max)

    # Angular velocity
    omega = np.clip(omega_nom, -prm.w_max, prm.w_max)

    return v, omega


def safety_override(
    v: float,
    w: float,
    e_heading: float,
    dmin: float,
    closing: bool,
    prm: RoverParams
) -> Tuple[float, float]:
    """
    Safety override: brake and turn away if inside safety bubble.

    Args:
        v: Nominal forward velocity
        w: Nominal angular velocity
        e_heading: Heading error to goal
        dmin: Minimum distance to nearest obstacle
        closing: True if approaching nearest obstacle
        prm: Rover parameters

    Returns:
        (v_safe, omega_safe): Overridden commands
    """
    if dmin < prm.d_safe and closing:
        # Emergency: stop and turn away
        v_safe = 0.0

        # Turn direction based on heading error sign
        omega_safe = prm.w_max * (1.0 if e_heading > 0 else -1.0)

        return v_safe, omega_safe

    # No override needed
    return v, w
