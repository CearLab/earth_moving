"""
Data models for APF navigation system.

Defines rover parameters, state representation, and type aliases.
"""

from dataclasses import dataclass, field
import numpy as np
from typing import Tuple


@dataclass
class RoverParams:
    """
    Complete parameter set for APF navigation.

    All distances in meters, angles in radians, velocities in m/s.
    """
    # Attractive field (goal seeking)
    k_att: float          # Attractive gain
    d_switch: float       # Switch distance for conic-parabolic transition

    # Static repulsion (obstacles)
    k_rep: float          # Repulsive gain
    d0: float             # Cutoff distance for repulsion

    # Dynamic repulsion (other rovers)
    k_dyn: float          # Dynamic repulsion gain
    alpha: float          # Sigmoid steepness for approach weighting
    beta: float           # Closing speed weight factor
    dyn_range: float      # Range for dynamic obstacle consideration

    # Safety distances
    d_safe: float         # Minimum safe distance
    d_stop: float         # Emergency stop distance
    d_slow: float         # Start slowing down distance

    # Control parameters
    v0: float             # Nominal forward velocity (m/s)
    v_max: float          # Maximum velocity (m/s)
    w_max: float          # Maximum angular velocity (rad/s)
    k_w: float            # Angular velocity gain for heading error
    w_slow: float         # Angular velocity threshold for speed reduction

    # Optional tangential bias (for head-on scenarios)
    bias_eta: float = 0.0  # Tangential bias strength


@dataclass
class RoverState:
    """
    Current state of a rover.

    Attributes:
        x, y: Position in meters
        psi: Heading angle in radians
        v_est: Estimated velocity vector [vx, vy] in m/s
        radius: Rover physical radius for collision checking
    """
    x: float
    y: float
    psi: float
    v_est: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 0.35  # Default radius for earth-moving rover


# Type aliases for clarity
Pose = Tuple[float, float, float]  # (x, y, heading)
Circle = Tuple[float, float, float]  # (center_x, center_y, radius)
