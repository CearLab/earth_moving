"""
APF Navigation - Artificial Potential Field Navigation for Non-Holonomic Rovers

This package implements obstacle avoidance and goal-seeking navigation
using potential fields for differential-drive (unicycle model) rovers.

Designed for integration with PyBullet earth-moving simulation.
"""

from .models import RoverParams, RoverState, Pose, Circle
from .fields import goal_force, static_repulsion, dynamic_repulsion, tangential_bias
from .control import force_to_commands, safety_override
from .sim import integrate_unicycle, nearest_distance, step_rovers
from .tuning import defaults, conservative, aggressive, custom

__all__ = [
    'RoverParams',
    'RoverState',
    'Pose',
    'Circle',
    'goal_force',
    'static_repulsion',
    'dynamic_repulsion',
    'tangential_bias',
    'force_to_commands',
    'safety_override',
    'integrate_unicycle',
    'nearest_distance',
    'step_rovers',
    'defaults',
    'conservative',
    'aggressive',
    'custom',
]

__version__ = '0.1.0'
