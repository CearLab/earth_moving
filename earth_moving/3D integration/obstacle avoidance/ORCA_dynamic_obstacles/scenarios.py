"""
Pre-defined test scenarios for VO collision avoidance.
"""

import numpy as np


def scenario_2_robots_head_on():
    """Two robots moving toward each other head-on."""
    return {
        "name": "2 Robots Head-On",
        "description": "Two robots moving directly toward each other",
        "rovers": [
            {
                "start_pos": [-4.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [4.0, 0.0],
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [4.0, 0.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-4.0, 0.0],
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
        ]
    }


def scenario_2_robots_crossing():
    """Two robots crossing paths."""
    return {
        "name": "2 Robots Crossing",
        "description": "Two robots on crossing paths (90 degree collision)",
        "rovers": [
            {
                "start_pos": [-2.0, 1.1, 0.0],
                "start_heading": 0.0,
                "goal_pos": [2.0, 0.0],
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [0.0, -2.0, 0.0],
                "start_heading": np.pi / 2,
                "goal_pos": [0.0, 2.0],
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
        ]
    }


def scenario_4_robots_crossing():
    """Four robots in a crossing configuration."""
    return {
        "name": "4 Robots Crossing",
        "description": "Four robots crossing at a central point",
        "rovers": [
            {
                "start_pos": [-3.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [3.0, 0.0],
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [3.0, 0.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-3.0, 0.0],
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
            {
                "start_pos": [0.0, -3.0, 0.0],
                "start_heading": np.pi / 2,
                "goal_pos": [0.0, 3.0],
                "color": (0.1, 1.0, 0.1, 1.0),  # green
            },
            {
                "start_pos": [0.0, 3.0, 0.0],
                "start_heading": -np.pi / 2,
                "goal_pos": [0.0, -3.0],
                "color": (1.0, 0.1, 0.1, 1.0),  # red
            },
        ]
    }


def scenario_4_robots_shuffle():
    """Four robots in a shuffle/lane change scenario."""
    return {
        "name": "4 Robots Shuffle",
        "description": "Two pairs of robots swapping lanes",
        "rovers": [
            {
                "start_pos": [-5.0, 1.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [5.0, -1.0],
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [-5.0, -1.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [5.0, 1.0],
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
            {
                "start_pos": [5.0, 1.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-5.0, -1.0],
                "color": (0.1, 1.0, 0.1, 1.0),  # green
            },
            {
                "start_pos": [5.0, -1.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-5.0, 1.0],
                "color": (1.0, 0.1, 0.1, 1.0),  # red
            },
        ]
    }


def scenario_circular_assembly():
    """Multiple robots converging to a central point."""
    return {
        "name": "Circular Assembly",
        "description": "Multiple robots converging from different directions",
        "rovers": [
            {
                "start_pos": [-6.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [0.0, 0.0],
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [0.0, -6.0, 0.0],
                "start_heading": np.pi / 2,
                "goal_pos": [0.0, 0.0],
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
            {
                "start_pos": [6.0, 0.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [0.0, 0.0],
                "color": (0.1, 1.0, 0.1, 1.0),  # green
            },
            {
                "start_pos": [0.0, 6.0, 0.0],
                "start_heading": -np.pi / 2,
                "goal_pos": [0.0, 0.0],
                "color": (1.0, 0.1, 0.1, 1.0),  # red
            },
        ]
    }


SCENARIOS = {
    "2_head_on": scenario_2_robots_head_on,
    "2_crossing": scenario_2_robots_crossing,
    "4_crossing": scenario_4_robots_crossing,
    "4_shuffle": scenario_4_robots_shuffle,
    "circular": scenario_circular_assembly,
}


def get_scenario(scenario_name: str) -> dict:
    """
    Get scenario by name.

    Args:
        scenario_name: Name of scenario (key in SCENARIOS dict)

    Returns:
        Scenario configuration dict
    """
    if scenario_name not in SCENARIOS:
        print(f"Unknown scenario '{scenario_name}'. Available: {list(SCENARIOS.keys())}")
        return SCENARIOS["2_crossing"]()

    return SCENARIOS[scenario_name]()
