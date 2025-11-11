"""
Test scenarios for hybrid collision avoidance system.

Scenarios test different combinations of:
- Multiple rovers (with different priorities)
- Static obstacles (pebbles)
- Various goal configurations
"""

import numpy as np


def scenario_2_rovers_head_on_with_pebbles():
    """
    Two rovers head-on with pebbles in between.
    Tests rover-rover avoidance priority over pebple avoidance.
    """
    return {
        "name": "2 Rovers Head-On (with Pebbles)",
        "description": "Two robots approaching each other with obstacles in between",
        "rovers": [
            {
                "start_pos": [-4.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [4.0, 0.0],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [4.0, 0.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-4.0, 0.0],
                "priority": 1,
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
        ],
        "pebbles": [
            (-0.5, 0.3),   # Group on left side
            (-0.5, -0.3),
            (0.5, 0.3),    # Group on right side
            (0.5, -0.3),
        ]
    }


def scenario_4_robots_crossing_with_obstacles():
    """
    Four robots crossing at center with scattered pebbles.
    Tests multi-rover coordination with obstacle field.
    """
    return {
        "name": "4 Robots Crossing (with Obstacles)",
        "description": "Four robots crossing at center, navigating around pebbles",
        "rovers": [
            {
                "start_pos": [-3.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [3.0, 0.0],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [3.0, 0.0, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-3.0, 0.0],
                "priority": 1,
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
            {
                "start_pos": [0.0, -3.0, 0.0],
                "start_heading": np.pi / 2,
                "goal_pos": [0.0, 3.0],
                "priority": 2,
                "color": (0.1, 1.0, 0.1, 1.0),  # green
            },
            {
                "start_pos": [0.0, 3.0, 0.0],
                "start_heading": -np.pi / 2,
                "goal_pos": [0.0, -3.0],
                "priority": 3,
                "color": (1.0, 0.1, 0.1, 1.0),  # red
            },
        ],
        "pebbles": [
            # (-0.8, -0.8), (-0.8, 0.0), (-0.8, 0.8),    # Left
            # (0.0, -0.8), (0.0, 0.8),                    # Center sides
            # (0.8, -0.8), (0.8, 0.0), (0.8, 0.8),       # Right
        ]
    }


def scenario_sparse_pebbles_two_rovers():
    """
    Two rovers, sparse pebbles, different starting positions.
    Tests smooth navigation with minimal obstacle interference.
    """
    return {
        "name": "2 Rovers (Sparse Pebbles)",
        "description": "Two robots with sparse pebbles, smooth navigation expected",
        "rovers": [
            {
                "start_pos": [-2.5, -1.5, 0.0],
                "start_heading": 0.0,
                "goal_pos": [2.5, 1.5],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [2.5, 1.5, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-2.5, -1.5],
                "priority": 1,
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
        ],
        "pebbles": [
            (0.0, 0.0),
            (1.0, 0.5),
            (-1.0, -0.5),
        ]
    }


def scenario_dense_pebbles_corridor():
    """
    Two rovers navigating through corridor of pebbles.
    Tests pushing behavior and tight navigation.
    """
    return {
        "name": "2 Rovers (Dense Corridor)",
        "description": "Two robots navigating through narrow pebble corridor",
        "rovers": [
            {
                "start_pos": [-2.0, 0.5, 0.0],
                "start_heading": 0.0,
                "goal_pos": [2.0, 0.5],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [2.0, -0.5, 0.0],
                "start_heading": np.pi,
                "goal_pos": [-2.0, -0.5],
                "priority": 1,
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
        ],
        "pebbles": [
            # Corridor walls
            (-1.5, 1.0), (-1.0, 1.2), (-0.5, 1.0),
            (0.0, 1.2), (0.5, 1.0), (1.0, 1.2), (1.5, 1.0),
            (-1.5, -1.0), (-1.0, -1.2), (-0.5, -1.0),
            (0.0, -1.2), (0.5, -1.0), (1.0, -1.2), (1.5, -1.0),
        ]
    }


def scenario_three_rovers_convergence():
    """
    Three rovers converging to center from different angles.
    Tests multi-agent negotiation with obstacles.
    """
    return {
        "name": "3 Rovers Convergence",
        "description": "Three robots converging to center, avoiding pebbles and each other",
        "rovers": [
            {
                "start_pos": [-3.0, 0.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [0.0, 0.0],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
            {
                "start_pos": [1.5, 2.5, 0.0],
                "start_heading": -np.pi / 4,
                "goal_pos": [0.0, 0.0],
                "priority": 1,
                "color": (1.0, 0.5, 0.1, 1.0),  # orange
            },
            {
                "start_pos": [1.5, -2.5, 0.0],
                "start_heading": np.pi / 4,
                "goal_pos": [0.0, 0.0],
                "priority": 2,
                "color": (0.1, 1.0, 0.1, 1.0),  # green
            },
        ],
        "pebbles": [
            (-1.0, 0.0), (1.0, 0.0),
            (0.0, -1.0), (0.0, 1.0),
            (0.5, 0.5), (-0.5, -0.5),
        ]
    }


def scenario_single_rover_maze():
    """
    Single rover navigating through pebble maze.
    Tests APF obstacle avoidance without rover-rover dynamics.
    """
    return {
        "name": "Single Rover Maze",
        "description": "Single robot navigating complex pebble arrangement",
        "rovers": [
            {
                "start_pos": [-2.0, -2.0, 0.0],
                "start_heading": 0.0,
                "goal_pos": [2.0, 2.0],
                "priority": 0,
                "color": (0.1, 0.5, 1.0, 1.0),  # blue
            },
        ],
        "pebbles": [
            # Maze structure
            (-1.0, -1.0), (-1.0, 0.0), (-1.0, 1.0),
            (0.0, -1.5), (0.0, 1.5),
            (1.0, -1.0), (1.0, 0.0), (1.0, 1.0),
        ]
    }


# Scenario registry
SCENARIOS = {
    "2_head_on_pebbles": scenario_2_rovers_head_on_with_pebbles,
    "4_crossing_obstacles": scenario_4_robots_crossing_with_obstacles,
    "2_sparse": scenario_sparse_pebbles_two_rovers,
    "2_corridor": scenario_dense_pebbles_corridor,
    "3_convergence": scenario_three_rovers_convergence,
    "1_maze": scenario_single_rover_maze,
}


def get_scenario(scenario_name: str) -> dict:
    """
    Get scenario by name.

    Args:
        scenario_name: Key from SCENARIOS dict

    Returns:
        Scenario configuration dict
    """
    if scenario_name not in SCENARIOS:
        print(f"Unknown scenario '{scenario_name}'")
        print(f"Available: {list(SCENARIOS.keys())}")
        return SCENARIOS["2_head_on_pebbles"]()

    return SCENARIOS[scenario_name]()


def list_scenarios():
    """Print available scenarios."""
    print("\nAvailable Scenarios:")
    print("=" * 70)
    for key, func in SCENARIOS.items():
        scenario = func()
        print(f"  {key:<25} - {scenario['description']}")
    print("=" * 70)
