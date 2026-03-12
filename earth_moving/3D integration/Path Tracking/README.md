# Path Tracking with Flow Fields

This folder contains a PyBullet-based implementation of autonomous rover navigation using flow field path planning methods.

## Overview

Two navigation approaches are implemented for a differential-drive rover:

1. **Goal-Directed Navigation** (`flowfield_pybullet.py`) - Navigate to a goal position while avoiding obstacles using Dijkstra-based flow fields
2. **Path-Following Navigation** (`path_following_flowfield.py`) - Follow a predefined curved path with precise shovel-point tracking (not center of mass)

## Files

- `flowfield_pybullet.py` - Base flow field implementation with goal-based navigation
- `path_following_flowfield.py` - Path-following extension using vector field guidance
- `2_wheel_rover.urdf` - Differential drive rover model
- `pebbles.urdf` - Small obstacle model

## Key Components

### Flow Field Generation
- **Grid-based approach**: World space discretized into a 2D grid
- **Dijkstra's algorithm**: Computes distance field from goal position (used in `flowfield_pybullet.py`)
- **Direction field**: Gradient-based vector field pointing toward goal
- **Obstacle avoidance**: Implemented in `flowfield_pybullet.py` for goal-directed navigation among obstacles

### Path Following
- **Shovel-point tracking**: Path following tracks the **shovel position** (front offset point), not the rover's center of mass, ensuring the material collection point follows the desired trajectory
- **Vector field guidance**: Creates flow field along predefined path using lateral error correction
- **Arc-length tracking**: Monitors progress along path with ETA estimation and landmark detection
- **Multiple path shapes**: Supports straight, sinusoidal, and hairpin path generation
- **Offset distance**: Uses 0.17m forward offset from rover center to represent shovel position

### Controller
- **Unicycle model**: Forward velocity (v) and angular velocity (ω) control
- **Turn-in-place logic**: Stationary rotation for large heading errors
- **Speed regulation**: Slows down near goal and adjusts for alignment

## Usage

Run goal-directed navigation:
```bash
python flowfield_pybullet.py
```

Run path-following navigation:
```bash
python path_following_flowfield.py
```

## Parameters

Key tunable parameters in `path_following_flowfield.py`:
- `k_t`: Tangential gain along path (default: 2.0)
- `k_n`: Normal gain for lateral error correction (default: 10.0)
- `band_radius`: Width of corridor around path (default: 0.5 m)
- `v_max`: Maximum forward velocity (default: 1.0 m/s)
- `w_max`: Maximum angular velocity (default: 6.0 rad/s)

## Environment

- **Simulation**: PyBullet physics engine
- **Robot model**: Two-wheeled differential drive rover (0.5m wheelbase)
- **World scale**: Configurable (default: 5m radius for goal-directed navigation, path-adaptive bounds for path following)
- **Obstacles**: Optional pebble obstacles (0.05m radius) - primarily used in `flowfield_pybullet.py` for demonstrating obstacle avoidance
