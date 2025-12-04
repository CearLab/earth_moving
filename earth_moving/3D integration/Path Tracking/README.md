# Path Tracking with Flow Fields

This folder contains a PyBullet-based implementation of autonomous rover navigation using flow field path planning methods.

## Overview

Two navigation approaches are implemented for a differential-drive rover:

1. **Goal-Directed Navigation** (`flowfield_pybullet.py`) - Navigate to a goal position while avoiding obstacles
2. **Path-Following Navigation** (`path_following_flowfield.py`) - Follow a predefined curved path with precise tracking

## Files

- `flowfield_pybullet.py` - Base flow field implementation with goal-based navigation
- `path_following_flowfield.py` - Path-following extension using vector field guidance
- `2_wheel_rover.urdf` - Differential drive rover model
- `pebbles.urdf` - Small obstacle model

## Key Components

### Flow Field Generation
- **Grid-based approach**: World space discretized into a 2D grid
- **Dijkstra's algorithm**: Computes distance field from goal position
- **Direction field**: Gradient-based vector field pointing toward goal
- **Obstacle handling**: Automatic stamping of forbidden regions around obstacles

### Path Following
- **Vector field guidance**: Creates flow field along predefined path using lateral error correction
- **Arc-length tracking**: Monitors progress along path with ETA estimation
- **Shovel offset tracking**: Uses front-offset tracking point for improved accuracy
- **Multiple path shapes**: Supports straight, sinusoidal, and hairpin path generation

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
- **Obstacles**: Randomly placed pebbles with 0.05m radius
- **World scale**: Configurable (default: 5m radius circular region)
