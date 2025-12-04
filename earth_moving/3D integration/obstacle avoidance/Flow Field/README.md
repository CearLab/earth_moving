# Flow Field Navigation with Obstacle Avoidance

This folder contains a PyBullet-based implementation of autonomous rover navigation using Dijkstra-based flow fields for goal-directed navigation with dynamic obstacle avoidance.

## Overview

Flow field navigation guides a differential-drive rover from a start position to a goal position while avoiding obstacles. The system uses **shovel-point tracking** (0.17m forward offset from rover center) to ensure the material collection point, not the rover's center of mass, reaches the goal.

## How Flow Field Calculation Works

### 1. Grid Discretization

The continuous world space is discretized into a 2D grid:
- **World bounds**: `[world_xmin, world_xmax] x [world_ymin, world_ymax]`
- **Grid resolution**: `grid_w x grid_h` cells (default: 41x41)
- **Cell size**: Computed as `cell_w = (world_xmax - world_xmin) / grid_w`

Each cell represents a small region of the world where the rover can potentially be.

### 2. Obstacle Stamping

Before computing the flow field, obstacles are marked on the grid:

```python
R_block = rover_radius + pebble_radius + clearance
```

For each obstacle (pebble):
- Find all grid cells within distance `R_block` from the obstacle center
- Mark these cells as **blocked** (boolean grid `obstacles[y, x] = True`)
- This ensures the rover cannot plan paths through obstacles

The blocking radius accounts for:
- **rover_radius** (0.25m): Physical footprint of the rover
- **pebble_radius** (0.05m): Size of each obstacle
- **clearance** (0.02m): Safety margin

### 3. Distance Field Computation (Dijkstra's Algorithm)

The distance field assigns each free cell a value representing the shortest obstacle-free distance to the goal:

**Algorithm:**
1. Initialize all cells to `dist = ∞`
2. Set goal cell to `dist = 0`
3. Use a priority queue (min-heap) starting from the goal
4. For each cell, propagate to 8-connected neighbors:
   ```
   new_dist = current_dist + step_length
   ```
   where `step_length = sqrt(dx² + dy²)` for diagonal vs. cardinal moves
5. Update neighbor distance if `new_dist < old_dist`
6. Skip cells that are blocked (obstacles)

**Result:** Each free cell contains the shortest distance to the goal following only obstacle-free paths.

### 4. Direction Field Computation (Gradient Descent)

The direction field converts distances into navigation vectors:

**For each grid cell:**
1. Compute the **gradient** of the distance field using finite differences:
   ```python
   grad_x = Σ (dist[neighbor_x] - dist[current]) * dx
   grad_y = Σ (dist[neighbor_y] - dist[current]) * dy
   ```
   Sum over all 8 neighbors

2. The **flow direction** is the negative gradient (points downhill toward goal):
   ```python
   direction = -gradient
   ```

3. Normalize to unit vector:
   ```python
   dir_field[y, x] = direction / ||direction||
   ```

**Result:** Each cell contains a unit vector pointing in the optimal direction to reach the goal while avoiding obstacles.

### 5. Runtime Query

During navigation, the rover queries the flow field:

```python
direction = flow_field.get_direction_world(x_shovel, y_shovel)
```

This:
1. Converts shovel world coordinates `(x, y)` to grid indices `(ix, iy)`
2. Returns the precomputed direction vector at that cell
3. Controller steers the rover to align with this direction

## Files

- `flowfield_pybullet.py` - Complete flow field navigation implementation
- `2_wheel_rover.urdf` - Differential drive rover model
- `pebbles.urdf` - Small obstacle model (0.05m radius)

## Key Components

### FlowField2D Class

**Methods:**
- `stamp_pebbles(pebble_centers)` - Mark obstacle regions on the grid
- `compute_distance_field(goal_cell)` - Dijkstra's algorithm from goal
- `compute_direction_field()` - Compute gradient-based direction vectors
- `rebuild(goal_world, pebble_centers)` - Full rebuild pipeline
- `get_direction_world(x, y)` - Query direction at world position
- `draw_debug()` - Visualize flow field as arrows (blue) and obstacles (red crosses)

### FlowFieldController Class

**Control law:**
1. Query flow field direction at shovel position
2. Compute desired heading: `θ_desired = atan2(dir_y, dir_x)`
3. Heading error: `e_θ = θ_desired - θ_current`
4. **Turn-in-place logic**: If nearly stationary and large heading error, rotate in place with zero forward velocity
5. **Normal steering**:
   - Angular velocity: `ω = k_θ * e_θ` (P-controller)
   - Forward velocity: `v = v_max * cos(e_θ) * dist_factor`
   - Slows down near goal and when misaligned

**Parameters:**
- `v_max`: Maximum forward velocity (1.5 m/s)
- `w_max`: Maximum angular velocity (10.0 rad/s)
- `k_theta`: Heading proportional gain (10.0)
- `turn_in_place_angle_deg`: Threshold for stationary rotation (50°)
- `w_turn_in_place`: Angular speed for in-place rotation (25.0 rad/s)

## Shovel-Point Tracking

The system tracks the **shovel position**, not the rover center:

```python
SHOVEL_OFFSET = 0.17  # meters forward from rover center
x_shovel = x_rover + SHOVEL_OFFSET * cos(yaw)
y_shovel = y_rover + SHOVEL_OFFSET * sin(yaw)
```

**Why?** In earth-moving applications, the material collection point (shovel) must reach the target, not the vehicle center. This ensures accurate positioning for scooping operations.

## Algorithm Complexity

- **Grid size**: N = `grid_w × grid_h` cells
- **Distance field**: O(N log N) - Dijkstra with priority queue
- **Direction field**: O(N) - Single pass over all cells
- **Runtime query**: O(1) - Direct grid lookup

## Usage

Run the flow field navigation:
```bash
python flowfield_pybullet.py
```

**Visualization options:**
- Set `DRAW_FLOWFIELD = True` to visualize direction vectors (blue arrows) and obstacles (red crosses)
- Green sphere: Goal position
- Blue rover: Autonomous agent

## Environment Parameters

- **World scale**: 5m radius circular region
- **Grid resolution**: 41x41 cells (~0.24m per cell)
- **Obstacles**: 90 randomly placed pebbles (0.05m radius)
- **Rover footprint**: 0.25m radius
- **Shovel offset**: 0.17m forward

## Advantages of Flow Fields

1. **Precomputed**: Distance and direction fields computed once, reused for entire trajectory
2. **Smooth navigation**: Gradient-based directions create smooth, natural paths
3. **Obstacle avoidance**: Dijkstra naturally routes around blocked regions
4. **Computational efficiency**: O(1) runtime queries after O(N log N) preprocessing
5. **Multiple agents**: Single field can guide many rovers simultaneously

## Limitations

1. **Static obstacles**: Field must be recomputed if obstacles move
2. **Grid resolution**: Trade-off between accuracy and computation time
3. **Local planning**: Does not plan full trajectory, follows local gradient
4. **Grid artifacts**: Discretization can create slight path irregularities
