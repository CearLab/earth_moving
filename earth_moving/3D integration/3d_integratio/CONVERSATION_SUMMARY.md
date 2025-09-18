# Earth Moving Rover Simulation - Complete Development Summary

## Project Overview
This is a PyBullet-based earth moving rover simulation that integrates 2D grid-based path planning with 3D physics execution. The rover uses differential drive control to collect pebbles from a 2D grid environment and deliver them to target zones with spillage modeling.

## Development Timeline & Major Achievements

### 1. Spillage Visualization Fix
**Problem**: Spillage preview cells weren't showing in orchestrator mode (worked in standalone 2D Algorithm)
**Root Cause**: Missing `impacted_cells` data handling in trajectory preview
**Solution**: Enhanced `visualizer.py:set_trajectory_preview()` method
- Added `self.preview_spillage_cells = path_info.get('impacted_cells', {})` 
- Implemented `draw_preview_spillage_cells()` method
- Now shows purple cells with 'S' markers before trajectory execution

### 2. Trajectory Approach Evolution

#### Phase 1: Simple Approach
- **Goal**: Rover goes to starting point → turns in place → executes task
- **Implementation**: `build_simple_world_path()` and `execute_simple_trajectory()`
- **Key Feature**: Maintained post-delivery turn-around for target zone deliveries only

#### Phase 2: Alignment Gate Pivot Approach (Current)
**Concept**: Intelligent approach based on alignment conditions rather than fixed waypoints

**3-Phase Execution**:
1. **Phase A1**: Navigate BASE LINK to gate point G behind start point S0
2. **Phase A2**: Move toward S0 while monitoring alignment gate conditions
3. **Phase B**: Turn in place to face S0 when alignment conditions are met
4. **Phase C**: Execute task trajectory using Pure Pursuit

**Key Technical Fixes**:
- **Base vs TCP Navigation**: Base link reaches G point for proper turning, TCP used only for alignment detection
- **Correct Pivot Direction**: Rover faces toward S0 after pivot, not S0→S1 direction
- **Absolute Heading**: Turn-in-place uses absolute task heading instead of relative drift correction

### 3. Pure Pursuit++ Enhancement
**Goal**: Improve curve tracking with advanced control features
**Features** (partially implemented):
- Curvature-aware lookahead distance
- Lateral-g speed limiting
- Feed-forward curvature compensation
- Stanley cross-track error correction
- Yaw slew rate limiting

**Configuration** (Enhanced for "curvy mode"):
```python
PURE_PURSUIT_CONFIG = {
    'v_nom': 0.30,          # Nominal velocity
    'a_lat_max': 1.0,       # Allow more cornering authority
    'yaw_slew_rate': 7.0,   # Let heading catch up faster
    'lookahead': 0.10,      # Nominal L_MAX; shrinks on curves  
}
```

**Status**: UnboundLocalError fixed by adding missing variable initializations

## Core File Analysis

### orchestrator.py (Main Coordinator)
**Purpose**: Primary simulation coordinator with user interaction
**Key Features**:
- Interactive controls: S(spillage), V(visualization), T(turns), E(extension), H(help)
- 2D/3D environment synchronization
- Trajectory preview and execution confirmation
- Post-delivery environment recreation when needed

**Configuration Sections**:
```python
SIMULATION_CONFIG = {
    'use_spillage_model': True,
    'visualize_potential': True, 
    'enable_post_delivery_turn': True
}

PATH_EXTENSION_CONFIG = {
    'enable_extension': True,
    'extension_length': 1.8,    # Updated from 1.0m
    'extension_smoothing': 0.8
}
```

**Event Handling**: Mouse clicks for cell selection, keyboard controls for configuration

### pybullet_integration.py (3D Physics Core)
**Purpose**: PyBullet physics integration and trajectory execution
**Key Components**:

**Alignment Gate System**:
- `_alignment_metrics()`: Calculates dot product alignment and distance checks
- `_goto_point()`: Navigates to specific points with base/TCP mode selection  
- `execute_alignment_gate_pivot_trajectory()`: 3-phase trajectory execution

**Trajectory Building**:
- `build_world_path()`: Converts 2D grid coordinates to 3D world path
- `build_simple_world_path()`: Direct approach trajectory (legacy)
- Path extension and smoothing algorithms

**Pure Pursuit Control**:
- Enhanced with curvature awareness and slew rate limiting
- TCP (Tool Center Point) vs base link control modes
- Cross-track error correction

**Post-Trajectory Processing**:
- `_post_trajectory_update_with_turnaround()`: Target delivery with 180° turn
- `_post_trajectory_update()`: Highway delivery without turn
- 3D→2D object position synchronization

### visualizer.py (3D Visualization)
**Purpose**: 3D visualization with trajectory preview and debug markers
**Key Features**:
- Trajectory path visualization with colored line strips
- Spillage preview cells (purple with 'S' markers)
- Debug visualizations for development
- Real-time trajectory updates during preview

**Preview System**:
```python
def set_trajectory_preview(self, clicked_cell, path_type, path_info):
    # Sets trajectory path and spillage cells for preview
    self.preview_spillage_cells = path_info.get('impacted_cells', {})
```

### 2D Algorithm/ Folder (Complete 2D Planning System)

#### main.py (2D Simulation Entry)
**Purpose**: Standalone 2D algorithm with interactive testing
**Configuration**:
```python
TARGET_ANGLE_TOLERANCE = 45   # Target zone visibility angle
HIGHWAY_ANGLE_TOLERANCE = 60  # Highway visibility angle
HIGHWAY_MIN_HEAT_RATIO = 0.3  # Minimum heat threshold
```

#### env.py (2D Environment Core)
**Purpose**: Grid-based environment with A* pathfinding and spillage modeling
**Key Systems**:
- **Grid Management**: 25x25 cell grid with object distribution
- **Visibility Calculations**: Target zone and highway visibility with angle constraints
- **Potential Field**: Flow simulation with spillage effects
- **Heat Map**: Dynamic heat distribution for highway target selection
- **A* Pathfinding**: Multi-objective path optimization

#### visualizer.py (2D Visualization)
**Purpose**: 2D grid visualization with debug overlays
**Features**:
- Grid rendering with object counts and colors
- Trajectory preview with path highlighting
- Debug markers: T(target visibility), H(highway), A(affected), R(recalculation), S(spillage)
- Interactive cell clicking and path preview

#### cell.py, search.py, agents.py, spillage_model.py
**Purpose**: Supporting classes for grid cells, search algorithms, agent definitions, and spillage physics

### URDF Files
- **2_wheel_rover.urdf**: Differential drive rover model with sensors and actuators
- **pebbles.urdf**: Cylindrical pebble objects for collection

## Interactive Controls & Features

### Keyboard Controls (Orchestrator)
- **S**: Toggle spillage model ON/OFF
- **V**: Toggle potential field visualization  
- **T**: Toggle post-delivery 180° turn for target deliveries
- **E**: Toggle path extension (1.8m backward extension from 2D start point)
- **H**: Show help menu
- **ESC**: Exit simulation

### Mouse Interaction
- **Click**: Select cell for trajectory planning
- **ENTER**: Execute trajectory preview
- **ESC**: Cancel trajectory preview

### Configuration Features
- **Spillage Model**: Physics-based object redistribution during collection
- **Path Extension**: Smoother approach with backward curve extension
- **Post-Delivery Turn**: 180° away from target center after deliveries
- **Visualization Modes**: Potential fields, heat maps, debug overlays

## Technical Implementation Details

### Coordinate Systems
- **2D Grid**: Integer grid coordinates (0-24 range)
- **3D World**: Floating-point world coordinates with proper scaling
- **Conversion**: `coordinate_converter.py` handles transformations

### Control Architecture
1. **2D Planning**: A* pathfinding with visibility constraints and spillage prediction
2. **3D Execution**: PyBullet physics with Pure Pursuit trajectory following
3. **Synchronization**: Bidirectional state updates between 2D and 3D environments

### Alignment Gate Algorithm
```
Phase A1: Navigate base to gate point G (no alignment checking)
Phase A2: Move toward S0 while monitoring:
  - dot_align >= 0.93 (facing alignment)
  - behind = True (approaching from behind)
  - 0.20m <= distance <= 0.60m (distance band)
Phase B: Pivot to face S0 direction
Phase C: Execute task trajectory
```

### Pure Pursuit++ Features
- **Curvature-aware lookahead**: Shorter lookahead on tight curves
- **Lateral-g limiting**: Speed reduction for safe cornering
- **Yaw slew rate**: Prevents excessive heading changes
- **Feed-forward**: Anticipates curvature for smoother tracking

## Development Status

### Completed Features ✅
- Alignment gate pivot trajectory approach
- Spillage visualization in orchestrator mode
- Post-delivery turn-around for target zones
- Interactive configuration controls
- 2D/3D environment synchronization
- Path extension with configurable smoothing

### In Progress 🔄
- Pure Pursuit++ implementation (variable initialization fixed, algorithm partially complete)
- Full curvature-aware control system

### Future Enhancements 🎯
- Complete Pure Pursuit++ with Stanley cross-track correction
- Advanced spillage modeling improvements
- Performance optimization for large grids
- Additional trajectory approach methods

## Usage Instructions

1. **Start Simulation**: Run `python orchestrator.py`
2. **Configure**: Use keyboard controls (S/V/T/E) to toggle features
3. **Plan Trajectory**: Click on cells with objects
4. **Preview**: Review trajectory and spillage predictions  
5. **Execute**: Press ENTER to execute, ESC to cancel
6. **Monitor**: Watch 3D physics execution and environment updates

## File Dependencies
```
orchestrator.py → pybullet_integration.py → 2D Algorithm/main.py
                ↘ visualizer.py          ↘ 2D Algorithm/env.py
                                         ↘ coordinate_converter.py
```

This system provides a complete 2D planning + 3D execution pipeline for autonomous earth moving operations with realistic physics and spillage modeling.