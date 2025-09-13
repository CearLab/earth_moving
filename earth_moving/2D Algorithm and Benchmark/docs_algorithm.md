# Earth Moving Simulation Algorithm - Comprehensive Documentation

## Table of Contents
1. [Algorithm Overview](#1-algorithm-overview)
2. [Recent Major Enhancements](#2-recent-major-enhancements)
3. [Core Data Structures](#3-core-data-structures)
4. [Key Algorithms with Pseudocode](#4-key-algorithms-with-pseudocode)
5. [Visualization System](#5-visualization-system)
6. [Strategic Planning Infrastructure](#6-strategic-planning-infrastructure)
7. [Design Rationale](#7-design-rationale)
8. [Integration Architecture](#8-integration-architecture)
9. [Performance Optimizations](#9-performance-optimizations)

## 1. Algorithm Overview

### 1.1 Problem Statement
The earth moving simulation algorithm addresses the optimal collection and transportation of objects scattered across a 2D grid environment to a centralized target zone. The algorithm must handle:
- Path planning with potential spillage during transport
- Dynamic environment updates after path execution
- Real-time visualization and interaction
- Dual optimization objectives (delivery vs density)

### 1.2 Core Components
1. **Pathfinding System**: A* search with spillage optimization and dual value tracking
2. **Spillage Model**: Spline-based object distribution simulation with conservation
3. **Visibility System**: Cone-constrained neighbor detection with geometric optimization
4. **Heat Map System**: Flow-based potential field calculation with unified consistency
5. **Environment Management**: Incremental updates with dependency tracking
6. **Visualization System**: Real-time preview with spillage cell highlighting
7. **Multi-Agent Coordination**: Forward planning with strategic coordination
8. **State Management**: Complete environment serialization for scenario planning

### 1.3 Algorithm Modes
- **Non-Spillage Mode**: Direct object transport with memoization and early stopping
- **Spillage Mode**: Realistic transport with spline-based distribution and visualization

### 1.4 Recent Capabilities Added
- **Interactive Spillage Visualization**: Real-time preview of spillage effects during path planning
- **Strategic Planning Framework**: Multi-scenario exploration for complete aggregate transportation
- **Enhanced Multi-Agent System**: Coordination, forward planning, and collision avoidance
- **State Preservation**: Complete environment serialization for strategic analysis

## 2. Recent Major Enhancements

### 2.1 Spillage Cell Visualization System ✨ **NEW**

**Problem Solved**: Users could not see spillage effects during path preview, making it difficult to understand the impact of spillage on planned moves.

**Implementation**: Enhanced visualizer with real-time spillage cell highlighting:
```python
# In visualizer.py
def draw_preview_spillage_cells(self):
    """Draw spillage cells for path preview when in spillage mode."""
    if not self.preview_spillage_cells:
        return
        
    spillage_color = (255, 100, 255)  # Magenta highlighting
    for cell_key, spillage_info in self.preview_spillage_cells.items():
        # Semi-transparent overlay + spillage amount text
        spillage_surface = pygame.Surface((self.cell_size, self.cell_size), pygame.SRCALPHA)
        spillage_surface.fill((255, 100, 255, 80))
        self.screen.blit(spillage_surface, (x * self.cell_size, y * self.cell_size))
```

**Key Features**:
- **Magenta Cell Highlighting**: Semi-transparent overlay on spillage-affected cells
- **Spillage Amount Display**: Numerical spillage values shown in affected cells
- **Real-Time Preview**: Updates dynamically as user selects different paths
- **Integration with Path Preview**: Seamlessly integrated with existing trajectory preview

### 2.2 Fixed Spillage Visualization Coordinate System ✅ **FIXED**

**Problem Solved**: The `visualize_spillage_for_all_paths()` function displayed spillage locations horizontally flipped relative to the main grid visualization.

**Root Cause**: Matplotlib's default coordinate system (bottom-to-top) conflicted with grid visualization (top-to-bottom).

**Solution Applied**:
```python
# In env.py - visualize_spillage_for_all_paths()
plt.xlim(0, self.grid_size)
plt.ylim(0, self.grid_size)
plt.gca().invert_yaxis()  # Match grid visualization orientation
```

**Result**: Spillage curvatures now align perfectly with object locations in main visualization.

### 2.3 Enhanced Multi-Agent Coordination System 🤖 **ENHANCED**

**Capabilities Added**:
- **Forward Planning**: Agents can plan multiple moves ahead with lookahead
- **Coordination Engine**: MultiAgentCoordinator manages agent interactions
- **Collision Avoidance**: Prevents agents from interfering with each other
- **Strategic Decision Making**: Agents consider spillage effects in planning

**Key Components**:
```python
class Agent:
    def __init__(self, agent_id: str, initial_position: Tuple[int, int], 
                 capacity: int = 10, planning_horizon: int = 3):
        self.planned_actions = []  # Queue of planned actions
        self.known_agents = {}     # Other agents' states for coordination
        
class MultiAgentCoordinator:
    def coordinate_agents(self, env):
        """Coordinate all agents for the next round of actions."""
        # Share agent states and plan coordinated actions
```

### 2.4 Strategic Planning Infrastructure 🚀 **NEW FRAMEWORK**

**Major Addition**: Comprehensive infrastructure for multi-scenario strategic planning:

**State Management**:
- **Complete Serialization**: `get_state()`, `set_state()`, `copy_state()` methods
- **Environment Duplication**: Full environment copying for scenario exploration
- **State Validation**: Consistency checking between algorithm and real simulation

**Scenario Framework**:
```python
class ScenarioManager:
    """Manages multiple scenarios and their branching exploration."""
    def create_scenario(self, parent_id=None) -> str
    def execute_move_in_scenario(self, scenario_id: str, move: MoveAction) -> ExecutionResult
    def compare_scenarios(self, scenario_ids: List[str]) -> ComparisonReport
```

**Strategic Planning**: Multi-objective optimization across different move sequences.

### 2.5 Performance and Memory Optimizations ⚡ **OPTIMIZED**

**Improvements Made**:
- **Selective Updates**: Only recalculate affected cells and their dependencies
- **Lazy Computation**: Compute expensive operations on-demand
- **Memory Efficiency**: Smart cleanup of unused scenario states
- **Geometric Optimizations**: Use squared distances to avoid expensive sqrt operations

**Dependency Tracking**:
```python
# Only update cells that can see affected areas
for candidate in env.cells_with_objects:
    for affected_cell in all_affected_for_visibility:
        if IS_CELL_IN_VISIBILITY_SCOPE(candidate, affected_cell):
            ADD_TO_SET(recalculation_cells, candidate)
            break
```

### 2.6 Data Integration Enhancements 🔗 **IMPROVED**

**Fixed Data Flow Issues**:
- **Spillage Data Passing**: Fixed missing `impacted_cells` data in main.py → visualizer.py
- **Path Preview Integration**: Enhanced `get_path_for_preview()` to include all spillage information
- **Visualization Consistency**: Ensured spillage data reaches visualization layer correctly

**Before/After**:
```python
# BEFORE: Missing spillage data
path_info = {
    'path': best_current_path['path'],
    'objects': best_current_path['objects'],
    'distance': best_current_path['distance']
}

# AFTER: Complete spillage integration  
path_info = {
    'path': best_current_path['path'],
    'objects': best_current_path['objects'],
    'distance': best_current_path['distance'],
    'impacted_cells': best_current_path.get('impacted_cells', {})  # ✅ Fixed!
}
```

## 3. Core Data Structures

### 3.1 Cell Structure
```python
class Cell:
    # Position and basic attributes
    x, y: int                    # Grid coordinates
    num_objects: int             # Current objects in cell
    current_objects: int         # Objects currently available
    is_target_zone: bool         # Whether cell is inside target zone
    
    # Distance and geometry
    closest_x, closest_y: float  # Closest point on target zone boundary
    closest_distance: float      # Distance to target zone
    distance_to_target: float    # Same as closest_distance
    
    # Dual value tracking (KEY INNOVATION)
    total_objects_target: int    # Spillage-affected objects (for target estimation)
    total_objects_raw: int       # Raw objects (for propagation density)
    
    # Pathfinding results
    best_path_target: List[Cell]     # Optimal path to target zone
    best_path_highway: List[Cell]    # Optimal path to highway
    total_distance_target: float     # Total distance along target path
    total_distance_highway: float    # Total distance along highway path
    
    # Visibility and navigation
    visible_cells_target: List[Dict] # Visible neighbors toward target
    visible_cells_highway: List[Dict] # Visible neighbors toward highway
    distance_to_children_target: Dict # Distances to visible target neighbors
    distance_to_children_highway: Dict # Distances to visible highway neighbors
    
    # Flow and potential field
    velocity_target: Tuple[float, float] # Normalized velocity toward target
    velocity_highway: Tuple[float, float] # Normalized velocity toward highway
    heat_map: float              # Heat map value for highway selection
    
    # Optimization and memoization
    solved_target: bool          # Whether target path is finalized
    h_vis_target: float         # Optimistic heuristic (sum of visible objects)
    h_resolved_target: float    # Exact heuristic (actual path yield)
    
    # Spillage tracking
    impacted_cells_target: Dict  # Cells impacted by target spillage
    impacted_cells_highway: Dict # Cells impacted by highway spillage
```

### 3.2 Environment Structure
```python
class SimulationEnv:
    # Grid and geometry
    grid_size: int
    target_zone: Polygon         # Shapely polygon for target zone
    all_cells: List[Cell]        # All cells in grid
    cells_by_xy: Dict[Tuple, Cell] # Fast O(1) cell lookup
    
    # Object tracking (KEY ARCHITECTURAL DECISION)
    cells_with_objects: List[Cell]    # Regular cells with objects (pathfinding)
    target_zone_cells: List[Cell]     # Target zone cells (visualization only)
    
    # Algorithm configuration
    use_spillage_model: bool
    max_path_length_factor: float
    target_angle_tolerance: float
    highway_angle_tolerance: float
    highway_threshold: float
    
    # Spillage parameters
    agent_capacity: int
    spillage_factor: float
    min_spillage_threshold: float
```

## 4. Visualization System

### 4.1 Interactive Path Preview with Spillage Visualization

The visualization system provides real-time feedback during path planning, including spillage effects:

#### Core Visualization Components
```python
class SimulationVisualizer:
    def __init__(self, env, screen_size=500):
        # Path preview visualization
        self.preview_path = None
        self.preview_spillage_cells = {}  # Spillage cells for path preview
        
    def set_trajectory_preview(self, start_cell, path_type, path_info):
        """Set trajectory preview including spillage data."""
        self.preview_spillage_cells = path_info.get('impacted_cells', {})
        
    def draw_preview_spillage_cells(self):
        """Draw spillage cells with distinctive magenta highlighting."""
```

#### Spillage Cell Visualization Features
- **Real-Time Preview**: Shows spillage effects as user selects different paths
- **Magenta Highlighting**: Semi-transparent overlays on spillage-affected cells  
- **Spillage Amounts**: Numerical values displayed in each affected cell
- **Coordinate Consistency**: Fixed horizontal flip issue in matplotlib plots

#### Path Preview Integration
```python
# Enhanced path preview data flow
path_info = {
    'path': best_current_path['path'],
    'objects': best_current_path['objects'], 
    'distance': best_current_path['distance'],
    'impacted_cells': best_current_path.get('impacted_cells', {})  # ✅ Now included
}
```

### 4.2 Multi-Agent Visualization

Enhanced visualization support for multi-agent scenarios:

#### Agent Visualization Features
```python
def draw_agents(self):
    """Draw agents with unique colors and orientation indicators."""
    for i, agent in enumerate(self.agents):
        x, y = agent.position
        color = self.agent_colors[i % len(self.agent_colors)]
        
        # Draw agent as colored circle with orientation arrow
        pygame.draw.circle(self.screen, color, (screen_x, screen_y), agent_radius)
        
def draw_agent_paths(self):
    """Draw planned paths for all agents with distinctive colors."""
```

#### Visualization Capabilities
- **Agent Identification**: Unique colors and ID labels for each agent
- **Path Visualization**: Planned move sequences shown as colored paths
- **Real-Time Updates**: Dynamic updates as agents move and replan
- **Coordination Display**: Visual indication of agent interactions

## 5. Strategic Planning Infrastructure

### 5.1 State Management System

Complete environment state serialization for scenario exploration:

#### State Serialization Architecture
```python
def get_state(self):
    """Export complete environment state for scenario planning."""
    return {
        'grid_size': self.grid_size,
        'use_spillage_model': self.use_spillage_model,
        'cell_states': self._serialize_cell_states(),
        'cells_with_objects_coords': [(cell.x, cell.y) for cell in self.cells_with_objects],
        'target_zone_wkt': self.target_zone.wkt,
        # ... complete parameter preservation
    }

def set_state(self, state):
    """Restore environment from serialized state."""
    # Restore all parameters, cell states, and object tracking
    
def copy_state(self):
    """Create deep copy for scenario exploration."""
    return SimulationEnv.from_state(self.get_state())
```

#### Cell State Serialization
```python
def _serialize_cell_states(self):
    """Serialize all meaningful cell states."""
    cell_states = {}
    for cell in self.all_cells:
        if (cell.num_objects > 0 or hasattr(cell, 'best_path_target')):
            cell_states[(cell.x, cell.y)] = {
                'num_objects': cell.num_objects,
                'heat_map': getattr(cell, 'heat_map', 0),
                'best_path_target_coords': [(c.x, c.y) for c in cell.best_path_target],
                'total_objects_target': getattr(cell, 'total_objects_target', 0),
                'impacted_cells_target': getattr(cell, 'impacted_cells_target', {}),
                # ... complete state preservation
            }
    return cell_states
```

### 5.2 Scenario Management Framework

**Designed Architecture** (Implementation Ready):

#### Multi-Scenario Exploration
```python
class ScenarioManager:
    """Manages multiple scenarios and their branching exploration."""
    
    def __init__(self, base_env: SimulationEnv):
        self.base_state = base_env.get_state()
        self.scenarios = {}  # scenario_id -> ScenarioInstance
        
    def create_scenario(self, parent_id=None) -> str:
        """Create new scenario branch for exploration."""
        
    def execute_move_in_scenario(self, scenario_id: str, move: MoveAction):
        """Execute move in specific scenario without affecting others."""
        
    def compare_scenarios(self, scenario_ids: List[str]) -> ComparisonReport:
        """Compare scenarios across multiple evaluation metrics."""
```

#### Strategic Move Planning
```python
class StrategyPlanner:
    """High-level strategic planning for complete aggregate transportation."""
    
    def generate_move_sequence(self, strategy: PlanningStrategy) -> List[MoveAction]:
        """Generate optimal sequence for complete transportation."""
        
    def optimize_strategy(self, moves: List[MoveAction]) -> List[MoveAction]:
        """Optimize through scenario exploration."""
```

### 5.3 Integration with Real Simulation

**Framework for External Integration**:

#### Real Simulation Interface
```python
class RealSimulationInterface:
    """Interface between 2D algorithm and real simulation."""
    
    def import_simulation_state(self, sim_data: dict) -> SimulationEnv:
        """Import current state from real simulation."""
        
    def export_move_commands(self, moves: List[MoveAction]) -> dict:
        """Export move commands for real simulation execution."""
        
    def validate_state_consistency(self, real_state, algorithm_state) -> ValidationReport:
        """Validate consistency between systems."""
```

#### Strategic Planning Capabilities
- **Multi-Scenario Exploration**: Evaluate different move sequences
- **Rollback Capability**: Return to previous states for alternative exploration  
- **Strategy Optimization**: Multi-objective optimization across scenarios
- **Real-Time Integration**: Sync with external simulation systems

## 6. Key Algorithms with Pseudocode

### 6.1 Main Algorithm Loop

```pseudocode
ALGORITHM: EarthMovingSimulation
INPUT: grid_size, num_objects, target_zone_radius, use_spillage_model
OUTPUT: Interactive simulation environment

BEGIN
    // Initialize environment
    env = CREATE_ENVIRONMENT(grid_size, target_zone_radius)
    SPAWN_RANDOM_OBJECTS(env, num_objects)
    INITIALIZE_CELLS_WITH_TARGET_DISTANCES(env)
    
    // Precompute visibility for all cells
    FOR each cell IN env.cells_with_objects:
        cell.visible_cells_target = CALCULATE_TARGET_ZONE_VISIBILITY(cell)
        cell.h_vis_target = SUM(visible_cell.num_objects for each visible_cell)
    END FOR
    
    // Compute potential field and paths
    CALCULATE_POTENTIAL_FIELD(env, use_spillage_model)
    CALCULATE_VELOCITY_FIELD(env)
    UPDATE_HEAT_MAP(env)
    CALCULATE_HIGHWAY_PATHS(env, use_spillage_model)
    
    // Main interaction loop
    WHILE simulation_running:
        DISPLAY_VISUALIZATION(env)
        
        IF user_clicks_cell(clicked_cell):
            path_type = GET_USER_PATH_CHOICE() // 'target' or 'highway'
            
            // Get current optimal path from environment
            IF path_type == 'target':
                current_paths = env.GET_PATH_FOR_PREVIEW(clicked_cell, "target")
            ELSE:
                current_paths = env.GET_PATH_FOR_PREVIEW(clicked_cell, "highway")
            END IF
            
            // Show preview and wait for confirmation
            SHOW_TRAJECTORY_PREVIEW(current_paths[0])
            
            IF user_confirms_execution():
                EXECUTE_PATH(clicked_cell, path_type, use_spillage_model, current_paths[0])
                UPDATE_ENVIRONMENT(env) // Selective recalculation
            END IF
        END IF
    END WHILE
END
```

### 6.2 Spillage-Optimized Pathfinding

```pseudocode
ALGORITHM: CalculatePotentialField
INPUT: env, use_spillage_model, affected_cells
OUTPUT: Updated cell paths and object estimates

BEGIN
    env.use_spillage_model = use_spillage_model
    
    IF affected_cells is None:
        affected_cells = env.cells_with_objects
    END IF
    
    // Sort by distance for optimal processing order
    SORT(affected_cells, key=lambda cell: cell.distance_to_target)
    
    // Clear old flow tracking values
    FOR each cell IN affected_cells:
        cell.total_objects_path_target = 0
    END FOR
    
    FOR each cell IN affected_cells:
        // Skip cells without visibility
        IF NOT cell.visible_cells_target:
            CONTINUE
        END IF
        
        // Get candidate paths using A*
        candidate_paths = A_STAR_SEARCH_TARGET(cell, env.target_zone, env.max_path_length_factor, env)
        
        IF NOT candidate_paths:
            CONTINUE
        END IF
        
        // Path selection based on mode
        IF use_spillage_model:
            // DUAL PATH SELECTION (KEY INNOVATION)
            best_target_path = None
            best_raw_path = None
            max_estimated = 0
            max_raw = 0
            
            FOR each path_info IN candidate_paths:
                path = path_info.path
                raw_objects = path_info.objects
                
                // Simulate spillage for this path
                waypoints = [(cell.x, cell.y) for cell in path]
                objects_at_cells = {(cell.x, cell.y): cell.num_objects for cell in path if cell.num_objects > 0}
                
                spline_points, impacted_cells, estimated_objects, total_objects = SIMULATE_SPILLAGE(
                    waypoints, objects_at_cells, env.agent_capacity, env.spillage_factor, env.min_spillage_threshold
                )
                
                // Select best path for TARGET ESTIMATION (spillage-affected)
                IF estimated_objects > max_estimated:
                    max_estimated = estimated_objects
                    best_target_path = path
                    best_impacted_cells = impacted_cells
                END IF
                
                // Select best path for PROPAGATION (raw objects)
                IF raw_objects > max_raw:
                    max_raw = raw_objects
                    best_raw_path = path
                END IF
            END FOR
            
            // Use target path for main behavior
            cell.best_path_target = best_target_path
            cell.total_objects_target = max_estimated        // Spillage-affected
            cell.total_objects_raw = max_raw                 // Raw objects
            cell.impacted_cells_target = best_impacted_cells
            
        ELSE:
            // Non-spillage mode: same path for both purposes
            best_path = candidate_paths[0].path
            total_objects = candidate_paths[0].objects
            
            cell.best_path_target = best_path
            cell.total_objects_target = total_objects   // Same value
            cell.total_objects_raw = total_objects      // Same value
            cell.impacted_cells_target = {}            // No spillage
            
            // Mark as solved for memoization
            cell.solved_target = True
            cell.h_resolved_target = total_objects
        END IF
    END FOR
    
    // Propagate values through all computed paths
    PROPAGATE_TOTAL_OBJECTS_PATH_TARGET(env)
END
```

### 6.3 A* Search with Optimization

```pseudocode
ALGORITHM: AStarSearchTarget
INPUT: start_cell, target_zone, max_path_length_factor, env
OUTPUT: List of optimal paths with objects and distances

BEGIN
    // Distance constraint
    max_allowed_distance = INFINITY
    IF max_path_length_factor > 0:
        max_allowed_distance = max_path_length_factor * start_cell.distance_to_target
    END IF
    
    // Initialize priority queue and tracking
    open_set = PRIORITY_QUEUE()
    HEAP_PUSH(open_set, (-start_cell.num_objects, start_cell.distance_to_target, 0, 
                         start_cell, [start_cell], start_cell.num_objects, 0.0))
    
    best_collected = {(start_cell.x, start_cell.y): start_cell.num_objects}
    best_paths = []
    best_objects = 0
    
    WHILE open_set NOT empty:
        neg_f, tie_dist, tie_id, current, path, collected_so_far, distance_so_far = HEAP_POP(open_set)
        
        // EARLY STOPPING OPTIMIZATION (non-spillage mode only)
        IF NOT env.use_spillage_model AND current.solved_target:
            // Stitch current path prefix with precomputed optimal suffix
            suffix = current.best_path_target[1:]  // Skip duplicate current cell
            full_path = path + suffix
            
            // Calculate exact objects: prefix objects + exact suffix yield
            total_objects = (collected_so_far - current.num_objects) + current.total_objects_target
            
            // Calculate remaining distance along precomputed path
            remaining_distance = 0
            FOR i FROM 0 TO LENGTH(suffix) - 1:
                u, v = suffix[i], suffix[i+1]
                remaining_distance += current.distance_to_children_target[(v.x, v.y)]
            END FOR
            
            total_distance = distance_so_far + remaining_distance
            
            IF total_distance <= max_allowed_distance:
                ADD_TO_RESULTS(best_paths, full_path, total_objects, total_distance)
                CONTINUE  // Skip normal expansion
            END IF
        END IF
        
        // Goal test: no more successors toward target
        IF NOT current.visible_cells_target:
            ADD_TO_RESULTS(best_paths, path, collected_so_far, distance_so_far)
            CONTINUE
        END IF
        
        // Expand successors
        FOR each neighbor_info IN current.visible_cells_target:
            child = neighbor_info.cell
            edge_distance = current.distance_to_children_target[(child.x, child.y)]
            new_distance = distance_so_far + edge_distance
            
            IF new_distance > max_allowed_distance:
                CONTINUE
            END IF
            
            // SMART HEURISTICS (non-spillage mode only)
            IF NOT env.use_spillage_model AND child.solved_target:
                heuristic = child.h_resolved_target  // Exact heuristic
            ELSE:
                heuristic = child.h_vis_target       // Optimistic heuristic
            END IF
            
            // Calculate objects collected
            child_objects = child.num_objects
            IF POINT_IN_TARGET_ZONE(child):
                child_objects = 0  // Don't double-count target zone objects
            END IF
            
            new_collected = collected_so_far + child_objects
            f_score = heuristic + new_collected
            
            // Relaxation: only proceed if we improved collection to this cell
            cell_key = (child.x, child.y)
            IF new_collected <= best_collected.get(cell_key, -1):
                CONTINUE
            END IF
            best_collected[cell_key] = new_collected
            
            // Add to queue
            tie_id += 1
            HEAP_PUSH(open_set, (-f_score, child.distance_to_target, tie_id,
                                 child, path + [child], new_collected, new_distance))
        END FOR
    END WHILE
    
    // Sort results: maximize objects, minimize distance
    SORT(best_paths, key=lambda p: (-p.objects, p.distance))
    RETURN best_paths
END
```

### 6.4 Spillage Simulation

```pseudocode
ALGORITHM: SimulateSpillage
INPUT: waypoints, objects_at_cells, agent_capacity, spillage_factor, min_spillage_threshold
OUTPUT: spline_points, impacted_cells, objects_at_target, total_objects

BEGIN
    // Create smooth spline path
    spline_points, curvature, success = SMOOTH_PATH_WITH_SPLINE(waypoints)
    
    IF NOT success:
        // Fallback for edge cases
        IF LENGTH(waypoints) == 1:
            target_cell = waypoints[0]
            total_objects = SUM(objects_at_cells.values())
            RETURN [(target_cell[0] + 0.5, target_cell[1] + 0.5)], {target_cell: total_objects}, total_objects, total_objects
        ELSE:
            RETURN [], {}, 0, SUM(objects_at_cells.values())
        END IF
    END IF
    
    max_possible_objects = SUM(objects_at_cells.values())
    impacted_cells = {}
    total_objects = 0
    total_spilled = 0
    visited_cells = SET()
    
    // Simulate agent movement along spline
    FOR i FROM 0 TO LENGTH(spline_points) - 1:
        px, py = spline_points[i]
        cell_x, cell_y = FLOOR(px), FLOOR(py)
        
        // Pick up objects from original cells
        IF (cell_x, cell_y) IN objects_at_cells AND (cell_x, cell_y) NOT IN visited_cells:
            total_objects += objects_at_cells[(cell_x, cell_y)]
            ADD_TO_SET(visited_cells, (cell_x, cell_y))
        END IF
        
        // Calculate spillage based on curvature
        spilled_objects = spillage_factor * curvature[i] * total_objects
        spilled_objects = MIN(spilled_objects, total_objects)
        
        // Apply spillage if above threshold
        IF spilled_objects >= min_spillage_threshold:
            impacted_cells[(cell_x, cell_y)] = GET(impacted_cells, (cell_x, cell_y), 0) + spilled_objects
            total_objects -= spilled_objects
            total_spilled += spilled_objects
        END IF
    END FOR
    
    // Ensure remaining objects reach target
    target_cell_x, target_cell_y = FLOOR(spline_points[-1][0]), FLOOR(spline_points[-1][1])
    impacted_cells[(target_cell_x, target_cell_y)] = GET(impacted_cells, (target_cell_x, target_cell_y), 0) + total_objects
    
    // OBJECT CONSERVATION VERIFICATION (KEY SAFETY FEATURE)
    total_distributed = SUM(impacted_cells.values())
    IF ABS(total_distributed - max_possible_objects) > 0.01:
        adjustment = max_possible_objects - total_distributed
        impacted_cells[(target_cell_x, target_cell_y)] += adjustment
        PRINT("Conservation corrected by", adjustment)
    END IF
    
    // Conservative rounding to preserve integer objects
    rounded_cells = {}
    total_fractional_loss = 0
    
    FOR each (cell, value) IN impacted_cells:
        rounded_value = FLOOR(value)
        fractional_part = value - rounded_value
        total_fractional_loss += fractional_part
        IF rounded_value > 0:
            rounded_cells[cell] = rounded_value
        END IF
    END FOR
    
    // Distribute fractional losses to preserve conservation
    IF total_fractional_loss >= 0.5:
        fractional_cells = SORT_BY_FRACTIONAL_PART(impacted_cells, descending=True)
        extra_objects_needed = ROUND(total_fractional_loss)
        
        FOR i FROM 0 TO MIN(extra_objects_needed, LENGTH(fractional_cells)) - 1:
            cell = fractional_cells[i].cell
            rounded_cells[cell] = GET(rounded_cells, cell, 0) + 1
        END FOR
    END IF
    
    objects_at_target = GET(rounded_cells, (target_cell_x, target_cell_y), 0)
    RETURN spline_points, rounded_cells, objects_at_target, max_possible_objects
END
```

### 6.5 Visibility Calculation with Cone Constraints

```pseudocode
ALGORITHM: CalculateTargetZoneVisibility
INPUT: current_cell, angle_tolerance
OUTPUT: visible_cells, distance_to_children

BEGIN
    // Immediate return for target zone cells (KEY ARCHITECTURAL DECISION)
    IF current_cell.is_target_zone:
        RETURN [], {}
    END IF
    
    visible_cells = []
    distance_to_children = {}
    
    // Use cell centers for geometric calculations
    source_x, source_y = current_cell.x + 0.5, current_cell.y + 0.5
    target_x, target_y = current_cell.closest_x, current_cell.closest_y
    
    // Precompute vector to target and geometric constraints
    vector_x, vector_y = target_x - source_x, target_y - source_y
    vector_length_squared = vector_x * vector_x + vector_y * vector_y
    
    IF vector_length_squared == 0:
        RETURN visible_cells, distance_to_children  // On target, no direction
    END IF
    
    cos_angle_tolerance = COS(RADIANS(angle_tolerance))
    
    // Check each potential neighbor
    FOR each neighbor IN env.cells_with_objects:
        IF neighbor == current_cell:
            CONTINUE
        END IF
        
        // Skip target zone cells - they're destinations, not path steps
        IF neighbor.is_target_zone:
            CONTINUE
        END IF
        
        neighbor_x, neighbor_y = neighbor.x + 0.5, neighbor.y + 0.5
        to_neighbor_x, to_neighbor_y = neighbor_x - source_x, neighbor_y - source_y
        
        // CONE CONSTRAINT: Angular alignment check
        dot_product = to_neighbor_x * vector_x + to_neighbor_y * vector_y
        neighbor_distance_squared = to_neighbor_x * to_neighbor_x + to_neighbor_y * to_neighbor_y
        
        IF neighbor_distance_squared == 0:
            CONTINUE
        END IF
        
        // Efficient cone check using squared distances (avoids sqrt)
        in_cone = dot_product >= SQRT(neighbor_distance_squared * vector_length_squared) * cos_angle_tolerance
        
        // GATE CONSTRAINT: Distance within reasonable range
        neighbor_distance = SQRT(neighbor_distance_squared)
        within_gate = neighbor_distance <= current_cell.closest_distance * env.target_zone_gate_factor
        
        // PROGRESS CONSTRAINT: Monotonic progress toward target
        makes_progress = neighbor.distance_to_target <= current_cell.distance_to_target
        
        // Include if all constraints satisfied
        IF in_cone AND within_gate AND makes_progress:
            ADD_TO_LIST(visible_cells, {cell: neighbor})
            distance_to_children[(neighbor.x, neighbor.y)] = neighbor_distance
        END IF
    END FOR
    
    // Fallback: include closest boundary cell if no visible cells found
    IF LENGTH(visible_cells) == 0:
        boundary_x, boundary_y = INT(current_cell.closest_x), INT(current_cell.closest_y)
        boundary_cell = env.GET_CELL(boundary_x, boundary_y)
        IF boundary_cell:
            ADD_TO_LIST(visible_cells, {cell: boundary_cell})
            distance_to_children[(boundary_x, boundary_y)] = current_cell.closest_distance
        END IF
    END IF
    
    RETURN visible_cells, distance_to_children
END
```

### 6.6 Unified Heat Map Calculation

```pseudocode
ALGORITHM: UpdateHeatMap
INPUT: env
OUTPUT: Updated heat_map values for all cells

BEGIN
    max_potential = 0
    
    FOR each cell IN env.all_cells:
        cell.heat_map = 0
        
        // Skip cells inside target zone
        IF POINT_IN_TARGET_ZONE(cell):
            CONTINUE
        END IF
        
        // Calculate direction vector from target to current cell
        from_target_x = (cell.x + 0.5) - cell.closest_x
        from_target_y = (cell.y + 0.5) - cell.closest_y
        magnitude_from_target = SQRT(from_target_x^2 + from_target_y^2)
        
        IF magnitude_from_target == 0:
            CONTINUE
        END IF
        
        unit_from_target_x = from_target_x / magnitude_from_target
        unit_from_target_y = from_target_y / magnitude_from_target
        
        max_heat_value = 0
        
        // Check contributions from all candidate cells
        FOR each candidate IN env.cells_with_objects:
            IF candidate == cell:
                CONTINUE
            END IF
            
            // Skip candidate cells inside target zone
            IF POINT_IN_TARGET_ZONE(candidate):
                CONTINUE
            END IF
            
            // Vector from candidate to current cell
            to_current_x = (cell.x + 0.5) - (candidate.x + 0.5)
            to_current_y = (cell.y + 0.5) - (candidate.y + 0.5)
            magnitude_to_current = SQRT(to_current_x^2 + to_current_y^2)
            
            IF magnitude_to_current == 0:
                CONTINUE
            END IF
            
            unit_to_current_x = to_current_x / magnitude_to_current
            unit_to_current_y = to_current_y / magnitude_to_current
            
            // Check alignment between flow directions
            inverse_unit_to_current_x = -unit_to_current_x
            inverse_unit_to_current_y = -unit_to_current_y
            
            dot_from_target = MAX(0, unit_from_target_x * inverse_unit_to_current_x + 
                                    unit_from_target_y * inverse_unit_to_current_y)
            
            // Only consider aligned candidates (75-degree tolerance)
            IF dot_from_target > COS(RADIANS(75)):
                // Check candidate velocity
                velocity_x, velocity_y = candidate.velocity_target
                IF ABS(velocity_x) < 1e-9 AND ABS(velocity_y) < 1e-9:
                    CONTINUE
                END IF
                
                // Calculate velocity alignment
                dot_velocity = MAX(0, velocity_x * unit_to_current_x + velocity_y * unit_to_current_y)
                
                // PATH INTERFERENCE FILTER (UNIFIED FOR BOTH MODES)
                first_child = candidate.next_child_target
                IF first_child:
                    distance_to_first_child = SQRT((first_child.x - candidate.x)^2 + (first_child.y - candidate.y)^2)
                    IF distance_to_first_child < magnitude_to_current:
                        CONTINUE  // Skip if candidate's path is shorter than flow distance
                    END IF
                END IF
                
                // UNIFIED VALUE CALCULATION (KEY CONSISTENCY FIX)
                psi = candidate.total_objects_path_target
                IF psi > 0 AND NOT candidate.best_path_target:
                    psi = candidate.num_objects  // Fallback for stale data
                END IF
                
                // UNIFIED HEAT CALCULATION (same for both modes)
                heat_value = dot_velocity * psi
                max_heat_value = MAX(max_heat_value, heat_value)
            END IF
        END FOR
        
        cell.heat_map = max_heat_value
        max_potential = MAX(max_potential, max_heat_value)
    END FOR
    
    // Set highway threshold as percentage of maximum potential
    env.highway_threshold = max_potential * env.highway_threshold_ratio
    
    PRINT("Heat map updated: max_potential =", max_potential, "threshold =", env.highway_threshold)
END
```

### 6.7 Selective Environment Update

```pseudocode
ALGORITHM: UpdateEnvironment
INPUT: env (with affected_cells set from last path execution)
OUTPUT: Selectively updated environment state

BEGIN
    IF NOT env.affected_cells:
        PRINT("No affected cells, skipping update")
        RETURN
    END IF
    
    // Ensure cell tracking consistency
    removed_count, added_count = UPDATE_CELLS_WITH_OBJECTS_TRACKING(env)
    
    // Separate different types of affected cells
    all_affected_cells_raw = SET(env.affected_cells)
    emptied_cells = SET()         // Cells that had objects but now don't
    cells_with_objects_affected = SET()  // Cells that still/now have objects
    
    FOR each cell IN all_affected_cells_raw:
        IF cell.num_objects > 0:
            ADD_TO_SET(cells_with_objects_affected, cell)
        ELSE:
            ADD_TO_SET(emptied_cells, cell)
        END IF
    END FOR
    
    // Add spillage cells (these always have objects)
    spillage_cells = SET()
    IF env.spillage_affected_cells:
        spillage_cells = SET(env.spillage_affected_cells)
        spillage_cells_with_objects = FILTER(spillage_cells, lambda cell: cell.num_objects > 0)
        UNION(cells_with_objects_affected, spillage_cells_with_objects)
    END IF
    
    // Filter out target zone cells - they don't need pathfinding
    all_direct_affected = FILTER(cells_with_objects_affected, lambda cell: NOT cell.is_target_zone)
    
    // DEPENDENCY TRACKING: Find cells that need recalculation
    all_affected_for_visibility = UNION(emptied_cells, cells_with_objects_affected)
    all_affected_for_visibility = FILTER(all_affected_for_visibility, lambda cell: NOT cell.is_target_zone)
    
    recalculation_cells = SET()
    
    FOR each candidate IN env.cells_with_objects:
        IF candidate IN all_direct_affected:
            CONTINUE  // Already directly affected
        END IF
        
        IF candidate.is_target_zone:
            CONTINUE  // Target zone cells don't need pathfinding
        END IF
        
        // Check if candidate can see any affected cell
        FOR each affected_cell IN all_affected_for_visibility:
            IF IS_CELL_IN_VISIBILITY_SCOPE(candidate, affected_cell):
                ADD_TO_SET(recalculation_cells, candidate)
                BREAK  // Found one dependency, that's enough
            END IF
        END FOR
    END FOR
    
    // Combine for total affected cells
    all_affected_cells = UNION(all_direct_affected, recalculation_cells)
    
    // Store for visualization
    env.direct_affected_cells = LIST(emptied_cells)
    env.spillage_cells = LIST(spillage_cells)
    env.recalculation_cells = LIST(recalculation_cells)
    
    // INVALIDATE MEMOIZATION (non-spillage mode only)
    IF NOT env.use_spillage_model:
        FOR each cell IN all_affected_cells:
            cell.solved_target = False
            cell.h_resolved_target = 0
        END FOR
    END IF
    
    // STEP 1: Refresh visibility using current environment state
    FOR each cell IN all_affected_cells:
        IF cell.num_objects > 0:
            cell.visible_cells_target, cell.distance_to_children_target = 
                CALCULATE_TARGET_ZONE_VISIBILITY(cell, env.target_angle_tolerance)
            cell.h_vis_target = SUM(neighbor.cell.num_objects for neighbor in cell.visible_cells_target)
        END IF
    END FOR
    
    // STEP 2: Recalculate paths for affected cells
    CALCULATE_POTENTIAL_FIELD(env, env.use_spillage_model, all_affected_cells)
    
    // STEP 3: Clear stale propagation and propagate fresh values
    FOR each cell IN all_affected_cells:
        cell.total_objects_path_target = 0
    END FOR
    
    PROPAGATE_TOTAL_OBJECTS_PATH_TARGET_SELECTIVE(env, all_affected_cells)
    
    // STEP 4: Update velocity field
    CALCULATE_VELOCITY_FIELD(env, affected_cells=all_affected_cells)
    
    // STEP 5: Update heat map (global recalculation needed)
    UPDATE_HEAT_MAP(env)
    
    // STEP 6: Recalculate highway paths
    CALCULATE_PATH_TO_HIGHWAY(env, env.use_spillage_model)
    
    // STEP 7: Clean up tracking lists
    env.cells_with_objects = FILTER(env.cells_with_objects, lambda cell: cell.num_objects > 0)
    env.target_zone_cells = FILTER(env.target_zone_cells, lambda cell: cell.num_objects > 0)
    
    PRINT("Environment update complete")
END
```

## 7. Design Rationale

### 7.1 Why Dual Value Tracking?

**Challenge**: The algorithm needs to optimize for two different objectives:
1. **Target Estimation**: How many objects will actually reach the target zone (considering spillage)
2. **Density Propagation**: What is the raw object density for flow calculations (ignoring spillage)

**Solution**: Separate tracking variables:
- `total_objects_target`: Spillage-affected objects for target estimation
- `total_objects_raw`: Raw objects for propagation and heat map calculations

**Benefits**:
- Eliminates optimization conflicts
- Allows spillage model to work correctly with heat map system
- Maintains algorithmic consistency across modes

### 7.2 Why Separate Target Zone Cell Lists?

**Challenge**: Cells inside the target zone should be:
- Visible for visualization purposes
- Excluded from pathfinding calculations (they're destinations, not intermediate steps)

**Solution**: Separate lists:
- `cells_with_objects`: Regular cells for pathfinding
- `target_zone_cells`: Target zone cells for visualization only

**Benefits**:
- Clean architectural separation
- Prevents pathfinding confusion
- Maintains complete visualization

### 7.3 Why Cone-Based Visibility?

**Challenge**: Unrestricted visibility leads to:
- Inefficient backwards movement
- Non-monotonic progress toward objectives
- Exponential search space

**Solution**: Angular cone constraints with three components:
1. **Cone Constraint**: Angular tolerance toward objective
2. **Gate Constraint**: Maximum distance from observer
3. **Progress Constraint**: Monotonic progress toward objective

**Benefits**:
- Ensures sensible movement patterns
- Reduces search space significantly
- Maintains optimality within constraints
- Configurable for different scenarios

### 7.4 Why Spillage-Optimized Path Selection?

**Challenge**: Standard A* maximizes raw object collection, but spillage model means:
- Path with more raw objects might deliver fewer objects due to spillage
- Different paths have different spillage characteristics
- Need to balance collection vs delivery

**Solution**: Spillage simulation during path evaluation:
1. Get candidate paths from A*
2. Simulate spillage for each path
3. Select path that maximizes delivered objects (not collected objects)

**Benefits**:
- Realistic optimization for spillage scenarios
- Maintains optimality under spillage model
- Allows comparison of spillage vs non-spillage approaches

### 7.5 Why Selective Environment Updates?

**Challenge**: Full environment recalculation after each path execution:
- Computationally expensive for large environments
- Most cells unaffected by individual path executions
- Prevents real-time interaction

**Solution**: Dependency tracking system:
1. Track directly affected cells (path execution)
2. Find cells that can see affected cells (dependency)
3. Only recalculate paths for affected + dependent cells

**Benefits**:
- Major performance improvement (O(affected) vs O(all))
- Maintains correctness through dependency tracking
- Enables real-time interaction with large environments

### 7.6 Why Unified Heat Map Calculation?

**Challenge**: Different heat map calculations for spillage vs non-spillage modes:
- Inconsistent behavior confuses users
- Different visualization for same underlying flow
- Algorithmic complexity

**Solution**: Single calculation method using propagated values:
- Same object value source (propagated path targets)
- Same distance constraints
- Same flow alignment calculations

**Benefits**:
- Consistent user experience
- Simplified algorithm
- Easier debugging and validation

## 8. Integration Architecture

### 8.1 Component Dependencies

```
main.py (Entry Point)
├── env.py (Core Algorithm)
│   ├── cell.py (Data Structures)
│   ├── search.py (A* Algorithms)
│   └── spillage_model.py (Spillage Simulation)
└── visualizer.py (User Interface)
```

### 8.2 Data Flow

```
1. Initialization:
   main.py → env.py → cell.py (create environment and cells)

2. Precomputation:
   env.py → search.py (visibility calculations)
   env.py → search.py → spillage_model.py (path planning with spillage)

3. User Interaction:
   main.py → visualizer.py (display and input)
   main.py → env.py (path preview and execution)
   env.py → search.py (selective recalculation)

4. Path Preview:
   main.py → env.get_path_for_preview() (unified path access)
   env.py uses pre-computed paths (no redundant calculations)

5. Visualization:
   visualizer.py → env.py (read state)
   visualizer.py → cell.py (access cell data)
```

### 8.3 Configuration Management

All algorithm parameters centralized in `main.py`:
```python
TARGET_ANGLE_TOLERANCE = 45      # Visibility cone angle
HIGHWAY_ANGLE_TOLERANCE = 60     # Highway visibility cone angle
HIGHWAY_MIN_HEAT_RATIO = 0.3     # Minimum heat threshold
HIGHWAY_THRESHOLD_RATIO = 0.5    # Highway selection threshold
HIGHWAY_HEAT_WEIGHT = 0.7        # Heat vs distance weighting
HIGHWAY_DISTANCE_WEIGHT = 0.3    # Distance weighting
```

**Benefits**:
- Single point of configuration
- Easy parameter tuning
- Consistent values across components

## 9. Performance Optimizations

### 9.1 Memoization and Caching

**A* Early Stopping** (`search.py`):
- Cache solved cells with exact heuristics
- Stitch partial paths with precomputed suffixes
- Skip redundant computation for solved subproblems

**Pre-computed Path Usage**:
- Use already-computed spillage-optimized paths from `cell.best_path_target`
- Eliminate redundant spillage calculations during UI interactions
- Consistent path representation between computation and display

### 9.2 Selective Updates

**Affected Cell Tracking**:
- Only recalculate paths for directly affected cells
- Find dependent cells through visibility relationships
- Avoid full environment recalculation

**Incremental Propagation**:
- Propagate only through affected paths
- Preserve unaffected computations
- Maintain global consistency with minimal work

### 9.3 Geometric Optimizations

**Squared Distance Calculations**:
- Use squared distances for comparisons
- Avoid expensive sqrt operations
- Only compute actual distance when needed

**Precomputed Vectors**:
- Cache direction vectors and distances
- Reuse geometric calculations
- Minimize redundant trigonometric operations

### 9.4 Data Structure Optimizations

**O(1) Cell Lookup**:
```python
cells_by_xy = {(cell.x, cell.y): cell for cell in all_cells}
```

**Adjacent Neighbor Caching**:
```python
# Precompute 8-connected neighbors for each cell
for cell in all_cells:
    cell.adjacent_neighbors = [neighbor for neighbor in get_8_connected(cell)]
```

**Visibility List Caching**:
- Cache visible cells for each observer
- Recompute only when environment changes
- Fast visibility queries for dependency tracking

## Conclusion

This algorithm represents a comprehensive solution to the earth moving problem with several key innovations:

1. **Dual Value System**: Enables optimization for both delivery and density objectives
2. **Spillage Integration**: Realistic modeling of object transport with loss
3. **Selective Updates**: Major performance improvement for real-time interaction
4. **Unified Calculations**: Consistent behavior across different modes
5. **Comprehensive Optimization**: Multiple levels of memoization and caching

The architecture is modular, well-documented, and suitable for both research and practical applications. The algorithm handles edge cases robustly and provides extensive debugging and visualization capabilities.