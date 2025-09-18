"""
Comprehensive Interactive Demo
Main entry point combining all functionality: visual interface, benchmarking, analysis
Replaces all separate demo files with one complete solution
"""

import json
import time
import pygame
import math
import numpy as np
from pathlib import Path
from typing import Optional
from scipy.spatial import ConvexHull

class StateJSONEncoder(json.JSONEncoder):
    """Custom JSON encoder for environment state serialization"""
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return obj.__dict__
        elif isinstance(obj, tuple):
            return list(obj)
        elif hasattr(obj, 'wkt'):  # Shapely geometry objects
            return obj.wkt
        return super().default(obj)

def convert_tuple_keys_for_json(obj):
    """Convert tuple keys to strings for JSON serialization"""
    if isinstance(obj, dict):
        new_dict = {}
        for key, value in obj.items():
            if isinstance(key, tuple):
                # Convert tuple key to string like "(x,y)"
                str_key = f"({key[0]},{key[1]})"
            else:
                str_key = str(key)
            new_dict[str_key] = convert_tuple_keys_for_json(value)
        return new_dict
    elif isinstance(obj, list):
        return [convert_tuple_keys_for_json(item) for item in obj]
    else:
        return obj

def convert_string_keys_to_tuples(obj):
    """Convert string keys back to tuples after JSON deserialization"""
    if isinstance(obj, dict):
        new_dict = {}
        for key, value in obj.items():
            # Check if key looks like a tuple string "(x,y)"
            if isinstance(key, str) and key.startswith('(') and key.endswith(')') and ',' in key:
                try:
                    # Parse "(x,y)" back to tuple
                    coords = key[1:-1].split(',')
                    if len(coords) == 2:
                        tuple_key = (int(coords[0]), int(coords[1]))
                    else:
                        tuple_key = key  # Keep as string if parsing fails
                except (ValueError, IndexError):
                    tuple_key = key  # Keep as string if parsing fails
            else:
                tuple_key = key
            new_dict[tuple_key] = convert_string_keys_to_tuples(value)
        return new_dict
    elif isinstance(obj, list):
        return [convert_string_keys_to_tuples(item) for item in obj]
    else:
        return obj

# Import core components
from core_env import SimulationEnv
from strategic_scenario_manager import ScenarioManager
from strategic_strategy_planner import StrategyPlanner, PlanningStrategy
from strategic_move_history import MoveHistoryTracker
from strategic_orchestrator import StrategicOrchestrator, OperationConfig, OperationMode

class ComprehensiveDemo:
    """Complete demo with all features: visual, analysis, benchmarking, save/load"""
    
    def __init__(self):
        self.storage_path = Path("./demo_storage")
        self.storage_path.mkdir(exist_ok=True)
        
        # Core components
        self.env = None
        self.visualizer = None
        self.scenario_manager = None
        self.strategy_planner = None
        self.move_history = None
        self.orchestrator = None
        
        # State management
        self.current_scenario_id = None
        self.selected_strategy = PlanningStrategy.GREEDY_NEAREST
        self.use_spillage_model = True
        
        print(">>> Comprehensive Interactive Demo Initialized")
    
    def create_simulation_quick(self, use_spillage=False, grid_size=25, object_count=55, seed=31, 
                                use_suffix_stitching=True, use_affected_only_updates=True):
        """Create simulation quickly without heavy calculations"""
        print(f"\n=== Quick Earth Moving Simulation ===")
        print(f"Spillage model: {'ENABLED' if use_spillage else 'DISABLED'}")
        print(f"Grid size: {grid_size}x{grid_size}, Objects: {object_count}, Seed: {seed}")
        self.use_spillage_model = use_spillage
        
        print("Initializing environment...")
        
        # Use provided parameters for flexible benchmarking
        self.env = SimulationEnv(
            grid_size=grid_size,
            target_zone_radius=max(2, grid_size // 8),  # Scale target zone with grid size
            agent_positions=None,
            num_random_objects=object_count,
            seed=seed,
            max_path_length_factor=2,
            target_angle_tolerance=45,
            highway_angle_tolerance=60,
            highway_min_heat_ratio=0.3,
            highway_threshold_ratio=0.5,
            highway_heat_weight=0.7,
            highway_distance_weight=0.3,
            use_suffix_stitching=use_suffix_stitching
        )
        
        print("Environment initialized!")
        print(f"Number of cells with objects: {len(self.env.cells_with_objects)}")
        
        # Show first few cells
        for i, cell in enumerate(self.env.cells_with_objects[:5]):
            print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")
        if len(self.env.cells_with_objects) > 5:
            print(f"... and {len(self.env.cells_with_objects)-5} more cells")
        
        # Skip heavy calculations for quick start - calculate only when needed
        print("Skipping heavy calculations for quick start...")
        print("(Calculations will be done on-demand when executing paths)")
        
        # Initialize strategic components
        self.scenario_manager = ScenarioManager(self.env, str(self.storage_path))
        self.strategy_planner = StrategyPlanner(self.scenario_manager)
        self.move_history = MoveHistoryTracker(str(self.storage_path))
        
        # Create orchestrator
        config = OperationConfig(
            operation_mode=OperationMode.STANDALONE,
            planning_strategy=self.selected_strategy,
            max_operation_time=300.0
        )
        
        self.orchestrator = StrategicOrchestrator(
            base_env=self.env,
            config=config,
            storage_path=str(self.storage_path)
        )
        
        self.current_scenario_id = self.scenario_manager.root_scenario_id
        
        # Initialize visualization if available
        try:
            from core_visualizer import SimulationVisualizer
            self.visualizer = SimulationVisualizer(self.env, screen_size=800)
            print("Visualization initialized!")
        except ImportError:
            print("Warning: Visualization not available (core_visualizer.py not found)")
            self.visualizer = None
        
        print("Quick simulation setup complete!")
        
        # Ensure scenario IDs are synchronized
        self._synchronize_scenario_ids()
        
        return True
    
    def calculate_on_demand(self):
        """Calculate strategic fields on-demand"""
        if not hasattr(self.env, '_fields_calculated'):
            print("\nCalculating strategic fields (this may take a moment)...")
            
            # Precompute visibility for all cells
            print("- Calculating visibility...")
            for cell in self.env.cells_with_objects:
                closest_point_target = self.env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
                tx, ty = int(closest_point_target[0]), int(closest_point_target[1])
                target_cell_target = self.env.get_cell(tx, ty)
                visible_cells_target, distance_to_children_target = self.env.calculate_target_zone_visibility(
                    cell, angle_tolerance=45)
                cell.visible_cells_target = visible_cells_target
                cell.distance_to_children_target = distance_to_children_target
                cell.h_vis_target = sum(n["cell"].num_objects for n in cell.visible_cells_target)
            
            self.env.audit_visibility()
            
            # Calculate fields
            print("- Calculating potential field...")
            self.env.calculate_potential_field(use_spillage_model=self.use_spillage_model, visualize=False)
            
            print("- Calculating velocity field...")
            self.env.calculate_velocity_field()
            
            print("- Updating heat map...")
            self.env.update_heat_map()
            
            print("- Calculating highway paths...")
            self.env.calculate_path_to_highway(use_spillage_model=self.use_spillage_model)
            
            self.env._fields_calculated = True
            print("Strategic calculations complete!")
    
    def calculate_strategic_fields_now(self):
        """Alias for benchmarking compatibility"""
        return self.calculate_on_demand()
    
    def _synchronize_scenario_ids(self):
        """Synchronize scenario IDs across all components"""
        if self.scenario_manager and self.orchestrator:
            # Get the actual root scenario from scenario manager
            actual_root = self.scenario_manager.root_scenario_id
            
            # Update current scenario ID to match
            self.current_scenario_id = actual_root
            
            print(f"   Scenario IDs synchronized: {actual_root[:8]}")
    
    def run_visual_interface(self):
        """Run the visual interface with strategic features and menu integration"""
        if not self.visualizer:
            print("Visual interface not available (visualizer not found)")
            return
            
        print(f"\nStarting Visual Strategic Interface (Spillage: {'ON' if self.use_spillage_model else 'OFF'})")
        print("="*60)
        print("CONTROLS:")
        print("  Mouse Click: Select cell with objects")
        print("  't' after click: Execute target path")
        print("  'h' after click: Execute highway path") 
        print("  ENTER: Confirm trajectory execution")
        print("  'a': Analyze strategic situation (while visualizing)")
        print("  's': Save current state")
        print("  'f': Calculate strategic fields")
        print("  'c': Clear debug visualizations")
        print("  'm': Show menu options")
        print("  ESC: Return to main menu (keeps environment)")
        print("="*60)
        
        running = True
        clicked_cell = None
        
        while running:
            # Clear and draw
            self.visualizer.screen.fill((255, 255, 255))
            self.visualizer.draw_elements()
            
            pygame.display.flip()
            self.visualizer.clock.tick(30)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("Quitting visualization...")
                    running = False
                    
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("Returning to main menu (environment preserved)...")
                        running = False
                        
                    elif event.key == pygame.K_c:
                        # Clear all debug visualizations
                        self.visualizer.clear_visibility_preview()
                        self.visualizer.clear_affected_cells()
                        self.visualizer.clear_recalculation_cells()
                        self.visualizer.clear_spillage_cells()
                        print("CLEARED: All debug visualizations")
                        
                    elif event.key == pygame.K_a:
                        # Analyze situation while visualizing
                        print("\n" + "="*50)
                        print("ANALYZING SITUATION (Visualization continues)")
                        print("="*50)
                        self.analyze_strategic_situation()
                        print("="*50)
                        print("Analysis complete. Continue with visualization.")
                        
                    elif event.key == pygame.K_f:
                        # Calculate strategic fields
                        print("\nCalculating strategic fields...")
                        self.calculate_on_demand()
                        print("Strategic field calculation complete.")
                        
                    elif event.key == pygame.K_s:
                        # Save current state
                        print("\nSaving current state...")
                        # Temporarily close pygame to allow console input
                        pygame.display.set_mode((1, 1))  # Minimize window
                        self.save_state_with_timestamp()
                        # Reinitialize display
                        self.visualizer.screen = pygame.display.set_mode((self.visualizer.screen_size, self.visualizer.screen_size))
                        print("Save complete. Press any key to continue...")
                        
                    elif event.key == pygame.K_m:
                        # Show menu options
                        print("\n" + "-"*40)
                        print("VISUAL INTERFACE MENU:")
                        print("  'a' - Analyze strategic situation")
                        print("  'f' - Calculate strategic fields")
                        print("  's' - Save current state")
                        print("  'c' - Clear debug visualizations")
                        print("  'ESC' - Return to main menu")
                        print("-"*40)
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # Handle cell selection
                    pos = pygame.mouse.get_pos()
                    clicked_cell = self.visualizer.handle_click_event(pos)
                    
                    if clicked_cell and clicked_cell.num_objects > 0:
                        print(f"\nClicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects")
                        
                        # Show quick menu for path execution
                        print("Quick options: 't' = target path, 'h' = highway path, 'a' = analyze situation")
                        choice_input = input("Choose action: ").strip().lower()
                        
                        if choice_input == 't':
                            choice = "target"
                            success = self.execute_path_with_preview(clicked_cell, choice)
                            if success:
                                print(f"Target path executed successfully!")
                            else:
                                print(f"Target path execution failed or cancelled")
                                
                        elif choice_input == 'h':
                            choice = "highway"
                            success = self.execute_path_with_preview(clicked_cell, choice)
                            if success:
                                print(f"Highway path executed successfully!")
                            else:
                                print(f"Highway path execution failed or cancelled")
                                
                        elif choice_input == 'a':
                            print("\n" + "="*50)
                            print("ANALYZING SITUATION (Visualization continues)")
                            print("="*50)
                            self.analyze_strategic_situation()
                            print("="*50)
                            print("Analysis complete. Continue with visualization.")
                            
                        else:
                            print("Invalid choice. Use 't' for target, 'h' for highway, or 'a' for analysis.")
        
        # Don't quit pygame here - keep it running for potential return to visualization
        print("Visual interface paused. Environment preserved for main menu.")
    
    def execute_path_with_preview(self, clicked_cell, choice):
        """Execute path with preview and confirmation"""
        # Ensure calculations are done
        self.calculate_on_demand()
        
        # Refresh visibility for clicked cell
        TARGET_ANGLE_TOLERANCE = 45
        closest_point_target = self.env.find_closest_point_on_target((clicked_cell.x + 0.5, clicked_cell.y + 0.5))
        tx, ty = int(closest_point_target[0]), int(closest_point_target[1])
        
        clicked_cell.visible_cells_target, clicked_cell.distance_to_children_target = \
            self.env.calculate_target_zone_visibility(clicked_cell, angle_tolerance=TARGET_ANGLE_TOLERANCE)
        clicked_cell.h_vis_target = sum(n["cell"].num_objects for n in clicked_cell.visible_cells_target)
        
        # Get path for preview
        if choice == "target":
            current_paths = self.env.get_path_for_preview(clicked_cell, "target")
            if current_paths and len(current_paths) > 0:
                self.visualizer.set_visibility_preview(clicked_cell, "target", clicked_cell.visible_cells_target)
                print(f"VISIBILITY: Target visibility: {len(clicked_cell.visible_cells_target)} cells marked in light blue")
        else:
            # Highway path
            if hasattr(clicked_cell, 'best_path_highway') and clicked_cell.best_path_highway:
                cached_path = clicked_cell.best_path_highway
                
                if hasattr(clicked_cell, 'visible_cells_highway') and clicked_cell.visible_cells_highway:
                    self.visualizer.set_visibility_preview(clicked_cell, "highway", clicked_cell.visible_cells_highway)
                    print(f"VISIBILITY: Highway visibility: {len(clicked_cell.visible_cells_highway)} cells marked in light orange")
                
                current_paths = [{
                    'path': clicked_cell.best_path_highway,
                    'objects': clicked_cell.total_objects_highway,
                    'distance': clicked_cell.total_distance_highway
                }]
            else:
                print(f"No cached highway path available for cell ({clicked_cell.x}, {clicked_cell.y})")
                current_paths = []
        
        # Show trajectory preview
        if current_paths and len(current_paths) > 0:
            best_current_path = current_paths[0]
            path_info = {
                'path': best_current_path['path'],
                'objects': best_current_path['objects'],
                'distance': best_current_path['distance'],
                'impacted_cells': best_current_path.get('impacted_cells', {})
            }
            
            print(f"Showing {choice} trajectory preview...")
            self.visualizer.set_trajectory_preview(clicked_cell, choice, path_info)
            
            # Show preview details
            print(f"=== TRAJECTORY PREVIEW ===")
            print(f"Path Type: {choice.upper()}")
            print(f"Path Length: {len(best_current_path['path'])} cells")
            print(f"Objects: {best_current_path['objects']}")
            print(f"Distance: {best_current_path['distance']:.1f}")
            
            # Wait for confirmation
            print("Review the trajectory. Press ENTER to execute or ESC to cancel.")
            return self.wait_for_trajectory_confirmation(clicked_cell, choice, best_current_path)
        else:
            print(f"No valid {choice} path found for this cell.")
            return False
    
    def wait_for_trajectory_confirmation(self, clicked_cell, choice, path_info):
        """Wait for user confirmation of trajectory"""
        waiting_for_confirmation = True
        
        while waiting_for_confirmation:
            # Keep updating display during preview
            self.visualizer.screen.fill((255, 255, 255))
            self.visualizer.draw_elements()
            pygame.display.flip()
            self.visualizer.clock.tick(30)
            
            for preview_event in pygame.event.get():
                if preview_event.type == pygame.QUIT:
                    return False
                elif preview_event.type == pygame.KEYDOWN:
                    if preview_event.key == pygame.K_RETURN:  # ENTER to execute
                        print("Executing trajectory...")
                        self.env.execute_path(clicked_cell, choice, use_spillage=self.use_spillage_model, precomputed_path=path_info)
                        self.env.update_environment()
                        
                        # Show execution results
                        if hasattr(self.env, 'direct_affected_cells') and self.env.direct_affected_cells:
                            self.visualizer.set_affected_cells(list(self.env.direct_affected_cells))
                            print(f"DIRECTLY AFFECTED: {len(self.env.direct_affected_cells)} cells marked in magenta")
                        
                        if hasattr(self.env, 'recalculation_cells') and self.env.recalculation_cells:
                            self.visualizer.set_recalculation_cells(list(self.env.recalculation_cells))
                            print(f"RECALCULATION: {len(self.env.recalculation_cells)} cells marked in yellow")
                        
                        if hasattr(self.env, 'spillage_cells') and self.env.spillage_cells:
                            self.visualizer.set_spillage_cells(list(self.env.spillage_cells))
                            print(f"SPILLAGE CELLS: {len(self.env.spillage_cells)} cells marked in purple")
                        
                        self.visualizer.clear_trajectory_preview()
                        self.visualizer.clear_visibility_preview()
                        return True
                        
                    elif preview_event.key == pygame.K_ESCAPE:  # ESC to cancel
                        print("Trajectory cancelled.")
                        self.visualizer.clear_trajectory_preview()
                        self.visualizer.clear_visibility_preview()
                        return False
        
        return False
    
    def run_comprehensive_menu(self):
        """Complete menu with all features"""
        while True:
            print("\n" + "="*80)
            print("COMPREHENSIVE INTERACTIVE DEMO")
            print("="*80)
            print("Features: Visual interface | Strategic analysis | Benchmarking | Save/Load")
            
            if self.env:
                total_objects = sum(cell.num_objects for cell in self.env.cells_with_objects)
                fields_status = "Ready" if hasattr(self.env, '_fields_calculated') else "On-demand"
                print(f"\nEnvironment: {self.env.grid_size}x{self.env.grid_size}, {total_objects} objects")
                print(f"Strategy: {self.selected_strategy.value}")
                print(f"Spillage: {'ON' if self.use_spillage_model else 'OFF'}")
                print(f"Strategic fields: {fields_status}")
                if self.current_scenario_id:
                    print(f"Active scenario: {self.current_scenario_id[:8]}")
            else:
                print("\nNo environment loaded")
            
            print("\n--- SETUP ---")
            print("1. Create quick simulation (no spillage - fast)")
            print("2. Create simulation with spillage (slower)")
            
            print("\n--- OPERATE ---") 
            print("3. Start/Resume visual interface")
            print("4. Calculate strategic fields now")
            print("5. Analyze strategic situation")
            
            print("\n--- BENCHMARKING ---")
            print("6. Run quick benchmarks")
            print("7. Run comprehensive benchmarks")
            
            print("\n--- SAVE/LOAD ---")
            print("8. Save current state")
            print("9. Load saved state")
            
            print("\n--- EXIT ---")
            print("0. Exit")
            
            choice = input(f"\nEnter choice (0-9): ").strip()
            
            if choice == '1':
                self.create_simulation_quick(use_spillage=False)
                
            elif choice == '2':
                print("Creating simulation with spillage model...")
                confirm = input("This will take longer. Continue? (y/n): ").strip().lower()
                if confirm == 'y':
                    self.create_simulation_quick(use_spillage=True)
                    
            elif choice == '3':
                if self.env and self.visualizer:
                    print("Starting visual interface...")
                    print("Use 'a' key for analysis, 'ESC' to return to menu (preserves environment)")
                    self.run_visual_interface()
                else:
                    print("Environment or visualizer not available")
                    if not self.env:
                        print("Create a simulation first (option 1 or 2)")
                    
            elif choice == '4':
                if self.env:
                    print("Calculating strategic fields...")
                    self.calculate_on_demand()
                else:
                    print("No environment loaded")
                    
            elif choice == '5':
                if self.env:
                    self.analyze_strategic_situation()
                else:
                    print("No environment loaded")
                    
            elif choice == '6':
                print("Quick benchmarking feature - would run basic performance tests")
                print("(Integration with benchmarking system)")
                    
            elif choice == '7':
                print("Comprehensive benchmarking feature - would run full performance analysis")
                print("(Integration with benchmarking system)")
                    
            elif choice == '8':
                if self.env:
                    self.save_state_with_timestamp()
                else:
                    print("No environment to save")
                    
            elif choice == '9':
                self.load_state_by_selection()
                    
            elif choice == '0':
                print("Thank you for using the Comprehensive Demo!")
                break
                
            else:
                print("Invalid choice. Please enter a number from 0-9.")
    
    def analyze_strategic_situation(self):
        """Enhanced strategic analysis with convex hull, distance, and heatmap metrics"""
        if not self.env:
            print("No environment data available")
            return
        
        total_objects = sum(cell.num_objects for cell in self.env.cells_with_objects)
        total_cells = len(self.env.cells_with_objects)
        
        print(f"\n=== STRATEGIC ANALYSIS ===")
        print(f"Environment: {total_objects} objects in {total_cells} cells")
        print(f"Grid size: {self.env.grid_size}x{self.env.grid_size}")
        print(f"Current strategy: {self.selected_strategy.value}")
        print(f"Strategic calculations: {'Done' if hasattr(self.env, '_fields_calculated') else 'Pending'}")
        
        if total_cells == 0:
            print("No cells with objects to analyze")
            return
            
        # Basic statistics
        avg_objects = total_objects / total_cells
        print(f"Average objects per cell: {avg_objects:.1f}")
        
        # === 1. CONVEX HULL ANALYSIS ===
        convex_hull_area = self.calculate_convex_hull_area()
        print(f"\n--- Convex Hull Analysis ---")
        print(f"Convex hull area: {convex_hull_area:.2f} square units")
        print(f"Efficiency metric: {'Good (compact)' if convex_hull_area < (self.env.grid_size ** 2) * 0.3 else 'Poor (spread out)'}")
        
        # === 2. DISTANCE TO TARGET ANALYSIS ===
        avg_distance, total_distance = self.calculate_distance_metrics()
        print(f"\n--- Distance to Target Analysis ---")
        print(f"Average distance to target: {avg_distance:.2f} units")
        print(f"Total cumulative distance: {total_distance:.2f} units")
        print(f"Distance efficiency: {'Good (close to target)' if avg_distance < self.env.grid_size * 0.4 else 'Poor (far from target)'}")
        
        # === 3. HEATMAP SCORE ANALYSIS ===
        avg_heatmap, total_heatmap = self.calculate_heatmap_metrics()
        print(f"\n--- Heatmap Score Analysis ---")
        print(f"Average heatmap score: {avg_heatmap:.3f}")
        print(f"Total weighted heatmap score: {total_heatmap:.2f}")
        print(f"Highway positioning: {'Excellent' if avg_heatmap > 0.5 else 'Good' if avg_heatmap > 0.2 else 'Poor'}")
        
        # === OVERALL ASSESSMENT ===
        print(f"\n--- Overall Strategic Assessment ---")
        overall_score = self.calculate_overall_efficiency_score(convex_hull_area, avg_distance, avg_heatmap)
        print(f"Overall efficiency score: {overall_score:.3f} (0.0 = worst, 1.0 = best)")
        
        # Strategy recommendations
        if avg_objects > 2:
            print("• Recommendation: Consider GREEDY_EFFICIENT strategy")
        else:
            print("• Recommendation: GREEDY_NEAREST should work well")
        
        if avg_distance > self.env.grid_size * 0.5:
            print("• Consider focusing on cells closer to target zone first")
        
        if avg_heatmap < 0.2:
            print("• Consider using HIGHWAY_FORMATION strategy to improve positioning")
    
    def calculate_convex_hull_area(self):
        """Calculate the area of convex hull encompassing all cells with objects and target zone"""
        if not self.env.cells_with_objects:
            return 0.0
        
        # Collect all points: cells with objects and target zone boundary points
        points = []
        
        # Add cell centers
        for cell in self.env.cells_with_objects:
            points.append([cell.x + 0.5, cell.y + 0.5])
        
        # Add target zone boundary points (approximate with circle points)
        target_center_x = self.env.grid_size // 2
        target_center_y = self.env.grid_size // 2
        radius = self.env.target_zone_radius
        
        # Add points around target zone boundary (8 points for approximation)
        for angle in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            x = target_center_x + radius * np.cos(angle)
            y = target_center_y + radius * np.sin(angle)
            points.append([x, y])
        
        # Calculate convex hull
        if len(points) < 3:
            return 0.0
        
        try:
            points_array = np.array(points)
            hull = ConvexHull(points_array)
            return hull.volume  # In 2D, volume is area
        except Exception as e:
            print(f"Warning: Convex hull calculation failed: {e}")
            return 0.0
    
    def calculate_distance_metrics(self):
        """Calculate average and total distance from objects to target zone"""
        if not self.env.cells_with_objects:
            return 0.0, 0.0
        
        total_distance = 0.0
        total_objects = 0
        
        for cell in self.env.cells_with_objects:
            # Weight by number of objects in each cell
            cell_contribution = cell.distance_to_target * cell.num_objects
            total_distance += cell_contribution
            total_objects += cell.num_objects
        
        avg_distance = total_distance / total_objects if total_objects > 0 else 0.0
        return avg_distance, total_distance
    
    def calculate_heatmap_metrics(self):
        """Calculate average and total heatmap scores for cells with objects"""
        if not self.env.cells_with_objects:
            return 0.0, 0.0
        
        total_heatmap_score = 0.0
        total_objects = 0
        
        for cell in self.env.cells_with_objects:
            # Weight by number of objects in each cell
            cell_contribution = cell.heat_map * cell.num_objects
            total_heatmap_score += cell_contribution
            total_objects += cell.num_objects
        
        avg_heatmap = total_heatmap_score / total_objects if total_objects > 0 else 0.0
        return avg_heatmap, total_heatmap_score
    
    def calculate_overall_efficiency_score(self, convex_hull_area, avg_distance, avg_heatmap):
        """Calculate an overall efficiency score combining all metrics"""
        # Normalize convex hull area (smaller is better)
        max_area = self.env.grid_size ** 2
        hull_score = 1.0 - min(1.0, convex_hull_area / max_area)
        
        # Normalize distance (smaller is better)
        max_distance = math.sqrt(2) * self.env.grid_size  # Maximum possible distance
        distance_score = 1.0 - min(1.0, avg_distance / max_distance)
        
        # Heatmap score (higher is better, already normalized 0-1)
        heatmap_score = min(1.0, avg_heatmap)
        
        # Weighted combination (can be adjusted based on importance)
        overall_score = (0.3 * hull_score + 0.4 * distance_score + 0.3 * heatmap_score)
        return overall_score
    
    def save_current_state(self, save_name: str):
        """Save current environment state"""
        if not self.env:
            print("No environment to save.")
            return
        
        try:
            save_path = self.storage_path / save_name
            save_path.mkdir(exist_ok=True)
            
            # Get full environment state with all calculations
            full_state = self.env.get_state()
            
            # Convert tuple keys to strings for JSON serialization
            json_compatible_state = convert_tuple_keys_for_json(full_state)
            
            # Save complete state with all calculated fields
            with open(save_path / "environment_state.json", 'w') as f:
                json.dump(json_compatible_state, f, indent=2, cls=StateJSONEncoder)
            
            # Save demo configuration
            config = {
                'selected_strategy': self.selected_strategy.value,
                'use_spillage_model': self.use_spillage_model,
                'current_scenario_id': self.current_scenario_id
            }
            with open(save_path / "demo_config.json", 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"State saved to: {save_path}")
            
        except Exception as e:
            print(f"Failed to save state: {e}")
    
    def save_state_with_timestamp(self):
        """Save current environment state with user-provided name and timestamp"""
        if not self.env:
            print("No environment to save.")
            return
        
        # Ask user for save name
        user_name = input("Enter save name: ").strip()
        if not user_name:
            print("Save name required.")
            return
        
        # Add timestamp to save name
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_name = f"{user_name}_{timestamp}"
        
        print(f"Saving as: {save_name}")
        self.save_current_state(save_name)
    
    def load_state_by_selection(self):
        """Load saved state by numbered selection"""
        if not self.storage_path.exists():
            print("No saved states found.")
            return
        
        saves = [d for d in self.storage_path.iterdir() if d.is_dir()]
        
        if not saves:
            print("No saved states found.")
            return
        
        print("\nAvailable saved states:")
        valid_saves = []
        for i, save_dir in enumerate(saves, 1):
            state_file = save_dir / "environment_state.json"
            if state_file.exists():
                import time
                mod_time = time.ctime(state_file.stat().st_mtime)
                print(f"{i}. {save_dir.name} (saved: {mod_time})")
                valid_saves.append(save_dir.name)
            else:
                print(f"{i}. {save_dir.name} (incomplete save - skipping)")
        
        if not valid_saves:
            print("No valid saved states found.")
            return
        
        try:
            choice = input(f"\nEnter number (1-{len(valid_saves)}) or press Enter to cancel: ").strip()
            if not choice:
                print("Load cancelled.")
                return
            
            choice_num = int(choice)
            if 1 <= choice_num <= len(valid_saves):
                selected_save = valid_saves[choice_num - 1]
                print(f"Loading: {selected_save}")
                self.load_saved_state(selected_save)
            else:
                print(f"Invalid selection. Please choose 1-{len(valid_saves)}")
                
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    def load_saved_state(self, save_name: str):
        """Load previously saved environment state"""
        try:
            save_path = self.storage_path / save_name
            
            if not save_path.exists():
                print(f"Save file not found: {save_path}")
                return
            
            # Load environment state
            state_file = save_path / "environment_state.json"
            config_file = save_path / "demo_config.json"
            
            if not state_file.exists():
                print("Environment state file not found.")
                return
            
            with open(state_file, 'r') as f:
                json_state = json.load(f)
            
            # Convert string keys back to tuples
            state = convert_string_keys_to_tuples(json_state)
            
            # Create new environment with parameters from state
            from core_env import SimulationEnv
            self.env = SimulationEnv(
                grid_size=state.get('grid_size', 25),
                target_zone_radius=state.get('target_zone_radius', 10),
                agent_positions=None,
                num_random_objects=0,  # Will be restored from state
                seed=None
            )
            
            # Restore complete environment state with all calculated fields
            self.env.set_state(state)
            
            print(f"Environment restored with {len(self.env.cells_with_objects)} cells containing objects")
            print("All calculated fields (heat maps, paths, distances) have been restored")
            
            # Load demo configuration if available
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = json.load(f)
                
                # Restore configuration
                from strategic_strategy_planner import PlanningStrategy
                try:
                    self.selected_strategy = PlanningStrategy(config.get('selected_strategy', 'greedy_nearest'))
                    self.use_spillage_model = config.get('use_spillage_model', False)
                    self.current_scenario_id = config.get('current_scenario_id')
                except:
                    print("Warning: Could not restore all configuration settings")
            
            # Reinitialize visualizer
            try:
                from core_visualizer import SimulationVisualizer
                self.visualizer = SimulationVisualizer(self.env, screen_size=800)
                print("Visualization reinitialized")
            except:
                self.visualizer = None
                print("Warning: Could not reinitialize visualization")
            
            print(f"State loaded successfully from: {save_path}")
            
        except Exception as e:
            print(f"Failed to load state: {e}")
    
    def list_saved_states(self):
        """List all saved states"""
        if not self.storage_path.exists():
            print("No saved states found.")
            return
        
        saves = [d for d in self.storage_path.iterdir() if d.is_dir()]
        
        if not saves:
            print("No saved states found.")
            return
        
        print("\nAvailable saved states:")
        for i, save_dir in enumerate(saves, 1):
            state_file = save_dir / "environment_state.json"
            if state_file.exists():
                import time
                mod_time = time.ctime(state_file.stat().st_mtime)
                print(f"{i}. {save_dir.name} (saved: {mod_time})")
            else:
                print(f"{i}. {save_dir.name} (incomplete save)")

if __name__ == "__main__":
    print("Comprehensive Interactive Demo")
    print("All-in-one solution: Visual interface | Analysis | Benchmarking")
    print("="*60)
    
    demo = ComprehensiveDemo()
    demo.run_comprehensive_menu()