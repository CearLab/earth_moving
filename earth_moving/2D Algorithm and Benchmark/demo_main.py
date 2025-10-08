from core_env import SimulationEnv
import pygame
from core_visualizer import SimulationVisualizer
import warnings
import math
from core_cell import Cell  # Import the Cell class

warnings.filterwarnings("ignore", category=UserWarning, module="numpy")

# ✅ SINGLE CONFIGURATION POINT FOR ANGLE TOLERANCE
TARGET_ANGLE_TOLERANCE = 45   # Fixed angle for all target zone visibility calculations
HIGHWAY_ANGLE_TOLERANCE = 60  # Fixed angle for all highway visibility calculations

# ✅ HIGHWAY TARGET SELECTION CONFIGURATION
HIGHWAY_MIN_HEAT_RATIO = 0.3      # Minimum heat as % of max heat (0.1-0.5)
HIGHWAY_THRESHOLD_RATIO = 0.5     # Highway threshold as % of max potential (0.3-0.7)
HIGHWAY_HEAT_WEIGHT = 0.7         # Weight for heat map score (0.0-1.0, higher = prioritize heat)
HIGHWAY_DISTANCE_WEIGHT = 0.3     # Weight for distance score (0.0-1.0, higher = prioritize proximity)



def main():
    # ✅ Ask for spillage model preference at startup
    print("=== Earth Moving Simulation Setup ===")
    spillage_choice = input("Use spillage model for this simulation? Press 'y' for yes, 'n' for no: ").strip().lower()
    use_spillage_model = spillage_choice == 'y'
    print(f"Spillage model: {'ENABLED' if use_spillage_model else 'DISABLED'}")
    
    print("Initializing environment...")
    env = SimulationEnv(
        grid_size=25,  # Define the grid size
        target_zone_radius=3,  # Define the target zone radius
        agent_positions=None,  # No predefined agent positions
        num_random_objects=55,  # Number of random objects to spawn
        seed=31,  # Set a fixed random seed for testing
        max_path_length_factor=2,  # Limit A* path length to 2.5x straight distance
        target_angle_tolerance=TARGET_ANGLE_TOLERANCE,  # Use configured target angle
        highway_angle_tolerance=HIGHWAY_ANGLE_TOLERANCE,  # Use configured highway angle
        highway_min_heat_ratio=HIGHWAY_MIN_HEAT_RATIO,    # Minimum heat quality threshold
        highway_threshold_ratio=HIGHWAY_THRESHOLD_RATIO,  # Highway threshold configuration
        highway_heat_weight=HIGHWAY_HEAT_WEIGHT,          # Heat weight in hybrid scoring
        highway_distance_weight=HIGHWAY_DISTANCE_WEIGHT,  # Distance weight in hybrid scoring
    )
    print("Environment initialized!")
    print(f"Number of cells with objects: {len(env.cells_with_objects)}")
    print(f"Angle Configuration: Target={TARGET_ANGLE_TOLERANCE}°, Highway={HIGHWAY_ANGLE_TOLERANCE}°")
    print(f"Highway Configuration: MinHeatRatio={HIGHWAY_MIN_HEAT_RATIO}, ThresholdRatio={HIGHWAY_THRESHOLD_RATIO}, HeatWeight={HIGHWAY_HEAT_WEIGHT}, DistanceWeight={HIGHWAY_DISTANCE_WEIGHT}")

    for cell in env.cells_with_objects:
        print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")

    # Precompute visibility for all cells in the grid (Target Zone Path-related only)
    print("Calculating visibility for all cells (Target Zone Path-related)...")
    for cell in env.cells_with_objects:
        closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
        tx, ty = int(closest_point_target[0]), int(closest_point_target[1])
        target_cell_target = env.get_cell(tx, ty)
        visible_cells_target, distance_to_children_target = env.calculate_target_zone_visibility(
            cell, angle_tolerance=TARGET_ANGLE_TOLERANCE)  # Use configured target angle
        cell.visible_cells_target = visible_cells_target
        cell.distance_to_children_target = distance_to_children_target
        
        # Compute optimistic heuristic (sum of objects in visibility scope)
        cell.h_vis_target = sum(n["cell"].num_objects for n in cell.visible_cells_target)
    print("Visibility for Target Zone Path-related attributes calculated.")
    
    # Audit visibility after target visibility computation
    env.audit_visibility()

    # ✅ Compute potential field with selected spillage model
    print("Calculating potential field...")
    env.calculate_potential_field(use_spillage_model=use_spillage_model, visualize=True)
    print("Potential field calculated.")

    # ✅ Compute velocity field
    print("Calculating velocity field...")
    env.calculate_velocity_field()
    print("Velocity field calculated.")

    # ✅ Simulate flow and update the heat map
    print("Simulating flow and updating heat map...")
    env.update_heat_map()
    print("Heat map updated.")

    # ✅ Calculate paths to highways (needed for execution)
    print("Calculating paths to highways for low-potential cells...")
    env.calculate_path_to_highway(use_spillage_model=use_spillage_model)
    print("Paths to highways calculated.")

    # Initialize visualization
    visualizer = SimulationVisualizer(env, screen_size=800)

    print(f"\nStarting interactive simulation (Spillage: {'ON' if use_spillage_model else 'OFF'})")
    print("Click on cells with objects, then press 't' for target path or 'h' for highway path")
    print("You'll see a trajectory preview - Press ENTER to execute or ESC to cancel")
    print("")
    print("DEBUG VISUALIZATIONS:")
    print("  • Target visibility: Light blue cells with 'T' markers")
    print("  • Highway visibility: Light orange cells with 'H' markers") 
    print("  • Affected cells after execution: Magenta cells with 'A' markers")
    print("  • Recalculation cells: Yellow cells with 'R' markers")
    print("  • Spillage cells: Purple cells with 'S' markers")
    print("  • Press 'C' to clear all debug visualizations")
    print("")
    print("Press ESC to exit simulation\n")

    # ✅ Visualization loop with interaction
    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))  # Clear screen
        visualizer.draw_elements()  # Draw all elements
        pygame.display.flip()
        visualizer.clock.tick(30)  # Limit FPS to 30

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quitting visualization...")
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("Escape key pressed. Exiting...")
                    running = False
                elif event.key == pygame.K_c:
                    # Clear all debug visualizations
                    visualizer.clear_visibility_preview()
                    visualizer.clear_affected_cells()
                    visualizer.clear_recalculation_cells()
                    visualizer.clear_spillage_cells()
                    print("CLEARED: All debug visualizations")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle click interaction
                pos = pygame.mouse.get_pos()
                clicked_cell = visualizer.handle_click_event(pos)
                if clicked_cell:
                    if clicked_cell.x==14 and clicked_cell.y==15:
                        print("test")
                    print(f"Clicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")
                    # Debug line removed (was typo: y--1 is y+1 in Python)

                    # ✅ Ask user for path choice with single key press
                    choice_input = input("Choose path type - Press 't' for target, 'h' for highway: ").strip().lower()
                    if choice_input == 't':
                        choice = "target"
                    elif choice_input == 'h':
                        choice = "highway"
                    else:
                        print("⚠️ Invalid choice. Press 't' for target or 'h' for highway.")
                        continue

                    # ✅ Recompute visibility for clicked cell to eliminate stale data
                    print(f"Refreshing visibility for clicked cell...")
                    closest_point_target = env.find_closest_point_on_target((clicked_cell.x + 0.5, clicked_cell.y + 0.5))
                    tx, ty = int(closest_point_target[0]), int(closest_point_target[1])
                    target_cell_target = env.get_cell(tx, ty)
                    
                    # Debug: Show configured angle usage
                    dST = math.hypot(target_cell_target.x + 0.5 - (clicked_cell.x + 0.5), target_cell_target.y + 0.5 - (clicked_cell.y + 0.5))
                    print(f"[Target Angle] configured={TARGET_ANGLE_TOLERANCE}°  dST={dST:.1f}/{env.grid_diagonal:.1f}")
                    
                    clicked_cell.visible_cells_target, clicked_cell.distance_to_children_target = \
                        env.calculate_target_zone_visibility(clicked_cell, angle_tolerance=TARGET_ANGLE_TOLERANCE)  # Use configured target angle
                    
                    # Compute optimistic heuristic (sum of objects in visibility scope)
                    clicked_cell.h_vis_target = sum(n["cell"].num_objects for n in clicked_cell.visible_cells_target)
                    
                    # ✅ Also refresh highway visibility for consistency (same as target)
                    if hasattr(clicked_cell, 'chosen_highway_target') and clicked_cell.chosen_highway_target:
                        print(f"Refreshing highway visibility for clicked cell...")
                        clicked_cell.visible_cells_highway, clicked_cell.distance_to_children_highway = \
                            env.calculate_highway_visibility(clicked_cell, clicked_cell.chosen_highway_target, 
                                                           angle_tolerance=HIGHWAY_ANGLE_TOLERANCE)  # Use configured highway angle

                    # ✅ Show visibility scope for debugging
                    if choice == "target":
                        visualizer.set_visibility_preview(clicked_cell, "target", clicked_cell.visible_cells_target)
                        print(f"VISIBILITY: Target visibility: {len(clicked_cell.visible_cells_target)} cells marked in light blue")
                    
                    # ✅ Recompute path in real-time to get current state
                    print(f"Computing current {choice} path...")
                    if choice == "target":
                        # Use pre-computed path from environment
                        current_paths = env.get_path_for_preview(clicked_cell, "target")
                    else:
                        # ✅ Use cached highway plan instead of recomputing
                        if hasattr(clicked_cell, 'best_path_highway') and clicked_cell.best_path_highway:
                            # Use the pre-computed highway plan
                            cached_path = clicked_cell.best_path_highway
                            target = cached_path[-1]  # Final destination in the path
                            
                            print(f"Using cached highway path from ({clicked_cell.x}, {clicked_cell.y}) to ({target.x}, {target.y}) with heat={target.heat_map:.3f}")
                            
                            # Show highway visibility scope for debugging (use cached visibility)
                            if hasattr(clicked_cell, 'visible_cells_highway') and clicked_cell.visible_cells_highway:
                                visualizer.set_visibility_preview(clicked_cell, "highway", clicked_cell.visible_cells_highway)
                                print(f"VISIBILITY: Highway visibility: {len(clicked_cell.visible_cells_highway)} cells marked in light orange")
                            
                            # Create path info from cached data
                            current_paths = [{
                                'path': clicked_cell.best_path_highway,
                                'objects': clicked_cell.total_objects_highway,
                                'distance': clicked_cell.total_distance_highway
                            }]
                        else:
                            print(f"No cached highway path available for cell ({clicked_cell.x}, {clicked_cell.y})")
                            current_paths = []
                    
                    # ✅ Show trajectory preview with current data
                    if current_paths and len(current_paths) > 0:
                        best_current_path = current_paths[0]  # Get best current path
                        path_info = {
                            'path': best_current_path['path'],
                            'objects': best_current_path['objects'],
                            'distance': best_current_path['distance'],
                            'impacted_cells': best_current_path.get('impacted_cells', {})
                        }
                        print(f"Showing {choice} trajectory preview...")
                        visualizer.set_trajectory_preview(clicked_cell, choice, path_info)
                        
                        # Wait for user confirmation (ENTER to execute, ESC to cancel)
                        print("Review the trajectory. Press ENTER to execute or ESC to cancel.")
                        waiting_for_confirmation = True
                        
                        while waiting_for_confirmation:
                            # Keep updating the display during preview
                            visualizer.screen.fill((255, 255, 255))
                            visualizer.draw_elements()
                            pygame.display.flip()
                            visualizer.clock.tick(30)
                            
                            for preview_event in pygame.event.get():
                                if preview_event.type == pygame.QUIT:
                                    running = False
                                    waiting_for_confirmation = False
                                elif preview_event.type == pygame.KEYDOWN:
                                    if preview_event.key == pygame.K_RETURN:  # ENTER to execute
                                        print("Executing trajectory...")
                                        env.execute_path(clicked_cell, choice, use_spillage=use_spillage_model, precomputed_path=best_current_path)
                                        env.update_environment()
                                        
                                        # ✅ Show affected cells from execution
                                        if hasattr(env, 'direct_affected_cells') and env.direct_affected_cells:
                                            visualizer.set_affected_cells(list(env.direct_affected_cells))
                                            print(f"DIRECTLY AFFECTED: {len(env.direct_affected_cells)} cells marked in magenta 'A'")
                                        
                                        if hasattr(env, 'recalculation_cells') and env.recalculation_cells:
                                            visualizer.set_recalculation_cells(list(env.recalculation_cells))
                                            print(f"RECALCULATION: {len(env.recalculation_cells)} cells marked in yellow 'R'")
                                        
                                        # ✅ Show spillage cells created from execution
                                        if hasattr(env, 'spillage_cells') and env.spillage_cells:
                                            visualizer.set_spillage_cells(list(env.spillage_cells))
                                            print(f"SPILLAGE CELLS: {len(env.spillage_cells)} cells marked in purple 'S'")
                                        
                                        visualizer.clear_trajectory_preview()
                                        visualizer.clear_visibility_preview()  # Clear visibility scope after execution
                                        waiting_for_confirmation = False
                                    elif preview_event.key == pygame.K_ESCAPE:  # ESC to cancel
                                        print("Trajectory cancelled.")
                                        visualizer.clear_trajectory_preview()
                                        visualizer.clear_visibility_preview()  # Clear visibility scope when canceling
                                        waiting_for_confirmation = False
                    else:
                        print(f"⚠️ No valid {choice} path found for this cell.")
                        continue

    pygame.quit()
    print("Simulation ended successfully.")

if __name__ == "__main__":
    main()
