from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np


# ===================== SIMULATION CONFIG =====================
# Enhanced 2D Algorithm Configuration
SIMULATION_CONFIG = {
    'use_spillage_model': True,   # Enable spillage effects in 2D planning
    'visualize_potential': True,  # Show potential field visualization
    'enable_post_delivery_turn': True,  # Enable 180deg turn after target zone delivery
}

# Pure Pursuit configuration - can be overridden per trajectory
PURE_PURSUIT_CONFIG = {
    'mode': 'TCP',           # 'TCP' or 'BASE_SHIFT'
    'tcp_fwd': 0.12,        # TCP forward offset
    'tcp_lat': 0.00,        # TCP lateral offset  
    'v_nom': 0.30,          # Nominal velocity for Pure Pursuit++
    'a_lat_max': 1.0,       # Allow more cornering authority for tight curves
    'yaw_slew_rate': 7.0,   # Let heading catch up faster
    'lookahead': 0.10,      # Nominal L_MAX; code will shrink on curves  
    'resample_ds': 0.03,    # Keep fine spacing (already good)
    'draw_tool_tick': True  # Draw TCP marker
}

# Path Extension Configuration
PATH_EXTENSION_CONFIG = {
    'enable_extension': True,  # Enable backward path extension from 2D start point
    'extension_length': 0.8,    # How far to extend backwards (meters)
    'extension_smoothing': 0.8, # Smoothing factor for extended curve (0.0=sharp, 1.0=smooth)
    'extension_approach_angle': 30,  # Maximum approach angle deviation (degrees)
}

# Spillage Model Trajectory Configuration
SPILLAGE_TRAJECTORY_CONFIG = {
    'use_spillage_model_trajectories': True,  # Use spillage model for 3D trajectory generation
    'smoothing_factor': 0.5,       # Spline smoothing factor (0.0=sharp, 1.0=smooth)
    'num_points': 1000,            # Number of points for initial spline generation
    'target_spacing': 0.03,        # Final trajectory point spacing (meters)
}

# Trajectory Visualization Configuration
TRAJECTORY_VISUALIZATION_CONFIG = {
    'show_extension_preview': True,  # Show orange extension lines in 3D trajectory visualization
    'show_turn_markers': True,       # Show turn-in-place position markers
    'show_approach_lines': True,     # Show green approach trajectory lines
    'show_grid_lines': True,         # Show white grid lines in 3D PyBullet environment
    'show_trajectory_curves': True,   # Show blue curvature trajectory lines during execution
}

def handle_2d_events(visualizer, env, integration, current_selection=None):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False, visualizer, env, current_selection
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                # Clear selection if ESC is pressed
                if current_selection is not None:
                    print("\nESC pressed - clearing selection and preview")
                    visualizer.clear_trajectory_preview()
                    integration.clear_trajectory()
                    current_selection = None
                else:
                    return False, visualizer, env, current_selection
            elif event.key == pygame.K_s:
                # Toggle spillage model
                SIMULATION_CONFIG['use_spillage_model'] = not SIMULATION_CONFIG['use_spillage_model']
                print(f"(ROTATING) Spillage model: {'ENABLED' if SIMULATION_CONFIG['use_spillage_model'] else 'DISABLED'}")
                print("  Note: Changes will take effect on next environment recreation")
            elif event.key == pygame.K_v:
                # Toggle visualization
                SIMULATION_CONFIG['visualize_potential'] = not SIMULATION_CONFIG['visualize_potential']
                print(f"(ROTATING) Potential field visualization: {'ENABLED' if SIMULATION_CONFIG['visualize_potential'] else 'DISABLED'}")
                print("  Note: Changes will take effect on next environment recreation")
            elif event.key == pygame.K_t:
                # Toggle post-delivery turn
                SIMULATION_CONFIG['enable_post_delivery_turn'] = not SIMULATION_CONFIG['enable_post_delivery_turn']
                print(f"(ROTATING) Post-delivery turn: {'ENABLED' if SIMULATION_CONFIG['enable_post_delivery_turn'] else 'DISABLED'}")
                print("  Note: Rover will turn 180deg away from target center after deliveries")
            elif event.key == pygame.K_e:
                # Toggle path extension
                PATH_EXTENSION_CONFIG['enable_extension'] = not PATH_EXTENSION_CONFIG['enable_extension']
                print(f"(ROTATING) Path extension: {'ENABLED' if PATH_EXTENSION_CONFIG['enable_extension'] else 'DISABLED'}")
                if PATH_EXTENSION_CONFIG['enable_extension']:
                    print(f"  (MEASUREMENT) Extension length: {PATH_EXTENSION_CONFIG['extension_length']:.1f}m")
                    print(f"  (WAVE) Extension smoothing: {PATH_EXTENSION_CONFIG['extension_smoothing']:.1f}")
                print("  Note: Creates smoother approach by extending 2D path backwards")
            elif event.key == pygame.K_g:
                # Toggle 3D grid visualization
                TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines'] = not TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines']
                print(f"(ROTATING) 3D Grid lines: {'ENABLED' if TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines'] else 'DISABLED'}")
                # Apply the change immediately to the 3D environment
                integration.toggle_grid_overlay(TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines'])
            elif event.key == pygame.K_c:
                # Toggle trajectory curve visualization
                TRAJECTORY_VISUALIZATION_CONFIG['show_trajectory_curves'] = not TRAJECTORY_VISUALIZATION_CONFIG['show_trajectory_curves']
                print(f"(ROTATING) Blue trajectory curves: {'ENABLED' if TRAJECTORY_VISUALIZATION_CONFIG['show_trajectory_curves'] else 'DISABLED'}")
                print("  Note: Changes will take effect on next trajectory execution")
            elif event.key == pygame.K_h:
                # Show help
                print("\n=== KEYBOARD CONTROLS ===")
                print("S = Toggle spillage model ON/OFF")
                print("V = Toggle potential field visualization ON/OFF")
                print("T = Toggle post-delivery 180deg turn ON/OFF")
                print("E = Toggle path extension ON/OFF")
                print("G = Toggle 3D grid lines ON/OFF")
                print("C = Toggle blue trajectory curves ON/OFF")
                print("H = Show this help")
                print("Mouse = Left-click cells to select/toggle path type")
                print("  First click: Shows TARGET path")
                print("  Second click (same cell): Toggles to HIGHWAY path")
                print("  Different cell: Shows TARGET path for new cell")
                print("ENTER = Execute selected trajectory")
                print("ESC = Cancel selection and exit preview")
                print("========================\n")
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            clicked_cell = visualizer.handle_click_event(pos)
            if clicked_cell:
                print(f"\nClicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")

                # Determine path type based on current selection state
                if current_selection is not None and current_selection['cell'].x == clicked_cell.x and current_selection['cell'].y == clicked_cell.y:
                    # Same cell clicked again - toggle to next path type
                    current_path_type = current_selection['path_type']
                    if current_path_type == "target":
                        choice = "highway"
                    else:
                        choice = "target"
                    print(f"Toggling path type: {current_path_type} -> {choice}")
                else:
                    # New cell clicked - start with target path
                    choice = "target"
                    print(f"New cell selected - starting with {choice} path")

                # Get trajectory using enhanced 2D algorithm
                current_paths = env.get_path_for_preview(clicked_cell, choice)

                if not current_paths or len(current_paths) == 0:
                    print(f"WARNING: No valid {choice} path found for this cell.")
                    # If toggle failed, try to revert to previous path
                    if current_selection is not None and current_selection['cell'].x == clicked_cell.x and current_selection['cell'].y == clicked_cell.y:
                        choice = current_selection['path_type']
                        print(f"Reverting to {choice} path")
                        current_paths = env.get_path_for_preview(clicked_cell, choice)
                        if not current_paths or len(current_paths) == 0:
                            print(f"ERROR: No paths available for this cell")
                            current_selection = None
                            return True, visualizer, env, current_selection
                    else:
                        print(f"No paths available for this cell")
                        return True, visualizer, env, current_selection

                trajectory = current_paths[0]['path']  # Get the path points (Cell objects)
                path_info = current_paths[0]  # Store for later use (includes impacted_cells from env.py)

                # Convert Cell objects to coordinate tuples for both 3D visualization AND build_world_path
                trajectory_coords = [(cell.x, cell.y) for cell in trajectory]

                # Store coordinate tuples in env.current_trajectory (needed by build_world_path)
                env.current_trajectory = trajectory_coords
                env.current_cell = clicked_cell
                env.current_path_type = choice
                env.current_use_spillage = True

                print(f"\nVisualizing {choice} trajectory with {len(trajectory)} waypoints...")
                print(f"Path info: Objects={current_paths[0]['objects']}, Distance={current_paths[0]['distance']:.2f}")

                # Show 2D path preview
                # Set 2D trajectory preview in pygame (user wants to keep this)
                visualizer.set_trajectory_preview(clicked_cell, choice, path_info)
                print(f"(POSITION) 2D Path Preview: {len(trajectory)} waypoints shown in 2D visualization")
                for i, cell in enumerate(trajectory):
                    print(f"  Waypoint {i+1}: Cell({cell.x}, {cell.y}) - {cell.num_objects} objects")

                # No 3D trajectory preview - user only wants to see actual execution curves
                print(f"(POSITION) 3D Preview: Skipped - will show actual spline curvature during execution only")

                # Update current selection to track the selected cell and path type
                current_selection = {
                    'cell': clicked_cell,
                    'path_type': choice,
                    'path_info': path_info
                }

                # Ask for confirmation
                print(f"\n(THINKING) Execute this {choice} trajectory? ")
                print("   Press ENTER to execute, ESC to cancel, or click to toggle path type")
                
                # Wait for user input - also allow mouse clicks to toggle path type
                waiting_for_confirmation = True
                confirm = None
                while waiting_for_confirmation:
                    for confirm_event in pygame.event.get():
                        if confirm_event.type == pygame.QUIT:
                            return False, visualizer, env, current_selection
                        elif confirm_event.type == pygame.KEYDOWN:
                            if confirm_event.key == pygame.K_RETURN:
                                print("OK Executing trajectory...")
                                waiting_for_confirmation = False
                                confirm = "yes"
                            elif confirm_event.key == pygame.K_ESCAPE:
                                print("ERROR Trajectory cancelled")
                                # Clear previews
                                visualizer.clear_trajectory_preview()
                                integration.clear_trajectory()
                                waiting_for_confirmation = False
                                confirm = "no"
                                current_selection = None
                        elif confirm_event.type == pygame.MOUSEBUTTONDOWN:
                            # Allow clicking to toggle path type during confirmation
                            pos = pygame.mouse.get_pos()
                            new_clicked_cell = visualizer.handle_click_event(pos)
                            if new_clicked_cell:
                                print(f"\nClicked cell during confirmation: ({new_clicked_cell.x}, {new_clicked_cell.y})")
                                # Check if it's the same cell
                                if new_clicked_cell.x == clicked_cell.x and new_clicked_cell.y == clicked_cell.y:
                                    # Same cell - toggle the path type
                                    print("Same cell clicked - toggling path type")
                                    current_path_type = current_selection['path_type']
                                    if current_path_type == "target":
                                        new_choice = "highway"
                                    else:
                                        new_choice = "target"
                                    print(f"Toggling: {current_path_type} -> {new_choice}")

                                    # Try to get the new path
                                    new_paths = env.get_path_for_preview(new_clicked_cell, new_choice)
                                    if new_paths and len(new_paths) > 0:
                                        # Update with new path
                                        trajectory = new_paths[0]['path']
                                        path_info = new_paths[0]
                                        trajectory_coords = [(cell.x, cell.y) for cell in trajectory]

                                        env.current_trajectory = trajectory_coords
                                        env.current_path_type = new_choice

                                        # Update visualization
                                        visualizer.set_trajectory_preview(new_clicked_cell, new_choice, path_info)
                                        print(f"(POSITION) Updated preview to {new_choice} path with {len(trajectory)} waypoints")

                                        # Update selection
                                        current_selection = {
                                            'cell': new_clicked_cell,
                                            'path_type': new_choice,
                                            'path_info': path_info
                                        }
                                        choice = new_choice
                                        print(f"\n(THINKING) Execute this {new_choice} trajectory? ")
                                        print("   Press ENTER to execute, ESC to cancel, or click to toggle path type")
                                    else:
                                        print(f"WARNING: No {new_choice} path available for this cell")
                                else:
                                    print("Different cell clicked - exiting confirmation")
                                    waiting_for_confirmation = False
                                    confirm = "toggle"

                    # Update display during confirmation wait
                    visualizer.screen.fill((255, 255, 255))
                    visualizer.draw_elements()
                    pygame.display.flip()
                    visualizer.clock.tick(30)

                if confirm == "yes":
                    # Execute alignment gate pivot trajectory: approach from behind, pivot when aligned
                    print("(ROCKET) Using ALIGNMENT GATE PIVOT trajectory approach...")
                    integration.execute_alignment_gate_pivot_trajectory(
                        env,
                        mode=PURE_PURSUIT_CONFIG['mode'],
                        resample_ds=PURE_PURSUIT_CONFIG['resample_ds'],
                        v_nom=PURE_PURSUIT_CONFIG['v_nom'],
                        a_lat_max=PURE_PURSUIT_CONFIG['a_lat_max'],
                        yaw_slew_rate=PURE_PURSUIT_CONFIG['yaw_slew_rate'],
                        lookahead=PURE_PURSUIT_CONFIG['lookahead'],
                        tcp_fwd=PURE_PURSUIT_CONFIG['tcp_fwd'],
                        tcp_lat=PURE_PURSUIT_CONFIG['tcp_lat'],
                        # Alignment gate tuning:
                        align_dot=0.93,          # try 0.90-0.96
                        gate_back=0.50,          # Distance to place turn-in-place point behind first waypoint (try 0.45-0.60 m)
                        dist_band_min=0.20,
                        dist_band_max=0.60,
                        # Spillage model trajectory config:
                        spillage_trajectory_config=SPILLAGE_TRAJECTORY_CONFIG,
                        # Trajectory visualization config:
                        trajectory_visualization_config=TRAJECTORY_VISUALIZATION_CONFIG,
                    )
                    
                    # Enhanced post-trajectory processing with configurable turn-around for target deliveries
                    if SIMULATION_CONFIG['enable_post_delivery_turn'] and choice == "target":
                        print(f"(ROTATING) Post-delivery turn-around is ENABLED for target delivery")
                        print(f"  (POSITION) Final trajectory point: {env.current_trajectory[-1] if env.current_trajectory else 'None'}")
                        transfer_result = integration._post_trajectory_update_with_turnaround(env)
                    else:
                        if choice == "target":
                            print(f"(ROTATING) Post-delivery turn-around is DISABLED for target delivery")
                        else:
                            print(f"(ROTATING) No turn-around needed for highway delivery")
                        transfer_result = integration._post_trajectory_update(env)
                    
                    # Display transfer results and handle 2D environment recreation
                    if transfer_result:
                        print(f"(ROTATING) Transfer Summary:")
                        print(f"  - Total 3D objects: {transfer_result['total_objects']}")
                        print(f"  - Pebbles transferred to 2D: {transfer_result['pebbles_transferred']}")
                        
                        # Check if 2D environment needs recreation
                        if transfer_result.get('needs_2d_recreation', False):
                            print(f"(ROTATING) Completely recreating 2D environment and visualizer...")

                            # Get updated object positions and dynamic radius
                            updated_positions_3d = transfer_result.get('new_object_positions_3d', [])
                            updated_env_radius = transfer_result.get('dynamic_env_radius', dynamic_env_radius)

                            print(f"  (POSITION) Using {len(updated_positions_3d)} updated 3D positions")
                            print(f"  (MEASUREMENT) Using dynamic environment radius: {updated_env_radius:.3f}m")

                            # Update integration's coordinate converter with new radius
                            integration.update_env_radius(updated_env_radius)

                            # Clear ALL trajectory-related state first to prevent interference
                            integration.clear_trajectory()
                            visualizer.clear_trajectory_preview()

                            # COMPLETELY recreate 2D environment from scratch (no state preservation!)
                            from main import run_2d_env
                            env, new_visualizer = run_2d_env(
                                env_radius=updated_env_radius,  # Use dynamic radius
                                target_zone_radius=target_zone_radius,
                                shovel_width=integration.shovel_width,
                                real_objects=updated_positions_3d,  # Use NEW positions!
                                manual_mode=False,
                                use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
                                visualize_potential=SIMULATION_CONFIG['visualize_potential']
                            )

                            print(f"OK 2D environment completely recreated from scratch!")
                            print(f"  - New cells with objects: {len(env.cells_with_objects) if hasattr(env, 'cells_with_objects') else 'unknown'}")
                            print(f"  - Fresh environment with updated object positions and recalculated heatmap")

                            # Replace with the completely new visualizer and environment
                            visualizer = new_visualizer
                        
                        else:
                            print("  INFO: 2D environment update not needed")
                            # Still need to execute the 2D path action if no recreation was needed
                            env.execute_path(env.current_cell, env.current_path_type, use_spillage=SIMULATION_CONFIG['use_spillage_model'], precomputed_path=path_info)
                    
                    print("(TARGET) Trajectory execution and environment sync complete!")

                    # Cleanup visualizations and selection state after execution
                    integration.clear_trajectory()
                    visualizer.clear_trajectory_preview()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                    current_selection = None
                elif confirm == "toggle":
                    # Mouse click during confirmation - don't execute, allow next click to toggle
                    # The next iteration of the main event loop will handle the click
                    pass
                else:
                    print("\nTrajectory execution cancelled.")
                    integration.clear_trajectory()
                    visualizer.clear_trajectory_preview()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                    current_selection = None

    return True, visualizer, env, current_selection

if __name__ == "__main__":
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 40
    random_seed = 10
    initial_robot_pose = (0.0, -1.5, math.pi / 2)

    integration = PyBulletIntegration(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        num_pebbles=num_pebbles,
        random_seed=random_seed,
        gui=True,
        initial_robot_pose=initial_robot_pose
    )

    integration.set_robot_dim(L=0.2, R=0.07)

    print("\nLoading 3D simulation and automatically continuing to 2D visualization...")
    all_objects_3d = integration.run(auto_continue=True)

    # Filter out plane and robot (only get pebbles for 2D conversion)
    objects_3d = all_objects_3d[2:] if len(all_objects_3d) > 2 else []
    print(f"(PACKAGE) Filtered objects: {len(all_objects_3d)} total -> {len(objects_3d)} pebbles for 2D conversion")

    # Calculate dynamic environment radius based on actual object positions
    dynamic_env_radius = integration.calculate_dynamic_env_radius(safety_margin=0.1)

    # Update the integration's coordinate converter with the dynamic radius
    integration.update_env_radius(dynamic_env_radius)

    shovel_width = integration.shovel_width

    # Create 2D environment with dynamically calculated radius
    env, visualizer = run_2d_env(
        env_radius=dynamic_env_radius,  # Use dynamic radius instead of fixed
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width,
        real_objects=objects_3d,
        manual_mode=False,
        use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
        visualize_potential=SIMULATION_CONFIG['visualize_potential']
    )

    # Initialize grid overlay based on configuration
    integration.initialize_grid_overlay(TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines'])
    
    print(f"\n=== SIMULATION CONTROLS ===")
    print(f"Spillage Model: {'ENABLED' if SIMULATION_CONFIG['use_spillage_model'] else 'DISABLED'} (Press 'S' to toggle)")
    print(f"Potential Visualization: {'ENABLED' if SIMULATION_CONFIG['visualize_potential'] else 'DISABLED'} (Press 'V' to toggle)")
    print(f"Post-Delivery Turn: {'ENABLED' if SIMULATION_CONFIG['enable_post_delivery_turn'] else 'DISABLED'} (Press 'T' to toggle)")
    print(f"Path Extension: {'ENABLED' if PATH_EXTENSION_CONFIG['enable_extension'] else 'DISABLED'} (Press 'E' to toggle)")
    if PATH_EXTENSION_CONFIG['enable_extension']:
        print(f"  Extension Length: {PATH_EXTENSION_CONFIG['extension_length']:.1f}m, Smoothing: {PATH_EXTENSION_CONFIG['extension_smoothing']:.1f}")
    print(f"3D Grid Lines: {'ENABLED' if TRAJECTORY_VISUALIZATION_CONFIG['show_grid_lines'] else 'DISABLED'} (Press 'G' to toggle)")
    print(f"Blue Trajectory Curves: {'ENABLED' if TRAJECTORY_VISUALIZATION_CONFIG['show_trajectory_curves'] else 'DISABLED'} (Press 'C' to toggle)")
    print(f"Extension Visualization: {'ENABLED' if TRAJECTORY_VISUALIZATION_CONFIG['show_extension_preview'] else 'DISABLED'}")
    print(f"Press 'H' for help, ESC to exit")
    print("===========================\n")

    running = True
    current_selection = None  # Track the current cell selection and path type
    while running:
        visualizer.screen.fill((255, 255, 255))
        visualizer.draw_elements()
        pygame.display.flip()
        visualizer.clock.tick(30)
        running, visualizer, env, current_selection = handle_2d_events(visualizer, env, integration, current_selection)

    pygame.quit()
    integration.close_environment()
