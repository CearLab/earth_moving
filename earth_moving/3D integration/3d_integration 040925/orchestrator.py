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
    'enable_post_delivery_turn': True,  # Enable 180° turn after target zone delivery
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
    'extension_length': 1.8,    # How far to extend backwards (meters)
    'extension_smoothing': 0.8, # Smoothing factor for extended curve (0.0=sharp, 1.0=smooth)
    'extension_approach_angle': 30,  # Maximum approach angle deviation (degrees)
}

def handle_2d_events(visualizer, env, integration):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False, visualizer, env
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False, visualizer, env
            elif event.key == pygame.K_s:
                # Toggle spillage model
                SIMULATION_CONFIG['use_spillage_model'] = not SIMULATION_CONFIG['use_spillage_model']
                print(f"🔄 Spillage model: {'ENABLED' if SIMULATION_CONFIG['use_spillage_model'] else 'DISABLED'}")
                print("  Note: Changes will take effect on next environment recreation")
            elif event.key == pygame.K_v:
                # Toggle visualization
                SIMULATION_CONFIG['visualize_potential'] = not SIMULATION_CONFIG['visualize_potential']
                print(f"🔄 Potential field visualization: {'ENABLED' if SIMULATION_CONFIG['visualize_potential'] else 'DISABLED'}")
                print("  Note: Changes will take effect on next environment recreation")
            elif event.key == pygame.K_t:
                # Toggle post-delivery turn
                SIMULATION_CONFIG['enable_post_delivery_turn'] = not SIMULATION_CONFIG['enable_post_delivery_turn']
                print(f"🔄 Post-delivery turn: {'ENABLED' if SIMULATION_CONFIG['enable_post_delivery_turn'] else 'DISABLED'}")
                print("  Note: Rover will turn 180° away from target center after deliveries")
            elif event.key == pygame.K_e:
                # Toggle path extension
                PATH_EXTENSION_CONFIG['enable_extension'] = not PATH_EXTENSION_CONFIG['enable_extension']
                print(f"🔄 Path extension: {'ENABLED' if PATH_EXTENSION_CONFIG['enable_extension'] else 'DISABLED'}")
                if PATH_EXTENSION_CONFIG['enable_extension']:
                    print(f"  📏 Extension length: {PATH_EXTENSION_CONFIG['extension_length']:.1f}m")
                    print(f"  🌊 Extension smoothing: {PATH_EXTENSION_CONFIG['extension_smoothing']:.1f}")
                print("  Note: Creates smoother approach by extending 2D path backwards")
            elif event.key == pygame.K_h:
                # Show help
                print("\n=== KEYBOARD CONTROLS ===")
                print("S = Toggle spillage model ON/OFF")
                print("V = Toggle potential field visualization ON/OFF") 
                print("T = Toggle post-delivery 180° turn ON/OFF")
                print("E = Toggle path extension ON/OFF")
                print("H = Show this help")
                print("ESC = Exit simulation")
                print("Mouse = Click cells to plan trajectories")
                print("ENTER = Execute trajectory preview")
                print("========================\n")
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            clicked_cell = visualizer.handle_click_event(pos)
            if clicked_cell:
                print(f"\nClicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")

                # choice = input("Choose path type ('target' or 'highway'): ").strip().lower()
                choice = "target"
                if choice not in ["target", "highway"]:
                    print("⚠️ Invalid choice. Try again.")
                    return True, visualizer, env

                # Get trajectory using enhanced 2D algorithm
                current_paths = env.get_path_for_preview(clicked_cell, choice)
                
                if not current_paths or len(current_paths) == 0:
                    print(f"⚠️ No valid {choice} path found for this cell.")
                    continue
                    
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
                visualizer.set_trajectory_preview(clicked_cell, choice, path_info)
                print(f"📍 2D Path Preview: {len(trajectory)} waypoints shown in 2D visualization")
                for i, cell in enumerate(trajectory):
                    print(f"  Waypoint {i+1}: Cell({cell.x}, {cell.y}) - {cell.num_objects} objects")
                
                # Show 3D trajectory visualization
                integration.visualize_trajectory(trajectory_coords)
                print(f"📍 3D Trajectory: Visualized in PyBullet environment")
                
                # Ask for confirmation
                print(f"\n🤔 Execute this {choice} trajectory? ")
                print("   Press ENTER to execute, or ESC to cancel")
                
                # Wait for user input
                waiting_for_confirmation = True
                while waiting_for_confirmation:
                    for confirm_event in pygame.event.get():
                        if confirm_event.type == pygame.QUIT:
                            return False, visualizer, env
                        elif confirm_event.type == pygame.KEYDOWN:
                            if confirm_event.key == pygame.K_RETURN:
                                print("✅ Executing trajectory...")
                                waiting_for_confirmation = False
                                confirm = "yes"
                            elif confirm_event.key == pygame.K_ESCAPE:
                                print("❌ Trajectory cancelled")
                                # Clear previews
                                visualizer.clear_trajectory_preview()
                                integration.clear_trajectory()
                                waiting_for_confirmation = False
                                confirm = "no"
                    
                    # Update display during confirmation wait
                    visualizer.screen.fill((255, 255, 255))
                    visualizer.draw_elements()
                    pygame.display.flip()
                    visualizer.clock.tick(30)

                if confirm == "yes":
                    # Execute alignment gate pivot trajectory: approach from behind, pivot when aligned
                    print("🚀 Using ALIGNMENT GATE PIVOT trajectory approach...")
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
                        align_dot=0.93,          # try 0.90–0.96
                        gate_back=0.35,          # try 0.25–0.45 m
                        dist_band_min=0.20,
                        dist_band_max=0.60,
                    )
                    
                    # Enhanced post-trajectory processing with configurable turn-around for target deliveries
                    if SIMULATION_CONFIG['enable_post_delivery_turn'] and choice == "target":
                        print(f"🔄 Post-delivery turn-around is ENABLED for target delivery")
                        print(f"  📍 Final trajectory point: {env.current_trajectory[-1] if env.current_trajectory else 'None'}")
                        transfer_result = integration._post_trajectory_update_with_turnaround(env)
                    else:
                        if choice == "target":
                            print(f"🔄 Post-delivery turn-around is DISABLED for target delivery")
                        else:
                            print(f"🔄 No turn-around needed for highway delivery")
                        transfer_result = integration._post_trajectory_update(env)
                    
                    # Display transfer results and handle 2D environment recreation
                    if transfer_result:
                        print(f"🔄 Transfer Summary:")
                        print(f"  • Total 3D objects: {transfer_result['total_objects']}")
                        print(f"  • Pebbles transferred to 2D: {transfer_result['pebbles_transferred']}")
                        
                        # Check if 2D environment needs recreation
                        if transfer_result.get('needs_2d_recreation', False):
                            print(f"🔄 Completely recreating 2D environment and visualizer...")
                            
                            # Get updated object positions
                            updated_positions_3d = transfer_result.get('new_object_positions_3d', [])
                            print(f"  📍 Using {len(updated_positions_3d)} updated 3D positions")
                            
                            # Clear ALL trajectory-related state first to prevent interference
                            integration.clear_trajectory()
                            visualizer.clear_trajectory_preview()
                            
                            # COMPLETELY recreate 2D environment from scratch (no state preservation!)
                            from main import run_2d_env
                            env, new_visualizer = run_2d_env(
                                env_radius=1.0,  # Same as orchestrator main
                                target_zone_radius=0.3,
                                shovel_width=integration.shovel_width,
                                real_objects=updated_positions_3d,  # Use NEW positions!
                                manual_mode=False,
                                use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
                                visualize_potential=SIMULATION_CONFIG['visualize_potential']
                            )
                            
                            print(f"✅ 2D environment completely recreated from scratch!")
                            print(f"  • New cells with objects: {len(env.cells_with_objects) if hasattr(env, 'cells_with_objects') else 'unknown'}")
                            print(f"  • Fresh environment with updated object positions and recalculated heatmap")
                            
                            # Replace with the completely new visualizer and environment
                            visualizer = new_visualizer
                        
                        else:
                            print("  ℹ️ 2D environment update not needed")
                            # Still need to execute the 2D path action if no recreation was needed
                            env.execute_path(env.current_cell, env.current_path_type, use_spillage=SIMULATION_CONFIG['use_spillage_model'], precomputed_path=path_info)
                    
                    print("🎯 Trajectory execution and environment sync complete!")

                    # Cleanup visualizations only (keep environment stable)
                    integration.clear_trajectory()
                    visualizer.clear_trajectory_preview()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                else:
                    print("\nTrajectory execution cancelled.")
                    integration.clear_trajectory()
                    visualizer.clear_trajectory_preview()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None

    return True, visualizer, env

if __name__ == "__main__":
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 40
    random_seed = 10
    initial_robot_pose = (0.0, -2.0, math.pi / 2)

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
    objects_3d = integration.run(auto_continue=True)
    shovel_width = integration.shovel_width

    env, visualizer = run_2d_env(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width,
        real_objects=objects_3d,
        manual_mode=False,
        use_spillage_model=SIMULATION_CONFIG['use_spillage_model'],
        visualize_potential=SIMULATION_CONFIG['visualize_potential']
    )
    
    print(f"\n=== SIMULATION CONTROLS ===")
    print(f"Spillage Model: {'ENABLED' if SIMULATION_CONFIG['use_spillage_model'] else 'DISABLED'} (Press 'S' to toggle)")
    print(f"Potential Visualization: {'ENABLED' if SIMULATION_CONFIG['visualize_potential'] else 'DISABLED'} (Press 'V' to toggle)")
    print(f"Post-Delivery Turn: {'ENABLED' if SIMULATION_CONFIG['enable_post_delivery_turn'] else 'DISABLED'} (Press 'T' to toggle)")
    print(f"Path Extension: {'ENABLED' if PATH_EXTENSION_CONFIG['enable_extension'] else 'DISABLED'} (Press 'E' to toggle)")
    if PATH_EXTENSION_CONFIG['enable_extension']:
        print(f"  Extension Length: {PATH_EXTENSION_CONFIG['extension_length']:.1f}m, Smoothing: {PATH_EXTENSION_CONFIG['extension_smoothing']:.1f}")
    print(f"Press 'H' for help, ESC to exit")
    print("===========================\n")

    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))
        visualizer.draw_elements()
        pygame.display.flip()
        visualizer.clock.tick(30)
        running, visualizer, env = handle_2d_events(visualizer, env, integration)

    pygame.quit()
    integration.close_environment()
