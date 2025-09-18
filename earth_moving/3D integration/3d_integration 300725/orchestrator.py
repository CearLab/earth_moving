from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np


# ===================== PURE PURSUIT CONFIG =====================
# Pure Pursuit configuration - can be overridden per trajectory
PURE_PURSUIT_CONFIG = {
    'mode': 'TCP',           # 'TCP' or 'BASE_SHIFT'
    'tcp_fwd': 0.12,        # TCP forward offset
    'tcp_lat': 0.00,        # TCP lateral offset  
    'v_nom': 0.40,          # Nominal velocity
    'a_lat_max': 1.2,       # Max lateral acceleration
    'yaw_slew_rate': 6.0,   # Yaw slew rate limiting
    'lookahead': 0.10,      # Lookahead distance
    'resample_ds': 0.05,    # Path resampling distance
    'draw_tool_tick': True  # Draw TCP marker
}

def handle_2d_events(visualizer, env, integration):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False, visualizer, env
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False, visualizer, env
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

                trajectory = env.get_trajectory(clicked_cell, choice, True)

                env.current_trajectory = trajectory
                env.current_cell = clicked_cell
                env.current_path_type = choice
                env.current_use_spillage = True

                print("\nVisualizing trajectory...")
                visualizer.set_trajectory(trajectory)
                integration.visualize_trajectory(trajectory)

                confirm = "yes"
                if confirm in ["yes", "y"]:
                    # Build world path (approach + task) and resample - LIKE ORIGINAL
                    approach, task_world, path_world = integration.build_world_path(
                        env, 
                        approach_duration=2.0, 
                        approach_scale=0.3, 
                        ds=PURE_PURSUIT_CONFIG['resample_ds']
                    )
                    print(f"Approach: {len(approach)} pts, Task: {len(task_world)} pts, Path: {len(path_world)} pts")

                    # Handle different modes - LIKE ORIGINAL
                    if PURE_PURSUIT_CONFIG['mode'].upper() == "BASE_SHIFT":
                        from pybullet_integration import shift_path_along_s
                        base_path = shift_path_along_s(path_world, s_shift=-PURE_PURSUIT_CONFIG['tcp_fwd'])
                        path_for_controller = base_path
                        use_tcp_pose = False
                        # draw shifted path (purple)
                        for i in range(len(base_path) - 1):
                            p.addUserDebugLine([base_path[i][0], base_path[i][1], 0.035],
                                               [base_path[i+1][0], base_path[i+1][1], 0.035],
                                               [0.6, 0.0, 0.6], lineWidth=2.0)
                    else:
                        # TCP mode: controller uses the tool pose and the original resampled path
                        path_for_controller = path_world
                        use_tcp_pose = True

                    # Draw approach (green) and task (blue) - LIKE ORIGINAL  
                    for i in range(len(approach) - 1):
                        p.addUserDebugLine([approach[i][0], approach[i][1], 0.02],
                                           [approach[i+1][0], approach[i+1][1], 0.02],
                                           [0, 1, 0], lineWidth=3.0)
                    for i in range(len(task_world) - 1):
                        p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                                           [task_world[i+1][0], task_world[i+1][1], 0.02],
                                           [0, 0, 1], lineWidth=2.0)

                    # Draw resampled path (black) to verify spacing - LIKE ORIGINAL
                    for i in range(len(path_world) - 1):
                        p.addUserDebugLine([path_world[i][0], path_world[i][1], 0.03],
                                           [path_world[i+1][0], path_world[i+1][1], 0.03],
                                           [0, 0, 0], lineWidth=2.0)

                    # Execute Pure Pursuit - LIKE ORIGINAL
                    integration.follow_trajectory_pure_pursuit(
                        path_for_controller,
                        dt=1/240,
                        lookahead=PURE_PURSUIT_CONFIG['lookahead'],
                        v_nom=PURE_PURSUIT_CONFIG['v_nom'],
                        a_lat_max=PURE_PURSUIT_CONFIG['a_lat_max'],
                        yaw_slew=PURE_PURSUIT_CONFIG['yaw_slew_rate'],
                        use_tcp_pose=use_tcp_pose
                    )

                    # Post-trajectory processing
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
                            visualizer.clear_trajectory()
                            
                            # COMPLETELY recreate 2D environment from scratch (no state preservation!)
                            from main import run_2d_env
                            env, new_visualizer = run_2d_env(
                                env_radius=1.0,  # Same as orchestrator main
                                target_zone_radius=0.3,
                                shovel_width=integration.shovel_width,
                                real_objects=updated_positions_3d,  # Use NEW positions!
                                manual_mode=False
                            )
                            
                            print(f"✅ 2D environment completely recreated from scratch!")
                            print(f"  • New cells with objects: {len(env.cells_with_objects) if hasattr(env, 'cells_with_objects') else 'unknown'}")
                            print(f"  • Fresh environment with updated object positions and recalculated heatmap")
                            
                            # Replace with the completely new visualizer and environment
                            visualizer = new_visualizer
                        
                        else:
                            print("  ℹ️ 2D environment update not needed")
                            # Still need to execute the 2D path action if no recreation was needed
                            env.execute_path(env.current_cell, env.current_path_type, use_spillage=env.current_use_spillage)
                    
                    print("🎯 Trajectory execution and environment sync complete!")

                    # Cleanup visualizations only (keep environment stable)
                    integration.clear_trajectory()
                    visualizer.clear_trajectory()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                else:
                    print("\nTrajectory execution cancelled.")
                    integration.clear_trajectory()
                    visualizer.clear_trajectory()
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
        manual_mode=False
    )

    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))
        visualizer.draw_elements()
        pygame.display.flip()
        visualizer.clock.tick(30)
        running, visualizer, env = handle_2d_events(visualizer, env, integration)

    pygame.quit()
    integration.close_environment()
