from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np


def handle_2d_events(visualizer, env, integration):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            clicked_cell = visualizer.handle_click_event(pos)
            if clicked_cell:
                print(f"\nClicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")

                choice = "target"
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
                    print("\nExecuting trajectory with DIRECT WAYPOINT FOLLOWING...")
                    
                    # Convert 2D grid trajectory to 3D waypoints (cell centers)
                    waypoints_3d = []
                    for i, (grid_x, grid_y) in enumerate(env.current_trajectory):
                        world_x, world_y = integration.coord_converter.convert_2d_to_3d(grid_x, grid_y)
                        
                        # Each waypoint is the CENTER of the grid cell
                        waypoints_3d.append((world_x, world_y, 0.0))  # Simple heading for now
                    
                    print(f"Following {len(waypoints_3d)} waypoints directly...")
                    
                    # Draw the exact waypoints we'll visit
                    for i, (x, y, _) in enumerate(waypoints_3d):
                        # Add waypoint markers
                        p.addUserDebugText(str(i), [x, y, 0.1], textColorRGB=[1, 0, 0], textSize=1.0)
                        
                        if i > 0:
                            prev_x, prev_y, _ = waypoints_3d[i-1]
                            p.addUserDebugLine([prev_x, prev_y, 0.02], [x, y, 0.02], [0, 1, 0], lineWidth=2.0)
                    
                    # DIRECT WAYPOINT EXECUTION - visit each waypoint exactly
                    follow_waypoints_directly(integration, waypoints_3d)
                    
                    env.execute_path(env.current_cell, env.current_path_type, use_spillage=env.current_use_spillage)
                    env.update_environment()

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

    return True


def follow_waypoints_directly(integration, waypoints):
    """
    Visit each waypoint exactly, one by one, with simple control.
    No complex trajectory smoothing - just go to each point and stop.
    """
    print(f"🎯 Direct waypoint following: {len(waypoints)} waypoints")
    
    for i, (target_x, target_y, target_theta) in enumerate(waypoints):
        print(f"  → Waypoint {i+1}/{len(waypoints)}: ({target_x:.2f}, {target_y:.2f})")
        
        # Go to this exact waypoint using simple position control
        target_pos = (target_x, target_y, 0)
        target_quat = p.getQuaternionFromEuler([0, 0, target_theta])
        
        # Use simple position control to reach this waypoint
        reached = False
        max_iter = 500
        iteration = 0
        
        while not reached and iteration < max_iter:
            current_pos, current_quat = integration.get_robot_position()
            
            # Simple proportional controller
            error_x = target_x - current_pos[0]
            error_y = target_y - current_pos[1]
            distance = math.sqrt(error_x**2 + error_y**2)
            
            if distance < 0.05:  # 5cm tolerance
                reached = True
                print(f"    ✓ Reached waypoint {i+1} in {iteration} iterations")
                break
            
            # Simple heading towards target
            target_heading = math.atan2(error_y, error_x)
            current_heading = p.getEulerFromQuaternion(current_quat)[2]
            heading_error = target_heading - current_heading
            
            # Normalize heading error
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Simple controller
            forward_speed = 0.3 * min(distance, 0.5)  # Max 0.5 m/s, proportional to distance
            turn_speed = 0.5 * heading_error
            
            # Convert to wheel speeds
            left_speed = forward_speed - turn_speed
            right_speed = forward_speed + turn_speed
            
            # Apply control
            integration.control_rover_velocity(left_speed, right_speed, 1/240)
            iteration += 1
        
        if not reached:
            print(f"    ⚠️  Failed to reach waypoint {i+1} in {max_iter} iterations")
    
    print("🏁 Direct waypoint following completed!")


if __name__ == "__main__":
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 40
    random_seed = 10
    initial_robot_pose = (0.0, -2.5, math.pi / 2)

    integration = PyBulletIntegration(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        num_pebbles=num_pebbles,
        random_seed=random_seed,
        gui=True,
        initial_robot_pose=initial_robot_pose
    )

    integration.set_robot_dim(L=0.2, R=0.07)

    print("\nPress 'c' in the PyBullet window to continue to 2D visualization...")
    objects_3d = integration.run()
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
        running = handle_2d_events(visualizer, env, integration)

    pygame.quit()
    integration.close_environment()