from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np


class PathCenteredController:
    def __init__(self, wheelbase=0.2, cross_track_gain=2.0, heading_gain=1.0):
        self.wheelbase = wheelbase
        self.cross_track_gain = cross_track_gain  # How aggressively to correct lateral error
        self.heading_gain = heading_gain  # How aggressively to align with path heading
        
    def find_closest_point_on_path(self, current_pos, trajectory, start_index=0):
        """Find the closest point on the trajectory and the cross-track error."""
        min_distance = float('inf')
        closest_index = start_index
        
        # Search for closest point starting from current position
        search_range = min(20, len(trajectory) - start_index)
        for i in range(start_index, min(start_index + search_range, len(trajectory))):
            point = trajectory[i]
            distance = math.hypot(point[0] - current_pos[0], point[1] - current_pos[1])
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        return closest_index, min_distance
    
    def compute_cross_track_error(self, current_pos, trajectory, path_index):
        """Compute lateral deviation from the path."""
        if path_index >= len(trajectory) - 1:
            return 0.0, 0.0
            
        # Get path segment
        p1 = trajectory[path_index]
        p2 = trajectory[min(path_index + 1, len(trajectory) - 1)]
        
        # Path vector
        path_dx = p2[0] - p1[0]
        path_dy = p2[1] - p1[1]
        path_length = math.hypot(path_dx, path_dy)
        
        if path_length < 1e-6:
            return 0.0, 0.0
            
        # Normalize path vector
        path_dx /= path_length
        path_dy /= path_length
        
        # Vector from path point to robot
        robot_dx = current_pos[0] - p1[0]
        robot_dy = current_pos[1] - p1[1]
        
        # Cross track error (perpendicular distance from path)
        cross_track_error = robot_dx * (-path_dy) + robot_dy * path_dx
        
        # Path heading
        path_heading = math.atan2(path_dy, path_dx)
        
        return cross_track_error, path_heading
    
    def compute_control(self, current_pos, current_heading, trajectory, path_index):
        """Compute steering to stay centered on path."""
        
        # Get cross track error and desired path heading
        cross_track_error, path_heading = self.compute_cross_track_error(
            current_pos, trajectory, path_index
        )
        
        # Heading error
        heading_error = path_heading - current_heading
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Combined steering command
        # - Correct cross-track error (lateral deviation)
        # - Correct heading error (angular alignment)
        steering_angle = (-self.cross_track_gain * cross_track_error + 
                         self.heading_gain * heading_error)
        
        # Limit steering angle
        max_steering = math.pi / 3  # ±60 degrees
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        return steering_angle, cross_track_error, heading_error


def follow_trajectory_path_centered(integration, trajectory):
    """Follow trajectory using path-centered controller that stays on the path."""
    print(f"🎯 Path-centered following: {len(trajectory)} waypoints")
    print("Keeping robot centered on trajectory...")
    
    controller = PathCenteredController(
        wheelbase=0.2,
        cross_track_gain=3.0,  # Aggressive lateral correction
        heading_gain=2.0       # Strong heading alignment
    )
    
    current_index = 0
    max_iterations = len(trajectory) * 100
    iteration = 0
    
    target_speed = 1.5  # Even faster as requested!
    
    # Statistics tracking
    max_cross_track_error = 0.0
    total_cross_track_error = 0.0
    
    while current_index < len(trajectory) - 1 and iteration < max_iterations:
        current_pos, current_quat = integration.get_robot_position()
        current_heading = p.getEulerFromQuaternion(current_quat)[2]
        
        # Find closest point on path
        closest_index, distance_to_closest = controller.find_closest_point_on_path(
            current_pos, trajectory, current_index
        )
        current_index = closest_index
        
        # Compute path-centered control
        steering_angle, cross_track_error, heading_error = controller.compute_control(
            current_pos, current_heading, trajectory, current_index
        )
        
        # Track statistics
        abs_error = abs(cross_track_error)
        if abs_error > max_cross_track_error:
            max_cross_track_error = abs_error
        total_cross_track_error += abs_error
        
        # Debug output
        if iteration % 150 == 0:
            print(f"  Step {iteration}: index={current_index}/{len(trajectory)}")
            print(f"    Cross-track error: {cross_track_error:.3f}m")
            print(f"    Heading error: {math.degrees(heading_error):.1f}°")
            print(f"    Steering: {math.degrees(steering_angle):.1f}°")
            print(f"    Max error so far: {max_cross_track_error:.3f}m")
        
        # Convert to wheel speeds
        turn_rate = math.tan(steering_angle) / controller.wheelbase
        left_speed = target_speed - (turn_rate * target_speed * controller.wheelbase / 2)
        right_speed = target_speed + (turn_rate * target_speed * controller.wheelbase / 2)
        
        # Speed limits
        max_wheel_speed = 2.0
        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))
        
        # Apply control (remember the inversion!)
        integration.control_rover_velocity(-left_speed, -right_speed, 1/240)
        iteration += 1
        
        # Advance waypoint when close enough
        if distance_to_closest < 0.08:  # 8cm tolerance
            current_index = min(current_index + 1, len(trajectory) - 1)
            if current_index % 10 == 0:
                print(f"  → Progress: {current_index}/{len(trajectory)} waypoints")
        
        # Force advancement if stuck
        elif iteration > 0 and iteration % 200 == 0:
            print(f"  ⚠️  Forcing advancement (stuck on {current_index})")
            current_index = min(current_index + 1, len(trajectory) - 1)
        
        # Check completion
        final_distance = math.hypot(
            trajectory[-1][0] - current_pos[0], 
            trajectory[-1][1] - current_pos[1]
        )
        if final_distance < 0.05:
            break
    
    avg_error = total_cross_track_error / max(iteration, 1)
    print(f"🏁 Path-centered following completed!")
    print(f"📊 Performance: Max error={max_cross_track_error:.3f}m, Avg error={avg_error:.3f}m")


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
                    print("\nExecuting trajectory with PATH-CENTERED controller...")
                    
                    # Get current robot position
                    robot_id = integration.object_ids[1]
                    pos, quat = p.getBasePositionAndOrientation(robot_id)
                    theta = p.getEulerFromQuaternion(quat)[2]
                    current_pose = (pos[0], pos[1], theta)
                    
                    # Get start of 2D algorithm trajectory
                    start_grid = env.current_trajectory[0]
                    next_grid = env.current_trajectory[1] if len(env.current_trajectory) > 1 else start_grid
                    
                    start_world = integration.coord_converter.convert_2d_to_3d(*start_grid)
                    next_world = integration.coord_converter.convert_2d_to_3d(*next_grid)
                    target_theta = math.atan2(next_world[1] - start_world[1], next_world[0] - start_world[0])
                    start_pose = (start_world[0], start_world[1], target_theta)
                    
                    print(f"Robot at: ({current_pose[0]:.2f}, {current_pose[1]:.2f})")
                    print(f"Task starts at: ({start_pose[0]:.2f}, {start_pose[1]:.2f})")
                    
                    # Generate approach trajectory
                    approach_trajectory = integration.generate_bezier_trajectory(
                        current_pose, start_pose, duration=2.0, scale=0.3
                    )
                    print(f"Approach trajectory: {len(approach_trajectory)} points")
                    
                    # Convert 2D grid trajectory to 3D path points (no intermediate points)
                    task_points = []
                    for i, (grid_x, grid_y) in enumerate(env.current_trajectory):
                        world_x, world_y = integration.coord_converter.convert_2d_to_3d(grid_x, grid_y)
                        task_points.append((world_x, world_y, 0.0))
                    
                    print(f"Task trajectory: {len(task_points)} points")
                    
                    # Combine trajectories
                    full_path = approach_trajectory + task_points
                    print(f"Total path: {len(full_path)} points")
                    
                    # Enhanced visualization
                    # Approach trajectory (green)
                    for i in range(len(approach_trajectory) - 1):
                        p.addUserDebugLine(
                            [approach_trajectory[i][0], approach_trajectory[i][1], 0.02],
                            [approach_trajectory[i+1][0], approach_trajectory[i+1][1], 0.02],
                            [0, 1, 0], lineWidth=3.0  # Green
                        )
                    
                    # Task trajectory (blue)  
                    for i in range(len(task_points) - 1):
                        p.addUserDebugLine(
                            [task_points[i][0], task_points[i][1], 0.02],
                            [task_points[i+1][0], task_points[i+1][1], 0.02],
                            [0, 0, 1], lineWidth=2.0  # Blue
                        )
                    
                    # Original 2D algorithm path (red reference)
                    for i in range(len(env.current_trajectory) - 1):
                        start_grid = env.current_trajectory[i]
                        end_grid = env.current_trajectory[i + 1]
                        
                        start_world = integration.coord_converter.convert_2d_to_3d(*start_grid)
                        end_world = integration.coord_converter.convert_2d_to_3d(*end_grid)
                        
                        p.addUserDebugLine(
                            [start_world[0], start_world[1], 0.03],
                            [end_world[0], end_world[1], 0.03],
                            [1, 0, 0], lineWidth=1.5  # Red
                        )
                    
                    # Follow using path-centered control
                    follow_trajectory_path_centered(integration, full_path)
                    
                    env.execute_path(env.current_cell, env.current_path_type, use_spillage=env.current_use_spillage)
                    env.update_environment()

                    integration.clear_trajectory()
                    visualizer.clear_trajectory()

                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None

    return True


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