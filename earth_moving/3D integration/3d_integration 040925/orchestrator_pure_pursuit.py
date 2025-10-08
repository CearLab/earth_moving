from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np


class PurePursuitController:
    def __init__(self, look_ahead_distance=0.2, wheelbase=0.2):
        self.look_ahead_distance = look_ahead_distance
        self.wheelbase = wheelbase
        
    def find_lookahead_point(self, current_pos, trajectory, current_index):
        """Find the lookahead point on the trajectory."""
        for i in range(current_index, len(trajectory)):
            point = trajectory[i]
            distance = math.hypot(point[0] - current_pos[0], point[1] - current_pos[1])
            if distance >= self.look_ahead_distance:
                return i, point
        
        # If no point found at lookahead distance, return the last point
        return len(trajectory) - 1, trajectory[-1]
    
    def compute_steering_angle(self, current_pos, current_heading, lookahead_point):
        """Compute steering angle using pure pursuit algorithm."""
        # Vector from current position to lookahead point
        dx = lookahead_point[0] - current_pos[0]
        dy = lookahead_point[1] - current_pos[1]
        
        # Distance to lookahead point
        distance = math.hypot(dx, dy)
        
        if distance < 1e-6:  # Very close to target
            return 0.0
        
        # Angle to lookahead point in global frame
        angle_to_point = math.atan2(dy, dx)
        
        # Angle relative to vehicle heading
        alpha = angle_to_point - current_heading
        
        # Normalize angle to [-π, π]
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        
        # Check if lookahead point is behind us
        if abs(alpha) > math.pi / 2:
            # Point is behind us, this shouldn't happen with proper lookahead
            print(f"    ⚠️  Lookahead point behind robot! alpha={math.degrees(alpha):.1f}°")
            # Force small forward steering
            alpha = math.copysign(math.pi / 4, alpha)
        
        # Pure pursuit steering angle (limit to reasonable range)
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), distance)
        steering_angle = max(-math.pi/3, min(math.pi/3, steering_angle))  # Limit to ±60°
        
        return steering_angle


def follow_trajectory_pure_pursuit(integration, trajectory):
    """Follow trajectory using Pure Pursuit controller."""
    print(f"🚗 Pure Pursuit following: {len(trajectory)} waypoints")
    print("🔍 Starting detailed debugging...")
    
    controller = PurePursuitController(look_ahead_distance=0.1, wheelbase=0.2)  # Increased lookahead for higher speed
    
    current_index = 0
    max_iterations = len(trajectory) * 50
    iteration = 0
    
    target_speed = 1.0  # m/s (increased from 0.3)
    
    while current_index < len(trajectory) - 1 and iteration < max_iterations:
        current_pos, current_quat = integration.get_robot_position()
        current_heading = p.getEulerFromQuaternion(current_quat)[2]
        
        # Debug robot state every 100 iterations (less frequent due to higher speed)
        if iteration % 100 == 0:
            print(f"\n--- Iteration {iteration} ---")
            print(f"Robot pos: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
            print(f"Robot heading: {math.degrees(current_heading):.1f}°")
            print(f"Current trajectory index: {current_index}/{len(trajectory)}")
        
        # Find current closest point on trajectory
        min_distance = float('inf')
        closest_index = current_index
        for i in range(current_index, len(trajectory)):
            point = trajectory[i]
            distance = math.hypot(point[0] - current_pos[0], point[1] - current_pos[1])
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        current_index = closest_index
        
        # Debug closest point
        if iteration % 100 == 0:
            closest_point = trajectory[current_index]
            print(f"Closest point [{current_index}]: ({closest_point[0]:.3f}, {closest_point[1]:.3f}), dist={min_distance:.3f}")
        
        # Check if we're close enough to advance
        if min_distance < 0.10:  # 10cm tolerance (more generous)
            current_index = min(current_index + 1, len(trajectory) - 1)  # Advance one step
            if current_index % 5 == 0:
                print(f"  → Progress: {current_index}/{len(trajectory)} waypoints")
        
        # Force advancement if stuck on same waypoint too long
        elif iteration > 0 and iteration % 100 == 0:
            print(f"  ⚠️  Forcing waypoint advancement (stuck on {current_index})")
            current_index = min(current_index + 1, len(trajectory) - 1)
        
        # Find lookahead point
        lookahead_index, lookahead_point = controller.find_lookahead_point(
            current_pos, trajectory, current_index
        )
        
        # Debug lookahead point
        if iteration % 100 == 0:
            print(f"Lookahead point [{lookahead_index}]: ({lookahead_point[0]:.3f}, {lookahead_point[1]:.3f})")
            
            # Check direction to lookahead point
            dx = lookahead_point[0] - current_pos[0]
            dy = lookahead_point[1] - current_pos[1]
            angle_to_target = math.atan2(dy, dx)
            relative_angle = angle_to_target - current_heading
            while relative_angle > math.pi:
                relative_angle -= 2 * math.pi
            while relative_angle < -math.pi:
                relative_angle += 2 * math.pi
            
            print(f"Angle to lookahead: {math.degrees(angle_to_target):.1f}° (global)")
            print(f"Relative angle: {math.degrees(relative_angle):.1f}° ({'FORWARD' if abs(relative_angle) < 90 else 'BACKWARD'})")
        
        # Compute steering angle
        steering_angle = controller.compute_steering_angle(
            current_pos, current_heading, lookahead_point
        )
        
        # Simpler differential drive control
        # Use steering angle to determine turn rate
        turn_rate = math.tan(steering_angle) / controller.wheelbase
        
        # Differential drive: V_L = V - ω*L/2, V_R = V + ω*L/2  
        left_speed = target_speed - (turn_rate * target_speed * controller.wheelbase / 2)
        right_speed = target_speed + (turn_rate * target_speed * controller.wheelbase / 2)
        
        # Debug the calculation
        if iteration % 100 == 0:
            print(f"Steering angle: {math.degrees(steering_angle):.1f}°")
            print(f"Turn rate: {turn_rate:.3f}")
            print(f"Raw speeds - left: {left_speed:.3f}, right: {right_speed:.3f}")
        
        # Always ensure forward motion base speed
        min_forward_speed = 0.1
        if left_speed < min_forward_speed and right_speed < min_forward_speed:
            # Both too slow, boost forward motion
            speed_boost = min_forward_speed - min(left_speed, right_speed)
            left_speed += speed_boost
            right_speed += speed_boost
            if iteration % 100 == 0:
                print(f"Applied speed boost: {speed_boost:.3f}")
        
        # Limit maximum speeds
        max_speed = 5  # Increased from 0.5 to match higher target speed
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))
        
        # Final debug output
        if iteration % 100 == 0:
            print(f"Final speeds - left: {left_speed:.3f}, right: {right_speed:.3f}")
            direction = "FORWARD" if (left_speed + right_speed) > 0 else "BACKWARD"
            print(f"Overall motion: {direction}")
        
        # Apply control - TRY INVERTING THE SPEEDS
        integration.control_rover_velocity(-left_speed, -right_speed, 1/240)
        iteration += 1
        
        # Check if we've reached the end
        final_distance = math.hypot(
            trajectory[-1][0] - current_pos[0], 
            trajectory[-1][1] - current_pos[1]
        )
        if final_distance < 0.05:
            break
    
    print(f"🏁 Pure Pursuit completed in {iteration} iterations!")


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
                    print("\nExecuting trajectory with PURE PURSUIT controller...")
                    
                    # Get current robot position
                    robot_id = integration.object_ids[1]  # assuming robot is 2nd loaded object
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
                    
                    # Generate approach trajectory using Bezier
                    approach_trajectory = integration.generate_bezier_trajectory(
                        current_pose, start_pose, duration=2.0, scale=0.3
                    )
                    print(f"Approach trajectory: {len(approach_trajectory)} points")
                    
                    # Convert 2D grid trajectory to 3D path points (NO intermediate points - too many!)
                    task_points = []
                    for i, (grid_x, grid_y) in enumerate(env.current_trajectory):
                        world_x, world_y = integration.coord_converter.convert_2d_to_3d(grid_x, grid_y)
                        task_points.append((world_x, world_y, 0.0))
                        
                        # NO intermediate points - they create too many waypoints!
                    
                    print(f"Task trajectory: {len(task_points)} points")
                    
                    # Combine approach + task trajectories
                    full_path = approach_trajectory + task_points
                    print(f"Total path: {len(full_path)} points")
                    
                    path_points = full_path
                    
                    # Draw the approach trajectory (green)
                    for i in range(len(approach_trajectory) - 1):
                        p.addUserDebugLine(
                            [approach_trajectory[i][0], approach_trajectory[i][1], 0.02],
                            [approach_trajectory[i+1][0], approach_trajectory[i+1][1], 0.02],
                            [0, 1, 0], lineWidth=3.0  # Green for approach
                        )
                    
                    # Draw the task trajectory (blue)
                    for i in range(len(task_points) - 1):
                        p.addUserDebugLine(
                            [task_points[i][0], task_points[i][1], 0.02],
                            [task_points[i+1][0], task_points[i+1][1], 0.02],
                            [0, 0, 1], lineWidth=2.0  # Blue for task trajectory
                        )
                    
                    # Draw discrete 2D algorithm path (red) for comparison
                    for i in range(len(env.current_trajectory) - 1):
                        start_grid = env.current_trajectory[i]
                        end_grid = env.current_trajectory[i + 1]
                        
                        start_world = integration.coord_converter.convert_2d_to_3d(*start_grid)
                        end_world = integration.coord_converter.convert_2d_to_3d(*end_grid)
                        
                        p.addUserDebugLine(
                            [start_world[0], start_world[1], 0.03],
                            [end_world[0], end_world[1], 0.03],
                            [1, 0, 0], lineWidth=1.0  # Red for discrete 2D path
                        )
                    
                    # Follow using Pure Pursuit
                    follow_trajectory_pure_pursuit(integration, path_points)
                    
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