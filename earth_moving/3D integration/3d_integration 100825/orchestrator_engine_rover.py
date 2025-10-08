from main import run_2d_env
import pygame
import math
import pybullet as p
from engine_rover import PyBulletEnvironment  # Use engine_rover instead of pybullet_integration
from coordinate_converter import CoordinateConverter
import numpy as np
import os


def handle_2d_events(visualizer, env, engine_env, coord_converter):
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

                confirm = "yes"
                if confirm in ["yes", "y"]:
                    print("\nExecuting trajectory...")
                    robot_id = engine_env.ID[1]  # Robot ID from engine_rover
                    pos, quat = p.getBasePositionAndOrientation(robot_id)
                    theta = p.getEulerFromQuaternion(quat)[2]
                    current_pose = (pos[0], pos[1], theta)

                    start_grid = env.current_trajectory[0]
                    next_grid = env.current_trajectory[1] if len(env.current_trajectory) > 1 else start_grid

                    start_world = coord_converter.convert_2d_to_3d(*start_grid)
                    next_world = coord_converter.convert_2d_to_3d(*next_grid)
                    target_theta = math.atan2(next_world[1] - start_world[1], next_world[0] - start_world[0])
                    start_pose = (start_world[0], start_world[1], target_theta)

                    # Generate approach trajectory using engine_rover
                    approach_trajectory = engine_env.generate_bezier_trajectory(
                        current_pose, start_pose, duration=2.0, scale=0.3
                    )

                    # Convert 2D trajectory to 3D world coordinates
                    trajectory_3d = []
                    for i in range(len(env.current_trajectory)):
                        grid_x, grid_y = env.current_trajectory[i]
                        world_x, world_y = coord_converter.convert_2d_to_3d(grid_x, grid_y)

                        if i < len(env.current_trajectory) - 1:
                            next_x, next_y = env.current_trajectory[i + 1]
                            next_world_x, next_world_y = coord_converter.convert_2d_to_3d(next_x, next_y)
                            theta = math.atan2(next_world_y - world_y, next_world_x - world_x)
                        else:
                            prev_x, prev_y = env.current_trajectory[i - 1]
                            prev_world_x, prev_world_y = coord_converter.convert_2d_to_3d(prev_x, prev_y)
                            theta = math.atan2(world_y - prev_world_y, world_x - prev_world_x)

                        trajectory_3d.append((world_x, world_y, theta))

                    # Generate simple interpolated trajectory that follows 2D algorithm path
                    if len(trajectory_3d) > 1:
                        # Simple linear interpolation between waypoints with reasonable spacing
                        task_trajectory = []
                        
                        for i in range(len(trajectory_3d)):
                            current_waypoint = trajectory_3d[i]
                            task_trajectory.append(current_waypoint)
                            
                            # Add intermediate points only if distance to next waypoint is large
                            if i < len(trajectory_3d) - 1:
                                next_waypoint = trajectory_3d[i + 1]
                                
                                # Calculate distance between waypoints
                                dx = next_waypoint[0] - current_waypoint[0]
                                dy = next_waypoint[1] - current_waypoint[1]
                                distance = math.sqrt(dx*dx + dy*dy)
                                    
                                # Only add intermediate points if distance > 0.15m (about 2 grid cells)
                                if distance > 0.15:
                                    # Add 1-2 intermediate points for smooth motion
                                    num_interp = min(2, int(distance / 0.1))  # Max 2 points, one every 10cm
                                    for j in range(1, num_interp + 1):
                                        t = j / (num_interp + 1)
                                        interp_x = current_waypoint[0] + t * dx
                                        interp_y = current_waypoint[1] + t * dy
                                        interp_theta = current_waypoint[2] + t * (next_waypoint[2] - current_waypoint[2])
                                        task_trajectory.append((interp_x, interp_y, interp_theta))
                        
                        full_trajectory = approach_trajectory + task_trajectory
                        print(f"Generated trajectory: approach({len(approach_trajectory)}) + task({len(task_trajectory)}) = {len(full_trajectory)} points")
                    else:
                        full_trajectory = approach_trajectory + trajectory_3d

                    # Draw the 2D algorithm trajectory (discrete grid path)
                    print(f"\n2D Algorithm trajectory ({len(env.current_trajectory)} waypoints):")
                    for i, (gx, gy) in enumerate(env.current_trajectory):
                        print(f"  {i}: grid({gx}, {gy})")
                    
                    # Visualize the discrete 2D trajectory in 3D
                    for i in range(len(env.current_trajectory) - 1):
                        start_grid = env.current_trajectory[i]
                        end_grid = env.current_trajectory[i + 1]
                        
                        start_world = coord_converter.convert_2d_to_3d(*start_grid)
                        end_world = coord_converter.convert_2d_to_3d(*end_grid)
                        
                        # Draw discrete trajectory in blue
                        p.addUserDebugLine(
                            [start_world[0], start_world[1], 0.02],
                            [end_world[0], end_world[1], 0.02],
                            [0, 0, 1],  # Blue color for 2D algorithm path
                            lineWidth=3.0
                        )
                    
                    # Draw and follow smooth trajectory using engine_rover methods
                    engine_env.draw_trajectory(full_trajectory, color=[1, 0, 0])  # Red for smooth trajectory
                    
                    # Call it the same way as run_trajectory.py does
                    engine_env.follow_smooth_trajectory(full_trajectory)
                    
                    env.execute_path(env.current_cell, env.current_path_type, use_spillage=env.current_use_spillage)
                    env.update_environment()

                    visualizer.clear_trajectory()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                else:
                    print("\nTrajectory execution cancelled.")
                    visualizer.clear_trajectory()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None

    return True

def create_grid_overlay(engine_env, coord_converter):
    """Create visual grid overlay in PyBullet using grid size/cell size from CoordinateConverter."""
    grid_size = coord_converter.grid_size
    cell_size = coord_converter.cell_size
    env_radius = coord_converter.env_radius
    
    # Create grid lines
    for i in range(grid_size + 1):
        pos = -env_radius + i * cell_size
        
        # Horizontal line
        line_id = p.addUserDebugLine(
            [-env_radius, pos, 0.01],
            [env_radius, pos, 0.01],
            [0.5, 0.5, 0.5, 0.5],  # Gray color
            2.0
        )
        
        # Vertical line  
        line_id = p.addUserDebugLine(
            [pos, -env_radius, 0.01],
            [pos, env_radius, 0.01],
            [0.5, 0.5, 0.5, 0.5],  # Gray color
            2.0
        )

def setup_scene_with_engine_rover(engine_env, env_radius, target_zone_radius, num_pebbles, random_seed, initial_robot_pose, coord_converter):
    """Setup the scene with pebbles and target zone using engine_rover environment."""
    
    # Set random seed for reproducible pebble placement
    np.random.seed(random_seed)
    
    # Create target zone visualization (red circle)
    target_zone_visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=target_zone_radius,
        length=0.01,
        rgbaColor=[1, 0, 0, 0.3]  # Red with transparency
    )
    target_zone_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=target_zone_visual,
        basePosition=[0, 0, 0.005]
    )
    engine_env.ID.append(target_zone_id)
    
    # Create grid overlay
    create_grid_overlay(engine_env, coord_converter)
    
    # Generate random pebble positions
    pebble_positions = []
    for _ in range(num_pebbles):
        # Generate position within environment but outside target zone
        while True:
            angle = np.random.uniform(0, 2 * np.pi)
            # Ensure pebbles are outside target zone but inside environment
            min_radius = target_zone_radius + 0.1
            max_radius = env_radius - 0.1
            radius = np.random.uniform(min_radius, max_radius)
            
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            
            if np.sqrt(x**2 + y**2) > target_zone_radius + 0.05:
                pebble_positions.append([x, y, 0.1])
                break
    
    # Load pebbles
    pebbles_urdf = os.path.join(os.path.dirname(__file__), "pebbles.urdf")
    for pos in pebble_positions:
        try:
            pebble_id = p.loadURDF(pebbles_urdf, pos, useFixedBase=False)
            engine_env.ID.append(pebble_id)
        except:
            print(f"Warning: Could not load pebble at position {pos}")
    
    print(f"Scene setup complete: {len(pebble_positions)} pebbles loaded")
    return pebble_positions

if __name__ == "__main__":
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 100
    random_seed = 10
    initial_robot_pose = (0.0, -2.5, math.pi / 2)

    # Create PyBulletEnvironment using engine_rover.py
    robot_urdf = os.path.join(os.path.dirname(__file__), "2_wheel_rover.urdf")
    engine_env = PyBulletEnvironment(gui=True)
    engine_env.open_environment(robot_urdf=robot_urdf,
                               init_pos=initial_robot_pose[:2] + (0,),
                               init_quat=p.getQuaternionFromEuler([0, 0, initial_robot_pose[2]]))
    engine_env.set_robot_dim(L=0.2, R=0.05)

    # Create coordinate converter first
    shovel_width = 0.22
    coord_converter = CoordinateConverter(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width
    )

    # Setup the scene with pebbles, target zone, and grid
    pebble_positions = setup_scene_with_engine_rover(
        engine_env, env_radius, target_zone_radius, num_pebbles, random_seed, initial_robot_pose, coord_converter
    )

    # Set camera position
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=0,
        cameraPitch=-45,
        cameraTargetPosition=[0, 0, 0]
    )

    print("\nPyBullet scene ready with pebbles and target zone!")
    print("Press ENTER in the console to continue to 2D visualization...")
    
    # Simple console input - much more reliable
    input("Press ENTER to continue...")

    # Convert pebble positions for 2D environment (need x, y, z format)
    objects_3d = [(pos[0], pos[1], pos[2]) for pos in pebble_positions]

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
        running = handle_2d_events(visualizer, env, engine_env, coord_converter)

    pygame.quit()
    engine_env.close_environment()