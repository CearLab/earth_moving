from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np
from scipy.interpolate import splprep, splev   # requires SciPy


def make_bspline(world_pts, ds=0.06):
    """
    world_pts : list of (x, y, theta) OR (x, y) in world metres
    ds        : max distance between consecutive samples
    returns   : list (x, y, theta) uniformly spaced along a cubic B-spline
    """
    # 1.  prune duplicates < 3 cm
    pruned = [world_pts[0]]
    for pt in world_pts[1:]:
        if np.hypot(pt[0]-pruned[-1][0], pt[1]-pruned[-1][1]) > 0.03:
            pruned.append(pt)
    xy = np.array([(p[0], p[1]) for p in pruned])

    # 2.  fit spline (k = 3 unless only 2 points)
    tck, _ = splprep([xy[:,0], xy[:,1]], s=0, k=min(3, len(pruned)-1))

    # 3.  sample so ∆s ≤ ds
    length_est = np.sum(np.hypot(np.diff(xy[:,0]), np.diff(xy[:,1])))
    N = max(int(length_est/ds)+1, 2*len(pruned))
    u = np.linspace(0, 1, N)
    x_s, y_s = splev(u, tck)

    # 4.  heading from finite diff
    dx = np.gradient(x_s); dy = np.gradient(y_s)
    th = np.arctan2(dy, dx)
    return list(zip(x_s, y_s, th))


def make_spline_path(points, max_step=0.06):
    """
    * points – list[(x, y, θ)] or list[(x, y)] in WORLD coords
    * max_step – maximum segment length [m] after sampling
    Returns list[(x, y, θ)] uniformly spaced along a cubic B-spline
    """
    xy = np.asarray([(p[0], p[1]) for p in points])
    # 1) prune duplicates closer than 3 cm:
    keep = [xy[0]]
    for pt in xy[1:]:
        if np.hypot(pt[0]-keep[-1][0], pt[1]-keep[-1][1]) > 0.03:
            keep.append(pt)
    keep = np.vstack(keep)

    # 2) fit periodic='False' cubic spline through pruned points
    tck, _ = splprep([keep[:,0], keep[:,1]], s=0, k=min(3, len(keep)-1))

    # 3) sample so consecutive samples are ≤ max_step
    u_fine = np.linspace(0, 1, int(np.ceil(splev(1, tck)[0]/max_step)*len(keep)))
    x_s, y_s = splev(u_fine, tck)

    # 4) headings from finite differences
    dx = np.gradient(x_s); dy = np.gradient(y_s)
    th = np.arctan2(dy, dx)
    return list(zip(x_s, y_s, th))



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

                # choice = input("Choose path type ('target' or 'highway'): ").strip().lower()
                choice = "target"
                if choice not in ["target", "highway"]:
                    print("⚠️ Invalid choice. Try again.")
                    return True

                trajectory = env.get_trajectory(clicked_cell, choice, True)

                env.current_trajectory = trajectory
                env.current_cell = clicked_cell
                env.current_path_type = choice
                env.current_use_spillage = True

                print("\nVisualizing trajectory...")
                visualizer.set_trajectory(trajectory)
                integration.visualize_trajectory(trajectory)

                # confirm = input("\nExecute trajectory? (yes/no): ").strip().lower()
                confirm = "yes"
                if confirm in ["yes", "y"]:
                    print("\nExecuting trajectory...")
                    robot_id = integration.object_ids[1]  # assuming robot is 2nd loaded object
                    pos, quat = p.getBasePositionAndOrientation(robot_id)
                    theta = p.getEulerFromQuaternion(quat)[2]
                    current_pose = (pos[0], pos[1], theta)

                    start_grid = env.current_trajectory[0]
                    next_grid = env.current_trajectory[1] if len(env.current_trajectory) > 1 else start_grid

                    start_world = integration.coord_converter.convert_2d_to_3d(*start_grid)
                    next_world = integration.coord_converter.convert_2d_to_3d(*next_grid)
                    target_theta = math.atan2(next_world[1] - start_world[1], next_world[0] - start_world[0])
                    start_pose = (start_world[0], start_world[1], target_theta)

                    integration.set_robot_dim(L=0.2, R=0.05)

                    approach_trajectory = integration.generate_bezier_trajectory(
                        current_pose, start_pose, duration=2.0, scale=0.3
                    )

                    trajectory_3d = []
                    for i in range(len(env.current_trajectory)):
                        grid_x, grid_y = env.current_trajectory[i]
                        world_x, world_y = integration.coord_converter.convert_2d_to_3d(grid_x, grid_y)

                        if i < len(env.current_trajectory) - 1:
                            next_x, next_y = env.current_trajectory[i + 1]
                            next_world_x, next_world_y = integration.coord_converter.convert_2d_to_3d(next_x, next_y)
                            theta = math.atan2(next_world_y - world_y, next_world_x - world_x)
                        else:
                            prev_x, prev_y = env.current_trajectory[i - 1]
                            prev_world_x, prev_world_y = integration.coord_converter.convert_2d_to_3d(prev_x, prev_y)
                            theta = math.atan2(world_y - prev_world_y, world_x - prev_world_x)

                        trajectory_3d.append((world_x, world_y, theta))

                    # Build → spline → thin
                    world_path = approach_trajectory + trajectory_3d  # as before
                    bspline_path = make_bspline(world_path, ds=0.5)

                    integration.draw_trajectory([(x, y) for x, y, _ in bspline_path], color=[0, 0, 0])
                    integration.follow_smooth_trajectory(bspline_path,
                                                         lookahead=4,
                                                         pos_tol=0.05,  # 5 cm
                                                         angle_tol=0.25)
                    
                    # integration.follow_trajectory(bspline_path)
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
