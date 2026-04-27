import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

import time
import gc
import numpy as np
import cv2
import matplotlib.pyplot as plt
from push_environment_gym_wrapper import PushEnvironmentGym
from preprocess_environment_v1 import PreprocessEnvironment
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "algorithms" / "rl-agents"))
from rl_agents.agents.tree_search.mcts import MCTSAgent

RECT_SIZE = (700, 500) # size of the rectangle area in the top-down view (in pixels or mm)
SQUARE_SIZE = 500     # size of the square area in the top-down view (in pixels or mm)
np.random.seed(42)


def initialize_environment_and_agent(raw_image, transformation_matrix):
    # Define environment and agent parameters
    # grid_size = 25 # number of cells in the grid (e.g., 25x25)
    # grid_size = 24 # number of cells in the grid
    grid_size = 16 # number of cells in the grid
    shovel_width = 0.25 # width of the shovel in the same units as the grid (e.g., 0.25 for 25% of the full grid size)
    # shovel_width = 0.26 # width of the shovel in the same units as the grid (e.g., 0.25 for 25% of the full grid size)
    s0 = 0  # Start at the first boundary oint
    max_pushes = 3  # Maximum number of pushes allowed
    coverage_threshold = 0.95  # Coverage threshold for stopping

    boundary_points = [(0, y) for y in np.linspace(0, 1, grid_size, endpoint=False)] + \
                [(x, 1) for x in np.linspace(0, 1, grid_size, endpoint=False)] + \
                [(1, y) for y in np.linspace(1, 0, grid_size, endpoint=False)] + \
                [(x, 0) for x in np.linspace(1, 0, grid_size, endpoint=False)]
    # delta_s_options = list(np.arange(-int(len(boundary_points)/2), int(len(boundary_points)/2)))
    delta_s_options = list(np.arange(-int(len(boundary_points)/2), int(len(boundary_points)/2), 2))
    theta_options = np.linspace(0, np.pi, 9)  # 8 orientations (0, 22.5, 45, ..., 157.5 degrees)

    # Initialize preprocessor module and preprocess the environment
    preprocess = PreprocessEnvironment(    
        boundary_points=boundary_points,
        push_width=shovel_width,  
        stochasticity=True,
        grid_size=grid_size,
        binary_grid=False
        )
    
    # binary_mask = preprocess.transform_raw_image_to_binary_mask_old(raw_image, transformation_matrix, rect_size=RECT_SIZE, square_size=SQUARE_SIZE, debug=True)
    # flipped_env_image = cv2.flip(binary_mask, 0)
    # occ_grid, total_ooccupied = preprocess.binary_img_to_grid(flipped_env_image)
    # total_pixels_in_cell = preprocess.total_pixels_in_cell
    
    occ_grid, total_ooccupied = preprocess.transform_raw_image_to_grid_old(raw_image, transformation_matrix, rect_size=RECT_SIZE, square_size=SQUARE_SIZE, debug=True)
    total_pixels_in_cell = preprocess.total_pixels_in_cell
    
    # Create environment
    stochastic_env = PushEnvironmentGym(
                                        boundary_points=boundary_points,
                                        delta_s_options=delta_s_options,
                                        theta_options=theta_options,
                                        push_width=shovel_width, 
                                        initial_pos=s0,          
                                        max_pushes=max_pushes,
                                        coverage_thresh=coverage_threshold,
                                        occupancy_grid=occ_grid,
                                        total_occupied=total_ooccupied,
                                        total_pixels_in_cell=total_pixels_in_cell,
                                        stochasticity=True,
                                        grid_size=grid_size
                                    )
    stochastic_env.draw_grid_environment(stochastic_env.initial_state['grid_mask'])
    # Create MCTS agent
    # agent = MCTSAgent(env=stochastic_env, config=dict(horizon=max_pushes, episodes=10000, gamma=0.8, max_depth=max_pushes, temperature=50))
    agent = MCTSAgent(env=stochastic_env, config=dict(horizon=max_pushes, episodes=10000, gamma=0.8, max_depth=5, temperature=50))

    return stochastic_env, agent

def run_offline_mcts_planning(env, agent):
    # Reset environment and get initial state
    state, info = env.reset()
    done = truncated = False
    total_reward = 0

    initial_state = env.initial_state.copy()
    current_state = state
    push_env = env.get_push_env()

    pushes = []
    normed_pushes = []
    normed_path = []
    global_pushes = []
    global_path = []

    normed_start_point = push_env.transform_s_to_normed_world(initial_state['s_idx'])
    alpha_start = 0.0  # Initial alpha is 0.0
    alpha = alpha_start
    normed_path.append((normed_start_point, alpha))  # Add initial position with initisl alpha of 0.0
    global_start_point = push_env.transform_normed_world_to_global(normed_start_point)
    alpha_global = alpha  # Initial alpha is 0.0
    global_path.append((global_start_point, alpha_global))  # Add initial position and alpha to global path

    print("number of actions={}".format(len(push_env.action_space)))

    print("Initial state:")
    print(f"Start idx={initial_state['s_idx']}")
    print("Starting MCTS planning...")
    gc.disable() # Disable during the heavy planning phase to prevent unnecessary overhead from garbage collection
    start_planning = time.time()
    optimal_path = agent.act_offline(state)
    print(f"Planning took: {time.time() - start_planning} seconds")
    gc.enable()
    print("Optimal path (action indices):", optimal_path)
    for step, action in enumerate(optimal_path):
        # MCTS planning to find best action
        print("############ mcts action is", action)
        
        # Convert action index back to delta_s, theta
        # The action_space in the new environment maps indices to (delta_s, theta) pairs
        if isinstance(action, (int, np.integer)):
            delta_s, theta = push_env.action_space[action]
        else:
            delta_s, theta = action
        # delta_s, theta = push_env.action_space[action]
        print(f"delta_s: {delta_s}, theta: {np.degrees(theta):.1f} deg")
        
        prev_state = state
        s_before = prev_state['s_idx']

        # Take the action in the environment
        state, reward, done, truncated, info = env.step(action)

        print("############ state is", state)
        print("############ reward is", reward)
        print("############ done", done)

        total_reward += reward
        
        push_len = info.get('t_max', 0)
        print("push_len", push_len)
        
        if delta_s is not None and theta is not None:
            print(f"Push {step+1}: Start idx={s_before}, delta_s={delta_s}, theta={np.degrees(theta):.1f} deg, reward={reward:.4f}")
            print("length", state['length'])

        uncovered = np.count_nonzero(state['grid_mask'])
        coverage = env.compute_coverage(state['grid_mask'])
        print(f"coverage: {coverage} \n")

        normed_corner1, normed_corner2 = push_env.find_boundry_corners_in_ds_path(s_before, delta_s)
        print(f"s_corner1: {normed_corner1}, s_corner2: {normed_corner2}")
        if normed_corner1 is not None:
            normed_path.append((normed_corner1, alpha))
            global_corner1 = push_env.transform_normed_world_to_global(normed_corner1)
            global_path.append((global_corner1, alpha_global))
        if normed_corner2 is not None:
            normed_path.append((normed_corner2, alpha))
            global_corner2 = push_env.transform_normed_world_to_global(normed_corner2)
            global_path.append((global_corner2, alpha_global))
        start_point, end_point, alpha = push_env.transform_push_to_normed_world(s_before, delta_s, theta)
        print(f"Normed start_point: {start_point}, Normed end_point: {end_point}, Normed alpha: {alpha}")
        normed_path.append((start_point, alpha))
        normed_path.append((end_point, alpha))
    
        start_point_global, end_point_global, alpha_global = push_env.transform_push_to_global_world(s_before, delta_s, theta)
        print(f"Global start_point: {start_point_global}, Global end_point: {end_point_global}, Global alpha: {alpha_global}")
        global_path.append((start_point_global, alpha_global))
        global_path.append((end_point_global, alpha_global))

        direction = np.array([np.cos(alpha), np.sin(alpha)])

        pushes.append((start_point, direction))
        normed_pushes.append((start_point, end_point, alpha))
        # TODO: Add delta_s movement along the boundary to the path
        global_pushes.append((start_point_global, end_point_global, alpha_global))

        env.draw_grid_environment(prev_state['grid_mask'], pushes=pushes, current_push=(start_point, direction))
        env.draw_grid_environment(state['grid_mask'], pushes=pushes, current_push=(start_point, direction))

        if done or truncated:
            print(f"Episode finished after {step+1} steps with total reward {total_reward:.4f}")

    print(f"Coverage threshold reached: {coverage*100:.2f}%")
    print("OR Reached the end of the environment or maximum pushes.")
    print("final_coverage:", env.compute_coverage(state['grid_mask']), "final_length:", state['length'], "final_reward:", reward)
    # print("final_coverage:", env.compute_coverage(state['grid_mask']), "final_length:", state['length'], "final_reward:", reward)
    # max_total_length = ((grid_size * (1 / shovel_width)) + grid_size - shovel_width * 0.5)
    # normed_length = state['length'] / max_total_length
    # print("normed_length:", normed_length)
    # print("cov/length ratio:", coverage / normed_length)
    print("total_reward:", total_reward)

    return optimal_path, normed_path, global_path

def trnsformation_from_global_to_robot_base_coordinates(global_pos):
    # This function should implement the transformation from global coordinates to robot base coordinates
    # TODO: Implement the actual transformation based on the robot's position and orientation in the global frame
    x_base, y_base = 0.36, 0.845  # Robot base position in global coordinates
    theta_base = np.pi  # Robot base orientation in radians

    # Convert global_pos to robot base coordinates
    x_global, y_global = global_pos
    x_robot = (x_global - x_base) * np.cos(theta_base) + (y_global - y_base) * np.sin(theta_base)
    y_robot = -(x_global - x_base) * np.sin(theta_base) + (y_global - y_base) * np.cos(theta_base)

    return (x_robot, y_robot)

def transform_normed_yaw_to_robot_yaw(alpha):
    # Convert a normalized yaw angle to global world coordinates using the bounding box size and offset
    # Assuming alpha is in radians and normalized to [0, 2*pi]
    robot_alpha = alpha  # TODO CHECK if any transformation is needed based on the bounding box orientation
    
    return robot_alpha

def transform_path_to_robot_base_coordinates(path):
    # This function takes a path defined in global coordinates and transforms it to robot base coordinates
    # TODO: Add delta_s movement along the boundary to the path
    robot_base_path = []
    for (point, alpha) in path:
        point_robot = trnsformation_from_global_to_robot_base_coordinates(point)
        yaw_robot = transform_normed_yaw_to_robot_yaw(alpha)
        robot_base_path.append((point_robot, yaw_robot))
    return robot_base_path

def convert_path_to_csv_format(optimal_path):
    # This function converts the path to a CSV format suitable for the robot's control system
    csv_lines = []
    csv_lines.append("x  , y  , z    , ro , pi , ya , planner_type, execute, left_angle, right_angle")
    for (point, alpha) in optimal_path:
        x, y = point
        alpha_degrees = np.degrees(alpha)
        csv_lines.append(f"{x},{y},0.045,180,0,{alpha_degrees},cartesian,true,45,30")
    csv_path = "robot_path.csv"
    with open(csv_path, "w") as f:
        f.write("\n".join(csv_lines))
    return csv_path
    
if __name__ == "__main__":
    # Define paths
    base_dir = Path(__file__).resolve().parent
    batch_path = base_dir / "real_data" / "red_marks_old"
    image_path = batch_path / "raw" / "WIN_20260223_14_55_32_Pro.jpg"
    matrix_path = batch_path / "transformation_matrix.csv"

    if not image_path.exists():
        raise FileNotFoundError(f"Image file not found: {image_path}")
    if not matrix_path.exists():
        raise FileNotFoundError(f"Transformation matrix file not found: {matrix_path}")

    # Load raw image
    raw_image = plt.imread(image_path)
    plt.figure()
    plt.imshow(cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB))
    # Load transformation matrix
    transformation_matrix = np.loadtxt(matrix_path, delimiter=",")
    
    # Preprocess raw image and create environment and agent
    stochastic_env, agent = initialize_environment_and_agent(raw_image, transformation_matrix)
    _, normed_optimal_path, global_optimal_path = run_offline_mcts_planning(stochastic_env, agent)
    # print("Global optimal path (start_point_global, end_point_global, alpha_global):", global_optimal_path)
    print("Global optimal path:", global_optimal_path)
    print("Normed optimal path:", normed_optimal_path)
    robot_base_optimal_path = transform_path_to_robot_base_coordinates(global_optimal_path)
    # print("Robot base optimal path (start_point_robot, end_point_robot):", robot_base_optimal_path)
    print("Robot base optimal path:", robot_base_optimal_path)

    # # Convert the robot base optimal path to CSV format
    # csv_path = convert_path_to_csv_format(robot_base_optimal_path)
    # print("CSV path:", csv_path)





