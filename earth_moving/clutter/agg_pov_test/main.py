from env import SimulationEnv
from visualizer import SimulationVisualizer
from search import a_star_search


def main():
    print("Initializing environment...")
    env = SimulationEnv(
        grid_size=30,
        target_zone_radius=4,
        agent_positions=None,
        num_random_objects=40,
    )
    print("Environment initialized!")

    # Precompute visibility for all cells
    print("Calculating visibility for all cells...")
    for cell in env.cells_with_objects:
        object_position = (cell.x + 0.5, cell.y + 0.5)
        straight_triangle = env.create_triangle_area(object_position, angle_left=30, angle_right=30)
        env.calculate_visibility(cell, straight_triangle)
    print("Visibility calculated for all cells.")

    # Choose a specific cell with objects to analyze
    start_cell = env.cells_with_objects[9]  # Example: Use the 11th cell for testing
    print(f"Starting A* search from cell at ({start_cell.x}, {start_cell.y})...")

    # Perform A* search
    best_path, max_objects = a_star_search(start_cell, env.target_zone)

    # Print the best path and backpropagated values
    print("Best path:")
    for cell in best_path:
        print(cell)

    # Visualize the environment
    visualizer = SimulationVisualizer(env, screen_size=800)

    # Set the reference cell for visible cells
    visualizer.current_cell = start_cell

    # Set the triangle for visualization
    object_position = (start_cell.x + 0.5, start_cell.y + 0.5)
    straight_triangle = env.create_triangle_area(object_position, angle_left=30, angle_right=30)
    visualizer.triangles = {"straight": straight_triangle}  # Pass the triangle for visualization

    # Highlight visible cells (blue) and best path cells (green)
    visualizer.highlighted_cells = [(cell.x, cell.y) for cell in best_path]

    print("Starting visualization...")
    visualizer.run()


if __name__ == "__main__":
    main()
