
from env import SimulationEnv
from visualizer import SimulationVisualizer
import warnings
# from search import a_star_search
from cell import Cell  # Import the Cell class

warnings.filterwarnings("ignore", category=UserWarning, module="numpy")


# Suppress numpy warnings
warnings.filterwarnings("ignore", category=UserWarning, module="numpy")


def main():
    print("Initializing environment...")
    env = SimulationEnv(
        grid_size=15,  # Define the grid size
        target_zone_radius=3,  # Define the target zone radius
        agent_positions=None,  # No predefined agent positions
        num_random_objects=30,  # Number of random objects to spawn
        seed=42,  # Set a fixed random seed for testing
    )
    print("Environment initialized!")
    print(f"Number of cells with objects: {len(env.cells_with_objects)}")
    for cell in env.cells_with_objects:
        print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")

    # Precompute visibility for all cells in the grid (Target Zone Path-related only)
    print("Calculating visibility for all cells (Target Zone Path-related)...")
    for cell in env.all_cells:
        # Calculate visibility toward the target zone
        closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
        target_cell_target = Cell(
            int(closest_point_target[0]), int(closest_point_target[1]), 0, env.target_zone, env.grid_size
        )
        visible_cells_target, distance_to_children_target = env.calculate_visibility_simple(
            cell, angle_tolerance=40, target_cell=target_cell_target
        )
        cell.visible_cells_target = visible_cells_target
        cell.distance_to_children_target = distance_to_children_target
    print("Visibility for Target Zone Path-related attributes calculated.")

    # Proceed with potential field calculation
    print("Calculating potential field...")
    env.calculate_potential_field()
    print("Potential field calculated.")

    # Calculate velocity field
    print("Calculating velocity field...")
    env.calculate_velocity_field()
    print("Velocity field calculated.")

    # Simulate flow and update the heat map
    print("Simulating flow and updating heat map...")
    env.update_heat_map()
    print("Heat map updated.")

    # Calculate paths to highways
    print("Calculating paths to highways for low-potential cells...")
    env.calculate_path_to_highway()
    print("Paths to highways calculated.")

    # Initialize the visualization
    visualizer = SimulationVisualizer(env, screen_size=800)
    visualizer.run()
    env2=env
    print("end")


if __name__ == "__main__":
    main()
