from env import SimulationEnv
import pygame
from visualizer import SimulationVisualizer
import warnings
from cell import Cell  # Import the Cell class
from coordinate_converter import CoordinateConverter  # Import the coordinate converter
import json
import sys

warnings.filterwarnings("ignore", category=UserWarning, module="numpy")

def run_2d_env(env_radius, target_zone_radius, shovel_width, real_objects, manual_mode=False):
    """Run 2D environment with real object positions."""
    # Initialize coordinate converter
    coord_converter = CoordinateConverter(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width
    )
    
    # Convert 3D positions to 2D grid coordinates
    objects_2d = coord_converter.convert_objects_to_2d(real_objects)
    
    # Get environment info for initialization
    env_info = coord_converter.get_environment_info()
    
    print("\nEnvironment Configuration:")
    print(f"Grid size: {env_info['grid_size']}x{env_info['grid_size']}")
    print(f"Target zone radius: {env_info['target_zone_cells']} cells")
    print(f"Shovel coverage: {env_info['shovel_cells']:.1f} cells")
    print(f"Cell size: {env_info['cell_size']:.3f} meters")
    
    # Initialize environment with converted 2D coordinates
    env = SimulationEnv(
        grid_size=env_info['grid_size'],
        target_zone_radius=env_info['target_zone_cells'],
        real_objects=objects_2d,
        use_real_objects=True,
        real_world_size=2*env_radius,
        shovel_size=shovel_width,
        manual_mode=manual_mode
    )
    
    print("\nEnvironment initialized!")
    print(f"Number of cells with objects: {len(env.cells_with_objects)}")

    # Initialize all cells and calculate their properties
    print("\nInitializing cells and calculating properties...")
    for cell in env.cells_with_objects:
        print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")
        
        # Calculate closest point on target zone
        closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
        target_cell_target = Cell(
            int(closest_point_target[0]), int(closest_point_target[1]), 0, env.target_zone, env.grid_size
        )
        
        # Calculate visibility and distances
        visible_cells_target, distance_to_children_target = env.calculate_visibility_simple(
            cell, angle_tolerance=60, target_cell=target_cell_target
        )
        cell.visible_cells_target = visible_cells_target
        cell.distance_to_children_target = distance_to_children_target

    # Compute potential field including spillage effects
    print("\nCalculating potential field...")
    env.calculate_potential_field(use_spillage_model=False, visualize=False)
    print("Potential field calculated.")

    # Compute velocity field
    print("\nCalculating velocity field...")
    env.calculate_velocity_field()
    print("Velocity field calculated.")

    # Simulate flow and update the heat map
    print("\nSimulating flow and updating heat map...")
    env.update_heat_map()
    print("Heat map updated.")

    # Calculate paths to highways
    print("\nCalculating paths to highways for low-potential cells...")
    env.calculate_path_to_highway()
    print("Paths to highways calculated.")

    # Initialize visualizer
    visualizer = SimulationVisualizer(env, screen_size=800)
    
    return env, visualizer

def main():
    # If state file is provided, load it; else run manual mode
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            state = json.load(f)
        run_2d_env(
            env_radius=state.get('env_radius'),
            target_zone_radius=state.get('target_zone_radius'),
            shovel_width=state.get('shovel_width'),
            real_objects=state.get('objects'),
            manual_mode=False
        )
    else:
        # For testing without 3D simulation
        test_objects = [(0.5, 0.5, 0.1), (-0.3, 0.2, 0.1), (0.1, -0.4, 0.1)]
        run_2d_env(
            env_radius=1.0,
            target_zone_radius=0.3,
            shovel_width=0.22,
            real_objects=test_objects,
            manual_mode=False
        )

if __name__ == "__main__":
    main()
