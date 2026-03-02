from env import SimulationEnv
import pygame
from visualizer import SimulationVisualizer
import warnings
from cell import Cell  # Import the Cell class
from coordinate_converter import CoordinateConverter  # Import the coordinate converter
import json
import sys

warnings.filterwarnings("ignore", category=UserWarning, module="numpy")

def compute_2d_env(env_radius, target_zone_radius, shovel_width, real_objects, manual_mode=False, use_spillage_model=True, visualize_potential=True):
    """Compute the 2D environment properties without launching a PyGame window."""
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
    
    # Initialize environment with enhanced 2D algorithm parameters
    env = SimulationEnv(
        grid_size=env_info['grid_size'],
        target_zone_radius=env_info['target_zone_cells'],
        agent_positions=None,
        num_random_objects=0,  # We'll add real objects manually
        seed=None,  # Don't use random seed for real object placement
        max_path_length_factor=2.5,  # Enhanced path planning constraint
        target_angle_tolerance=45,   # Enhanced visibility angle for target paths
        highway_angle_tolerance=60,  # Enhanced visibility angle for highway paths
        highway_min_heat_ratio=0.3,      # Minimum heat quality threshold
        highway_threshold_ratio=0.5,     # Highway threshold configuration
        highway_heat_weight=0.7,          # Heat weight in hybrid scoring
        highway_distance_weight=0.3,      # Distance weight in hybrid scoring
    )
    
    # Manually add real objects to the environment (replacing random spawn)
    env._add_real_objects(objects_2d)
    
    print("\nEnvironment initialized!")
    print(f"Number of cells with objects: {len(env.cells_with_objects)}")

    # Initialize all cells and calculate their properties using enhanced algorithm
    print("\nInitializing cells and calculating properties...")
    for cell in env.cells_with_objects:
        print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")
    
    # Precompute visibility for all cells in the grid (Target Zone Path-related only)
    print("Calculating visibility for all cells (Target Zone Path-related)...")
    for cell in env.cells_with_objects:
        closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
        tx, ty = int(closest_point_target[0]), int(closest_point_target[1])
        target_cell_target = env.get_cell(tx, ty)
        visible_cells_target, distance_to_children_target = env.calculate_target_zone_visibility(
            cell, angle_tolerance=45)  # Use enhanced target angle tolerance
        cell.visible_cells_target = visible_cells_target
        cell.distance_to_children_target = distance_to_children_target
        
        # Compute optimistic heuristic (sum of objects in visibility scope)
        cell.h_vis_target = sum(n["cell"].num_objects for n in cell.visible_cells_target)
    print("Visibility for Target Zone Path-related attributes calculated.")
    
    # Audit visibility after target visibility computation
    env.audit_visibility()

    # Compute potential field using enhanced algorithm with configurable spillage
    print(f"\nCalculating potential field (Spillage: {'ON' if use_spillage_model else 'OFF'}, Visualization: {'ON' if visualize_potential else 'OFF'})...")
    env.calculate_potential_field(use_spillage_model=use_spillage_model, visualize=visualize_potential)
    print("Potential field calculated.")

    # Compute velocity field
    print("\nCalculating velocity field...")
    env.calculate_velocity_field()
    print("Velocity field calculated.")

    # Simulate flow and update the heat map
    print("\nSimulating flow and updating heat map...")
    env.update_heat_map()
    print("Heat map updated.")

    # Calculate paths to highways using enhanced algorithm with configurable spillage
    print("\nCalculating paths to highways for low-potential cells...")
    env.calculate_path_to_highway(use_spillage_model=use_spillage_model)
    print("Paths to highways calculated.")

    return env

def run_2d_env(env_radius, target_zone_radius, shovel_width, real_objects, manual_mode=False, use_spillage_model=True, visualize_potential=True):
    """Run 2D environment with real object positions and display the PyGame visualizer."""
    env = compute_2d_env(env_radius, target_zone_radius, shovel_width, real_objects, manual_mode, use_spillage_model, visualize_potential)
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
