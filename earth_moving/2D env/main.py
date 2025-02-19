
from env import SimulationEnv
import pygame
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
        grid_size=25,  # Define the grid size
        target_zone_radius=3,  # Define the target zone radius
        agent_positions=None,  # No predefined agent positions
        num_random_objects=55,  # Number of random objects to spawn
        seed=31,  # Set a fixed random seed for testing
    )
    print("Environment initialized!")
    print(f"Number of cells with objects: {len(env.cells_with_objects)}")
    for cell in env.cells_with_objects:
        print(f"Cell at ({cell.x}, {cell.y}) has {cell.num_objects} objects.")

    # Precompute visibility for all cells in the grid (Target Zone Path-related only)
    print("Calculating visibility for all cells (Target Zone Path-related)...")
    for cell in env.cells_with_objects:
        # Calculate visibility toward the target zone
        closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
        target_cell_target = Cell(
            int(closest_point_target[0]), int(closest_point_target[1]), 0, env.target_zone, env.grid_size
        )
        visible_cells_target, distance_to_children_target = env.calculate_visibility_simple(
            cell, angle_tolerance=60, target_cell=target_cell_target
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
    # visualizer.run()

    # Visualization loop with interaction
    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))  # Clear screen
        visualizer.draw_elements()  # Draw all elements
        pygame.display.flip()
        visualizer.clock.tick(30)  # Limit FPS to 30

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quitting visualization...")
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("Escape key pressed. Exiting...")
                    running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle click interaction
                pos = pygame.mouse.get_pos()
                clicked_cell = visualizer.handle_click_event(pos)
                if clicked_cell:
                    print(
                        f"Clicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")
                    # Ask the user for path choice
                    choice = input("Choose path type ('target' or 'highway'): ").strip().lower()
                    if choice == "target":
                        env.execute_path(clicked_cell, "target")
                    elif choice == "highway":
                        env.execute_path(clicked_cell, "highway")
                    else:
                        print("Invalid choice. Try again.")
                    # Update the environment after execution
                    env.update_environment()
    pygame.quit()
    print("Simulation ended successfully.")

if __name__ == "__main__":
    main()
