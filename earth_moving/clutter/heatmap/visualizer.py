import pygame
import sys
from shapely.geometry import Polygon
from shapely.geometry import Point
import math
from cell import Cell  # Import the Cell class



class SimulationVisualizer:
    def __init__(self, env, screen_size=500):
        pygame.init()
        self.env = env
        self.screen_size = screen_size
        self.grid_size = env.grid_size
        self.cell_size = screen_size / self.grid_size
        self.screen = pygame.display.set_mode((screen_size, screen_size))
        pygame.display.set_caption("Multi-Agent Simulation")
        self.clock = pygame.time.Clock()
        self.triangles = {}  # For visualization of triangles
        self.highlighted_cells = []  # Cells in the best path
        self.current_cell = None  # The cell whose visible cells are highlighted

    def draw_grid(self):
        """Draw the grid lines."""
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                rect = pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size)
                pygame.draw.rect(self.screen, (200, 200, 200), rect, 1)

    def draw_target_zone(self):
        """Draw the circular target zone."""
        center = self.env.target_zone.centroid
        radius = self.env.target_zone_radius * self.cell_size
        screen_center = (center.x * self.cell_size, center.y * self.cell_size)
        pygame.draw.circle(self.screen, (255, 0, 0), screen_center, radius, 2)  # Red hollow circle

    def draw_objects(self):
        """Draw objects as circles in their grid cells."""
        for cell in self.env.cells_with_objects:
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            radius = min(self.cell_size / 3, cell.num_objects * (self.cell_size / 6))  # Adjust size based on count
            pygame.draw.circle(self.screen, (0, 255, 0), (center_x, center_y), radius)

    def draw_lines_to_target(self):
        """Draw lines from each cell with objects to the closest point on the target zone boundary."""
        for cell in self.env.cells_with_objects:
            object_center = (cell.x + 0.5, cell.y + 0.5)  # Center of the cell
            closest_point = self.env.find_closest_point_on_target(object_center)
            pygame.draw.line(
                self.screen,
                (0, 0, 255),  # Blue line
                (object_center[0] * self.cell_size, object_center[1] * self.cell_size),
                (closest_point[0] * self.cell_size, closest_point[1] * self.cell_size),
                2,
            )

    def draw_heat_map(self):
        """
        Draw the heat map using a color gradient from blue (low heat) to red (high heat).
        All cells in the grid are visualized, and cells inside the target zone are skipped.
        """
        # Get all cells, excluding those inside the target zone
        cells_outside_target = [
            cell for cell in self.env.all_cells
            if not self.env.target_zone.contains(Point(cell.x + 0.5, cell.y + 0.5))
        ]

        if not cells_outside_target:
            print("No cells to display in the heat map.")
            return

        # Calculate the minimum and maximum heat map values
        min_heat = min(cell.heat_map for cell in cells_outside_target)
        max_heat = max(cell.heat_map for cell in cells_outside_target)

        for cell in self.env.all_cells:
            if self.env.target_zone.contains(Point(cell.x + 0.5, cell.y + 0.5)):
                # Skip cells inside the target zone
                continue

            # Normalize heat map value between 0 and 1
            if max_heat > min_heat:
                normalized_value = (cell.heat_map - min_heat) / (max_heat - min_heat)
            else:
                normalized_value = 0  # Avoid division by zero

            # Calculate color (blue to red gradient)
            red = int(255 * normalized_value)
            blue = int(255 * (1 - normalized_value))
            color = (red, 0, blue)

            # Draw the cell
            rect = pygame.Rect(
                cell.x * self.cell_size,
                cell.y * self.cell_size,
                self.cell_size,
                self.cell_size,
            )
            pygame.draw.rect(self.screen, color, rect)

    def draw_velocity_highway(self):
        """Draw the velocity highway vectors for cells in the grid with a distinct color."""
        for cell in self.env.cells_with_objects:
            if not hasattr(cell, "velocity_highway") or cell.velocity_highway is None:
                continue

            # Draw velocity_highway vector
            start_x = (cell.x + 0.5) * self.cell_size
            start_y = (cell.y + 0.5) * self.cell_size
            dx, dy = cell.velocity_highway
            end_x = start_x + dx * self.cell_size / 2  # Scale for visualization
            end_y = start_y + dy * self.cell_size / 2

            # Adjust arrow color and size based on velocity magnitude
            magnitude = math.sqrt(dx ** 2 + dy ** 2)
            color_intensity = min(255, int(magnitude * 100))
            color = (255, 165, 0)  # Orange for the highway vectors
            pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 2)
            pygame.draw.circle(self.screen, color, (int(end_x), int(end_y)), 3)  # Arrowhead

    def draw_velocity_field(self):
        """Draw velocity vectors for all cells in the grid."""
        for x in range(self.env.grid_size):
            for y in range(self.env.grid_size):
                # Find or create a cell
                cell = next((c for c in self.env.cells_with_objects if c.x == x and c.y == y), None)
                if not cell:
                    continue
                    cell = Cell(x, y, 0, self.env.target_zone, self.env.grid_size)
                    cell.velocity_target = self.env._calculate_velocity(cell)

                # Draw velocity vector
                start_x = (cell.x + 0.5) * self.cell_size
                start_y = (cell.y + 0.5) * self.cell_size
                dx, dy = cell.velocity_target
                end_x = start_x + dx * self.cell_size / 2  # Scale for visualization
                end_y = start_y + dy * self.cell_size / 2

                # Adjust arrow color and size based on velocity magnitude
                magnitude = math.sqrt(dx**2 + dy**2)
                color_intensity = min(255, int(magnitude * 100))
                color = (0, 255 - color_intensity, 0)  # Blue-green gradient
                pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 2)
                pygame.draw.circle(self.screen, color, (int(end_x), int(end_y)), 3)  # Arrowhead


    def draw_visible_cells(self):
        """Highlight all visible cells for the current cell."""
        if self.current_cell and self.current_cell.visible_cells:
            for visible in self.current_cell.visible_cells:
                visible_cell = visible["cell"]
                rect = pygame.Rect(
                    visible_cell.x * self.cell_size, visible_cell.y * self.cell_size,
                    self.cell_size, self.cell_size
                )
                pygame.draw.rect(self.screen, (0, 0, 255), rect, 2)  # Blue boundary for visible cells

    def draw_best_path(self):
        """Highlight the best path cells."""
        for (x, y) in self.highlighted_cells:  # Best path cells
            rect = pygame.Rect(
                x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (0, 255, 0), rect, 3)  # Green boundary for best path

    def draw_path_lines(self):
        """Draw lines connecting cells in the best path and the last cell to the target zone."""
        if len(self.highlighted_cells) > 1:  # Ensure there are at least two cells in the path
            for i in range(len(self.highlighted_cells) - 1):
                start_cell = self.highlighted_cells[i]
                end_cell = self.highlighted_cells[i + 1]
                start_pos = (start_cell[0] * self.cell_size + self.cell_size / 2,
                             start_cell[1] * self.cell_size + self.cell_size / 2)
                end_pos = (end_cell[0] * self.cell_size + self.cell_size / 2,
                           end_cell[1] * self.cell_size + self.cell_size / 2)
                pygame.draw.line(self.screen, (255, 0, 0), start_pos, end_pos, 3)  # Red line

            # Draw a line from the last cell to the target zone
            last_cell = self.highlighted_cells[-1]
            object_center = (last_cell[0] + 0.5, last_cell[1] + 0.5)  # Center of the last cell
            closest_point = self.env.find_closest_point_on_target(object_center)
            start_pos = (object_center[0] * self.cell_size, object_center[1] * self.cell_size)
            end_pos = (closest_point[0] * self.cell_size, closest_point[1] * self.cell_size)
            pygame.draw.line(self.screen, (255, 0, 0), start_pos, end_pos, 3)  # Red line to target

    def handle_click_event(self, pos):
        """
        Handle mouse click events.
        :param pos: Tuple (x, y) of the click position in pixels.
        """
        grid_x = int(pos[0] // self.cell_size)
        grid_y = int(pos[1] // self.cell_size)

        # Find the clicked cell
        clicked_cell = next(
            (cell for cell in self.env.cells_with_objects if cell.x == grid_x and cell.y == grid_y), None
        )

        if not clicked_cell or clicked_cell.num_objects <= 0:
            print("Invalid click. No objects in the clicked cell.")
            return

        # Mark the cell as selected and ask the user for the path type
        print(f"Clicked on cell ({grid_x}, {grid_y}) with {clicked_cell.num_objects} objects.")
        path_type = input("Enter 'target' for path to the target zone or 'highway' for path to the highway: ").strip()

        if path_type in ["target", "highway"]:
            # Execute the path via the environment
            self.env.execute_path(clicked_cell, path_type)

            # Update the environment
            print("Updating the environment...")
            self.env.update_environment()
            print("Environment updated.")
        else:
            print("Invalid path type selected.")

    def run_once(self):
        """
        Draw the current state of the environment once and update the display.
        """
        self.screen.fill((255, 255, 255))  # Clear the screen with white
        self.draw_elements()  # Draw all elements on the screen
        pygame.display.flip()  # Update the display

    def draw_elements(self):
        """Draw all elements on the screen."""
        self.screen.fill((255, 255, 255))  # Clear screen with white
        self.draw_grid()  # Draw the grid
        self.draw_target_zone()  # Draw the target zone
        self.draw_heat_map()  # Visualize heat map
        self.draw_velocity_field()  # Visualize velocity field
        self.draw_velocity_highway()  # Visualize velocity highway
        self.draw_objects()  # Draw the objects
        # self.draw_triangles()  # Draw the visibility triangles
        # self.draw_visible_cells()  # Highlight visible cells
        # self.draw_best_path()  # Highlight best path cells
        # self.draw_path_lines()  # Draw the lines connecting best path cells
        # self.draw_lines_to_target()  # Draw lines to the target zone


    def run(self):
        """Main loop for running the Pygame visualization."""
        print("Pygame visualization started...")
        running = True
        while running:
            self.screen.fill((255, 255, 255))  # Clear screen
            self.draw_elements()  # Draw all elements
            pygame.display.flip()
            self.clock.tick(30)  # Limit FPS to 30

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("Quitting Pygame...")
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:  # Allow escape key to quit
                        print("Escape key pressed. Exiting...")
                        running = False

        pygame.quit()
        print("Pygame terminated successfully.")
        sys.exit()
