import pygame
import sys
from shapely.geometry import Polygon


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

    def draw_triangles(self):
        """Draw the visibility triangles."""
        if self.triangles:
            colors = {"straight": (255, 165, 0)}  # Orange for the straight triangle
            for zone, triangle in self.triangles.items():
                scaled_triangle = [
                    (x * self.cell_size, y * self.cell_size) for x, y in triangle.exterior.coords[:-1]
                ]
                pygame.draw.polygon(self.screen, colors[zone], scaled_triangle, 2)  # Hollow polygon for the triangle

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
                # Get start and end cells
                start_cell = self.highlighted_cells[i]
                end_cell = self.highlighted_cells[i + 1]

                # Calculate pixel coordinates for the centers of the cells
                start_pos = (start_cell[0] * self.cell_size + self.cell_size / 2,
                            start_cell[1] * self.cell_size + self.cell_size / 2)
                end_pos = (end_cell[0] * self.cell_size + self.cell_size / 2,
                        end_cell[1] * self.cell_size + self.cell_size / 2)

                # Draw a red line between the two cell centers
                pygame.draw.line(self.screen, (255, 0, 0), start_pos, end_pos, 3)  # Red line

            # Draw a line from the last cell to the target zone
            last_cell = self.highlighted_cells[-1]
            object_center = (last_cell[0] + 0.5, last_cell[1] + 0.5)  # Center of the last cell
            closest_point = self.env.find_closest_point_on_target(object_center)
            start_pos = (object_center[0] * self.cell_size, object_center[1] * self.cell_size)
            end_pos = (closest_point[0] * self.cell_size, closest_point[1] * self.cell_size)
            pygame.draw.line(self.screen, (255, 0, 0), start_pos, end_pos, 3)  # Red line to target


    def draw_elements(self):
        """Draw all elements on the screen."""
        self.screen.fill((255, 255, 255))  # Clear screen with white
        self.draw_grid()  # Draw the grid
        self.draw_target_zone()  # Draw the target zone
        self.draw_objects()  # Draw the objects
        self.draw_triangles()  # Draw the visibility triangles
        # self.draw_visible_cells()  # Highlight visible cells
        self.draw_best_path()  # Highlight best path cells
        self.draw_path_lines()  # Draw the lines connecting best path cells
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
