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
        
        # Visibility visualization
        self.visibility_cells_target = []  # Target visibility scope
        self.visibility_cells_highway = []  # Highway visibility scope
        self.selected_cell = None  # Currently selected cell for visibility
        
        # Affected cells visualization
        self.affected_cells = []  # Cells affected by last execution
        self.recalculation_cells = []  # Cells that need recalculation due to dependencies
        self.spillage_cells = []  # Cells created by spillage (shown in different color)
        
        # Trajectory preview
        self.preview_path = None
        self.preview_start_cell = None
        self.preview_path_type = None
        self.preview_objects = 0
        self.preview_distance = 0
        self.preview_spillage_cells = {}  # Spillage cells for path preview
        
        # Multi-agent visualization
        self.agents = []  # List of agent objects to visualize
        self.agent_paths = {}  # Planned paths for each agent {agent_id: [positions]}
        self.agent_colors = [
            (255, 0, 0),    # Red
            (0, 0, 255),    # Blue  
            (0, 255, 0),    # Green
            (255, 165, 0),  # Orange
            (128, 0, 128),  # Purple
            (255, 20, 147), # Deep Pink
            (0, 255, 255),  # Cyan
            (255, 255, 0),  # Yellow
        ]

    def update_env(self, env):
        """Update the underlying simulation environment without recreating the visualizer."""
        self.env = env
        self.grid_size = env.grid_size
        self.cell_size = self.screen_size / self.grid_size
        self.clear_trajectory_preview()
        self.clear_affected_cells()
        self.clear_recalculation_cells()
        self.clear_spillage_cells()
        self.clear_visibility_preview()

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
        for cell in self.env.get_all_cells_with_objects():
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            radius = min(self.cell_size / 3, cell.num_objects * (self.cell_size / 6))  # Adjust size based on count
            pygame.draw.circle(self.screen, (0, 255, 0), (center_x, center_y), radius)

    def draw_lines_to_target(self):
        """Draw lines from each cell with objects to the closest point on the target zone boundary."""
        for cell in self.env.get_all_cells_with_objects():
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
        """Draw the highway path vectors derived from actual paths for cells in the grid."""
        for cell in self.env.get_all_cells_with_objects():
            # ✅ Derive direction from best_path_highway instead of velocity_highway
            if hasattr(cell, 'best_path_highway') and cell.best_path_highway and len(cell.best_path_highway) > 1:
                # Get direction from current cell to next cell in path
                next_cell = cell.best_path_highway[1]
                dx = next_cell.x - cell.x
                dy = next_cell.y - cell.y
                magnitude = math.sqrt(dx**2 + dy**2) + 1e-5
                dx_norm, dy_norm = dx / magnitude, dy / magnitude
            elif hasattr(cell, "velocity_highway") and cell.velocity_highway is not None:
                # Fallback to cached velocity if no path available
                dx_norm, dy_norm = cell.velocity_highway
                magnitude = math.sqrt(dx_norm**2 + dy_norm**2)
            else:
                # No highway data available
                continue

            # Draw highway direction vector
            start_x = (cell.x + 0.5) * self.cell_size
            start_y = (cell.y + 0.5) * self.cell_size
            end_x = start_x + dx_norm * self.cell_size / 2  # Scale for visualization
            end_y = start_y + dy_norm * self.cell_size / 2

            # Adjust arrow color and size based on direction magnitude
            color_intensity = min(255, int(magnitude * 100))
            color = (255, 165, 0)  # Orange for the highway vectors
            pygame.draw.line(self.screen, color, (start_x, start_y), (end_x, end_y), 2)
            pygame.draw.circle(self.screen, color, (int(end_x), int(end_y)), 3)  # Arrowhead

    def draw_velocity_field(self):
        """Draw velocity vectors for all cells in the grid."""
        for x in range(self.env.grid_size):
            for y in range(self.env.grid_size):
                # Find or create a cell
                cell = next((c for c in self.env.get_all_cells_with_objects() if c.x == x and c.y == y), None)
                if not cell:
                    continue

                # Draw velocity vector - prefer actual path direction over velocity field
                start_x = (cell.x + 0.5) * self.cell_size
                start_y = (cell.y + 0.5) * self.cell_size
                
                # Priority 1: Use actual target path if available and has at least 2 cells
                if hasattr(cell, 'best_path_target') and cell.best_path_target and len(cell.best_path_target) >= 2:
                    next_cell = cell.best_path_target[1]  # First hop in path
                    dx = next_cell.x - cell.x
                    dy = next_cell.y - cell.y
                    magnitude = math.sqrt(dx**2 + dy**2) + 1e-5
                    dx, dy = dx / magnitude, dy / magnitude  # Normalize
                else:
                    # Fallback: Use velocity field
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
            (cell for cell in self.env.get_all_cells_with_objects() if cell.x == grid_x and cell.y == grid_y), None
        )

        if not clicked_cell or clicked_cell.num_objects <= 0:
            print("Invalid click. No objects in the clicked cell.")
            return None

        # Return the clicked cell for main.py to handle
        return clicked_cell

    def set_trajectory_preview(self, start_cell, path_type, path_info):
        """
        Set trajectory preview for visualization.
        :param start_cell: The starting cell
        :param path_type: 'target' or 'highway'
        :param path_info: Dictionary with path information including 'path', 'objects', 'distance', 'impacted_cells'
        """
        self.preview_start_cell = start_cell
        self.preview_path_type = path_type
        self.preview_path = path_info['path'] if path_info else []
        self.preview_objects = path_info['objects'] if path_info else 0
        self.preview_distance = path_info['distance'] if path_info else 0
        # Add spillage cell tracking
        self.preview_spillage_cells = path_info.get('impacted_cells', {}) if path_info else {}

    def clear_trajectory_preview(self):
        """Clear trajectory preview."""
        self.preview_path = None
        self.preview_start_cell = None
        self.preview_path_type = None
        self.preview_objects = 0
        self.preview_distance = 0
        self.preview_spillage_cells = {}

    def set_visibility_preview(self, selected_cell, path_type, visible_cells):
        """
        Set visibility scope preview for debugging.
        :param selected_cell: The cell whose visibility is being shown
        :param path_type: 'target' or 'highway'
        :param visible_cells: List of cells in visibility scope
        """
        self.selected_cell = selected_cell
        if path_type == 'target':
            self.visibility_cells_target = [info["cell"] if isinstance(info, dict) else info for info in visible_cells]
            self.visibility_cells_highway = []  # Clear highway visibility
        elif path_type == 'highway':
            self.visibility_cells_highway = [info["cell"] if isinstance(info, dict) else info for info in visible_cells]
            self.visibility_cells_target = []  # Clear target visibility

    def clear_visibility_preview(self):
        """Clear visibility scope preview."""
        self.visibility_cells_target = []
        self.visibility_cells_highway = []
        self.selected_cell = None

    def set_affected_cells(self, affected_cells):
        """
        Set affected cells from last execution.
        :param affected_cells: List of cells affected by execution
        """
        self.affected_cells = affected_cells if affected_cells else []

    def clear_affected_cells(self):
        """Clear affected cells visualization."""
        self.affected_cells = []

    def set_recalculation_cells(self, recalculation_cells):
        """
        Set recalculation cells that need path recomputation due to dependencies.
        :param recalculation_cells: List of cells that need recalculation
        """
        self.recalculation_cells = recalculation_cells if recalculation_cells else []

    def clear_recalculation_cells(self):
        """Clear recalculation cells visualization."""
        self.recalculation_cells = []
    
    def set_spillage_cells(self, spillage_cells):
        """
        Set spillage cells for visualization with purple borders.
        :param spillage_cells: List of cells created by spillage
        """
        self.spillage_cells = spillage_cells if spillage_cells else []
    
    def clear_spillage_cells(self):
        """Clear spillage cells visualization."""
        self.spillage_cells = []

    def draw_trajectory_preview(self):
        """Draw the planned trajectory preview with enhanced visualization."""
        if not self.preview_path or len(self.preview_path) == 0:
            return

        # Highlight the starting cell with a bright border
        start_cell = self.preview_start_cell
        if start_cell:
            start_rect = pygame.Rect(
                start_cell.x * self.cell_size, start_cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (255, 255, 0), start_rect, 4)  # Yellow border for start

        # Draw path cells with colored borders
        path_color = (0, 255, 0) if self.preview_path_type == 'target' else (255, 165, 0)  # Green for target, orange for highway
        
        for i, cell in enumerate(self.preview_path):
            rect = pygame.Rect(
                cell.x * self.cell_size, cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            # Fade the border color along the path
            alpha = max(100, 255 - i * 20)  # Fade from 255 to 100
            border_width = max(2, 4 - i // 3)  # Reduce border width along path
            pygame.draw.rect(self.screen, path_color, rect, border_width)

        # Draw connecting lines between path cells
        if len(self.preview_path) > 1:
            for i in range(len(self.preview_path) - 1):
                start_cell = self.preview_path[i]
                end_cell = self.preview_path[i + 1]
                start_pos = (start_cell.x * self.cell_size + self.cell_size / 2,
                           start_cell.y * self.cell_size + self.cell_size / 2)
                end_pos = (end_cell.x * self.cell_size + self.cell_size / 2,
                         end_cell.y * self.cell_size + self.cell_size / 2)
                pygame.draw.line(self.screen, path_color, start_pos, end_pos, 3)

        # Draw final destination with special marker
        if self.preview_path:
            final_cell = self.preview_path[-1]
            center_x = final_cell.x * self.cell_size + self.cell_size / 2
            center_y = final_cell.y * self.cell_size + self.cell_size / 2
            
            if self.preview_path_type == 'target':
                # Draw target symbol (circle with cross)
                pygame.draw.circle(self.screen, (255, 0, 0), (int(center_x), int(center_y)), 8, 3)
                pygame.draw.line(self.screen, (255, 0, 0), (center_x-5, center_y), (center_x+5, center_y), 2)
                pygame.draw.line(self.screen, (255, 0, 0), (center_x, center_y-5), (center_x, center_y+5), 2)
            else:
                # Draw highway symbol (diamond)
                points = [(center_x, center_y-8), (center_x+8, center_y), (center_x, center_y+8), (center_x-8, center_y)]
                pygame.draw.polygon(self.screen, (255, 165, 0), points, 3)

        # Draw spillage cells if in spillage mode
        self.draw_preview_spillage_cells()
        
        # Draw trajectory info text
        # self.draw_trajectory_info()

    def draw_trajectory_info(self):
        """Draw trajectory information text overlay."""
        if not hasattr(self, 'font'):
            pygame.font.init()
            self.font = pygame.font.Font(None, 24)
        
        # Create info text
        info_lines = [
            f"Path Type: {'TARGET' if self.preview_path_type == 'target' else 'HIGHWAY'}",
            f"Path Length: {len(self.preview_path)} cells",
            f"Objects: {self.preview_objects}",
            f"Distance: {self.preview_distance:.1f}",
            "",
            "Press ENTER to execute or ESC to cancel"
        ]
        
        # Draw background for text
        text_height = len(info_lines) * 25
        text_rect = pygame.Rect(10, 10, 300, text_height + 10)
        pygame.draw.rect(self.screen, (0, 0, 0), text_rect)  # Black background
        pygame.draw.rect(self.screen, (255, 255, 255), text_rect, 2)  # White border
        
        # Draw text lines
        for i, line in enumerate(info_lines):
            if line:  # Skip empty lines
                color = (255, 255, 0) if "Press ENTER" in line else (255, 255, 255)
                text_surface = self.font.render(line, True, color)
                self.screen.blit(text_surface, (15, 15 + i * 25))

    def draw_preview_spillage_cells(self):
        """Draw spillage cells for path preview when in spillage mode."""
        if not self.preview_spillage_cells:
            return
            
        # Draw spillage cells with distinctive marking
        spillage_color = (255, 100, 255)  # Magenta for spillage cells
        
        for cell_key, spillage_info in self.preview_spillage_cells.items():
            if isinstance(cell_key, tuple) and len(cell_key) == 2:
                x, y = cell_key
                
                # Draw spillage cell with distinctive pattern
                rect = pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size)
                
                # Fill with semi-transparent color
                spillage_surface = pygame.Surface((self.cell_size, self.cell_size), pygame.SRCALPHA)
                spillage_surface.fill((255, 100, 255, 80))  # Semi-transparent magenta
                self.screen.blit(spillage_surface, (x * self.cell_size, y * self.cell_size))
                
                # Draw border
                pygame.draw.rect(self.screen, spillage_color, rect, 2)
                
                # Draw spillage amount as text if significant
                if isinstance(spillage_info, (int, float)) and spillage_info > 0:
                    if not hasattr(self, 'small_font'):
                        self.small_font = pygame.font.Font(None, 16)
                    
                    spillage_text = f"{spillage_info:.1f}"
                    text_surface = self.small_font.render(spillage_text, True, (255, 255, 255))
                    text_rect = text_surface.get_rect(center=(x * self.cell_size + self.cell_size/2, 
                                                            y * self.cell_size + self.cell_size/2))
                    self.screen.blit(text_surface, text_rect)

    def draw_visibility_scope(self):
        """Draw visibility scope for selected cell."""
        if not self.selected_cell:
            return
            
        # Draw selected cell with special highlight
        selected_rect = pygame.Rect(
            self.selected_cell.x * self.cell_size, 
            self.selected_cell.y * self.cell_size, 
            self.cell_size, self.cell_size
        )
        pygame.draw.rect(self.screen, (255, 255, 0), selected_rect, 5)  # Thick yellow border
        
        # Draw target visibility cells in light blue
        for cell in self.visibility_cells_target:
            vis_rect = pygame.Rect(
                cell.x * self.cell_size, 
                cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (173, 216, 230), vis_rect, 3)  # Light blue border
            # Add small "T" indicator
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            if not hasattr(self, 'small_font'):
                self.small_font = pygame.font.Font(None, 16)
            text = self.small_font.render("T", True, (0, 0, 255))
            self.screen.blit(text, (center_x - 5, center_y - 8))
            
        # Draw highway visibility cells in light orange
        for cell in self.visibility_cells_highway:
            vis_rect = pygame.Rect(
                cell.x * self.cell_size, 
                cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (255, 218, 185), vis_rect, 3)  # Light orange border
            # Add small "H" indicator
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            if not hasattr(self, 'small_font'):
                self.small_font = pygame.font.Font(None, 16)
            text = self.small_font.render("H", True, (255, 140, 0))
            self.screen.blit(text, (center_x - 5, center_y - 8))

    def draw_affected_cells(self):
        """Draw cells affected by last execution."""
        for cell in self.affected_cells:
            affected_rect = pygame.Rect(
                cell.x * self.cell_size, 
                cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (255, 0, 255), affected_rect, 2)  # Magenta border
            # Add small "A" indicator
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            if not hasattr(self, 'small_font'):
                self.small_font = pygame.font.Font(None, 16)
            text = self.small_font.render("A", True, (255, 0, 255))
            self.screen.blit(text, (center_x - 5, center_y + 5))

    def draw_recalculation_cells(self):
        """Draw cells that need recalculation due to dependencies."""
        for cell in self.recalculation_cells:
            recalc_rect = pygame.Rect(
                cell.x * self.cell_size, 
                cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (255, 255, 0), recalc_rect, 2)  # Light yellow border
            # Add small "R" indicator
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            if not hasattr(self, 'small_font'):
                self.small_font = pygame.font.Font(None, 16)
            text = self.small_font.render("R", True, (255, 140, 0))
            self.screen.blit(text, (center_x - 5, center_y - 2))
    
    def draw_spillage_cells(self):
        """Draw spillage cells with distinctive purple color and 'S' marker."""
        for cell in self.spillage_cells:
            spillage_rect = pygame.Rect(
                cell.x * self.cell_size, 
                cell.y * self.cell_size, 
                self.cell_size, self.cell_size
            )
            pygame.draw.rect(self.screen, (128, 0, 128), spillage_rect, 3)  # Purple border
            # Add small "S" indicator
            center_x = (cell.x + 0.5) * self.cell_size
            center_y = (cell.y + 0.5) * self.cell_size
            if not hasattr(self, 'small_font'):
                self.small_font = pygame.font.Font(None, 16)
            text = self.small_font.render("S", True, (128, 0, 128))
            self.screen.blit(text, (center_x - 5, center_y - 2))

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
        
        # Debug visualizations (always enabled for debugging)
        self.draw_affected_cells()  # Draw affected cells from last execution
        self.draw_recalculation_cells()  # Draw cells that need recalculation
        self.draw_spillage_cells()  # Draw spillage cells in purple
        self.draw_visibility_scope()  # Draw visibility scope for selected cell
        
        # Trajectory preview (enabled when showing planned path)
        if hasattr(self, 'preview_path') and self.preview_path:
            self.draw_trajectory_preview()
            
        # Optional visualizations (can be enabled for debugging)
        # self.draw_triangles()  # Draw the visibility triangles
        # self.draw_visible_cells()  # Highlight visible cells
        # self.draw_lines_to_target()  # Draw lines to the target zone
        self.draw_agents()  # Draw agents
        self.draw_agent_paths()  # Draw agent paths

    def draw_agents(self):
        """Draw all agents with their current positions and orientations."""
        if not self.agents:
            return
            
        for i, agent in enumerate(self.agents):
            # Get agent position (2D coordinates)
            x, y = agent.position
            orientation = 0  # Default orientation since agents don't have orientation in this version
            
            # Convert to screen coordinates
            screen_x = int(x * self.cell_size + self.cell_size / 2)
            screen_y = int(y * self.cell_size + self.cell_size / 2)
            
            # Use different colors for different agents
            color = self.agent_colors[i % len(self.agent_colors)]
            
            # Draw agent as a circle
            agent_radius = max(3, int(self.cell_size / 3))
            pygame.draw.circle(self.screen, color, (screen_x, screen_y), agent_radius)
            
            # Draw orientation arrow
            import math
            arrow_length = agent_radius * 2
            end_x = screen_x + int(arrow_length * math.cos(math.radians(orientation)))
            end_y = screen_y + int(arrow_length * math.sin(math.radians(orientation)))
            pygame.draw.line(self.screen, (0, 0, 0), (screen_x, screen_y), (end_x, end_y), 2)
            
            # Draw agent ID
            font = pygame.font.Font(None, 24)
            text = font.render(f"A{i}", True, (0, 0, 0))
            text_rect = text.get_rect(center=(screen_x, screen_y - agent_radius - 15))
            self.screen.blit(text, text_rect)

    def draw_agent_paths(self):
        """Draw planned paths for all agents."""
        if not self.agent_paths:
            return
            
        for agent_id, path_points in self.agent_paths.items():
            if not path_points or len(path_points) < 2:
                continue
                
            # Use agent color but make it semi-transparent for path
            color = self.agent_colors[agent_id % len(self.agent_colors)]
            
            # Draw path as connected line segments
            screen_points = []
            for point in path_points:
                screen_x = int(point[0] * self.cell_size + self.cell_size / 2)
                screen_y = int(point[1] * self.cell_size + self.cell_size / 2)
                screen_points.append((screen_x, screen_y))
            
            # Draw the path line
            if len(screen_points) >= 2:
                pygame.draw.lines(self.screen, color, False, screen_points, 2)
                
            # Draw small circles at each waypoint
            for point in screen_points[1:]:  # Skip first point (current position)
                pygame.draw.circle(self.screen, color, point, 3)

    def update_agents(self, agents):
        """Update the list of agents to visualize."""
        self.agents = agents

    def update_agent_paths(self, agent_paths_dict):
        """Update the paths for agents to visualize."""
        self.agent_paths = agent_paths_dict

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
