from shapely.geometry import Point
import math


class Cell:
    def __init__(self, x, y, num_objects, target_zone, grid_size, target_metrics=None):
        self.x = x
        self.y = y
        self.num_objects = num_objects
        self.current_objects = num_objects  # Number of objects currently in the cell
        
        self._apply_target_metrics(target_zone, target_metrics)

        # Separate lists for visible cells
        self.visible_cells_target = []  # List of visible cells toward the target zone
        self.visible_cells_highway = []  # List of visible cells toward the highway

        # Distances to visible cells
        self.distance_to_children_target = {}  # Distances to visible cells for the target zone
        self.distance_to_children_highway = {}  # Distances to visible cells for the highway

        # Other attributes
        self.flow_density = 0  # Default flow density is 0
        self.heat_map = 0  # Initialize heat map value


        # Target zone path-related attributes
        self.best_path_target = []  # Best path to the target zone
        self.total_objects_target = self.num_objects  # Objects collected on the way to the target zone (spillage-affected)
        self.total_objects_raw = self.num_objects  # Raw objects collected (no spillage effects, for propagation)
        self.total_objects_path_target = 0  # Maximum objects collected for paths to the target zone
        self.velocity_target = (0, 0)  # Velocity toward the target zone
        self.total_distance_target = 0

        # Highway path-related attributes
        self.best_path_highway = []  # Best path to the highway
        self.total_objects_highway = 0  # Objects collected on the way to the highway
        self.total_objects_path_highway = 0  # Maximum objects collected for paths to the highway
        self.velocity_highway = (0, 0)  # Velocity toward the highway
        self.total_distance_highway = 0
        self.distance_to_highway = float("inf")  # ג… New attribute

        self.next_child_target = None
        self.next_child_highway = None
        self.total_distance = float('inf')  # Default total distance
        self.potential = None  # Potential field value
        self.velocity = (0, 0)  # Default velocity

        self.impacted_cells_target = {}  # Stores impacted cells from spillage (target)
        self.impacted_cells_highway = {}  # Stores impacted cells from spillage (highway)

        # Memoization / heuristics (target mode)
        self.solved_target = False          # True once best_path_target is finalized
        self.h_vis_target = 0               # optimistic: sum(objects) in visibility scope
        self.h_resolved_target = 0          # exact: objects delivered by best_path_target when solved

    def _apply_target_metrics(self, target_zone, target_metrics=None):
        """Install precomputed target geometry or use the legacy scalar fallback."""
        if target_metrics is None:
            object_position = (self.x + 0.5, self.y + 0.5)
            cell_point = Point(*object_position)
            self.is_target_zone = target_zone.contains(cell_point)
            closest_point = target_zone.exterior.interpolate(
                target_zone.exterior.project(cell_point)
            )
            closest_x, closest_y = float(closest_point.x), float(closest_point.y)
            closest_distance = math.hypot(
                closest_x - object_position[0], closest_y - object_position[1])
        else:
            is_target, closest_x, closest_y, closest_distance = target_metrics
            self.is_target_zone = bool(is_target)
            closest_x, closest_y = float(closest_x), float(closest_y)
            closest_distance = float(closest_distance)
        self.closest_x = closest_x
        self.closest_y = closest_y
        self.closest_distance = closest_distance
        self.distance_to_target = closest_distance

    def apply_target_metrics(self, target_metrics):
        self._apply_target_metrics(None, target_metrics)

    def calculate_potential(self):
        """
        Calculate potential for this cell based on distance to target and aggregate density.
        Lower potential indicates a more attractive cell for movement.
        """
        self.potential = self.distance_to_target / (self.num_objects + 1e-5)  # Avoid division by zero

    def update_flow_density(self, agent_speed):
        """
        Update flow density based on agent speed and velocity direction.
        """
        if hasattr(self, 'velocity') and self.velocity is not None:
            dx, dy = self.velocity
            self.flow_density += agent_speed * (abs(dx) + abs(dy))  # Update flow density

    def set_velocity_target(self, next_cell):
        """
        Set the velocity toward the target zone based on the next cell in the path.
        """
        if next_cell:
            dx = next_cell.x - self.x
            dy = next_cell.y - self.y
            magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
            self.velocity_target = (dx / magnitude, dy / magnitude)

    def set_velocity_highway(self, next_cell):
        """
        Set the velocity toward the highway based on the next cell in the path.
        """
        if next_cell:
            dx = next_cell.x - self.x
            dy = next_cell.y - self.y
            magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
            self.velocity_highway = (dx / magnitude, dy / magnitude)

    def __repr__(self):
        return (f"Cell({self.x}, {self.y}, num_objects={self.num_objects}, "
                f"distance_to_target={self.distance_to_target:.2f}, "
                f"total_objects_target={self.total_objects_target}, total_objects_highway={self.total_objects_highway}, "
                f"potential={self.potential}, flow_density={self.flow_density:.2f}, heat_map={self.heat_map:.2f})")


