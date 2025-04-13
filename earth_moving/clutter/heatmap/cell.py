from shapely.geometry import Point
import math


class Cell:
    def __init__(self, x, y, num_objects, target_zone, grid_size):
        self.x = x
        self.y = y
        self.num_objects = num_objects
        self.current_objects = num_objects  # Number of objects currently in the cell

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
        self.total_objects_target = self.num_objects  # Objects collected on the way to the target zone
        self.velocity_target = (0, 0)  # Velocity toward the target zone

        # Highway path-related attributes
        self.best_path_highway = []  # Best path to the highway
        self.total_objects_highway = 0  # Objects collected on the way to the highway
        self.velocity_highway = (0, 0)  # Velocity toward the highway

        # Calculate the closest point on the target zone boundary
        object_position = (self.x + 0.5, self.y + 0.5)
        closest_point = target_zone.exterior.interpolate(
            target_zone.exterior.project(Point(object_position))
        )
        self.closest_x = closest_point.x  # Store the x-coordinate of the closest boundary point
        self.closest_y = closest_point.y  # Store the y-coordinate of the closest boundary point
        self.closest_distance = math.sqrt(
            (closest_point.x - object_position[0]) ** 2 + (closest_point.y - object_position[1]) ** 2
        )

        self.distance_to_target = self.closest_distance  # Distance to the target zone
        self.best_child = None
        self.total_distance = float('inf')  # Default total distance
        self.potential = None  # Potential field value
        self.velocity = (0, 0)  # Default velocity

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
