from shapely.geometry import Point
import math


class Cell:
    def __init__(self, x, y, num_objects, target_zone, grid_size):
        self.x = x
        self.y = y
        self.num_objects = num_objects
        self.visible_cells = []  # Each item is a dictionary with child cell and related info
        self.distance_to_children = {}  # Distances to visible cells

        # Calculate distance to the closest point on the target zone boundary
        object_position = (self.x + 0.5, self.y + 0.5)
        closest_point = target_zone.exterior.interpolate(
            target_zone.exterior.project(Point(object_position))
        )
        self.distance_to_target = math.sqrt(
            (closest_point.x - object_position[0])**2 + (closest_point.y - object_position[1])**2
        )

        # Best path information
        self.best_child = None
        self.total_objects = self.num_objects
        self.total_distance = self.distance_to_target

    def __repr__(self):
        return (f"Cell({self.x}, {self.y}, num_objects={self.num_objects}, "
                f"distance_to_target={self.distance_to_target:.2f}, "
                f"total_objects={self.total_objects}, total_distance={self.total_distance:.2f})")
