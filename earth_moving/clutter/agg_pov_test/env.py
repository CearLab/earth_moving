from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math
import random
from cell import Cell  # Import the Cell class


class SimulationEnv:
    def __init__(self, grid_size, target_zone_radius=10, agent_positions=None, num_random_objects=0):
        self.grid_size = grid_size
        self.target_zone_radius = target_zone_radius
        self.num_agents = len(agent_positions) if agent_positions else 0
        self.num_objects = num_random_objects

        # Initialize attributes
        self.agents = []
        self.cells_with_objects = []  # List of Cell objects
        self.target_zone = None

        # Initialize environment elements
        self.target_zone = self._set_target_zone()
        self.agents = self._spawn_agents(agent_positions)
        self._spawn_objects_randomly()

    def _set_target_zone(self):
        """Set a circular target zone in the center of the grid."""
        center_x = self.grid_size // 2
        center_y = self.grid_size // 2
        center = Point(center_x, center_y)
        return center.buffer(self.target_zone_radius)  # Create a circular Polygon

    def _spawn_agents(self, agent_positions):
        """Spawn agents at specific positions or randomly."""
        agents = []
        if agent_positions:
            for i, (x, y, orientation) in enumerate(agent_positions):
                agents.append({"id": i, "position": (x, y), "orientation": orientation, "size": 3})
        else:
            for i in range(self.num_agents):
                x = random.uniform(0, self.grid_size)
                y = random.uniform(0, self.grid_size)
                orientation = random.uniform(0, 360)
                agents.append({"id": i, "position": (x, y), "orientation": orientation, "size": 3})
        return agents

    def _spawn_objects_randomly(self):
        """Randomly spawn objects that do not reside in the target zone."""
        random.seed(42)  # Use a fixed seed for deterministic results
        for _ in range(self.num_objects):
            while True:
                cell_x = random.randint(0, self.grid_size - 1)
                cell_y = random.randint(0, self.grid_size - 1)
                cell_center = Point(cell_x + 0.5, cell_y + 0.5)  # Center of the grid cell
                if not self.target_zone.contains(cell_center):  # Ensure the cell is not in the target zone
                    break

            # Check if a cell already exists at this location
            existing_cell = next((cell for cell in self.cells_with_objects if cell.x == cell_x and cell.y == cell_y), None)
            if existing_cell:
                # Update the existing cell's num_objects
                existing_cell.num_objects += 1
            else:
                # Add a new cell with one object
                self.cells_with_objects.append(Cell(cell_x, cell_y, 1, self.target_zone, self.grid_size))




    def calculate_visibility(self, cell, triangle):
        """
        Calculate which cells the given cell can see using the triangular polygon.
        :param cell: The Cell object to calculate visibility for.
        :param triangle: The Shapely Polygon representing the triangular area of visibility.
        """
        cell.visible_cells = []  # Reset visible cells
        cell.distance_to_children = {}  # Reset distances to children

        for other_cell in self.cells_with_objects:
            if cell == other_cell:
                continue  # Skip the current cell itself

            # Create a rectangle representing the boundary of the other cell
            cell_boundary = Polygon([
                (other_cell.x, other_cell.y),
                (other_cell.x + 1, other_cell.y),
                (other_cell.x + 1, other_cell.y + 1),
                (other_cell.x, other_cell.y + 1),
                (other_cell.x, other_cell.y),
            ])

            # Check if the cell boundary intersects the triangle
            if triangle.intersects(cell_boundary):
                # Compute distance to the child cell
                object_position = (cell.x + 0.5, cell.y + 0.5)
                child_position = (other_cell.x + 0.5, other_cell.y + 0.5)
                distance_to_child = math.sqrt(
                    (child_position[0] - object_position[0])**2 +
                    (child_position[1] - object_position[1])**2
                )

                # Add the child information to visible_cells and distances
                cell.visible_cells.append({
                    "cell": other_cell,
                })
                cell.distance_to_children[(other_cell.x, other_cell.y)] = distance_to_child





    def find_closest_point_on_target(self, object_position):
        """
        Find the closest point on the target zone boundary for a given object.
        :param object_position: (x, y) tuple representing the object's position.
        :return: (x, y) tuple of the closest point on the boundary.
        """
        object_point = Point(object_position)
        closest_point = nearest_points(object_point, self.target_zone.boundary)[1]
        return closest_point.x, closest_point.y


    def create_triangle_area(self, object_position, angle_left=45, angle_right=45):
        """
        Create a triangular area from the object to the target zone.
        :param object_position: (x, y) tuple of the object.
        :param angle_left: Angle in degrees to the left of the direction to the target zone.
        :param angle_right: Angle in degrees to the right of the direction to the target zone.
        :return: Shapely Polygon representing the triangle.
        """
        obj_x, obj_y = object_position
        object_point = Point(obj_x, obj_y)

        # Find the closest point on the target zone boundary
        closest_x, closest_y = self.find_closest_point_on_target(object_position)
        closest_point = Point(closest_x, closest_y)

        # Direction vector from object to the closest point on the target zone
        direction_vector = (closest_x - obj_x, closest_y - obj_y)
        direction_angle = math.atan2(direction_vector[1], direction_vector[0])

        # Perpendicular vector (90 degrees to the direction vector)
        perpendicular_angle = direction_angle + math.pi / 2  # 90 degrees in radians
        perpendicular_line = LineString([
            (closest_x - math.cos(perpendicular_angle) * self.grid_size,  # Extend in one direction
            closest_y - math.sin(perpendicular_angle) * self.grid_size),
            (closest_x + math.cos(perpendicular_angle) * self.grid_size,  # Extend in the other direction
            closest_y + math.sin(perpendicular_angle) * self.grid_size)
        ])

        # Left angled line from the object
        left_angle = direction_angle + math.radians(angle_left)
        left_line = LineString([
            (obj_x, obj_y),
            (obj_x + math.cos(left_angle) * self.grid_size,  # Extend outward
            obj_y + math.sin(left_angle) * self.grid_size)
        ])

        # Right angled line from the object
        right_angle = direction_angle - math.radians(angle_right)
        right_line = LineString([
            (obj_x, obj_y),
            (obj_x + math.cos(right_angle) * self.grid_size,  # Extend outward
            obj_y + math.sin(right_angle) * self.grid_size)
        ])

        # Intersect lines to find the vertices of the triangle
        left_intersection = left_line.intersection(perpendicular_line)
        right_intersection = right_line.intersection(perpendicular_line)

        if left_intersection.is_empty or right_intersection.is_empty:
            raise ValueError("Failed to create a valid triangle. Check the angles or object position.")

        # Create the triangle polygon
        triangle = Polygon([
            (obj_x, obj_y),  # Object's position
            (left_intersection.x, left_intersection.y),
            (right_intersection.x, right_intersection.y)
        ])
        return triangle

    
    
    def create_straight_triangle(self, object_position, angle_straight=30):
        """
        Create a single straight vision triangle for the given object position.
        :param object_position: (x, y) tuple of the object.
        :param angle_straight: Half-angle for the straight vision zone.
        :return: Shapely Polygon representing the straight triangle.
        """
        obj_x, obj_y = object_position

        # Find the closest point on the target zone boundary
        closest_x, closest_y = self.find_closest_point_on_target(object_position)
        direction_vector = (closest_x - obj_x, closest_y - obj_y)
        direction_angle = math.atan2(direction_vector[1], direction_vector[0])  # Angle to target zone center

        def compute_point(angle):
            """Helper to compute a point on the line extending from the object."""
            max_distance = self.grid_size  # Extend to the grid boundary
            end_x = obj_x + math.cos(angle) * max_distance
            end_y = obj_y + math.sin(angle) * max_distance
            line = LineString([(obj_x, obj_y), (end_x, end_y)])
            intersection = line.intersection(self.target_zone.boundary)
            if not intersection.is_empty:
                return (intersection.x, intersection.y)  # Point on the target zone boundary
            return (end_x, end_y)  # Point at the grid boundary

        # Compute the two boundary points for the triangle
        left_point = compute_point(direction_angle - math.radians(angle_straight))
        right_point = compute_point(direction_angle + math.radians(angle_straight))

        # Create the triangle polygon
        triangle = Polygon([(obj_x, obj_y), left_point, right_point, (closest_x, closest_y)])
        return triangle

    
    def create_vision_triangles(self, object_position, angle_straight=30, angle_peripheral=90):
        """
        Create three vision polygons (straight, right peripheral, left peripheral) for the given object position.
        :param object_position: (x, y) tuple of the object.
        :param angle_straight: Half-angle for the straight vision zone.
        :param angle_peripheral: Angle for the peripheral vision zones (relative to the straight direction).
        :return: A dictionary with three Shapely Polygons: {'straight', 'right', 'left'}.
        """
        obj_x, obj_y = object_position

        # Find the closest point on the target zone boundary
        closest_x, closest_y = self.find_closest_point_on_target(object_position)
        direction_vector = (closest_x - obj_x, closest_y - obj_y)
        direction_angle = math.atan2(direction_vector[1], direction_vector[0])  # Angle to target zone center

        def compute_point(angle, max_distance):
            """Helper to compute a point at a specific angle and distance."""
            end_x = obj_x + math.cos(angle) * max_distance
            end_y = obj_y + math.sin(angle) * max_distance
            line = LineString([(obj_x, obj_y), (end_x, end_y)])
            intersection = line.intersection(self.target_zone.boundary)
            if not intersection.is_empty:
                return (intersection.x, intersection.y)  # Point on the target zone boundary
            return (end_x, end_y)  # Point at the grid boundary

        def create_polygon(start_angle, end_angle, use_closest_point=False):
            """Helper to create a triangle polygon for a given angular range."""
            points = [(obj_x, obj_y)]  # Start at the object position
            start_point = compute_point(direction_angle + math.radians(start_angle), self.grid_size)
            end_point = compute_point(direction_angle + math.radians(end_angle), self.grid_size)
            points.append(start_point)
            points.append(end_point)

            if use_closest_point:
                # Use the closest point on the target zone boundary for the straight triangle
                points.append((closest_x, closest_y))

            return Polygon(points)

        # Create the three vision polygons
        straight_polygon = create_polygon(-angle_straight, angle_straight, use_closest_point=True)
        right_polygon = create_polygon(angle_straight, angle_peripheral)
        left_polygon = create_polygon(-angle_peripheral, -angle_straight)

        return {
            "straight": straight_polygon,
            "right": right_polygon,
            "left": left_polygon,
        }



