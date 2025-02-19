from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math
import random
from cell import Cell  # Import the Cell class
from search import a_star_search_target, a_star_search_highway  # Import A* search function



class SimulationEnv:
    def __init__(self, grid_size, target_zone_radius=10, agent_positions=None, num_random_objects=0, seed=None):
        self.grid_size = grid_size
        self.target_zone_radius = target_zone_radius
        self.num_agents = len(agent_positions) if agent_positions else 0
        self.num_objects = num_random_objects
        self.highway_threshold =0

        # Initialize attributes
        self.agents = []
        self.cells_with_objects = []  # List of Cell objects
        self.cells_without_objects = []  # List of cells without objects
        self.all_cells = []  # List of all cells (with and without objects)
        self.target_zone = None

        # Initialize environment elements
        self.target_zone = self._set_target_zone()
        self.agents = self._spawn_agents(agent_positions)
        self._spawn_objects_randomly(seed)  # Pass the seed here
        self._initialize_cells()  # Populate `cells_without_objects` and `all_cells`

    def _initialize_cells(self):
        """Initialize all cells and populate `cells_without_objects` and `all_cells`."""
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                # Check if the cell is already in `cells_with_objects`
                existing_cell = next((cell for cell in self.cells_with_objects if cell.x == x and cell.y == y), None)
                if existing_cell:
                    self.all_cells.append(existing_cell)  # Add to all_cells
                else:
                    # Create a cell without objects
                    empty_cell = Cell(x, y, 0, self.target_zone, self.grid_size)
                    self.cells_without_objects.append(empty_cell)
                    self.all_cells.append(empty_cell)  # Add to all_cells

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

    def _spawn_objects_randomly(self, seed=None):
        """Randomly spawn objects that do not reside in the target zone."""
        if seed is not None:
            random.seed(seed)  # Set the random seed for reproducibility

        total_objects_spawned = 0

        while total_objects_spawned < self.num_objects:
            cell_x = random.randint(0, self.grid_size - 1)
            cell_y = random.randint(0, self.grid_size - 1)
            cell_center = Point(cell_x + 0.5, cell_y + 0.5)  # Center of the grid cell

            if self.target_zone.contains(cell_center):
                continue  # Skip cells within the target zone

            # Check if a cell already exists at this location
            existing_cell = next(
                (cell for cell in self.cells_with_objects if cell.x == cell_x and cell.y == cell_y),
                None
            )
            if existing_cell:
                # Increment the object count for the cell
                existing_cell.num_objects += 1
                existing_cell.current_objects += 1
            else:
                # Add a new cell with one object
                self.cells_with_objects.append(Cell(cell_x, cell_y, 1, self.target_zone, self.grid_size))

            total_objects_spawned += 1  # Increment the total object count


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


    def calculate_potential_field(self):
        """
        Calculate potential field for all cells based on the best path found via A* search.
        This assumes that visibility for all cells has already been precomputed.
        """
        print("Calculating potential field for all cells...")
        for cell in self.cells_with_objects:
            # Perform A* search using precomputed visible cells
            best_path, max_objects = a_star_search_target(cell)

            # Update cell attributes
            cell.best_path_target = best_path  # Store the best path in the cell
            cell.total_objects_target = max_objects  # Store the max objects collected in the path
            #
        print("Potential field calculation complete.")

    def calculate_velocity_field(self, context="target"):
        """
        Calculate velocity field for all cells based on potential gradients.
        Cells with objects will calculate their velocity using _calculate_velocity.
        For cells without objects, find their closest target or neighbor.

        :param context: Specify whether the calculation is for "target" or "highway".
        """
        print(f"Calculating velocity field for {context}...")
        for cell in self.cells_with_objects:
            if cell.num_objects > 0:  # Cells with objects
                if context == "target":
                    cell.velocity_target = self._calculate_velocity(cell, context)
                elif context == "highway":
                    cell.velocity_highway = self._calculate_velocity(cell, context)
            else:
                if context == "target":
                    cell.velocity_target = (0, 0)  # Default velocity for empty cells
                elif context == "highway":
                    cell.velocity_highway = (0, 0)  # Default velocity for empty cells

        for cell in self.all_cells:
            if cell not in self.cells_with_objects:  # Cells without objects
                if context == "target":
                    cell.velocity_target = self._calculate_velocity(cell, context)
                elif context == "highway":
                    cell.velocity_highway = self._calculate_velocity(cell, context)

        print(f"Velocity field for {context} calculated.")

    def _calculate_velocity(self, cell, context="target"):
        """
        Calculate velocity for a single cell based on the order of the best path found via A* search.
        If no path is found, direct the cell toward the target zone boundary or highway.

        :param cell: The cell for which to calculate velocity.
        :param context: Specify whether the calculation is for "target" or "highway".
        """
        if context == "target":
            best_path = cell.best_path_target
            closest_x, closest_y = cell.closest_x, cell.closest_y
        elif context == "highway":
            best_path = cell.best_path_highway
            closest_x, closest_y = cell.highway_x, cell.highway_y  # Assuming highway closest coordinates are stored

        # If no best path exists, fallback to the boundary of the context
        if not best_path or len(best_path) < 2:
            dx = closest_x - (cell.x + 0.5)
            dy = closest_y - (cell.y + 0.5)
        else:
            # Direct toward the next cell in the best path
            next_cell = best_path[1]
            dx = next_cell.x + 0.5 - (cell.x + 0.5)
            dy = next_cell.y + 0.5 - (cell.y + 0.5)

        # Normalize the velocity vector
        magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
        return dx / magnitude, dy / magnitude

    def update_heat_map(self):
        """
        Update the heat map for all cells in the grid and dynamically set the highway threshold.
        The heat map value for each cell is determined by the neighbor with the maximum contribution
        based on the alignment of velocity vectors and direction from the target zone.
        """
        max_potential = 0  # Initialize maximum potential value

        for cell in self.all_cells:
            # Reset heat map value
            cell.heat_map = 0
            # if cell.x ==5 and cell.y==4:
            #     print("test")

            # Skip cells inside the target zone
            if self.target_zone.contains(Point(cell.x + 0.5, cell.y + 0.5)):
                continue

            # Inverse vector pointing from the target to the current cell
            from_target_x = (cell.x + 0.5) - cell.closest_x
            from_target_y = (cell.y + 0.5) - cell.closest_y
            magnitude_from_target = math.sqrt(from_target_x ** 2 + from_target_y ** 2)
            if magnitude_from_target == 0:  # Avoid division by zero
                continue
            unit_from_target = (from_target_x / magnitude_from_target, from_target_y / magnitude_from_target)

            # Reset max heat value for this specific cell
            max_heat_value = 0

            for candidate in self.cells_with_objects:
                if candidate == cell:
                    continue

                # if candidate.x == 4 and candidate.y == 4:
                #     print("test")
                # Vector from the candidate to the current cell
                to_current_x = (cell.x + 0.5) - (candidate.x + 0.5)
                to_current_y = (cell.y + 0.5) - (candidate.y + 0.5)
                magnitude_to_current = math.sqrt(to_current_x ** 2 + to_current_y ** 2)
                if magnitude_to_current == 0:  # Avoid division by zero
                    continue
                unit_to_current = (to_current_x / magnitude_to_current, to_current_y / magnitude_to_current)

                # Inverse vector of `unit_to_current`
                inverse_unit_to_current = (-unit_to_current[0], -unit_to_current[1])

                # Dot product between `unit_from_target` and the inverse of `unit_to_current`
                dot_from_target = max(0, unit_from_target[0] * inverse_unit_to_current[0] +
                                      unit_from_target[1] * inverse_unit_to_current[1])

                # Only consider candidate cells aligned with the target-to-current direction
                if dot_from_target > math.cos(math.radians(60)):  # Example: 45-degree tolerance

                    # Retrieve the distance to the candidate's first child in its path
                    if candidate.next_child_target:
                        first_child = candidate.next_child_target
                        distance_to_first_child = math.sqrt(
                            (first_child.x - candidate.x) ** 2 + (first_child.y - candidate.y) ** 2
                        )
                        # Skip candidate if the distance to its first child is shorter
                        if distance_to_first_child < 1*magnitude_to_current:
                            continue

                    # Dot product between candidate's velocity and the vector to the current cell
                    dot_velocity = max(0, candidate.velocity_target[0] * unit_to_current[0] +
                                       candidate.velocity_target[1] * unit_to_current[1])

                    # Compute the contribution for this candidate
                    heat_value = dot_velocity * candidate.total_objects_path_target

                    # Update the maximum heat value for this cell
                    max_heat_value = max(max_heat_value, heat_value)

            # Assign the maximum heat value to the cell
            cell.heat_map = max_heat_value

            # Update the maximum potential value across all cells
            max_potential = max(max_potential, max_heat_value)

        # Set the highway threshold as a percentage of the maximum potential
        self.highway_threshold = max_potential * 0.5  # Example: 50% of the max potential

    def calculate_path_to_highway(self):
        """
        For cells in low-potential areas, calculate the best path to a nearby high-potential cell (highway).
        """
        for cell in self.cells_with_objects:
            # Skip cells already in high-potential locations
            if cell.heat_map >= self.highway_threshold:  # Define a threshold for "high-potential"
                continue

            best_target = None
            best_score = 0

            # Loop through all cells in the environment to find the best target
            for target_cell in self.all_cells:
                if target_cell == cell:
                    continue

                # Calculate distance to the target
                distance_to_target = math.sqrt(
                    (target_cell.x - cell.x) ** 2 + (target_cell.y - cell.y) ** 2
                )
                if distance_to_target == 0:
                    continue

                # Calculate the score as heat_map divided by distance
                score = target_cell.heat_map / math.log(distance_to_target+1,2.7183)
                if score > best_score:
                    best_score = score
                    best_target = target_cell

            if not best_target:
                continue  # Skip if no valid target was found

            # Dynamically calculate visibility for highway-related pathfinding
            cell.visible_cells_highway, cell.distance_to_children_highway = self.calculate_visibility_simple(
                cell, angle_tolerance=60, target_cell=best_target)

            # Find the best path to the target cell using A*
            best_path, max_objects = a_star_search_highway(cell)

            # Assign the best path and highway-related attributes
            cell.best_path_highway = best_path
            cell.total_objects_highway = max_objects

            # Assign a velocity direction toward the first cell in the path
            if len(best_path) > 1:
                next_cell = best_path[1]
                dx = next_cell.x - cell.x
                dy = next_cell.y - cell.y
                magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
                cell.velocity_highway = (dx / magnitude, dy / magnitude)
            else:
                cell.velocity_highway = (0, 0)  # No valid movement


    def find_closest_point_on_target(self, object_position):
        """
        Find the closest point on the target zone boundary for a given object.
        :param object_position: (x, y) tuple representing the object's position.
        :return: (x, y) tuple of the closest point on the boundary.
        """
        object_point = Point(object_position)
        closest_point = nearest_points(object_point, self.target_zone.boundary)[1]
        return closest_point.x, closest_point.y

    def execute_path(self, start_cell, path_type):
        """
        Execute the path for a given start cell based on the specified path type.
        Transfer objects from the start cell to the target or highway and update the relevant attributes.

        :param start_cell: The cell where the path starts.
        :param path_type: The type of path ('target' or 'highway').
        """
        # Determine the best path and total objects based on the path type
        if path_type == "target":
            best_path = start_cell.best_path_target
            total_objects = start_cell.total_objects_target
        elif path_type == "highway":
            best_path = start_cell.best_path_highway
            total_objects = start_cell.total_objects_highway
        else:
            print(f"Invalid path type: {path_type}")
            return

        # Check if the best path exists
        if not best_path or len(best_path) < 2:
            print(f"No valid path found for cell ({start_cell.x}, {start_cell.y}) with path type '{path_type}'.")
            return

        # Transfer objects to the final cell in the path
        final_cell = best_path[-1]
        final_cell.num_objects += total_objects  # Add the objects to the final cell
        final_cell.current_objects += total_objects  # Update current objects in the final cell

        # If the final cell was previously empty, add it to cells_with_objects
        if final_cell not in self.cells_with_objects:
            self.cells_with_objects.append(final_cell)

        # Remove objects from all cells along the path, except the final cell
        for cell in best_path[:-1]:  # Exclude the final cell
            if cell in self.cells_with_objects:
                self.cells_with_objects.remove(cell)  # Remove the cell from the list if it becomes empty
            cell.num_objects = 0
            cell.current_objects = 0

        print(
            f"Executed path ({path_type}) for cell ({start_cell.x}, {start_cell.y}). Objects moved to cell ({final_cell.x}, {final_cell.y}).")

    def update_environment(self):
        """
        Recalculate visibility, potential fields, velocity fields, and heat maps for all cells.
        """
        print("Updating visibility for all cells...")
        for cell in self.cells_with_objects:
            # Calculate visibility toward the target zone
            closest_point_target = self.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
            target_cell_target = Cell(
                int(closest_point_target[0]), int(closest_point_target[1]), 0, self.target_zone, self.grid_size
            )
            visible_cells_target, distance_to_children_target = self.calculate_visibility_simple(
                cell, angle_tolerance=60, target_cell=target_cell_target
            )
            cell.visible_cells_target = visible_cells_target
            cell.distance_to_children_target = distance_to_children_target

        print("Recalculating potential field...")
        self.calculate_potential_field()

        print("Recalculating velocity field...")
        self.calculate_velocity_field()

        print("Recalculating heat map...")
        self.update_heat_map()

        print("Environment update complete.")
        # Calculate paths to highways
        print("Calculating paths to highways for low-potential cells...")
        self.calculate_path_to_highway()
        print("Paths to highways calculated.")

    def calculate_visibility_simple(self, current_cell, angle_tolerance=30, target_cell=None):
        """
        Calculate visibility for a cell, adding neighbors within a cone of vision toward the target zone or a specific target.
        Always include the closest boundary cell of the target zone or the specified target cell if no visible cells are found.

        :param current_cell: The cell whose visibility is being calculated.
        :param angle_tolerance: Angle in degrees defining the cone of vision.
        :param target_cell: Optional specific target cell to calculate visibility toward. Defaults to the target zone.
        :return: A tuple (visible_cells, distance_to_children) where:
                 - visible_cells is a list of visible cells within the cone of vision.
                 - distance_to_children is a dictionary with keys as (x, y) tuples and values as distances.
        """
        visible_cells = []
        distance_to_children = {}

        # Determine the target direction (either target_cell or closest point in the target zone)
        if target_cell:
            target_direction = (
                target_cell.x - current_cell.x,
                target_cell.y - current_cell.y,
            )
        else:
            target_direction = (
                current_cell.closest_x - (current_cell.x + 0.5),
                current_cell.closest_y - (current_cell.y + 0.5),
            )

        target_magnitude = math.sqrt(target_direction[0] ** 2 + target_direction[1] ** 2)
        if target_magnitude == 0:
            return visible_cells, distance_to_children  # Prevent division by zero if current cell is on the target zone boundary
        target_unit_vector = (target_direction[0] / target_magnitude, target_direction[1] / target_magnitude)

        # Compute cosine threshold for the angle tolerance
        cos_threshold = math.cos(math.radians(angle_tolerance))

        # Add visible cells that satisfy distance and angular criteria
        for neighbor in self.cells_with_objects:
            if neighbor == current_cell:
                continue

            # Compute the direction vector to the neighbor
            neighbor_direction = (
                neighbor.x - current_cell.x,
                neighbor.y - current_cell.y,
            )
            distance_to_neighbor = math.sqrt(neighbor_direction[0] ** 2 + neighbor_direction[1] ** 2)
            if distance_to_neighbor == 0:
                continue  # Skip if the neighbor is in the same position as the current cell
            neighbor_unit_vector = (
                neighbor_direction[0] / distance_to_neighbor,
                neighbor_direction[1] / distance_to_neighbor,
            )

            # Compute the dot product of the two unit vectors
            dot_product = target_unit_vector[0] * neighbor_unit_vector[0] + target_unit_vector[1] * \
                          neighbor_unit_vector[1]

            # Add the neighbor if it satisfies the criteria
            if (
                    neighbor.distance_to_target <= current_cell.distance_to_target
                    and distance_to_neighbor <= current_cell.closest_distance * 0.7
                    and dot_product >= cos_threshold  # Ensure angle is within tolerance
            ):
                visible_cells.append({"cell": neighbor})
                distance_to_children[(neighbor.x, neighbor.y)] = distance_to_neighbor

        # Always include a fallback if no visible cells are found
        if not visible_cells:
            if target_cell:  # Add the specified target cell for highway-related visibility
                visible_cells.append({"cell": target_cell})
                distance_to_children[(target_cell.x, target_cell.y)] = math.sqrt(
                    (target_cell.x - current_cell.x) ** 2 + (target_cell.y - current_cell.y) ** 2
                )
            else:  # Add the closest boundary cell for target zone visibility
                boundary_cell = Cell(
                    int(current_cell.closest_x), int(current_cell.closest_y), 0, self.target_zone, self.grid_size
                )
                visible_cells.append({"cell": boundary_cell})
                distance_to_children[(boundary_cell.x, boundary_cell.y)] = current_cell.closest_distance

        return visible_cells, distance_to_children

