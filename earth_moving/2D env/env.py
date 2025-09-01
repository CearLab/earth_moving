from shapely.geometry import Point, Polygon, LineString
from shapely.ops import nearest_points
import math
import random
import pickle
import copy
from cell import Cell  # Import the Cell class
from search import a_star_search_target, a_star_search_highway  # Import A* search function
from spillage_model import simulate_spillage


class SimulationEnv:
    def __init__(self, grid_size, target_zone_radius=10, agent_positions=None, num_random_objects=0, seed=None, max_path_length_factor=2.5, target_angle_tolerance=30, highway_angle_tolerance=60, highway_min_heat_ratio=0.2, highway_threshold_ratio=0.5, highway_heat_weight=0.7, highway_distance_weight=0.3):
        self.grid_size = grid_size
        self.target_zone_radius = target_zone_radius
        self.num_agents = len(agent_positions) if agent_positions else 0
        self.num_objects = num_random_objects
        self.highway_threshold = 0
        self.max_path_length_factor = max_path_length_factor  # Path length constraint hyperparameter
        
        # ✅ HIGHWAY TARGET SELECTION CONFIGURATION
        self.highway_min_heat_ratio = highway_min_heat_ratio          # Minimum heat as % of max heat
        self.highway_threshold_ratio = highway_threshold_ratio        # Highway threshold as % of max potential
        self.highway_heat_weight = highway_heat_weight                # Weight for heat map score in hybrid scoring
        self.highway_distance_weight = highway_distance_weight        # Weight for distance score in hybrid scoring
        
        # ✅ CONSISTENT ANGLE TOLERANCE CONFIGURATION
        self.target_angle_tolerance = target_angle_tolerance    # Fixed angle for all target visibility
        self.highway_angle_tolerance = highway_angle_tolerance  # Fixed angle for all highway visibility
        
        # Dynamic angle settings (legacy - kept for compatibility)
        self.angle_min_deg = 30      # tight when far (per GPT recommendation)
        self.angle_max_deg = 60      # wider when near (per GPT recommendation)
        self.target_zone_gate_factor = 1.0  # Gate factor for target zone visibility (1.0 = no restriction)
        self.grid_diagonal = math.hypot(self.grid_size, self.grid_size)

        # Initialize attributes
        self.agents = []
        self.cells_with_objects = []  # List of Cell objects (excluding target zone)
        self.target_zone_cells = []  # Separate list for target zone cells with objects (for visualization)
        self.cells_without_objects = []  # List of cells without objects
        self.all_cells = []  # List of all cells (with and without objects)
        self.target_zone = None

        # Initialize environment elements
        self.target_zone = self._set_target_zone()
        self.agents = self._spawn_agents(agent_positions)
        self._spawn_objects_randomly(seed)  # Pass the seed here
        self._initialize_cells()  # Populate `cells_without_objects` and `all_cells`
        self._setup_adjacent_neighbors()  # Setup fast neighbor lookup

        self.agent_capacity = 8
        self.spillage_factor = 0.05
        self.min_spillage_threshold = 0.03
        self.use_spillage_model = False  # Initialize spillage flag (will be set by calculate_potential_field)

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

        # Create O(1) lookup mapping for canonical cell instances
        self.cells_by_xy = {(cell.x, cell.y): cell for cell in self.all_cells}

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

    def _setup_adjacent_neighbors(self):
        """Setup fast lookup for adjacent neighbors (8-connected) for each cell."""
        # Create lookup dictionary for fast cell access by coordinates
        self.cells_by_xy = {(c.x, c.y): c for c in self.all_cells}
        
        # Compute adjacent neighbors for each cell (8-connected)
        for c in self.all_cells:
            adj = []
            for nx in (c.x - 1, c.x, c.x + 1):
                for ny in (c.y - 1, c.y, c.y + 1):
                    if nx == c.x and ny == c.y:
                        continue  # Skip self
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        adj.append(self.cells_by_xy[(nx, ny)])
            # Cache adjacent neighbors on the Cell object
            c.adjacent_neighbors = adj

    def get_cell(self, x, y):
        """
        Retrieve a cell at a specific (x, y) location from the grid.
        Returns the cell if found, otherwise None.
        """
        # O(1) lookup to the canonical instance
        return getattr(self, "cells_by_xy", {}).get((x, y))

    def calculate_potential_field(self, use_spillage_model=True, visualize=True, affected_cells=None):
        """
        Calculate potential field using A* search.
        Can update the entire environment or only a subset of affected cells.

        :param use_spillage_model: If True, selects paths based on estimated spillage.
        :param visualize: If True, generates a spillage visualization plot.
        :param affected_cells: If provided, only these cells will be updated; otherwise, update all.
        """
        # Store spillage flag for later use by heat map and other methods
        self.use_spillage_model = use_spillage_model
        
        print(f"Calculating potential field... (Spillage Model: {use_spillage_model})")

        # Track if this is initial calculation before affected_cells gets reassigned
        is_initial_calculation = (affected_cells is None)
        
        # If no specific affected cells, use all object-containing cells
        if affected_cells is None:
            affected_cells = self.cells_with_objects

        # Sort cells by distance to target (closest first)
        sorted_cells = sorted(affected_cells, key=lambda c: c.distance_to_target)
        
        # ✅ Clear flow tracking values before A* runs (will be updated during search)
        print("Clearing flow tracking values for affected cells...")
        for cell in sorted_cells:
            cell.total_objects_path_target = 0

        for cell in sorted_cells:
            # Guard: Skip cells with no visibility to prevent A* hanging
            if not hasattr(cell, 'visible_cells_target') or not cell.visible_cells_target:
                print(f"Warning: Skipping cell ({cell.x}, {cell.y}) - no visibility for target zone")
                continue
                
            best_paths = a_star_search_target(cell, target_zone=self.target_zone, max_path_length_factor=self.max_path_length_factor, env=self)
            if not best_paths:
                print(f"Warning: No valid paths found for cell ({cell.x}, {cell.y})")
                continue

            # Select best path based on chosen method
            best_path = None
            max_objects = 0                  # spillage-affected objects (for target estimation)
            max_raw_objects = 0             # raw objects (for propagation density)
            best_distance = float("inf")
            best_impacted_cells = {}  # Store impacted cells

            if use_spillage_model:
                # For spillage mode: need separate selection for target vs propagation
                best_target_path = None
                best_raw_path = None
                max_estimated = 0
                max_raw = 0
                best_target_distance = float("inf")
                best_raw_distance = float("inf")
                
                for path_info in best_paths:
                    path = path_info["path"]
                    total_objects = path_info["objects"]  # Raw objects from path
                    total_distance = path_info["distance"]

                    # Estimate objects reaching the target
                    _, impacted_cells, estimated_objects, _ = simulate_spillage(
                        waypoints=[(c.x, c.y) for c in path],
                        objects_at_cells={(c.x, c.y): c.num_objects for c in path if c.num_objects > 0},
                        agent_capacity=self.agent_capacity,
                        spillage_factor=self.spillage_factor,
                        min_spillage_threshold=self.min_spillage_threshold
                    )

                    # Select best path for TARGET ESTIMATION (based on estimated_objects)
                    if estimated_objects > max_estimated or (
                            estimated_objects == max_estimated and total_distance < best_target_distance
                    ):
                        max_estimated = estimated_objects
                        # max_raw = estimated_objects
                        best_target_distance = total_distance
                        best_target_path = path
                        best_impacted_cells = impacted_cells

                    # Select best path for PROPAGATION (based on raw total_objects)
                    if total_objects > max_raw or (
                            total_objects == max_raw and total_distance < best_raw_distance
                    ):
                        max_raw = total_objects
                        best_raw_distance = total_distance
                        best_raw_path = path

                # Use target path for main path (this determines cell behavior)
                best_path = best_target_path
                max_objects = max_estimated
                max_raw_objects = max_raw
                best_distance = best_target_distance
                
            else:
                # For non-spillage mode: same path for both purposes
                for path_info in best_paths:
                    path = path_info["path"]
                    total_objects = path_info["objects"]  # Raw objects from path
                    total_distance = path_info["distance"]
                    estimated_objects = total_objects  # Same as raw
                    impacted_cells = {}  # No spillage impact

                    # Select the best path (same for both target and propagation)
                    if estimated_objects > max_objects or (
                            estimated_objects == max_objects and total_distance < best_distance
                    ):
                        max_objects = estimated_objects        # spillage-affected (for target)
                        max_raw_objects = total_objects        # raw objects (for propagation)
                        best_distance = total_distance
                        best_path = path
                        best_impacted_cells = impacted_cells

            # ✅ Store path and estimated results
            cell.best_path_target = best_path
            cell.total_objects_target = max_objects            # spillage-affected (for target estimation)
            cell.total_objects_raw = max_raw_objects           # raw objects (for propagation)
            cell.total_distance_target = best_distance
            cell.impacted_cells_target = best_impacted_cells  # ✅ Store impacted cells!
            
            # ✅ Mark cell as solved and compute exact heuristic (spillage OFF only)
            if not use_spillage_model:
                cell.solved_target = True
                cell.h_resolved_target = max_objects

        print("Potential field calculation complete.")

        # ✅ Run global propagation for initial calculations (needed for heat map)
        # Individual paths are propagated immediately when found, but we also need
        # global propagation for initial setup
        if is_initial_calculation:
            self.propagate_total_objects_path_target()

        # If visualization is enabled and spillage is ON, keep the viz call as-is
        if use_spillage_model and visualize:
            self.visualize_spillage_for_all_paths()

    def propagate_total_objects_path_target(self):
        """
        Propagate `total_objects_path_target` through all computed paths.
        This ensures that each cell knows the **maximum total objects** that can be pushed
        from the root of the path down to the target zone.
        """
        print("Propagating total_objects_path_target...")

        for cell in self.cells_with_objects:
            if not cell.best_path_target:
                continue  # Skip cells without a best path

            best_path = cell.best_path_target
            max_objects = cell.total_objects_raw  # Use raw objects for density propagation

            for i in range(len(best_path) - 1, -1, -1):
                current_cell = best_path[i]

                if i == len(best_path) - 1:  # Last cell in the path
                    current_cell.next_child_target = None
                    current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)
                else:
                    next_cell = best_path[i + 1]
                    current_cell.next_child_target = next_cell
                    current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)

        print("Propagation complete.")

    def propagate_total_objects_path_highway(self):
        """
        Propagate `total_objects_path_highway` through all computed paths.
        Ensures each cell knows the **maximum total objects** that can be pushed toward the highway.
        """
        print("Propagating total_objects_path_highway...")

        for cell in self.cells_with_objects:
            if not cell.best_path_highway:
                continue  # Skip cells without a best path

            best_path = cell.best_path_highway
            max_objects = cell.total_objects_highway  # Start with max collected objects

            for i in range(len(best_path) - 1, -1, -1):
                current_cell = best_path[i]

                if i == len(best_path) - 1:  # Last cell in the path
                    current_cell.next_child_highway = None
                    current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)
                else:
                    next_cell = best_path[i + 1]
                    current_cell.next_child_highway = next_cell
                    current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)

        print("Highway path propagation complete.")

    def propagate_total_objects_path_target_selective(self, affected_cells):
        """
        Propagate `total_objects_path_target` through computed paths for affected cells only.
        This ensures that each affected cell knows the **maximum total objects** that can be pushed
        from the root of the path down to the target zone.
        """
        print(f"Propagating target paths for {len(affected_cells)} affected cells...")

        for cell in affected_cells:
            if not cell.best_path_target or cell.num_objects == 0:
                continue  # Skip cells without a best path or objects

            best_path = cell.best_path_target
            max_objects = cell.total_objects_raw  # Use raw objects for density propagation

            for i in range(len(best_path) - 1, -1, -1):
                current_cell = best_path[i]

                if i == len(best_path) - 1:  # Last cell in the path
                    current_cell.next_child_target = None
                    current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)
                else:
                    next_cell = best_path[i + 1]
                    current_cell.next_child_target = next_cell
                    current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)

        print("Target path propagation complete for affected cells.")

    def propagate_total_objects_path_highway_selective(self, affected_cells):
        """
        Propagate `total_objects_path_highway` through computed paths for affected cells only.
        Ensures each affected cell knows the **maximum total objects** that can be pushed toward the highway.
        """
        print(f"Propagating highway paths for {len(affected_cells)} affected cells...")

        for cell in affected_cells:
            if not cell.best_path_highway or cell.num_objects == 0:
                continue  # Skip cells without a best path or objects

            best_path = cell.best_path_highway
            max_objects = cell.total_objects_highway  # Start with max collected objects

            for i in range(len(best_path) - 1, -1, -1):
                current_cell = best_path[i]

                if i == len(best_path) - 1:  # Last cell in the path
                    current_cell.next_child_highway = None
                    current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)
                else:
                    next_cell = best_path[i + 1]
                    current_cell.next_child_highway = next_cell
                    current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)

        print("Highway path propagation complete for affected cells.")

    def visualize_spillage_for_all_paths(self):
        """
        Visualize spillage effects for all paths in the environment.
        """
        import matplotlib.pyplot as plt

        plt.figure(figsize=(8, 8))

        for cell in self.cells_with_objects:
            if not cell.best_path_target:
                continue

            # Get best path
            waypoints = [(c.x + 0.5, c.y + 0.5) for c in cell.best_path_target]

            # Extract object distribution from environment
            objects_at_cells = {(c.x, c.y): c.num_objects for c in cell.best_path_target if c.num_objects > 0}

            # Run spillage simulation
            spline_points, impacted_cells, _, _ = simulate_spillage(  # Expect 4 values
                waypoints, objects_at_cells, agent_capacity=self.agent_capacity,
                spillage_factor=self.spillage_factor, min_spillage_threshold=self.min_spillage_threshold
            )

            # Plot the spline path
            if spline_points:
                smooth_x, smooth_y = zip(*spline_points)
                plt.plot(smooth_x, smooth_y, color="blue", alpha=0.6)

            # Plot spillage locations
            if impacted_cells:
                plt.scatter(*zip(*impacted_cells.keys()), s=50, color="orange", marker='x')

        plt.xlabel("X-axis (Cells)")
        plt.ylabel("Y-axis (Cells)")
        plt.title("Spillage Visualization for All Paths")
        plt.legend(["Smoothed Paths", "Spillage Locations"])
        plt.grid()
        
        # Set axis limits to match grid size and maintain proper orientation
        plt.xlim(0, self.grid_size)
        plt.ylim(0, self.grid_size)
        # Invert Y-axis to match grid visualization (top-to-bottom)
        plt.gca().invert_yaxis()
        
        plt.show()

    def calculate_velocity_field(self, context="target", affected_cells=None):
        """
        Calculate velocity field for all cells based on potential gradients.
        Can update all cells or only a subset.

        :param context: Specify whether the calculation is for "target" or "highway".
        :param affected_cells: If provided, only these cells will be updated; otherwise, update all.
        """
        print(f"Calculating velocity field for {context}...")

        # If no specific affected cells, use all object-containing cells
        if affected_cells is None:
            affected_cells = self.cells_with_objects

        for cell in affected_cells:
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

        print(f"Velocity field for {context} calculated.")

    def _calculate_velocity(self, cell, context="target"):
        """
        Calculate velocity for a single cell based on the order of the best path found via A* search.
        If no path is found, direct the cell toward the target zone boundary or highway.

        :param cell: The cell for which to calculate velocity.
        :param context: Specify whether the calculation is for "target" or "highway".
        """
        if context == "target":
            # Use pre-computed best path (already spillage-optimized if spillage model is enabled)
            best_path = cell.best_path_target
            closest_x, closest_y = cell.closest_x, cell.closest_y
        elif context == "highway":
            best_path = cell.best_path_highway
            # For highway, use target zone as reference direction (highways lead toward target eventually)
            closest_x, closest_y = cell.closest_x, cell.closest_y

        # If no best path exists, fallback to the boundary of the context
        if not best_path or len(best_path) < 2:
            dx = closest_x - (cell.x + 0.5)
            dy = closest_y - (cell.y + 0.5)
        else:
            # Point directly to the first next cell in the best path to target
            next_cell = best_path[1]  # The first next cell in the path
            dx = next_cell.x - cell.x
            dy = next_cell.y - cell.y

        # Normalize the velocity vector
        magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
        return dx / magnitude, dy / magnitude


    def get_path_for_preview(self, cell, path_type="target"):
        """
        Get the pre-computed path for UI preview.
        
        :param cell: The cell to get path for
        :param path_type: 'target' or 'highway'
        :return: List of path dictionaries for UI display
        """
        if path_type == "target":
            if not cell.best_path_target:
                return []
            
            # Return pre-computed path with proper formatting for UI
            path_info = {
                'path': cell.best_path_target,
                'objects': cell.total_objects_target,
                'distance': cell.total_distance_target,
                'impacted_cells': getattr(cell, 'impacted_cells_target', {})
            }
            return [path_info]
            
        elif path_type == "highway":
            if not cell.best_path_highway:
                return []
                
            # Return pre-computed highway path
            path_info = {
                'path': cell.best_path_highway,
                'objects': cell.total_objects_highway,
                'distance': cell.total_distance_highway,
                'impacted_cells': getattr(cell, 'impacted_cells_highway', {})
            }
            return [path_info]
        
        return []

    def update_heat_map(self):
        """
        Update the heat map for all cells in the grid and dynamically set the highway threshold.
        The heat map value for each cell is determined by the neighbor with the maximum contribution
        based on the alignment of velocity vectors and direction from the target zone.
        
        Performance optimized version that preserves exact behavior.
        """
        # OPTIMIZATION 1: Cache spillage mode flag and precompute constants
        use_spillage_model = getattr(self, "use_spillage_model", False)
        cos_threshold = math.cos(math.radians(75))
        
        # OPTIMIZATION 2: Filter and preprocess candidates once (preserve original logic exactly)
        candidates = []
        for candidate in self.cells_with_objects:
            # Skip candidate cells inside the target zone - preserve original Shapely check
            if self.target_zone.contains(Point(candidate.x + 0.5, candidate.y + 0.5)):
                continue
                
            # Guard velocity use - ensure candidate has non-tiny velocity
            vx, vy = getattr(candidate, "velocity_target", (0, 0))
            if abs(vx) < 1e-9 and abs(vy) < 1e-9:
                continue
                
            # PRESERVE ORIGINAL PSI CALCULATION LOGIC EXACTLY
            psi = candidate.total_objects_path_target
            # If cell has no path but has propagated value, it's likely stale
            if psi > 0 and not candidate.best_path_target:
                psi = candidate.num_objects  # Fallback to current objects
                
            if psi <= 0:
                continue
                
            candidates.append((candidate, vx, vy, psi))

        max_potential = 0

        for cell in self.all_cells:
            cell.heat_map = 0

            # PRESERVE ORIGINAL: Skip cells inside the target zone (keep Shapely check)
            if self.target_zone.contains(Point(cell.x + 0.5, cell.y + 0.5)):
                continue

            # PRESERVE ORIGINAL: Inverse vector pointing from the target to the current cell
            from_target_x = (cell.x + 0.5) - cell.closest_x
            from_target_y = (cell.y + 0.5) - cell.closest_y
            magnitude_from_target = math.sqrt(from_target_x ** 2 + from_target_y ** 2)
            if magnitude_from_target == 0:
                continue
            unit_from_target = (from_target_x / magnitude_from_target, from_target_y / magnitude_from_target)

            max_heat_value = 0

            # OPTIMIZATION 3: Use preprocessed candidates but preserve all original logic
            for candidate, vx, vy, psi in candidates:
                if candidate == cell:
                    continue

                # PRESERVE ORIGINAL: Vector calculations (exactly as before)
                to_current_x = (cell.x + 0.5) - (candidate.x + 0.5)
                to_current_y = (cell.y + 0.5) - (candidate.y + 0.5)
                magnitude_to_current = math.sqrt(to_current_x ** 2 + to_current_y ** 2)
                if magnitude_to_current == 0:
                    continue
                unit_to_current = (to_current_x / magnitude_to_current, to_current_y / magnitude_to_current)

                # PRESERVE ORIGINAL: Inverse vector calculation
                inverse_unit_to_current = (-unit_to_current[0], -unit_to_current[1])

                # PRESERVE ORIGINAL: Dot product alignment check
                dot_from_target = max(0, unit_from_target[0] * inverse_unit_to_current[0] +
                                      unit_from_target[1] * inverse_unit_to_current[1])

                # PRESERVE ORIGINAL: 75-degree tolerance check
                if dot_from_target > cos_threshold:
                    # PRESERVE ORIGINAL: Velocity dot product calculation
                    dot_velocity = max(0, vx * unit_to_current[0] + vy * unit_to_current[1])

                    # PRESERVE ORIGINAL: Path interference filter logic
                    first_child = candidate.next_child_target
                    if first_child:
                        distance_to_first_child = math.sqrt(
                            (first_child.x - candidate.x) ** 2 + (first_child.y - candidate.y) ** 2
                        )
                        # Skip if candidate's own path is shorter than flow to current cell
                        if distance_to_first_child < magnitude_to_current:
                            continue

                    # PRESERVE ORIGINAL: Heat calculation (same for both modes, no distance decay)
                    heat_value = dot_velocity * psi

                    # Update the maximum heat value for this cell
                    max_heat_value = max(max_heat_value, heat_value)

            cell.heat_map = max_heat_value
            max_potential = max(max_potential, max_heat_value)

        # Set the highway threshold as a percentage of the maximum potential
        self.highway_threshold = max_potential * self.highway_threshold_ratio

        # Diagnostic print to verify heat map is working
        print(f"[Heat] max_potential={max_potential:.3f}, threshold={self.highway_threshold:.3f}")

    def calculate_path_to_highway(self, use_spillage_model=False):
        """
        Calculate the best path to a high-potential highway cell for all object-containing cells.
        Uses simple distance constraint: target must be within distance_to_target of current cell.
        Supports two modes:
        1. **Without spillage model**: Maximizes collected objects.
        2. **With spillage model**: Accounts for estimated object spillage.

        :param use_spillage_model: If True, selects paths based on estimated spillage.
        """
        print(f"Calculating paths to highway... (Spillage Model: {use_spillage_model})")

        # ✅ PERFORMANCE OPTIMIZATION: Calculate candidate pool once outside the loop
        max_heat = max((c.heat_map for c in self.all_cells), default=0)
        min_required_heat = max_heat * self.highway_min_heat_ratio
        quality_candidates = [c for c in self.all_cells if c.heat_map >= min_required_heat]
        
        if not quality_candidates:
            print("WARNING: No cells meet heat quality threshold globally - no highway targets available")
            return

        for cell in self.cells_with_objects:
            # ✅ Skip cells already in high-potential locations
            if cell.heat_map >= self.highway_threshold:
                continue

            # ✅ SIMPLE DISTANCE CONSTRAINT: Find highest heat cell within distance_to_target
            distance_constrained_candidates = [
                c for c in quality_candidates 
                if math.hypot(c.x - cell.x, c.y - cell.y) <= cell.distance_to_target
            ]
            
            if not distance_constrained_candidates:
                print(f"WARNING: No highway targets within distance constraint for cell ({cell.x}, {cell.y}) (max_distance={cell.distance_to_target:.1f})")
                cell.best_path_highway = []
                cell.velocity_highway = (0, 0)
                continue
            
            # ✅ HYBRID SCORING: Combine heat map and distance with configurable weights
            def hybrid_score(candidate):
                # Normalize heat score (0-1, where 1 is max heat)
                heat_score = candidate.heat_map / max_heat if max_heat > 0 else 0
                
                # Normalize distance score (0-1, where 1 is closest distance)
                distance_to_candidate = math.hypot(candidate.x - cell.x, candidate.y - cell.y)
                max_allowed_distance = cell.distance_to_target
                distance_score = 1.0 - (distance_to_candidate / max_allowed_distance) if max_allowed_distance > 0 else 0
                
                # Weighted combination
                combined_score = (self.highway_heat_weight * heat_score) + (self.highway_distance_weight * distance_score)
                return combined_score
            
            # Select the candidate with the highest hybrid score
            target = max(distance_constrained_candidates, key=hybrid_score)
            
            if not target:
                print(f"WARNING: No valid highway target found for ({cell.x}, {cell.y})")
                cell.best_path_highway = []
                cell.velocity_highway = (0, 0)
                continue
            
            print(f"Cell ({cell.x}, {cell.y}) targeting highway cell ({target.x}, {target.y}) with heat={target.heat_map:.3f}")
            
            # ✅ Store the originally chosen target (before A* might change it)
            cell.chosen_highway_target = target
            
            # ✅ STEP 2: Calculate visibility toward the SPECIFIC chosen target
            cell.visible_cells_highway, cell.distance_to_children_highway = self.calculate_highway_visibility(
                cell, target, angle_tolerance=self.highway_angle_tolerance  # Use configured highway angle
            )

            # ✅ STEP 3: Find the best path to the SPECIFIC highway target
            # A* now handles direct paths when no visible cells available
            best_paths = a_star_search_highway(cell, target_cell=target, highway_threshold=self.highway_threshold)

            if not best_paths:
                print(f"ERROR: No valid highway paths found for ({cell.x}, {cell.y})")
                cell.best_path_highway = []
                cell.velocity_highway = (0, 0)  # No movement
                continue

            # ✅ Select the **best path** among candidates
            best_path = None
            max_objects = 0
            best_distance = float("inf")
            best_impacted_cells = {}

            for path_info in best_paths:
                path = path_info["path"]
                total_objects = path_info["objects"]
                total_distance = path_info["distance"]

                if use_spillage_model:
                    # ✅ Estimate objects reaching the highway **considering spillage**
                    _, impacted_cells, estimated_objects, _ = simulate_spillage(
                        waypoints=[(c.x, c.y) for c in path],
                        objects_at_cells={(c.x, c.y): c.num_objects for c in path if c.num_objects > 0},
                        agent_capacity=self.agent_capacity,
                        spillage_factor=self.spillage_factor,
                        min_spillage_threshold=self.min_spillage_threshold
                    )
                else:
                    estimated_objects = total_objects
                    impacted_cells = {}

                # ✅ Choose the best path considering estimated objects & distance
                if estimated_objects > max_objects or (
                        estimated_objects == max_objects and total_distance < best_distance):
                    max_objects = estimated_objects
                    best_distance = total_distance
                    best_path = path
                    best_impacted_cells = impacted_cells  # Store impacted cells!

            # ✅ Assign the best found path
            cell.best_path_highway = best_path
            cell.total_objects_highway = max_objects
            cell.total_distance_highway = best_distance
            cell.impacted_cells_highway = best_impacted_cells  # ✅ Store impacted cells!

            # ✅ Compute velocity direction for the agent
            if len(best_path) > 1:
                next_cell = best_path[1]
                dx = next_cell.x - cell.x
                dy = next_cell.y - cell.y
                magnitude = math.sqrt(dx ** 2 + dy ** 2) + 1e-5  # Avoid division by zero
                cell.velocity_highway = (dx / magnitude, dy / magnitude)
            else:
                cell.velocity_highway = (0, 0)  # No movement

        print("Highway path calculation complete.")

    def find_closest_point_on_target(self, object_position):
        """
        Find the closest point on the target zone boundary for a given object.
        :param object_position: (x, y) tuple representing the object's position.
        :return: (x, y) tuple of the closest point on the boundary.
        """
        object_point = Point(object_position)
        closest_point = nearest_points(object_point, self.target_zone.boundary)[1]
        return closest_point.x, closest_point.y

    def _clear_cell_objects(self, cell):
        """Clear objects and reset path attributes for a cell."""
        cell.num_objects = 0
        cell.current_objects = 0
        cell.best_path_target = []
        cell.next_child_target = None
        cell.total_objects_path_target = 0
        cell.total_objects_target = 0
        cell.best_path_highway = []
        cell.next_child_highway = None
        cell.total_objects_path_highway = 0
        cell.total_objects_highway = 0
        self._clear_cell_cache(cell)
    
    def _clear_cell_cache(self, cell):
        """Clear spillage and other caches for a cell."""
        if hasattr(cell, 'cached_spillage_path'):
            delattr(cell, 'cached_spillage_path')
        if hasattr(cell, 'spillage_cache_state'):
            delattr(cell, 'spillage_cache_state')
    
    def _update_cells_with_objects_tracking(self):
        """
        Ensure cells_with_objects list is consistent with actual object counts.
        Remove cells with 0 objects, add cells with >0 objects.
        """
        # Remove cells that no longer have objects
        cells_to_remove = [cell for cell in self.cells_with_objects if cell.num_objects <= 0]
        for cell in cells_to_remove:
            self.cells_with_objects.remove(cell)
            print(f"Removed cell ({cell.x}, {cell.y}) from tracking (no objects)")
            
        # Remove target zone cells that no longer have objects
        target_zone_cells_to_remove = [cell for cell in self.target_zone_cells if cell.num_objects <= 0]
        for cell in target_zone_cells_to_remove:
            self.target_zone_cells.remove(cell)
            print(f"Removed target zone cell ({cell.x}, {cell.y}) from visualization (no objects)")
        
        # Find cells with objects that aren't being tracked
        cells_to_add = []
        target_zone_cells_to_add = []
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                cell = self.get_cell(x, y)
                if cell and cell.num_objects > 0:
                    if cell.is_target_zone:
                        # Target zone cells go to separate list
                        if cell not in self.target_zone_cells:
                            target_zone_cells_to_add.append(cell)
                    else:
                        # Regular cells go to cells_with_objects
                        if cell not in self.cells_with_objects:
                            cells_to_add.append(cell)
        
        # Add target zone cells to separate list (for visualization only)
        for cell in target_zone_cells_to_add:
            self.target_zone_cells.append(cell)
            print(f"Added target zone cell ({cell.x}, {cell.y}) to visualization list ({cell.num_objects} objects)")
        
        # Add and initialize new regular cells with objects
        for cell in cells_to_add:
            self.cells_with_objects.append(cell)
            print(f"Added cell ({cell.x}, {cell.y}) to tracking ({cell.num_objects} objects)")
            # Initialize the cell for pathfinding if not already done
            if not hasattr(cell, 'visible_cells_target') or not cell.visible_cells_target:
                print(f"  Initializing cell ({cell.x}, {cell.y}) for pathfinding...")
                self._initialize_spillage_cell(cell)
        
        return len(cells_to_remove), len(cells_to_add) + len(target_zone_cells_to_add)
    
    def get_all_cells_with_objects(self):
        """Get all cells with objects (both regular and target zone) for visualization."""
        return self.cells_with_objects + self.target_zone_cells
    
    def _initialize_spillage_cell(self, spillage_cell):
        """Initialize a spillage cell with required attributes for pathfinding."""
        # Set up distance to target (same as main.py does for original cells)
        closest_point = self.find_closest_point_on_target((spillage_cell.x + 0.5, spillage_cell.y + 0.5))
        spillage_cell.closest_x = closest_point[0]
        spillage_cell.closest_y = closest_point[1]
        spillage_cell.distance_to_target = math.hypot(
            spillage_cell.x + 0.5 - closest_point[0], 
            spillage_cell.y + 0.5 - closest_point[1]
        )
        
        # Set up visibility scope (use configured angle tolerance)
        visible_cells_target, distance_to_children_target = self.calculate_target_zone_visibility(
            spillage_cell, angle_tolerance=self.target_angle_tolerance
        )
        spillage_cell.visible_cells_target = visible_cells_target
        spillage_cell.distance_to_children_target = distance_to_children_target
        spillage_cell.h_vis_target = sum(n["cell"].num_objects for n in visible_cells_target)
        
        # Essential path attributes (using same defaults as Cell constructor)
        spillage_cell.best_path_target = []
        spillage_cell.total_objects_target = spillage_cell.num_objects
        spillage_cell.total_objects_raw = spillage_cell.num_objects  # Same as target initially
        spillage_cell.total_objects_path_target = 0
        spillage_cell.solved_target = False
        spillage_cell.next_child_target = None
        spillage_cell.total_distance_target = 0
        
        # Ensure current_objects is set correctly
        spillage_cell.current_objects = spillage_cell.num_objects

    def execute_path(self, start_cell, path_type, use_spillage=True, precomputed_path=None):
        """
        Execute the path for a given start cell based on the specified path type.
        Transfers objects along the path, optionally using precomputed spillage effects.

        :param start_cell: The cell where the path starts.
        :param path_type: The type of path ('target' or 'highway').
        :param use_spillage: If True, use precomputed spillage; if False, move all objects to the final cell.
        :param precomputed_path: If provided, use this path instead of cached path (dict with 'path', 'objects', 'distance').
        """
        # Use precomputed path if provided, otherwise fall back to cached path
        if precomputed_path:
            best_path = precomputed_path['path']
            total_objects = precomputed_path['objects']
            impacted_cells = precomputed_path.get('impacted_cells', None)  # Use spillage from path planning
        elif path_type == "target":
            best_path = start_cell.best_path_target
            total_objects = start_cell.total_objects_target
            impacted_cells = start_cell.impacted_cells_target if use_spillage else None
        elif path_type == "highway":
            best_path = start_cell.best_path_highway
            total_objects = start_cell.total_objects_highway
            impacted_cells = start_cell.impacted_cells_highway if use_spillage else None
        else:
            print(f"WARNING: Invalid path type: {path_type}")
            return

        # ✅ Ensure a valid path exists
        if not best_path or len(best_path) < 2:
            print(f"WARNING: No valid path found for cell ({start_cell.x}, {start_cell.y}) with path type '{path_type}'.")
            return

        # ✅ Track affected cells (for backward propagation later)
        affected_cells = set()
        
        # ✅ Track objects for conservation verification
        initial_objects = sum(cell.num_objects for cell in best_path)
        
        # Note: Spillage simulation should be pre-computed during path planning, not here
        
        # Print information about the path
        print(f"Path from ({start_cell.x}, {start_cell.y}) to ({best_path[-1].x}, {best_path[-1].y}):")
        print(f"Initial objects in all path cells: {initial_objects}")
        if use_spillage and impacted_cells:
            print(f"Total impacted cells: {len(impacted_cells)}")
            print(f"Sum of objects in impacted cells: {sum(impacted_cells.values())}")
            
            # Check for conservation
            if abs(initial_objects - sum(impacted_cells.values())) > 0.01:
                print(f"WARNING: Conservation issue detected! Difference: {initial_objects - sum(impacted_cells.values())}")

        # Apply execution logic based on spillage mode
        if use_spillage and impacted_cells:
            # Step 1: Clear objects from path cells (simulating agent pickup)
            print("Clearing objects from path cells:")
            for cell in best_path:
                print(f"  Clearing cell ({cell.x}, {cell.y}): {cell.num_objects} objects")
                self._clear_cell_objects(cell)
                affected_cells.add(cell)
                if cell in self.cells_with_objects:
                    self.cells_with_objects.remove(cell)

            # Step 2: Distribute objects based on spillage simulation
            print("Distributing objects according to spillage model:")
            for (cell_x, cell_y), spilled_objects in impacted_cells.items():
                print(f"  Cell ({cell_x}, {cell_y}): {spilled_objects} objects")
                cell = self.get_cell(cell_x, cell_y)
                if cell:
                    cell.num_objects += spilled_objects
                    cell.current_objects += spilled_objects
                    self._clear_cell_cache(cell)
                    affected_cells.add(cell)

                    # Add spillage cells to tracking immediately for spillage_affected_cells tracking
                    if cell not in self.cells_with_objects and cell.num_objects > 0:
                        self.cells_with_objects.append(cell)
                        print(f"  Added spillage cell ({cell.x}, {cell.y}) to cells_with_objects")
                        # Initialization will be handled by _update_cells_with_objects_tracking() if needed

        else:
            # ✅ **Move all objects to the last cell, clearing intermediate ones**
            final_cell = best_path[-1]
            final_cell.num_objects += total_objects
            final_cell.current_objects += total_objects
            affected_cells.add(final_cell)

            # ✅ Ensure final cell is tracked in cells_with_objects
            if final_cell not in self.cells_with_objects and final_cell.num_objects > 0:
                self.cells_with_objects.append(final_cell)
                # Note: visibility will be set up in update_environment

            # ✅ Remove objects from all path cells (except the final cell)
            for cell in best_path[:-1]:  # Exclude final cell
                cell.num_objects = 0
                cell.current_objects = 0
                # ✅ Clear both target and highway path cache for zeroed cells
                cell.best_path_target = []
                cell.next_child_target = None
                cell.total_objects_path_target = 0
                cell.total_objects_target = 0
                # Clear highway cache as well
                cell.best_path_highway = []
                cell.next_child_highway = None
                cell.total_objects_path_highway = 0
                cell.total_objects_highway = 0
                affected_cells.add(cell)
                if cell in self.cells_with_objects:
                    self.cells_with_objects.remove(cell)

        # Verify conservation after execution
        final_objects = sum(cell.num_objects for cell in affected_cells)
        print(f"Objects after execution: {final_objects}")
        if abs(initial_objects - final_objects) > 0.01:
            print(f"WARNING: Post-execution conservation issue detected! Difference: {initial_objects - final_objects}")
        
        # ✅ Ensure cells_with_objects tracking is consistent
        print("Verifying cells_with_objects tracking consistency...")
        removed_count, added_count = self._update_cells_with_objects_tracking()
        if removed_count > 0 or added_count > 0:
            print(f"Tracking updated: removed {removed_count}, added {added_count}")
        else:
            print("Tracking is consistent")
        
        # ✅ Store affected cells for `update_environment`
        self.affected_cells = affected_cells
        
        # ✅ Store spillage-specific affected cells for visualization
        if use_spillage and impacted_cells:
            self.spillage_affected_cells = set()
            for (cell_x, cell_y), spilled_objects in impacted_cells.items():
                spillage_cell = self.get_cell(cell_x, cell_y)
                if spillage_cell:
                    self.spillage_affected_cells.add(spillage_cell)
            print(f"SPILLAGE: {len(self.spillage_affected_cells)} cells impacted by spillage")
        else:
            self.spillage_affected_cells = set()

        print(f"Executed path ({path_type}) for ({start_cell.x}, {start_cell.y}). "
              f"Objects moved to ({best_path[-1].x}, {best_path[-1].y}). "
              f"Using spillage: {use_spillage}")

    def update_environment(self):
        """
        Update only the affected cells instead of recalculating everything.
        """
        if not hasattr(self, "affected_cells") or not self.affected_cells:
            print("No affected cells. Skipping update.")
            return

        print("Updating visibility for affected cells...")
        
        # ✅ Ensure cells_with_objects tracking is consistent before recalculation
        print("Final verification of cells_with_objects tracking...")
        removed_count, added_count = self._update_cells_with_objects_tracking()
        if removed_count > 0 or added_count > 0:
            print(f"Final tracking update: removed {removed_count}, added {added_count}")

        # ✅ Separate different types of affected cells
        all_affected_cells_raw = set(self.affected_cells)
        
        # Separate cells that now have no objects vs cells that have objects
        emptied_cells = set()  # Cells that had objects but now don't (for visualization only)
        cells_with_objects_affected = set()  # Cells that still/now have objects (for A* processing)
        
        for cell in all_affected_cells_raw:
            if cell.num_objects > 0:
                cells_with_objects_affected.add(cell)
            else:
                emptied_cells.add(cell)
        
        # ✅ Add spillage cells (these always have objects by definition)
        spillage_cells = set()
        if hasattr(self, 'spillage_affected_cells') and self.spillage_affected_cells:
            spillage_cells = set(self.spillage_affected_cells)
            # Spillage cells should all have objects, but let's verify and filter
            spillage_cells_with_objects = {cell for cell in spillage_cells if cell.num_objects > 0}
            cells_with_objects_affected.update(spillage_cells_with_objects)
            print(f"Added {len(spillage_cells_with_objects)} spillage cells with objects to processing")
        
        print(f"Cell breakdown: {len(emptied_cells)} emptied, {len(cells_with_objects_affected)} with objects")
        
        # ✅ Filter out target zone cells - they don't need pathfinding processing
        all_direct_affected = {cell for cell in cells_with_objects_affected if not cell.is_target_zone}
        target_zone_cells_filtered = len(cells_with_objects_affected) - len(all_direct_affected)
        if target_zone_cells_filtered > 0:
            print(f"Filtered out {target_zone_cells_filtered} target zone cells from processing")
        
        # ✅ Find recalculation cells: cells that can see ANY affected cell (including emptied ones)
        # Need to check visibility against ALL affected cells (emptied + with objects) to update stale visibility
        all_affected_for_visibility_check = emptied_cells | cells_with_objects_affected
        # Filter out target zone cells from visibility check set
        all_affected_for_visibility_check = {cell for cell in all_affected_for_visibility_check if not cell.is_target_zone}
        
        recalculation_cells = set()
        
        for candidate in self.cells_with_objects:
            if candidate in all_direct_affected:
                continue  # Skip cells that are already directly affected
                
            # Skip target zone cells - they don't need pathfinding
            if candidate.is_target_zone:
                continue
            
            # Check if candidate can see any affected cell (including emptied ones)
            for affected_cell in all_affected_for_visibility_check:
                if self.is_cell_in_visibility_scope(candidate, affected_cell):
                    recalculation_cells.add(candidate)
                    status = "emptied" if affected_cell in emptied_cells else "with objects"
                    print(f"Cell ({candidate.x}, {candidate.y}) can see affected cell ({affected_cell.x}, {affected_cell.y}) [{status}] - marked for recalculation")
                    break  # Found one dependency, that's enough
        
        # ✅ Combine for total affected cells  
        all_affected_cells = all_direct_affected | recalculation_cells
        
        # ✅ Store different types for visualization
        self.direct_affected_cells = list(emptied_cells)  # Cells that were emptied (visualization only)
        self.spillage_cells = list(spillage_cells)  # Cells created by spillage  
        self.recalculation_cells = list(recalculation_cells)  # Cells that need recalculation
        
        print(f"Emptied cells (for visualization): {len(emptied_cells)}")
        print(f"Spillage cells: {len(spillage_cells)}")  
        print(f"Recalculation cells: {len(recalculation_cells)}")
        print(f"Cells with objects (for A* processing): {len(all_affected_cells)}")
        print(f"Total affected cells: {len(all_affected_cells)}")
        
        # Show spillage cells being included in recalculation
        if spillage_cells:
            print(f"Spillage cells included in recalculation:")
            for spillage_cell in spillage_cells:
                print(f"  Cell ({spillage_cell.x}, {spillage_cell.y}): {spillage_cell.num_objects} objects")

        # ✅ STEP 0: Invalidate solved flags for affected cells (spillage OFF only)
        if not self.use_spillage_model:
            print("Invalidating solved flags for affected cells...")
            self.invalidate_target_memo(all_affected_cells)

        # ✅ STEP 1: Refresh visibility for affected cells using current truth
        print("Refreshing visibility for affected cells...")
        for cell in all_affected_cells:
            if cell.num_objects > 0:  # Only refresh visibility for cells that still have objects
                cell.visible_cells_target, cell.distance_to_children_target = \
                    self.calculate_target_zone_visibility(cell, angle_tolerance=self.target_angle_tolerance)  # Use configured target angle
                
                # Compute optimistic heuristic (sum of objects in visibility scope)
                cell.h_vis_target = sum(n["cell"].num_objects for n in cell.visible_cells_target)

        # ✅ STEP 2: Recalculate target A* paths for cells with objects only (uses fresh visibility)
        print(f"Recalculating target paths for {len(all_affected_cells)} cells with objects...")
        self.calculate_potential_field(use_spillage_model=self.use_spillage_model, affected_cells=all_affected_cells)
        
        # ✅ STEP 3: Clear stale propagation values and propagate target paths
        print("Clearing stale propagation values for all affected cells...")
        for cell in all_affected_cells:
            cell.total_objects_path_target = 0  # Clear stale propagation values
            
        # Propagate target paths for all affected cells (both spillage and non-spillage modes)
        print("Propagating target paths for all affected cells...")
        self.propagate_total_objects_path_target_selective(all_affected_cells)

        # ✅ STEP 4: Update velocity field (uses fresh target paths) - all affected cells
        print("Updating velocity field for all affected cells...")
        self.calculate_velocity_field(affected_cells=all_affected_cells)

        # ✅ STEP 5: Heat map recalculation (uses propagated target data)
        print("Updating heat map...")
        self.update_heat_map()

        # ✅ STEP 6: Recalculate highway paths (uses updated heat map)
        print("Calculating paths to highways for low-potential cells...")
        self.calculate_path_to_highway(use_spillage_model=self.use_spillage_model)

        # ✅ STEP 7: Remove cells that no longer contain objects AFTER all calculations
        self.cells_with_objects = [cell for cell in self.cells_with_objects if cell.num_objects > 0]
        self.target_zone_cells = [cell for cell in self.target_zone_cells if cell.num_objects > 0]

        print("Environment update complete.")
        
        # ✅ Orphan cell diagnostics - ensure all cells are properly tracked
        self.audit_orphan_cells()

    def invalidate_target_memo(self, cells):
        """
        Reset solved flags and heuristic cache for specified cells.
        Called after path execution to invalidate affected cells.
        """
        for c in cells:
            c.solved_target = False
            c.h_resolved_target = 0
            # Note: We keep h_vis_target since it's based on geometry, not paths
            # Note: We keep total_objects_target and best_path_target for now - they'll be recomputed

    def audit_orphan_cells(self):
        """Audit for orphan cells and ensure all cells are properly tracked."""
        orphan_count = 0
        tracked_positions = {(cell.x, cell.y) for cell in self.all_cells}
        
        # Check cells_with_objects for orphans
        for cell in self.cells_with_objects:
            if (cell.x, cell.y) not in tracked_positions:
                print(f"WARNING: ORPHAN CELL DETECTED: ({cell.x}, {cell.y}) with {cell.num_objects} objects")
                orphan_count += 1
                
        # Verify no duplicate positions
        positions_with_objects = [(cell.x, cell.y) for cell in self.cells_with_objects]
        unique_positions = set(positions_with_objects)
        if len(positions_with_objects) != len(unique_positions):
            duplicates = len(positions_with_objects) - len(unique_positions)
            print(f"WARNING: DUPLICATE CELLS DETECTED: {duplicates} duplicate positions in cells_with_objects")
            
        if orphan_count == 0 and len(positions_with_objects) == len(unique_positions):
            print("Cell tracking audit passed - no orphans or duplicates detected")
        else:
            print(f"Cell tracking issues: {orphan_count} orphans detected")
            
        return orphan_count == 0

    def in_cone_and_gate(self, observer, neighbor, target_cell=None):
        """
        Helper function to check if neighbor is within observer's cone and gate.
        Uses the same math as actual visibility calculations to ensure consistency.
        
        :param observer: The observing cell
        :param neighbor: The potential neighbor cell
        :param target_cell: Optional target cell (for highway mode), if None uses target zone
        :return: True if neighbor is within cone and gate constraints
        """
        # Use configured target angle tolerance
        angle_tolerance = self.target_angle_tolerance
        
        # Use cell centers for consistency
        Sx, Sy = observer.x + 0.5, observer.y + 0.5
        
        if target_cell is None:
            # Target zone mode - use closest point in target zone
            Tx, Ty = float(observer.closest_x), float(observer.closest_y)
            gate_distance = observer.closest_distance * self.target_zone_gate_factor
        else:
            # Highway mode - use specific target cell
            Tx, Ty = target_cell.x + 0.5, target_cell.y + 0.5
            angle_tolerance = self.highway_angle_tolerance
            gate_distance = math.hypot(Tx - Sx, Ty - Sy) * 1.5  # Highway gate factor
        
        # Precompute observer-to-target vector and squared distance
        vx, vy = Tx - Sx, Ty - Sy
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return False  # Observer is on target - no meaningful direction
        
        cos_th = math.cos(math.radians(angle_tolerance))
        
        # Check the neighbor cell using same logic as visibility calculations
        Nx, Ny = neighbor.x + 0.5, neighbor.y + 0.5
        dx, dy = Nx - Sx, Ny - Sy
        
        # Skip self-check  
        candidate_dist_sq = dx * dx + dy * dy
        if candidate_dist_sq == 0:
            return False
        
        # ✅ CONE CONSTRAINT: Angular tolerance check
        dot = dx * vx + dy * vy
        in_cone = dot >= math.sqrt(candidate_dist_sq * L2) * cos_th
        
        # ✅ GATE CONSTRAINT: Distance within gate factor
        distance_to_candidate = math.sqrt(candidate_dist_sq)
        within_gate = distance_to_candidate <= gate_distance
        
        # ✅ PROGRESS CONSTRAINT: Monotone progress toward target (target zone mode only)
        if target_cell is None:
            progress_constraint = neighbor.distance_to_target <= observer.distance_to_target
            return in_cone and within_gate and progress_constraint
        else:
            # Highway mode: just cone and gate
            return in_cone and within_gate

    def is_geometrically_visible_to_target(self, observer_cell, candidate_cell, angle_tolerance=None):
        """
        Check if candidate_cell is geometrically visible from observer_cell toward target zone.
        Uses the same geometric constraints as calculate_target_zone_visibility() but for individual cell pairs.
        This function does NOT rely on pre-computed visibility lists.
        
        :param observer_cell: The cell whose visibility scope we're checking from
        :param candidate_cell: The cell we want to know if it's geometrically visible
        :param angle_tolerance: Angle in degrees defining cone of vision. If None, uses configured target angle.
        :return: True if candidate_cell satisfies all geometric visibility constraints
        """
        # Use configured target angle tolerance if not specified
        if angle_tolerance is None:
            angle_tolerance = self.target_angle_tolerance
        
        # Use cell centers for consistency (same as calculate_target_zone_visibility)
        Sx, Sy = observer_cell.x + 0.5, observer_cell.y + 0.5
        
        # Target direction using closest point in target zone (same logic)
        Tx, Ty = float(observer_cell.closest_x), float(observer_cell.closest_y)
        
        # Precompute observer-to-target vector and squared distance
        vx, vy = Tx - Sx, Ty - Sy
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return False  # Observer is on target - no meaningful direction
        
        cos_th = math.cos(math.radians(angle_tolerance))
        
        # Check the candidate cell using same logic as calculate_target_zone_visibility
        Nx, Ny = candidate_cell.x + 0.5, candidate_cell.y + 0.5
        dx, dy = Nx - Sx, Ny - Sy
        
        # Skip self-check  
        candidate_dist_sq = dx * dx + dy * dy
        if candidate_dist_sq == 0:
            return False
        
        # ✅ CONE CONSTRAINT: Angular tolerance check (same math as original)
        dot = dx * vx + dy * vy
        in_cone = dot >= math.sqrt(candidate_dist_sq * L2) * cos_th
        
        # ✅ GATE CONSTRAINT: Distance within gate factor (same logic)
        distance_to_candidate = math.sqrt(candidate_dist_sq)
        within_gate = distance_to_candidate <= observer_cell.closest_distance * self.target_zone_gate_factor
        
        # ✅ PROGRESS CONSTRAINT: Monotone progress toward target (same constraint)
        progress_constraint = candidate_cell.distance_to_target <= observer_cell.distance_to_target
        
        # Apply all three constraints (same as calculate_target_zone_visibility)
        return in_cone and within_gate and progress_constraint

    def is_cell_in_visibility_scope(self, observer_cell, target_cell):
        """
        Check if target_cell is within observer_cell's visibility scope toward target zone.
        Uses hybrid approach: fast path for pre-computed cells + geometric fallback for new cells.
        
        :param observer_cell: The cell whose visibility scope we're checking
        :param target_cell: The cell we want to know if it's visible
        :return: True if target_cell is in observer_cell's target visibility scope
        """
        # ✅ FAST PATH: Check pre-computed visibility list first (for cells that had objects during calculation)
        if hasattr(observer_cell, 'visible_cells_target') and observer_cell.visible_cells_target:
            for visible_info in observer_cell.visible_cells_target:
                if visible_info["cell"] == target_cell:
                    return True
        
        # ✅ FALLBACK PATH: Use consistent cone and gate check (matches actual visibility calculations)
        # This handles newly populated cells after path execution and ensures consistency
        return self.in_cone_and_gate(observer_cell, target_cell, target_cell=None)

    def is_visible_from(self, candidate, reference_cell, angle_tolerance=90):
        """
        Checks if `candidate` is within the visibility scope of `reference_cell`
        based on an angular tolerance.

        :param candidate: The cell being checked for visibility.
        :param reference_cell: The cell that was impacted and needs updates.
        :param angle_tolerance: Maximum angle deviation allowed for visibility. If None, uses dynamic angle.
        :return: True if candidate is visible from reference_cell, False otherwise.
        """
        # Use dynamic angle tolerance if not specified
        if angle_tolerance is None:
            angle_tolerance = self.get_angle_tolerance_deg(reference_cell)
        # Compute vector from reference_cell to candidate
        to_candidate_x = (candidate.x + 0.5) - (reference_cell.x + 0.5)
        to_candidate_y = (candidate.y + 0.5) - (reference_cell.y + 0.5)
        magnitude_to_candidate = math.sqrt(to_candidate_x ** 2 + to_candidate_y ** 2)

        if magnitude_to_candidate == 0:  # Prevent self-check
            return False

        unit_to_candidate = (to_candidate_x / magnitude_to_candidate, to_candidate_y / magnitude_to_candidate)

        # Compute reference vector (from target to reference_cell)
        from_target_x = (reference_cell.x + 0.5) - reference_cell.closest_x
        from_target_y = (reference_cell.y + 0.5) - reference_cell.closest_y
        magnitude_from_target = math.sqrt(from_target_x ** 2 + from_target_y ** 2)

        if magnitude_from_target == 0:  # Avoid division by zero
            return False

        unit_from_target = (from_target_x / magnitude_from_target, from_target_y / magnitude_from_target)

        # Compute dot product between reference direction and candidate direction
        dot_product = max(0, unit_from_target[0] * unit_to_candidate[0] +
                          unit_from_target[1] * unit_to_candidate[1])

        # Check if within angular tolerance
        return dot_product > math.cos(math.radians(angle_tolerance))

    def get_angle_tolerance_deg(self, current_cell, target_cell=None):
        """
        Calculate dynamic angle tolerance based on distance to target.
        Far cells get tight angles (angle_min_deg), near cells get wider angles (angle_max_deg).
        
        :param current_cell: The cell whose angle tolerance is being calculated.
        :param target_cell: Optional specific target cell. If None, uses closest point on target zone.
        :return: Angle tolerance in degrees.
        """
        Sx, Sy = current_cell.x + 0.5, current_cell.y + 0.5
        if target_cell is not None:
            Tc = self.get_cell(int(target_cell.x), int(target_cell.y))
            if not Tc:
                return self.angle_min_deg
            Tx, Ty = Tc.x + 0.5, Tc.y + 0.5
        else:
            Tx, Ty = float(current_cell.closest_x), float(current_cell.closest_y)
        
        dST = math.hypot(Tx - Sx, Ty - Sy)
        frac = min(dST / (self.grid_diagonal or 1.0), 1.0)  # 1.0 when far, 0 near
        return self.angle_min_deg + (self.angle_max_deg - self.angle_min_deg) * (1.0 - frac)

    def calculate_visibility_simple(self, current_cell, angle_tolerance=None, target_cell=None):
        """
        Calculate visibility for a cell, adding neighbors within a cone of vision toward the target zone or a specific target.
        Always include the closest boundary cell of the target zone or the specified target cell if no visible cells are found.

        :param current_cell: The cell whose visibility is being calculated.
        :param angle_tolerance: Angle in degrees defining the cone of vision. If None, uses dynamic angle.
        :param target_cell: Optional specific target cell to calculate visibility toward. Defaults to the target zone.
        :return: A tuple (visible_cells, distance_to_children) where:
                 - visible_cells is a list of visible cells within the cone of vision.
                 - distance_to_children is a dictionary with keys as (x, y) tuples and values as distances.
        """
        # Use dynamic angle tolerance if not specified
        if angle_tolerance is None:
            angle_tolerance = self.get_angle_tolerance_deg(current_cell, target_cell)
        
        visible_cells = []
        distance_to_children = {}

        # ALL vectors use centers for consistency
        Sx, Sy = current_cell.x + 0.5, current_cell.y + 0.5
        
        # Determine the target direction (either target_cell or closest point in the target zone)
        if target_cell:
            # Map to canonical cell instance
            tx, ty = int(target_cell.x), int(target_cell.y)
            target_cell = self.get_cell(tx, ty)
            if not target_cell:
                return visible_cells, distance_to_children
            Tx, Ty = target_cell.x + 0.5, target_cell.y + 0.5
        else:
            Tx, Ty = float(current_cell.closest_x), float(current_cell.closest_y)

        # Precompute once: vx,vy = T - S; L2 = vx*vx + vy*vy; cos_th = cos(radians(angle_tolerance))
        vx, vy = Tx - Sx, Ty - Sy
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return visible_cells, distance_to_children  # Prevent division by zero if current cell is on target
        cos_th = math.cos(math.radians(angle_tolerance))

        # Add visible cells that satisfy distance and angular criteria
        for neighbor in self.cells_with_objects:
            if neighbor == current_cell:
                continue
                
            # Skip cells inside target zone - they're destinations, not path steps
            if neighbor.is_target_zone:
                continue

            # Use centers for all calculations - per neighbor: dx = N - S
            Nx, Ny = neighbor.x + 0.5, neighbor.y + 0.5
            dx, dy = Nx - Sx, Ny - Sy
            
            # Efficient cone check using squared distances (no sqrt per neighbor)
            dot = dx * vx + dy * vy
            neighbor_dist_sq = dx * dx + dy * dy
            if neighbor_dist_sq == 0:
                continue  # Skip if same position
            
            in_cone = dot >= math.sqrt(neighbor_dist_sq * L2) * cos_th
            
            # Calculate actual distance for return value
            distance_to_neighbor = math.sqrt(neighbor_dist_sq)

            # Add the neighbor if it satisfies the criteria
            if target_cell:
                # GPT's strict cone constraints for highway paths to specific target
                # No overshoot: d²(S,N) ≤ d²(S,T) (using squared distances)
                no_overshoot = neighbor_dist_sq <= L2
                
                # Monotone progress: d²(N,T) < d²(S,T) (using squared distances)
                progress = ((Tx - Nx)**2 + (Ty - Ny)**2) < L2
                
                # Include if: in cone + no overshoot + makes progress + (has objects OR is exact target)
                if in_cone and no_overshoot and progress and (neighbor.num_objects > 0 or neighbor is target_cell):
                    visible_cells.append({"cell": neighbor})
                    distance_to_children[(neighbor.x, neighbor.y)] = distance_to_neighbor
            else:
                # Target zone constraints with gate factor
                within_gate = distance_to_neighbor <= current_cell.closest_distance * self.target_zone_gate_factor
                if (
                        neighbor.distance_to_target <= current_cell.distance_to_target
                        and in_cone  # Use improved cone check
                        and within_gate  # Use named parameter for gate
                ):
                    visible_cells.append({"cell": neighbor})
                    distance_to_children[(neighbor.x, neighbor.y)] = distance_to_neighbor

        # Always include the exact target_cell as a reachable option (GPT requirement)
        if target_cell:
            target_distance = math.hypot(target_cell.x - current_cell.x, target_cell.y - current_cell.y)
            # Only add if not already present
            target_key = (target_cell.x, target_cell.y)
            if target_key not in distance_to_children:
                visible_cells.append({"cell": target_cell})
                distance_to_children[target_key] = target_distance
        
        # Fallback for target zone visibility if no visible cells are found
        if not visible_cells and not target_cell:
                # Use the canonical boundary cell
                bx, by = int(current_cell.closest_x), int(current_cell.closest_y)
                boundary_cell = self.get_cell(bx, by)
                if boundary_cell:
                    visible_cells.append({"cell": boundary_cell})
                    distance_to_children[(bx, by)] = current_cell.closest_distance

        return visible_cells, distance_to_children

    def calculate_target_zone_visibility(self, current_cell, angle_tolerance=None):
        """
        Calculate visibility for target zone paths using closest point on target boundary.
        No gate factor constraint to maintain A* admissibility.
        
        :param current_cell: The cell whose visibility is being calculated.
        :param angle_tolerance: Angle in degrees defining the cone of vision. If None, uses dynamic angle.
        :return: A tuple (visible_cells, distance_to_children)
        """
        # Immediate return for target zone cells - they don't need pathfinding
        if current_cell.is_target_zone:
            return [], {}
        
        # Use dynamic angle tolerance if not specified
        if angle_tolerance is None:
            angle_tolerance = self.get_angle_tolerance_deg(current_cell, target_cell=None)
        
        visible_cells = []
        distance_to_children = {}

        # Use cell centers for consistency
        Sx, Sy = current_cell.x + 0.5, current_cell.y + 0.5
        
        # Target direction using closest point in target zone
        Tx, Ty = float(current_cell.closest_x), float(current_cell.closest_y)

        # Precompute vector and squared distance
        vx, vy = Tx - Sx, Ty - Sy
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return visible_cells, distance_to_children
        cos_th = math.cos(math.radians(angle_tolerance))

        # Check each neighbor (exclude target zone cells as they shouldn't be path targets)
        for neighbor in self.cells_with_objects:
            if neighbor == current_cell:
                continue
                
            # Skip cells inside target zone - they're destinations, not path steps
            if neighbor.is_target_zone:
                continue

            # Use centers for all calculations
            Nx, Ny = neighbor.x + 0.5, neighbor.y + 0.5
            dx, dy = Nx - Sx, Ny - Sy
            
            # Efficient cone check using squared distances
            dot = dx * vx + dy * vy
            neighbor_dist_sq = dx * dx + dy * dy
            if neighbor_dist_sq == 0:
                continue
            
            in_cone = dot >= math.sqrt(neighbor_dist_sq * L2) * cos_th
            
            # Target zone constraints: in-cone + monotone progress + gate factor
            distance_to_neighbor = math.sqrt(neighbor_dist_sq)
            within_gate = distance_to_neighbor <= current_cell.closest_distance * self.target_zone_gate_factor
            if (neighbor.distance_to_target <= current_cell.distance_to_target and in_cone and within_gate):
                visible_cells.append({"cell": neighbor})
                distance_to_children[(neighbor.x, neighbor.y)] = distance_to_neighbor

        # Fallback: include closest boundary cell if no visible cells found
        if not visible_cells:
            bx, by = int(current_cell.closest_x), int(current_cell.closest_y)
            boundary_cell = self.get_cell(bx, by)
            if boundary_cell:
                visible_cells.append({"cell": boundary_cell})
                distance_to_children[(bx, by)] = current_cell.closest_distance

        return visible_cells, distance_to_children

    def calculate_highway_visibility(self, current_cell, target_cell, angle_tolerance=None):
        """
        Calculate visibility for highway paths toward a specific target cell.
        Uses overshoot and progress constraints.
        
        :param current_cell: The cell whose visibility is being calculated.
        :param target_cell: The specific target cell to calculate visibility toward.
        :param angle_tolerance: Angle in degrees defining the cone of vision. If None, uses dynamic angle.
        :return: A tuple (visible_cells, distance_to_children)
        """
        # Use dynamic angle tolerance if not specified
        if angle_tolerance is None:
            angle_tolerance = self.get_angle_tolerance_deg(current_cell, target_cell)
        
        visible_cells = []
        distance_to_children = {}

        # Map to canonical cell instance
        tx, ty = int(target_cell.x), int(target_cell.y)
        target_cell = self.get_cell(tx, ty)
        if not target_cell:
            return visible_cells, distance_to_children

        # Use cell centers for consistency
        Sx, Sy = current_cell.x + 0.5, current_cell.y + 0.5
        Tx, Ty = target_cell.x + 0.5, target_cell.y + 0.5

        # Precompute vector and squared distance
        vx, vy = Tx - Sx, Ty - Sy
        L2 = vx * vx + vy * vy
        if L2 == 0:
            return visible_cells, distance_to_children
        cos_th = math.cos(math.radians(angle_tolerance))

        # Check each neighbor (exclude target zone cells as they shouldn't be path targets)
        for neighbor in self.cells_with_objects:
            if neighbor == current_cell:
                continue
                
            # Skip cells inside target zone - they're destinations, not path steps
            if neighbor.is_target_zone:
                continue

            # Use centers for all calculations
            Nx, Ny = neighbor.x + 0.5, neighbor.y + 0.5
            dx, dy = Nx - Sx, Ny - Sy
            
            # Efficient cone check using squared distances
            dot = dx * vx + dy * vy
            neighbor_dist_sq = dx * dx + dy * dy
            if neighbor_dist_sq == 0:
                continue
            
            in_cone = dot >= math.sqrt(neighbor_dist_sq * L2) * cos_th
            
            # Highway constraints: in-cone + no overshoot + monotone progress
            no_overshoot = neighbor_dist_sq <= L2
            progress = ((Tx - Nx)**2 + (Ty - Ny)**2) < L2
            
            # Include if: in cone + no overshoot + makes progress + (has objects OR is exact target)
            if in_cone and no_overshoot and progress and (neighbor.num_objects > 0 or neighbor is target_cell):
                distance_to_neighbor = math.sqrt(neighbor_dist_sq)
                visible_cells.append({"cell": neighbor})
                distance_to_children[(neighbor.x, neighbor.y)] = distance_to_neighbor

        # Always include the exact target_cell as a reachable option
        target_distance = math.sqrt(L2)
        target_key = (target_cell.x, target_cell.y)
        if target_key not in distance_to_children:
            visible_cells.append({"cell": target_cell})
            distance_to_children[target_key] = target_distance

        return visible_cells, distance_to_children

    def audit_visibility(self):
        """Collect cells that have no visible successors; print one summary line."""
        self.dead_visibility_target = []
        self.dead_visibility_highway = []

        for c in self.all_cells:
            # Target: skip cells strictly inside the target zone
            inside_target = False
            try:
                from shapely.geometry import Point
                inside_target = self.target_zone.contains(Point(c.x + 0.5, c.y + 0.5))
            except Exception:
                pass

            t = getattr(c, "visible_cells_target", []) or []
            h = getattr(c, "visible_cells_highway", []) or []

            if not t and not inside_target:
                self.dead_visibility_target.append((c.x, c.y))
            if not h:
                self.dead_visibility_highway.append((c.x, c.y))

        if self.dead_visibility_target or self.dead_visibility_highway:
            print(
                f"[Visibility Audit] Dead cells — target:{len(self.dead_visibility_target)} "
                f"highway:{len(self.dead_visibility_highway)}"
            )

    # ==================== STATE MANAGEMENT FOR MULTI-AGENT PLANNING ====================
    
    def get_state(self):
        """
        Export the current environment state for multi-agent planning.
        Returns a dictionary containing all necessary state information.
        """
        state = {
            # Core environment parameters
            'grid_size': self.grid_size,
            'target_zone_radius': self.target_zone_radius,
            'use_spillage_model': getattr(self, 'use_spillage_model', False),
            
            # Algorithm parameters
            'max_path_length_factor': self.max_path_length_factor,
            'target_angle_tolerance': self.target_angle_tolerance,
            'highway_angle_tolerance': self.highway_angle_tolerance,
            'highway_threshold': self.highway_threshold,
            'highway_min_heat_ratio': self.highway_min_heat_ratio,
            'highway_threshold_ratio': self.highway_threshold_ratio,
            'highway_heat_weight': self.highway_heat_weight,
            'highway_distance_weight': self.highway_distance_weight,
            
            # Spillage parameters
            'agent_capacity': getattr(self, 'agent_capacity', 8),
            'spillage_factor': getattr(self, 'spillage_factor', 0.05),
            'min_spillage_threshold': getattr(self, 'min_spillage_threshold', 0.03),
            
            # Cell states (most important for planning)
            'cell_states': self._serialize_cell_states(),
            
            # Object tracking
            'cells_with_objects_coords': [(cell.x, cell.y) for cell in self.cells_with_objects],
            'target_zone_cells_coords': [(cell.x, cell.y) for cell in self.target_zone_cells],
            
            # Target zone geometry (serialize as WKT string)
            'target_zone_wkt': self.target_zone.wkt if hasattr(self.target_zone, 'wkt') else None,
        }
        
        return state
    
    def set_state(self, state):
        """
        Import/restore environment state from a state dictionary.
        Used to restore previous states after planning simulations.
        """
        # Restore core parameters
        self.grid_size = state['grid_size']
        self.target_zone_radius = state['target_zone_radius']
        self.use_spillage_model = state['use_spillage_model']
        
        # Restore algorithm parameters
        self.max_path_length_factor = state['max_path_length_factor']
        self.target_angle_tolerance = state['target_angle_tolerance']
        self.highway_angle_tolerance = state['highway_angle_tolerance']
        self.highway_threshold = state['highway_threshold']
        self.highway_min_heat_ratio = state['highway_min_heat_ratio']
        self.highway_threshold_ratio = state['highway_threshold_ratio']
        self.highway_heat_weight = state['highway_heat_weight']
        self.highway_distance_weight = state['highway_distance_weight']
        
        # Restore spillage parameters
        self.agent_capacity = state['agent_capacity']
        self.spillage_factor = state['spillage_factor']
        self.min_spillage_threshold = state['min_spillage_threshold']
        
        # Restore target zone geometry
        if state['target_zone_wkt']:
            from shapely import wkt
            self.target_zone = wkt.loads(state['target_zone_wkt'])
        
        # Restore cell states
        self._deserialize_cell_states(state['cell_states'])
        
        # Rebuild object tracking lists
        self._rebuild_tracking_lists_from_coords(
            state['cells_with_objects_coords'],
            state['target_zone_cells_coords']
        )
        
        print(f"Environment state restored: {len(self.cells_with_objects)} cells with objects")
    
    def copy_state(self):
        """
        Create a deep copy of the environment for planning simulations.
        Returns a new SimulationEnv instance with identical state.
        """
        # Get current state
        current_state = self.get_state()
        
        # Create new environment with same basic parameters
        new_env = SimulationEnv(
            grid_size=current_state['grid_size'],
            target_zone_radius=current_state['target_zone_radius'],
            agent_positions=[],  # No agents in planning copy
            num_random_objects=0  # Objects will be restored from state
        )
        
        # Restore the complete state
        new_env.set_state(current_state)
        
        return new_env
    
    def _serialize_cell_states(self):
        """
        Serialize all cell states to a dictionary.
        Returns only the essential state information for each cell.
        """
        cell_states = {}
        
        for cell in self.all_cells:
            # Only serialize cells that have meaningful state
            if (cell.num_objects > 0 or 
                hasattr(cell, 'heat_map') or 
                hasattr(cell, 'best_path_target') or
                hasattr(cell, 'best_path_highway')):
                
                cell_state = {
                    'x': cell.x,
                    'y': cell.y,
                    'num_objects': cell.num_objects,
                    'current_objects': cell.current_objects,
                    'is_target_zone': getattr(cell, 'is_target_zone', False),
                    'heat_map': getattr(cell, 'heat_map', 0),
                    
                    # Path information (store as coordinate lists for serialization)
                    'best_path_target_coords': [(c.x, c.y) for c in getattr(cell, 'best_path_target', [])],
                    'best_path_highway_coords': [(c.x, c.y) for c in getattr(cell, 'best_path_highway', [])],
                    
                    # Object tracking values
                    'total_objects_target': getattr(cell, 'total_objects_target', 0),
                    'total_objects_raw': getattr(cell, 'total_objects_raw', 0),
                    'total_objects_path_target': getattr(cell, 'total_objects_path_target', 0),
                    'total_objects_highway': getattr(cell, 'total_objects_highway', 0),
                    
                    # Distance information
                    'distance_to_target': getattr(cell, 'distance_to_target', 0),
                    'total_distance_target': getattr(cell, 'total_distance_target', 0),
                    'total_distance_highway': getattr(cell, 'total_distance_highway', 0),
                    
                    # Velocity information
                    'velocity_target': getattr(cell, 'velocity_target', (0, 0)),
                    'velocity_highway': getattr(cell, 'velocity_highway', (0, 0)),
                }
                
                cell_states[(cell.x, cell.y)] = cell_state
        
        return cell_states
    
    def _deserialize_cell_states(self, cell_states):
        """
        Restore cell states from serialized data.
        Reconstructs all cell attributes and relationships.
        """
        # First pass: restore basic cell attributes
        for (x, y), cell_state in cell_states.items():
            cell = self.get_cell(x, y)
            if cell:
                cell.num_objects = cell_state['num_objects']
                cell.current_objects = cell_state['current_objects']
                cell.is_target_zone = cell_state['is_target_zone']
                cell.heat_map = cell_state['heat_map']
                
                # Restore object tracking values
                cell.total_objects_target = cell_state['total_objects_target']
                cell.total_objects_raw = cell_state['total_objects_raw']
                cell.total_objects_path_target = cell_state['total_objects_path_target']
                cell.total_objects_highway = cell_state['total_objects_highway']
                
                # Restore distances
                cell.distance_to_target = cell_state['distance_to_target']
                cell.total_distance_target = cell_state['total_distance_target']
                cell.total_distance_highway = cell_state['total_distance_highway']
                
                # Restore velocities
                cell.velocity_target = cell_state['velocity_target']
                cell.velocity_highway = cell_state['velocity_highway']
        
        # Second pass: restore path relationships (requires all cells to be restored first)
        for (x, y), cell_state in cell_states.items():
            cell = self.get_cell(x, y)
            if cell:
                # Restore best_path_target
                cell.best_path_target = []
                for path_x, path_y in cell_state['best_path_target_coords']:
                    path_cell = self.get_cell(path_x, path_y)
                    if path_cell:
                        cell.best_path_target.append(path_cell)
                
                # Restore best_path_highway
                cell.best_path_highway = []
                for path_x, path_y in cell_state['best_path_highway_coords']:
                    path_cell = self.get_cell(path_x, path_y)
                    if path_cell:
                        cell.best_path_highway.append(path_cell)
    
    def _rebuild_tracking_lists_from_coords(self, cells_with_objects_coords, target_zone_cells_coords):
        """
        Rebuild cells_with_objects and target_zone_cells lists from coordinate lists.
        """
        self.cells_with_objects = []
        for x, y in cells_with_objects_coords:
            cell = self.get_cell(x, y)
            if cell and cell.num_objects > 0:
                self.cells_with_objects.append(cell)
        
        self.target_zone_cells = []
        for x, y in target_zone_cells_coords:
            cell = self.get_cell(x, y)
            if cell and cell.num_objects > 0:
                self.target_zone_cells.append(cell)
        
        print(f"Rebuilt tracking: {len(self.cells_with_objects)} regular cells, {len(self.target_zone_cells)} target zone cells")

    # ==================== ENVIRONMENT EVALUATION METRICS FOR PLANNING ====================
    
    def evaluate_environment_state(self, weights=None):
        """
        Comprehensive evaluation of the current environment state.
        Returns a weighted composite score for multi-agent planning decisions.
        
        :param weights: Dictionary of weights for different metrics
        :return: Dictionary containing individual scores and composite score
        """
        if weights is None:
            weights = {
                'target_progress': 0.4,      # Highest priority: objects reaching target
                'highway_utilization': 0.25, # Objects positioned on efficient highways
                'spatial_concentration': 0.2, # Objects clustering toward target
                'flow_efficiency': 0.15       # Overall flow field quality
            }
        
        # Calculate individual metrics
        metrics = {
            'target_progress': self.calculate_target_progress_score(),
            'highway_utilization': self.calculate_highway_utilization_score(),
            'spatial_concentration': self.calculate_spatial_concentration_score(),
            'flow_efficiency': self.calculate_flow_efficiency_score()
        }
        
        # Calculate weighted composite score
        composite_score = sum(weights[metric] * score for metric, score in metrics.items())
        
        # Add composite score to metrics
        metrics['composite_score'] = composite_score
        metrics['weights_used'] = weights
        
        return metrics
    
    def calculate_target_progress_score(self):
        """
        Calculate target progress score based on:
        1. Objects already in target zone (highest value)
        2. Objects close to target zone (distance-weighted)
        3. Objects with good paths to target zone (path efficiency)
        
        :return: Normalized score (0.0 to 1.0)
        """
        if not self.cells_with_objects and not self.target_zone_cells:
            return 1.0  # Perfect score if no objects remain
        
        total_score = 0.0
        total_objects = 0
        max_distance = self.grid_size * 1.414  # Max possible distance (diagonal)
        
        # Score objects in target zone (maximum value: 1.0 per object)
        for cell in self.target_zone_cells:
            total_score += cell.num_objects * 1.0
            total_objects += cell.num_objects
        
        # Score objects outside target zone (distance and path weighted)
        for cell in self.cells_with_objects:
            # Distance component (0.0 to 0.8): closer = better
            distance_score = 0.8 * (1.0 - cell.distance_to_target / max_distance)
            
            # Path efficiency component (0.0 to 0.2): better paths = higher score
            if cell.best_path_target and cell.total_distance_target > 0:
                # Path efficiency: ratio of straight-line to actual path distance
                straight_distance = cell.distance_to_target
                path_efficiency = min(1.0, straight_distance / cell.total_distance_target)
                path_score = 0.2 * path_efficiency
            else:
                path_score = 0.0
            
            cell_score = distance_score + path_score
            total_score += cell.num_objects * cell_score
            total_objects += cell.num_objects
        
        # Normalize by total number of objects
        if total_objects > 0:
            return min(1.0, total_score / total_objects)
        else:
            return 1.0
    
    def calculate_highway_utilization_score(self):
        """
        Calculate highway utilization score based on:
        1. Objects positioned on high-heat highway cells
        2. Distribution of objects across efficient flow patterns
        
        :return: Normalized score (0.0 to 1.0)
        """
        if not self.cells_with_objects:
            return 1.0  # Perfect if no objects to optimize
        
        if not hasattr(self, 'highway_threshold') or self.highway_threshold <= 0:
            return 0.5  # Neutral score if no highway data
        
        total_weighted_objects = 0.0
        total_objects = 0
        max_heat = max((getattr(cell, 'heat_map', 0) for cell in self.all_cells), default=1.0)
        
        for cell in self.cells_with_objects:
            cell_heat = getattr(cell, 'heat_map', 0)
            # Normalize heat to 0-1 scale
            normalized_heat = cell_heat / max_heat if max_heat > 0 else 0.0
            
            # Weight objects by their position's heat value
            total_weighted_objects += cell.num_objects * normalized_heat
            total_objects += cell.num_objects
        
        if total_objects > 0:
            return total_weighted_objects / total_objects
        else:
            return 1.0
    
    def calculate_spatial_concentration_score(self):
        """
        Calculate spatial concentration score using convex hull area.
        Smaller convex hull = objects closer together = better score.
        
        :return: Normalized score (0.0 to 1.0)
        """
        if len(self.cells_with_objects) < 3:
            return 1.0  # Perfect score if too few objects for meaningful hull
        
        try:
            from shapely.geometry import MultiPoint
            
            # Get all object positions (weighted by object count)
            points = []
            for cell in self.cells_with_objects:
                # Add multiple points for cells with multiple objects
                for _ in range(min(cell.num_objects, 10)):  # Cap to avoid too many points
                    points.append((cell.x + 0.5, cell.y + 0.5))
            
            if len(points) < 3:
                return 1.0
            
            # Calculate convex hull area
            multipoint = MultiPoint(points)
            convex_hull = multipoint.convex_hull
            current_area = convex_hull.area
            
            # Calculate maximum possible area (entire grid)
            max_area = self.grid_size * self.grid_size
            
            # Score: smaller area = better score
            # Use exponential decay for more sensitive scoring
            import math
            area_ratio = current_area / max_area
            concentration_score = math.exp(-3 * area_ratio)  # Exponential decay
            
            return min(1.0, concentration_score)
            
        except Exception as e:
            print(f"Warning: Spatial concentration calculation failed: {e}")
            return 0.5  # Neutral score on error
    
    def calculate_flow_efficiency_score(self):
        """
        Calculate flow efficiency score based on:
        1. Heat map distribution quality
        2. Velocity field alignment
        3. Path utilization efficiency
        
        :return: Normalized score (0.0 to 1.0)
        """
        if not self.cells_with_objects:
            return 1.0
        
        total_score = 0.0
        scored_cells = 0
        
        for cell in self.cells_with_objects:
            cell_score = 0.0
            
            # Heat map component (0.0 to 0.4): higher heat = better positioning
            cell_heat = getattr(cell, 'heat_map', 0)
            max_heat = getattr(self, 'highway_threshold', 1.0)
            if max_heat > 0:
                heat_score = 0.4 * min(1.0, cell_heat / max_heat)
                cell_score += heat_score
            
            # Velocity alignment component (0.0 to 0.3): good velocity direction
            velocity = getattr(cell, 'velocity_target', (0, 0))
            velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2)
            if velocity_magnitude > 0:
                # Score based on velocity magnitude (higher = more decisive direction)
                velocity_score = 0.3 * min(1.0, velocity_magnitude)
                cell_score += velocity_score
            
            # Path quality component (0.0 to 0.3): efficient paths
            if cell.best_path_target and cell.distance_to_target > 0:
                path_length = cell.total_distance_target
                straight_distance = cell.distance_to_target
                if path_length > 0:
                    path_efficiency = min(1.0, straight_distance / path_length)
                    path_score = 0.3 * path_efficiency
                    cell_score += path_score
            
            total_score += cell_score
            scored_cells += 1
        
        if scored_cells > 0:
            return total_score / scored_cells
        else:
            return 1.0
    
    def get_environment_summary(self):
        """
        Get a summary of current environment state for debugging and analysis.
        
        :return: Dictionary with key environment statistics
        """
        summary = {
            'total_objects': sum(cell.num_objects for cell in self.cells_with_objects),
            'objects_in_target_zone': sum(cell.num_objects for cell in self.target_zone_cells),
            'active_cells': len(self.cells_with_objects),
            'target_zone_cells': len(self.target_zone_cells),
            'highway_threshold': getattr(self, 'highway_threshold', 0),
            'max_heat': max((getattr(cell, 'heat_map', 0) for cell in self.all_cells), default=0),
            'avg_distance_to_target': 0,
            'cells_on_highways': 0
        }
        
        # Calculate average distance to target
        if self.cells_with_objects:
            total_distance = sum(cell.distance_to_target * cell.num_objects for cell in self.cells_with_objects)
            total_objects = sum(cell.num_objects for cell in self.cells_with_objects)
            summary['avg_distance_to_target'] = total_distance / total_objects if total_objects > 0 else 0
            
            # Count cells on highways
            highway_threshold = getattr(self, 'highway_threshold', 0)
            if highway_threshold > 0:
                summary['cells_on_highways'] = sum(
                    1 for cell in self.cells_with_objects 
                    if getattr(cell, 'heat_map', 0) >= highway_threshold
                )
        
        return summary

