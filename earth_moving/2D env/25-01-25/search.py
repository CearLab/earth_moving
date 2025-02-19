import heapq

def a_star_search_target(start_cell):
    """
    Perform A* search to find the best path from the start cell to the target zone,
    maximizing the number of objects collected.
    Updates the best path and related attributes for Target Zone Path-related calculations.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, total_objects) where:
             - best_path is a list of Cell objects representing the path.
             - total_objects is the total number of objects collected along the path.
    """
    # Priority queue for A* (stores tuples of (-f, distance_to_target, tie_breaker, current_cell, path_so_far, collected_objects, distance_so_far))
    open_set = []
    tie_breaker = 0  # Unique counter to ensure items in the queue are always comparable
    heapq.heappush(
        open_set,
        (-start_cell.num_objects, start_cell.distance_to_target, tie_breaker, start_cell, [start_cell],
         start_cell.num_objects, 0),
    )

    # Early exit if start_cell has no visible cells for the target
    if not start_cell.visible_cells_target:
        print(f"Warning: Cell ({start_cell.x}, {start_cell.y}) has no visible cells for target zone.")
        return [], start_cell.num_objects

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        # Pop the cell with the highest "f" value (negative for maximization)
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # Check if we've reached the target zone (no visible cells left to expand)
        if not current_cell.visible_cells_target:
            if collected_objects > max_objects or (
                    collected_objects == max_objects and distance_so_far < best_distance
            ):
                max_objects = collected_objects
                best_distance = distance_so_far
                best_path = path
            continue

        # Expand the current cell
        for child_info in current_cell.visible_cells_target:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_target:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) in cell ({current_cell.x}, {current_cell.y})")
                continue

            # Compute h and c
            h = len(child_cell.visible_cells_target)  # Heuristic: visible objects from this child
            c = child_cell.num_objects  # Cost: objects in the child cell
            f = h + c

            # Increment tie_breaker to ensure no two entries are compared based on cells alone
            tie_breaker += 1

            # Push the new state into the priority queue
            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell], collected_objects + c,
                 distance_so_far + current_cell.distance_to_children_target[(child_cell.x, child_cell.y)]),
            )

    # Backpropagate to update best path information for all cells in the path
    for i in range(len(best_path) - 1, -1, -1):
        current_cell = best_path[i]
        if i == len(best_path) - 1:  # Last cell in the path
            current_cell.best_child = None
            current_cell.total_objects_target = current_cell.num_objects
            current_cell.total_distance = current_cell.distance_to_target
        else:  # Update based on the next cell
            next_cell = best_path[i + 1]
            current_cell.best_child = next_cell
            current_cell.total_objects_target = current_cell.num_objects + next_cell.total_objects_target
            current_cell.total_distance = (
                    current_cell.distance_to_children_target[(next_cell.x, next_cell.y)] + next_cell.total_distance
            )

    return best_path, max_objects

def a_star_search_highway(start_cell):
    """
    Perform A* search to find the best path from the start cell to the highway,
    maximizing the number of objects collected.
    Updates the best path and related attributes for Highway Path-related calculations.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, total_objects) where:
             - best_path is a list of Cell objects representing the path.
             - total_objects is the total number of objects collected along the path.
    """
    # Priority queue for A* (stores tuples of (-f, distance_to_highway, tie_breaker, current_cell, path_so_far, collected_objects, distance_so_far))
    open_set = []
    tie_breaker = 0  # Unique counter to ensure items in the queue are always comparable
    heapq.heappush(
        open_set,
        (-start_cell.num_objects, start_cell.distance_to_target, tie_breaker, start_cell, [start_cell], start_cell.num_objects, 0),
    )

    # Early exit if start_cell has no visible cells for the highway
    if not start_cell.visible_cells_highway:
        print(f"Warning: Cell ({start_cell.x}, {start_cell.y}) has no visible cells for highway.")
        return [], start_cell.num_objects

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        # Pop the cell with the highest "f" value (negative for maximization)
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # Check if we've reached the highway (no visible cells left to expand)
        if not current_cell.visible_cells_highway:
            if collected_objects > max_objects or (
                collected_objects == max_objects and distance_so_far < best_distance
            ):
                max_objects = collected_objects
                best_distance = distance_so_far
                best_path = path
            continue

        # Expand the current cell
        for child_info in current_cell.visible_cells_highway:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_highway:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) in cell ({current_cell.x}, {current_cell.y})")
                continue

            # Compute h and c
            h = len(child_cell.visible_cells_highway)  # Heuristic: visible objects from this child
            c = child_cell.num_objects                # Cost: objects in the child cell
            f = h + c

            # Increment tie_breaker to ensure no two entries are compared based on cells alone
            tie_breaker += 1

            # Push the new state into the priority queue
            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell], collected_objects + c,
                 distance_so_far + current_cell.distance_to_children_highway[(child_cell.x, child_cell.y)]),
            )

    # Backpropagate to update best path information for all cells in the path
    for i in range(len(best_path) - 1, -1, -1):
        current_cell = best_path[i]
        if i == len(best_path) - 1:  # Last cell in the path
            current_cell.best_child = None
            current_cell.total_objects_highway = current_cell.num_objects
            current_cell.total_distance = current_cell.distance_to_target
        else:  # Update based on the next cell
            next_cell = best_path[i + 1]
            current_cell.best_child = next_cell
            current_cell.total_objects_highway = current_cell.num_objects + next_cell.total_objects_highway
            current_cell.total_distance = (
                current_cell.distance_to_children_highway[(next_cell.x, next_cell.y)] + next_cell.total_distance
            )

    return best_path, max_objects
