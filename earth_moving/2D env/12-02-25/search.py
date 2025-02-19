import heapq

def a_star_search_target(start_cell):
    """
    Perform A* search to find the best path from the start cell to the target zone.
    Updates the best path, child pointers, and propagates the maximum objects collected.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, max_objects) where:
             - best_path is a list of Cell objects representing the path.
             - max_objects is the total number of objects collected along the path.
    """
    open_set = []
    tie_breaker = 0
    heapq.heappush(
        open_set,
        (-start_cell.num_objects, start_cell.distance_to_target, tie_breaker, start_cell, [start_cell],
         start_cell.num_objects, 0),
    )

    if not start_cell.visible_cells_target:
        print(f"Warning: Cell ({start_cell.x}, {start_cell.y}) has no visible cells for target zone.")
        return [], start_cell.num_objects

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        if not current_cell.visible_cells_target:
            if collected_objects > max_objects or (
                collected_objects == max_objects and distance_so_far < best_distance
            ):
                max_objects = collected_objects
                best_distance = distance_so_far
                best_path = path
            continue

        for child_info in current_cell.visible_cells_target:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_target:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) "
                    f"in cell ({current_cell.x}, {current_cell.y})")
                continue

            h = sum(
                neighbor["cell"].num_objects for neighbor in child_cell.visible_cells_target)  # Sum all objects visible

            # **Compute c** (Objects in child cell + total objects collected so far)
            c = child_cell.num_objects + collected_objects
            f = h + c

            tie_breaker += 1

            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell], c,
                 distance_so_far + current_cell.distance_to_children_target[(child_cell.x, child_cell.y)]),
            )

    # Backpropagate to update best path information for all cells in the path
    for i in range(len(best_path) - 1, -1, -1):
        current_cell = best_path[i]
        if i == len(best_path) - 1:  # Last cell in the path
            current_cell.next_child_target = None
            current_cell.total_objects_target = current_cell.num_objects
            current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)
            current_cell.total_distance = current_cell.distance_to_target
        else:
            next_cell = best_path[i + 1]
            current_cell.next_child_target = next_cell
            current_cell.total_objects_target = current_cell.num_objects + next_cell.total_objects_target
            current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)
            current_cell.total_distance = (
                current_cell.distance_to_children_target[(next_cell.x, next_cell.y)] + next_cell.total_distance
            )

    return best_path, max_objects

def a_star_search_highway(start_cell):
    """
    Perform A* search to find the best path from the start cell to the highway.
    Updates the best path, child pointers, and propagates the maximum objects collected.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, max_objects) where:
             - best_path is a list of Cell objects representing the path.
             - max_objects is the total number of objects collected along the path.
    """
    open_set = []
    tie_breaker = 0
    heapq.heappush(
        open_set,
        (-start_cell.num_objects, start_cell.distance_to_target, tie_breaker, start_cell, [start_cell],
         start_cell.num_objects, 0),
    )

    if not start_cell.visible_cells_highway:
        print(f"Warning: Cell ({start_cell.x}, {start_cell.y}) has no visible cells for highway.")
        return [], start_cell.num_objects

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        if not current_cell.visible_cells_highway:
            if collected_objects > max_objects or (
                collected_objects == max_objects and distance_so_far < best_distance
            ):
                max_objects = collected_objects
                best_distance = distance_so_far
                best_path = path
            continue

        for child_info in current_cell.visible_cells_highway:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_highway:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) "
                    f"in cell ({current_cell.x}, {current_cell.y})")
                continue

            h = len(child_cell.visible_cells_highway)  # Heuristic
            c = child_cell.num_objects
            f = h + c

            tie_breaker += 1

            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell], collected_objects + c,
                 distance_so_far + current_cell.distance_to_children_highway[(child_cell.x, child_cell.y)]),
            )

    # Backpropagate to update best path information for all cells in the path
    for i in range(len(best_path) - 1, -1, -1):
        current_cell = best_path[i]
        if i == len(best_path) - 1:  # Last cell in the path
            current_cell.next_child_highway = None
            current_cell.total_objects_highway = current_cell.num_objects
            current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)
            current_cell.total_distance = current_cell.distance_to_target
        else:
            next_cell = best_path[i + 1]
            current_cell.next_child_highway = next_cell
            current_cell.total_objects_highway = current_cell.num_objects + next_cell.total_objects_highway
            current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)
            current_cell.total_distance = (
                current_cell.distance_to_children_highway[(next_cell.x, next_cell.y)] + next_cell.total_distance
            )

    return best_path, max_objects

