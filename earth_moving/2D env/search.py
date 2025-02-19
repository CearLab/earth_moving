import heapq

def a_star_search_target(start_cell):
    """
    Perform A* search to find the best path from the start cell to the target zone.
    Updates the best path, child pointers, and propagates the maximum objects collected.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, max_objects, min_distance) where:
             - best_path is a list of Cell objects representing the path.
             - max_objects is the total number of objects collected along the path.
             - min_distance is the total distance to the target.
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
        return [], start_cell.num_objects, start_cell.distance_to_target

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # If current cell already has a best path, merge it and stop further exploration
        if current_cell.best_path_target:
            stored_objects = current_cell.total_objects_target
            stored_distance = current_cell.total_distance_target

            # Merge stored path with our search path
            if collected_objects + stored_objects > max_objects or (
                collected_objects + stored_objects == max_objects and distance_so_far + stored_distance < best_distance
            ):
                max_objects = collected_objects + stored_objects
                best_distance = distance_so_far + stored_distance
                best_path = path + current_cell.best_path_target[1:]  # Merge paths

            continue  # Skip expanding further

        # If we've reached the target zone, store the best path found so far
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

            # If the child already has a path, use its total_objects as heuristic value
            h = child_cell.total_objects_target if child_cell.best_path_target else sum(
                neighbor["cell"].num_objects for neighbor in child_cell.visible_cells_target
            )

            # Compute c (Objects in child cell + total objects collected so far)
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
        else:
            next_cell = best_path[i + 1]
            current_cell.next_child_target = next_cell

        # Store max total objects path
        current_cell.total_objects_path_target = max(current_cell.total_objects_path_target, max_objects)

        # Ensure distance information is correctly stored
        if i < len(best_path) - 1 and (next_cell.x, next_cell.y) in current_cell.distance_to_children_target:
            current_cell.total_distance_target = (
                current_cell.distance_to_children_target[(next_cell.x, next_cell.y)] + next_cell.total_distance_target
            )
        else:
            current_cell.total_distance_target = best_distance  # Fallback if no recorded distance

    return best_path, max_objects, best_distance





def a_star_search_highway(start_cell):
    """
    Perform A* search to find the best path from the start cell to the highway.
    If a child already has a computed path, we reuse it.
    For multiple equal-object paths, we choose the shortest distance one.

    :param start_cell: The starting Cell object.
    :return: A tuple (best_path, max_objects, min_distance) where:
             - best_path is a list of Cell objects representing the path.
             - max_objects is the total number of objects collected along the path.
             - min_distance is the total distance to the highway.
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
        return [], start_cell.num_objects, start_cell.distance_to_target

    best_path = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # If current cell already has a best path, merge it and stop further expansion
        if current_cell.best_path_highway:
            stored_objects = current_cell.total_objects_highway
            stored_distance = current_cell.total_distance_highway

            # Merge stored path with our search path
            if collected_objects + stored_objects > max_objects or (
                    collected_objects + stored_objects == max_objects and distance_so_far + stored_distance < best_distance
            ):
                max_objects = collected_objects + stored_objects
                best_distance = distance_so_far + stored_distance
                best_path = path + current_cell.best_path_highway[1:]  # Merge paths

            continue  # Skip expanding further

        # If we've reached the highway, store the best path found so far
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

            # If the child already has a path, use its total_objects as heuristic value
            h = child_cell.total_objects_highway if child_cell.best_path_highway else sum(
                neighbor["cell"].num_objects for neighbor in child_cell.visible_cells_highway
            )

            # Compute c (Objects in child cell + total objects collected so far)
            c = child_cell.num_objects + collected_objects
            f = h + c

            tie_breaker += 1

            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell], c,
                 distance_so_far + current_cell.distance_to_children_highway[(child_cell.x, child_cell.y)]),
            )

    # Backpropagate to update best path information for all cells in the path
    for i in range(len(best_path) - 1, -1, -1):
        current_cell = best_path[i]
        if i == len(best_path) - 1:  # Last cell in the path
            current_cell.next_child_highway = None
        else:
            next_cell = best_path[i + 1]
            current_cell.next_child_highway = next_cell

        # Store max total objects path
        current_cell.total_objects_path_highway = max(current_cell.total_objects_path_highway, max_objects)

        # Ensure distance information is correctly stored
        if i < len(best_path) - 1 and (next_cell.x, next_cell.y) in current_cell.distance_to_children_highway:
            current_cell.total_distance_highway = (
                    current_cell.distance_to_children_highway[
                        (next_cell.x, next_cell.y)] + next_cell.total_distance_highway
            )
        else:
            current_cell.total_distance_highway = best_distance  # Fallback if no recorded distance

    return best_path, max_objects, best_distance


