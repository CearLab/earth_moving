import heapq


def a_star_search_target(start_cell, alternative_paths_threshold=2):
    """
    Perform A* search to find multiple best paths from start_cell to the target zone.
    Allows paths that collect fewer objects but are close to optimal.

    :param start_cell: The starting Cell object.
    :param alternative_paths_threshold: The acceptable difference in objects to consider a path.
    :return: List of best paths with their object count and total distance.
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
        return []

    best_paths = []
    best_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # **If we reach the target zone, store this path**
        if not current_cell.visible_cells_target:
            if collected_objects >= best_objects - alternative_paths_threshold:
                best_paths.append({
                    "path": path,
                    "objects": collected_objects,
                    "distance": distance_so_far
                })
                best_objects = max(best_objects, collected_objects)
                best_distance = min(best_distance, distance_so_far)
            continue  # Don't expand further

        # Expand the current cell
        for child_info in current_cell.visible_cells_target:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_target:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) in cell ({current_cell.x}, {current_cell.y})")
                continue

            # **Compute h and c**
            h = sum(neighbor["cell"].num_objects for neighbor in child_cell.visible_cells_target)
            c = child_cell.num_objects + collected_objects  # Accumulate total objects
            f = h + c

            tie_breaker += 1
            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_target, tie_breaker, child_cell, path + [child_cell],
                 c, distance_so_far + current_cell.distance_to_children_target[(child_cell.x, child_cell.y)]),
            )

    return best_paths


import heapq

def a_star_search_highway(start_cell):
    """
    Perform A* search to find multiple best paths from the start cell to the highway.
    Returns paths maximizing objects while minimizing distance.

    :param start_cell: The starting Cell object.
    :return: A list of dictionaries, each representing a path with:
             - "path": The list of Cell objects in the path.
             - "objects": The total number of objects collected.
             - "distance": The total distance to the highway.
    """
    open_set = []
    tie_breaker = 0
    heapq.heappush(
        open_set,
        (-start_cell.num_objects, start_cell.distance_to_highway, tie_breaker, start_cell, [start_cell],
         start_cell.num_objects, 0),
    )

    if not start_cell.visible_cells_highway:
        print(f"Warning: Cell ({start_cell.x}, {start_cell.y}) has no visible cells for highway.")
        return []

    best_paths = []
    max_objects = 0
    best_distance = float("inf")

    while open_set:
        _, _, _, current_cell, path, collected_objects, distance_so_far = heapq.heappop(open_set)

        # If we've reached the highway, store this path
        if not current_cell.visible_cells_highway:
            if collected_objects > max_objects or (
                    collected_objects == max_objects and distance_so_far < best_distance):
                max_objects = collected_objects
                best_distance = distance_so_far
                best_paths = [{"path": path, "objects": collected_objects, "distance": distance_so_far}]
            elif collected_objects == max_objects and distance_so_far == best_distance:
                best_paths.append({"path": path, "objects": collected_objects, "distance": distance_so_far})
            continue

        for child_info in current_cell.visible_cells_highway:
            child_cell = child_info["cell"]
            if (child_cell.x, child_cell.y) not in current_cell.distance_to_children_highway:
                print(
                    f"Error: Missing distance for child cell ({child_cell.x}, {child_cell.y}) "
                    f"in cell ({current_cell.x}, {current_cell.y})")
                continue

            # Compute h (future potential) and c (current collected objects)
            h = sum(neighbor["cell"].num_objects for neighbor in child_cell.visible_cells_highway)
            c = child_cell.num_objects + collected_objects
            f = h + c

            tie_breaker += 1
            heapq.heappush(
                open_set,
                (-f, child_cell.distance_to_highway, tie_breaker, child_cell, path + [child_cell], c,
                 distance_so_far + current_cell.distance_to_children_highway[(child_cell.x, child_cell.y)]),
            )

    return best_paths  # Return multiple best paths for further evaluation



