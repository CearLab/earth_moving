# --- A* (TARGET) -----------------------------------------------------------
import heapq, math

def a_star_search_target(start_cell, alternative_paths_threshold=2, target_zone=None,
                         max_path_length_factor=None, env=None, use_suffix_stitching=True):
    # Straight-line cap (optional)
    if start_cell.x==21 and start_cell.y==1:
        print("test")
    max_allowed_distance = float('inf')
    if max_path_length_factor and max_path_length_factor > 0:
        straight = getattr(start_cell, "distance_to_target", float("inf"))
        max_allowed_distance = max_path_length_factor * straight

    # Heap entries: (-f, tie_dist, tie_id, curr, path, c_so_far, dist_so_far)
    open_set, tie_id = [], 0
    heapq.heappush(open_set, (-start_cell.num_objects, start_cell.distance_to_target,
                              tie_id, start_cell, [start_cell], start_cell.num_objects, 0.0))

    # Best “collected so far” seen for each cell (for relaxation)
    best_c = {(start_cell.x, start_cell.y): start_cell.num_objects}

    best_paths, best_objects, best_distance = [], 0, float("inf")

    while open_set:
        _, _, _, current, path, c_so_far, d_so_far = heapq.heappop(open_set)

        # Early-stop stitching if we pop a solved node (spillage OFF only, and suffix stitching enabled)
        if (use_suffix_stitching and env and not getattr(env, "use_spillage_model", True) and 
            getattr(current, "solved_target", False)):
            # Stitch prefix + current.best_path_target (skip duplicate current)
            tail = current.best_path_target[1:] if current.best_path_target and current.best_path_target[0] is current else current.best_path_target
            full_path = path + tail
            
            # Objects: replace current's local count with exact tail yield to avoid double count
            total_objects = (c_so_far - current.num_objects) + current.total_objects_target
            
            # Distance: add remaining distance along current.best_path_target
            rem_dist = 0.0
            for u, v in zip(current.best_path_target, current.best_path_target[1:]):
                rem_dist += current.distance_to_children_target.get((v.x, v.y), math.hypot(v.x - u.x, v.y - u.y))
            
            # Check if stitched path respects distance constraint
            total_distance = d_so_far + rem_dist
            if total_distance <= max_allowed_distance:
                best_paths.append({"path": full_path, "objects": total_objects, "distance": total_distance})
                continue  # Early-stop: use optimized stitched path
            # If stitched path violates constraint, fall back to normal expansion to find shorter alternative

        # Goal: a cell with no successors toward the target (boundary or fallback)
        if not getattr(current, "visible_cells_target", []):
            if c_so_far >= best_objects - alternative_paths_threshold:
                best_paths.append({"path": path, "objects": c_so_far, "distance": d_so_far})
                if c_so_far > best_objects or (c_so_far == best_objects and d_so_far < best_distance):
                    best_objects, best_distance = c_so_far, d_so_far
            continue

        # Expand successors
        for info in current.visible_cells_target:
            child = info["cell"]
            # edge length (precomputed if available)
            edge_d = current.distance_to_children_target.get((child.x, child.y),
                        math.hypot(child.x - current.x, child.y - current.y))
            new_d = d_so_far + edge_d
            if new_d > max_allowed_distance:
                continue

            # Smart heuristic: use h_resolved for solved cells, h_vis for unsolved (spillage OFF only, with suffix stitching)
            if (use_suffix_stitching and env and not getattr(env, "use_spillage_model", True) and 
                getattr(child, "solved_target", False)):
                h = child.h_resolved_target  # Use exact heuristic for solved cells
            else:
                h = getattr(child, "h_vis_target", 0)  # Use optimistic heuristic for unsolved cells
            child_objs = child.num_objects
            if target_zone is not None:
                from shapely.geometry import Point
                if target_zone.contains(Point(child.x + 0.5, child.y + 0.5)):
                    child_objs = 0
            new_c = c_so_far + child_objs
            f = h + new_c

            key = (child.x, child.y)
            # Relaxation: only queue if we improved collected objects to this cell
            if new_c <= best_c.get(key, -1):
                continue
            best_c[key] = new_c

            tie_id += 1
            heapq.heappush(open_set, (-f, child.distance_to_target, tie_id,
                                      child, path + [child], new_c, new_d))

    # Sort so index 0 is always the best (max objects, then min distance)
    best_paths.sort(key=lambda p: (-p["objects"], p["distance"]))
    return best_paths

# --- A* (HIGHWAY) ----------------------------------------------------------
def a_star_search_highway(start_cell, target_cell, highway_threshold=None,
                          max_path_length_factor=None, use_suffix_stitching=True):
    if target_cell is None:
        return []

    # Straight-line cap (optional)
    max_allowed_distance = float('inf')
    if max_path_length_factor and max_path_length_factor > 0:
        straight = math.hypot(target_cell.x - start_cell.x, target_cell.y - start_cell.y)
        max_allowed_distance = max_path_length_factor * straight

    open_set, tie_id = [], 0
    # Use actual Euclidean distance to target for meaningful tiebreaking
    start_distance_to_target = math.hypot(target_cell.x - start_cell.x, target_cell.y - start_cell.y)
    heapq.heappush(open_set, (-start_cell.num_objects, start_distance_to_target,
                              tie_id, start_cell, [start_cell], start_cell.num_objects, 0.0))
    best_c = {(start_cell.x, start_cell.y): start_cell.num_objects}
    best_paths, best_objects, best_distance = [], 0, float("inf")

    # Use the precomputed, cone-filtered successors toward this specific target
    while open_set:
        _, _, _, current, path, c_so_far, d_so_far = heapq.heappop(open_set)

        # Goal: reached the chosen highway target (or no successors toward it)
        if (current.x, current.y) == (target_cell.x, target_cell.y) or \
           not getattr(current, "visible_cells_highway", []):
            # Check if goal path respects distance constraint
            if d_so_far <= max_allowed_distance:
                best_paths.append({"path": path, "objects": c_so_far, "distance": d_so_far})
                if c_so_far > best_objects or (c_so_far == best_objects and d_so_far < best_distance):
                    best_objects, best_distance = c_so_far, d_so_far
            # keep collecting equal-object/shorter variants; break is optional
            continue

        for info in current.visible_cells_highway:
            child = info["cell"]
            edge_d = current.distance_to_children_highway.get((child.x, child.y),
                        math.hypot(child.x - current.x, child.y - current.y))
            new_d = d_so_far + edge_d
            if new_d > max_allowed_distance:
                continue

            # f = h + c (same rule)
            h = sum(n["cell"].num_objects for n in getattr(child, "visible_cells_highway", []))
            new_c = c_so_far + child.num_objects
            f = h + new_c

            key = (child.x, child.y)
            if new_c <= best_c.get(key, -1):
                continue
            best_c[key] = new_c

            # Use actual Euclidean distance to target for meaningful tiebreaking
            child_distance_to_target = math.hypot(target_cell.x - child.x, target_cell.y - child.y)
            tie_id += 1
            heapq.heappush(open_set, (-f, child_distance_to_target,
                                      tie_id, child, path + [child], new_c, new_d))

    best_paths.sort(key=lambda p: (-p["objects"], p["distance"]))
    return best_paths
