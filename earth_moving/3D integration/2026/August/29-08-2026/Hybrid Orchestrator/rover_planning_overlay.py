"""Persistent, rover-specific planning overlays built from one canonical heatmap."""
from __future__ import annotations
from dataclasses import dataclass, replace
from functools import lru_cache
import heapq
import math

from pebble_profiles import DEFAULT_MATERIAL_VALUE_MODE, MASS_MODE, normalize_material_mode
from shapely.geometry import MultiPoint, box
from shapely.ops import unary_union

TARGET_PATH_MODES = frozenset({"material_aware", "straight_nearest"})
TARGET_CANDIDATE_VALUE_MODES = frozenset({"material_aware", "source_only"})
TARGET_SOURCE_MODES = frozenset({"all", "root", "convex_hull"})
CAPACITY_SCORING_MODES = frozenset({"profile_cap", "uncapped_corridor"})
CAPACITY_SCORING_MODE = "profile_cap"


def normalize_target_path_mode(value):
    mode = str(value or "material_aware").strip().lower()
    if mode not in TARGET_PATH_MODES:
        raise ValueError(f"target path mode must be one of {sorted(TARGET_PATH_MODES)}")
    return mode


def normalize_target_candidate_value_mode(value):
    mode = str(value or "material_aware").strip().lower()
    if mode not in TARGET_CANDIDATE_VALUE_MODES:
        raise ValueError(
            "target candidate value mode must be one of "
            f"{sorted(TARGET_CANDIDATE_VALUE_MODES)}"
        )
    return mode


def normalize_target_source_mode(value):
    mode = str(value or "all").strip().lower()
    if mode not in TARGET_SOURCE_MODES:
        raise ValueError(
            f"target source mode must be one of {sorted(TARGET_SOURCE_MODES)}"
        )
    return mode


def normalize_capacity_scoring_mode(value):
    mode = str(value or "profile_cap").strip().lower()
    if mode not in CAPACITY_SCORING_MODES:
        raise ValueError(
            f"capacity scoring mode must be one of {sorted(CAPACITY_SCORING_MODES)}"
        )
    return mode

@dataclass(frozen=True)
class OverlayCell:
    x: int
    y: int

@dataclass(frozen=True)
class OverlayPath:
    cell_size: float
    cells: tuple
    world_path: tuple
    swept_canonical_cells: frozenset

@dataclass(frozen=True)
class OverlaySourceCell:
    key: tuple
    world_center: tuple
    canonical_cells: tuple
    object_count: float
    material_mass: float
    planning_quantity: float
    heat: float

@dataclass(frozen=True)
class OverlayTaskCandidate:
    source: OverlaySourceCell
    task_type: str
    representative_cell: object
    path_info: dict
    reserved_path_cells: frozenset
    reserved_object_cells: frozenset
    expected_collected: float
    expected_delivered: float
    expected_spillage: float
    expected_collected_objects: float = 0.0
    expected_collected_mass: float = 0.0
    is_root_source: bool = True
    is_convex_hull_source: bool = False
    upstream_source_keys: tuple = ()

@dataclass(frozen=True)
class ProfileOverlaySnapshot:
    map_epoch: int
    profile_key: tuple
    source_cell_size: float
    path_cell_size: float
    sources: tuple
    candidates: tuple
    target_path_mode: str = "material_aware"
    target_candidate_value_mode: str = "material_aware"
    convex_hull_source_cell_keys: frozenset = frozenset()
    convex_hull_definition: str = "outside_material_plus_target_boundary"
    generation_failures: tuple = ()

@lru_cache(maxsize=64)
def _target_lookup_grid(zone, env_radius, cell_size, grid_size):
    """Precompute exact target membership and distance once per overlay grid."""
    env_radius = float(env_radius)
    cell_size = float(cell_size)
    grid_size = int(grid_size)
    inside = bytearray(grid_size * grid_size)
    distances = [0.0] * (grid_size * grid_size)
    for iy in range(grid_size):
        wy = env_radius - iy * cell_size
        row_offset = iy * grid_size
        for ix in range(grid_size):
            wx = -env_radius + ix * cell_size
            signed_distance = zone.signed_distance_world(wx, wy)
            index = row_offset + ix
            inside[index] = 1 if signed_distance <= 1e-12 else 0
            distances[index] = max(0.0, float(signed_distance))
    return bytes(inside), tuple(distances)


def target_lookup_cache_info():
    """Expose cache statistics for diagnostics and regression tests."""
    return _target_lookup_grid.cache_info()


def _cell_key(cell):
    return int(cell.x), int(cell.y)

def _point_segment_distance(point, a, b):
    px, py = point; ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def _distance_to_polyline(point, points):
    if not points:
        return 0.0
    if len(points) < 2:
        return math.hypot(point[0] - points[0][0], point[1] - points[0][1])
    return min(_point_segment_distance(point, a, b) for a, b in zip(points, points[1:]))

def _reconstruct(came_from, current):
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return cells

def _swept_canonical_cells(world_path, converter, radius):
    if not world_path:
        return frozenset()
    canonical_size = float(converter.cell_size)
    sample_step = max(0.01, canonical_size * 0.5)
    samples = [world_path[0]]
    for a, b in zip(world_path, world_path[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        count = max(1, int(math.ceil(length / sample_step)))
        for index in range(1, count + 1):
            t = index / count
            samples.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    reach = int(math.ceil(radius / canonical_size)) + 1
    result = set()
    for wx, wy in samples:
        cx, cy = converter.convert_3d_to_2d(wx, wy)
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                key = (cx + dx, cy + dy)
                if not (0 <= key[0] < converter.grid_size and 0 <= key[1] < converter.grid_size):
                    continue
                cell_world = converter.convert_2d_to_3d(key[0], key[1])
                if converter.is_in_target_zone(key[0], key[1]):
                    continue
                if math.hypot(cell_world[0] - wx, cell_world[1] - wy) <= radius + canonical_size * 0.75:
                    result.add(key)
    return frozenset(result)

def world_path_is_sane(world_path, env_radius, tolerance=0.25):
    """Reject corrupt coordinates before they can expand a controller world frame."""
    if world_path is None or len(world_path) < 2:
        return False
    radius_limit = float(env_radius) + max(0.0, float(tolerance))
    for point in world_path:
        if point is None or len(point) < 2:
            return False
        x, y = float(point[0]), float(point[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if math.hypot(x, y) > radius_limit:
            return False
    return True


def bounded_approach_gate(path_start, path_direction, desired_back, max_radius):
    """Keep the behind-object approach gate inside the usable planning envelope."""
    sx, sy = float(path_start[0]), float(path_start[1])
    dx, dy = float(path_direction[0]), float(path_direction[1])
    norm = math.hypot(dx, dy)
    if not all(math.isfinite(v) for v in (sx, sy, dx, dy, desired_back, max_radius)):
        raise ValueError("approach gate received non-finite geometry")
    if norm <= 1e-12 or desired_back <= 0.0:
        return (sx, sy), 0.0, False
    dx, dy = dx / norm, dy / norm
    desired_back = float(desired_back)
    max_radius = max(0.0, float(max_radius))

    def point(back):
        return sx - back * dx, sy - back * dy

    desired = point(desired_back)
    if math.hypot(*desired) <= max_radius:
        return desired, desired_back, False
    # If even the source lies outside the envelope, do not drive farther out.
    if math.hypot(sx, sy) > max_radius:
        return (sx, sy), 0.0, True
    lo, hi = 0.0, desired_back
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if math.hypot(*point(mid)) <= max_radius:
            lo = mid
        else:
            hi = mid
    return point(lo), lo, True


def build_straight_target_overlay_path(
        env_2d, converter, start_world, cell_size, reservation_radius):
    """Return one straight push from a source to its nearest target boundary.

    This is an intentionally simple comparison path.  It bypasses heat-map and
    multi-object route optimization while preserving the same rover overlay,
    swept-corridor reservation, execution, and physics used by the full method.
    """
    start = (float(start_world[0]), float(start_world[1]))
    zone = converter.target_zone_spec
    if zone.contains_world(*start):
        return None
    boundary = zone.closest_boundary_point_world(*start)
    dx, dy = float(boundary[0]) - start[0], float(boundary[1]) - start[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return None
    ux, uy = dx / distance, dy / distance

    # End just inside the target.  The scheduled executor may still append its
    # configured push extension, exactly as it does for material-aware paths.
    inside_step = max(0.02, min(0.08, 0.5 * float(cell_size)))
    goal = (float(boundary[0]) + ux * inside_step,
            float(boundary[1]) + uy * inside_step)
    if not zone.contains_world(*goal):
        goal = (float(boundary[0]), float(boundary[1]))
    world_path = (start, goal)
    if not world_path_is_sane(
            world_path, converter.env_radius,
            tolerance=max(0.25, float(cell_size) * 2.0)):
        return None

    env_radius = float(converter.env_radius)
    grid_size = max(3, int(math.ceil(2.0 * env_radius / float(cell_size))))
    if grid_size % 2 == 0:
        grid_size += 1

    def world_to_overlay(point):
        return (
            max(0, min(grid_size - 1, int(round((point[0] + env_radius) / cell_size)))),
            max(0, min(grid_size - 1, int(round((env_radius - point[1]) / cell_size)))),
        )

    sample_count = max(1, int(math.ceil(math.hypot(
        goal[0] - start[0], goal[1] - start[1]) / max(float(cell_size), 1e-9))))
    cells = []
    for index in range(sample_count + 1):
        fraction = index / sample_count
        cell = OverlayCell(*world_to_overlay((
            start[0] + fraction * (goal[0] - start[0]),
            start[1] + fraction * (goal[1] - start[1]),
        )))
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return OverlayPath(
        float(cell_size), tuple(cells), world_path,
        _swept_canonical_cells(world_path, converter, reservation_radius),
    )

def build_rover_overlay_path(env_2d, converter, canonical_path, cell_size, shovel_width,
                             reservation_radius, goal_mode="target", start_world=None):
    """Generate rover-cell A* guided by the canonical heatmap/path fabric."""
    if not canonical_path:
        return None
    canonical_world = [converter.convert_2d_to_3d(cell.x, cell.y) for cell in canonical_path]
    if start_world is not None:
        canonical_world[0] = (float(start_world[0]), float(start_world[1]))
    env_radius = float(converter.env_radius)
    grid_size = max(3, int(math.ceil(2.0 * env_radius / cell_size)))
    if grid_size % 2 == 0:
        grid_size += 1

    target_inside = None
    target_distances = None
    if goal_mode != "highway":
        target_inside, target_distances = _target_lookup_grid(
            converter.target_zone_spec, env_radius, float(cell_size), grid_size)

    def clamp(value):
        return max(0, min(grid_size - 1, value))
    def world_to_overlay(point):
        return (clamp(int(round((point[0] + env_radius) / cell_size))),
                clamp(int(round((env_radius - point[1]) / cell_size))))
    def overlay_to_world(cell):
        return (-env_radius + cell[0] * cell_size, env_radius - cell[1] * cell_size)

    start = world_to_overlay(canonical_world[0])
    highway_goal = world_to_overlay(canonical_world[-1])
    heat_by_cell = {(int(c.x), int(c.y)): float(getattr(c, "heat_map", 0.0))
                    for c in getattr(env_2d, "all_cells", [])}
    max_heat = max(heat_by_cell.values(), default=0.0)

    def heat_at(world):
        key = converter.convert_3d_to_2d(world[0], world[1])
        return heat_by_cell.get(key, 0.0) / max_heat if max_heat > 1e-12 else 0.0
    def heuristic(cell):
        if goal_mode == "highway":
            wx, wy = overlay_to_world(cell)
            gx, gy = overlay_to_world(highway_goal)
            return math.hypot(wx - gx, wy - gy)
        return target_distances[cell[1] * grid_size + cell[0]]
    def is_goal(cell):
        if goal_mode == "highway":
            return cell == highway_goal
        return bool(target_inside[cell[1] * grid_size + cell[0]])
    frontier = [(heuristic(start), 0.0, start)]
    came_from = {}
    best_cost = {start: 0.0}
    goal = None
    neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    while frontier:
        _, cost_so_far, current = heapq.heappop(frontier)
        if cost_so_far > best_cost.get(current, float("inf")) + 1e-12:
            continue
        if is_goal(current):
            goal = current
            break
        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < grid_size and 0 <= nxt[1] < grid_size):
                continue
            world = overlay_to_world(nxt)
            step = cell_size * (math.sqrt(2.0) if dx and dy else 1.0)
            heat_penalty = 0.35 * (1.0 - heat_at(world))
            fabric_penalty = 0.12 * _distance_to_polyline(world, canonical_world) / max(cell_size, 1e-9)
            new_cost = cost_so_far + step * (1.0 + heat_penalty + fabric_penalty)
            if new_cost + 1e-12 < best_cost.get(nxt, float("inf")):
                best_cost[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(frontier, (new_cost + heuristic(nxt), new_cost, nxt))
    if goal is None:
        return None
    raw_cells = _reconstruct(came_from, goal)
    world_path = [overlay_to_world(cell) for cell in raw_cells]
    if len(world_path) == 1 and goal_mode != "highway":
        # A coarse rover overlay cell can have its center inside the target
        # even though the exact source point is still outside.  Preserve both
        # points so this remains an executable push rather than a degenerate
        # one-point path that gets silently discarded.
        interior = world_path[0]
        world_path = [canonical_world[0], interior]
    else:
        world_path[0] = canonical_world[0]
    if goal_mode == "highway":
        world_path[-1] = canonical_world[-1]
    if not world_path_is_sane(world_path, env_radius, tolerance=max(0.25, cell_size * 2.0)):
        return None
    return OverlayPath(float(cell_size), tuple(OverlayCell(*cell) for cell in raw_cells),
                       tuple(world_path), _swept_canonical_cells(world_path, converter, reservation_radius))

def _build_sources(env_2d, converter, source_cell_size):
    groups = {}
    env_radius = float(converter.env_radius)
    for cell in getattr(env_2d, "cells_with_objects", []):
        count = float(getattr(cell, "num_objects", 0.0))
        if count <= 0.0:
            continue
        wx, wy = converter.convert_2d_to_3d(cell.x, cell.y)
        key = (int(math.floor((wx + env_radius) / source_cell_size)),
               int(math.floor((env_radius - wy) / source_cell_size)))
        groups.setdefault(key, []).append((cell, wx, wy))
    sources = []
    for key, members in sorted(groups.items()):
        planning_total = sum(float(getattr(cell, "num_objects", 0.0)) for cell, _, _ in members)
        if planning_total <= 0.0:
            continue
        physical_count = sum(float(getattr(cell, "physical_object_count", getattr(cell, "num_objects", 0.0))) for cell, _, _ in members)
        material_mass = sum(float(getattr(cell, "material_mass", getattr(cell, "num_objects", 0.0))) for cell, _, _ in members)
        wx = sum(x * float(getattr(cell, "num_objects", 0.0)) for cell, x, _ in members) / planning_total
        wy = sum(y * float(getattr(cell, "num_objects", 0.0)) for cell, _, y in members) / planning_total
        heat = sum(float(getattr(cell, "heat_map", 0.0)) for cell, _, _ in members)
        sources.append(OverlaySourceCell(
            key, (wx, wy), tuple(cell for cell, _, _ in members),
            physical_count, material_mass, planning_total, heat))
    return tuple(sources)


def convex_hull_source_cell_keys(env_2d, converter):
    """Return occupied outside cells exposed on the live material/target hull.

    The hull is geometric rather than graph based.  It is built from the
    centers of all occupied cells outside the target together with the exact
    target-zone polygon.  A source cell is exposed when its physical grid
    footprint intersects the boundary of that combined convex hull.  Testing
    the footprint (instead of exact center equality) makes the result stable
    on a discrete grid and for non-circular target zones.
    """
    occupied = []
    for cell in getattr(env_2d, "cells_with_objects", ()):
        if float(getattr(cell, "num_objects", 0.0)) <= 0.0:
            continue
        if converter.is_in_target_zone(int(cell.x), int(cell.y)):
            continue
        wx, wy = converter.convert_2d_to_3d(cell.x, cell.y)
        occupied.append((cell, float(wx), float(wy)))
    if not occupied:
        return frozenset()

    target_geometry = converter.target_zone_spec.geometry
    material_points = MultiPoint([(wx, wy) for _, wx, wy in occupied])
    combined_hull = unary_union((target_geometry, material_points)).convex_hull
    if combined_hull.is_empty:
        return frozenset()

    canonical_size = float(converter.cell_size)
    half = 0.5 * canonical_size
    # A small numerical buffer prevents floating-point boundary misses without
    # turning the geometric rule into an arbitrary distance threshold.
    hull_boundary = combined_hull.boundary.buffer(max(1e-9, canonical_size * 1e-6))
    exposed = {
        (int(cell.x), int(cell.y))
        for cell, wx, wy in occupied
        if box(wx - half, wy - half, wx + half, wy + half).intersects(hull_boundary)
    }
    return frozenset(exposed)

def _path_keys(path_info):
    keys = {_cell_key(cell) for cell in path_info.get("path", []) or []}
    for value in (path_info.get("impacted_cells", {}) or {}).keys():
        if hasattr(value, "x"):
            keys.add(_cell_key(value))
        elif isinstance(value, (tuple, list)) and len(value) >= 2:
            keys.add((int(value[0]), int(value[1])))
    return keys

def _best_guide(env_2d, source, task_type):
    choices = []
    for member in source.canonical_cells:
        paths = env_2d.get_path_for_preview(member, task_type)
        if not paths:
            continue
        info = paths[0]
        path = info.get("path", []) or []
        if not path:
            continue
        raw = float(info.get("raw_objects", getattr(member, "num_objects", 0.0)))
        delivered = float(info.get("objects", 0.0))
        canonical_info = dict(info)
        canonical_info.setdefault("guide_mode", "canonical")
        choices.append((delivered, raw, -float(info.get("distance", len(path))), member, canonical_info))
    if not choices and task_type == "target":
        # Last-resort guide for an occupied source that the legacy visibility
        # fabric failed to connect.  The rover-specific overlay still plans the
        # actual route; this guide merely guarantees a valid source-to-target
        # direction without inventing another heatmap.
        terminal_finder = getattr(env_2d, "nearest_target_terminal_cell", None)
        for member in source.canonical_cells:
            terminal = terminal_finder(member) if callable(terminal_finder) else None
            if terminal is None or terminal is member:
                continue
            raw = float(getattr(member, "num_objects", 0.0))
            distance = math.hypot(terminal.x - member.x, terminal.y - member.y)
            info = {
                "path": [member, terminal],
                "objects": raw,
                "raw_objects": raw,
                "distance": distance,
                "impacted_cells": {},
                "guide_mode": "direct_target_orphan_recovery",
            }
            choices.append((raw, raw, -distance, member, info))
    return max(choices, default=None, key=lambda value: value[:3])


def _mark_target_source_candidates(candidates, convex_hull_keys):
    """Annotate target candidates with graph-root and geometric-hull status.

    Source B is upstream of source A when B is farther along the push graph
    and B's rover-specific swept collection corridor includes one or more
    canonical object cells belonging to A. In that case, starting at B can
    collect A on the way to the target, so A is not a root source.
    """
    target_candidates = [
        candidate for candidate in candidates if candidate.task_type == "target"
    ]
    source_cells = {
        candidate.source.key: {
            _cell_key(cell) for cell in candidate.source.canonical_cells
        }
        for candidate in target_candidates
    }
    path_distance = {
        candidate.source.key: float(candidate.path_info.get("distance", 0.0))
        for candidate in target_candidates
    }
    upstream_by_source = {}
    for candidate in target_candidates:
        current_cells = source_cells.get(candidate.source.key, set())
        current_distance = path_distance.get(candidate.source.key, 0.0)
        upstream_keys = []
        for other in target_candidates:
            if other.source.key == candidate.source.key:
                continue
            if path_distance.get(other.source.key, 0.0) <= current_distance + 1e-9:
                continue
            if current_cells.intersection(other.reserved_object_cells):
                upstream_keys.append(other.source.key)
        upstream_by_source[candidate.source.key] = tuple(sorted(set(upstream_keys)))

    annotated = []
    for candidate in candidates:
        if candidate.task_type != "target":
            annotated.append(candidate)
            continue
        upstream_keys = upstream_by_source.get(candidate.source.key, ())
        is_root = not upstream_keys
        source_keys = {
            _cell_key(cell) for cell in candidate.source.canonical_cells
        }
        is_convex_hull = bool(source_keys.intersection(convex_hull_keys))
        path_info = dict(candidate.path_info)
        path_info.update({
            "is_root_source": is_root,
            "is_convex_hull_source": is_convex_hull,
            "upstream_source_keys": list(upstream_keys),
            "root_source_rule": "no_upstream_target_corridor_collects_source",
            "convex_hull_source_rule": (
                "occupied_cell_footprint_intersects_hull_of_"
                "outside_material_plus_target_boundary"
            ),
        })
        annotated.append(replace(
            candidate,
            path_info=path_info,
            is_root_source=is_root,
            is_convex_hull_source=is_convex_hull,
            upstream_source_keys=upstream_keys,
        ))
    return tuple(annotated)


def build_profile_overlay_snapshot(
        map_epoch, env_2d, converter, rover_type,
        material_value_mode=DEFAULT_MATERIAL_VALUE_MODE,
        target_path_mode="material_aware",
        target_candidate_value_mode="material_aware"):
    """Build all source cells and physically-supported task candidates once."""
    geometry = rover_type.geometry
    material_value_mode = normalize_material_mode(material_value_mode)
    target_path_mode = normalize_target_path_mode(target_path_mode)
    target_candidate_value_mode = normalize_target_candidate_value_mode(
        target_candidate_value_mode)
    capacity_objects = float(rover_type.capabilities.capacity_objects)
    capacity_mass = float(rover_type.capabilities.capacity_mass)
    capacity = capacity_mass if material_value_mode == MASS_MODE else capacity_objects
    sources = _build_sources(env_2d, converter, geometry.shovel_width)
    convex_hull_keys = convex_hull_source_cell_keys(env_2d, converter)
    candidates = []
    generation_failures = {}

    def record_generation(name):
        generation_failures[name] = generation_failures.get(name, 0) + 1
    object_by_key = {_cell_key(cell): float(getattr(cell, "num_objects", 0.0))
                     for cell in getattr(env_2d, "cells_with_objects", [])}
    physical_count_by_key = {
        _cell_key(cell): float(getattr(cell, "physical_object_count", getattr(cell, "num_objects", 0.0)))
        for cell in getattr(env_2d, "cells_with_objects", [])}
    material_mass_by_key = {
        _cell_key(cell): float(getattr(cell, "material_mass", getattr(cell, "num_objects", 0.0)))
        for cell in getattr(env_2d, "cells_with_objects", [])}
    for source in sources:
        source_keys = {_cell_key(cell) for cell in source.canonical_cells}
        representative = min(source.canonical_cells, key=lambda cell: math.hypot(
            converter.convert_2d_to_3d(cell.x, cell.y)[0] - source.world_center[0],
            converter.convert_2d_to_3d(cell.x, cell.y)[1] - source.world_center[1]))
        for task_type in sorted(rover_type.capabilities.supported_tasks):
            if task_type == "target" and target_path_mode == "straight_nearest":
                terminal = env_2d.nearest_target_terminal_cell(representative)
                if terminal is None or terminal is representative:
                    record_generation("missing_target_terminal")
                    continue
                canonical_path = [representative, terminal]
                guide_raw = float(source.planning_quantity)
                canonical_info = {
                    "path": canonical_path,
                    "objects": guide_raw,
                    "raw_objects": guide_raw,
                    "distance": float(representative.distance_to_target),
                    "impacted_cells": {},
                    "guide_mode": "straight_nearest_boundary",
                }
                overlay = build_straight_target_overlay_path(
                    env_2d, converter, source.world_center,
                    geometry.overlay_cell_size, geometry.reservation_radius)
            else:
                guide = _best_guide(env_2d, source, task_type)
                if guide is None:
                    record_generation(f"missing_{task_type}_guide")
                    continue
                _, guide_raw, _, _, canonical_info = guide
                canonical_path = canonical_info.get("path", []) or []
                overlay = build_rover_overlay_path(
                    env_2d, converter, canonical_path, geometry.overlay_cell_size,
                    geometry.shovel_width, geometry.reservation_radius,
                    goal_mode=task_type, start_world=source.world_center)
            if overlay is None:
                record_generation(f"{task_type}_overlay_path_failed")
                continue
            if len(overlay.world_path) < 2:
                record_generation(f"{task_type}_overlay_path_degenerate")
                continue
            reserved_path = _path_keys(canonical_info)
            reserved_path.update(overlay.swept_canonical_cells)
            reserved_objects = set(source_keys)
            reserved_objects.update(key for key in overlay.swept_canonical_cells
                                    if object_by_key.get(key, 0.0) > 0.0)
            collectable = sum(object_by_key.get(key, 0.0) for key in reserved_objects)
            collectable_physical_count = sum(physical_count_by_key.get(key, 0.0) for key in reserved_objects)
            collectable_material_mass = sum(material_mass_by_key.get(key, 0.0) for key in reserved_objects)
            source_only_value = (
                task_type == "target"
                and target_candidate_value_mode == "source_only"
            )
            scoring_quantity = (
                float(source.planning_quantity)
                if source_only_value
                else max(float(source.planning_quantity), collectable)
            )
            scoring_physical_count = (
                float(source.object_count)
                if source_only_value
                else collectable_physical_count
            )
            scoring_material_mass = (
                float(source.material_mass)
                if source_only_value
                else collectable_material_mass
            )
            uncapped = CAPACITY_SCORING_MODE == "uncapped_corridor"
            expected_collected = scoring_quantity if uncapped else min(capacity, scoring_quantity)
            delivery_ratio = max(0.0, min(1.0, float(canonical_info.get("objects", guide_raw)) /
                                          max(guide_raw, 1.0)))
            expected_delivered = expected_collected * delivery_ratio
            expected_spillage = max(0.0, expected_collected - expected_delivered)
            expected_collected_objects = (
                scoring_physical_count if uncapped
                else min(capacity_objects, scoring_physical_count)
            )
            expected_collected_mass = (
                scoring_material_mass if uncapped
                else min(capacity_mass, scoring_material_mass)
            )
            path_info = dict(canonical_info)
            path_info.update({
                "path": list(canonical_path), "canonical_path": list(canonical_path),
                "world_path": list(overlay.world_path), "overlay_cells": list(overlay.cells),
                "overlay_cell_size": overlay.cell_size, "source_cell_size": geometry.shovel_width,
                "source_overlay_key": source.key, "source_canonical_cells": sorted(source_keys),
                "aggregate_objects": source.object_count,
                "aggregate_material_mass": source.material_mass,
                "aggregate_planning_quantity": source.planning_quantity,
                "capacity_scoring_mode": CAPACITY_SCORING_MODE,
                "collectable_objects": collectable_physical_count,
                "collectable_material_mass": collectable_material_mass,
                "scoring_collectable_objects": scoring_physical_count,
                "scoring_collectable_material_mass": scoring_material_mass,
                "target_path_mode": target_path_mode,
                "target_candidate_value_mode": target_candidate_value_mode,
                "expected_collected": expected_collected,
                "expected_delivered": expected_delivered,
                "expected_spillage": expected_spillage,
                "expected_collected_objects": expected_collected_objects,
                "expected_collected_mass": expected_collected_mass,
                "capacity_objects": capacity_objects,
                "capacity_mass": capacity_mass,
                "capacity_quantity": capacity,
                "material_value_mode": material_value_mode,
                "task_type": task_type, "shovel_width": geometry.shovel_width,
                "distance": sum(math.hypot(b[0]-a[0], b[1]-a[1])
                                for a, b in zip(overlay.world_path, overlay.world_path[1:])),
                "objects": expected_delivered, "raw_objects": expected_collected,
            })
            candidates.append(OverlayTaskCandidate(
                source, task_type, representative, path_info, frozenset(reserved_path),
                frozenset(reserved_objects), expected_collected, expected_delivered,
                expected_spillage, expected_collected_objects, expected_collected_mass))
    candidates = _mark_target_source_candidates(candidates, convex_hull_keys)
    return ProfileOverlaySnapshot(
        map_epoch=int(map_epoch),
        profile_key=rover_type.overlay_cache_key,
        source_cell_size=float(geometry.shovel_width),
        path_cell_size=float(geometry.overlay_cell_size),
        sources=sources,
        candidates=candidates,
        target_path_mode=target_path_mode,
        target_candidate_value_mode=target_candidate_value_mode,
        convex_hull_source_cell_keys=convex_hull_keys,
        generation_failures=tuple(sorted(generation_failures.items())),
    )





