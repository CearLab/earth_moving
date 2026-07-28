"""Persistent, rover-specific planning overlays built from one canonical heatmap."""
from __future__ import annotations
from dataclasses import dataclass, replace
from functools import lru_cache
import heapq
import math

from pebble_profiles import DEFAULT_MATERIAL_VALUE_MODE, MASS_MODE, normalize_material_mode

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
    upstream_source_keys: tuple = ()

@dataclass(frozen=True)
class ProfileOverlaySnapshot:
    map_epoch: int
    profile_key: tuple
    source_cell_size: float
    path_cell_size: float
    sources: tuple
    candidates: tuple

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
        choices.append((delivered, raw, -float(info.get("distance", len(path))), member, info))
    return max(choices, default=None, key=lambda value: value[:3])


def _mark_target_root_candidates(candidates):
    """Annotate target candidates with their upstream source relationships.

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
        path_info = dict(candidate.path_info)
        path_info.update({
            "is_root_source": is_root,
            "upstream_source_keys": list(upstream_keys),
            "root_source_rule": "no_upstream_target_corridor_collects_source",
        })
        annotated.append(replace(
            candidate,
            path_info=path_info,
            is_root_source=is_root,
            upstream_source_keys=upstream_keys,
        ))
    return tuple(annotated)


def build_profile_overlay_snapshot(map_epoch, env_2d, converter, rover_type, material_value_mode=DEFAULT_MATERIAL_VALUE_MODE):
    """Build all source cells and physically-supported task candidates once."""
    geometry = rover_type.geometry
    material_value_mode = normalize_material_mode(material_value_mode)
    capacity_objects = float(rover_type.capabilities.capacity_objects)
    capacity_mass = float(rover_type.capabilities.capacity_mass)
    capacity = capacity_mass if material_value_mode == MASS_MODE else capacity_objects
    sources = _build_sources(env_2d, converter, geometry.shovel_width)
    candidates = []
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
            guide = _best_guide(env_2d, source, task_type)
            if guide is None:
                continue
            _, guide_raw, _, _, canonical_info = guide
            canonical_path = canonical_info.get("path", []) or []
            overlay = build_rover_overlay_path(
                env_2d, converter, canonical_path, geometry.overlay_cell_size,
                geometry.shovel_width, geometry.reservation_radius,
                goal_mode=task_type, start_world=source.world_center)
            if overlay is None or len(overlay.world_path) < 2:
                continue
            reserved_path = _path_keys(canonical_info)
            reserved_path.update(overlay.swept_canonical_cells)
            reserved_objects = set(source_keys)
            reserved_objects.update(key for key in overlay.swept_canonical_cells
                                    if object_by_key.get(key, 0.0) > 0.0)
            collectable = sum(object_by_key.get(key, 0.0) for key in reserved_objects)
            expected_collected = min(capacity, max(source.planning_quantity, collectable))
            delivery_ratio = max(0.0, min(1.0, float(canonical_info.get("objects", guide_raw)) /
                                          max(guide_raw, 1.0)))
            expected_delivered = expected_collected * delivery_ratio
            expected_spillage = max(0.0, expected_collected - expected_delivered)
            collectable_physical_count = sum(physical_count_by_key.get(key, 0.0) for key in reserved_objects)
            collectable_material_mass = sum(material_mass_by_key.get(key, 0.0) for key in reserved_objects)
            expected_collected_objects = min(capacity_objects, collectable_physical_count)
            expected_collected_mass = min(capacity_mass, collectable_material_mass)
            path_info = dict(canonical_info)
            path_info.update({
                "path": list(canonical_path), "canonical_path": list(canonical_path),
                "world_path": list(overlay.world_path), "overlay_cells": list(overlay.cells),
                "overlay_cell_size": overlay.cell_size, "source_cell_size": geometry.shovel_width,
                "source_overlay_key": source.key, "source_canonical_cells": sorted(source_keys),
                "aggregate_objects": source.object_count,
                "aggregate_material_mass": source.material_mass,
                "aggregate_planning_quantity": source.planning_quantity,
                "collectable_objects": collectable_physical_count,
                "collectable_material_mass": collectable_material_mass,
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
    candidates = _mark_target_root_candidates(candidates)
    return ProfileOverlaySnapshot(int(map_epoch), rover_type.overlay_cache_key,
                                  float(geometry.shovel_width), float(geometry.overlay_cell_size),
                                  sources, candidates)





