import heapq
import math
import os
import time

import numpy as np
import pybullet as p
import pybullet_data

from flowfield_pybullet import (
    FlowField2D,
    FlowFieldController,
    yaw_to_quat,
    get_state_from_bullet,
    find_wheel_joints,
    apply_diff_drive_control,
)
from path_following_flowfield import (
    PathGuidanceField2D,
    precompute_arc_length,
    project_point_to_path_s,
)


# Drawing all field vectors is useful for debugging, but slow at high grid
# resolution. Keep this False when you only want to see the planned trajectory.
DRAW_FLOWFIELD = False
DRAW_ASTAR_RAW_PATH = True
DRAW_SMOOTH_TRAJECTORY = True
COLOR_RELAXED_IGNORED_PEBBLES = True

# Shortcutting removes A* waypoints when a straight segment is collision-free,
# but in dense scenes it can look like the rover is skipping the intended route.
USE_PATH_SHORTCUT = False

# Avoid jumping to a far-ahead path segment when the route folds near itself.
PROGRESS_AWARE_TRACKING = True
PROGRESS_BACKTRACK_M = 0.35
PROGRESS_LOOKAHEAD_M = 1.20

# If True, do not precompute path-flow vectors on the grid. The rover computes
# the local path tangent and cross-track correction online at each control step.
USE_ONLINE_PATH_GUIDANCE = True

SCENARIO_NAME = "random"

# Safety/inflation parameters. FlowField2D stamps each pebble with:
#   inflated_radius = ROVER_RADIUS + PEBBLE_RADIUS + OBSTACLE_CLEARANCE
ROVER_RADIUS = 0.15
PEBBLE_RADIUS = 0.05
OBSTACLE_CLEARANCE = 0.01

# If full-obstacle A* fails, retry after ignoring isolated pebbles. The physical
# pebbles are still spawned; they are just not considered hard planning blockers.
ALLOW_ISOLATED_OBSTACLE_RELAXATION = True
RELAX_MIN_CLUSTER_SIZE = 2
RELAX_PROGRESSIVE = True
RELAX_MAX_IGNORED_CLUSTER_SIZE = None  # None means keep relaxing until a path exists.
RELAX_CLUSTER_LINK_RADIUS = 0.35
RELAX_CLUSTER_EXTRA_GAP = 0.10


# =========================================================
#                  A* GRID PATH PLANNER
# =========================================================

class AStarGridPlanner:
    """
    A* planner over the same inflated obstacle grid used by FlowField2D.

    The output is a world-space polyline from point A to point B. Obstacles are
    already inflated by rover radius + pebble radius + clearance, so a free grid
    path should leave room for the rover footprint.
    """

    def __init__(self, flow_field):
        self.field = flow_field
        self.neighbor_offsets = [
            (-1,  0), (1,  0),
            (0, -1), (0,  1),
            (-1, -1), (-1, 1),
            (1, -1), (1,  1),
        ]

    def _in_bounds(self, ix, iy):
        return 0 <= ix < self.field.grid_w and 0 <= iy < self.field.grid_h

    def _is_free_cell(self, ix, iy):
        return self._in_bounds(ix, iy) and not self.field.obstacles[iy, ix]

    def _nearest_free_cell(self, ix, iy):
        if self._is_free_cell(ix, iy):
            return ix, iy

        max_r = max(self.field.grid_w, self.field.grid_h)
        for r in range(1, max_r + 1):
            best = None
            best_d2 = float("inf")
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx = ix + dx
                    ny = iy + dy
                    if not self._is_free_cell(nx, ny):
                        continue
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (nx, ny)
            if best is not None:
                return best

        return None

    @staticmethod
    def _heuristic(ix, iy, gx, gy):
        return math.hypot(gx - ix, gy - iy)

    def _can_step(self, x, y, dx, dy):
        nx = x + dx
        ny = y + dy
        if not self._is_free_cell(nx, ny):
            return False

        # For diagonal moves, avoid squeezing through blocked corners.
        if dx != 0 and dy != 0:
            if not self._is_free_cell(x + dx, y):
                return False
            if not self._is_free_cell(x, y + dy):
                return False

        return True

    def plan(self, start_world, goal_world):
        sx, sy = self.field.world_to_cell(float(start_world[0]), float(start_world[1]))
        gx, gy = self.field.world_to_cell(float(goal_world[0]), float(goal_world[1]))

        start_cell = self._nearest_free_cell(sx, sy)
        goal_cell = self._nearest_free_cell(gx, gy)
        if start_cell is None or goal_cell is None:
            return None

        sx, sy = start_cell
        gx, gy = goal_cell

        gh, gw = self.field.grid_h, self.field.grid_w
        g_score = np.full((gh, gw), np.inf, dtype=float)
        closed = np.zeros((gh, gw), dtype=bool)
        came_from = {}

        g_score[sy, sx] = 0.0
        heap = []
        counter = 0
        h0 = self._heuristic(sx, sy, gx, gy)
        heapq.heappush(heap, (h0, h0, counter, sx, sy))

        found = False
        while heap:
            _, _, _, x, y = heapq.heappop(heap)
            if closed[y, x]:
                continue
            closed[y, x] = True

            if x == gx and y == gy:
                found = True
                break

            for dx, dy in self.neighbor_offsets:
                if not self._can_step(x, y, dx, dy):
                    continue

                nx = x + dx
                ny = y + dy
                step_cost = math.hypot(dx, dy)
                tentative_g = g_score[y, x] + step_cost

                if tentative_g >= g_score[ny, nx]:
                    continue

                came_from[(nx, ny)] = (x, y)
                g_score[ny, nx] = tentative_g
                h = self._heuristic(nx, ny, gx, gy)
                counter += 1
                heapq.heappush(heap, (tentative_g + h, h, counter, nx, ny))

        if not found:
            return None

        cells = [(gx, gy)]
        cur = (gx, gy)
        while cur != (sx, sy):
            cur = came_from[cur]
            cells.append(cur)
        cells.reverse()

        path = []
        for ix, iy in cells:
            wx, wy = self.field.cell_to_world_center(ix, iy)
            path.append(np.array([wx, wy], dtype=float))

        path[0] = np.asarray(start_world, dtype=float)
        path[-1] = np.asarray(goal_world, dtype=float)
        return path


# =========================================================
#              TRAJECTORY SMOOTHING HELPERS
# =========================================================

def point_is_free(field, point):
    x, y = float(point[0]), float(point[1])
    if x < field.world_xmin or x > field.world_xmax:
        return False
    if y < field.world_ymin or y > field.world_ymax:
        return False
    ix, iy = field.world_to_cell(x, y)
    return not field.obstacles[iy, ix]


def segment_is_free(field, a, b, sample_step=None):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if sample_step is None:
        sample_step = 0.35 * min(field.cell_w, field.cell_h)

    length = float(np.linalg.norm(b - a))
    n = max(1, int(math.ceil(length / max(sample_step, 1e-6))))
    for k in range(n + 1):
        t = k / n
        p_world = (1.0 - t) * a + t * b
        if not point_is_free(field, p_world):
            return False
    return True


def polyline_is_free(field, path):
    if len(path) < 2:
        return False
    for i in range(len(path) - 1):
        if not segment_is_free(field, path[i], path[i + 1]):
            return False
    return True


def polyline_length(path):
    if len(path) < 2:
        return 0.0
    length = 0.0
    for i in range(len(path) - 1):
        length += float(np.linalg.norm(path[i + 1] - path[i]))
    return length


def shortcut_path(field, raw_path):
    """
    Remove unnecessary A* waypoints when a straight, collision-free segment exists.
    """
    if raw_path is None or len(raw_path) <= 2:
        return raw_path

    out = [raw_path[0]]
    i = 0
    while i < len(raw_path) - 1:
        j = len(raw_path) - 1
        while j > i + 1:
            if segment_is_free(field, raw_path[i], raw_path[j]):
                break
            j -= 1
        out.append(raw_path[j])
        i = j

    return out


def chaikin_once(path, cut=0.25):
    if len(path) <= 2:
        return path

    cut = float(np.clip(cut, 0.05, 0.45))
    smoothed = [np.asarray(path[0], dtype=float)]
    for i in range(len(path) - 1):
        a = np.asarray(path[i], dtype=float)
        b = np.asarray(path[i + 1], dtype=float)
        q = (1.0 - cut) * a + cut * b
        r = cut * a + (1.0 - cut) * b
        smoothed.append(q)
        smoothed.append(r)
    smoothed.append(np.asarray(path[-1], dtype=float))
    return smoothed


def smooth_path_collision_checked(field, path, iterations=4, cut=0.25):
    """
    Corner-cut the path while keeping only collision-free smoothing steps.
    """
    if path is None or len(path) <= 2:
        return path

    current = [np.asarray(p_world, dtype=float) for p_world in path]
    for _ in range(iterations):
        candidate = chaikin_once(current, cut=cut)
        if polyline_is_free(field, candidate):
            current = candidate
        else:
            break
    return current


def resample_polyline(path, spacing=0.08):
    """
    Convert the smoothed polyline into a dense, continuous trajectory.
    """
    if path is None or len(path) == 0:
        return []
    if len(path) == 1:
        return [np.asarray(path[0], dtype=float)]

    pts = [np.asarray(p_world, dtype=float) for p_world in path]
    total = polyline_length(pts)
    if total < 1e-9:
        return [pts[0], pts[-1]]

    spacing = max(float(spacing), 1e-3)
    s_queries = np.arange(0.0, total, spacing)
    if len(s_queries) == 0 or s_queries[-1] < total:
        s_queries = np.append(s_queries, total)

    dense = []
    seg_idx = 0
    seg_start_s = 0.0
    seg_len = float(np.linalg.norm(pts[1] - pts[0]))

    for s_query in s_queries:
        while seg_idx < len(pts) - 2 and s_query > seg_start_s + seg_len:
            seg_start_s += seg_len
            seg_idx += 1
            seg_len = float(np.linalg.norm(pts[seg_idx + 1] - pts[seg_idx]))

        if seg_len < 1e-9:
            dense.append(pts[seg_idx].copy())
            continue

        u = (s_query - seg_start_s) / seg_len
        u = float(np.clip(u, 0.0, 1.0))
        dense.append((1.0 - u) * pts[seg_idx] + u * pts[seg_idx + 1])

    dense[0] = pts[0].copy()
    dense[-1] = pts[-1].copy()
    return dense


def project_point_to_path_s_windowed(p_world,
                                     pts,
                                     segs,
                                     seg_lens,
                                     s_cum,
                                     s_ref,
                                     backtrack_s,
                                     lookahead_s):
    """
    Project to the path only near the current progress.

    This prevents a folded A* path from snapping the shovel to a later segment
    that happens to be physically nearby.
    """
    total_s = float(s_cum[-1])
    s_min = max(0.0, float(s_ref) - float(backtrack_s))
    s_max = min(total_s, float(s_ref) + float(lookahead_s))

    best_d2 = float("inf")
    best_s = float(s_ref)
    best_proj = None
    best_tangent = None

    for i in range(len(segs)):
        seg_s0 = float(s_cum[i])
        seg_s1 = float(s_cum[i + 1])
        L = float(seg_lens[i])
        if L < 1e-12:
            continue
        if seg_s1 < s_min or seg_s0 > s_max:
            continue

        a = pts[i]
        ab = segs[i]
        L2 = L * L

        t_raw = float(np.dot(p_world - a, ab) / L2)
        t_low = max(0.0, (s_min - seg_s0) / L)
        t_high = min(1.0, (s_max - seg_s0) / L)
        t = float(np.clip(t_raw, t_low, t_high))

        proj = a + t * ab
        d2 = float(np.dot(p_world - proj, p_world - proj))
        if d2 < best_d2:
            best_d2 = d2
            best_s = seg_s0 + t * L
            best_proj = proj
            best_tangent = ab / (L + 1e-9)

    if best_proj is None:
        best_s, best_d = project_point_to_path_s(p_world, pts, segs, seg_lens, s_cum)
        return best_s, best_d, None, None

    return best_s, math.sqrt(best_d2), best_proj, best_tangent


class ProgressAwarePathFollowerField:
    """
    Field-like adapter for FlowFieldController that only tracks local path
    progress instead of the closest segment anywhere on the route.

    This is the online equivalent of the path-guidance field. The direction is
    Stanley-like:

        v = k_t * t_hat - k_n * e_n * n_hat

    which is equivalent to selecting a desired heading along the path tangent
    plus a cross-track error correction.
    """

    def __init__(self,
                 pts,
                 segs,
                 seg_lens,
                 s_cum,
                 k_t=2.0,
                 k_n=9.0,
                 backtrack_s=0.35,
                 lookahead_s=1.20):
        self.pts = pts
        self.segs = segs
        self.seg_lens = seg_lens
        self.s_cum = s_cum
        self.k_t = float(k_t)
        self.k_n = float(k_n)
        self.backtrack_s = float(backtrack_s)
        self.lookahead_s = float(lookahead_s)
        self.s_ref = 0.0

    def update_progress(self, s_ref):
        self.s_ref = float(np.clip(s_ref, 0.0, self.s_cum[-1]))

    def get_direction_world(self, x_world, y_world):
        p_world = np.array([x_world, y_world], dtype=float)
        _, _, path_pt, t_hat = project_point_to_path_s_windowed(
            p_world,
            self.pts,
            self.segs,
            self.seg_lens,
            self.s_cum,
            self.s_ref,
            self.backtrack_s,
            self.lookahead_s,
        )

        if path_pt is None or t_hat is None:
            return np.array([0.0, 0.0], dtype=float)

        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        e_vec = p_world - path_pt
        e_n = float(np.dot(e_vec, n_hat))

        v = self.k_t * t_hat - self.k_n * e_n * n_hat
        mag = float(np.linalg.norm(v))
        if mag < 1e-9:
            return np.array([0.0, 0.0], dtype=float)
        return v / mag


# =========================================================
#              OBSTACLE RELAXATION HELPERS
# =========================================================

def build_obstacle_field(env_radius,
                         grid_w,
                         grid_h,
                         pebble_centers,
                         rover_radius,
                         pebble_radius,
                         clearance):
    field = FlowField2D(
        world_xmin=-env_radius,
        world_xmax=+env_radius,
        world_ymin=-env_radius,
        world_ymax=+env_radius,
        grid_w=grid_w,
        grid_h=grid_h,
        rover_radius=rover_radius,
        pebble_radius=pebble_radius,
        clearance=clearance,
    )
    field.clear_obstacles()
    field.stamp_pebbles(pebble_centers)
    return field


def cluster_pebbles_by_distance(pebble_centers, link_radius):
    """
    Group pebbles into connected components by center distance.

    Two pebbles are considered part of the same obstacle cluster if their
    centers are closer than link_radius, directly or through other neighbors.
    """
    n = len(pebble_centers)
    if n == 0:
        return []

    pts = np.asarray(pebble_centers, dtype=float)
    visited = np.zeros(n, dtype=bool)
    components = []

    for seed in range(n):
        if visited[seed]:
            continue

        visited[seed] = True
        stack = [seed]
        comp = []

        while stack:
            i = stack.pop()
            comp.append(i)

            deltas = pts - pts[i]
            dists = np.linalg.norm(deltas, axis=1)
            neighbors = np.where((dists <= link_radius) & (~visited))[0]
            for j in neighbors:
                visited[j] = True
                stack.append(int(j))

        components.append(comp)

    return components


def filter_isolated_pebbles(pebble_centers,
                            inflated_radius,
                            min_cluster_size=2,
                            extra_gap=0.10,
                            link_radius=None):
    """
    Keep clustered pebbles as hard obstacles and ignore small isolated groups.

    This is the "relaxed" planning mask. For the default min_cluster_size=2, a
    single lonely pebble is ignored if full-obstacle A* cannot find a route.
    """
    if link_radius is None:
        link_radius = 2.0 * inflated_radius + extra_gap
    components = cluster_pebbles_by_distance(pebble_centers, link_radius)

    keep_indices = set()
    ignored_indices = set()
    for comp in components:
        target = keep_indices if len(comp) >= min_cluster_size else ignored_indices
        target.update(comp)

    kept = [pebble_centers[i] for i in sorted(keep_indices)]
    ignored = [pebble_centers[i] for i in sorted(ignored_indices)]
    return kept, ignored, components, link_radius


def split_pebbles_by_cluster_size(pebble_centers, components, min_cluster_size):
    """
    Build one relaxed planning mask from precomputed pebble clusters.

    Components smaller than min_cluster_size are treated as movable and ignored
    by A*. Components at least min_cluster_size remain hard obstacles.
    """
    keep_indices = set()
    ignored_indices = set()
    for comp in components:
        target = keep_indices if len(comp) >= min_cluster_size else ignored_indices
        target.update(comp)

    kept = [pebble_centers[i] for i in sorted(keep_indices)]
    ignored = [pebble_centers[i] for i in sorted(ignored_indices)]
    return kept, ignored


def plan_astar_with_optional_relaxation(env_radius,
                                        grid_w,
                                        grid_h,
                                        start_pos,
                                        goal_pos,
                                        pebble_centers,
                                        rover_radius,
                                        pebble_radius,
                                        clearance,
                                        allow_relaxation=True,
                                        min_cluster_size=2,
                                        progressive_relaxation=True,
                                        max_ignored_cluster_size=None,
                                        cluster_extra_gap=0.10,
                                        cluster_link_radius=None):
    """
    First try A* with every pebble as a hard obstacle. If that fails, build
    progressively relaxed masks:

      min_cluster_size=2: ignore single-pebble clusters
      min_cluster_size=3: ignore clusters of size 1 or 2
      ...

    If max_ignored_cluster_size is None, keep relaxing until a path is found or
    even the no-obstacle mask fails.
    """
    full_field = build_obstacle_field(
        env_radius,
        grid_w,
        grid_h,
        pebble_centers,
        rover_radius,
        pebble_radius,
        clearance,
    )
    full_path = AStarGridPlanner(full_field).plan(start_pos, goal_pos)
    if full_path is not None:
        return full_field, full_path, list(pebble_centers), [], "full"

    if not allow_relaxation:
        return full_field, None, list(pebble_centers), [], "failed"

    inflated_radius = rover_radius + pebble_radius + clearance
    _, _, components, link_radius = filter_isolated_pebbles(
        pebble_centers,
        inflated_radius=inflated_radius,
        min_cluster_size=min_cluster_size,
        extra_gap=cluster_extra_gap,
        link_radius=cluster_link_radius,
    )

    print(
        "Full-obstacle A* failed. Starting progressive relaxation: "
        f"clusters={len(components)}, link_radius={link_radius:.2f} m, "
        f"inflated_radius={inflated_radius:.3f} m"
    )

    max_component_size = max((len(comp) for comp in components), default=0)
    if max_ignored_cluster_size is None:
        max_min_cluster_size = max_component_size + 1
    else:
        max_min_cluster_size = min(max_component_size + 1,
                                   int(max_ignored_cluster_size) + 1)

    if not progressive_relaxation:
        max_min_cluster_size = min_cluster_size

    last_field = full_field
    last_kept = list(pebble_centers)
    last_ignored = []

    for keep_threshold in range(int(min_cluster_size), max_min_cluster_size + 1):
        kept, ignored = split_pebbles_by_cluster_size(
            pebble_centers,
            components,
            min_cluster_size=keep_threshold,
        )
        ignored_max_size = keep_threshold - 1
        print(
            f"  Relax attempt: ignore cluster size <= {ignored_max_size}  "
            f"kept={len(kept)}  ignored={len(ignored)}"
        )

        relaxed_field = build_obstacle_field(
            env_radius,
            grid_w,
            grid_h,
            kept,
            rover_radius,
            pebble_radius,
            clearance,
        )
        relaxed_path = AStarGridPlanner(relaxed_field).plan(start_pos, goal_pos)

        last_field = relaxed_field
        last_kept = kept
        last_ignored = ignored

        if relaxed_path is not None:
            mode = f"relaxed_ignore_clusters_le_{ignored_max_size}"
            print(f"  Relaxation succeeded: {mode}")
            return relaxed_field, relaxed_path, kept, ignored, mode

    print("  Relaxation failed: no valid path found within configured limits.")
    return last_field, None, last_kept, last_ignored, "failed"


# =========================================================
#                  SIMULATION HELPERS
# =========================================================

def make_random_pebbles(env_radius,
                        num_pebbles,
                        start_pos,
                        goal_pos,
                        seed=41,
                        keepout_radius=0.65):
    rng = np.random.default_rng(seed)
    centers = []
    tries = 0
    max_tries = max(1000, 40 * num_pebbles)

    while len(centers) < num_pebbles and tries < max_tries:
        tries += 1
        r_rand = env_radius * math.sqrt(float(rng.random()))
        phi = 2.0 * math.pi * float(rng.random())
        px = r_rand * math.cos(phi)
        py = r_rand * math.sin(phi)
        p_xy = np.array([px, py], dtype=float)

        if np.linalg.norm(p_xy - start_pos) < keepout_radius:
            continue
        if np.linalg.norm(p_xy - goal_pos) < keepout_radius:
            continue

        centers.append((px, py))

    if len(centers) < num_pebbles:
        print(f"WARNING: placed only {len(centers)} / {num_pebbles} pebbles.")

    return centers


def line_pebbles(a, b, spacing=0.32):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    length = float(np.linalg.norm(b - a))
    n = max(2, int(math.ceil(length / spacing)) + 1)
    ts = np.linspace(0.0, 1.0, n)
    return [
        (float((1.0 - t) * a[0] + t * b[0]),
         float((1.0 - t) * a[1] + t * b[1]))
        for t in ts
    ]


def build_scenario(name):
    """
    Return deterministic A-to-B planning scenes.

    Available names:
      - random: same style as the original flowfield_pybullet demo.
      - edge_gate_relaxation: a wall blocks the world except for one side gate
        that is closed by a single isolated pebble.
      - two_gates_relaxation: a vertical wall has two gates, both closed by
        isolated pebbles.
    """
    name = name.lower().strip()
    env_radius = 5.0
    random_seed = 41

    if name == "random":
        start_pos = np.array([3.5, 2.9], dtype=float)
        goal_pos = np.array([-2.8, -2.8], dtype=float)
        pebble_centers = make_random_pebbles(
            env_radius=env_radius,
            num_pebbles=450,
            start_pos=start_pos,
            goal_pos=goal_pos,
            seed=random_seed,
            keepout_radius=0.75,
        )
        description = "Random pebbles; full-obstacle A* usually succeeds."

    elif name == "edge_gate_relaxation":
        start_pos = np.array([0.0, 3.6], dtype=float)
        goal_pos = np.array([0.0, -3.6], dtype=float)
        pebble_centers = []
        pebble_centers += line_pebbles([-4.85, 0.0], [4.37, 0.0], spacing=0.32)
        pebble_centers.append((4.78, 0.0))
        description = (
            "Horizontal cluster-wall touches the left boundary. The only right "
            "gate is blocked by one isolated pebble, so relaxation should be used."
        )

    elif name == "two_gates_relaxation":
        start_pos = np.array([-3.6, 0.0], dtype=float)
        goal_pos = np.array([3.6, 0.0], dtype=float)
        pebble_centers = []
        pebble_centers += line_pebbles([0.0, -4.85], [0.0, -2.41], spacing=0.32)
        pebble_centers += line_pebbles([0.0, -1.59], [0.0, 1.59], spacing=0.32)
        pebble_centers += line_pebbles([0.0, 2.41], [0.0, 4.85], spacing=0.32)
        pebble_centers.append((0.0, -2.0))
        pebble_centers.append((0.0, 2.0))
        description = (
            "Vertical cluster-wall with two gates. Each gate is closed by an "
            "isolated pebble, forcing the relaxed mask."
        )

    else:
        raise ValueError(
            f"Unknown SCENARIO_NAME={name!r}. Use 'random', "
            "'edge_gate_relaxation', or 'two_gates_relaxation'."
        )

    return {
        "name": name,
        "description": description,
        "env_radius": env_radius,
        "random_seed": random_seed,
        "start_pos": start_pos,
        "goal_pos": goal_pos,
        "pebble_centers": pebble_centers,
    }


def draw_polyline(points, color, z=0.035, line_width=2.0):
    for i in range(len(points) - 1):
        a = points[i]
        b = points[i + 1]
        p.addUserDebugLine(
            [float(a[0]), float(a[1]), z],
            [float(b[0]), float(b[1]), z],
            color,
            lineWidth=line_width,
            lifeTime=0,
        )


def setup_rover(agent, rover_urdf_path, start_pos, start_yaw):
    body_id = p.loadURDF(
        rover_urdf_path,
        basePosition=[float(start_pos[0]), float(start_pos[1]), 0.02],
        baseOrientation=yaw_to_quat(start_yaw),
        useFixedBase=False,
    )
    left_j, right_j = find_wheel_joints(body_id)
    agent["body"] = body_id
    agent["left_joint"] = left_j
    agent["right_joint"] = right_j

    r, g, b, a = agent["color"]
    p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, a])

    for j in (left_j, right_j):
        p.setJointMotorControl2(
            body_id, j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=0.0,
        )


# =========================================================
#                         MAIN
# =========================================================

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    rover_urdf_path = os.path.join(here, "2_wheel_rover.urdf")
    pebble_urdf_path = os.path.join(here, "pebbles.urdf")

    # --- Environment and task ---
    scenario = build_scenario(SCENARIO_NAME)
    env_radius = scenario["env_radius"]
    start_pos = scenario["start_pos"]
    goal_pos = scenario["goal_pos"]
    pebble_centers = scenario["pebble_centers"]

    print(f"Scenario: {scenario['name']}")
    print(f"  {scenario['description']}")

    rover_radius = ROVER_RADIUS
    pebble_radius = PEBBLE_RADIUS
    clearance = OBSTACLE_CLEARANCE

    grid_w = 101
    grid_h = 101

    # --- Build obstacle grid before simulation starts ---
    t_astar_start = time.perf_counter()
    planner_field, raw_astar_path, planning_pebbles, ignored_pebbles, planning_mode = \
        plan_astar_with_optional_relaxation(
            env_radius=env_radius,
            grid_w=grid_w,
            grid_h=grid_h,
            start_pos=start_pos,
            goal_pos=goal_pos,
            pebble_centers=pebble_centers,
            rover_radius=rover_radius,
            pebble_radius=pebble_radius,
            clearance=clearance,
            allow_relaxation=ALLOW_ISOLATED_OBSTACLE_RELAXATION,
            min_cluster_size=RELAX_MIN_CLUSTER_SIZE,
            progressive_relaxation=RELAX_PROGRESSIVE,
            max_ignored_cluster_size=RELAX_MAX_IGNORED_CLUSTER_SIZE,
            cluster_extra_gap=RELAX_CLUSTER_EXTRA_GAP,
            cluster_link_radius=RELAX_CLUSTER_LINK_RADIUS,
        )
    t_astar = time.perf_counter() - t_astar_start
    if raw_astar_path is None:
        raise RuntimeError("A* failed: no free path from start to goal.")

    t_path_start = time.perf_counter()
    shortcut = shortcut_path(planner_field, raw_astar_path)
    trajectory_base = shortcut if USE_PATH_SHORTCUT else raw_astar_path
    trajectory_base_name = "shortcut" if USE_PATH_SHORTCUT else "raw_astar"
    smoothed = smooth_path_collision_checked(
        planner_field,
        trajectory_base,
        iterations=5,
        cut=0.25,
    )
    trajectory = resample_polyline(smoothed, spacing=0.08)

    if not polyline_is_free(planner_field, trajectory):
        print("WARNING: smoothed trajectory touched an occupied cell; using unsmoothed base path.")
        smoothed = trajectory_base
        trajectory = resample_polyline(smoothed, spacing=0.08)
    t_path_generation = time.perf_counter() - t_path_start

    print("A* path planning complete:")
    print(f"  planning mode:    {planning_mode}")
    print(f"  hard obstacles:   {len(planning_pebbles)} / {len(pebble_centers)}")
    print(f"  relaxed pebbles:  {len(ignored_pebbles)}")
    print(f"  raw nodes:        {len(raw_astar_path)}")
    print(f"  shortcut nodes:   {len(shortcut)}")
    print(f"  trajectory base:  {trajectory_base_name}")
    print(f"  trajectory nodes: {len(trajectory)}")
    print(f"  raw length:       {polyline_length(raw_astar_path):.3f} m")
    print(f"  trajectory length:{polyline_length(trajectory):.3f} m")

    pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(trajectory)

    first_vec = pts[min(1, len(pts) - 1)] - pts[0]
    if np.linalg.norm(first_vec) < 1e-9:
        start_yaw = 0.0
    else:
        start_yaw = math.atan2(first_vec[1], first_vec[0])

    # --- PyBullet setup ---
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(
        cameraDistance=6.5,
        cameraYaw=45,
        cameraPitch=-60,
        cameraTargetPosition=[0, 0, 0],
    )
    p.setGravity(0, 0, -9.81)
    sim_dt = 1.0 / 240.0
    p.setTimeStep(sim_dt)
    plane_id = p.loadURDF("plane.urdf")

    # --- Spawn pebbles ---
    pebble_ids = []
    ignored_pebble_set = set(ignored_pebbles)
    for px, py in pebble_centers:
        bid = p.loadURDF(
            pebble_urdf_path,
            basePosition=[px, py, 0.01],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=False,
            globalScaling=1.0,
        )
        pebble_ids.append(bid)
        p.changeDynamics(bid, -1, lateralFriction=0.8)
        if COLOR_RELAXED_IGNORED_PEBBLES and (px, py) in ignored_pebble_set:
            p.changeVisualShape(bid, -1, rgbaColor=[0.75, 0.75, 0.75, 0.45])

    # --- Visualize planned path ---
    if DRAW_ASTAR_RAW_PATH:
        draw_polyline(raw_astar_path, [1.0, 0.35, 0.0], z=0.035, line_width=1.0)
    if DRAW_SMOOTH_TRAJECTORY:
        draw_polyline(trajectory, [1.0, 1.0, 0.0], z=0.055, line_width=3.0)

    # --- Path guidance ---
    t_field_start = time.perf_counter()
    guidance_mode = "online_stanley_progress_aware"
    pgf = None
    tracking_field = ProgressAwarePathFollowerField(
        pts,
        segs,
        seg_lens,
        s_cum,
        k_t=2.0,
        k_n=9.0,
        backtrack_s=PROGRESS_BACKTRACK_M,
        lookahead_s=PROGRESS_LOOKAHEAD_M,
    )

    if not USE_ONLINE_PATH_GUIDANCE:
        guidance_mode = "precomputed_grid_path_flow"
        pgf = PathGuidanceField2D(
            world_xmin=-env_radius,
            world_xmax=+env_radius,
            world_ymin=-env_radius,
            world_ymax=+env_radius,
            grid_w=grid_w,
            grid_h=grid_h,
            rover_radius=rover_radius,
            pebble_radius=pebble_radius,
            clearance=clearance,
            k_t=2.0,
            k_n=9.0,
            band_radius=0.75,
        )
        pgf.rebuild_for_path(trajectory, planning_pebbles)
        tracking_field = pgf
        if PROGRESS_AWARE_TRACKING:
            tracking_field = ProgressAwarePathFollowerField(
                pts,
                segs,
                seg_lens,
                s_cum,
                k_t=2.0,
                k_n=9.0,
                backtrack_s=PROGRESS_BACKTRACK_M,
                lookahead_s=PROGRESS_LOOKAHEAD_M,
            )
    t_field_build = time.perf_counter() - t_field_start

    print("==== PLANNING TIMING ====")
    print(f"A* calculation:        {t_astar:.4f} s")
    print(f"Trajectory generation: {t_path_generation:.4f} s")
    print(f"Path flow-field build: {t_field_build:.4f} s")
    print(f"Guidance mode:         {guidance_mode}")
    print("=========================")

    if DRAW_FLOWFIELD:
        if pgf is None:
            print("DRAW_FLOWFIELD requested, but online guidance has no precomputed grid field to draw.")
        else:
            pgf.draw_debug(scale=0.16, life_time=0.0)

    # --- Goal and start markers ---
    start_vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.07,
        rgbaColor=[0.1, 0.5, 1.0, 1.0],
    )
    p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=start_vis,
        basePosition=[start_pos[0], start_pos[1], 0.06],
    )

    goal_vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.08,
        rgbaColor=[0.0, 1.0, 0.0, 1.0],
    )
    p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=goal_vis,
        basePosition=[goal_pos[0], goal_pos[1], 0.06],
    )

    # --- Rover ---
    agent = {
        "id": "R0",
        "state": np.array([start_pos[0], start_pos[1], start_yaw, 0.0, 0.0]),
        "goal": goal_pos.copy(),
        "control": (0.0, 0.0),
        "color": (0.1, 0.5, 1.0, 1.0),
    }
    setup_rover(agent, rover_urdf_path, start_pos, start_yaw)

    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    p.changeDynamics(agent["body"], -1, lateralFriction=0.8)
    for link in (agent["left_joint"], agent["right_joint"]):
        p.changeDynamics(
            agent["body"],
            link,
            lateralFriction=1.0,
            rollingFriction=0.0,
            spinningFriction=0.0,
        )

    controller = FlowFieldController(
        v_max=1.0,
        w_max=6.0,
        k_theta=6.0,
        turn_in_place_angle_deg=50.0,
        static_speed_threshold=0.03,
        w_turn_in_place=25.0,
        stop_dist=0.14,
    )

    print("A* trajectory following started. Close the GUI window to stop.")

    # --- Simulation loop ---
    acc = 0.0
    sim_start = time.time()
    t_print = sim_start

    shovel_offset = 0.17
    s_prev = 0.0
    t_prev = sim_start
    v_s_ema = 0.0
    ema_alpha = 0.15
    progress_tracking_active = USE_ONLINE_PATH_GUIDANCE or PROGRESS_AWARE_TRACKING

    stop_hold_time = 0.4
    low_speed_acc = 0.0
    s_tol = 0.18
    v_tol = 0.035

    agent["state"] = get_state_from_bullet(agent["body"])
    prev_x, prev_y, _, _, _ = agent["state"]
    actual_dist = 0.0

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        if acc >= 0.05:
            acc = 0.0
            agent["state"] = get_state_from_bullet(agent["body"])
            x, y, yaw, v_fwd, w_yaw = agent["state"]

            step_dist = math.hypot(x - prev_x, y - prev_y)
            actual_dist += step_dist
            prev_x, prev_y = x, y

            x_s = x + shovel_offset * math.cos(yaw)
            y_s = y + shovel_offset * math.sin(yaw)
            p_shovel = np.array([x_s, y_s], dtype=float)

            now = time.time()
            t_rel = now - sim_start

            if progress_tracking_active:
                s_now, d_now, _, _ = project_point_to_path_s_windowed(
                    p_shovel,
                    pts,
                    segs,
                    seg_lens,
                    s_cum,
                    s_prev,
                    PROGRESS_BACKTRACK_M,
                    PROGRESS_LOOKAHEAD_M,
                )
                tracking_field.update_progress(s_now)
            else:
                s_now, d_now = project_point_to_path_s(
                    p_shovel,
                    pts,
                    segs,
                    seg_lens,
                    s_cum,
                )

            dt = max(1e-6, now - t_prev)
            v_s = (s_now - s_prev) / dt
            v_s_ema = (1.0 - ema_alpha) * v_s_ema + ema_alpha * v_s
            s_prev = s_now
            t_prev = now

            tracking_state = np.array([x_s, y_s, yaw, v_fwd, w_yaw], dtype=float)
            v_cmd, w_cmd = controller.compute_control(
                tracking_state,
                agent["goal"],
                tracking_field,
            )
            agent["control"] = (v_cmd, w_cmd)

            remaining_end = max(0.0, total_L - s_now)
            if remaining_end < s_tol:
                if abs(v_s_ema) < v_tol:
                    low_speed_acc += 0.05
                else:
                    low_speed_acc = 0.0

                if low_speed_acc >= stop_hold_time:
                    agent["control"] = (0.0, 0.0)
                    apply_diff_drive_control(agent)
                    total_time = time.time() - sim_start
                    print(
                        f"Reached planned goal region "
                        f"(remaining={remaining_end:.3f} m, d_to_path={d_now:.3f} m)."
                    )
                    print("==== RUN SUMMARY ====")
                    print(f"Total time: {total_time:.2f} s")
                    print(f"A* trajectory length: {total_L:.3f} m")
                    print(f"Rover center traveled: {actual_dist:.3f} m")
                    print("=====================")
                    break
            else:
                low_speed_acc = 0.0

            if now - t_print > 1.0:
                t_print = now
                print(
                    f"t={t_rel:5.2f}s  "
                    f"s={s_now:.2f}/{total_L:.2f}  "
                    f"d_to_path={d_now:.2f}  "
                    f"remaining={remaining_end:.2f}  "
                    f"v_s={v_s_ema:.2f}  "
                    f"v={v_cmd:.3f}  w={w_cmd:.3f}"
                )

        apply_diff_drive_control(agent)

    if p.isConnected():
        for _ in range(180):
            p.stepSimulation()
            time.sleep(sim_dt)
        p.disconnect()


if __name__ == "__main__":
    main()
