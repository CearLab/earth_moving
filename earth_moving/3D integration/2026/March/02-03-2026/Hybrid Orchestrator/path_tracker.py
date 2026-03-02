"""
path_tracker.py - Path Tracking for Hybrid Orchestrator

This module provides:
- PathGuidanceField2D: Vector-field-based path following
- Arc-length based progress monitoring
- Shovel-point tracking (0.17m forward offset)
"""

import math
import numpy as np

from flowfield_base import FlowField2D, FlowFieldController, wrap_angle

try:
    import pybullet as p
    HAS_PYBULLET = True
except ImportError:
    HAS_PYBULLET = False


class PathGuidanceField2D(FlowField2D):
    """
    Vector-field-based path following on the same 2D grid as FlowField2D.

    For each grid cell:
      - Find closest point on the reference path and its tangent t_hat.
      - Compute signed lateral error e_n to the path (using left normal n_hat).
      - Direction is v = k_t * t_hat - k_n * e_n * n_hat, then normalized.
    """

    def __init__(self,
                 world_xmin=-1.2,
                 world_xmax=+1.2,
                 world_ymin=-1.2,
                 world_ymax=+1.2,
                 grid_w=81,
                 grid_h=81,
                 rover_radius=0.25,
                 pebble_radius=0.05,
                 clearance=0.02,
                 k_t=2.0,
                 k_n=10.0,
                 band_radius=0.5):
        """
        Initialize path guidance field.

        Args:
            k_t: tangent gain (how strongly to follow path direction)
            k_n: normal gain (how strongly to correct lateral error)
            band_radius: corridor width around path to compute field
        """
        super().__init__(world_xmin, world_xmax,
                         world_ymin, world_ymax,
                         grid_w, grid_h,
                         rover_radius, pebble_radius, clearance)
        self.k_t = float(k_t)
        self.k_n = float(k_n)
        self.band_radius = float(band_radius)
        self.path_points = None

    def _closest_point_on_path(self, px, py):
        """
        Return (closest_point_on_path, unit_tangent_there).
        """
        assert self.path_points is not None and len(self.path_points) >= 2

        p_world = np.array([px, py], dtype=float)
        best_dist2 = float("inf")
        best_closest = None
        best_tangent = None

        for i in range(len(self.path_points) - 1):
            a = self.path_points[i]
            b = self.path_points[i + 1]
            ab = b - a
            ab_len2 = float(np.dot(ab, ab))
            if ab_len2 < 1e-9:
                continue

            t = float(np.dot(p_world - a, ab) / ab_len2)
            t = max(0.0, min(1.0, t))
            closest = a + t * ab
            diff = p_world - closest
            d2 = float(np.dot(diff, diff))
            if d2 < best_dist2:
                best_dist2 = d2
                best_closest = closest
                ab_len = math.sqrt(ab_len2)
                t_hat = ab / (ab_len + 1e-9)
                best_tangent = t_hat

        if best_closest is None:
            a = self.path_points[0]
            b = self.path_points[1]
            ab = b - a
            ab_len = float(np.linalg.norm(ab))
            t_hat = ab / (ab_len + 1e-9)
            return a, t_hat

        return best_closest, best_tangent

    def compute_path_direction_field(self, path_points):
        """
        Compute direction vectors only in a corridor (band_radius) around the path.
        """
        self.path_points = np.array(path_points, dtype=float)
        gh, gw = self.grid_h, self.grid_w
        self.dir_field[:, :, :] = 0.0

        cell_size = min(self.cell_w, self.cell_h)
        band_cells = int(math.ceil(self.band_radius / cell_size))
        band_cells = max(band_cells, 1)

        mask = np.zeros((gh, gw), dtype=bool)

        # Mark band cells
        for (px, py) in self.path_points:
            ix, iy = self.world_to_cell(px, py)
            for dy in range(-band_cells, band_cells + 1):
                yy = iy + dy
                if yy < 0 or yy >= gh:
                    continue
                for dx in range(-band_cells, band_cells + 1):
                    xx = ix + dx
                    if xx < 0 or xx >= gw:
                        continue
                    if dx * dx + dy * dy <= band_cells * band_cells:
                        mask[yy, xx] = True

        ys, xs = np.where(mask)
        for iy, ix in zip(ys, xs):
            if self.obstacles[iy, ix]:
                continue

            cx, cy = self.cell_to_world_center(ix, iy)
            path_pt, t_hat = self._closest_point_on_path(cx, cy)

            n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
            e_vec = np.array([cx, cy], dtype=float) - path_pt
            e_n = float(np.dot(e_vec, n_hat))

            v = self.k_t * t_hat - self.k_n * e_n * n_hat
            mag = float(np.linalg.norm(v))
            if mag < 1e-6:
                continue
            v /= mag

            self.dir_field[iy, ix, 0] = v[0]
            self.dir_field[iy, ix, 1] = v[1]

    def rebuild_for_path(self, path_points, pebble_centers):
        """
        Stamp obstacles from pebbles, then build the banded path-guidance field.
        """
        self.clear_obstacles()
        self.stamp_pebbles(pebble_centers)
        self.compute_path_direction_field(path_points)


# =========================================================
#        PATH GEOMETRY (ARC LENGTH)
# =========================================================

def precompute_arc_length(path_points):
    """
    Precompute arc-length information for a path.

    Returns:
        (pts, segs, seg_lens, s_cum, total_L)
    """
    pts = np.array(path_points, dtype=float)
    segs = pts[1:] - pts[:-1]
    seg_lens = np.linalg.norm(segs, axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_L = s_cum[-1]
    return pts, segs, seg_lens, s_cum, total_L


def project_point_to_path_s(p_world, pts, segs, seg_lens, s_cum):
    """
    Project a point onto the path and return arc-length position.

    Returns:
        (s_position, cross_track_distance)
    """
    best_d2 = float("inf")
    best_s = 0.0

    for i in range(len(segs)):
        a = pts[i]
        ab = segs[i]
        L2 = seg_lens[i] ** 2
        if L2 < 1e-12:
            continue

        t = np.dot(p_world - a, ab) / L2
        t = np.clip(t, 0.0, 1.0)
        proj = a + t * ab

        d2 = np.dot(p_world - proj, p_world - proj)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s_cum[i] + t * seg_lens[i]

    return best_s, math.sqrt(best_d2)


def interpolate_along_path(s_query, s_cum, values):
    """
    Interpolate a value at arc-length s_query.
    """
    s_total = s_cum[-1]
    if s_query <= s_cum[0]:
        return float(values[0])
    if s_query >= s_total:
        return float(values[-1])

    j = int(np.searchsorted(s_cum, s_query))
    j = max(1, min(j, len(s_cum) - 1))
    s0 = s_cum[j - 1]
    s1 = s_cum[j]
    t = (s_query - s0) / (s1 - s0 + 1e-9)
    return float((1.0 - t) * values[j - 1] + t * values[j])


class PathTracker:
    """
    High-level path tracking controller.

    Combines PathGuidanceField2D with a FlowFieldController and
    provides shovel-point tracking with arc-length progress monitoring.
    """

    # Default parameters
    DEFAULT_K_T = 2.0
    DEFAULT_K_N = 10.0
    DEFAULT_BAND_RADIUS = 0.5
    DEFAULT_V_MAX = 1.0
    DEFAULT_W_MAX = 6.0
    DEFAULT_SHOVEL_OFFSET = 0.17

    def __init__(self,
                 world_bounds=(-3.5, 3.5, -3.5, 3.5),
                 grid_size=(81, 81),
                 k_t=None,
                 k_n=None,
                 band_radius=None,
                 v_max=None,
                 w_max=None,
                 shovel_offset=None,
                 rover_radius=0.25,
                 pebble_radius=0.05):
        """
        Initialize path tracker.

        Args:
            world_bounds: (xmin, xmax, ymin, ymax)
            grid_size: (grid_w, grid_h)
            k_t: tangent gain
            k_n: normal gain
            band_radius: corridor width
            v_max: max forward velocity
            w_max: max angular velocity
            shovel_offset: forward offset for tracking point
            rover_radius: rover safety radius
            pebble_radius: pebble radius
        """
        self.world_bounds = world_bounds
        self.grid_size = grid_size
        self.k_t = k_t or self.DEFAULT_K_T
        self.k_n = k_n or self.DEFAULT_K_N
        self.band_radius = band_radius or self.DEFAULT_BAND_RADIUS
        self.v_max = v_max or self.DEFAULT_V_MAX
        self.w_max = w_max or self.DEFAULT_W_MAX
        self.shovel_offset = shovel_offset or self.DEFAULT_SHOVEL_OFFSET
        self.rover_radius = rover_radius
        self.pebble_radius = pebble_radius

        # State
        self.field = None
        self.controller = None
        self.path_points = None
        self.path_geometry = None
        self.goal = None

        # Progress tracking
        self.s_prev = 0.0
        self.v_s_ema = 0.0
        self.ema_alpha = 0.15

    def build_field(self, path_points, pebble_centers):
        """
        Build the path guidance field for a new path.

        Args:
            path_points: list of (x, y) waypoints
            pebble_centers: list of (x, y) pebble positions
        """
        self.path_points = path_points
        self.goal = np.array(path_points[-1], dtype=float)

        # Compute path bounds with margin
        xs = [pt[0] for pt in path_points]
        ys = [pt[1] for pt in path_points]
        path_margin = 1.0
        world_xmin = min(xs) - path_margin
        world_xmax = max(xs) + path_margin
        world_ymin = min(ys) - path_margin
        world_ymax = max(ys) + path_margin

        # Clamp to environment bounds
        world_xmin = max(world_xmin, self.world_bounds[0])
        world_xmax = min(world_xmax, self.world_bounds[1])
        world_ymin = max(world_ymin, self.world_bounds[2])
        world_ymax = min(world_ymax, self.world_bounds[3])

        # Build field
        self.field = PathGuidanceField2D(
            world_xmin=world_xmin,
            world_xmax=world_xmax,
            world_ymin=world_ymin,
            world_ymax=world_ymax,
            grid_w=self.grid_size[0],
            grid_h=self.grid_size[1],
            rover_radius=self.rover_radius,
            pebble_radius=self.pebble_radius,
            clearance=0.02,
            k_t=self.k_t,
            k_n=self.k_n,
            band_radius=self.band_radius,
        )
        self.field.rebuild_for_path(path_points, pebble_centers)

        # Build controller
        self.controller = FlowFieldController(
            v_max=self.v_max,
            w_max=self.w_max,
            k_theta=6.0,
            turn_in_place_angle_deg=50.0,
            static_speed_threshold=0.03,
            w_turn_in_place=25.0,
        )

        # Precompute path geometry
        self.path_geometry = precompute_arc_length(path_points)

        # Reset progress
        self.s_prev = 0.0
        self.v_s_ema = 0.0

    def compute_control(self, state, dt=0.05):
        """
        Compute control commands for path tracking.

        Args:
            state: [x, y, yaw, v_fwd, w]
            dt: time step for progress computation

        Returns:
            dict with:
                - v_cmd, w_cmd: control commands
                - progress: 0.0-1.0 progress along path
                - s_now: current arc-length position
                - d_now: cross-track distance
                - v_s: tangential velocity along path
                - completed: True if path is complete
        """
        if self.field is None or self.path_geometry is None:
            return {
                "v_cmd": 0.0,
                "w_cmd": 0.0,
                "progress": 0.0,
                "s_now": 0.0,
                "d_now": 0.0,
                "v_s": 0.0,
                "completed": False,
            }

        x, y, yaw, v_fwd, w = state
        pts, segs, seg_lens, s_cum, total_L = self.path_geometry

        # Compute shovel point (tracking point)
        x_s = x + self.shovel_offset * math.cos(yaw)
        y_s = y + self.shovel_offset * math.sin(yaw)
        p_shovel = np.array([x_s, y_s], dtype=float)

        # Project shovel point to path
        s_now, d_now = project_point_to_path_s(p_shovel, pts, segs, seg_lens, s_cum)

        # Update tangential velocity estimate
        v_s = (s_now - self.s_prev) / max(dt, 1e-6)
        self.v_s_ema = (1.0 - self.ema_alpha) * self.v_s_ema + self.ema_alpha * v_s
        self.s_prev = s_now

        # Compute progress
        progress = s_now / max(total_L, 1e-6)

        # Compute control from field (using shovel point)
        tracking_state = np.array([x_s, y_s, yaw, v_fwd, w], dtype=float)
        v_cmd, w_cmd = self.controller.compute_control(tracking_state, self.goal, self.field)

        # Check completion
        S_TOL = 0.18
        V_TOL = 0.03
        remaining = max(0.0, total_L - s_now)
        completed = (remaining < S_TOL) and (abs(self.v_s_ema) < V_TOL)

        return {
            "v_cmd": v_cmd,
            "w_cmd": w_cmd,
            "progress": progress,
            "s_now": s_now,
            "d_now": d_now,
            "v_s": self.v_s_ema,
            "total_L": total_L,
            "completed": completed,
        }

    def draw_debug(self, scale=0.2, life_time=0.0):
        """Draw debug visualization in PyBullet."""
        if self.field is not None:
            self.field.draw_debug(scale=scale, life_time=life_time)
