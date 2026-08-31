import argparse
import heapq
import math
import os
import time
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pybullet as p
import pybullet_data

from flowfield_pybullet import (
    yaw_to_quat,
    get_state_from_bullet,
    find_wheel_joints,
    apply_diff_drive_control,
)
from path_following_flowfield import (
    precompute_arc_length,
    interpolate_along_path,
)
from astar_path_following_flowfield import (
    AStarGridPlanner,
    build_obstacle_field,
    draw_polyline,
    make_random_pebbles,
    plan_astar_with_optional_relaxation,
    polyline_is_free,
    project_point_to_path_s_windowed,
    resample_polyline,
    shortcut_path,
    smooth_path_collision_checked,
)
from multi_astar_rovers_eta import precompute_geometric_time_profile


# =========================================================
#                      CONFIGURATION
# =========================================================

SCENARIO_NAME = "2_head_on" # "2_crossing_slow_yield_gate", "4_crossing", "4_shuffle", "crossing_replan"
PEBBLE_MODE = "random"      # "none", "random", or "scenario"
NUM_PEBBLES = 150        # used when PEBBLE_MODE = "random"
PEBBLE_SEED = 57

DRAW_ASTAR_RAW_PATH = False              # bool: draw discrete raw A* path.
DRAW_TRAJECTORY = False                  # bool: draw smooth trajectory.
DRAW_REPLANS = False                     # bool: briefly draw newly installed replans.
DRAW_CONFLICT_VISUALS = False            # bool: draw conflict timing ellipses/labels.
DRAW_NONOVERLAP_CONFLICT_VISUALS = False # bool: also draw close paths with no time overlap.
DRAW_PLAN_LIFETIME = 0.0                # s: 0 keeps plan lines until explicitly refreshed.
DRAW_REPLAN_LIFETIME = 4.0              # s: lifetime for highlighted newly installed trajectory.

ROVER_RADIUS = 0.15                     # m: rover safety/body radius used by planning.
PEBBLE_RADIUS = 0.05                    # m: radius of each pebble obstacle proxy.
OBSTACLE_CLEARANCE = 0.01               # m: extra static obstacle inflation.

ALLOW_ISOLATED_OBSTACLE_RELAXATION = True # bool: allow A* to ignore tiny isolated obstacle clusters.
RELAX_MIN_CLUSTER_SIZE = 2              # cells/objects: clusters smaller than this may be relaxed.
RELAX_PROGRESSIVE = True                # bool: progressively relax if initial A* fails.
RELAX_MAX_IGNORED_CLUSTER_SIZE = None   # cells/objects or None: largest cluster allowed to ignore.
RELAX_CLUSTER_LINK_RADIUS = 0.35        # m: distance for grouping obstacle proxies into clusters.
RELAX_CLUSTER_EXTRA_GAP = 0.10          # m: extra clearance kept around relaxed clusters.

GRID_W = 101                            # cells: A* grid width.
GRID_H = 101                            # cells: A* grid height.

SHOVEL_OFFSET = 0.17                    # m: tracking/reference point offset from rover center.
CONTROL_DT = 0.05                       # s: low-level controller update period.
SCHEDULER_DT = 0.25                     # s: conflict scheduler update period.
CELL_REFRESH_DT = 0.75                  # s: raw A* cell-path refresh period for all rovers.

V_MAX = 1.0                             # m/s: maximum forward command.
W_MAX = 10.0                             # rad/s: maximum yaw-rate command.
K_THETA = 6.0                           # 1/s: heading controller gain.
TURN_IN_PLACE_ANGLE = math.radians(50.0) # rad: heading error above this allows turn-in-place.
STATIC_SPEED_THRESHOLD = 0.03           # m/s: rover treated as stationary below this speed.
W_TURN_IN_PLACE = 25.0                  # rad/s command: in-place turn command before clamping.
PATH_STOP_S = 0.14                      # m: remaining path length considered finished/at hold point.
ENDPOINT_PASS_CROSS_TRACK_TOL = 0.18   # m: finish after crossing the endpoint plane inside this corridor.
ENDPOINT_REACQUIRE_SPEED = 0.35         # m/s: bounded speed while returning to a missed endpoint.

WHEEL_RADIUS = 0.07                     # m: wheel radius used for rover-speed to wheel-speed conversion.
TRACK_WIDTH = 0.20                      # m: distance between left/right wheels in differential-drive kinematics.
MAX_WHEEL_SPEED = 40.0                  # rad/s: baseline wheel speed limit used by PyBullet motors.
MAX_TORQUE = 15.0                       # N*m: baseline wheel motor torque used by PyBullet motors.
AUTO_SCALE_WHEEL_LIMITS = True          # bool: raise wheel speed/torque automatically when V_MAX increases.
WHEEL_SPEED_TURN_MARGIN = 1.10          # ratio: extra wheel-speed margin for driving while turning at W_MAX.
TORQUE_SPEED_SCALE_GAIN = 2.0           # ratio: torque growth per speed-scale increase above reference.

PATH_K_T = 2.0                          # dimensionless: path tangent attraction gain.
PATH_K_N = 15.0                          # 1/m: lateral path-error correction gain.
PROGRESS_BACKTRACK_M = 0.35             # m: backward search window for progress projection.
PROGRESS_LOOKAHEAD_M = 1.20             # m: forward search window for progress projection.

# Conflict detection uses center-line trajectories inflated by rover size.
CONFLICT_DISTANCE = 0.5 * ROVER_RADIUS + 0.01 # m: centerline distance that counts as path conflict.
CONFLICT_TIME_BUFFER = 0.80             # s: time overlap padding for ETA conflict checks.
CONFLICT_ZONE_HALF_S = 0.55             # m: half-length of conflict zone along winner trajectory.
HOLD_BACK_DISTANCE = 0.85               # m: yielder hold target before the conflict point.
WAIT_RELEASE_BUFFER = 0.15              # s: extra delay after winner is predicted clear.
PASSED_CONFLICT_S_BUFFER = 0.02         # m: tolerance for treating a conflict as already passed.

# Slowing tries to absorb a scheduled wait by reducing speed before stopping.
SLOW_YIELD_ENABLED = True               # bool: use slow-yield before full wait when possible.
SLOW_YIELD_STOP_MARGIN = 0.06           # m: stop once yielder is this close to the hold point.
SLOW_YIELD_TIME_EPS = 0.05              # s: release-time tolerance before returning to normal speed.
SLOW_YIELD_MIN_SPEED = 0.04             # m/s: minimum creep speed when there is room to keep moving.
SLOW_YIELD_CREEP_DISTANCE = 0.25        # m: only enforce creep speed if hold point is farther than this.

# Replanning is attempted only when the schedule delay is large enough.
REPLAN_DELAY_THRESHOLD = 2.25           # s: minimum predicted wait delay before trying normal replan.
REPLAN_COOLDOWN = 3.00                  # s: minimum time between normal replan requests.
REPLAN_WORKERS = 2                      # processes: background planner worker count.
REPLAN_MAX_LENGTH_FACTOR = 2.30         # ratio: reject replan if path is this many times longer.
REPLAN_EXTRA_ALLOWANCE = 1.20           # m: extra length allowed beyond the length ratio.
REPLAN_ACCEPT_DIST = 0.75               # m: reject finished replan if rover is now farther away.
REPLAN_PENDING_HOLD_MARGIN = 0.20       # m: switch to hold/slow if replan is unfinished near hold.
REPLAN_START_PREDICTION_DT = 0.65       # s: predict replan start this far ahead while rover is moving.

PATH_REFRESH_CHANGE_RATIO = 0.35        # ratio: raw cell path must change this much to rebuild smooth path.
PATH_REFRESH_MIN_CELLS = 8              # cells: ignore change ratio for very short paths.
TRAJECTORY_REBUILD_COOLDOWN = 2.50      # s: minimum time between smooth trajectory rebuilds.

EMERGENCY_LOOKAHEAD_S = 2.2             # m: future path arc-length checked for immediate blockers.
EMERGENCY_PATH_RADIUS = 2.0 * ROVER_RADIUS + 0.12 # m: blocker distance from path that triggers ESTOP.
EMERGENCY_STOP_DIST = 1.10              # m: Euclidean range for immediate ESTOP checks.
EMERGENCY_BLOCKER_SPEED = 0.08          # m/s: other rover treated as static/blocking below this speed.
EMERGENCY_MIN_AHEAD_S = 0.08            # m: blocker must be at least this far ahead on path.
EMERGENCY_PRIORITY_HEAD_ON_ANGLE = math.radians(15.0) # rad: forward cone for yielding to higher priority.
EMERGENCY_CLOSING_SPEED = 0.03          # m/s: minimum positive closing speed for moving-rover ESTOP.
EMERGENCY_USE_CELL_PATH = True          # bool: use latest raw A* cells for path-distance ESTOP.
EMERGENCY_DEBUG_PRINT = True            # bool: print throttled ESTOP reason diagnostics.
EMERGENCY_DEBUG_PRINT_DT = 1.00         # s: minimum time between repeated ESTOP reason prints per rover.

# Offline path-time profile used by the scheduler. These are intentionally less
# conservative than the older ETA demo defaults.
ETA_TIME_SCALE_LOWER = 1.5             # s/m scale: lower-bound offline ETA multiplier.
ETA_TIME_SCALE_UPPER = 1.6             # s/m scale: upper-bound offline ETA multiplier.

# Time-reserved A* calibration. These values are the current assumptions for
# average cell-to-cell travel time. RUN_CELL_TIME_CALIBRATION prints a small
# geometric timing report at startup; measured PyBullet calibration can later
# replace these constants.
USE_TIME_RESERVED_ASTAR = True
RUN_CELL_TIME_CALIBRATION = True        # bool: print cell-time assumption report at startup.
REFERENCE_V_MAX = 1.0                   # m/s: speed used when the cell-time model was calibrated.
CELL_TIME_SPEED_FRACTION_OF_VMAX = 0.90 # ratio: requested effective cell speed as fraction of V_MAX.
CELL_TIME_WHEEL_SPEED_FRACTION = 0.90   # ratio: effective cell speed as fraction of wheel-limited speed.
CELL_TIME_EFFECTIVE_SPEED_CAP = 1.30    # m/s or None: cap for high-speed curved-path tracking.
CELL_TIME_CURVE_TURN_PENALTY_PER_RAD_AT_REFERENCE = 0.04 # s/rad: turn penalty at REFERENCE_V_MAX.
CELL_TIME_LOWER_SCALE = 0.90            # ratio: lower ETA bound relative to nominal cell time.
CELL_TIME_UPPER_SCALE = 1.15            # ratio: upper ETA bound relative to nominal cell time.
CELL_TIME_SIGMA_BASE_AT_REFERENCE = 0.05 # s: base time-window uncertainty at REFERENCE_V_MAX.
CELL_TIME_SIGMA_GROWTH_PER_CELL_AT_REFERENCE = 0.010 # s/cell: linear uncertainty growth at reference.
CELL_RESERVATION_TIME_MARGIN = 0.25     # s: safety margin around reserved cell time windows.
CELL_WAIT_ACTION_DT = 0.25              # s: reserved-A* wait action granularity placeholder.
CELL_ASTAR_MAX_EXPANSIONS = 80000       # nodes: maximum expansions for time-reserved A*.

# Local start-direction scoring for accepting newly generated paths. This is a
# narrow filter: it only looks at the first few cells from the rover's current
# position so a replan will not suddenly ask the rover to turn in place unless
# the local start direction is worth the extra time.
START_DIRECTION_SCORING = True          # bool: reject new paths with bad initial heading change.
START_DIRECTION_MIN_CELLS = 2           # cells: minimum cells used for local start direction average.
START_DIRECTION_LOOKAHEAD_M = 0.55      # m: local distance used for start direction average.
START_DIRECTION_TURN_DEADBAND = math.radians(25.0) # rad: heading error ignored by start-turn filter.
START_DIRECTION_EFFECTIVE_TURN_RATE = 2.5 # rad/s: converts heading mismatch into time penalty.
START_DIRECTION_STATIONARY_DELAY = 0.35 # s: extra penalty when turning from near-stationary state.
START_DIRECTION_ROUTE_REFRESH_MARGIN = 0.20 # s: allowed extra start-turn time for route refresh.
START_DIRECTION_REPLAN_MARGIN = 0.30    # s: allowed extra start-turn time for conflict replan.

CONFLICT_VISUAL_LIFETIME = 0.85         # s: debug conflict drawing lifetime.
CONFLICT_VISUAL_MAX_ITEMS = 16          # count: maximum conflict visuals drawn at once.
CONFLICT_OVAL_SEGMENTS = 32             # segments: polygon resolution for uncertainty ellipses.
CONFLICT_OVAL_MIN_LONG_RADIUS = 0.18    # m: minimum ETA uncertainty ellipse long radius.
CONFLICT_OVAL_MAX_LONG_RADIUS = 1.25    # m: maximum ETA uncertainty ellipse long radius.
CONFLICT_OVAL_LATERAL_RADIUS = 0.5 * CONFLICT_DISTANCE # m: lateral uncertainty ellipse radius.
CONFLICT_OVAL_Z = 0.115                 # m: debug conflict drawing height.

BLOCKING_WINNER_PATH_ENABLED = True     # bool: detect yielder physically blocking winner corridor.
BLOCKING_WINNER_PATH_RADIUS = EMERGENCY_PATH_RADIUS # m: distance from winner path treated as blocking.
BLOCKING_WINNER_PATH_MIN_AHEAD = 1.40   # m: minimum winner lookahead for blocking checks.
BLOCKING_WINNER_PATH_LOOKAHEAD = 2.25   # m: winner lookahead for detecting a blocking yielder.
BLOCKING_WINNER_PATH_EXTRA_AHEAD = 0.75 # m: extra corridor ahead blocked for hard-clear A*.
BLOCKING_WINNER_STATIC_RADIUS = 0.55    # m: enlarged disk obstacle radius around winner during hard-clear A*.
BLOCKING_WINNER_STATIC_ESCAPE_RADIUS = 0.32 # m: local free pocket around yielder start inside winner disk.
BLOCKING_START_ESCAPE_RADIUS = 0.85     # m: do not block cells near yielder start during escape.
BLOCKING_CORRIDOR_HALF_WIDTH = 0.48     # m: half-width of winner corridor blocked for hard-clear A*.
BLOCKING_CORRIDOR_POINT_SPACING = 0.16  # m: spacing for proxy obstacles along winner corridor.
BLOCKING_CORRIDOR_LAYERS = (-1.0, -0.6, -0.25, 0.0, 0.25, 0.6, 1.0) # ratios across corridor half-width.
BLOCKING_FREEZE_AHEAD_S = 0.85          # m: freeze winner only when blocker is this close ahead.
BLOCKING_FREEZE_DIST = 0.95             # m: freeze winner if blocker is within this Euclidean distance.

# Close-pair reverse escape gives a lower-priority yielder room before a hard-clear turn.
BACKOFF_ESCAPE_ENABLED = True           # bool: allow a yielder to reverse away from a close ESTOP pair.
BACKOFF_TRIGGER_DIST = 0.55             # m: start/extend backoff if winner-yielder distance is below this.
BACKOFF_RELEASE_DIST = 0.78             # m: stop backoff once this much spacing exists.
BACKOFF_SPEED = 0.24                    # m/s: bounded reverse speed command during backoff.
BACKOFF_W_MAX = 6.0                     # rad/s: yaw-rate limit while aligning for reverse escape.
BACKOFF_DURATION = 0.90                 # s: maximum continuous reverse duration.
BACKOFF_MAX_DISTANCE = 0.42             # m: maximum distance in one reverse episode.
BACKOFF_PROGRESS_EPS = 0.035            # m: separation increase counted as progress.
BACKOFF_PROGRESS_TIMEOUT = 0.45         # s: abort if separation does not improve.
BACKOFF_FAILURE_COOLDOWN = 1.50         # s: prevent immediate repeated reverse attempts.
BACKOFF_COOLDOWN = 0.50                 # s: minimum time between new backoff triggers.
BACKOFF_MIN_REVERSE_ALIGN = 0.25        # ratio: minimum reverse speed scale when mostly facing blocker.
BACKOFF_DEBUG_PRINT = True              # bool: print when a reverse escape starts.

# Reservation obstacles are represented as dense proxy pebbles. They are
# stamped with the same inflated-mask logic as the normal A* planner.
RESERVATION_BEHIND_PATH = 0.45         # m: temporary obstacle length behind winner conflict point.
RESERVATION_AHEAD_PATH = 1.25          # m: temporary obstacle length ahead of winner conflict point.
RESERVATION_POINT_SPACING = 0.18       # m: proxy obstacle spacing for reservation corridor.
RESERVATION_HALF_WIDTH = 0.42          # m: half-width of normal conflict reservation corridor.
RESERVATION_LATERAL_LAYERS = (-1.0, -0.5, 0.0, 0.5, 1.0) # ratios across reservation half-width.

GOAL_DIST_TOL = 0.20                   # m: strict distance tolerance for goal completion.
GOAL_REMAINING_S_TOL = 0.35            # m: relaxed remaining-path tolerance near path end.
GOAL_DONE_DIST_TOL = 0.35              # m: relaxed goal distance near path end to mark rover done.
VS_MIN_FOR_ETA = 0.05                  # m/s: minimum progress speed used for online ETA factor.
SPEED_FACTOR_MIN = 0.25                # ratio: lower clamp for online speed factor.
SPEED_FACTOR_MAX = 5.0                 # ratio: upper clamp for online speed factor.
EMA_ALPHA = 0.15                       # ratio: exponential moving average weight for path speed.


# =========================================================
#                         DATA
# =========================================================

@dataclass
class TrajectoryConflict:
    i: int
    j: int
    s_i: float
    s_j: float
    p_i: np.ndarray
    p_j: np.ndarray
    center: np.ndarray
    distance: float
    t_i_low: float
    t_i_high: float
    t_j_low: float
    t_j_high: float
    winner_idx: int
    yielder_idx: int
    winner_s: float
    yielder_s: float
    release_time: float
    wait_delay: float
    cell_idx_i: int = -1
    cell_idx_j: int = -1
    time_overlap: bool = True
    blocking_winner_path: bool = False


# =========================================================
#                       GEOMETRY
# =========================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="A* trajectory scheduling without ORCA or goal flow fields."
    )
    parser.add_argument("--scenario", default=SCENARIO_NAME)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-time", type=float, default=90.0)
    parser.add_argument("--no-draw", action="store_true")
    parser.add_argument("--eta-lower-scale", type=float, default=ETA_TIME_SCALE_LOWER)
    parser.add_argument("--eta-upper-scale", type=float, default=ETA_TIME_SCALE_UPPER)
    parser.add_argument(
        "--pebbles",
        choices=("scenario", "none", "random"),
        default=PEBBLE_MODE,
        help="'scenario' keeps the scenario default, 'none' removes pebbles, 'random' adds a shared random pebble field.",
    )
    parser.add_argument("--num-pebbles", type=int, default=NUM_PEBBLES)
    parser.add_argument("--pebble-seed", type=int, default=PEBBLE_SEED)
    return parser.parse_args()


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def raw_vmax_speed_scale():
    return max(float(V_MAX), 1e-6) / max(float(REFERENCE_V_MAX), 1e-6)


def configured_max_wheel_speed():
    base = max(float(MAX_WHEEL_SPEED), 1e-6)
    if (not AUTO_SCALE_WHEEL_LIMITS) or float(V_MAX) <= float(REFERENCE_V_MAX):
        return base

    turn_allowance = max(0.0, float(W_MAX)) * float(TRACK_WIDTH) / 2.0
    required = (max(0.0, float(V_MAX)) + turn_allowance) / max(float(WHEEL_RADIUS), 1e-6)
    return max(base, float(WHEEL_SPEED_TURN_MARGIN) * required)


def configured_max_torque():
    base = max(float(MAX_TORQUE), 1e-6)
    if (not AUTO_SCALE_WHEEL_LIMITS) or float(V_MAX) <= float(REFERENCE_V_MAX):
        return base

    return base * (1.0 + float(TORQUE_SPEED_SCALE_GAIN) * max(0.0, raw_vmax_speed_scale() - 1.0))


def wheel_limited_straight_speed():
    return max(1e-6, configured_max_wheel_speed() * float(WHEEL_RADIUS))


def requested_cell_time_straight_speed(v_max):
    return max(1e-6, float(CELL_TIME_SPEED_FRACTION_OF_VMAX) * float(v_max))


def reference_cell_time_straight_speed():
    requested = requested_cell_time_straight_speed(REFERENCE_V_MAX)
    wheel_limited = float(CELL_TIME_WHEEL_SPEED_FRACTION) * wheel_limited_straight_speed()
    return max(1e-6, min(requested, wheel_limited))


def effective_cell_time_straight_speed():
    requested = requested_cell_time_straight_speed(V_MAX)
    wheel_limited = float(CELL_TIME_WHEEL_SPEED_FRACTION) * wheel_limited_straight_speed()
    effective = min(requested, wheel_limited)
    if CELL_TIME_EFFECTIVE_SPEED_CAP is not None:
        effective = min(effective, max(1e-6, float(CELL_TIME_EFFECTIVE_SPEED_CAP)))
    return max(1e-6, effective)


def cell_time_speed_scale():
    return effective_cell_time_straight_speed() / reference_cell_time_straight_speed()


def effective_cell_time_curve_turn_penalty_per_rad():
    return CELL_TIME_CURVE_TURN_PENALTY_PER_RAD_AT_REFERENCE / cell_time_speed_scale()


def effective_cell_time_sigma_base():
    return CELL_TIME_SIGMA_BASE_AT_REFERENCE / cell_time_speed_scale()


def effective_cell_time_sigma_growth_per_cell():
    return CELL_TIME_SIGMA_GROWTH_PER_CELL_AT_REFERENCE / cell_time_speed_scale()


def apply_configured_diff_drive_control(agent):
    apply_diff_drive_control(
        agent,
        wheel_radius=WHEEL_RADIUS,
        track_width=TRACK_WIDTH,
        max_wheel_speed=configured_max_wheel_speed(),
        max_torque=configured_max_torque(),
    )


def configure_timing_from_args(args):
    global ETA_TIME_SCALE_LOWER, ETA_TIME_SCALE_UPPER
    ETA_TIME_SCALE_LOWER = float(args.eta_lower_scale)
    ETA_TIME_SCALE_UPPER = float(args.eta_upper_scale)
    if ETA_TIME_SCALE_UPPER < ETA_TIME_SCALE_LOWER:
        ETA_TIME_SCALE_UPPER = ETA_TIME_SCALE_LOWER


def point_segment_projection(point, a, b):
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return 0.0, a.copy()
    u = float(np.dot(point - a, ab) / denom)
    u = clamp(u, 0.0, 1.0)
    return u, a + u * ab


def cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def segment_segment_closest(a, b, c, d):
    r = b - a
    s = d - c
    denom = cross2(r, s)

    if abs(denom) > 1e-12:
        qmp = c - a
        u = cross2(qmp, s) / denom
        v = cross2(qmp, r) / denom
        if 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0:
            q = a + u * r
            return 0.0, float(u), float(v), q, q.copy()

    candidates = []

    v, q_cd = point_segment_projection(a, c, d)
    candidates.append((float(np.linalg.norm(a - q_cd)), 0.0, v, a.copy(), q_cd))

    v, q_cd = point_segment_projection(b, c, d)
    candidates.append((float(np.linalg.norm(b - q_cd)), 1.0, v, b.copy(), q_cd))

    u, q_ab = point_segment_projection(c, a, b)
    candidates.append((float(np.linalg.norm(q_ab - c)), u, 0.0, q_ab, c.copy()))

    u, q_ab = point_segment_projection(d, a, b)
    candidates.append((float(np.linalg.norm(q_ab - d)), u, 1.0, q_ab, d.copy()))

    return min(candidates, key=lambda item: item[0])


def point_at_s(geom, s_query):
    s_query = clamp(float(s_query), 0.0, float(geom["total_L"]))
    x = interpolate_along_path(s_query, geom["s_cum"], geom["pts"][:, 0])
    y = interpolate_along_path(s_query, geom["s_cum"], geom["pts"][:, 1])
    return np.array([x, y], dtype=float)


def compress_consecutive(items):
    compressed = []
    for item in items:
        if not compressed or compressed[-1] != item:
            compressed.append(item)
    return compressed


def path_cells_from_raw_path(field, raw_path):
    cells = [
        field.world_to_cell(float(pt[0]), float(pt[1]))
        for pt in raw_path
    ]
    return compress_consecutive(cells)


def cell_points_from_path(field, cell_path):
    return np.array(
        [
            field.cell_to_world_center(ix, iy)
            for ix, iy in cell_path
        ],
        dtype=float,
    )


def cell_arc_length(cell_points):
    if cell_points is None or len(cell_points) == 0:
        return np.zeros(0, dtype=float)
    if len(cell_points) == 1:
        return np.zeros(1, dtype=float)
    diffs = cell_points[1:] - cell_points[:-1]
    lens = np.linalg.norm(diffs, axis=1)
    return np.concatenate([[0.0], np.cumsum(lens)])


def cell_time_sigma(step_count):
    return (
        effective_cell_time_sigma_base()
        + effective_cell_time_sigma_growth_per_cell() * max(0, int(step_count))
    )


def direction_angle(prev_dir, new_dir):
    if prev_dir is None or new_dir is None:
        return 0.0
    ax, ay = prev_dir
    bx, by = new_dir
    amag = math.hypot(ax, ay)
    bmag = math.hypot(bx, by)
    if amag < 1e-9 or bmag < 1e-9:
        return 0.0
    dot = (ax * bx + ay * by) / (amag * bmag)
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot)


def estimate_cell_move_time(distance, prev_dir, new_dir):
    turn_angle = direction_angle(prev_dir, new_dir)
    nominal = distance / effective_cell_time_straight_speed()
    nominal += effective_cell_time_curve_turn_penalty_per_rad() * turn_angle
    lower = CELL_TIME_LOWER_SCALE * nominal
    upper = CELL_TIME_UPPER_SCALE * nominal
    return lower, nominal, upper


def print_cell_time_calibration_report(env_radius):
    cell_w = (2.0 * env_radius) / max(GRID_W - 1, 1)
    cell_h = (2.0 * env_radius) / max(GRID_H - 1, 1)
    straight_dist = min(cell_w, cell_h)
    diag_dist = math.hypot(cell_w, cell_h)
    straight = estimate_cell_move_time(straight_dist, (1, 0), (1, 0))
    ninety = estimate_cell_move_time(straight_dist, (1, 0), (0, 1))
    diag = estimate_cell_move_time(diag_dist, (1, 0), (1, 1))
    print("==== CELL TIME CALIBRATION ASSUMPTION ====")
    print(f"cell size: {cell_w:.3f} x {cell_h:.3f} m")
    print(f"V_MAX: {V_MAX:.2f} m/s")
    print(f"reference V_MAX: {REFERENCE_V_MAX:.2f} m/s")
    print(f"speed scale: {cell_time_speed_scale():.2f} x reference")
    print(f"auto wheel limits: {AUTO_SCALE_WHEEL_LIMITS}")
    print(f"max wheel speed: {configured_max_wheel_speed():.1f} rad/s (base {MAX_WHEEL_SPEED:.1f})")
    print(f"wheel straight cap: {wheel_limited_straight_speed():.2f} m/s ({WHEEL_RADIUS:.2f}m wheel radius)")
    print(f"max torque: {configured_max_torque():.1f} N*m (base {MAX_TORQUE:.1f})")
    print(
        "straight speed assumption: "
        f"{effective_cell_time_straight_speed():.2f} m/s "
        f"(request {CELL_TIME_SPEED_FRACTION_OF_VMAX:.2f} * V_MAX, capped by wheel/track limit)"
    )
    print(f"curve penalty: {effective_cell_time_curve_turn_penalty_per_rad():.3f} s/rad")
    print(f"straight cell dt [low,nom,up]: [{straight[0]:.2f},{straight[1]:.2f},{straight[2]:.2f}] s")
    print(f"90deg curve dt [low,nom,up]: [{ninety[0]:.2f},{ninety[1]:.2f},{ninety[2]:.2f}] s")
    print(f"diagonal curve dt [low,nom,up]: [{diag[0]:.2f},{diag[1]:.2f},{diag[2]:.2f}] s")
    print(
        "time-range sigma grows linearly: "
        f"{effective_cell_time_sigma_base():.2f} + "
        f"{effective_cell_time_sigma_growth_per_cell():.3f} * cells"
    )
    print("==========================================")


def tangent_at_s(geom, s_query):
    s_cum = geom["s_cum"]
    segs = geom["segs"]
    seg_lens = geom["seg_lens"]
    if len(segs) == 0:
        return np.array([1.0, 0.0], dtype=float)

    idx = int(np.searchsorted(s_cum, s_query, side="right") - 1)
    idx = max(0, min(idx, len(segs) - 1))
    if seg_lens[idx] < 1e-9:
        return np.array([1.0, 0.0], dtype=float)
    return segs[idx] / seg_lens[idx]


def remaining_path_length(agent):
    return max(0.0, float(agent["geom"]["total_L"]) - float(agent["s"]))


def tracking_offset_for(agent):
    return float(agent.get("tracking_offset", SHOVEL_OFFSET))


def tracking_point_for(agent, state=None):
    state = agent["state"] if state is None else state
    x, y, yaw = float(state[0]), float(state[1]), float(state[2])
    offset = tracking_offset_for(agent)
    return np.array(
        [x + offset * math.cos(yaw), y + offset * math.sin(yaw)],
        dtype=float,
    )


def endpoint_completion_metrics(agent, tracking_point=None):
    """Describe the tracking point relative to the directed final endpoint."""
    geom = agent["geom"]
    total_L = float(geom["total_L"])
    endpoint = point_at_s(geom, total_L)
    tangent = np.array(tangent_at_s(geom, total_L), dtype=float)
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1e-9:
        tangent = np.array([1.0, 0.0], dtype=float)
    else:
        tangent /= tangent_norm
    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    tracking = (
        tracking_point_for(agent)
        if tracking_point is None
        else np.array(tracking_point, dtype=float)
    )
    delta = tracking - endpoint
    cross_track_tol = float(
        agent.get("endpoint_cross_track_tol", ENDPOINT_PASS_CROSS_TRACK_TOL)
    )
    return {
        "endpoint": endpoint,
        "tangent": tangent,
        "distance": float(np.linalg.norm(delta)),
        "along": float(np.dot(delta, tangent)),
        "cross_track": abs(float(np.dot(delta, normal))),
        "cross_track_tol": cross_track_tol,
    }


def endpoint_passed(agent, tracking_point=None):
    metrics = endpoint_completion_metrics(agent, tracking_point)
    return (
        metrics["along"] >= 0.0
        and metrics["cross_track"] <= metrics["cross_track_tol"]
    )


def reproject_agent_progress(agent, tracking_point=None):
    """Globally reproject progress after an external/manual reposition."""
    geom = agent["geom"]
    tracking = (
        tracking_point_for(agent)
        if tracking_point is None
        else np.array(tracking_point, dtype=float)
    )
    total_L = float(geom["total_L"])
    s_now, d_now, path_proj, t_hat = project_point_to_path_s_windowed(
        tracking,
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        0.0,
        0.0,
        max(total_L, 1e-6),
    )
    agent["s"] = float(s_now)
    agent["s_prev"] = float(s_now)
    agent["d_path"] = float(d_now)
    if path_proj is not None:
        agent["path_proj"] = path_proj
    if t_hat is not None:
        agent["path_tangent"] = t_hat
    agent["endpoint_reacquiring"] = False
    return float(s_now)


def average_direction_from_points(points, min_segments, lookahead_m):
    if points is None:
        return None
    pts = [np.array(pt, dtype=float) for pt in points]
    if len(pts) < 2:
        return None

    vec_sum = np.zeros(2, dtype=float)
    dist_sum = 0.0
    used_segments = 0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            continue

        vec_sum += seg
        dist_sum += seg_len
        used_segments += 1
        if used_segments >= min_segments and dist_sum >= lookahead_m:
            break

    norm = float(np.linalg.norm(vec_sum))
    if norm < 1e-9:
        return None
    return vec_sum / norm


def current_local_path_direction(agent):
    geom = agent["geom"]
    total_L = float(geom["total_L"])
    if total_L <= 1e-9:
        return np.array([math.cos(float(agent["state"][2])), math.sin(float(agent["state"][2]))], dtype=float)

    s0 = clamp(float(agent["s"]), 0.0, total_L)
    remaining = max(0.0, total_L - s0)
    if remaining <= 1e-6:
        return tangent_at_s(geom, s0)

    sample_dist = min(START_DIRECTION_LOOKAHEAD_M, remaining)
    step = max(0.05, sample_dist / max(START_DIRECTION_MIN_CELLS, 1))
    samples = []
    count = max(1, int(math.ceil(sample_dist / step)))
    for idx in range(count + 1):
        s = clamp(s0 + idx * step, 0.0, total_L)
        samples.append(point_at_s(geom, s))
    end_point = point_at_s(geom, min(total_L, s0 + sample_dist))
    if np.linalg.norm(samples[-1] - end_point) > 1e-6:
        samples.append(end_point)

    direction = average_direction_from_points(
        samples,
        min(START_DIRECTION_MIN_CELLS, max(1, len(samples) - 1)),
        sample_dist,
    )
    if direction is None:
        return tangent_at_s(geom, s0)
    return direction


def candidate_local_cell_direction(agent, plan):
    start_xy = np.array(agent["state"][:2], dtype=float)
    points = [start_xy]

    cell_points = plan.get("cell_points")
    if cell_points is not None and len(cell_points) > 0:
        for pt in cell_points:
            arr = np.array(pt, dtype=float)
            if np.linalg.norm(arr - points[-1]) > 1e-6:
                points.append(arr)

    if len(points) < 2:
        raw_path = plan.get("raw_path", [])
        for pt in raw_path:
            arr = np.array(pt, dtype=float)
            if np.linalg.norm(arr - points[-1]) > 1e-6:
                points.append(arr)

    direction = average_direction_from_points(
        points,
        START_DIRECTION_MIN_CELLS,
        START_DIRECTION_LOOKAHEAD_M,
    )
    if direction is not None:
        return direction

    geom = plan.get("geom")
    if geom is not None:
        return tangent_at_s(geom, 0.0)
    return np.array([math.cos(float(agent["state"][2])), math.sin(float(agent["state"][2]))], dtype=float)


def start_direction_turn_report(agent, direction):
    direction = np.array(direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return {"angle": 0.0, "turn_time": 0.0}

    state = agent.get("state", np.zeros(5, dtype=float))
    yaw = float(state[2])
    v_fwd = abs(float(state[3]))
    theta = math.atan2(float(direction[1]), float(direction[0]))
    err = abs(wrap_angle(theta - float(yaw)))
    turn_time = max(0.0, err - START_DIRECTION_TURN_DEADBAND) / max(START_DIRECTION_EFFECTIVE_TURN_RATE, 1e-6)
    if v_fwd < STATIC_SPEED_THRESHOLD and err > START_DIRECTION_TURN_DEADBAND:
        turn_time += START_DIRECTION_STATIONARY_DELAY
    return {
        "angle": float(err),
        "turn_time": float(turn_time),
    }


def candidate_start_direction_report(agent, plan):
    current_direction = current_local_path_direction(agent)
    candidate_direction = candidate_local_cell_direction(agent, plan)
    current = start_direction_turn_report(agent, current_direction)
    candidate = start_direction_turn_report(agent, candidate_direction)
    return {
        "current": current,
        "candidate": candidate,
        "delta": float(candidate["turn_time"] - current["turn_time"]),
    }


def candidate_start_direction_is_reasonable(agent, plan, margin, label, sim_time=None):
    if not START_DIRECTION_SCORING:
        return True, None
    report = candidate_start_direction_report(agent, plan)
    if report["candidate"]["turn_time"] <= report["current"]["turn_time"] + float(margin):
        return True, report

    prefix = "" if sim_time is None else f"t={sim_time:.2f}s "
    print(
        f"{prefix}{agent['id']} rejected {label}: local start turn "
        f"current={math.degrees(report['current']['angle']):.1f}deg/"
        f"{report['current']['turn_time']:.2f}s, "
        f"candidate={math.degrees(report['candidate']['angle']):.1f}deg/"
        f"{report['candidate']['turn_time']:.2f}s"
    )
    return False, report


def _estimate_speed_factor(distance_remaining, nominal_time_remaining, measured_speed):
    if distance_remaining <= 1e-6 or nominal_time_remaining <= 1e-6:
        return 1.0
    v_nominal_avg = distance_remaining / nominal_time_remaining
    speed_factor = v_nominal_avg / max(VS_MIN_FOR_ETA, abs(measured_speed))
    return clamp(speed_factor, SPEED_FACTOR_MIN, SPEED_FACTOR_MAX)


# =========================================================
#                       SCENARIOS
# =========================================================

def path_color_from_rgba(rgba):
    r, g, b, _ = rgba
    return [
        min(1.0, 0.55 + 0.45 * float(r)),
        min(1.0, 0.55 + 0.45 * float(g)),
        min(1.0, 0.55 + 0.45 * float(b)),
    ]


def make_agent_cfg(idx, start_pos, goal_pos, color, priority=None, allow_replan=True):
    if priority is None:
        priority = float(idx)
    return {
        "id": f"R{idx}",
        "start": np.array([float(start_pos[0]), float(start_pos[1])], dtype=float),
        "goal": np.array([float(goal_pos[0]), float(goal_pos[1])], dtype=float),
        "priority": float(priority),
        "allow_replan": bool(allow_replan),
        "color": color,
        "path_color": path_color_from_rgba(color),
    }


def generate_shared_random_pebbles(env_radius,
                                   agents,
                                   num_pebbles,
                                   seed,
                                   keepout_radius=0.75):
    protected = []
    for cfg in agents:
        protected.append(np.asarray(cfg["start"], dtype=float))
        protected.append(np.asarray(cfg["goal"], dtype=float))

    pebble_centers = []
    next_seed = int(seed)
    while len(pebble_centers) < int(num_pebbles):
        batch = make_random_pebbles(
            env_radius,
            num_pebbles=max(25, int(num_pebbles) - len(pebble_centers)),
            start_pos=protected[0],
            goal_pos=protected[1],
            seed=next_seed,
            keepout_radius=0.0,
        )
        next_seed += 1
        for px, py in batch:
            p_xy = np.array([px, py], dtype=float)
            if all(np.linalg.norm(p_xy - q) > keepout_radius for q in protected):
                pebble_centers.append((float(px), float(py)))
                if len(pebble_centers) >= int(num_pebbles):
                    break
    return pebble_centers


def apply_pebble_mode(scenario,
                      pebble_mode="scenario",
                      num_pebbles=None,
                      pebble_seed=57):
    mode = str(pebble_mode).lower().strip()
    if mode == "none":
        scenario["pebble_centers"] = []
        scenario["description"] += " Pebbles disabled."
        return scenario

    if mode == "random":
        count = int(num_pebbles) if num_pebbles is not None else int(scenario.get("default_num_pebbles", 120))
        scenario["pebble_centers"] = generate_shared_random_pebbles(
            scenario["env_radius"],
            scenario["agents"],
            count,
            pebble_seed,
        )
        scenario["description"] += f" Random pebble field enabled ({count} pebbles)."
        return scenario

    if num_pebbles is not None and len(scenario.get("pebble_centers", [])) == 0:
        scenario["pebble_centers"] = generate_shared_random_pebbles(
            scenario["env_radius"],
            scenario["agents"],
            int(num_pebbles),
            pebble_seed,
        )
        scenario["description"] += f" Random pebble field enabled ({int(num_pebbles)} pebbles)."
    return scenario


def build_scheduling_scenario(name,
                              pebble_mode="scenario",
                              num_pebbles=None,
                              pebble_seed=57):
    name = name.lower().strip()
    env_radius = 5.0

    if name == "2_head_on":
        colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
        ]
        agents = [
            make_agent_cfg(0, [-2.0, 0.0], [2.0, 0.0], colors[0], priority=0.0),
            make_agent_cfg(1, [2.0, 0.0], [-2.0, 0.0], colors[1], priority=1.0),
        ]
        pebble_centers = []
        description = "ORCA-style 2_head_on scenario converted to A* trajectory scheduling."

    elif name == "2_crossing":
        colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
        ]
        agents = [
            make_agent_cfg(0, [-2.0, 0.0], [2.0, 0.0], colors[0], priority=0.0),
            make_agent_cfg(1, [0.0, -2.0], [0.0, 2.0], colors[1], priority=1.0),
        ]
        pebble_centers = []
        description = "ORCA-style 2_crossing scenario converted to A* trajectory scheduling."

    elif name == "2_crossing_slow_yield_gate":
        colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
        ]
        agents = [
            make_agent_cfg(0, [-4.0, 0.0], [3.0, 0.0], colors[0], priority=0.0),
            make_agent_cfg(1, [0.0, -4.0], [0.0, 3.0], colors[1], priority=1.0),
        ]

        # Two vertical pebble rows form a narrow gate for the yielder. The rows
        # have a gap at the origin, so the shortest feasible route is still
        # through the natural crossing cell instead of around the conflict.
        pebble_centers = []
        wall_xs = (-0.55, 0.55)
        y_values = np.arange(-3.55, 3.55 + 1e-9, 0.22)
        for x_wall in wall_xs:
            for y_wall in y_values:
                if abs(float(y_wall)) < 0.60:
                    continue
                pebble_centers.append((float(x_wall), float(y_wall)))

        description = (
            "Two crossing A* trajectories with static pebble gate rows. "
            "The yielder corridor is blocked from easy detours, leaving the "
            "origin crossing as the natural best route for testing slow-yield "
            "timing behavior."
        )

    elif name == "4_crossing":
        colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
            (0.1, 1.0, 0.1, 1.0),
            (1.0, 0.1, 0.1, 1.0),
        ]
        agents = [
            make_agent_cfg(0, [-3.0, 0.0], [3.0, 0.0], colors[0], priority=0.0),
            make_agent_cfg(1, [3.0, 0.0], [-3.0, 0.0], colors[1], priority=0.35),
            make_agent_cfg(2, [0.0, -3.0], [0.0, 3.0], colors[2], priority=0.70),
            make_agent_cfg(3, [0.0, 3.0], [0.0, -3.0], colors[3], priority=1.05),
        ]
        pebble_centers = []
        description = "ORCA-style 4_crossing scenario converted to A* trajectory scheduling."

    elif name == "4_shuffle":
        colors = [
            (0.1, 0.5, 1.0, 1.0),
            (1.0, 0.5, 0.1, 1.0),
            (0.1, 1.0, 0.1, 1.0),
            (1.0, 0.1, 0.1, 1.0),
        ]
        agents = [
            make_agent_cfg(0, [-3.0, 1.0], [3.0, -1.0], colors[0], priority=0.0),
            make_agent_cfg(1, [-3.0, -1.0], [3.0, 1.0], colors[1], priority=0.35),
            make_agent_cfg(2, [3.0, 1.0], [-3.0, -1.0], colors[2], priority=0.70),
            make_agent_cfg(3, [3.0, -1.0], [-3.0, 1.0], colors[3], priority=1.05),
        ]
        pebble_centers = []
        description = "ORCA-style 4_shuffle lane-swap scenario converted to A* trajectory scheduling."

    elif name == "crossing_priority":
        agents = [
            {
                "id": "R0",
                "start": np.array([-3.6, 0.0], dtype=float),
                "goal": np.array([3.6, 0.0], dtype=float),
                "priority": 0.0,
                "allow_replan": False,
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([0.0, -2.0], dtype=float),
                "goal": np.array([0.0, 3.6], dtype=float),
                "priority": 1.0,
                "allow_replan": False,
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
        ]
        pebble_centers = []
        description = "Two A* trajectories cross at the origin; R0 has priority."

    elif name == "crossing_replan":
        agents = [
            {
                "id": "R0",
                "start": np.array([-3.7, 0.0], dtype=float),
                "goal": np.array([3.7, 0.0], dtype=float),
                "priority": 0.0,
                "allow_replan": True,
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([0.0, -3.7], dtype=float),
                "goal": np.array([0.0, 3.7], dtype=float),
                "priority": 1.0,
                "allow_replan": True,
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
        ]
        pebble_centers = []
        description = (
            "Open crossing scene. The yielder can usually replan around a "
            "temporary reservation obstacle instead of waiting."
        )

    elif name == "three_rovers_priority":
        agents = [
            {
                "id": "R0",
                "start": np.array([-3.6, 0.0], dtype=float),
                "goal": np.array([3.6, 0.0], dtype=float),
                "priority": 0.0,
                "allow_replan": True,
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([0.0, -3.6], dtype=float),
                "goal": np.array([0.0, 3.6], dtype=float),
                "priority": 0.6,
                "allow_replan": True,
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
            {
                "id": "R2",
                "start": np.array([3.3, -3.3], dtype=float),
                "goal": np.array([-3.3, 3.3], dtype=float),
                "priority": 1.0,
                "allow_replan": True,
                "color": (0.25, 0.9, 0.35, 1.0),
                "path_color": [0.25, 1.0, 0.65],
            },
        ]
        pebble_centers = []
        description = "Three crossing A* trajectories with fixed priorities."

    elif name == "pebble_crossing":
        agents = [
            {
                "id": "R0",
                "start": np.array([-3.7, -0.2], dtype=float),
                "goal": np.array([3.7, 0.2], dtype=float),
                "priority": 0.0,
                "allow_replan": True,
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([0.0, -3.7], dtype=float),
                "goal": np.array([0.0, 3.7], dtype=float),
                "priority": 1.0,
                "allow_replan": True,
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
        ]
        protected = []
        for cfg in agents:
            protected.append(cfg["start"])
            protected.append(cfg["goal"])
        pebble_centers = []
        seed = 57
        while len(pebble_centers) < 120:
            batch = make_random_pebbles(
                env_radius,
                num_pebbles=max(20, 120 - len(pebble_centers)),
                start_pos=protected[0],
                goal_pos=protected[1],
                seed=seed,
                keepout_radius=0.0,
            )
            seed += 1
            for px, py in batch:
                p_xy = np.array([px, py], dtype=float)
                if all(np.linalg.norm(p_xy - q) > 0.75 for q in protected):
                    pebble_centers.append((px, py))
                    if len(pebble_centers) >= 120:
                        break
        description = "Two crossing A* trajectories through a shared pebble field."

    else:
        raise ValueError(
            "Unknown scenario. Use 2_head_on, 2_crossing, "
            "2_crossing_slow_yield_gate, 4_crossing, 4_shuffle, "
            "crossing_priority, crossing_replan, "
            "three_rovers_priority, or pebble_crossing."
        )

    scenario = {
        "name": name,
        "description": description,
        "env_radius": env_radius,
        "agents": agents,
        "pebble_centers": pebble_centers,
        "default_num_pebbles": 120,
    }
    return apply_pebble_mode(
        scenario,
        pebble_mode=pebble_mode,
        num_pebbles=num_pebbles,
        pebble_seed=pebble_seed,
    )


# =========================================================
#                   TRAJECTORY PLANNING
# =========================================================

def _finalize_plan(agent_cfg,
                   planner_field,
                   raw_path,
                   planning_pebbles,
                   ignored_pebbles,
                   planning_mode,
                   start_yaw=None):
    shortcut = shortcut_path(planner_field, raw_path)
    path_base = raw_path
    path_base_name = "raw_astar"

    smoothed = smooth_path_collision_checked(
        planner_field,
        path_base,
        iterations=5,
        cut=0.25,
    )
    trajectory = resample_polyline(smoothed, spacing=0.08)
    if not polyline_is_free(planner_field, trajectory):
        smoothed = path_base
        trajectory = resample_polyline(smoothed, spacing=0.08)

    pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(trajectory)
    cell_path = path_cells_from_raw_path(planner_field, raw_path)
    cell_points = cell_points_from_path(planner_field, cell_path)
    cell_s_cum = cell_arc_length(cell_points)
    cell_time_windows = estimate_windows_for_cell_path(cell_points, 0.0)
    eta_lower = float(agent_cfg.get("eta_scale_lower", ETA_TIME_SCALE_LOWER))
    eta_upper = float(agent_cfg.get("eta_scale_upper", ETA_TIME_SCALE_UPPER))
    eta_nominal = 0.5 * (eta_lower + eta_upper)
    (
        T_lower_nodes,
        T_upper_nodes,
        T_nominal_nodes,
        v_nom_nodes,
        landmark_s,
        landmark_T_lower,
        landmark_T_upper,
        landmark_T_nominal,
    ) = precompute_geometric_time_profile(
        pts,
        segs,
        seg_lens,
        s_cum,
        time_scale_lower=eta_lower,
        time_scale_upper=eta_upper,
        time_scale_nominal=eta_nominal,
    )

    if start_yaw is None:
        first_vec = pts[min(1, len(pts) - 1)] - pts[0]
        start_yaw = 0.0
        if np.linalg.norm(first_vec) > 1e-9:
            start_yaw = math.atan2(first_vec[1], first_vec[0])

    return {
        "raw_path": raw_path,
        "shortcut_path": shortcut,
        "trajectory": trajectory,
        "path_base_name": path_base_name,
        "planning_mode": planning_mode,
        "planning_pebbles": planning_pebbles,
        "ignored_pebbles": ignored_pebbles,
        "cell_path": cell_path,
        "cell_points": cell_points,
        "cell_s_cum": cell_s_cum,
        "cell_time_windows": cell_time_windows,
        "cell_w": float(planner_field.cell_w),
        "cell_h": float(planner_field.cell_h),
        "eta_scale_lower": eta_lower,
        "eta_scale_upper": eta_upper,
        "geom": {
            "pts": pts,
            "segs": segs,
            "seg_lens": seg_lens,
            "s_cum": s_cum,
            "total_L": total_L,
        },
        "start_yaw": start_yaw,
        "T_lower_nodes": T_lower_nodes,
        "T_upper_nodes": T_upper_nodes,
        "T_nominal_nodes": T_nominal_nodes,
        "v_nom_nodes": v_nom_nodes,
        "landmark_s": landmark_s,
        "landmark_T_lower": landmark_T_lower,
        "landmark_T_upper": landmark_T_upper,
        "landmark_T_nominal": landmark_T_nominal,
        "T_total_lower": float(T_lower_nodes[-1]),
        "T_total_upper": float(T_upper_nodes[-1]),
        "T_total_nominal": float(T_nominal_nodes[-1]),
    }


def build_initial_agent_plan(agent_cfg, env_radius, pebble_centers):
    t0 = time.perf_counter()
    planner_field, raw_path, planning_pebbles, ignored_pebbles, planning_mode = (
        plan_astar_with_optional_relaxation(
            env_radius=env_radius,
            grid_w=GRID_W,
            grid_h=GRID_H,
            start_pos=agent_cfg["start"],
            goal_pos=agent_cfg["goal"],
            pebble_centers=pebble_centers,
            rover_radius=ROVER_RADIUS,
            pebble_radius=PEBBLE_RADIUS,
            clearance=OBSTACLE_CLEARANCE,
            allow_relaxation=ALLOW_ISOLATED_OBSTACLE_RELAXATION,
            min_cluster_size=RELAX_MIN_CLUSTER_SIZE,
            progressive_relaxation=RELAX_PROGRESSIVE,
            max_ignored_cluster_size=RELAX_MAX_IGNORED_CLUSTER_SIZE,
            cluster_extra_gap=RELAX_CLUSTER_EXTRA_GAP,
            cluster_link_radius=RELAX_CLUSTER_LINK_RADIUS,
        )
    )
    if raw_path is None:
        raise RuntimeError(f"{agent_cfg['id']}: initial A* failed.")

    plan = _finalize_plan(
        agent_cfg,
        planner_field,
        raw_path,
        planning_pebbles,
        ignored_pebbles,
        planning_mode,
    )
    plan["timing"] = {"total_plan": time.perf_counter() - t0}
    return {
        "id": agent_cfg["id"],
        "start": agent_cfg["start"].copy(),
        "goal": agent_cfg["goal"].copy(),
        "priority": float(agent_cfg.get("priority", 1.0)),
        "allow_replan": bool(agent_cfg.get("allow_replan", True)),
        "eta_scale_lower": float(agent_cfg.get("eta_scale_lower", ETA_TIME_SCALE_LOWER)),
        "eta_scale_upper": float(agent_cfg.get("eta_scale_upper", ETA_TIME_SCALE_UPPER)),
        "color": agent_cfg["color"],
        "path_color": agent_cfg["path_color"],
        "hard_static_pebbles": list(planning_pebbles),
        **plan,
    }


def build_reservation_replan(agent,
                             env_radius,
                             reservation_obstacles,
                             start_xy,
                             start_yaw):
    hard_obstacles = list(agent.get("hard_static_pebbles", [])) + list(reservation_obstacles)
    field = build_obstacle_field(
        env_radius,
        GRID_W,
        GRID_H,
        hard_obstacles,
        ROVER_RADIUS,
        PEBBLE_RADIUS,
        OBSTACLE_CLEARANCE,
    )
    raw_path = AStarGridPlanner(field).plan(start_xy, agent["goal"])
    if raw_path is None:
        return None

    pseudo_cfg = {
        "id": agent["id"],
        "start": np.array(start_xy, dtype=float),
        "goal": agent["goal"],
        "eta_scale_lower": float(agent.get("eta_scale_lower", ETA_TIME_SCALE_LOWER)),
        "eta_scale_upper": float(agent.get("eta_scale_upper", ETA_TIME_SCALE_UPPER)),
    }
    return _finalize_plan(
        pseudo_cfg,
        field,
        raw_path,
        hard_obstacles,
        [],
        "reservation_hard_replan",
        start_yaw=start_yaw,
    )


def build_raw_cell_plan(agent,
                        env_radius,
                        reservation_obstacles,
                        start_xy):
    hard_obstacles = list(agent.get("hard_static_pebbles", [])) + list(reservation_obstacles)
    field = build_obstacle_field(
        env_radius,
        GRID_W,
        GRID_H,
        hard_obstacles,
        ROVER_RADIUS,
        PEBBLE_RADIUS,
        OBSTACLE_CLEARANCE,
    )
    raw_path = AStarGridPlanner(field).plan(start_xy, agent["goal"])
    if raw_path is None:
        return None

    cell_path = path_cells_from_raw_path(field, raw_path)
    cell_points = cell_points_from_path(field, cell_path)
    return {
        "raw_path": raw_path,
        "cell_path": cell_path,
        "cell_points": cell_points,
        "cell_s_cum": cell_arc_length(cell_points),
        "cell_time_windows": estimate_windows_for_cell_path(cell_points, 0.0),
        "cell_w": float(field.cell_w),
        "cell_h": float(field.cell_h),
    }


def build_smooth_from_raw_path(agent,
                               env_radius,
                               raw_path,
                               start_yaw):
    field = build_obstacle_field(
        env_radius,
        GRID_W,
        GRID_H,
        list(agent.get("hard_static_pebbles", [])),
        ROVER_RADIUS,
        PEBBLE_RADIUS,
        OBSTACLE_CLEARANCE,
    )
    pseudo_cfg = {
        "id": agent["id"],
        "start": np.array(raw_path[0], dtype=float),
        "goal": agent["goal"],
        "eta_scale_lower": float(agent.get("eta_scale_lower", ETA_TIME_SCALE_LOWER)),
        "eta_scale_upper": float(agent.get("eta_scale_upper", ETA_TIME_SCALE_UPPER)),
    }
    return _finalize_plan(
        pseudo_cfg,
        field,
        [np.array(pt, dtype=float) for pt in raw_path],
        list(agent.get("hard_static_pebbles", [])),
        [],
        "time_reserved_astar_smooth",
        start_yaw=start_yaw,
    )


TIME_ASTAR_NEIGHBORS = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def time_astar_nearest_free(field, ix, iy):
    planner = AStarGridPlanner(field)
    return planner._nearest_free_cell(ix, iy)


def time_astar_is_free(field, ix, iy):
    return 0 <= ix < field.grid_w and 0 <= iy < field.grid_h and not field.obstacles[iy, ix]


def time_astar_can_step(field, x, y, dx, dy):
    nx = x + dx
    ny = y + dy
    if not time_astar_is_free(field, nx, ny):
        return False
    if dx != 0 and dy != 0:
        if not time_astar_is_free(field, x + dx, y):
            return False
        if not time_astar_is_free(field, x, y + dy):
            return False
    return True


def reservation_cells_for_path_cell(ix, iy, cell_w, cell_h):
    radius_cells = int(math.ceil(CONFLICT_DISTANCE / max(min(cell_w, cell_h), 1e-6)))
    cells = []
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * cell_w, dy * cell_h) <= CONFLICT_DISTANCE:
                cells.append((ix + dx, iy + dy))
    return cells


def apply_reservation_wait(cell, t_low, t_nom, t_high, reservations):
    intervals = reservations.get(cell, [])
    if not intervals:
        return t_low, t_nom, t_high, None

    waited_until = None
    changed = True
    guard = 0
    while changed and guard < 20:
        guard += 1
        changed = False
        for r_low, r_high, _ in intervals:
            r0 = r_low - CELL_RESERVATION_TIME_MARGIN
            r1 = r_high + CELL_RESERVATION_TIME_MARGIN
            if t_low <= r1 and r0 <= t_high:
                wait = max(0.0, r1 - t_low)
                if wait > 1e-9:
                    t_low += wait
                    t_nom += wait
                    t_high += wait
                    waited_until = r1
                    changed = True
                    break
    return t_low, t_nom, t_high, waited_until


def estimate_windows_for_cell_path(cell_points, sim_time):
    if cell_points is None or len(cell_points) == 0:
        return []

    windows = [(float(sim_time), float(sim_time))]
    t_nom = float(sim_time)
    prev_dir = None
    t_low = float(sim_time)
    t_high = float(sim_time)
    for idx in range(1, len(cell_points)):
        delta = cell_points[idx] - cell_points[idx - 1]
        dist = float(np.linalg.norm(delta))
        if dist < 1e-9:
            windows.append((t_low, t_high))
            continue
        step_dir = (
            int(np.sign(delta[0])),
            int(np.sign(delta[1])),
        )
        move_low, move_nom, move_high = estimate_cell_move_time(dist, prev_dir, step_dir)
        t_nom += move_nom
        sigma = cell_time_sigma(idx)
        t_low = max(float(sim_time), t_nom - sigma)
        t_high = t_nom + sigma + max(0.0, move_high - move_nom)
        windows.append((t_low, t_high))
        prev_dir = step_dir
    return windows


def add_plan_to_reservations(reservations, plan, rover_id):
    cell_path = plan.get("cell_path", [])
    windows = plan.get("cell_time_windows", [])
    cell_w = float(plan.get("cell_w", 0.10))
    cell_h = float(plan.get("cell_h", 0.10))
    if not windows:
        cell_points = plan.get("cell_points")
        windows = estimate_windows_for_cell_path(cell_points, 0.0)

    for idx, cell in enumerate(cell_path):
        if idx >= len(windows):
            break
        t_low, t_high = windows[idx]
        ix, iy = cell
        for key in reservation_cells_for_path_cell(ix, iy, cell_w, cell_h):
            reservations.setdefault(key, []).append((float(t_low), float(t_high), rover_id))


def time_reserved_astar(field, start_xy, goal_xy, reservations, sim_time):
    sx, sy = field.world_to_cell(float(start_xy[0]), float(start_xy[1]))
    gx, gy = field.world_to_cell(float(goal_xy[0]), float(goal_xy[1]))
    start_cell = time_astar_nearest_free(field, sx, sy)
    goal_cell = time_astar_nearest_free(field, gx, gy)
    if start_cell is None or goal_cell is None:
        return None

    sx, sy = start_cell
    gx, gy = goal_cell
    start_state = (sx, sy, -1)

    g_nom = {start_state: float(sim_time)}
    t_low_map = {start_state: float(sim_time)}
    t_high_map = {start_state: float(sim_time)}
    step_map = {start_state: 0}
    parent = {}
    waited_until = {}

    heap = []
    counter = 0
    h0 = (
        math.hypot(gx - sx, gy - sy)
        * min(field.cell_w, field.cell_h)
        / effective_cell_time_straight_speed()
    )
    heapq.heappush(heap, (float(sim_time) + h0, float(sim_time), counter, start_state))
    best_goal = None
    expansions = 0

    while heap and expansions < CELL_ASTAR_MAX_EXPANSIONS:
        _, cur_nom, _, state = heapq.heappop(heap)
        if cur_nom > g_nom.get(state, float("inf")) + 1e-9:
            continue

        expansions += 1
        x, y, dir_idx = state
        if x == gx and y == gy:
            best_goal = state
            break

        prev_dir = None if dir_idx < 0 else TIME_ASTAR_NEIGHBORS[dir_idx]
        for ndir_idx, (dx, dy) in enumerate(TIME_ASTAR_NEIGHBORS):
            if not time_astar_can_step(field, x, y, dx, dy):
                continue

            nx = x + dx
            ny = y + dy
            dist = math.hypot(dx * field.cell_w, dy * field.cell_h)
            move_low, move_nom, move_high = estimate_cell_move_time(dist, prev_dir, (dx, dy))
            next_steps = step_map[state] + 1
            cand_nom = cur_nom + move_nom
            sigma = cell_time_sigma(next_steps)
            cand_low = max(float(sim_time), cand_nom - sigma)
            cand_high = cand_nom + sigma + max(0.0, move_high - move_nom)

            cand_low, cand_nom, cand_high, wait_until = apply_reservation_wait(
                (nx, ny),
                cand_low,
                cand_nom,
                cand_high,
                reservations,
            )

            next_state = (nx, ny, ndir_idx)
            if cand_nom >= g_nom.get(next_state, float("inf")) - 1e-9:
                continue

            g_nom[next_state] = cand_nom
            t_low_map[next_state] = cand_low
            t_high_map[next_state] = cand_high
            step_map[next_state] = next_steps
            parent[next_state] = state
            waited_until[next_state] = wait_until

            h = (
                math.hypot(gx - nx, gy - ny)
                * min(field.cell_w, field.cell_h)
                / effective_cell_time_straight_speed()
            )
            counter += 1
            heapq.heappush(heap, (cand_nom + h, cand_nom, counter, next_state))

    if best_goal is None:
        return None

    states = []
    cur = best_goal
    while cur != start_state:
        states.append(cur)
        cur = parent[cur]
    states.append(start_state)
    states.reverse()

    cells = [(state[0], state[1]) for state in states]
    raw_path = [np.array(field.cell_to_world_center(ix, iy), dtype=float) for ix, iy in cells]
    raw_path[0] = np.array(start_xy, dtype=float)
    raw_path[-1] = np.array(goal_xy, dtype=float)

    windows = [(t_low_map[state], t_high_map[state]) for state in states]
    holds = []
    for idx in range(1, len(states)):
        wait_until = waited_until.get(states[idx])
        if wait_until is not None:
            holds.append({"cell_idx": idx - 1, "wait_until": float(wait_until)})

    return {
        "raw_path": raw_path,
        "cell_path": cells,
        "cell_points": cell_points_from_path(field, cells),
        "cell_time_windows": windows,
        "time_reserved_holds": holds,
        "cell_w": float(field.cell_w),
        "cell_h": float(field.cell_h),
    }


def build_time_reserved_cell_plan(agent, env_radius, start_xy, reservations, sim_time):
    field = build_obstacle_field(
        env_radius,
        GRID_W,
        GRID_H,
        list(agent.get("hard_static_pebbles", [])),
        ROVER_RADIUS,
        PEBBLE_RADIUS,
        OBSTACLE_CLEARANCE,
    )
    plan = time_reserved_astar(field, start_xy, agent["goal"], reservations, sim_time)
    if plan is None:
        return None
    plan["cell_s_cum"] = cell_arc_length(plan["cell_points"])
    return plan


def build_prioritized_time_reserved_cell_plans(agent_snapshots, env_radius, sim_time):
    reservations = {}
    results = {}
    sorted_agents = sorted(agent_snapshots, key=lambda ag: ag["priority"])
    for ag in sorted_agents:
        start_xy = np.array(ag["start_xy"], dtype=float)
        if USE_TIME_RESERVED_ASTAR:
            plan = build_time_reserved_cell_plan(ag, env_radius, start_xy, reservations, sim_time)
        else:
            plan = build_raw_cell_plan(ag, env_radius, [], start_xy)

        if plan is None:
            plan = build_raw_cell_plan(ag, env_radius, [], start_xy)
        if plan is None:
            results[ag["id"]] = None
            continue

        if "cell_time_windows" not in plan:
            plan["cell_time_windows"] = estimate_windows_for_cell_path(plan["cell_points"], sim_time)
        plan.setdefault("time_reserved_holds", [])
        results[ag["id"]] = plan
        add_plan_to_reservations(reservations, plan, ag["id"])
    return results


def planner_warmup():
    return True


def rover_reference_xy(agent):
    state = agent.get("state")
    if state is None:
        return np.array(agent.get("start", [0.0, 0.0]), dtype=float)
    return tracking_point_for(agent, state)


def predicted_replan_start_xy(agent):
    state = agent.get("state")
    if state is None:
        return np.array(agent.get("start", [0.0, 0.0]), dtype=float)

    start_xy = np.array(state[:2], dtype=float)
    if REPLAN_START_PREDICTION_DT <= 0.0 or agent.get("emergency_stop"):
        return start_xy
    if agent.get("yield_mode") in ("wait", "slow"):
        return start_xy

    speed_s = max(0.0, float(agent.get("v_s_ema", 0.0)))
    if speed_s < VS_MIN_FOR_ETA:
        return start_xy

    geom = agent.get("geom")
    if geom is None or float(geom.get("total_L", 0.0)) <= 1e-9:
        return start_xy

    s_pred = clamp(
        float(agent.get("s", 0.0)) + speed_s * REPLAN_START_PREDICTION_DT,
        0.0,
        float(geom["total_L"]),
    )
    if s_pred <= float(agent.get("s", 0.0)) + 0.03:
        return start_xy
    return point_at_s(geom, s_pred)


def nearest_index_in_points(points, xy, start_idx=0, window=80):
    if points is None or len(points) == 0:
        return 0
    xy = np.array(xy, dtype=float)
    start_idx = max(0, min(int(start_idx), len(points) - 1))
    start = max(0, start_idx - 5)
    end = min(len(points), start_idx + int(window))
    if start >= end:
        start, end = 0, len(points)
    local = points[start:end]
    d2 = np.sum((local - xy) ** 2, axis=1)
    return start + int(np.argmin(d2))


def current_progress_on_plan(agent, plan):
    geom = plan["geom"]
    total_L = float(geom["total_L"])
    if total_L <= 1e-9:
        return 0.0, 0.0, geom["pts"][0], np.array([1.0, 0.0], dtype=float)

    s_now, d_now, path_proj, t_hat = project_point_to_path_s_windowed(
        rover_reference_xy(agent),
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        0.0,
        0.0,
        max(PROGRESS_LOOKAHEAD_M, total_L),
    )
    if path_proj is None:
        path_proj = point_at_s(geom, s_now)
    if t_hat is None:
        t_hat = tangent_at_s(geom, s_now)
    return float(s_now), float(d_now), path_proj, t_hat


def install_plan(agent, plan, sim_time):
    for key, value in plan.items():
        agent[key] = value

    s_now, d_now, path_proj, t_hat = current_progress_on_plan(agent, plan)
    agent["s"] = s_now
    agent["s_prev"] = s_now
    agent["cell_idx"] = nearest_index_in_points(plan.get("cell_points"), np.array(agent["state"][:2], dtype=float))
    agent["d_path"] = d_now
    agent["path_proj"] = path_proj
    agent["path_tangent"] = t_hat
    agent["time_prev"] = None
    agent["v_s_ema"] = 0.0
    agent["speed_factor"] = 1.0
    agent["last_replan_time"] = sim_time
    agent["last_replan_request_time"] = sim_time
    agent["yield_blockers"] = []
    agent["emergency_stop"] = False
    agent["emergency_blocker"] = None
    agent["route_refresh_future"] = None
    agent["cell_refresh_future"] = None
    agent["last_route_refresh_time"] = sim_time
    agent["last_cell_refresh_time"] = sim_time
    agent["time_reserved_holds"] = []
    agent["time_reserved_hold"] = False
    agent["clearing_winner_path"] = False
    agent["clear_winner_ids"] = []
    agent["emergency_reason"] = None
    agent["emergency_details"] = {}
    agent["last_emergency_print_time"] = -1e9
    agent.setdefault("plan_debug_items", [])
    agent.setdefault("replan_debug_items", [])


# =========================================================
#                    ONLINE PATH CONTROL
# =========================================================

def spawn_agent_rover(agent, rover_urdf_path):
    body_id = p.loadURDF(
        rover_urdf_path,
        basePosition=[float(agent["start"][0]), float(agent["start"][1]), 0.02],
        baseOrientation=yaw_to_quat(agent["start_yaw"]),
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
            body_id,
            j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=0.0,
        )

    agent["state"] = np.array(
        [agent["start"][0], agent["start"][1], agent["start_yaw"], 0.0, 0.0],
        dtype=float,
    )
    agent["control"] = (0.0, 0.0)
    agent["s"] = 0.0
    agent["s_prev"] = 0.0
    agent["cell_idx"] = 0
    agent["d_path"] = 0.0
    agent["path_proj"] = agent["geom"]["pts"][0]
    agent["path_tangent"] = tangent_at_s(agent["geom"], 0.0)
    agent["time_prev"] = None
    agent["v_s_ema"] = 0.0
    agent["speed_factor"] = 1.0
    agent["yield_mode"] = "normal"
    agent["hold_s"] = None
    agent["emergency_stop"] = False
    agent["emergency_blocker"] = None
    agent["emergency_reason"] = None
    agent["emergency_details"] = {}
    agent["last_emergency_print_time"] = -1e9
    agent["backoff_until"] = 0.0
    agent["backoff_from_id"] = None
    agent["backoff_from_xy"] = None
    agent["backoff_last_trigger_time"] = -1e9
    agent["backoff_reason"] = None
    agent["backoff_started_at"] = None
    agent["backoff_start_xy"] = None
    agent["backoff_best_dist"] = 0.0
    agent["backoff_last_progress_at"] = None
    agent["wait_until"] = 0.0
    agent["yield_winner_idx"] = None
    agent["winner_release_s"] = None
    agent["yield_blockers"] = []
    agent["last_replan_time"] = -1e9
    agent["last_replan_request_time"] = -1e9
    agent["replan_future"] = None
    agent["pending_replan"] = None
    agent["route_refresh_future"] = None
    agent["cell_refresh_future"] = None
    agent["last_route_refresh_time"] = -1e9
    agent["last_cell_refresh_time"] = -1e9
    agent["time_reserved_holds"] = []
    agent["time_reserved_hold"] = False
    agent["clearing_winner_path"] = False
    agent["clear_winner_ids"] = []
    agent["plan_debug_items"] = []
    agent["replan_debug_items"] = []
    agent["done"] = False
    agent["actual_dist"] = 0.0
    agent["prev_xy"] = agent["start"].copy()


def update_agent_state_and_progress(agent, now):
    if agent["done"]:
        return

    agent["state"] = get_state_from_bullet(agent["body"])
    x, y, yaw, v_fwd, w_yaw = agent["state"]

    xy = np.array([x, y], dtype=float)
    agent["actual_dist"] += float(np.linalg.norm(xy - agent["prev_xy"]))
    agent["prev_xy"] = xy

    shovel = tracking_point_for(agent, agent["state"])

    geom = agent["geom"]
    s_now, d_now, path_proj, t_hat = project_point_to_path_s_windowed(
        shovel,
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        agent["s_prev"],
        PROGRESS_BACKTRACK_M,
        PROGRESS_LOOKAHEAD_M,
    )

    agent["s"] = s_now
    agent["d_path"] = d_now
    if path_proj is not None and t_hat is not None:
        agent["path_proj"] = path_proj
        agent["path_tangent"] = t_hat

    if agent["time_prev"] is None:
        agent["time_prev"] = now
        agent["s_prev"] = s_now
    else:
        dt = max(1e-6, now - agent["time_prev"])
        v_s = (s_now - agent["s_prev"]) / dt
        agent["v_s_ema"] = (1.0 - EMA_ALPHA) * agent["v_s_ema"] + EMA_ALPHA * v_s
        agent["time_prev"] = now
        agent["s_prev"] = s_now

    T_now_nominal = interpolate_along_path(
        s_now,
        geom["s_cum"],
        agent["T_nominal_nodes"],
    )
    eta_nominal = max(0.0, agent["T_total_nominal"] - T_now_nominal)
    agent["speed_factor"] = 1.0
    if abs(agent["v_s_ema"]) > VS_MIN_FOR_ETA and eta_nominal > 0.05:
        agent["speed_factor"] = _estimate_speed_factor(
            remaining_path_length(agent),
            eta_nominal,
            agent["v_s_ema"],
        )
    update_agent_cell_progress(agent)


def update_agent_cell_progress(agent):
    cell_points = agent.get("cell_points")
    if cell_points is None or len(cell_points) == 0:
        agent["cell_idx"] = 0
        return

    xy = np.array(agent["state"][:2], dtype=float)
    prev_idx = int(agent.get("cell_idx", 0))
    start = max(0, prev_idx - 3)
    end = min(len(cell_points), prev_idx + 45)
    if start >= end:
        start, end = 0, len(cell_points)

    local = cell_points[start:end]
    d2 = np.sum((local - xy) ** 2, axis=1)
    best_idx = start + int(np.argmin(d2))
    agent["cell_idx"] = max(prev_idx, best_idx)


def nearest_future_path_distance(agent, point_xy, lookahead_s):
    geom = agent["geom"]
    s_ref = float(agent["s"])
    s_proj, d_path, _, _ = project_point_to_path_s_windowed(
        np.array(point_xy, dtype=float),
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        s_ref,
        0.0,
        lookahead_s,
    )
    return float(s_proj), float(d_path)


def nearest_future_cell_path_distance(agent, point_xy, lookahead_s):
    pts = agent.get("cell_points")
    if pts is None or len(pts) == 0:
        return None

    pts = np.array(pts, dtype=float)
    s_cum = agent.get("cell_s_cum")
    if s_cum is None or len(s_cum) != len(pts):
        s_cum = cell_arc_length(pts)
    s_cum = np.array(s_cum, dtype=float)

    current_idx = max(0, min(int(agent.get("cell_idx", 0)), len(pts) - 1))
    current_s = float(s_cum[current_idx])
    s_limit = current_s + float(lookahead_s)
    point_xy = np.array(point_xy, dtype=float)

    best_ahead = 0.0
    best_dist = float("inf")

    if current_idx >= len(pts) - 1:
        best_dist = float(np.linalg.norm(point_xy - pts[current_idx]))
        return best_ahead, best_dist, "cell"

    for idx in range(current_idx, len(pts) - 1):
        seg_s0 = float(s_cum[idx])
        seg_s1 = float(s_cum[idx + 1])
        if seg_s0 > s_limit:
            break

        a = pts[idx]
        b = pts[idx + 1]
        seg = b - a
        seg_len2 = float(np.dot(seg, seg))
        if seg_len2 < 1e-12:
            continue

        max_t = 1.0
        if seg_s1 > s_limit and seg_s1 > seg_s0:
            max_t = clamp((s_limit - seg_s0) / (seg_s1 - seg_s0), 0.0, 1.0)
        t = clamp(float(np.dot(point_xy - a, seg) / seg_len2), 0.0, max_t)
        q = a + t * seg
        dist = float(np.linalg.norm(point_xy - q))
        if dist < best_dist:
            best_dist = dist
            best_s = seg_s0 + t * (seg_s1 - seg_s0)
            best_ahead = max(0.0, best_s - current_s)

    if not math.isfinite(best_dist):
        best_dist = float(np.linalg.norm(point_xy - pts[current_idx]))
    return best_ahead, best_dist, "cell"


def nearest_future_emergency_path_distance(agent, point_xy):
    if EMERGENCY_USE_CELL_PATH:
        cell_result = nearest_future_cell_path_distance(agent, point_xy, EMERGENCY_LOOKAHEAD_S)
        if cell_result is not None:
            return cell_result

    s_proj, d_path = nearest_future_path_distance(
        agent,
        point_xy,
        EMERGENCY_LOOKAHEAD_S,
    )
    return max(0.0, s_proj - float(agent["s"])), float(d_path), "smooth"


def rover_velocity_xy(agent):
    state = agent.get("state", np.zeros(5, dtype=float))
    yaw = float(state[2])
    speed = float(state[3])
    return np.array([speed * math.cos(yaw), speed * math.sin(yaw)], dtype=float)


def rover_closing_speed(agent, other, delta=None):
    if delta is None:
        delta = np.array(other["state"][:2], dtype=float) - np.array(agent["state"][:2], dtype=float)
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        return 0.0
    return float(np.dot(delta, rover_velocity_xy(agent) - rover_velocity_xy(other)) / dist)


def format_emergency_details(details):
    parts = []
    for key, value in details.items():
        if isinstance(value, float):
            parts.append(f"{key}={value:.2f}")
        else:
            parts.append(f"{key}={value}")
    return " ".join(parts)


def set_emergency_stop(agent, other, reason, sim_time=None, **details):
    agent["emergency_stop"] = True
    agent["emergency_blocker"] = other["id"]
    agent["emergency_reason"] = reason
    agent["emergency_details"] = dict(details)

    if not EMERGENCY_DEBUG_PRINT or sim_time is None:
        return
    last_print = float(agent.get("last_emergency_print_time", -1e9))
    if sim_time - last_print < EMERGENCY_DEBUG_PRINT_DT:
        return
    agent["last_emergency_print_time"] = float(sim_time)
    suffix = format_emergency_details(details)
    print(
        f"t={sim_time:.2f}s {agent['id']} ESTOP:{other['id']} "
        f"reason={reason}" + (f" {suffix}" if suffix else "")
    )


def update_emergency_stops(agents, sim_time=None):
    for agent in agents:
        agent["emergency_stop"] = False
        agent["emergency_blocker"] = None
        agent["emergency_reason"] = None
        agent["emergency_details"] = {}

    apply_clear_path_estop_guards(agents, sim_time)

    for i, agent in enumerate(agents):
        if agent.get("done"):
            continue
        for j, other in enumerate(agents):
            if i == j or other.get("done"):
                continue

            if is_clearing_path_of(agent, other):
                continue

            other_speed = abs(float(other["state"][3]))
            other_has_priority = float(other.get("priority", 0.0)) < float(agent.get("priority", 0.0))
            if other_speed > EMERGENCY_BLOCKER_SPEED and not other_has_priority:
                continue

            delta = np.array(other["state"][:2], dtype=float) - np.array(agent["state"][:2], dtype=float)
            euclid = float(np.linalg.norm(delta))
            if euclid > EMERGENCY_STOP_DIST:
                continue

            ahead_s, d_path, path_source = nearest_future_emergency_path_distance(
                agent,
                other["state"][:2],
            )
            closing_speed = rover_closing_speed(agent, other, delta)
            other_is_static = other_speed <= EMERGENCY_BLOCKER_SPEED

            if other_has_priority and euclid > 1e-9:
                bearing = math.atan2(float(delta[1]), float(delta[0]))
                heading_err = abs(wrap_angle(bearing - float(agent["state"][2])))
                if (
                    heading_err <= EMERGENCY_PRIORITY_HEAD_ON_ANGLE
                    and not is_clearing_path_of(agent, other)
                    and closing_speed > EMERGENCY_CLOSING_SPEED
                    and d_path <= EMERGENCY_PATH_RADIUS
                ):
                    set_emergency_stop(
                        agent,
                        other,
                        "priority_cone",
                        sim_time,
                        angle_deg=math.degrees(heading_err),
                        dist=euclid,
                        closing=closing_speed,
                        d_path=d_path,
                        path=path_source,
                    )
                    break

            if ahead_s < EMERGENCY_MIN_AHEAD_S:
                continue
            if d_path > EMERGENCY_PATH_RADIUS:
                continue
            if not other_is_static and closing_speed <= EMERGENCY_CLOSING_SPEED:
                continue

            set_emergency_stop(
                agent,
                other,
                "path_block",
                sim_time,
                ahead=ahead_s,
                d_path=d_path,
                dist=euclid,
                closing=closing_speed,
                path=path_source,
            )
            break


def slow_speed_limit(distance_to_hold, release_time, sim_time):
    if sim_time is None:
        return V_MAX

    distance_to_hold = max(0.0, float(distance_to_hold))
    if distance_to_hold <= SLOW_YIELD_STOP_MARGIN:
        return 0.0

    time_remaining = float(release_time) - float(sim_time)
    if time_remaining <= SLOW_YIELD_TIME_EPS:
        return V_MAX

    v_limit = distance_to_hold / max(time_remaining, 1e-6)
    if 0.0 < v_limit < SLOW_YIELD_MIN_SPEED and distance_to_hold > SLOW_YIELD_CREEP_DISTANCE:
        v_limit = SLOW_YIELD_MIN_SPEED
    return clamp(v_limit, 0.0, V_MAX)


def active_time_reserved_slow_target(agent, sim_time):
    if not SLOW_YIELD_ENABLED or sim_time is None:
        return None
    if agent.get("yield_mode") not in ("normal", "replan", "slow"):
        return None

    holds = agent.get("time_reserved_holds", [])
    if not holds:
        return None

    current_idx = int(agent.get("cell_idx", 0))
    for hold in sorted(holds, key=lambda item: int(item.get("cell_idx", 0))):
        wait_until = float(hold.get("wait_until", 0.0))
        if wait_until <= float(sim_time) + SLOW_YIELD_TIME_EPS:
            continue

        hold_idx = int(hold.get("cell_idx", 0))
        if hold_idx < current_idx:
            continue

        hold_s = float(agent["s"]) + cell_relative_distance(agent, hold_idx)
        return max(float(agent["s"]), hold_s), wait_until
    return None


def compute_path_control(agent, sim_time=None):
    if agent["done"]:
        return 0.0, 0.0

    backoff_control = compute_backoff_control(agent, sim_time)
    if backoff_control is not None:
        return backoff_control

    if agent.get("emergency_stop"):
        return 0.0, 0.0

    state = agent["state"]
    x, y, yaw, v_fwd, w_yaw = state
    effective_goal_s = float(agent["geom"]["total_L"])
    slow_target = active_time_reserved_slow_target(agent, sim_time)
    agent["time_reserved_hold"] = bool(slow_target is not None)

    if agent["yield_mode"] in ("wait", "slow") and agent["hold_s"] is not None:
        effective_goal_s = min(effective_goal_s, float(agent["hold_s"]))
        if agent["s"] >= effective_goal_s - SLOW_YIELD_STOP_MARGIN:
            return 0.0, 0.0
    elif slow_target is not None:
        effective_goal_s = min(effective_goal_s, float(slow_target[0]))
        if agent["s"] >= effective_goal_s - SLOW_YIELD_STOP_MARGIN:
            return 0.0, 0.0

    remaining_s = max(0.0, effective_goal_s - float(agent["s"]))
    target_point = point_at_s(agent["geom"], effective_goal_s)
    shovel = tracking_point_for(agent, state)

    dist_target = float(np.linalg.norm(target_point - shovel))

    if remaining_s <= PATH_STOP_S and dist_target <= GOAL_DIST_TOL:
        agent["endpoint_reacquiring"] = False
        return 0.0, 0.0

    # Projection clamps at total_L. If collision/deadlock recovery displaces a
    # rover after that point, following the final tangent sends it away forever.
    # Finish only inside the endpoint corridor; otherwise explicitly reacquire
    # the endpoint with a bounded point controller.
    if remaining_s <= PATH_STOP_S:
        endpoint = endpoint_completion_metrics(agent, shovel)
        agent["endpoint_distance"] = endpoint["distance"]
        agent["endpoint_along"] = endpoint["along"]
        agent["endpoint_cross_track"] = endpoint["cross_track"]
        if (
            endpoint["along"] >= 0.0
            and endpoint["cross_track"] <= endpoint["cross_track_tol"]
        ):
            agent["endpoint_reacquiring"] = False
            return 0.0, 0.0

        agent["endpoint_reacquiring"] = True
        delta = endpoint["endpoint"] - shovel
        theta_des = math.atan2(float(delta[1]), float(delta[0]))
        e_theta = wrap_angle(theta_des - yaw)
        if abs(v_fwd) < STATIC_SPEED_THRESHOLD and abs(e_theta) > TURN_IN_PLACE_ANGLE:
            return 0.0, math.copysign(min(W_MAX, W_TURN_IN_PLACE), e_theta)
        w_cmd = clamp(K_THETA * e_theta, -W_MAX, W_MAX)
        align = max(0.0, math.cos(e_theta))
        v_cmd = min(V_MAX, ENDPOINT_REACQUIRE_SPEED) * align * min(
            1.0, endpoint["distance"] / 0.35
        )
        return float(v_cmd), float(w_cmd)

    agent["endpoint_reacquiring"] = False

    t_hat = np.array(agent.get("path_tangent", [math.cos(yaw), math.sin(yaw)]), dtype=float)
    if np.linalg.norm(t_hat) < 1e-9:
        t_hat = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    else:
        t_hat = t_hat / np.linalg.norm(t_hat)

    n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
    path_proj = np.array(agent.get("path_proj", shovel), dtype=float)
    e_n = float(np.dot(shovel - path_proj, n_hat))
    desired = PATH_K_T * t_hat - PATH_K_N * e_n * n_hat
    desired_norm = float(np.linalg.norm(desired))
    if desired_norm < 1e-9:
        return 0.0, 0.0
    desired = desired / desired_norm

    theta_des = math.atan2(desired[1], desired[0])
    e_theta = wrap_angle(theta_des - yaw)

    if abs(v_fwd) < STATIC_SPEED_THRESHOLD and abs(e_theta) > TURN_IN_PLACE_ANGLE:
        return 0.0, math.copysign(W_TURN_IN_PLACE, e_theta)

    w_cmd = clamp(K_THETA * e_theta, -W_MAX, W_MAX)
    align = max(0.0, math.cos(e_theta))
    dist_factor = min(1.0, max(remaining_s, dist_target) / 0.45)
    v_cmd = V_MAX * align * dist_factor

    if agent["yield_mode"] == "slow":
        v_cmd = min(
            v_cmd,
            slow_speed_limit(remaining_s, agent.get("wait_until", sim_time or 0.0), sim_time),
        )
    elif slow_target is not None:
        hold_s, wait_until = slow_target
        v_cmd = min(v_cmd, slow_speed_limit(remaining_s, wait_until, sim_time))
    elif agent["yield_mode"] == "wait" and remaining_s < 0.60:
        v_cmd *= max(0.20, remaining_s / 0.60)

    return float(v_cmd), float(w_cmd)


def update_done_state(agent, sim_time):
    if agent["done"]:
        return
    remaining = remaining_path_length(agent)
    tracking = tracking_point_for(agent)
    dist_goal = float(np.linalg.norm(tracking - agent["goal"]))
    strict_done = remaining <= PATH_STOP_S and dist_goal <= GOAL_DIST_TOL
    near_path_end_done = remaining <= GOAL_REMAINING_S_TOL and dist_goal <= GOAL_DONE_DIST_TOL
    passed_end_done = remaining <= PATH_STOP_S and endpoint_passed(agent, tracking)
    if strict_done or near_path_end_done or passed_end_done:
        agent["done"] = True
        agent["control"] = (0.0, 0.0)
        agent["emergency_stop"] = False
        agent["emergency_blocker"] = None
        print(
            f"{agent['id']} reached goal at t={sim_time:.2f}s, "
            f"actual_dist={agent['actual_dist']:.2f}m"
        )


# =========================================================
#                  CONFLICT / SCHEDULING
# =========================================================

def profile_time_at_s(agent, s_query, key):
    return interpolate_along_path(s_query, agent["geom"]["s_cum"], agent[key])


def arrival_window(agent, s_target, sim_time):
    """
    Offline-only ETA window from the rover's current path progress to s_target.

    The online speed correction is deliberately not used for conflict timing
    because instantaneous progress speed can spike while turning or slipping.
    """
    s_now = float(agent["s"])
    s_target = clamp(float(s_target), 0.0, float(agent["geom"]["total_L"]))
    if s_target <= s_now + 1e-4:
        return sim_time, sim_time + 0.10

    t_low_now = profile_time_at_s(agent, s_now, "T_lower_nodes")
    t_high_now = profile_time_at_s(agent, s_now, "T_upper_nodes")
    t_low_target = profile_time_at_s(agent, s_target, "T_lower_nodes")
    t_high_target = profile_time_at_s(agent, s_target, "T_upper_nodes")

    d_low = max(0.0, t_low_target - t_low_now)
    d_high = max(0.0, t_high_target - t_high_now)

    hold_delay = 0.0
    if (
        agent.get("yield_mode") in ("wait", "slow")
        and agent.get("hold_s") is not None
        and s_target > float(agent["hold_s"])
    ):
        hold_delay = max(0.0, float(agent.get("wait_until", 0.0)) - sim_time)

    return (
        sim_time + hold_delay + d_low,
        sim_time + hold_delay + d_high,
    )


def cell_arrival_window(agent, cell_idx, sim_time):
    """
    Cell-level ETA from the current cell to a future raw A* cell.

    The timing comes from the calibrated cell model, so conflict visualization
    and later reservations are based on the same discrete phase of planning.
    """
    windows = agent.get("cell_time_windows")
    if windows is None or len(windows) == 0:
        s_target = float(agent["s"]) + cell_relative_distance(agent, cell_idx)
        return arrival_window(agent, s_target, sim_time)

    cell_idx = max(0, min(int(cell_idx), len(windows) - 1))
    current_idx = max(0, min(int(agent.get("cell_idx", 0)), len(windows) - 1))
    if cell_idx <= current_idx:
        return sim_time, sim_time + 0.10

    cur_low, cur_high = windows[current_idx]
    target_low, target_high = windows[cell_idx]
    d_low = max(0.0, float(target_low) - float(cur_low))
    d_high = max(0.0, float(target_high) - float(cur_high))
    if d_high < d_low:
        d_high = d_low + 0.10

    hold_delay = 0.0
    target_s = float(agent["s"]) + cell_relative_distance(agent, cell_idx)
    if (
        agent.get("yield_mode") in ("wait", "slow")
        and agent.get("hold_s") is not None
        and target_s > float(agent["hold_s"])
    ):
        hold_delay = max(0.0, float(agent.get("wait_until", 0.0)) - sim_time)

    return (
        sim_time + hold_delay + d_low,
        sim_time + hold_delay + d_high,
    )


def intervals_overlap(a_low, a_high, b_low, b_high, buffer_time):
    return a_low <= b_high + buffer_time and b_low <= a_high + buffer_time


def future_segment_start(agent):
    s_cum = agent["geom"]["s_cum"]
    s_now = max(0.0, float(agent["s"]) - 0.05)
    return max(0, int(np.searchsorted(s_cum, s_now, side="right") - 1))


def project_point_to_agent_s(agent, point):
    geom = agent["geom"]
    s_now = float(agent["s"])
    s_proj, d_path, _, _ = project_point_to_path_s_windowed(
        np.array(point, dtype=float),
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        s_now,
        0.0,
        max(0.10, float(geom["total_L"]) - s_now),
    )
    return max(s_now, float(s_proj)), float(d_path)


def future_cell_lookup(agent):
    cell_path = agent.get("cell_path", [])
    if not cell_path:
        return {}

    cell_w = float(agent.get("cell_w", 0.10))
    cell_h = float(agent.get("cell_h", 0.10))
    # Each path corridor gets half of the pairwise clearance. Intersecting two
    # inflated sets then represents the full rover-rover safety width.
    corridor_radius = 0.5 * CONFLICT_DISTANCE
    cell_radius = int(math.ceil(corridor_radius / max(min(cell_w, cell_h), 1e-6)))
    start_idx = max(0, int(agent.get("cell_idx", 0)) - 1)

    lookup = {}
    for path_idx in range(start_idx, len(cell_path)):
        ix, iy = cell_path[path_idx]
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                if math.hypot(dx * cell_w, dy * cell_h) > corridor_radius:
                    continue
                key = (ix + dx, iy + dy)
                if key not in lookup:
                    lookup[key] = path_idx
    return lookup


def cell_relative_distance(agent, path_idx):
    s_cum = agent.get("cell_s_cum")
    if s_cum is None or len(s_cum) == 0:
        return 0.0
    path_idx = max(0, min(int(path_idx), len(s_cum) - 1))
    current_idx = max(0, min(int(agent.get("cell_idx", 0)), len(s_cum) - 1))
    return max(0.0, float(s_cum[path_idx] - s_cum[current_idx]))


def yielder_blocks_winner_path(winner, yielder):
    if not BLOCKING_WINNER_PATH_ENABLED:
        return False, None, None
    if winner.get("done") or yielder.get("done"):
        return False, None, None

    remaining = max(0.0, float(winner["geom"]["total_L"]) - float(winner["s"]))
    lookahead = min(
        remaining,
        max(BLOCKING_WINNER_PATH_MIN_AHEAD, BLOCKING_WINNER_PATH_LOOKAHEAD),
    )
    if lookahead <= 1e-6:
        return False, None, None
    s_proj, d_path = nearest_future_path_distance(
        winner,
        yielder["state"][:2],
        lookahead,
    )
    ahead_s = float(s_proj) - float(winner["s"])
    blocked = (
        ahead_s >= -PASSED_CONFLICT_S_BUFFER
        and d_path <= BLOCKING_WINNER_PATH_RADIUS
    )
    return bool(blocked), float(s_proj), float(d_path)


def clear_winner_ids_for_yielder(yielder):
    ids = set()
    if yielder.get("clearing_winner_path"):
        ids.update(yielder.get("clear_winner_ids", []))
    pending = yielder.get("pending_replan") or {}
    if pending.get("clear_winner_path"):
        winner_id = pending.get("winner_id")
        if winner_id is not None:
            ids.add(winner_id)
        for blocker in pending.get("blockers", []):
            blocker_id = blocker.get("id")
            if blocker_id is not None:
                ids.add(blocker_id)
    return ids


def is_clearing_path_of(yielder, winner):
    if not yielder.get("clearing_winner_path"):
        return False
    return winner.get("id") in set(yielder.get("clear_winner_ids", []))


def apply_clear_path_estop_guards(agents, sim_time=None):
    by_id = {agent["id"]: agent for agent in agents}
    for yielder in agents:
        clear_ids = clear_winner_ids_for_yielder(yielder)
        if not clear_ids:
            yielder["clearing_winner_path"] = False
            yielder["clear_winner_ids"] = []
            continue

        still_blocking = []
        for winner_id in clear_ids:
            winner = by_id.get(winner_id)
            if winner is None or winner.get("done") or yielder.get("done"):
                continue
            blocked, blocker_s, _ = yielder_blocks_winner_path(winner, yielder)
            if not blocked:
                continue
            still_blocking.append(winner_id)

            ahead_s = float(blocker_s) - float(winner["s"])
            euclid = float(np.linalg.norm(np.array(winner["state"][:2]) - np.array(yielder["state"][:2])))
            if ahead_s <= BLOCKING_FREEZE_AHEAD_S or euclid <= BLOCKING_FREEZE_DIST:
                set_emergency_stop(
                    winner,
                    yielder,
                    "clear_path_guard",
                    sim_time,
                    ahead=ahead_s,
                    dist=euclid,
                )

        if yielder.get("clearing_winner_path") and not still_blocking:
            yielder["clearing_winner_path"] = False
            yielder["clear_winner_ids"] = []


def clear_backoff_escape(agent):
    agent["backoff_until"] = 0.0
    agent["backoff_from_id"] = None
    agent["backoff_from_xy"] = None
    agent["backoff_reason"] = None
    agent["backoff_started_at"] = None
    agent["backoff_start_xy"] = None
    agent["backoff_best_dist"] = 0.0
    agent["backoff_last_progress_at"] = None


def trigger_backoff_escape(yielder, winner, sim_time, reason, dist):
    active = yielder.get("backoff_from_id") == winner["id"] and sim_time <= float(yielder.get("backoff_until", 0.0))
    if sim_time < float(yielder.get("backoff_failed_until", -1e9)):
        return
    if (
        not active
        and sim_time - float(yielder.get("backoff_last_trigger_time", -1e9)) < BACKOFF_COOLDOWN
    ):
        return

    yielder["backoff_until"] = max(
        float(yielder.get("backoff_until", 0.0)),
        float(sim_time) + BACKOFF_DURATION,
    )
    yielder["backoff_from_id"] = winner["id"]
    yielder["backoff_from_xy"] = np.array(winner["state"][:2], dtype=float)
    yielder["backoff_reason"] = reason
    if not active:
        yielder["backoff_last_trigger_time"] = float(sim_time)
        yielder["backoff_started_at"] = float(sim_time)
        yielder["backoff_start_xy"] = np.array(yielder["state"][:2], dtype=float)
        yielder["backoff_best_dist"] = float(dist)
        yielder["backoff_last_progress_at"] = float(sim_time)
        if BACKOFF_DEBUG_PRINT:
            print(
                f"t={sim_time:.2f}s {yielder['id']} backs away from {winner['id']} "
                f"({reason}, dist={dist:.2f}m)."
            )


def update_close_pair_backoff(agents, sim_time):
    if not BACKOFF_ESCAPE_ENABLED or sim_time is None:
        return

    by_id = {agent["id"]: agent for agent in agents}

    for agent in agents:
        blocker_id = agent.get("backoff_from_id")
        if blocker_id is None:
            continue
        blocker = by_id.get(blocker_id)
        if blocker is None or agent.get("done") or blocker.get("done"):
            clear_backoff_escape(agent)
            continue
        dist = float(np.linalg.norm(np.array(agent["state"][:2]) - np.array(blocker["state"][:2])))
        agent["backoff_from_xy"] = np.array(blocker["state"][:2], dtype=float)
        if dist >= BACKOFF_RELEASE_DIST:
            clear_backoff_escape(agent)
        else:
            best_dist = float(agent.get("backoff_best_dist", dist))
            if dist >= best_dist + BACKOFF_PROGRESS_EPS:
                agent["backoff_best_dist"] = dist
                agent["backoff_last_progress_at"] = float(sim_time)
            started_at = agent.get("backoff_started_at")
            last_progress_at = agent.get("backoff_last_progress_at")
            start_xy = agent.get("backoff_start_xy")
            elapsed = float(sim_time) - float(started_at if started_at is not None else sim_time)
            no_progress = float(sim_time) - float(last_progress_at if last_progress_at is not None else sim_time)
            moved = (
                float(np.linalg.norm(np.array(agent["state"][:2], dtype=float) - np.array(start_xy, dtype=float)))
                if start_xy is not None else 0.0
            )
            if (
                elapsed >= BACKOFF_DURATION
                or moved >= BACKOFF_MAX_DISTANCE
                or no_progress >= BACKOFF_PROGRESS_TIMEOUT
            ):
                if BACKOFF_DEBUG_PRINT:
                    print(
                        f"t={sim_time:.2f}s {agent['id']} bounded backoff aborted "
                        f"(elapsed={elapsed:.2f}s, moved={moved:.2f}m, progress_age={no_progress:.2f}s)."
                    )
                clear_backoff_escape(agent)
                agent["backoff_failed_until"] = float(sim_time) + BACKOFF_FAILURE_COOLDOWN

    for i in range(len(agents)):
        for j in range(i + 1, len(agents)):
            a = agents[i]
            b = agents[j]
            if a.get("done") or b.get("done"):
                continue
            if abs(float(a.get("priority", 0.0)) - float(b.get("priority", 0.0))) < 1e-9:
                continue

            winner, yielder = (a, b) if float(a["priority"]) < float(b["priority"]) else (b, a)
            dist = float(np.linalg.norm(np.array(winner["state"][:2]) - np.array(yielder["state"][:2])))
            if dist > BACKOFF_TRIGGER_DIST:
                continue

            winner_stopped_by_yielder = (
                winner.get("emergency_stop")
                and winner.get("emergency_blocker") == yielder["id"]
            )
            yielder_stopped_by_winner = (
                yielder.get("emergency_stop")
                and yielder.get("emergency_blocker") == winner["id"]
            )
            clearing_pair = winner["id"] in clear_winner_ids_for_yielder(yielder)
            if winner_stopped_by_yielder:
                reason = "winner_estop"
            elif yielder_stopped_by_winner:
                reason = "yielder_estop"
            elif clearing_pair:
                reason = "clear_path"
            else:
                continue

            trigger_backoff_escape(yielder, winner, sim_time, reason, dist)


def compute_backoff_control(agent, sim_time):
    if not BACKOFF_ESCAPE_ENABLED or sim_time is None:
        return None
    if sim_time > float(agent.get("backoff_until", 0.0)):
        if agent.get("backoff_from_id") is not None:
            clear_backoff_escape(agent)
        return None

    blocker_xy = agent.get("backoff_from_xy")
    if blocker_xy is None:
        clear_backoff_escape(agent)
        return None

    x, y, yaw, _, _ = agent["state"]
    delta = np.array(blocker_xy, dtype=float) - np.array([x, y], dtype=float)
    dist = float(np.linalg.norm(delta))
    if dist >= BACKOFF_RELEASE_DIST:
        clear_backoff_escape(agent)
        return None
    if dist < 1e-9:
        return -BACKOFF_SPEED, 0.0

    heading_to_blocker = math.atan2(float(delta[1]), float(delta[0]))
    heading_err = wrap_angle(heading_to_blocker - float(yaw))
    align = max(0.0, math.cos(heading_err))
    if align <= 1e-3:
        v_cmd = 0.0
    else:
        v_cmd = -BACKOFF_SPEED * max(BACKOFF_MIN_REVERSE_ALIGN, align)
    w_cmd = clamp(K_THETA * heading_err, -BACKOFF_W_MAX, BACKOFF_W_MAX)
    return float(v_cmd), float(w_cmd)


def nearest_cell_index_to_point(agent, point_xy):
    pts = agent.get("cell_points")
    if pts is None or len(pts) == 0:
        return int(agent.get("cell_idx", 0))
    point_xy = np.array(point_xy, dtype=float)
    current_idx = max(0, min(int(agent.get("cell_idx", 0)), len(pts) - 1))
    start = max(0, current_idx - 5)
    end = min(len(pts), current_idx + 80)
    if start >= end:
        start, end = 0, len(pts)
    local = pts[start:end]
    d2 = np.sum((local - point_xy) ** 2, axis=1)
    return start + int(np.argmin(d2))


def direct_blocking_conflict(agent_i, agent_j, idx_i, idx_j, sim_time):
    if agent_i["priority"] <= agent_j["priority"]:
        winner = agent_i
        yielder = agent_j
        winner_idx = idx_i
        yielder_idx = idx_j
    else:
        winner = agent_j
        yielder = agent_i
        winner_idx = idx_j
        yielder_idx = idx_i

    blocking_path, blocker_s, d_path = yielder_blocks_winner_path(winner, yielder)
    if not blocking_path:
        return None

    winner_point = point_at_s(winner["geom"], blocker_s)
    yielder_point = np.array(yielder["state"][:2], dtype=float)
    center = 0.5 * (winner_point + yielder_point)

    winner_cell_idx = nearest_cell_index_to_point(winner, winner_point)
    yielder_cell_idx = nearest_cell_index_to_point(yielder, yielder_point)
    winner_low, winner_high = cell_arrival_window(winner, winner_cell_idx, sim_time)
    yielder_low, yielder_high = sim_time, sim_time + 0.25

    if winner_idx == idx_i:
        p_i, p_j = winner_point, yielder_point
        s_i, s_j = float(blocker_s), float(yielder["s"])
        t_i_low, t_i_high = winner_low, winner_high
        t_j_low, t_j_high = yielder_low, yielder_high
        cell_idx_i, cell_idx_j = winner_cell_idx, yielder_cell_idx
    else:
        p_i, p_j = yielder_point, winner_point
        s_i, s_j = float(yielder["s"]), float(blocker_s)
        t_i_low, t_i_high = yielder_low, yielder_high
        t_j_low, t_j_high = winner_low, winner_high
        cell_idx_i, cell_idx_j = yielder_cell_idx, winner_cell_idx

    winner_exit_s = min(
        float(winner["geom"]["total_L"]),
        float(blocker_s) + CONFLICT_ZONE_HALF_S,
    )
    _, winner_exit_high = arrival_window(winner, winner_exit_s, sim_time)
    release_time = winner_exit_high + WAIT_RELEASE_BUFFER
    wait_delay = max(0.0, release_time - yielder_low)

    return TrajectoryConflict(
        i=idx_i,
        j=idx_j,
        s_i=s_i,
        s_j=s_j,
        p_i=p_i,
        p_j=p_j,
        center=center,
        distance=float(d_path),
        t_i_low=t_i_low,
        t_i_high=t_i_high,
        t_j_low=t_j_low,
        t_j_high=t_j_high,
        winner_idx=winner_idx,
        yielder_idx=yielder_idx,
        winner_s=float(blocker_s),
        yielder_s=float(yielder["s"]),
        release_time=release_time,
        wait_delay=wait_delay,
        cell_idx_i=int(cell_idx_i),
        cell_idx_j=int(cell_idx_j),
        time_overlap=False,
        blocking_winner_path=True,
    )


def pair_conflict(agent_i, agent_j, idx_i, idx_j, sim_time, require_time_overlap=True):
    direct_blocker = direct_blocking_conflict(agent_i, agent_j, idx_i, idx_j, sim_time)

    lookup_i = future_cell_lookup(agent_i)
    lookup_j = future_cell_lookup(agent_j)
    if not lookup_i or not lookup_j:
        return direct_blocker

    common_cells = set(lookup_i).intersection(lookup_j)
    if not common_cells:
        return direct_blocker

    pts_i = agent_i.get("cell_points")
    pts_j = agent_j.get("cell_points")
    if pts_i is None or pts_j is None or len(pts_i) == 0 or len(pts_j) == 0:
        return None

    max_cell_diag = max(
        math.hypot(float(agent_i.get("cell_w", 0.10)), float(agent_i.get("cell_h", 0.10))),
        math.hypot(float(agent_j.get("cell_w", 0.10)), float(agent_j.get("cell_h", 0.10))),
    )
    best = None
    best_score = float("inf")

    for cell in common_cells:
        cell_idx_i = lookup_i[cell]
        cell_idx_j = lookup_j[cell]
        pi = pts_i[cell_idx_i]
        pj = pts_j[cell_idx_j]
        dist = float(np.linalg.norm(pi - pj))
        if dist > CONFLICT_DISTANCE + 0.5 * max_cell_diag:
            continue

        center = 0.5 * (pi + pj)
        s_i = float(agent_i["s"]) + cell_relative_distance(agent_i, cell_idx_i)
        s_j = float(agent_j["s"]) + cell_relative_distance(agent_j, cell_idx_j)
        if (
            s_i < agent_i["s"] - PASSED_CONFLICT_S_BUFFER
            or s_j < agent_j["s"] - PASSED_CONFLICT_S_BUFFER
        ):
            continue

        if agent_i["priority"] <= agent_j["priority"]:
            winner_idx = idx_i
            yielder_idx = idx_j
            winner_s = s_i
            yielder_s = s_j
            winner = agent_i
            yielder = agent_j
            yielder_low_key = "j"
        else:
            winner_idx = idx_j
            yielder_idx = idx_i
            winner_s = s_j
            yielder_s = s_i
            winner = agent_j
            yielder = agent_i
            yielder_low_key = "i"

        ti_low, ti_high = cell_arrival_window(agent_i, cell_idx_i, sim_time)
        tj_low, tj_high = cell_arrival_window(agent_j, cell_idx_j, sim_time)
        time_overlap = intervals_overlap(
            ti_low,
            ti_high,
            tj_low,
            tj_high,
            CONFLICT_TIME_BUFFER,
        )
        blocking_path, blocker_s, _ = yielder_blocks_winner_path(winner, yielder)
        if blocking_path and blocker_s is not None:
            winner_s = max(float(winner_s), float(blocker_s))

        if require_time_overlap and not (time_overlap or blocking_path):
            continue

        yielder_low = tj_low if yielder_low_key == "j" else ti_low

        if float(winner["s"]) >= winner_s + CONFLICT_ZONE_HALF_S:
            continue

        winner_exit_s = min(
            float(winner["geom"]["total_L"]),
            winner_s + CONFLICT_ZONE_HALF_S,
        )
        _, winner_exit_high = arrival_window(winner, winner_exit_s, sim_time)
        release_time = winner_exit_high + WAIT_RELEASE_BUFFER
        wait_delay = max(0.0, release_time - yielder_low)

        time_gap = 0.0
        if not time_overlap:
            time_gap = min(abs(ti_low - tj_high), abs(tj_low - ti_high))
        score = max(0.5 * (ti_low + ti_high), 0.5 * (tj_low + tj_high)) + 10.0 * time_gap
        if score < best_score:
            best_score = score
            best = TrajectoryConflict(
                i=idx_i,
                j=idx_j,
                s_i=s_i,
                s_j=s_j,
                p_i=pi,
                p_j=pj,
                center=center,
                distance=float(dist),
                t_i_low=ti_low,
                t_i_high=ti_high,
                t_j_low=tj_low,
                t_j_high=tj_high,
                winner_idx=winner_idx,
                yielder_idx=yielder_idx,
                winner_s=winner_s,
                yielder_s=yielder_s,
                release_time=release_time,
                wait_delay=wait_delay,
                cell_idx_i=int(cell_idx_i),
                cell_idx_j=int(cell_idx_j),
                time_overlap=bool(time_overlap),
                blocking_winner_path=bool(blocking_path),
            )

    return direct_blocker if direct_blocker is not None else best


def find_active_conflicts(agents, sim_time):
    conflicts = []
    for i in range(len(agents)):
        if agents[i]["done"]:
            continue
        for j in range(i + 1, len(agents)):
            if agents[j]["done"]:
                continue
            conflict = pair_conflict(agents[i], agents[j], i, j, sim_time, require_time_overlap=True)
            if conflict is not None:
                conflicts.append(conflict)
    conflicts.sort(key=lambda c: max(c.t_i_low, c.t_j_low))
    return conflicts


def find_estop_blocking_conflicts(agents, sim_time):
    conflicts = []
    id_to_idx = {agent["id"]: idx for idx, agent in enumerate(agents)}
    seen = set()
    for winner_idx, winner in enumerate(agents):
        if winner.get("done") or not winner.get("emergency_stop"):
            continue

        blocker_id = winner.get("emergency_blocker")
        if blocker_id is None or blocker_id not in id_to_idx:
            continue
        yielder_idx = id_to_idx[blocker_id]
        if yielder_idx == winner_idx:
            continue

        yielder = agents[yielder_idx]
        if yielder.get("done"):
            continue
        if float(winner["priority"]) > float(yielder["priority"]):
            continue
        if is_clearing_path_of(yielder, winner):
            continue

        key = (winner_idx, yielder_idx)
        if key in seen:
            continue
        seen.add(key)

        conflict = direct_blocking_conflict(winner, yielder, winner_idx, yielder_idx, sim_time)
        if conflict is not None:
            conflicts.append(conflict)
    return conflicts


def find_visual_conflicts(agents, sim_time):
    conflicts = []
    for i in range(len(agents)):
        if agents[i]["done"]:
            continue
        for j in range(i + 1, len(agents)):
            if agents[j]["done"]:
                continue
            conflict = pair_conflict(agents[i], agents[j], i, j, sim_time, require_time_overlap=False)
            if conflict is None:
                continue
            if conflict.time_overlap or conflict.blocking_winner_path or DRAW_NONOVERLAP_CONFLICT_VISUALS:
                conflicts.append(conflict)
    conflicts.sort(key=lambda c: (not (c.time_overlap or c.blocking_winner_path), max(c.t_i_low, c.t_j_low)))
    return conflicts[:CONFLICT_VISUAL_MAX_ITEMS]


def reservation_obstacles_for_conflict(winner, winner_s):
    geom = winner["geom"]
    winner_s = float(winner_s)
    # Bias the temporary obstacle forward along the winner's motion. This makes
    # the area the winner has already passed relatively cheaper for A*.
    s0 = max(0.0, winner_s - RESERVATION_BEHIND_PATH)
    s1 = min(float(geom["total_L"]), winner_s + RESERVATION_AHEAD_PATH)
    if s1 < s0:
        s0, s1 = s1, s0

    points = []
    count = max(2, int(math.ceil((s1 - s0) / RESERVATION_POINT_SPACING)) + 1)
    for k in range(count):
        s = s0 + (s1 - s0) * (k / max(1, count - 1))
        center = point_at_s(geom, s)
        tangent = tangent_at_s(geom, s)
        normal = np.array([-tangent[1], tangent[0]], dtype=float)
        for layer in RESERVATION_LATERAL_LAYERS:
            offset = float(layer) * RESERVATION_HALF_WIDTH
            q = center + offset * normal
            points.append((float(q[0]), float(q[1])))
    return points


def proxy_disk_obstacles(center_xy, radius, spacing):
    center_xy = np.array(center_xy, dtype=float)
    radius = float(radius)
    spacing = max(float(spacing), 1e-3)
    points = []
    steps = int(math.ceil(radius / spacing))
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            offset = np.array([ix * spacing, iy * spacing], dtype=float)
            if float(np.linalg.norm(offset)) <= radius + 1e-9:
                q = center_xy + offset
                points.append((float(q[0]), float(q[1])))
    return points


def winner_blocking_corridor_obstacles(winner, winner_s, yielder_start_xy):
    geom = winner["geom"]
    yielder_start_xy = np.array(yielder_start_xy, dtype=float)
    winner_s = float(winner_s)
    s0 = max(
        float(winner["s"]),
        winner_s - CONFLICT_ZONE_HALF_S - 0.20,
    )
    s1 = min(
        float(geom["total_L"]),
        winner_s + CONFLICT_ZONE_HALF_S + BLOCKING_WINNER_PATH_EXTRA_AHEAD,
    )
    if s1 < s0:
        s0, s1 = s1, s0

    points = []
    count = max(2, int(math.ceil((s1 - s0) / BLOCKING_CORRIDOR_POINT_SPACING)) + 1)
    for k in range(count):
        s = s0 + (s1 - s0) * (k / max(1, count - 1))
        center = point_at_s(geom, s)
        tangent = tangent_at_s(geom, s)
        normal = np.array([-tangent[1], tangent[0]], dtype=float)
        for layer in BLOCKING_CORRIDOR_LAYERS:
            q = center + float(layer) * BLOCKING_CORRIDOR_HALF_WIDTH * normal
            if np.linalg.norm(q - yielder_start_xy) <= BLOCKING_START_ESCAPE_RADIUS:
                continue
            points.append((float(q[0]), float(q[1])))

    winner_xy = np.array(winner["state"][:2], dtype=float)
    for q in proxy_disk_obstacles(
        winner_xy,
        BLOCKING_WINNER_STATIC_RADIUS,
        BLOCKING_CORRIDOR_POINT_SPACING,
    ):
        if np.linalg.norm(np.array(q, dtype=float) - yielder_start_xy) <= BLOCKING_WINNER_STATIC_ESCAPE_RADIUS:
            continue
        points.append(q)

    return points


def hard_clear_obstacles_for_conflicts(agents, conflicts, yielder):
    yielder_start_xy = np.array(yielder["state"][:2], dtype=float)
    points = []
    for conflict in conflicts:
        if not conflict.blocking_winner_path:
            continue
        winner = agents[conflict.winner_idx]
        points.extend(
            winner_blocking_corridor_obstacles(
                winner,
                conflict.winner_s,
                yielder_start_xy,
            )
        )
    return points


def reservation_obstacles_for_conflicts(agents, conflicts):
    points = []
    for conflict in conflicts:
        winner = agents[conflict.winner_idx]
        if float(winner["s"]) >= float(conflict.winner_s) + CONFLICT_ZONE_HALF_S:
            continue
        points.extend(reservation_obstacles_for_conflict(winner, conflict.winner_s))
    return points


def try_replan_yielder(yielder, winner, conflict, env_radius, sim_time):
    if sim_time - float(yielder.get("last_replan_time", -1e9)) < REPLAN_COOLDOWN:
        return False

    reservation = reservation_obstacles_for_conflict(winner, conflict.winner_s)
    start_xy = np.array(yielder["state"][:2], dtype=float)
    start_yaw = float(yielder["state"][2])
    plan = build_reservation_replan(
        yielder,
        env_radius,
        reservation,
        start_xy,
        start_yaw,
    )
    if plan is None:
        return False

    new_len = float(plan["geom"]["total_L"])
    old_remaining = remaining_path_length(yielder)
    if new_len > REPLAN_MAX_LENGTH_FACTOR * old_remaining + REPLAN_EXTRA_ALLOWANCE:
        return False

    install_plan(yielder, plan, sim_time)
    yielder["yield_mode"] = "replan"
    yielder["hold_s"] = None
    yielder["wait_until"] = 0.0
    yielder["yield_winner_idx"] = None
    yielder["winner_release_s"] = None
    print(
        f"t={sim_time:.2f}s {yielder['id']} replanned around "
        f"{winner['id']}'s reserved conflict zone. new_path={new_len:.2f}m"
    )
    return True


def clear_pending_replan(agent, cancel=True):
    future = agent.get("replan_future")
    if cancel and future is not None and not future.done():
        future.cancel()
    agent["replan_future"] = None
    agent["pending_replan"] = None


def planning_snapshot_for_agent(agent):
    return {
        "id": agent["id"],
        "goal": np.array(agent["goal"], dtype=float).copy(),
        "hard_static_pebbles": list(agent.get("hard_static_pebbles", [])),
        "eta_scale_lower": float(agent.get("eta_scale_lower", ETA_TIME_SCALE_LOWER)),
        "eta_scale_upper": float(agent.get("eta_scale_upper", ETA_TIME_SCALE_UPPER)),
    }


def pending_winner_has_cleared(agents, pending, sim_time):
    if pending.get("clear_winner_path"):
        return False

    blockers = pending.get("blockers", [])
    if blockers:
        all_cleared = True
        for blocker in blockers:
            idx = blocker.get("idx")
            if idx is None or idx < 0 or idx >= len(agents):
                continue
            winner = agents[idx]
            if winner.get("done"):
                continue
            if float(winner["s"]) < float(blocker.get("release_s", 0.0)):
                all_cleared = False
                break
        return all_cleared or sim_time >= float(pending.get("release_time", 0.0))

    winner_idx = pending.get("winner_idx")
    if winner_idx is None or winner_idx < 0 or winner_idx >= len(agents):
        return True
    winner = agents[winner_idx]
    if winner.get("done"):
        return True
    if float(winner["s"]) >= float(pending["winner_release_s"]):
        return True
    return sim_time >= float(pending["release_time"])


def set_yield_hold(agent,
                   hold_s,
                   release_time,
                   winner_idx,
                   winner_release_s,
                   blockers,
                   sim_time):
    hold_s = max(float(agent["s"]), float(hold_s))
    release_time = float(release_time)
    use_slow = (
        SLOW_YIELD_ENABLED
        and math.isfinite(release_time)
        and release_time > float(sim_time) + SLOW_YIELD_TIME_EPS
        and hold_s > float(agent["s"]) + SLOW_YIELD_STOP_MARGIN
    )

    agent["yield_mode"] = "slow" if use_slow else "wait"
    agent["hold_s"] = hold_s
    agent["wait_until"] = release_time
    agent["yield_winner_idx"] = winner_idx
    agent["winner_release_s"] = winner_release_s
    agent["yield_blockers"] = list(blockers)
    return agent["yield_mode"]


def set_wait_from_pending(agent, agents, pending, sim_time):
    winner_idx = pending.get("winner_idx")
    winner_id = pending.get("winner_id", "winner")
    mode = set_yield_hold(
        agent,
        pending["hold_s"],
        pending["release_time"],
        winner_idx,
        float(pending["winner_release_s"]),
        pending.get("blockers", []),
        sim_time,
    )
    print(
        f"t={sim_time:.2f}s {agent['id']} {mode}s while background replan "
        f"finishes; yielding to {winner_id} at hold_s={agent['hold_s']:.2f}"
    )


def distance_from_current_to_candidate(agent, plan):
    state = agent["state"]
    shovel = tracking_point_for(agent, state)
    geom = plan["geom"]
    _, d_path, _, _ = project_point_to_path_s_windowed(
        shovel,
        geom["pts"],
        geom["segs"],
        geom["seg_lens"],
        geom["s_cum"],
        0.0,
        0.0,
        max(PROGRESS_LOOKAHEAD_M, 2.0),
    )
    return float(d_path)


def collect_replan_jobs(agents, sim_time):
    for agent in agents:
        future = agent.get("replan_future")
        pending = agent.get("pending_replan")
        if future is None or pending is None:
            continue

        if pending_winner_has_cleared(agents, pending, sim_time):
            winner_id = pending.get("winner_id", "winner")
            clear_pending_replan(agent)
            if agent.get("yield_mode") in ("replan_pending", "wait", "slow"):
                agent["yield_mode"] = "normal"
                agent["hold_s"] = None
                agent["wait_until"] = 0.0
                agent["yield_winner_idx"] = None
                agent["winner_release_s"] = None
                agent["yield_blockers"] = []
            print(
                f"t={sim_time:.2f}s discarded {agent['id']}'s pending replan: "
                f"{winner_id} already cleared the conflict zone."
            )
            continue

        if not future.done():
            if pending.get("clear_winner_path"):
                continue
            if (
                agent.get("yield_mode") == "replan_pending"
                and float(agent["s"]) >= float(pending["hold_s"]) - REPLAN_PENDING_HOLD_MARGIN
            ):
                set_wait_from_pending(agent, agents, pending, sim_time)
            continue

        clear_pending_replan(agent, cancel=False)
        try:
            plan = future.result()
        except Exception as exc:
            print(f"t={sim_time:.2f}s {agent['id']} background replan failed: {exc}")
            if pending.get("clear_winner_path"):
                agent["yield_mode"] = "normal"
                agent["hold_s"] = None
                agent["wait_until"] = 0.0
                agent["yield_winner_idx"] = None
                agent["winner_release_s"] = None
                agent["yield_blockers"] = []
                continue
            set_wait_from_pending(agent, agents, pending, sim_time)
            continue

        if plan is None:
            print(f"t={sim_time:.2f}s {agent['id']} background replan found no route.")
            if pending.get("clear_winner_path"):
                agent["yield_mode"] = "normal"
                agent["hold_s"] = None
                agent["wait_until"] = 0.0
                agent["yield_winner_idx"] = None
                agent["winner_release_s"] = None
                agent["yield_blockers"] = []
                continue
            set_wait_from_pending(agent, agents, pending, sim_time)
            continue

        new_len = float(plan["geom"]["total_L"])
        old_remaining = remaining_path_length(agent)
        is_clear_replan = bool(pending.get("clear_winner_path"))
        if (
            not is_clear_replan
            and new_len > REPLAN_MAX_LENGTH_FACTOR * old_remaining + REPLAN_EXTRA_ALLOWANCE
        ):
            print(
                f"t={sim_time:.2f}s {agent['id']} rejected long replan "
                f"({new_len:.2f}m vs remaining {old_remaining:.2f}m)."
            )
            agent["yield_mode"] = "normal"
            agent["hold_s"] = None
            agent["wait_until"] = 0.0
            agent["yield_winner_idx"] = None
            agent["winner_release_s"] = None
            agent["yield_blockers"] = []
            continue

        current_dist = distance_from_current_to_candidate(agent, plan)
        accept_dist = max(REPLAN_ACCEPT_DIST, 1.25) if is_clear_replan else REPLAN_ACCEPT_DIST
        if current_dist > accept_dist:
            print(
                f"t={sim_time:.2f}s {agent['id']} rejected stale replan "
                f"(current distance to candidate path {current_dist:.2f}m)."
            )
            agent["yield_mode"] = "normal"
            agent["hold_s"] = None
            agent["wait_until"] = 0.0
            agent["yield_winner_idx"] = None
            agent["winner_release_s"] = None
            agent["yield_blockers"] = []
            continue

        if not is_clear_replan:
            time_ok, _ = candidate_start_direction_is_reasonable(
                agent,
                plan,
                START_DIRECTION_REPLAN_MARGIN,
                "background replan",
                sim_time,
            )
            if not time_ok:
                set_wait_from_pending(agent, agents, pending, sim_time)
                continue

        install_plan(agent, plan, sim_time)
        if is_clear_replan:
            blockers = pending.get("blockers", [])
            clear_ids = [b["id"] for b in blockers if b.get("id") is not None]
            if pending.get("winner_id") is not None:
                clear_ids.append(pending["winner_id"])
            agent["clearing_winner_path"] = True
            agent["clear_winner_ids"] = sorted(set(clear_ids))
        refresh_agent_debug_drawings(agent)
        agent["yield_mode"] = "replan"
        agent["hold_s"] = None
        agent["wait_until"] = 0.0
        agent["yield_winner_idx"] = None
        agent["winner_release_s"] = None
        agent["yield_blockers"] = []
        print(
            f"t={sim_time:.2f}s {agent['id']} installed background replan "
            f"around {pending.get('winner_id', 'winner')}. new_path={new_len:.2f}m"
        )


def refresh_agent_debug_drawings(agent):
    if p.isConnected():
        redraw_agent_plan(agent)
        draw_replan_highlight(agent)


def cell_path_change_ratio(agent, new_cell_path):
    current_future = list(agent.get("cell_path", []))[int(agent.get("cell_idx", 0)):]
    if len(current_future) < PATH_REFRESH_MIN_CELLS or len(new_cell_path) < PATH_REFRESH_MIN_CELLS:
        return 0.0

    current_set = set(current_future)
    new_set = set(new_cell_path)
    overlap = len(current_set.intersection(new_set))
    denom = max(1, min(len(current_set), len(new_set)))
    return 1.0 - overlap / denom


def install_cell_plan(agent, cell_plan):
    agent["raw_path"] = cell_plan["raw_path"]
    agent["cell_path"] = cell_plan["cell_path"]
    agent["cell_points"] = cell_plan["cell_points"]
    agent["cell_s_cum"] = cell_plan["cell_s_cum"]
    agent["cell_time_windows"] = cell_plan.get(
        "cell_time_windows",
        estimate_windows_for_cell_path(cell_plan["cell_points"], 0.0),
    )
    agent["time_reserved_holds"] = list(cell_plan.get("time_reserved_holds", []))
    agent["cell_w"] = cell_plan["cell_w"]
    agent["cell_h"] = cell_plan["cell_h"]
    agent["cell_idx"] = nearest_index_in_points(
        agent["cell_points"],
        np.array(agent["state"][:2], dtype=float),
        start_idx=int(agent.get("cell_idx", 0)),
    )


def request_smooth_refresh(agent, env_radius, sim_time, executor):
    if executor is None:
        return False
    if agent.get("route_refresh_future") is not None or agent.get("replan_future") is not None:
        return False
    if sim_time - float(agent.get("last_route_refresh_time", -1e9)) < TRAJECTORY_REBUILD_COOLDOWN:
        return False

    start_xy = predicted_replan_start_xy(agent)
    start_yaw = float(agent["state"][2])
    snapshot = planning_snapshot_for_agent(agent)
    agent["route_refresh_future"] = executor.submit(
        build_reservation_replan,
        snapshot,
        env_radius,
        [],
        start_xy,
        start_yaw,
    )
    agent["last_route_refresh_time"] = sim_time
    return True


def collect_cell_refresh_jobs(agents, env_radius, sim_time, executor):
    for agent in agents:
        future = agent.get("cell_refresh_future")
        if future is None:
            continue
        if not future.done():
            continue

        agent["cell_refresh_future"] = None
        try:
            cell_plan = future.result()
        except Exception as exc:
            print(f"t={sim_time:.2f}s {agent['id']} raw-cell refresh failed: {exc}")
            continue
        if cell_plan is None:
            continue

        change = cell_path_change_ratio(agent, cell_plan["cell_path"])
        install_cell_plan(agent, cell_plan)

        if change >= PATH_REFRESH_CHANGE_RATIO and agent.get("yield_mode") in ("normal", "replan", "slow"):
            if request_smooth_refresh(agent, env_radius, sim_time, executor):
                print(
                    f"t={sim_time:.2f}s {agent['id']} raw A* cells changed "
                    f"(change={change:.2f}); requested smooth trajectory rebuild."
                )


def collect_route_refresh_jobs(agents, sim_time):
    for agent in agents:
        future = agent.get("route_refresh_future")
        if future is None:
            continue
        if not future.done():
            continue

        agent["route_refresh_future"] = None
        try:
            plan = future.result()
        except Exception as exc:
            print(f"t={sim_time:.2f}s {agent['id']} route refresh failed: {exc}")
            continue
        if plan is None:
            continue

        if agent.get("yield_mode") not in ("normal", "replan", "slow"):
            continue

        current_dist = distance_from_current_to_candidate(agent, plan)
        if current_dist > REPLAN_ACCEPT_DIST:
            continue

        time_ok, _ = candidate_start_direction_is_reasonable(
            agent,
            plan,
            START_DIRECTION_ROUTE_REFRESH_MARGIN,
            "periodic route refresh",
            sim_time,
        )
        if not time_ok:
            continue

        install_plan(agent, plan, sim_time)
        refresh_agent_debug_drawings(agent)
        agent["yield_mode"] = "normal"
        print(
            f"t={sim_time:.2f}s {agent['id']} installed periodic route refresh "
            f"(path={float(plan['geom']['total_L']):.2f}m)."
        )


def start_periodic_cell_refresh_jobs(agents, env_radius, sim_time, executor):
    if executor is None:
        return
    for agent in agents:
        if agent.get("done"):
            continue
        if agent.get("yield_mode") not in ("normal", "replan", "replan_pending", "wait", "slow"):
            continue
        if agent.get("cell_refresh_future") is not None:
            continue
        if sim_time - float(agent.get("last_cell_refresh_time", -1e9)) < CELL_REFRESH_DT:
            continue

        start_xy = np.array(agent["state"][:2], dtype=float)
        snapshot = planning_snapshot_for_agent(agent)
        agent["cell_refresh_future"] = executor.submit(
            build_raw_cell_plan,
            snapshot,
            env_radius,
            [],
            start_xy,
        )
        agent["last_cell_refresh_time"] = sim_time


def start_background_replan_yielder(yielder, winner, conflict, env_radius, sim_time, executor):
    if executor is None:
        return False

    future = yielder.get("replan_future")
    if future is not None and not future.done():
        return True

    if sim_time - float(yielder.get("last_replan_request_time", -1e9)) < REPLAN_COOLDOWN:
        return False

    reservation = reservation_obstacles_for_conflict(winner, conflict.winner_s)
    start_xy = predicted_replan_start_xy(yielder)
    start_yaw = float(yielder["state"][2])
    yielder_snapshot = planning_snapshot_for_agent(yielder)
    future = executor.submit(
        build_reservation_replan,
        yielder_snapshot,
        env_radius,
        reservation,
        start_xy,
        start_yaw,
    )

    hold_s = max(float(yielder["s"]), float(conflict.yielder_s) - HOLD_BACK_DISTANCE)
    winner_release_s = min(
        float(winner["geom"]["total_L"]),
        float(conflict.winner_s) + CONFLICT_ZONE_HALF_S,
    )
    yielder["replan_future"] = future
    yielder["pending_replan"] = {
        "winner_idx": conflict.winner_idx,
        "winner_id": winner["id"],
        "winner_s": float(conflict.winner_s),
        "winner_release_s": winner_release_s,
        "yielder_s": float(conflict.yielder_s),
        "hold_s": hold_s,
        "release_time": float(conflict.release_time),
        "requested_at": sim_time,
    }
    yielder["yield_mode"] = "replan_pending"
    yielder["last_replan_request_time"] = sim_time
    print(
        f"t={sim_time:.2f}s {yielder['id']} started background replan "
        f"around {winner['id']}; continuing current trajectory for now."
    )
    return True


def start_background_replan_for_conflicts(yielder_idx, conflicts, agents, env_radius, sim_time, executor):
    if executor is None or not conflicts:
        return False

    yielder = agents[yielder_idx]
    future = yielder.get("replan_future")
    if future is not None and not future.done():
        return True

    if sim_time - float(yielder.get("last_replan_request_time", -1e9)) < REPLAN_COOLDOWN:
        return False

    valid_conflicts = [
        conflict
        for conflict in conflicts
        if float(agents[conflict.winner_idx]["s"]) < float(conflict.winner_s) + CONFLICT_ZONE_HALF_S
    ]
    if not valid_conflicts:
        return False

    reservation = reservation_obstacles_for_conflicts(agents, valid_conflicts)
    if not reservation:
        return False

    start_xy = predicted_replan_start_xy(yielder)
    start_yaw = float(yielder["state"][2])
    yielder_snapshot = planning_snapshot_for_agent(yielder)
    future = executor.submit(
        build_reservation_replan,
        yielder_snapshot,
        env_radius,
        reservation,
        start_xy,
        start_yaw,
    )

    hold_s = max(
        float(yielder["s"]),
        min(float(conflict.yielder_s) - HOLD_BACK_DISTANCE for conflict in valid_conflicts),
    )
    release_time = max(float(conflict.release_time) for conflict in valid_conflicts)
    primary = min(
        valid_conflicts,
        key=lambda conflict: agents[conflict.winner_idx]["priority"],
    )
    blockers = []
    for conflict in valid_conflicts:
        winner = agents[conflict.winner_idx]
        release_s = min(
            float(winner["geom"]["total_L"]),
            float(conflict.winner_s) + CONFLICT_ZONE_HALF_S,
        )
        blockers.append(
            {
                "idx": conflict.winner_idx,
                "id": winner["id"],
                "release_s": release_s,
            }
        )

    yielder["replan_future"] = future
    yielder["pending_replan"] = {
        "winner_idx": primary.winner_idx,
        "winner_id": agents[primary.winner_idx]["id"],
        "winner_s": float(primary.winner_s),
        "winner_release_s": blockers[0]["release_s"] if blockers else 0.0,
        "yielder_s": float(primary.yielder_s),
        "hold_s": hold_s,
        "release_time": release_time,
        "requested_at": sim_time,
        "blockers": blockers,
    }
    yielder["yield_mode"] = "replan_pending"
    yielder["last_replan_request_time"] = sim_time
    print(
        f"t={sim_time:.2f}s {yielder['id']} started background replan "
        f"around blockers {[b['id'] for b in blockers]}; continuing current trajectory."
    )
    return True


def start_background_clear_winner_path_for_conflicts(yielder_idx, conflicts, agents, env_radius, sim_time, executor):
    if executor is None or not conflicts:
        return False

    yielder = agents[yielder_idx]
    if yielder.get("clearing_winner_path"):
        return True

    future = yielder.get("replan_future")
    if future is not None and not future.done():
        pending = yielder.get("pending_replan") or {}
        if pending.get("clear_winner_path"):
            return True
        future.cancel()
        clear_pending_replan(yielder, cancel=False)

    blocking_conflicts = [
        conflict
        for conflict in conflicts
        if conflict.blocking_winner_path
        and float(agents[conflict.winner_idx]["s"]) < float(conflict.winner_s) + CONFLICT_ZONE_HALF_S
    ]
    if not blocking_conflicts:
        return False

    hard_obstacles = hard_clear_obstacles_for_conflicts(agents, blocking_conflicts, yielder)
    if not hard_obstacles:
        return False

    start_xy = predicted_replan_start_xy(yielder)
    start_yaw = float(yielder["state"][2])
    yielder_snapshot = planning_snapshot_for_agent(yielder)
    future = executor.submit(
        build_reservation_replan,
        yielder_snapshot,
        env_radius,
        hard_obstacles,
        start_xy,
        start_yaw,
    )

    primary = min(
        blocking_conflicts,
        key=lambda conflict: agents[conflict.winner_idx]["priority"],
    )
    blockers = []
    for conflict in blocking_conflicts:
        winner = agents[conflict.winner_idx]
        release_s = min(
            float(winner["geom"]["total_L"]),
            float(conflict.winner_s) + CONFLICT_ZONE_HALF_S,
        )
        blockers.append(
            {
                "idx": conflict.winner_idx,
                "id": winner["id"],
                "release_s": release_s,
            }
        )

    yielder["replan_future"] = future
    yielder["pending_replan"] = {
        "clear_winner_path": True,
        "winner_idx": primary.winner_idx,
        "winner_id": agents[primary.winner_idx]["id"],
        "winner_s": float(primary.winner_s),
        "winner_release_s": blockers[0]["release_s"] if blockers else 0.0,
        "yielder_s": float(primary.yielder_s),
        "hold_s": float(yielder["s"]),
        "release_time": float("inf"),
        "requested_at": sim_time,
        "blockers": blockers,
    }
    yielder["yield_mode"] = "replan_pending"
    yielder["clear_winner_ids"] = [b["id"] for b in blockers]
    yielder["last_replan_request_time"] = sim_time
    print(
        f"t={sim_time:.2f}s {yielder['id']} is already blocking winner path; "
        f"hard-clear replan around {[b['id'] for b in blockers]} started."
    )
    return True


def apply_wait_policy(yielder, winner, conflict, sim_time):
    hold_s = max(float(yielder["s"]), float(conflict.yielder_s) - HOLD_BACK_DISTANCE)
    winner_release_s = min(
        float(winner["geom"]["total_L"]),
        float(conflict.winner_s) + CONFLICT_ZONE_HALF_S,
    )
    blockers = [
        {
            "idx": conflict.winner_idx,
            "id": winner["id"],
            "release_s": winner_release_s,
        }
    ]
    mode = set_yield_hold(
        yielder,
        hold_s,
        conflict.release_time,
        conflict.winner_idx,
        winner_release_s,
        blockers,
        sim_time,
    )
    print(
        f"t={sim_time:.2f}s {yielder['id']} {mode}s for {winner['id']}: "
        f"hold_s={hold_s:.2f}, wait_delay={conflict.wait_delay:.2f}s"
    )


def apply_wait_policy_for_conflicts(yielder_idx, conflicts, agents, sim_time):
    yielder = agents[yielder_idx]
    hold_s = max(
        float(yielder["s"]),
        min(float(conflict.yielder_s) - HOLD_BACK_DISTANCE for conflict in conflicts),
    )
    release_time = max(float(conflict.release_time) for conflict in conflicts)
    primary = min(
        conflicts,
        key=lambda conflict: agents[conflict.winner_idx]["priority"],
    )
    blockers = []
    for conflict in conflicts:
        winner = agents[conflict.winner_idx]
        release_s = min(
            float(winner["geom"]["total_L"]),
            float(conflict.winner_s) + CONFLICT_ZONE_HALF_S,
        )
        blockers.append(
            {
                "idx": conflict.winner_idx,
                "id": winner["id"],
                "release_s": release_s,
            }
        )

    mode = set_yield_hold(
        yielder,
        hold_s,
        release_time,
        primary.winner_idx,
        blockers[0]["release_s"] if blockers else None,
        blockers,
        sim_time,
    )
    print(
        f"t={sim_time:.2f}s {yielder['id']} {mode}s for "
        f"{[b['id'] for b in blockers]}: hold_s={hold_s:.2f}, "
        f"wait_until={release_time:.2f}s"
    )


def release_waiting_agents(agents, sim_time):
    for agent in agents:
        if agent.get("yield_mode") in ("wait", "slow"):
            release_by_time = sim_time >= float(agent.get("wait_until", 0.0))
            blockers = agent.get("yield_blockers", [])
            if blockers:
                release_by_progress = True
                for blocker in blockers:
                    winner_idx = blocker.get("idx")
                    if winner_idx is None or winner_idx < 0 or winner_idx >= len(agents):
                        continue
                    winner = agents[winner_idx]
                    if winner.get("done"):
                        continue
                    if winner["s"] < float(blocker.get("release_s", 0.0)):
                        release_by_progress = False
                        break
            else:
                release_by_progress = False
                winner_idx = agent.get("yield_winner_idx")
                if winner_idx is not None and 0 <= winner_idx < len(agents):
                    winner = agents[winner_idx]
                    release_s = agent.get("winner_release_s")
                    if release_s is not None and winner["s"] >= float(release_s):
                        release_by_progress = True

            if release_by_time or release_by_progress:
                print(f"t={sim_time:.2f}s {agent['id']} released from {agent.get('yield_mode')}.")
                clear_pending_replan(agent)
                agent["yield_mode"] = "normal"
                agent["hold_s"] = None
                agent["wait_until"] = 0.0
                agent["yield_winner_idx"] = None
                agent["winner_release_s"] = None
                agent["yield_blockers"] = []

        elif agent.get("yield_mode") == "replan":
            if sim_time - float(agent.get("last_replan_time", 0.0)) > 2.0:
                agent["yield_mode"] = "normal"


def run_scheduler(agents, env_radius, sim_time, replan_executor, draw_conflicts=False):
    collect_replan_jobs(agents, sim_time)
    collect_cell_refresh_jobs(agents, env_radius, sim_time, replan_executor)
    collect_route_refresh_jobs(agents, sim_time)
    release_waiting_agents(agents, sim_time)
    start_periodic_cell_refresh_jobs(agents, env_radius, sim_time, replan_executor)

    if draw_conflicts:
        visual_conflicts = find_visual_conflicts(agents, sim_time)
        draw_conflict_visuals(agents, visual_conflicts, sim_time)
        conflicts = [
            conflict
            for conflict in visual_conflicts
            if conflict.time_overlap or conflict.blocking_winner_path
        ]
    else:
        conflicts = find_active_conflicts(agents, sim_time)

    estop_conflicts = find_estop_blocking_conflicts(agents, sim_time)
    if estop_conflicts:
        existing = {
            (conflict.winner_idx, conflict.yielder_idx)
            for conflict in conflicts
        }
        for conflict in estop_conflicts:
            key = (conflict.winner_idx, conflict.yielder_idx)
            if key not in existing:
                conflicts.append(conflict)
                existing.add(key)

    if not conflicts:
        return

    busy_yielders = {
        idx
        for idx, agent in enumerate(agents)
        if agent.get("yield_mode") in ("wait", "slow", "replan", "replan_pending")
    }

    conflicts_by_yielder = {}
    for conflict in conflicts:
        conflicts_by_yielder.setdefault(conflict.yielder_idx, []).append(conflict)

    yielder_order = sorted(
        conflicts_by_yielder,
        key=lambda idx: agents[idx]["priority"],
    )

    for yielder_idx in yielder_order:
        yielder = agents[yielder_idx]
        yielder_conflicts = [
            conflict
            for conflict in conflicts_by_yielder[yielder_idx]
            if float(agents[conflict.winner_idx]["s"]) < float(conflict.winner_s) + CONFLICT_ZONE_HALF_S
        ]
        if not yielder_conflicts:
            continue
        primary = min(
            yielder_conflicts,
            key=lambda conflict: agents[conflict.winner_idx]["priority"],
        )
        winner = agents[primary.winner_idx]
        if yielder["done"] or winner["done"]:
            continue

        blocking_conflicts = [
            conflict
            for conflict in yielder_conflicts
            if conflict.blocking_winner_path
        ]
        if blocking_conflicts:
            replanned = start_background_clear_winner_path_for_conflicts(
                yielder_idx,
                blocking_conflicts,
                agents,
                env_radius,
                sim_time,
                replan_executor,
            )
            if not replanned:
                print(
                    f"t={sim_time:.2f}s {yielder['id']} blocks winner path but "
                    "hard-clear replan is not available yet; not applying wait."
                )
            busy_yielders.add(yielder_idx)
            continue

        if yielder_idx in busy_yielders:
            continue

        print(
            f"t={sim_time:.2f}s conflicts {[agents[c.winner_idx]['id'] for c in yielder_conflicts]} "
            f"-> {yielder['id']} wait={max(c.wait_delay for c in yielder_conflicts):.2f}s"
        )

        replanned = False
        if (
            yielder.get("allow_replan", True)
            and max(c.wait_delay for c in yielder_conflicts) >= REPLAN_DELAY_THRESHOLD
        ):
            replanned = start_background_replan_for_conflicts(
                yielder_idx,
                yielder_conflicts,
                agents,
                env_radius,
                sim_time,
                replan_executor,
            )
        if not replanned:
            apply_wait_policy_for_conflicts(yielder_idx, yielder_conflicts, agents, sim_time)
        busy_yielders.add(yielder_idx)


# =========================================================
#                        DISPLAY
# =========================================================

def draw_goal(agent):
    goal_vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.08,
        rgbaColor=list(agent["color"]),
    )
    p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=goal_vis,
        basePosition=[agent["goal"][0], agent["goal"][1], 0.06],
    )


def remove_debug_items(item_ids):
    if not item_ids or not p.isConnected():
        return
    for item_id in item_ids:
        try:
            p.removeUserDebugItem(int(item_id))
        except Exception:
            pass


def draw_polyline_items(points, color, z=0.05, line_width=2.0, life_time=0.0):
    if points is None or len(points) < 2 or not p.isConnected():
        return []
    item_ids = []
    for a, b in zip(points[:-1], points[1:]):
        item_id = p.addUserDebugLine(
            [float(a[0]), float(a[1]), z],
            [float(b[0]), float(b[1]), z],
            color,
            lineWidth=line_width,
            lifeTime=life_time,
        )
        item_ids.append(item_id)
    return item_ids


def redraw_agent_plan(agent, z=0.055, line_width=3.0):
    remove_debug_items(agent.get("plan_debug_items", []))
    agent["plan_debug_items"] = []
    if not p.isConnected():
        return

    if DRAW_ASTAR_RAW_PATH:
        agent["plan_debug_items"].extend(
            draw_polyline_items(
                agent.get("raw_path", []),
                [0.8, 0.8, 0.8],
                z=0.035,
                line_width=1.0,
                life_time=DRAW_PLAN_LIFETIME,
            )
        )
    if DRAW_TRAJECTORY:
        agent["plan_debug_items"].extend(
            draw_polyline_items(
                agent.get("trajectory", []),
                agent["path_color"],
                z=z,
                line_width=line_width,
                life_time=DRAW_PLAN_LIFETIME,
            )
        )


def draw_replan_highlight(agent):
    remove_debug_items(agent.get("replan_debug_items", []))
    agent["replan_debug_items"] = []
    if not DRAW_REPLANS or not p.isConnected():
        return
    agent["replan_debug_items"] = draw_polyline_items(
        agent.get("trajectory", []),
        agent["path_color"],
        z=0.09,
        line_width=5.0,
        life_time=DRAW_REPLAN_LIFETIME,
    )


def cell_path_tangent(agent, cell_idx):
    pts = agent.get("cell_points")
    if pts is None or len(pts) < 2:
        yaw = float(agent.get("state", [0.0, 0.0, 0.0])[2])
        return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

    idx = max(0, min(int(cell_idx), len(pts) - 1))
    if idx == 0:
        vec = pts[1] - pts[0]
    elif idx >= len(pts) - 1:
        vec = pts[-1] - pts[-2]
    else:
        vec = pts[idx + 1] - pts[idx - 1]

    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        yaw = float(agent.get("state", [0.0, 0.0, 0.0])[2])
        return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    return vec / norm


def draw_oriented_oval(center, tangent, long_radius, lateral_radius, color, z, life_time, line_width=2.0):
    tangent = np.array(tangent, dtype=float)
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1e-9:
        tangent = np.array([1.0, 0.0], dtype=float)
    else:
        tangent = tangent / tangent_norm
    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    center = np.array(center, dtype=float)

    points = []
    for k in range(CONFLICT_OVAL_SEGMENTS + 1):
        theta = 2.0 * math.pi * k / CONFLICT_OVAL_SEGMENTS
        q = (
            center
            + float(long_radius) * math.cos(theta) * tangent
            + float(lateral_radius) * math.sin(theta) * normal
        )
        points.append(q)

    for a, b in zip(points[:-1], points[1:]):
        p.addUserDebugLine(
            [float(a[0]), float(a[1]), z],
            [float(b[0]), float(b[1]), z],
            color,
            lineWidth=line_width,
            lifeTime=life_time,
        )


def draw_conflict_marker(center, color, z, life_time):
    r = 0.10
    x, y = float(center[0]), float(center[1])
    p.addUserDebugLine(
        [x - r, y, z],
        [x + r, y, z],
        color,
        lineWidth=3.0,
        lifeTime=life_time,
    )
    p.addUserDebugLine(
        [x, y - r, z],
        [x, y + r, z],
        color,
        lineWidth=3.0,
        lifeTime=life_time,
    )


def _window_text(low, high):
    return f"[{float(low):.1f},{float(high):.1f}]"


def draw_conflict_visuals(agents, conflicts, sim_time):
    if not DRAW_CONFLICT_VISUALS or not p.isConnected():
        return

    for idx, conflict in enumerate(conflicts[:CONFLICT_VISUAL_MAX_ITEMS]):
        agent_i = agents[conflict.i]
        agent_j = agents[conflict.j]
        active = bool(conflict.time_overlap or conflict.blocking_winner_path)
        base_color = [1.0, 0.15, 0.05] if active else [0.35, 0.75, 1.0]
        z = CONFLICT_OVAL_Z + 0.025 * (idx % 4)
        life = CONFLICT_VISUAL_LIFETIME

        draw_conflict_marker(conflict.center, base_color, z + 0.02, life)
        p.addUserDebugLine(
            [float(conflict.p_i[0]), float(conflict.p_i[1]), z],
            [float(conflict.p_j[0]), float(conflict.p_j[1]), z],
            base_color,
            lineWidth=2.0,
            lifeTime=life,
        )

        for agent, cell_idx, point, low, high, offset in (
            (agent_i, conflict.cell_idx_i, conflict.p_i, conflict.t_i_low, conflict.t_i_high, 0.0),
            (agent_j, conflict.cell_idx_j, conflict.p_j, conflict.t_j_low, conflict.t_j_high, 0.01),
        ):
            window_width = max(0.05, float(high) - float(low))
            long_radius = clamp(
                0.5 * effective_cell_time_straight_speed() * window_width,
                CONFLICT_OVAL_MIN_LONG_RADIUS,
                CONFLICT_OVAL_MAX_LONG_RADIUS,
            )
            tangent = cell_path_tangent(agent, cell_idx)
            draw_oriented_oval(
                point,
                tangent,
                long_radius,
                CONFLICT_OVAL_LATERAL_RADIUS,
                agent["path_color"],
                z + offset,
                life,
                line_width=2.0 if active else 1.0,
            )

        label_title = "BLOCKING" if conflict.blocking_winner_path else ("CONFLICT" if active else "close paths")
        label = (
            f"{label_title}\n"
            f"{agent_i['id']} {_window_text(conflict.t_i_low, conflict.t_i_high)}\n"
            f"{agent_j['id']} {_window_text(conflict.t_j_low, conflict.t_j_high)}\n"
            f"d={conflict.distance:.2f}m"
        )
        label_pos = [
            float(conflict.center[0]) + 0.08,
            float(conflict.center[1]) + 0.08,
            z + 0.12,
        ]
        p.addUserDebugText(
            label,
            label_pos,
            textColorRGB=base_color,
            textSize=0.85,
            lifeTime=life,
        )


def print_agent_status(agents):
    parts = []
    for ag in agents:
        if ag["done"]:
            parts.append(f"{ag['id']}:done")
            continue
        estop_text = ""
        if ag.get("emergency_stop"):
            reason = ag.get("emergency_reason")
            if reason:
                estop_text = f"ESTOP:{ag.get('emergency_blocker')}({reason}) "
            else:
                estop_text = f"ESTOP:{ag.get('emergency_blocker')} "
        backoff_text = ""
        if ag.get("backoff_from_id") is not None:
            backoff_text = f"BACKOFF:{ag.get('backoff_from_id')} "
        parts.append(
            f"{ag['id']}:{ag['yield_mode']} "
            f"{estop_text}"
            f"{backoff_text}"
            f"{'reserved-slow ' if ag.get('time_reserved_hold') else ''}"
            f"s={ag['s']:.1f}/{ag['geom']['total_L']:.1f} "
            f"d={ag['d_path']:.2f} v_s={ag['v_s_ema']:.2f}"
        )
    print(" | ".join(parts))


# =========================================================
#                          MAIN
# =========================================================

def main():
    args = parse_args()
    configure_timing_from_args(args)
    here = os.path.dirname(os.path.abspath(__file__))
    rover_urdf_path = os.path.join(here, "2_wheel_rover.urdf")
    pebble_urdf_path = os.path.join(here, "pebbles.urdf")

    scenario = build_scheduling_scenario(
        args.scenario,
        pebble_mode=args.pebbles,
        num_pebbles=args.num_pebbles,
        pebble_seed=args.pebble_seed,
    )
    env_radius = scenario["env_radius"]
    pebble_centers = scenario["pebble_centers"]

    print(f"A* trajectory scheduling scenario: {scenario['name']}")
    print(f"  {scenario['description']}")
    print(f"  rovers={len(scenario['agents'])}, pebbles={len(pebble_centers)}")
    print("  priority: smaller number has right-of-way")
    print(
        f"  scheduler ETA scales: lower={ETA_TIME_SCALE_LOWER:.2f}, "
        f"upper={ETA_TIME_SCALE_UPPER:.2f} (offline only)"
    )
    if START_DIRECTION_SCORING:
        print(
            "  local start-direction acceptance: "
            f"cells={START_DIRECTION_MIN_CELLS}, "
            f"lookahead={START_DIRECTION_LOOKAHEAD_M:.2f}m, "
            f"deadband={math.degrees(START_DIRECTION_TURN_DEADBAND):.0f}deg"
        )
    if RUN_CELL_TIME_CALIBRATION:
        print_cell_time_calibration_report(env_radius)

    agents = [
        build_initial_agent_plan(cfg, env_radius, pebble_centers)
        for cfg in scenario["agents"]
    ]

    print("==== INITIAL PLANS ====")
    for agent in agents:
        print(
            f"{agent['id']}: priority={agent['priority']:.2f} "
            f"mode={agent['planning_mode']} path={agent['geom']['total_L']:.2f}m "
            f"T=[{agent['T_total_lower']:.2f},{agent['T_total_upper']:.2f}]s"
        )
    print("=======================")

    p.connect(p.DIRECT if args.headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    if not args.headless:
        p.resetDebugVisualizerCamera(
            cameraDistance=7.0,
            cameraYaw=45,
            cameraPitch=-60,
            cameraTargetPosition=[0, 0, 0],
        )
    p.setGravity(0, 0, -9.81)
    sim_dt = 1.0 / 240.0
    p.setTimeStep(sim_dt)
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)

    ignored_union = set()
    for agent in agents:
        ignored_union.update(agent["ignored_pebbles"])

    for px, py in pebble_centers:
        bid = p.loadURDF(
            pebble_urdf_path,
            basePosition=[px, py, 0.01],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=False,
            globalScaling=1.0,
        )
        p.changeDynamics(bid, -1, lateralFriction=0.8)
        if (px, py) in ignored_union:
            p.changeVisualShape(bid, -1, rgbaColor=[0.75, 0.75, 0.75, 0.45])

    for agent in agents:
        if not args.no_draw:
            redraw_agent_plan(agent)
            draw_goal(agent)

        spawn_agent_rover(agent, rover_urdf_path)
        p.changeDynamics(agent["body"], -1, lateralFriction=0.8)
        for link in (agent["left_joint"], agent["right_joint"]):
            p.changeDynamics(
                agent["body"],
                link,
                lateralFriction=1.0,
                rollingFriction=0.0,
                spinningFriction=0.0,
            )

    print("Running A* trajectory scheduling demo. Close GUI window to stop.")

    acc_control = 0.0
    acc_scheduler = 0.0
    replan_executor = ProcessPoolExecutor(max_workers=REPLAN_WORKERS)
    warmups = [replan_executor.submit(planner_warmup) for _ in range(REPLAN_WORKERS)]
    for future in warmups:
        future.result()
    sim_start = time.time()
    t_print = sim_start

    try:
        while p.isConnected():
            p.stepSimulation()
            if not args.headless:
                time.sleep(sim_dt)
            acc_control += sim_dt
            acc_scheduler += sim_dt

            now = time.time()
            sim_time = now - sim_start
            if sim_time > args.max_time:
                print("Reached max simulation time.")
                break

            if acc_control >= CONTROL_DT:
                acc_control = 0.0
                for agent in agents:
                    update_agent_state_and_progress(agent, now)

                if acc_scheduler >= SCHEDULER_DT:
                    acc_scheduler = 0.0
                    run_scheduler(
                        agents,
                        env_radius,
                        sim_time,
                        replan_executor,
                        draw_conflicts=(not args.no_draw and not args.headless),
                    )

                update_emergency_stops(agents, sim_time)
                update_close_pair_backoff(agents, sim_time)

                for agent in agents:
                    update_done_state(agent, sim_time)
                    v_cmd, w_cmd = compute_path_control(agent, sim_time)
                    agent["control"] = (v_cmd, w_cmd)
                    apply_configured_diff_drive_control(agent)

                if now - t_print > 1.0:
                    t_print = now
                    print_agent_status(agents)

                if all(agent["done"] for agent in agents):
                    print("All rovers reached their path ends.")
                    break
    finally:
        replan_executor.shutdown(wait=False, cancel_futures=True)

    if p.isConnected():
        for _ in range(120):
            p.stepSimulation()
            if not args.headless:
                time.sleep(sim_dt)
        p.disconnect()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()

