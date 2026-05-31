import math
import os
import time
from collections import deque

import numpy as np
import pybullet as p
import pybullet_data

from flowfield_pybullet import (
    FlowFieldController,
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
    ProgressAwarePathFollowerField,
    build_obstacle_field,
    draw_polyline,
    make_random_pebbles,
    plan_astar_with_optional_relaxation,
    polyline_is_free,
    polyline_length,
    project_point_to_path_s_windowed,
    resample_polyline,
    shortcut_path,
    smooth_path_collision_checked,
)


# =========================================================
#                      CONFIGURATION
# =========================================================

SCENARIO_NAME = "three_lanes"  # "three_lanes" or "crossing"

DRAW_ASTAR_RAW_PATH = True
DRAW_TRAJECTORY = True
USE_PATH_SHORTCUT = False
USE_ONLINE_PATH_GUIDANCE = True

ROVER_RADIUS = 0.15
PEBBLE_RADIUS = 0.05
OBSTACLE_CLEARANCE = 0.01

ALLOW_ISOLATED_OBSTACLE_RELAXATION = True
RELAX_MIN_CLUSTER_SIZE = 2
RELAX_PROGRESSIVE = True
RELAX_MAX_IGNORED_CLUSTER_SIZE = None
RELAX_CLUSTER_LINK_RADIUS = 0.35
RELAX_CLUSTER_EXTRA_GAP = 0.10

GRID_W = 101
GRID_H = 101

SHOVEL_OFFSET = 0.17
CONTROL_DT = 0.05

# ETA model
LANDMARK_FRACS = [0.25, 0.50, 0.75, 1.00]
OFFLINE_TIME_SCALE_LOWER = 1.6
OFFLINE_TIME_SCALE_UPPER = 2.4
OFFLINE_TIME_SCALE_NOMINAL = 0.5 * (OFFLINE_TIME_SCALE_LOWER + OFFLINE_TIME_SCALE_UPPER)
ETA_HISTORY_LEN = 8
VS_MIN_FOR_ETA = 0.05
SPEED_FACTOR_MIN = 0.25
SPEED_FACTOR_MAX = 5.0
EMA_ALPHA = 0.15

# Geometric timing speed profile
ETA_V_REF = 0.8
ETA_V_MIN = 0.06
ETA_CURVATURE_GAIN = 1.4
ETA_CURVATURE_POWER = 1.0

PROGRESS_BACKTRACK_M = 0.35
PROGRESS_LOOKAHEAD_M = 1.20


# =========================================================
#                 GEOMETRIC TIME ESTIMATION
# =========================================================

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_curvature_nodes(pts, segs, seg_lens):
    n = len(pts)
    kappa = np.zeros(n, dtype=float)
    if n < 3:
        return kappa

    tangents = np.zeros((max(n - 1, 1), 2), dtype=float)
    for i in range(n - 1):
        L = float(seg_lens[i])
        if L > 1e-9:
            tangents[i] = segs[i] / L

    for i in range(1, n - 1):
        t_prev = tangents[i - 1]
        t_next = tangents[i]
        if np.linalg.norm(t_prev) < 1e-9 or np.linalg.norm(t_next) < 1e-9:
            continue

        cross = t_prev[0] * t_next[1] - t_prev[1] * t_next[0]
        dot = float(np.clip(np.dot(t_prev, t_next), -1.0, 1.0))
        dtheta = math.atan2(cross, dot)
        ds = 0.5 * (seg_lens[i - 1] + seg_lens[i])
        if ds > 1e-9:
            kappa[i] = dtheta / ds

    if n >= 2:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def precompute_geometric_time_profile(pts,
                                      segs,
                                      seg_lens,
                                      s_cum,
                                      v_ref=ETA_V_REF,
                                      v_min=ETA_V_MIN,
                                      curvature_gain=ETA_CURVATURE_GAIN,
                                      curvature_power=ETA_CURVATURE_POWER,
                                      time_scale_lower=OFFLINE_TIME_SCALE_LOWER,
                                      time_scale_upper=OFFLINE_TIME_SCALE_UPPER,
                                      time_scale_nominal=OFFLINE_TIME_SCALE_NOMINAL):
    kappa = compute_curvature_nodes(pts, segs, seg_lens)
    v_base_nodes = v_ref / (
        1.0 + curvature_gain * np.power(np.abs(kappa), curvature_power)
    )
    v_base_nodes = np.maximum(v_base_nodes, v_min)

    T_base_nodes = np.zeros(len(pts), dtype=float)
    for i in range(len(seg_lens)):
        ds = float(seg_lens[i])
        v_seg = 0.5 * (v_base_nodes[i] + v_base_nodes[i + 1])
        v_seg = max(v_seg, v_min)
        T_base_nodes[i + 1] = T_base_nodes[i] + ds / v_seg

    T_lower_nodes = time_scale_lower * T_base_nodes
    T_upper_nodes = time_scale_upper * T_base_nodes
    T_nominal_nodes = time_scale_nominal * T_base_nodes
    v_nominal_nodes = v_base_nodes / max(time_scale_nominal, 1e-6)

    landmark_s = np.asarray([frac * s_cum[-1] for frac in LANDMARK_FRACS], dtype=float)
    landmark_T_lower = np.asarray(
        [interpolate_along_path(s_lm, s_cum, T_lower_nodes) for s_lm in landmark_s],
        dtype=float,
    )
    landmark_T_upper = np.asarray(
        [interpolate_along_path(s_lm, s_cum, T_upper_nodes) for s_lm in landmark_s],
        dtype=float,
    )
    landmark_T_nominal = np.asarray(
        [interpolate_along_path(s_lm, s_cum, T_nominal_nodes) for s_lm in landmark_s],
        dtype=float,
    )
    return (
        T_lower_nodes,
        T_upper_nodes,
        T_nominal_nodes,
        v_nominal_nodes,
        landmark_s,
        landmark_T_lower,
        landmark_T_upper,
        landmark_T_nominal,
    )


def _range_contains(value, lower, upper):
    return lower <= value <= upper


def _format_eta_range(lower, upper, offline=False):
    suffix = "s(off)" if offline else "s"
    return f"[{lower:.1f},{upper:.1f}]{suffix}"


def _estimate_speed_factor(distance_remaining, nominal_time_remaining, measured_speed):
    if distance_remaining <= 1e-6 or nominal_time_remaining <= 1e-6:
        return 1.0
    v_nominal_avg = distance_remaining / nominal_time_remaining
    speed_factor = v_nominal_avg / max(VS_MIN_FOR_ETA, abs(measured_speed))
    return max(SPEED_FACTOR_MIN, min(SPEED_FACTOR_MAX, speed_factor))


def update_agent_eta(agent, t_rel):
    geom = agent["geom"]
    s_now = agent["s"]

    T_now_lower = interpolate_along_path(s_now, geom["s_cum"], agent["T_lower_nodes"])
    T_now_upper = interpolate_along_path(s_now, geom["s_cum"], agent["T_upper_nodes"])
    T_now_nominal = interpolate_along_path(s_now, geom["s_cum"], agent["T_nominal_nodes"])
    v_nom_loc = interpolate_along_path(s_now, geom["s_cum"], agent["v_nom_nodes"])
    agent["T_now"] = T_now_nominal
    agent["v_nom_loc"] = v_nom_loc

    eta_off_lower = max(0.0, agent["T_total_lower"] - T_now_lower)
    eta_off_upper = max(0.0, agent["T_total_upper"] - T_now_upper)
    eta_off_nominal = max(0.0, agent["T_total_nominal"] - T_now_nominal)
    agent["eta_end_off_lower"] = eta_off_lower
    agent["eta_end_off_upper"] = eta_off_upper
    agent["eta_end_off"] = eta_off_nominal

    use_online = abs(agent["v_s_ema"]) > VS_MIN_FOR_ETA
    if use_online and eta_off_upper > 0.05:
        remaining_s = max(0.0, geom["total_L"] - s_now)
        speed_factor = _estimate_speed_factor(
            remaining_s,
            eta_off_nominal,
            agent["v_s_ema"],
        )
        agent["speed_factor"] = speed_factor
        agent["eta_end_on_lower"] = speed_factor * eta_off_lower
        agent["eta_end_on_upper"] = speed_factor * eta_off_upper
        agent["eta_end_on"] = speed_factor * eta_off_nominal
    else:
        agent["speed_factor"] = 1.0
        agent["eta_end_on_lower"] = None
        agent["eta_end_on_upper"] = None
        agent["eta_end_on"] = None

    next_idx = None
    for i, s_lm in enumerate(agent["landmark_s"]):
        if s_now < s_lm and agent["landmark_actual"][i] is None:
            next_idx = i
            break

    agent["next_landmark_idx"] = next_idx
    agent["eta_landmark_off_lower"] = None
    agent["eta_landmark_off_upper"] = None
    agent["eta_landmark_off"] = None
    agent["eta_landmark_on_lower"] = None
    agent["eta_landmark_on_upper"] = None
    agent["eta_landmark_on"] = None

    if next_idx is not None:
        rem_lm_lower = max(0.0, agent["landmark_T_lower"][next_idx] - T_now_lower)
        rem_lm_upper = max(0.0, agent["landmark_T_upper"][next_idx] - T_now_upper)
        rem_lm_nominal = max(0.0, agent["landmark_T_nominal"][next_idx] - T_now_nominal)
        agent["eta_landmark_off_lower"] = rem_lm_lower
        agent["eta_landmark_off_upper"] = rem_lm_upper
        agent["eta_landmark_off"] = rem_lm_nominal
        if agent["eta_end_on"] is not None and rem_lm_upper > 0.05:
            lm_speed_factor = _estimate_speed_factor(
                max(0.0, agent["landmark_s"][next_idx] - s_now),
                rem_lm_nominal,
                agent["v_s_ema"],
            )
            agent["eta_landmark_on_lower"] = lm_speed_factor * rem_lm_lower
            agent["eta_landmark_on_upper"] = lm_speed_factor * rem_lm_upper
            agent["eta_landmark_on"] = lm_speed_factor * rem_lm_nominal

    for i, s_lm in enumerate(agent["landmark_s"]):
        if agent["landmark_actual"][i] is None and s_now < s_lm:
            rem_lm_lower = max(0.0, agent["landmark_T_lower"][i] - T_now_lower)
            rem_lm_upper = max(0.0, agent["landmark_T_upper"][i] - T_now_upper)
            rem_lm_nominal = max(0.0, agent["landmark_T_nominal"][i] - T_now_nominal)
            if use_online and rem_lm_upper > 0.05:
                lm_speed_factor = _estimate_speed_factor(
                    max(0.0, s_lm - s_now),
                    rem_lm_nominal,
                    agent["v_s_ema"],
                )
                agent["landmark_pred_lower"][i] = (
                    t_rel + lm_speed_factor * rem_lm_lower
                )
                agent["landmark_pred_upper"][i] = (
                    t_rel + lm_speed_factor * rem_lm_upper
                )
                agent["landmark_pred"][i] = (
                    t_rel + lm_speed_factor * rem_lm_nominal
                )

    for i, s_lm in enumerate(agent["landmark_s"]):
        if agent["landmark_actual"][i] is None and s_now >= s_lm:
            agent["landmark_actual"][i] = t_rel
            off_lower = agent["landmark_T_lower"][i]
            off_upper = agent["landmark_T_upper"][i]
            pred_lower = agent["landmark_pred_lower"][i]
            pred_upper = agent["landmark_pred_upper"][i]
            off_hit = _range_contains(t_rel, off_lower, off_upper)
            if pred_lower is None or pred_upper is None:
                print(
                    f"{agent['id']} reached {LANDMARK_FRACS[i] * 100:.0f}% "
                    f"at t={t_rel:.2f}s  "
                    f"offline=[{off_lower:.2f},{off_upper:.2f}]s  "
                    f"hit_off={off_hit}  online_pred=none"
                )
            else:
                pred_hit = _range_contains(t_rel, pred_lower, pred_upper)
                print(
                    f"{agent['id']} reached {LANDMARK_FRACS[i] * 100:.0f}% "
                    f"at t={t_rel:.2f}s  "
                    f"offline=[{off_lower:.2f},{off_upper:.2f}]s  "
                    f"hit_off={off_hit}  "
                    f"online_pred=[{pred_lower:.2f},{pred_upper:.2f}]s  "
                    f"hit_on={pred_hit}"
                )


# =========================================================
#                       SCENARIOS
# =========================================================

def build_multi_scenario(name):
    name = name.lower().strip()
    env_radius = 5.0

    if name == "three_lanes":
        agents = [
            {
                "id": "R0",
                "start": np.array([3.6, 3.1], dtype=float),
                "goal": np.array([-3.5, 2.3], dtype=float),
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([-3.6, 0.2], dtype=float),
                "goal": np.array([3.6, -0.4], dtype=float),
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
            {
                "id": "R2",
                "start": np.array([3.3, -2.8], dtype=float),
                "goal": np.array([-3.4, -3.5], dtype=float),
                "color": (0.25, 0.9, 0.35, 1.0),
                "path_color": [0.25, 1.0, 0.65],
            },
        ]
        num_pebbles = 230
        description = "Three rovers on mostly separated A* lanes through a shared random pebble field."

    elif name == "crossing":
        agents = [
            {
                "id": "R0",
                "start": np.array([-3.6, 0.0], dtype=float),
                "goal": np.array([3.6, 0.0], dtype=float),
                "color": (0.1, 0.5, 1.0, 1.0),
                "path_color": [1.0, 1.0, 0.0],
            },
            {
                "id": "R1",
                "start": np.array([0.0, -3.6], dtype=float),
                "goal": np.array([0.0, 3.6], dtype=float),
                "color": (1.0, 0.25, 0.25, 1.0),
                "path_color": [1.0, 0.45, 0.2],
            },
        ]
        num_pebbles = 120
        description = "Two rovers with crossing goals. No priority/yield logic is applied."

    else:
        raise ValueError("Unknown SCENARIO_NAME. Use 'three_lanes' or 'crossing'.")

    protected = []
    for cfg in agents:
        protected.append(cfg["start"])
        protected.append(cfg["goal"])

    pebble_centers = []
    seed = 41
    while len(pebble_centers) < num_pebbles:
        candidate_batch = make_random_pebbles(
            env_radius,
            num_pebbles=max(25, num_pebbles - len(pebble_centers)),
            start_pos=protected[0],
            goal_pos=protected[1],
            seed=seed,
            keepout_radius=0.0,
        )
        seed += 1
        for px, py in candidate_batch:
            p_xy = np.array([px, py], dtype=float)
            if all(np.linalg.norm(p_xy - q) > 0.75 for q in protected):
                pebble_centers.append((px, py))
                if len(pebble_centers) >= num_pebbles:
                    break

    return {
        "name": name,
        "description": description,
        "env_radius": env_radius,
        "agents": agents,
        "pebble_centers": pebble_centers,
    }


# =========================================================
#                   AGENT CONSTRUCTION
# =========================================================

def build_agent_plan(agent_cfg, env_radius, pebble_centers):
    t_plan_start = time.perf_counter()
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
    t_astar = time.perf_counter() - t_plan_start

    if raw_path is None:
        raise RuntimeError(f"{agent_cfg['id']}: A* failed even after relaxation.")

    t_path_start = time.perf_counter()
    shortcut = shortcut_path(planner_field, raw_path)
    path_base = shortcut if USE_PATH_SHORTCUT else raw_path
    path_base_name = "shortcut" if USE_PATH_SHORTCUT else "raw_astar"
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
    t_path = time.perf_counter() - t_path_start

    pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(trajectory)
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
    )

    t_field_start = time.perf_counter()
    field = ProgressAwarePathFollowerField(
        pts,
        segs,
        seg_lens,
        s_cum,
        k_t=2.0,
        k_n=9.0,
        backtrack_s=PROGRESS_BACKTRACK_M,
        lookahead_s=PROGRESS_LOOKAHEAD_M,
    )
    t_field = time.perf_counter() - t_field_start

    first_vec = pts[min(1, len(pts) - 1)] - pts[0]
    start_yaw = 0.0
    if np.linalg.norm(first_vec) > 1e-9:
        start_yaw = math.atan2(first_vec[1], first_vec[0])

    return {
        "id": agent_cfg["id"],
        "start": agent_cfg["start"],
        "goal": agent_cfg["goal"],
        "color": agent_cfg["color"],
        "path_color": agent_cfg["path_color"],
        "raw_path": raw_path,
        "shortcut_path": shortcut,
        "trajectory": trajectory,
        "path_base_name": path_base_name,
        "planning_mode": planning_mode,
        "planning_pebbles": planning_pebbles,
        "ignored_pebbles": ignored_pebbles,
        "field": field,
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
        "T_nodes": T_nominal_nodes,
        "v_nom_nodes": v_nom_nodes,
        "T_total_lower": float(T_lower_nodes[-1]),
        "T_total_upper": float(T_upper_nodes[-1]),
        "T_total_nominal": float(T_nominal_nodes[-1]),
        "T_total": float(T_nominal_nodes[-1]),
        "landmark_s": landmark_s,
        "landmark_T_lower": landmark_T_lower,
        "landmark_T_upper": landmark_T_upper,
        "landmark_T_nominal": landmark_T_nominal,
        "landmark_T": landmark_T_nominal,
        "timing": {
            "astar": t_astar,
            "trajectory": t_path,
            "field": t_field,
        },
    }


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

    agent["controller"] = FlowFieldController(
        v_max=1.0,
        w_max=6.0,
        k_theta=6.0,
        turn_in_place_angle_deg=50.0,
        static_speed_threshold=0.03,
        w_turn_in_place=25.0,
        stop_dist=0.14,
    )
    agent["state"] = np.array(
        [agent["start"][0], agent["start"][1], agent["start_yaw"], 0.0, 0.0],
        dtype=float,
    )
    agent["control"] = (0.0, 0.0)
    agent["s"] = 0.0
    agent["s_prev"] = 0.0
    agent["d_path"] = 0.0
    agent["time_prev"] = None
    agent["v_s_ema"] = 0.0
    agent["speed_factor"] = 1.0
    agent["eta_end_off_lower"] = agent["T_total_lower"]
    agent["eta_end_off_upper"] = agent["T_total_upper"]
    agent["eta_end_off"] = agent["T_total"]
    agent["eta_end_on_lower"] = None
    agent["eta_end_on_upper"] = None
    agent["eta_end_on"] = None
    agent["eta_landmark_off_lower"] = None
    agent["eta_landmark_off_upper"] = None
    agent["eta_landmark_off"] = None
    agent["eta_landmark_on_lower"] = None
    agent["eta_landmark_on_upper"] = None
    agent["eta_landmark_on"] = None
    agent["next_landmark_idx"] = None
    agent["landmark_pred_lower"] = [None for _ in LANDMARK_FRACS]
    agent["landmark_pred_upper"] = [None for _ in LANDMARK_FRACS]
    agent["landmark_pred"] = [None for _ in LANDMARK_FRACS]
    agent["landmark_actual"] = [None for _ in LANDMARK_FRACS]
    agent["eta_hist"] = deque(maxlen=ETA_HISTORY_LEN)
    agent["low_speed_acc"] = 0.0
    agent["done"] = False
    agent["actual_dist"] = 0.0
    agent["prev_xy"] = agent["start"].copy()


def update_agent_progress_and_control(agent, now, t_rel):
    if agent["done"]:
        agent["control"] = (0.0, 0.0)
        return

    agent["state"] = get_state_from_bullet(agent["body"])
    x, y, yaw, v_fwd, w_yaw = agent["state"]

    xy = np.array([x, y], dtype=float)
    agent["actual_dist"] += float(np.linalg.norm(xy - agent["prev_xy"]))
    agent["prev_xy"] = xy

    shovel = np.array([
        x + SHOVEL_OFFSET * math.cos(yaw),
        y + SHOVEL_OFFSET * math.sin(yaw),
    ], dtype=float)

    geom = agent["geom"]
    s_now, d_now, _, _ = project_point_to_path_s_windowed(
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
    agent["field"].update_progress(s_now)

    if agent["time_prev"] is None:
        agent["time_prev"] = now
        agent["s_prev"] = s_now
    else:
        dt = max(1e-6, now - agent["time_prev"])
        v_s = (s_now - agent["s_prev"]) / dt
        agent["v_s_ema"] = (1.0 - EMA_ALPHA) * agent["v_s_ema"] + EMA_ALPHA * v_s
        agent["time_prev"] = now
        agent["s_prev"] = s_now

    update_agent_eta(agent, t_rel)

    tracking_state = np.array([shovel[0], shovel[1], yaw, v_fwd, w_yaw], dtype=float)
    v_cmd, w_cmd = agent["controller"].compute_control(
        tracking_state,
        agent["goal"],
        agent["field"],
    )

    remaining = max(0.0, geom["total_L"] - s_now)
    if remaining < 0.18:
        if abs(agent["v_s_ema"]) < 0.035:
            agent["low_speed_acc"] += CONTROL_DT
        else:
            agent["low_speed_acc"] = 0.0

        if agent["low_speed_acc"] >= 0.4:
            agent["done"] = True
            v_cmd, w_cmd = 0.0, 0.0
            off_hit = _range_contains(
                t_rel,
                agent["T_total_lower"],
                agent["T_total_upper"],
            )
            print(
                f"{agent['id']} reached goal path end at t={t_rel:.2f}s  "
                f"offline_end=[{agent['T_total_lower']:.2f},"
                f"{agent['T_total_upper']:.2f}]s  "
                f"hit_off={off_hit}  "
                f"actual_dist={agent['actual_dist']:.2f}m"
            )
    else:
        agent["low_speed_acc"] = 0.0

    agent["control"] = (float(v_cmd), float(w_cmd))


def print_agent_status(agents):
    parts = []
    for agent in agents:
        if agent["done"]:
            parts.append(f"{agent['id']}:done")
            continue

        next_idx = agent["next_landmark_idx"]
        lm_label = "-"
        eta_lm = "-"
        if next_idx is not None:
            lm_label = f"{int(LANDMARK_FRACS[next_idx] * 100)}%"
            if agent["eta_landmark_on_lower"] is not None:
                eta_lm = _format_eta_range(
                    agent["eta_landmark_on_lower"],
                    agent["eta_landmark_on_upper"],
                )
            elif agent["eta_landmark_off_lower"] is not None:
                eta_lm = _format_eta_range(
                    agent["eta_landmark_off_lower"],
                    agent["eta_landmark_off_upper"],
                    offline=True,
                )

        eta_end = "-"
        if agent["eta_end_on_lower"] is not None:
            eta_end = _format_eta_range(
                agent["eta_end_on_lower"],
                agent["eta_end_on_upper"],
            )
        else:
            eta_end = _format_eta_range(
                agent["eta_end_off_lower"],
                agent["eta_end_off_upper"],
                offline=True,
            )

        parts.append(
            f"{agent['id']}: s={agent['s']:.1f}/{agent['geom']['total_L']:.1f} "
            f"d={agent['d_path']:.2f} v_s={agent['v_s_ema']:.2f} "
            f"ETAend={eta_end} LM{lm_label}={eta_lm}"
        )
    print(" | ".join(parts))


# =========================================================
#                            MAIN
# =========================================================

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    rover_urdf_path = os.path.join(here, "2_wheel_rover.urdf")
    pebble_urdf_path = os.path.join(here, "pebbles.urdf")

    scenario = build_multi_scenario(SCENARIO_NAME)
    env_radius = scenario["env_radius"]
    pebble_centers = scenario["pebble_centers"]

    print(f"Multi-rover A* scenario: {scenario['name']}")
    print(f"  {scenario['description']}")
    print(f"  rovers={len(scenario['agents'])}, pebbles={len(pebble_centers)}")

    agents = [
        build_agent_plan(cfg, env_radius, pebble_centers)
        for cfg in scenario["agents"]
    ]

    print("==== PLANNING TIMING ====")
    for agent in agents:
        timing = agent["timing"]
        print(
            f"{agent['id']}: A*={timing['astar']:.4f}s  "
            f"trajectory={timing['trajectory']:.4f}s  "
            f"path_guidance={timing['field']:.4f}s  "
            f"mode={agent['planning_mode']}  "
            f"path={agent['path_base_name']}  "
            f"T_off_end=[{agent['T_total_lower']:.2f},"
            f"{agent['T_total_upper']:.2f}]s"
        )
    print("=========================")

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
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
        if DRAW_ASTAR_RAW_PATH:
            draw_polyline(agent["raw_path"], [0.8, 0.8, 0.8], z=0.035, line_width=1.0)
        if DRAW_TRAJECTORY:
            draw_polyline(agent["trajectory"], agent["path_color"], z=0.055, line_width=3.0)

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

    print("Running multi-rover A* ETA demo. Close the GUI window to stop.")

    acc = 0.0
    sim_start = time.time()
    t_print = sim_start

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        if acc >= CONTROL_DT:
            acc = 0.0
            now = time.time()
            t_rel = now - sim_start

            for agent in agents:
                update_agent_progress_and_control(agent, now, t_rel)
                apply_diff_drive_control(agent)

            if now - t_print > 1.0:
                t_print = now
                print_agent_status(agents)

            if all(agent["done"] for agent in agents):
                print("All rovers reached their path ends.")
                break

    if p.isConnected():
        for _ in range(180):
            p.stepSimulation()
            time.sleep(sim_dt)
        p.disconnect()


if __name__ == "__main__":
    main()
