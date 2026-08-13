import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

# Shared utilities from your goal-directed flowfield module
from flowfield_pybullet import (
    FlowField2D,
    FlowFieldController,
    yaw_to_quat,
    get_state_from_bullet,
    find_wheel_joints,
    apply_diff_drive_control,
    estimate_eta_along_field,  # returns (eta_lower, eta_upper)
)

DRAW_FLOWFIELD = True

# Offline field-based time profile scales
# Lower = tuned "optimistic" scale
OFFLINE_TIME_SCALE_LOWER = 3.0

# Upper = conservative safety factor over the lower bound
OFFLINE_TIME_SCALE_UPPER = 6.0

# Ratio between upper and lower
OFFLINE_TIME_RATIO = OFFLINE_TIME_SCALE_UPPER / OFFLINE_TIME_SCALE_LOWER


# =========================================================
#              PATH-GUIDANCE FLOW FIELD
# =========================================================

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
                 k_t=1.0,
                 k_n=30.0,
                 band_radius=1.0):
        super().__init__(world_xmin, world_xmax,
                         world_ymin, world_ymax,
                         grid_w, grid_h,
                         rover_radius, pebble_radius, clearance)
        self.k_t = float(k_t)
        self.k_n = float(k_n)
        self.band_radius = float(band_radius)
        self.path_points = None

    # ---------- geometry helpers ----------

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

    # ---------- build path-based vector field ----------

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
#        PATH GEOMETRY (ARC LENGTH ONLY)
# =========================================================

def precompute_arc_length(path_points):
    pts = np.array(path_points, dtype=float)
    segs = pts[1:] - pts[:-1]
    seg_lens = np.linalg.norm(segs, axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_L = s_cum[-1]
    return pts, segs, seg_lens, s_cum, total_L


def project_point_to_path_s(p_world, pts, segs, seg_lens, s_cum):
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


# =========================================================
#   FIELD-BASED OFFLINE TIME PROFILE ALONG THE PATH
# =========================================================

def precompute_field_time_profile(field,
                                  pts,
                                  s_cum,
                                  landmark_s,
                                  goal_pos,
                                  v_trans,
                                  w_turn,
                                  eta_scale=1.0,
                                  max_steps=3000):
    """
    Build T_field(s) from the vector field:
      - Integrate from path vertices to goal via estimate_eta_along_field.
      - T_field(j) = eta_scale * (ETA_start_up - ETA_up(j->goal)), then clipped and monotone.
      - From T_field(s) derive nominal v_nom(s) = ds/dt.
      - Also output offline times for landmarks (with the same scale).
    """
    pts = np.asarray(pts, dtype=float)
    N = len(pts)
    goal_pos = np.asarray(goal_pos, dtype=float)

    # ETA from path start to goal
    eta_start_low, eta_start_up = estimate_eta_along_field(
        field,
        start_world=(pts[0, 0], pts[0, 1]),
        goal_world=(goal_pos[0], goal_pos[1]),
        v_trans=v_trans,
        w_turn=w_turn,
        max_steps=max_steps,
    )
    if eta_start_up is None:
        print("WARNING: start->goal ETA (field) unreachable.")
        T_nodes = np.linspace(0.0, 1.0, N)
        v_nom_nodes = np.ones(N) * v_trans
        landmark_T_field = [0.0 for _ in landmark_s]
        return T_nodes, np.asarray(landmark_T_field), T_nodes[-1], v_nom_nodes

    # Remaining ETA from each path point to goal (upper bound from field)
    eta_rem_up_nodes = np.zeros(N, dtype=float)
    for j in range(N):
        start_world = (pts[j, 0], pts[j, 1])
        low_j, up_j = estimate_eta_along_field(
            field,
            start_world=start_world,
            goal_world=(goal_pos[0], goal_pos[1]),
            v_trans=v_trans,
            w_turn=w_turn,
            max_steps=max_steps,
        )
        if up_j is None:
            eta_rem_up_nodes[j] = 0.0
        else:
            eta_rem_up_nodes[j] = up_j

    # Absolute offline time along path (scaled)
    T_nodes = eta_scale * (eta_start_up - eta_rem_up_nodes)
    T_nodes = np.maximum(0.0, T_nodes)

    # enforce monotone non-decreasing vs s
    for j in range(1, N):
        if T_nodes[j] < T_nodes[j - 1]:
            T_nodes[j] = T_nodes[j - 1]

    # nominal tangential speed ds/dT_field
    v_nom_nodes = np.zeros(N, dtype=float)
    counts = np.zeros(N, dtype=float)
    for j in range(N - 1):
        ds = s_cum[j + 1] - s_cum[j]
        dt = T_nodes[j + 1] - T_nodes[j]
        if dt <= 1e-6:
            continue
        v_seg = ds / dt
        v_nom_nodes[j] += v_seg
        v_nom_nodes[j + 1] += v_seg
        counts[j] += 1.0
        counts[j + 1] += 1.0
    counts = np.maximum(counts, 1.0)
    v_nom_nodes /= counts
    v_nom_nodes = np.maximum(v_nom_nodes, 0.05)

    # offline field-based times for landmarks
    landmark_T_field = []
    for s_lm in landmark_s:
        T_lm = interpolate_along_path(s_lm, s_cum, T_nodes)
        landmark_T_field.append(T_lm)

    T_total = T_nodes[-1]
    return T_nodes, np.asarray(landmark_T_field), T_total, v_nom_nodes


# =========================================================
#        TIMING PLOT
# =========================================================

def plot_timing_history(time_hist,
                        s_hist,
                        total_L,
                        landmark_fracs,
                        landmark_T_field,
                        landmark_actual):
    import matplotlib.pyplot as plt

    time_arr = np.asarray(time_hist)
    frac_arr = np.asarray(s_hist) / max(total_L, 1e-6)

    plt.figure()
    plt.plot(time_arr, frac_arr, label="s(t)/L")

    for frac, T_off, t_act in zip(landmark_fracs, landmark_T_field, landmark_actual):
        label_off = f"{int(frac * 100)}% offline(field)"
        plt.axvline(T_off, linestyle="--", linewidth=1.0, label=label_off)
        if t_act is not None:
            label_act = f"{int(frac * 100)}% actual"
            plt.axvline(t_act, linestyle="-.", linewidth=1.0, label=label_act)

    plt.xlabel("time [s]")
    plt.ylabel("path fraction s/L")
    plt.ylim(0.0, 1.05)
    plt.title("Field-based offline ETA vs actual")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


# =========================================================
#        SAMPLE PATH
# =========================================================

def build_sample_path(start_pos,
                      goal_pos,
                      num_points=80,
                      wiggle_ampl=1.5,
                      wiggle_cycles=1.0,
                      mode="sine"):
    start_pos = np.asarray(start_pos, dtype=float)
    goal_pos = np.asarray(goal_pos, dtype=float)
    ts = np.linspace(0.0, 1.0, num_points)
    pts = []

    dir_vec = goal_pos - start_pos
    if np.linalg.norm(dir_vec) < 1e-6:
        dir_hat = np.array([1.0, 0.0])
    else:
        dir_hat = dir_vec / (np.linalg.norm(dir_vec) + 1e-9)
    ortho = np.array([-dir_hat[1], dir_hat[0]])

    for t in ts:
        base = (1.0 - t) * start_pos + t * goal_pos

        if mode == "straight":
            offset = np.zeros(2)
        elif mode == "sine":
            offset = wiggle_ampl * math.sin(2 * math.pi * wiggle_cycles * t) * ortho
        elif mode == "hairpin":
            sign = 1.0 if t < 0.5 else -1.0
            smooth = math.sin(math.pi * t)
            offset = sign * wiggle_ampl * smooth * ortho
        else:
            offset = np.zeros(2)

        pt = base + offset
        pts.append((float(pt[0]), float(pt[1])))

    return pts


# =========================================================
#        MAIN SIMULATION
# =========================================================

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ROVER_URDF_PATH = os.path.join(here, "2_wheel_rover.urdf")
    PEBBLE_URDF_PATH = os.path.join(here, "pebbles.urdf")

    # --- PyBullet setup ---
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=3.0,
                                 cameraYaw=45,
                                 cameraPitch=-60,
                                 cameraTargetPosition=[0, 0, 0])
    p.setGravity(0, 0, -9.81)
    sim_dt = 1.0 / 240.0
    p.setTimeStep(sim_dt)
    plane_id = p.loadURDF("plane.urdf")

    # --- Environment parameters ---
    env_radius = 3.0
    num_pebbles = 0
    random_seed = 41

    # --- Path start and goal ---
    start_pos = np.array([2.0, 2.0])
    goal_pos = np.array([-2.0, -2.0])

    PATH_MODE = "sine_hard"  # "straight", "sine_easy", "sine_hard", "hairpin"

    if PATH_MODE == "straight":
        path_points = build_sample_path(
            start_pos, goal_pos,
            num_points=80,
            wiggle_ampl=0.0,
            wiggle_cycles=0.0,
            mode="straight",
        )
    elif PATH_MODE == "sine_easy":
        path_points = build_sample_path(
            start_pos, goal_pos,
            num_points=80,
            wiggle_ampl=0.3,
            wiggle_cycles=1.0,
            mode="sine",
        )
    elif PATH_MODE == "sine_hard":
        path_points = build_sample_path(
            start_pos, goal_pos,
            num_points=80,
            wiggle_ampl=0.5,
            wiggle_cycles=3.0,
            mode="sine",
        )
    elif PATH_MODE == "hairpin":
        path_points = build_sample_path(
            start_pos, goal_pos,
            num_points=80,
            wiggle_ampl=0.9,
            wiggle_cycles=1.0,
            mode="hairpin",
        )
    else:
        raise ValueError(f"Unknown PATH_MODE: {PATH_MODE}")

    # --- Path geometry ---
    pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(path_points)

    landmark_fracs = [0.25, 0.5, 0.75, 1.0]
    landmark_s = [f * total_L for f in landmark_fracs]
    landmark_pred = [None] * len(landmark_s)
    landmark_actual = [None] * len(landmark_s)

    s_prev = 0.0
    t_prev = time.time()
    v_s_ema = 0.0
    ema_alpha = 0.15

    # --- Draw path (yellow) ---
    xs = [pt[0] for pt in path_points]
    ys = [pt[1] for pt in path_points]
    for i in range(len(path_points) - 1):
        a = path_points[i]
        b = path_points[i + 1]
        p.addUserDebugLine([a[0], a[1], 0.03],
                           [b[0], b[1], 0.03],
                           [1, 1, 0],
                           lifeTime=0)

    # grid bounds
    path_margin = 1.0
    world_xmin = min(xs) - path_margin
    world_xmax = max(xs) + path_margin
    world_ymin = min(ys) - path_margin
    world_ymax = max(ys) + path_margin

    desired_cell = 0.15
    grid_w = int(math.ceil((world_xmax - world_xmin) / desired_cell)) + 1
    grid_h = int(math.ceil((world_ymax - world_ymin) / desired_cell)) + 1

    print(f"Flow field bounds: x[{world_xmin:.2f}, {world_xmax:.2f}], "
          f"y[{world_ymin:.2f}, {world_ymax:.2f}], "
          f"grid={grid_w}x{grid_h}")

    # --- Rover ---
    START_YAW = math.radians(-90.0)
    agent = {
        "id": "R0",
        "state": np.array([start_pos[0], start_pos[1], START_YAW, 0.0, 0.0]),
        "goal": goal_pos.copy(),
        "control": (0.0, 0.0),
        "color": (0.1, 0.5, 1.0, 1.0),
    }

    body_id = p.loadURDF(
        ROVER_URDF_PATH,
        basePosition=[start_pos[0], start_pos[1], 0.02],
        baseOrientation=yaw_to_quat(START_YAW),
        useFixedBase=False,
    )
    left_j, right_j = find_wheel_joints(body_id)
    agent["body"] = body_id
    agent["left_joint"] = left_j
    agent["right_joint"] = right_j

    r, g, b, a = agent["color"]
    p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, a])

    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    p.changeDynamics(agent["body"], -1, lateralFriction=0.8)
    for link in (agent["left_joint"], agent["right_joint"]):
        p.changeDynamics(agent["body"], link,
                         lateralFriction=1.0,
                         rollingFriction=0.0,
                         spinningFriction=0.0)

    for j in (agent["left_joint"], agent["right_joint"]):
        p.setJointMotorControl2(
            agent["body"], j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=0.0
        )

    # --- optional pebbles (here none) ---
    np.random.seed(random_seed)
    pebble_ids = []
    pebble_centers = []
    pebble_radius = 0.05

    for i in range(num_pebbles):
        r_rand = env_radius * math.sqrt(np.random.rand())
        phi = 2 * math.pi * np.random.rand()
        px = r_rand * math.cos(phi)
        py = r_rand * math.sin(phi)
        bid = p.loadURDF(
            PEBBLE_URDF_PATH,
            basePosition=[px, py, 0.01],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=False,
            globalScaling=1.0,
        )
        pebble_ids.append(bid)
        pebble_centers.append((px, py))
        p.changeDynamics(bid, -1, lateralFriction=0.8)

    # --- Path-guidance field ---
    pgf = PathGuidanceField2D(
        world_xmin=world_xmin,
        world_xmax=world_xmax,
        world_ymin=world_ymin,
        world_ymax=world_ymax,
        grid_w=grid_w,
        grid_h=grid_h,
        rover_radius=0.25,
        pebble_radius=pebble_radius,
        clearance=0.02,
        k_t=2.0,
        k_n=10.0,
        band_radius=0.5,
    )
    t0_build = time.time()
    pgf.rebuild_for_path(path_points, pebble_centers)
    print(f"Flow field build took {time.time() - t0_build:.3f} seconds")

    if DRAW_FLOWFIELD:
        pgf.draw_debug(scale=0.2, life_time=0.0)

    # --- Visualize goal ---
    goal_vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.08,
        rgbaColor=[0, 1, 0, 1]
    )
    p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=goal_vis,
        basePosition=[goal_pos[0], goal_pos[1], 0.05],
    )

    # --- Controller ---
    controller = FlowFieldController(
        v_max=1.0,
        w_max=6.0,
        k_theta=6.0,
        turn_in_place_angle_deg=50.0,
        static_speed_threshold=0.03,
        w_turn_in_place=25.0,
    )

    # --- Offline field-based time profile along path (lower scale) ---
    T_field_nodes, landmark_T_field, T_field_total, v_nom_nodes = \
        precompute_field_time_profile(
            pgf,
            pts,
            s_cum,
            landmark_s,
            goal_pos,
            v_trans=0.8 * controller.v_max,
            w_turn=controller.w_turn_in_place,
            eta_scale=OFFLINE_TIME_SCALE_LOWER,
            max_steps=3000,
        )

    # Upper bound time profile is a scaled copy
    T_field_nodes_hi = OFFLINE_TIME_RATIO * T_field_nodes
    T_field_total_hi = OFFLINE_TIME_RATIO * T_field_total
    landmark_T_field_hi = OFFLINE_TIME_RATIO * landmark_T_field

    print(
        f"Offline field-based ETA (start->goal): "
        f"lower={T_field_total:.2f}s, upper={T_field_total_hi:.2f}s"
    )

    print("Path-following with field-integrated ETA. Close the GUI window to stop.")
    acc = 0.0
    sim_start = time.time()
    t_print = sim_start

    S_TOL = 0.18
    V_TOL = 0.03
    STOP_HOLD_TIME = 0.4
    low_speed_acc = 0.0

    VS_MIN_FOR_ETA = 0.05

    time_hist = []
    s_hist = []

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        if acc >= 0.05:
            acc = 0.0
            agent["state"] = get_state_from_bullet(agent["body"])
            x, y, yaw, v_fwd, w = agent["state"]

            # shovel
            SHOVEL_OFFSET = 0.17
            x_s = x + SHOVEL_OFFSET * math.cos(yaw)
            y_s = y + SHOVEL_OFFSET * math.sin(yaw)
            p_shovel = np.array([x_s, y_s], dtype=float)

            now = time.time()
            t_rel = now - sim_start

            # progress along path (shovel)
            s_now, d_now = project_point_to_path_s(
                p_shovel, pts, segs, seg_lens, s_cum
            )

            time_hist.append(t_rel)
            s_hist.append(s_now)

            # tangential speed along path
            dt = max(1e-6, now - t_prev)
            v_s = (s_now - s_prev) / dt
            v_s_ema = (1.0 - ema_alpha) * v_s_ema + ema_alpha * v_s
            s_prev = s_now
            t_prev = now

            # offline field time at this s (lower and upper)
            T_now = interpolate_along_path(s_now, s_cum, T_field_nodes)
            T_now_hi = OFFLINE_TIME_RATIO * T_now
            T_total = T_field_total
            T_total_hi = T_field_total_hi

            T_rem_field = max(0.0, T_total - T_now)
            T_rem_field_hi = OFFLINE_TIME_RATIO * T_rem_field

            # nominal tangential speed at this s
            v_nom_loc = interpolate_along_path(s_now, s_cum, v_nom_nodes)

            # speed factor for online correction
            use_online = False
            speed_factor = 1.0
            if abs(v_s_ema) > VS_MIN_FOR_ETA:
                speed_factor = v_nom_loc / max(VS_MIN_FOR_ETA, abs(v_s_ema))
                speed_factor = max(0.2, min(speed_factor, 5.0))
                use_online = True

            # ETA to END: offline (range) and online (single best)
            eta_end_off = T_rem_field
            eta_end_off_hi = T_rem_field_hi
            eta_end_on = speed_factor * eta_end_off if (use_online and eta_end_off > 0.05) else None

            # ETA to NEXT LANDMARK: offline (range) and online (single best)
            next_idx = None
            for i_lm, s_lm in enumerate(landmark_s):
                if s_now < s_lm:
                    next_idx = i_lm
                    break

            eta_lm_off = None
            eta_lm_off_hi = None
            eta_lm_on = None
            lm_frac_print = None
            if next_idx is not None:
                T_lm_off = landmark_T_field[next_idx]
                rem_lm = max(0.0, T_lm_off - T_now)
                eta_lm_off = rem_lm
                eta_lm_off_hi = OFFLINE_TIME_RATIO * rem_lm
                if use_online and rem_lm > 0.05:
                    eta_lm_on = speed_factor * rem_lm
                lm_frac_print = int(landmark_fracs[next_idx] * 100)

            # record landmark predictions (online) for error at crossing
            for i_lm, s_lm in enumerate(landmark_s):
                if landmark_actual[i_lm] is None and s_now < s_lm and use_online:
                    T_lm_off = landmark_T_field[i_lm]
                    delta_T_off = max(0.0, T_lm_off - T_now)
                    if delta_T_off > 0.0:
                        landmark_pred[i_lm] = t_rel + speed_factor * delta_T_off

            # detect reaching landmarks and compare offline (low / high) vs online
            for i_lm, s_lm in enumerate(landmark_s):
                if landmark_actual[i_lm] is None and s_now >= s_lm:
                    landmark_actual[i_lm] = t_rel
                    T_lm_off = landmark_T_field[i_lm]
                    T_lm_off_hi = landmark_T_field_hi[i_lm]
                    err_offline_low = landmark_actual[i_lm] - T_lm_off
                    pred_t = landmark_pred[i_lm]
                    if pred_t is not None:
                        err_online = landmark_actual[i_lm] - pred_t
                        print(
                            f"Reached landmark {landmark_fracs[i_lm] * 100:.0f}% "
                            f"at t={landmark_actual[i_lm]:.2f}s  "
                            f"(offline_low={T_lm_off:.2f}s, offline_high={T_lm_off_hi:.2f}s, "
                            f"err_low={err_offline_low:+.2f}s; "
                            f"online_pred={pred_t:.2f}s, err_online={err_online:+.2f}s)"
                        )
                    else:
                        print(
                            f"Reached landmark {landmark_fracs[i_lm] * 100:.0f}% "
                            f"at t={landmark_actual[i_lm]:.2f}s  "
                            f"(offline_low={T_lm_off:.2f}s, offline_high={T_lm_off_hi:.2f}s, "
                            f"err_low={err_offline_low:+.2f}s; "
                            f"no reliable online prediction)"
                        )

            # debug print once per about 1 s
            if now - t_print > 1.0:
                t_print = now
                line = (
                    f"s={s_now:.2f}/{total_L:.2f}  d_to_path={d_now:.2f}  "
                    f"v_s={v_s_ema:.2f}  v_nom_field={v_nom_loc:.2f}  "
                    f"ETA_end_off=[{eta_end_off:.1f},{eta_end_off_hi:.1f}]s"
                )
                if eta_end_on is not None:
                    line += f"  ETA_end_on={eta_end_on:.1f}s"
                if eta_lm_off is not None and lm_frac_print is not None:
                    line += f"  ETA_lm{lm_frac_print}_off=[{eta_lm_off:.1f},{eta_lm_off_hi:.1f}]s"
                    if eta_lm_on is not None:
                        line += f"  ETA_lm{lm_frac_print}_on={eta_lm_on:.1f}s"
                print(line)

            # --- control from field (shovel point) ---
            tracking_state = np.array([x_s, y_s, yaw, v_fwd, w], dtype=float)
            v_cmd, w_cmd = controller.compute_control(tracking_state, agent["goal"], pgf)
            agent["control"] = (v_cmd, w_cmd)

            # stop when near end of path with low s-velocity
            remaining_end = max(0.0, total_L - s_now)
            if remaining_end < S_TOL:
                if abs(v_s_ema) < V_TOL:
                    low_speed_acc += 0.05
                else:
                    low_speed_acc = 0.0

                if low_speed_acc >= STOP_HOLD_TIME:
                    agent["control"] = (0.0, 0.0)
                    print(
                        f"Reached end of path "
                        f"(remaining={remaining_end:.3f}m, v_s={v_s_ema:.3f}m/s)."
                    )
                    err_total_low = t_rel - T_field_total
                    print(
                        "End-of-path timing: "
                        f"offline_low={T_field_total:.2f}s, "
                        f"offline_high={T_field_total_hi:.2f}s, "
                        f"actual={t_rel:.2f}s, "
                        f"err_low={err_total_low:+.2f}s"
                    )
                    break
            else:
                low_speed_acc = 0.0

        apply_diff_drive_control(agent)

    if len(time_hist) > 0:
        plot_timing_history(
            time_hist,
            s_hist,
            total_L,
            landmark_fracs,
            landmark_T_field,
            landmark_actual,
        )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(sim_dt)

    p.disconnect()


if __name__ == "__main__":
    main()
