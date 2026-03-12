import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
import time

# Import your existing utilities
from flowfield_pybullet import (
    FlowField2D,
    FlowFieldController,
    yaw_to_quat,
    get_state_from_bullet,
    find_wheel_joints,
    apply_diff_drive_control,
)

# Set True to visualize the flow field (blue arrows + red crosses)
DRAW_FLOWFIELD = False


# =========================================================
#              PATH-GUIDANCE FLOW FIELD
# =========================================================

class PathGuidanceField2D(FlowField2D):
    """
    Vector-field-based path following on top of the same 2D grid structure.

    - You provide a polyline path in world coordinates: [(x0,y0), (x1,y1), ...].
    - For each grid cell, we:
        1) Find the closest point on the path and its segment tangent.
        2) Compute signed lateral error to the path.
        3) Create a vector v = k_t * t_hat - k_n * e_n * n_hat
           where:
               t_hat = unit tangent along path
               n_hat = left-hand unit normal
               e_n   = lateral error (signed distance to path)
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
                 k_n=30,
                 band_radius=1.0):
        super().__init__(world_xmin, world_xmax,
                         world_ymin, world_ymax,
                         grid_w, grid_h,
                         rover_radius, pebble_radius, clearance)
        self.k_t = k_t
        self.k_n = k_n
        self.band_radius = band_radius
        self.path_points = None  # (N,2)


    # ---------- geometry helpers ----------

    def _closest_point_on_path(self, px, py):
        """
        Given a world point (px, py), find:
          - closest point on the polyline
          - unit tangent at that closest point
        Returns (closest_point (2,), tangent_hat (2,))
        """
        assert self.path_points is not None and len(self.path_points) >= 2

        p_world = np.array([px, py], dtype=float)
        best_dist2 = float("inf")
        best_closest = None
        best_tangent = None

        # iterate over segments
        for i in range(len(self.path_points) - 1):
            a = self.path_points[i]
            b = self.path_points[i + 1]
            ab = b - a
            ab_len2 = float(np.dot(ab, ab))
            if ab_len2 < 1e-9:
                continue

            # projection of p_world onto segment [a,b]
            t = float(np.dot(p_world - a, ab) / ab_len2)
            t = max(0.0, min(1.0, t))  # clamp to segment
            closest = a + t * ab
            diff = p_world - closest
            d2 = float(np.dot(diff, diff))
            if d2 < best_dist2:
                best_dist2 = d2
                best_closest = closest
                # tangent along the segment
                ab_len = math.sqrt(ab_len2)
                t_hat = ab / (ab_len + 1e-9)
                best_tangent = t_hat

        if best_closest is None:
            # fallback: just use the first segment direction
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
        Build the direction field only in a corridor (band_radius) around the path.
        """
        self.path_points = np.array(path_points, dtype=float)
        gh, gw = self.grid_h, self.grid_w
        self.dir_field[:, :, :] = 0.0

        # --- build candidate mask: cells within band_radius of the path ---
        cell_size = min(self.cell_w, self.cell_h)
        band_cells = int(math.ceil(self.band_radius / cell_size))

        mask = np.zeros((gh, gw), dtype=bool)

        for (px, py) in self.path_points:
            ix, iy = self.world_to_cell(px, py)

            # stamp a disk of radius band_cells around each path point
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

        # --- now compute vectors ONLY for masked cells ---
        ys, xs = np.where(mask)
        for iy, ix in zip(ys, xs):
            if self.obstacles[iy, ix]:
                continue

            cx, cy = self.cell_to_world_center(ix, iy)
            path_pt, t_hat = self._closest_point_on_path(cx, cy)

            # left-hand normal
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
        Public API to rebuild the field:
        - stamp static obstacles from pebbles
        - compute path-based vector field
        """
        self.clear_obstacles()
        self.stamp_pebbles(pebble_centers)
        self.compute_path_direction_field(path_points)


# =========================================================
#        SAMPLE PATH + SIMPLE PATH-FOLLOWING MAIN
# =========================================================

def precompute_arc_length(path_points):
    pts = np.array(path_points, dtype=float)
    segs = pts[1:] - pts[:-1]
    seg_lens = np.linalg.norm(segs, axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_L = s_cum[-1]
    return pts, segs, seg_lens, s_cum, total_L


def project_point_to_path_s(p_world, pts, segs, seg_lens, s_cum):
    """
    Project point p_world onto polyline.
    Returns:
      s_star : arc-length position of closest projection
      d_star : distance to path at that projection
    """
    best_d2 = float("inf")
    best_s = 0.0

    for i in range(len(segs)):
        a = pts[i]
        ab = segs[i]
        L2 = seg_lens[i]**2
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

def build_sample_path(start_pos,
                      goal_pos,
                      num_points=80,
                      wiggle_ampl=1.5,
                      wiggle_cycles=1.0,
                      mode="sine"):
    """
    Build a 2D path between start_pos and goal_pos.

    Parameters:
        wiggle_ampl   : amplitude of the lateral "wiggle" (meters)
        wiggle_cycles : how many full sine cycles between start and goal
        mode          : "straight", "sine", or "hairpin"
    """
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
            # smooth S-shape, more cycles => more frequent turns
            offset = wiggle_ampl * math.sin(2 * math.pi * wiggle_cycles * t) * ortho
        elif mode == "hairpin":
            # sharper, plateaued side excursions (like hairpins)
            # sign flips around the mid-point, with saturated offset
            sign = 1.0 if t < 0.5 else -1.0
            # optional smoothing window
            smooth = math.sin(math.pi * t)
            offset = sign * wiggle_ampl * smooth * ortho
        else:
            # fallback: straight
            offset = np.zeros(2)

        pt = base + offset
        pts.append((float(pt[0]), float(pt[1])))

    return pts


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
    # start with no pebbles so you see pure path-following behavior.
    num_pebbles = 0
    random_seed = 41

    # --- Define start and goal for the path-following task ---
    start_pos = np.array([2.0, 2.0])
    goal_pos = np.array([-2.0, -2.0])

    # --- Build a sample S-shaped path between them ---
    # Mild S-curve
    # path_points = build_sample_path(start_pos, goal_pos,
    #                                 num_points=150,
    #                                 wiggle_ampl=1.0,
    #                                 wiggle_cycles=1.0,
    #                                 mode="sine")

    # Stronger, tighter S-curves (more turning)
    path_points = build_sample_path(start_pos, goal_pos,
                                    num_points=80,
                                    wiggle_ampl=0.5,
                                    wiggle_cycles=2.5,
                                    mode="sine")

    # Hairpin-style path (big sideways excursions, sharper direction changes)
    # path_points = build_sample_path(start_pos, goal_pos,
    #                                 num_points=200,
    #                                 wiggle_ampl=3.0,
    #                                 wiggle_cycles=1.0,
    #                                 mode="hairpin")

    # Straight line for comparison
    # path_points = build_sample_path(start_pos, goal_pos,
    #                                 num_points=150,
    #                                 wiggle_ampl=0.0,
    #                                 wiggle_cycles=0.0,
    #                                 mode="straight")

    # Precompute arc-length parameterization
    pts, segs, seg_lens, s_cum, total_L = precompute_arc_length(path_points)

    # Landmarks as fractions of path length (edit as you like)
    landmark_fracs = [0.25, 0.5, 0.75, 1.0]
    landmark_s = [f * total_L for f in landmark_fracs]
    landmark_times = [None] * len(landmark_s)

    # For ETA estimate
    s_prev = 0.0
    t_prev = time.time()
    v_s_ema = 0.0
    ema_alpha = 0.15  # smoothing for speed along path

    # --- Visualise path in PyBullet (yellow polyline) ---
    for i in range(len(path_points) - 1):
        a = path_points[i]
        b = path_points[i + 1]
        p.addUserDebugLine([a[0], a[1], 0.03],
                           [b[0], b[1], 0.03],
                           [1, 1, 0],
                           lifeTime=0)

        # === NEW: compute world bounds just around the path ===
        xs = [pt[0] for pt in path_points]
        ys = [pt[1] for pt in path_points]

        path_margin = 1.0  # [m] extra area around path (tune as you like)

        world_xmin = min(xs) - path_margin
        world_xmax = max(xs) + path_margin
        world_ymin = min(ys) - path_margin
        world_ymax = max(ys) + path_margin

        # desired cell size in meters (same everywhere)
        desired_cell = 0.15  # ~15 cm cells is pretty dense

        grid_w = int(math.ceil((world_xmax - world_xmin) / desired_cell)) + 1
        grid_h = int(math.ceil((world_ymax - world_ymin) / desired_cell)) + 1

        if i==0:
            print(f"Flow field bounds: x[{world_xmin:.2f}, {world_xmax:.2f}], "
                  f"y[{world_ymin:.2f}, {world_ymax:.2f}], "
                  f"grid={grid_w}x{grid_h}")
    # --- Spawn rover ---
    START_YAW = math.radians(-90.0)
    agent = {
        "id": "R0",
        "state": np.array([start_pos[0], start_pos[1], START_YAW, 0.0, 0.0]),
        "goal": goal_pos.copy(),  # used only for stop condition in controller
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

    # friction settings
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    p.changeDynamics(agent["body"], -1, lateralFriction=0.8)
    for link in (agent["left_joint"], agent["right_joint"]):
        p.changeDynamics(agent["body"], link,
                         lateralFriction=1.0,
                         rollingFriction=0.0,
                         spinningFriction=0.0)

    # disable built-in wheel motors; we drive by applying torque ourselves
    for j in (agent["left_joint"], agent["right_joint"]):
        p.setJointMotorControl2(
            agent["body"], j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=0.0
        )

    # --- Optional: spawn pebbles as static obstacles ---
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

    # --- Build path-guidance flow field on a finer grid (81x81) ---
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
    t0 = time.time()
    pgf.rebuild_for_path(path_points, pebble_centers)
    print(f"Flow field build took {time.time() - t0:.3f} seconds")

    if DRAW_FLOWFIELD:
        # same style as in flowfield_pybullet.py
        pgf.draw_debug(scale=0.2, life_time=0.0)

    # --- Visualise goal (end of path) ---
    goal_vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.08,
        rgbaColor=[0, 1, 0, 1]
    )
    goal_body = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=goal_vis,
        basePosition=[goal_pos[0], goal_pos[1], 0.05],
    )

    # --- Controller (same as in flowfield_pybullet) ---
    controller = FlowFieldController(
        v_max=1.0,
        w_max=6.0,
        k_theta=6.0,
        turn_in_place_angle_deg=50.0,
        static_speed_threshold=0.03,
        w_turn_in_place=25.0,
    )

    print("Path-following with vector-field guidance. Close the GUI window to stop.")
    acc = 0.0
    t0 = time.time()
    sim_start = time.time()
    landmark_pred = [None] * len(landmark_s)  # running predicted arrival times (seconds since start)
    landmark_actual = [None] * len(landmark_s)
    # --- stopping criteria (shovel-based) ---
    S_TOL = 0.11  # [m] remaining arc-length to consider "done"
    V_TOL = 0.02  # [m/s] tangential speed considered "stopped"
    STOP_HOLD_TIME = 1.0  # [s] require low speed for this long

    low_speed_acc = 0.0

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        if acc >= 0.05:  # ~20 Hz control
            acc = 0.0
            agent["state"] = get_state_from_bullet(agent["body"])
            x, y, yaw, v_fwd, w = agent["state"]

            # --- shovel tracking point ---
            SHOVEL_OFFSET = 0.17
            x_s = x + SHOVEL_OFFSET * math.cos(yaw)
            y_s = y + SHOVEL_OFFSET * math.sin(yaw)

            # ========= DROP THE ETA/LANDMARK SNIPPET HERE =========
            now = time.time()

            # shovel point in world
            p_shovel = np.array([x_s, y_s], dtype=float)

            # progress along path
            s_now, d_now = project_point_to_path_s(p_shovel, pts, segs, seg_lens, s_cum)

            # tangential speed estimate ds/dt
            dt = max(1e-6, now - t_prev)
            v_s = (s_now - s_prev) / dt
            v_s_ema = (1 - ema_alpha) * v_s_ema + ema_alpha * v_s


            s_prev = s_now
            t_prev = now

            t_rel = now - sim_start  # seconds since start

            # --- keep updating prediction for landmarks not yet reached ---
            for i, s_lm in enumerate(landmark_s):
                if landmark_actual[i] is None and s_now < s_lm:
                    remaining_lm = s_lm - s_now
                    if s_now > 0.3 and v_s_ema > 0.05:
                        landmark_pred[i] = t_rel + remaining_lm / v_s_ema

            # --- detect reaching landmarks and compare to last prediction ---
            for i, s_lm in enumerate(landmark_s):
                if landmark_actual[i] is None and s_now >= s_lm:
                    landmark_actual[i] = t_rel
                    pred_t = landmark_pred[i]
                    if pred_t is not None:
                        err = landmark_actual[i] - pred_t
                        print(
                            f"Reached landmark {landmark_fracs[i] * 100:.0f}% "
                            f"at t={landmark_actual[i]:.2f}s "
                            f"(pred {pred_t:.2f}s, err {err:+.2f}s)"
                        )
                    else:
                        print(
                            f"Reached landmark {landmark_fracs[i] * 100:.0f}% "
                            f"at t={landmark_actual[i]:.2f}s (no reliable pred yet)"
                        )

            # --- ETA to finish (only when meaningful) ---
            remaining = max(0.0, total_L - s_now)
            if s_now > 0.3 and remaining > 0.2 and v_s_ema > 0.05:
                eta = remaining / v_s_ema
                if time.time() - t0 > 1.0:
                    print(f"s={s_now:.2f}/{total_L:.2f}  d_to_path={d_now:.2f}  "
                          f"v_s={v_s_ema:.2f}  ETA={eta:.1f}s")
            else:
                if time.time() - t0 > 1.0:
                    print(f"s={s_now:.2f}/{total_L:.2f}  d_to_path={d_now:.2f}  v_s={v_s_ema:.2f}")

            # =======================================================

            # --- now compute control using shovel point ---
            tracking_state = np.array([x_s, y_s, yaw, v_fwd, w], dtype=float)
            v_cmd, w_cmd = controller.compute_control(tracking_state, agent["goal"], pgf)
            agent["control"] = (v_cmd, w_cmd)

            # --- stop based on shovel progress along the path ---
            remaining_end = max(0.0, total_L - s_now)

            if remaining_end < S_TOL:
                # confirm we actually settled (not just slowed briefly)
                if v_s_ema < V_TOL:
                    low_speed_acc += 0.05  # control period
                else:
                    low_speed_acc = 0.0

                if low_speed_acc >= STOP_HOLD_TIME:
                    agent["control"] = (0.0, 0.0)
                    print(f"Reached end of path (remaining={remaining_end:.3f}m, v_s={v_s_ema:.3f}m/s).")
                    break
            else:
                low_speed_acc = 0.0

        apply_diff_drive_control(agent)

    # keep window open a bit after stop
    for _ in range(240):
        p.stepSimulation()
        time.sleep(sim_dt)

    p.disconnect()


if __name__ == "__main__":
    main()
