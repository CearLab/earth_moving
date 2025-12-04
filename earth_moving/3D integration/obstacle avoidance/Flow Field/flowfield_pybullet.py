import os
import time
import math
import heapq
import numpy as np
import pybullet as p
import pybullet_data

# =========================================================
#     FLOW FIELD NAVIGATION WITH SHOVEL-POINT TRACKING
# =========================================================
# This implementation uses shovel-point tracking (0.17m forward offset
# from rover center) to ensure the material collection point reaches
# the goal, rather than the rover's center of mass.
# =========================================================

# =========================================================
#                   FLOW FIELD ON A GRID
# =========================================================
DRAW_FLOWFIELD = False

class FlowField2D:
    """
    Simple goal-based flow field on a 2D grid over the XY plane.

    - World region: [world_xmin, world_xmax] x [world_ymin, world_ymax]
    - Dijkstra from goal cell => distance field
    - Direction = -grad(distance), normalized
    - Obstacles are a boolean grid that we fill from pebble positions.
    """

    def __init__(self,
                 world_xmin=-1.2,
                 world_xmax=+1.2,
                 world_ymin=-1.2,
                 world_ymax=+1.2,
                 grid_w=41,
                 grid_h=41,
                 rover_radius=0.25,
                 pebble_radius=0.05,
                 clearance=0.02):
        self.world_xmin = world_xmin
        self.world_xmax = world_xmax
        self.world_ymin = world_ymin
        self.world_ymax = world_ymax
        self.grid_w = grid_w
        self.grid_h = grid_h

        self.cell_w = (world_xmax - world_xmin) / grid_w
        self.cell_h = (world_ymax - world_ymin) / grid_h

        # size assumptions (meters)
        self.rover_radius = rover_radius
        self.pebble_radius = pebble_radius
        self.clearance = clearance   # extra safety margin

        # grids
        self.obstacles = np.zeros((grid_h, grid_w), dtype=bool)
        self.dist      = np.full((grid_h, grid_w), np.inf, dtype=float)
        self.dir_field = np.zeros((grid_h, grid_w, 2), dtype=float)

        self.goal_cell = None

    # ------------ coordinate transforms ------------

    def world_to_cell(self, x, y):
        ix = int((x - self.world_xmin) / self.cell_w)
        iy = int((y - self.world_ymin) / self.cell_h)
        ix = max(0, min(self.grid_w - 1, ix))
        iy = max(0, min(self.grid_h - 1, iy))
        return ix, iy

    def cell_to_world_center(self, ix, iy):
        x = self.world_xmin + (ix + 0.5) * self.cell_w
        y = self.world_ymin + (iy + 0.5) * self.cell_h
        return x, y

    # ------------ obstacle stamping from pebbles ------------

    def clear_obstacles(self):
        self.obstacles[:, :] = False

    def stamp_pebbles(self, pebble_centers):
        """
        pebble_centers: list of (x, y) in world coordinates.

        We mark as blocked any cell whose center is within:
            rover_radius + pebble_radius + clearance
        of a pebble center. That way the rover disc will not fit there.
        """
        gh, gw = self.grid_h, self.grid_w

        # effective forbidden radius around each pebble
        R_block = self.rover_radius + self.pebble_radius + self.clearance

        for (px, py) in pebble_centers:
            # skip pebbles outside the grid
            if not (self.world_xmin <= px <= self.world_xmax and
                    self.world_ymin <= py <= self.world_ymax):
                continue

            cx, cy = self.world_to_cell(px, py)

            # how many cells to check around the pebble
            max_cells_x = int(math.ceil(R_block / self.cell_w))
            max_cells_y = int(math.ceil(R_block / self.cell_h))

            for dy in range(-max_cells_y, max_cells_y + 1):
                for dx in range(-max_cells_x, max_cells_x + 1):
                    ix = cx + dx
                    iy = cy + dy
                    if ix < 0 or ix >= gw or iy < 0 or iy >= gh:
                        continue
                    wx, wy = self.cell_to_world_center(ix, iy)
                    if math.hypot(wx - px, wy - py) <= R_block:
                        self.obstacles[iy, ix] = True

    # ------------ Dijkstra distance field ------------

    def compute_distance_field(self, goal_cell):
        self.goal_cell = goal_cell
        gh, gw = self.grid_h, self.grid_w
        gx, gy = goal_cell

        self.dist[:, :] = np.inf

        if self.obstacles[gy, gx]:
            print("[FlowField2D] WARNING: goal is inside an obstacle cell.")
            return

        self.dist[gy, gx] = 0.0
        pq = []
        heapq.heappush(pq, (0.0, gx, gy))

        # 8-connected grid
        neighbor_offsets = [
            (-1,  0), (1,  0),
            (0, -1), (0,  1),
            (-1, -1), (-1, 1),
            (1, -1), (1,  1),
        ]

        while pq:
            d_cur, x, y = heapq.heappop(pq)
            if d_cur > self.dist[y, x] + 1e-9:
                continue

            for dx, dy in neighbor_offsets:
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= gw or ny < 0 or ny >= gh:
                    continue
                if self.obstacles[ny, nx]:
                    continue

                step = math.hypot(dx, dy)
                nd = d_cur + step
                if nd < self.dist[ny, nx]:
                    self.dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))

    # ------------ direction field: -grad(dist) ------------

    def compute_direction_field(self):
        gh, gw = self.grid_h, self.grid_w
        self.dir_field[:, :, :] = 0.0

        neighbor_offsets = [
            (-1,  0), (1,  0),
            (0, -1), (0,  1),
            (-1, -1), (-1, 1),
            (1, -1), (1,  1),
        ]

        for y in range(gh):
            for x in range(gw):
                if self.obstacles[y, x]:
                    continue
                d_cur = self.dist[y, x]
                if not math.isfinite(d_cur):
                    continue

                grad_x = 0.0
                grad_y = 0.0

                for dx, dy in neighbor_offsets:
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or nx >= gw or ny < 0 or ny >= gh:
                        continue
                    d_n = self.dist[ny, nx]
                    if not math.isfinite(d_n):
                        continue

                    diff = d_n - d_cur
                    grad_x += diff * dx
                    grad_y += diff * dy

                vx = -grad_x
                vy = -grad_y
                mag = math.hypot(vx, vy)
                if mag < 1e-6:
                    continue
                self.dir_field[y, x, 0] = vx / mag
                self.dir_field[y, x, 1] = vy / mag

    # ------------ public API ------------

    def rebuild(self, goal_world, pebble_centers):
        """
        goal_world: (gx, gy) in world coords
        pebble_centers: list[(x,y)] in world coords
        """
        self.clear_obstacles()
        self.stamp_pebbles(pebble_centers)
        gx_cell, gy_cell = self.world_to_cell(goal_world[0], goal_world[1])
        self.compute_distance_field((gx_cell, gy_cell))
        self.compute_direction_field()

    def get_direction_world(self, x_world, y_world):
        ix, iy = self.world_to_cell(x_world, y_world)
        vx = self.dir_field[iy, ix, 0]
        vy = self.dir_field[iy, ix, 1]
        return np.array([vx, vy], dtype=float)

    def draw_debug(self, scale=0.2, life_time=0.0):
        gh, gw = self.grid_h, self.grid_w
        for y in range(gh):
            for x in range(gw):
                if self.obstacles[y, x]:
                    # draw a small red cross on obstacle cells
                    cx, cy = self.cell_to_world_center(x, y)
                    p.addUserDebugLine([cx - 0.01, cy, 0.02],
                                       [cx + 0.01, cy, 0.02],
                                       [1, 0, 0],
                                       lifeTime=life_time)
                    p.addUserDebugLine([cx, cy - 0.01, 0.02],
                                       [cx, cy + 0.01, 0.02],
                                       [1, 0, 0],
                                       lifeTime=life_time)
                    continue
                vx = self.dir_field[y, x, 0]
                vy = self.dir_field[y, x, 1]
                if abs(vx) < 1e-3 and abs(vy) < 1e-3:
                    continue
                cx, cy = self.cell_to_world_center(x, y)
                start = [cx, cy, 0.02]
                end   = [cx + vx * scale, cy + vy * scale, 0.02]
                p.addUserDebugLine(start, end, [0, 0, 1], lifeTime=life_time)


# =========================================================
#          SIMPLE FLOW-FIELD CONTROLLER (UNICYCLIC)
# =========================================================

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

class FlowFieldController:
    def __init__(self,
                 v_max=0.8,
                 w_max=3.0,
                 k_theta=3.0,
                 turn_in_place_angle_deg=50.0,
                 static_speed_threshold=0.03,
                 w_turn_in_place=5.0):
        """
        v_max: max forward speed
        w_max: max angular speed (for normal tracking)
        k_theta: heading P-gain
        turn_in_place_angle_deg: above this |heading error| (deg),
                                 and when nearly static, we spin in place.
        static_speed_threshold: |v_forward| below this is considered "static".
        w_turn_in_place: angular speed used for turn-in-place (rad/s).
        """
        self.v_max = v_max
        self.w_max = w_max
        self.k_theta = k_theta
        self.turn_in_place_angle = math.radians(turn_in_place_angle_deg)
        self.static_speed_threshold = static_speed_threshold
        self.w_turn_in_place = w_turn_in_place

    def compute_control(self, state, goal_world, flow_field):
        x, y, yaw, v_fwd, w = state
        gx, gy = goal_world

        dg = np.array([gx - x, gy - y], dtype=float)
        dist_goal = np.linalg.norm(dg)

        # Stop near the goal
        if dist_goal < 0.05:
            return 0.0, 0.0

        # Preferred direction from flow field
        v_dir = flow_field.get_direction_world(x, y)
        mag_dir = np.linalg.norm(v_dir)
        if mag_dir < 1e-3:
            v_dir = dg / (dist_goal + 1e-9)
        else:
            v_dir /= mag_dir

        theta_des = math.atan2(v_dir[1], v_dir[0])
        e_theta = wrap_angle(theta_des - yaw)

        # ---------- TURN-IN-PLACE LOGIC ----------
        # If we are almost not moving forward, and heading error is large,
        # then rotate in place quickly with zero forward speed.
        if abs(v_fwd) < self.static_speed_threshold and abs(e_theta) > self.turn_in_place_angle:
            w_cmd = math.copysign(self.w_turn_in_place, e_theta)
            return 0.0, w_cmd
        # -----------------------------------------

        # Normal steering (same as before)
        w_cmd = self.k_theta * e_theta
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))

        align = max(0.0, math.cos(e_theta))  # 1 when aligned, 0 when opposite
        v_base = self.v_max * align
        dist_factor = min(1.0, dist_goal / 0.4)  # slow near goal
        v_cmd = v_base * dist_factor

        return float(v_cmd), float(w_cmd)

# =========================================================
#                 PYBULLET HELPERS (ROVER)
# =========================================================

def yaw_to_quat(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

def get_state_from_bullet(body_id):
    pos, orn = p.getBasePositionAndOrientation(body_id)
    x, y, z = pos
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    lin_vel, ang_vel = p.getBaseVelocity(body_id)
    vx, vy, vz = lin_vel
    wz = ang_vel[2]
    v_forward = math.cos(yaw) * vx + math.sin(yaw) * vy
    w_yaw = wz
    return np.array([x, y, yaw, v_forward, w_yaw])

def find_wheel_joints(body_id):
    left = right = None
    n_joints = p.getNumJoints(body_id)
    for ji in range(n_joints):
        info = p.getJointInfo(body_id, ji)
        name = info[1].decode("utf-8")
        if name == "base_to_lwheel":
            left = ji
        elif name == "base_to_rwheel":
            right = ji
    if left is None or right is None:
        raise RuntimeError("Could not find base_to_lwheel / base_to_rwheel in URDF")
    return left, right

def apply_diff_drive_control(agent,
                             wheel_radius=0.07,
                             track_width=0.20,
                             max_wheel_speed=20.0,
                             max_torque=5.0):
    v_cmd, w_cmd = agent["control"]
    vL = v_cmd - w_cmd * track_width / 2.0
    vR = v_cmd + w_cmd * track_width / 2.0
    wL = -vL / wheel_radius
    wR = -vR / wheel_radius
    wL = max(min(wL, max_wheel_speed), -max_wheel_speed)
    wR = max(min(wR, max_wheel_speed), -max_wheel_speed)
    p.setJointMotorControl2(agent["body"], agent["left_joint"],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=wL,
                            force=max_torque)
    p.setJointMotorControl2(agent["body"], agent["right_joint"],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=wR,
                            force=max_torque)

# =========================================================
#                             MAIN
# =========================================================

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ROVER_URDF_PATH   = os.path.join(here, "2_wheel_rover.urdf")
    PEBBLE_URDF_PATH  = os.path.join(here, "pebbles.urdf")

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

    # --- Environment params (match APF scale) ---
    env_radius = 5.0  # meters
    num_pebbles = 90
    random_seed = 41

    # Rover start and goal (inside env_radius)
    start_pos = np.array([3.5, 2.9])
    goal_pos  = np.array([-2.8, -2.8])

    # --- Spawn rover ---
    agent = {
        "id": "R0",
        "state": np.array([start_pos[0], start_pos[1], 0.0, 0.0, 0.0]),
        "goal": goal_pos.copy(),
        "control": (0.0, 0.0),
        "color": (0.1, 0.5, 1.0, 1.0),
    }

    body_id = p.loadURDF(
        ROVER_URDF_PATH,
        basePosition=[start_pos[0], start_pos[1], 0.02],
        baseOrientation=yaw_to_quat(0.0),
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

    # --- Spawn small pebbles (matching APF style) ---
    np.random.seed(random_seed)
    pebble_ids = []
    pebble_centers = []
    pebble_radius = 0.05  # same as APF integration

    for i in range(num_pebbles):
        # random inside circle env_radius
        r_rand = env_radius * math.sqrt(np.random.rand())
        phi = 2 * math.pi * np.random.rand()
        px = r_rand * math.cos(phi)
        py = r_rand * math.sin(phi)

        bid = p.loadURDF(
            PEBBLE_URDF_PATH,
            basePosition=[px, py, 0.01],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=False,
            globalScaling=1.0  # keep URDF's natural small size
        )
        pebble_ids.append(bid)
        pebble_centers.append((px, py))

        # optional: give them some friction
        p.changeDynamics(bid, -1, lateralFriction=0.8)

    # --- Build flow field over world ---
    ff = FlowField2D(
        world_xmin=-env_radius,
        world_xmax=+env_radius,
        world_ymin=-env_radius,
        world_ymax=+env_radius,
        grid_w=41,
        grid_h=41,
        rover_radius=0.25,   # ~ rover footprint radius
        pebble_radius=pebble_radius,
        clearance=0.02,
    )

    ff.rebuild(goal_pos, pebble_centers)

    if DRAW_FLOWFIELD:
        ff.draw_debug(scale=0.15, life_time=0.0)

    # --- visualize goal as a small green sphere ---
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

    controller = FlowFieldController(
        v_max=1.5,
        w_max=10.0,
        k_theta=10.0,
        turn_in_place_angle_deg=50.0,
        static_speed_threshold=0.03,
        w_turn_in_place=25.0,
    )

    print("Flow-field navigation with small pebbles. Close the GUI window to stop.")
    acc = 0.0
    t0 = time.time()

    # Shovel offset from rover center (meters)
    SHOVEL_OFFSET = 0.17

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        if acc >= 0.05:  # 20 Hz control
            acc = 0.0
            agent["state"] = get_state_from_bullet(agent["body"])
            x, y, yaw, v_fwd, w = agent["state"]

            # Compute shovel tracking point
            x_s = x + SHOVEL_OFFSET * math.cos(yaw)
            y_s = y + SHOVEL_OFFSET * math.sin(yaw)

            # Create tracking state with shovel position
            tracking_state = np.array([x_s, y_s, yaw, v_fwd, w], dtype=float)

            # Compute control using shovel point
            v_cmd, w_cmd = controller.compute_control(tracking_state, agent["goal"], ff)
            agent["control"] = (v_cmd, w_cmd)

            if time.time() - t0 > 1.0:
                t0 = time.time()
                # Distance from shovel to goal
                dist_shovel = np.linalg.norm(agent["goal"] - np.array([x_s, y_s]))
                print(f"rover_pos=({x:.3f},{y:.3f})  shovel_pos=({x_s:.3f},{y_s:.3f})  "
                      f"shovel_dist_to_goal={dist_shovel:.3f}  v={v_cmd:.3f}  w={w_cmd:.3f}")

        apply_diff_drive_control(agent)


if __name__ == "__main__":
    main()
