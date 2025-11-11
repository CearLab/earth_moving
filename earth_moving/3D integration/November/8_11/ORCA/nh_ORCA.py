import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

from scenarios import get_scenario   # <- your scenarios.py

# =========================================================
#  ORCA-STYLE PLANNER (SAMPLING + TTC, PRIORITY / BLIND AWARE)
# =========================================================

class ORCAPlanner:
    def __init__(self,
                 tau=8.0,         # collision horizon (s) - long enough for head-on case
                 v_pref=0.8,      # preferred speed towards goal
                 v_max=1.0,       # max translational speed
                 w_max=3.0,       # max angular speed
                 k_theta=3.0,     # heading gain
                 turn_in_place_deg=45.0,
                 n_speed=6,       # radial samples in velocity space
                 n_angle=20):     # angular samples in velocity space
        self.tau = tau
        self.v_pref = v_pref
        self.v_max = v_max
        self.w_max = w_max
        self.k_theta = k_theta
        self.turn_in_place = math.radians(turn_in_place_deg)
        self.n_speed = n_speed
        self.n_angle = n_angle

    @staticmethod
    def _shape_radius(shape):
        """
        Approximate rover shape (two circles) as a single disc
        that bounds them both.
        """
        return max(math.hypot(cx, cy) + r for (cx, cy, r) in shape["circles"])

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _ttc_collision(p_ij, v_rel, r_sum, tau):
        """
        Check if there will be a collision (distance < r_sum) between now and tau,
        for relative position p_ij and relative velocity v_rel, where
        p_rel(t) = p_ij + v_rel * t.
        """
        r_eff = r_sum * 1.05  # small safety margin

        dist0 = np.linalg.norm(p_ij)
        # Already overlapping?
        if dist0 < r_eff:
            return True

        v2 = np.dot(v_rel, v_rel)
        if v2 < 1e-8:
            # Almost no relative motion and not overlapping => no collision
            return False

        t_star = -np.dot(p_ij, v_rel) / v2

        if t_star < 0.0:
            # Closest at t = 0
            d_min = dist0
        elif t_star > tau:
            # Closest at t = tau
            d_min = np.linalg.norm(p_ij + v_rel * tau)
        else:
            # Closest in [0, tau]
            d_min = np.linalg.norm(p_ij + v_rel * t_star)

        return d_min < r_eff

    def _project_to_unicycle(self, ego, v_new):
        """
        Map 2D velocity vector v_new back to (v, ω) commands
        for the differential-drive rover.
        """
        px, py, th, v_fwd, w = ego["state"]
        speed = np.linalg.norm(v_new)
        if speed < 1e-4:
            return 0.0, 0.0

        heading = math.atan2(v_new[1], v_new[0])
        e_th = self._wrap_angle(heading - th)

        # Turn in place if heading error is large
        if abs(e_th) > self.turn_in_place:
            v_cmd = 0.0
        else:
            v_cmd = min(speed, self.v_max)

        w_cmd = self.k_theta * e_th
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))
        return v_cmd, w_cmd

    def _sample_candidates(self, v_pref_vec):
        """
        Sample velocities in a disc of radius v_max,
        plus the preferred velocity explicitly and zero.
        """
        cand = [np.array([0.0, 0.0], dtype=float), v_pref_vec]
        speeds = np.linspace(0.0, self.v_max, self.n_speed)
        angles = np.linspace(-math.pi, math.pi, self.n_angle, endpoint=False)
        for s in speeds:
            for ang in angles:
                vx = s * math.cos(ang)
                vy = s * math.sin(ang)
                cand.append(np.array([vx, vy], dtype=float))
        return cand

    def plan(self, ego, goal, neighbors, shape):
        """
        ego:        agent dict with "state", "priority"
        goal:       np.array([gx, gy])
        neighbors:  list of dicts with "state", "shape", "priority", "is_blind"
        shape:      ego["shape"]
        """
        px, py, th, v_fwd, w = ego["state"]
        p_i = np.array([px, py], dtype=float)
        g = np.array(goal, dtype=float)
        dir_vec = g - p_i
        dist_goal = np.linalg.norm(dir_vec)

        if dist_goal > 1e-6:
            dir_unit = dir_vec / dist_goal
        else:
            dir_unit = np.zeros(2)

        # Preferred velocity towards goal (slow down near goal)
        s_pref = self.v_pref
        if dist_goal < 0.5:
            s_pref *= dist_goal / 0.5
        s_pref = min(self.v_max, max(0.0, s_pref))
        v_pref_vec = s_pref * dir_unit

        # If no neighbors at all: just go to preferred
        if not neighbors:
            v_cmd, w_cmd = self._project_to_unicycle(ego, v_pref_vec)
            return float(v_cmd), float(w_cmd)

        # Precompute neighbor info for TTC checks
        r_i = self._shape_radius(shape)
        pr_i = ego.get("priority", 0)

        neigh_info = []
        for nb in neighbors:
            pxj, pyj, thj, vj_fwd, wj = nb["state"]
            p_j = np.array([pxj, pyj], dtype=float)
            vx_j = math.cos(thj) * vj_fwd
            vy_j = math.sin(thj) * vj_fwd
            v_j = np.array([vx_j, vy_j], dtype=float)

            r_j = self._shape_radius(nb["shape"])
            r_sum = r_i + r_j

            pr_j = nb.get("priority", 0)
            is_blind = nb.get("is_blind", False)

            # Responsibility:
            #   - If neighbor is blind -> we MUST avoid it.
            #   - If neighbor has higher priority (smaller number) -> we avoid.
            #   - If neighbor has lower priority -> we assume it will avoid us.
            if is_blind or pr_j < pr_i:
                must_avoid = True
            elif pr_j > pr_i:
                must_avoid = False
            else:
                # equal priority -> both avoid (symmetric)
                must_avoid = True

            neigh_info.append({
                "p_ij": p_j - p_i,
                "v_j": v_j,
                "r_sum": r_sum,
                "must_avoid": must_avoid,
            })

        candidates = self._sample_candidates(v_pref_vec)

        best_v = v_pref_vec
        best_cost = float("inf")

        for v in candidates:
            if np.linalg.norm(v) > self.v_max + 1e-6:
                continue

            colliding = False
            for info in neigh_info:
                if not info["must_avoid"]:
                    continue
                p_ij = info["p_ij"]
                # *** FIXED SIGN: relative motion is v_j - v ***
                v_rel = info["v_j"] - v
                if self._ttc_collision(p_ij, v_rel, info["r_sum"], self.tau):
                    colliding = True
                    break

            if colliding:
                continue

            # Cost = distance to preferred velocity
            cost = np.linalg.norm(v - v_pref_vec)
            if cost < best_cost:
                best_cost = cost
                best_v = v

        # If no collision-free candidate was found, just try to stop
        if best_cost == float("inf"):
            best_v = np.zeros(2, dtype=float)

        v_cmd, w_cmd = self._project_to_unicycle(ego, best_v)
        return float(v_cmd), float(w_cmd)

# =========================================================
#                       PYBULLET HELPERS
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
    """
    Map (v, ω) to left/right wheel angular velocities and send VELOCITY_CONTROL.

    NOTE: sign flip so that v_cmd>0 moves the rover forward with this URDF.
    """
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
#                          MAIN
# =========================================================

# This is the key you pass to get_scenario().
# Make sure it matches your scenarios.py (e.g. "2_head_on").
SCENARIO_NAME = "4_crossing"  # or "4_crossing", etc.

# Only treat other agents as neighbors if they are within this radius (meters)
NEAR_RADIUS = 10.0

def create_blind_rovers(shape):
    """
    Example blind rovers (constant motion, ignore others).
    Reactive rovers will avoid them.
    """
    blind = [
        {
            "id": "B0",
            "priority": -1,               # higher "rank" than any R{i}
            "is_blind": True,
            "state": np.array([-3.0, -1.0, 0.0, 0.0, 0.0]),
            "control": (1.0, 0.0),        # constant forward
            "shape": shape,
            "color": (0.0, 0.0, 0.0, 1.0),
        },
        {
            "id": "B1",
            "priority": -1,
            "is_blind": True,
            "state": np.array([ 3.0,  1.0, math.pi, 0.0, 0.0]),
            "control": (2.7, 0.0),
            "shape": shape,
            "color": (0.0, 0.0, 0.0, 1.0),
        },
        {
            "id": "B2",
            "priority": -1,
            "is_blind": True,
            "state": np.array([3.5, -3.5, 3*math.pi/4, 0.0, 0.0]),
            "control": (3.5, 0.0),
            "shape": shape,
            "color": (0.0, 0.0, 0.0, 1.0),
        },
    ]
    return blind

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ROVER_URDF_PATH = os.path.join(here, "2_wheel_rover.urdf")

    # ---------- load scenario ----------
    scenario = get_scenario(SCENARIO_NAME)
    print(f"Scenario: {scenario['name']}")
    print(f"  {scenario.get('description', '')}")

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=12,
                                 cameraYaw=45,
                                 cameraPitch=-40,
                                 cameraTargetPosition=[0, 0, 0])

    # gravity + physics
    p.setGravity(0, 0, -9.81)
    sim_dt = 1.0 / 240.0
    p.setTimeStep(sim_dt)

    # plane
    plane_id = p.loadURDF("plane.urdf")

    # rover shape: two circles, slightly inflated
    ROVER_SHAPE = {
        "circles": [
            (+0.16, 0.0, 0.30),
            (-0.16, 0.0, 0.30),
        ]
    }

    # ---------- build reactive agents from scenario ----------
    reactive = []
    for i, cfg in enumerate(scenario["rovers"]):
        sx, sy, sz = cfg["start_pos"]
        th0 = cfg["start_heading"]
        gx, gy = cfg["goal_pos"]
        color = cfg.get("color", (0.7, 0.7, 0.7, 1.0))

        reactive.append({
            "id": f"R{i}",
            "priority": i,          # lower index = higher priority (if you want)
            "is_blind": True,
            "state": np.array([sx, sy, th0, 0.0, 0.0]),
            "goal":  np.array([gx, gy]),
            "control": (0.0, 0.0),
            "shape": ROVER_SHAPE,
            "color": color,
        })

    # ---------- blind rovers (dynamic obstacles) ----------
    # For now, disable blind rovers while we debug head-on behavior:
    blind = []
    # When you're ready to test them, uncomment:
    blind = create_blind_rovers(ROVER_SHAPE)

    # ---------- spawn rovers ----------

    def spawn_rover(agent):
        x, y, th, v, w = agent["state"]
        z = 0.02
        body_id = p.loadURDF(
            ROVER_URDF_PATH,
            basePosition=[x, y, z],
            baseOrientation=yaw_to_quat(th),
            useFixedBase=False,
        )
        left, right = find_wheel_joints(body_id)
        agent["body"] = body_id
        agent["left_joint"] = left
        agent["right_joint"] = right

        # set color on base
        r, g, b, a = agent.get("color", (0.7, 0.7, 0.7, 1.0))
        p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, a])

    for ag in reactive + blind:
        spawn_rover(ag)

    # friction
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    for ag in reactive + blind:
        p.changeDynamics(ag["body"], -1, lateralFriction=0.8)
        for link in (ag["left_joint"], ag["right_joint"]):
            p.changeDynamics(
                ag["body"], link,
                lateralFriction=1.0,
                rollingFriction=0.0,
                spinningFriction=0.0,
            )

    # zero initial motors
    for ag in reactive + blind:
        for j in (ag["left_joint"], ag["right_joint"]):
            p.setJointMotorControl2(
                ag["body"], j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0
            )

    # ---------- ORCA-style planners ----------
    planners = {}
    for ag in reactive:
        planners[ag["id"]] = ORCAPlanner(
            tau=8.0,         # must be >= time-to-collision for fast head-on
            v_pref=0.8,
            v_max=1.0,
            w_max=3.0,
            k_theta=3.0,
        )

    acc = 0.0
    t0 = time.time()
    print("Running ORCA-style TTC planner with priorities (blind rovers optional)... (close GUI to stop)")

    while p.isConnected():
        p.stepSimulation()
        time.sleep(sim_dt)
        acc += sim_dt

        # update & replan every 0.10 s
        if acc >= 0.10:
            acc = 0.0

            # update states from physics
            for ag in reactive + blind:
                ag["state"] = get_state_from_bullet(ag["body"])

            # plan for each reactive rover
            for ag in reactive:
                ego = ag
                goal = ag["goal"]

                neighbors = []

                # reactive neighbors (within NEAR_RADIUS)
                for other in reactive:
                    if other["id"] == ag["id"]:
                        continue
                    dx = other["state"][0] - ag["state"][0]
                    dy = other["state"][1] - ag["state"][1]
                    if dx * dx + dy * dy < NEAR_RADIUS ** 2:
                        neighbors.append({
                            "id": other["id"],
                            "priority": other["priority"],
                            "is_blind": other.get("is_blind", False),
                            "state": other["state"].copy(),
                            "shape": other["shape"],
                        })

                # blind neighbors (within NEAR_RADIUS)
                for b in blind:
                    dx = b["state"][0] - ag["state"][0]
                    dy = b["state"][1] - ag["state"][1]
                    if dx * dx + dy * dy < NEAR_RADIUS ** 2:
                        neighbors.append({
                            "id": b["id"],
                            "priority": b["priority"],
                            "is_blind": b.get("is_blind", True),
                            "state": b["state"].copy(),
                            "shape": b["shape"],
                        })

                v_cmd, w_cmd = planners[ag["id"]].plan(ego, goal, neighbors, ag["shape"])
                ag["control"] = (v_cmd, w_cmd)

            # debug print once per second
            if time.time() - t0 > 1.0:
                t0 = time.time()
                msg = " | ".join(
                    f"{ag['id']} v,w=({ag['control'][0]:.2f},{ag['control'][1]:.2f})"
                    for ag in reactive
                )
                print(msg)

        # low-level wheel control every physics step
        for ag in reactive + blind:
            apply_diff_drive_control(ag)

if __name__ == "__main__":
    main()
