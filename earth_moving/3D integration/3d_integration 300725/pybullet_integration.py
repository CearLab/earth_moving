import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time
import math
import warnings
import cv2
from coordinate_converter import CoordinateConverter

# Suppress warnings
warnings.filterwarnings("ignore", category=UserWarning, module="numpy")

# ===================== PATH UTILITIES =================

def prune_close(points_xy, min_dist=0.01):
    """Remove points closer than min_dist (m) keeping order. Normalizes to (x,y,theta)."""
    if not points_xy:
        return []
    out = []
    prev = None
    for pt in points_xy:
        x, y = pt[0], pt[1]
        if prev is None or math.hypot(x - prev[0], y - prev[1]) >= min_dist:
            out.append((x, y, pt[2] if len(pt) > 2 else 0.0))
            prev = (x, y)
    # Fill headings if zero
    pts = []
    for i in range(len(out)):
        if i < len(out) - 1:
            th = math.atan2(out[i+1][1] - out[i][1], out[i+1][0] - out[i][0])
        else:
            th = out[i][2]
        pts.append((out[i][0], out[i][1], th))
    return pts

def resample_polyline(points_xyz, ds=0.06):
    """
    Resample a polyline to ~uniform spacing ds (m).
    Input: list of (x,y) or (x,y,theta)
    Output: list of (x,y,theta) with headings from finite differences.
    """
    if len(points_xyz) < 2:
        return points_xyz[:]
    pts = [(p[0], p[1]) for p in points_xyz]
    # cumulative arclength
    L = [0.0]
    for i in range(1, len(pts)):
        L.append(L[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    total = L[-1]
    if total < 1e-6:
        return [(pts[0][0], pts[0][1], 0.0)]
    M = max(2, int(round(total / ds)) + 1)
    out = []
    idx = 0
    for k in range(M):
        s_tgt = k * total / (M - 1)
        while idx < len(L) - 1 and L[idx + 1] < s_tgt:
            idx += 1
        s0, s1 = L[idx], L[idx + 1] if idx + 1 < len(L) else L[idx]
        if s1 == s0:
            x, y = pts[idx]
        else:
            t = (s_tgt - s0) / (s1 - s0)
            x = pts[idx][0] + t * (pts[idx + 1][0] - pts[idx][0])
            y = pts[idx][1] + t * (pts[idx + 1][1] - pts[idx][1])
        out.append((x, y, 0.0))
    # headings via gradient
    xs = np.array([p[0] for p in out], float)
    ys = np.array([p[1] for p in out], float)
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    th = np.arctan2(dy, dx)
    return [(xs[i], ys[i], th[i]) for i in range(len(out))]

def shift_path_along_s(path_world, s_shift):
    """
    Shift a resampled path backward/forward along arclength by s_shift (m).
    Positive s_shift moves points toward the start (i.e., base behind TCP).
    """
    if len(path_world) < 2:
        return path_world[:]
    pts = np.array([(p[0], p[1]) for p in path_world], float)
    seg = pts[1:] - pts[:-1]
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    s_cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = s_cum[-1]

    def point_at_s_local(ss):
        ss = max(0.0, min(total, ss))
        i = int(np.searchsorted(s_cum, ss) - 1)
        i = max(0, min(i, len(pts) - 2))
        ds = ss - s_cum[i]
        if seg_len[i] < 1e-9:
            return pts[i]
        t = ds / seg_len[i]
        return pts[i] + t * (pts[i + 1] - pts[i])

    out = []
    for k in range(len(path_world)):
        s_target = s_cum[k] + s_shift  # note: + because we want base BEHIND tool (negative forward)
        x, y = point_at_s_local(s_target)
        out.append((x, y, 0.0))
    # headings
    xs = np.array([p[0] for p in out])
    ys = np.array([p[1] for p in out])
    th = np.arctan2(np.gradient(ys), np.gradient(xs))
    return [(xs[i], ys[i], th[i]) for i in range(len(out))]

def tool_pose_from_base(x, y, theta, d_fwd, d_lat):
    """Rigid offset in body frame (+X forward, +Y left)."""
    ct = math.cos(theta); st = math.sin(theta)
    x_tcp = x + d_fwd * ct - d_lat * st
    y_tcp = y + d_fwd * st + d_lat * ct
    return x_tcp, y_tcp, theta

def project_to_polyline(pts, s_cum, seg_len, xy):
    """Project xy onto a polyline pts (Nx2). Returns (s_proj, signed_cross_track)."""
    best_s = 0.0
    best_d2 = 1e18
    best_sign = 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            continue
        t = ((xy[0] - ax) * abx + (xy[1] - ay) * aby) / ab2
        t = max(0.0, min(1.0, t))
        qx, qy = ax + t * abx, ay + t * aby
        dx, dy = xy[0] - qx, xy[1] - qy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            # left-normal sign
            leftn_x, leftn_y = -aby, abx
            sign = math.copysign(1.0, leftn_x * dx + leftn_y * dy) if (dx or dy) else 0.0
            best_sign = sign
            best_s = s_cum[i] + t * seg_len[i]
    return best_s, best_sign * math.sqrt(best_d2)

def point_at_s(pts, s_cum, seg_len, s_target):
    """Interpolate a point at arclength s_target on polyline pts."""
    s_target = max(0.0, min(s_cum[-1], s_target))
    i = int(np.searchsorted(s_cum, s_target) - 1)
    i = max(0, min(i, len(pts) - 2))
    ds = s_target - s_cum[i]
    if seg_len[i] < 1e-9:
        return pts[i]
    t = ds / seg_len[i]
    return (pts[i][0] + t * (pts[i + 1][0] - pts[i][0]),
            pts[i][1] + t * (pts[i + 1][1] - pts[i][1]))

def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class PyBulletIntegration:
    def __init__(self, env_radius=1.0, target_zone_radius=0.3, num_pebbles=50, random_seed=42, gui=True, initial_robot_pose=(0, 0, 0)):
        # Initialize PyBullet
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1./240.)
        
        # Store object IDs
        self.object_ids = []
        self.target_zone_id = None
        self.grid_lines = []
        self.shovel_coverage = None
        
        # Store URDF path for pebbles
        self.pebbles_urdf = os.path.join("clutter", "urdf", "pebbles", "pebbles.urdf")
        
        # Environment parameters
        self.env_radius = env_radius  # meters
        self.target_zone_radius = target_zone_radius  # meters
        self.num_pebbles = num_pebbles
        self.random_seed = random_seed
        self.initial_robot_pose = initial_robot_pose  # Store initial pose
        
        # Rover parameters
        self.shovel_width = 0.22  # meters
        self.shovel_height = 0.08
        self.shovel_depth = 0.01
        
        # Rover control parameters
        self.control_threshold = 1e-2
        self.angle_threshold = 5
        self.iter_limit = 1e2
        self.Kp_vel = 0.0001
        self.Kphi_vel = 0.00001
        self.Kd_phi_vel = 0
        
        # Baseline control gains (matching engine_rover.py for better speed)
        self.K1_base = 2.0  # Linear velocity gain (matching engine_rover.py)
        self.K2_base = 2.0  # Angular velocity gain (matching engine_rover.py) 
        self.K3_base = 2.0  # Delta angle gain (matching engine_rover.py)
        self.vmax_base = 1.2  # Base max velocity (matching engine_rover.py)
        
        # Current adaptive gains (initialized to baseline)
        self.K1 = self.K1_base
        self.K2 = self.K2_base
        self.K3 = self.K3_base
        self.vmax = self.vmax_base
        self.phimax = 4.0  # Reduced from 3 for smoother turning
        self.control_dt = 1/240
        
        # Adaptive gain scheduling parameters
        self.enable_adaptive_gains = True
        self.curvature_window = 2  # Number of waypoints to look ahead for curvature calculation
        
        # Phase switching thresholds (tunable)
        self.approach_distance = 0.8  # Switch to tracking phase when closer than this
        self.precision_distance = 0.3  # Switch to precision phase when closer than this
        
        # Curvature thresholds (tunable)  
        self.high_curvature_threshold = 0.4  # High curvature turns
        self.medium_curvature_threshold = 0.2  # Medium curvature turns
        
        # Coordinate converter for grid/cell calculations
        self.coord_converter = CoordinateConverter(
            env_radius=self.env_radius,
            target_zone_radius=self.target_zone_radius,
            shovel_width=self.shovel_width
        )
        
        print("\nPyBullet initialized successfully")
        print(f"Environment radius: {self.env_radius:.3f} meters")
        print(f"Target zone radius: {self.target_zone_radius:.3f} meters")
        print(f"Number of pebbles: {self.num_pebbles}")
        print(f"Random seed: {self.random_seed}")
        print(f"Shovel width: {self.shovel_width:.3f} meters")
        print(f"Environment diameter: {2*self.env_radius:.3f} meters")

    def set_robot_dim(self, L=0.2, R=0.07):
        """Set robot dimensions."""
        self.L = L
        self.R = R

    def get_joint_index_by_name(self, joint_name):
        rover_id = self.object_ids[1]  # robot is the second object
        for i in range(p.getNumJoints(rover_id)):
            info = p.getJointInfo(rover_id, i)
            if info[1].decode("utf-8") == joint_name:
                return i
        raise ValueError(f"Joint '{joint_name}' not found")


    def get_robot_position(self):
        """
        Return (pos, quat) of the rover base – NOT the plane.
        """
        return p.getBasePositionAndOrientation(self.robot_id)

    def pose(self):
        """Alternative pose method for compatibility with advanced Pure Pursuit."""
        pos, quat = p.getBasePositionAndOrientation(self.robot_id)
        theta = p.getEulerFromQuaternion(quat)[2]
        return (pos[0], pos[1]), theta

    def get_velocities(self):
        """
        Forward linear velocity V [m/s] and yaw-rate φ [rad/s] measured
        in the rover’s body frame.
        """
        import numpy as np, pybullet as p

        lin_w, ang_w = p.getBaseVelocity(self.robot_id)
        _, quat = p.getBasePositionAndOrientation(self.robot_id)

        # world → body frame
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        lin_b = R.T @ lin_w            # body-frame linear velocity

        V = lin_b[0]                   # body-x component = forward speed
        phi = ang_w[2]                 # world-z is the same as body-z

        return V, phi

    def compute_wheel_velocities(self, V, phi):
        """Compute wheel velocities from linear and angular velocities."""
        V = np.clip(V, -self.vmax, self.vmax)
        phi = np.clip(phi, -self.phimax, self.phimax)
        
        omega_r = -(2 * V + phi * self.L) / (2 * self.R)
        omega_l = -(2 * V - phi * self.L) / (2 * self.R)
        
        return omega_r, omega_l


    def control_rover_velocity(self, left_wheel_vel, right_wheel_vel, time=1/240):
        rover_id = self.object_ids[1]  # assuming plane is [0], robot is [1]

        # Dynamically fetch correct joint indices
        left_joint = self.get_joint_index_by_name("base_to_lwheel")
        right_joint = self.get_joint_index_by_name("base_to_rwheel")

        # Apply lateral friction
        p.changeDynamics(rover_id, left_joint, lateralFriction=10.0)
        p.changeDynamics(rover_id, right_joint, lateralFriction=10.0)
        p.changeDynamics(self.object_ids[0], -1, lateralFriction=1.0)  # Plane

        # Apply motor control
        MAX_WHEEL_TORQUE = 150000  # N·m – good starting point
        p.setJointMotorControl2(rover_id, left_joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_wheel_vel, force=MAX_WHEEL_TORQUE)
        p.setJointMotorControl2(rover_id, right_joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_wheel_vel, force=MAX_WHEEL_TORQUE)

        self.simulate(time)

    def set_wheel_speeds_unitsafe(self, V, yaw_rate, dt=1/240):
        """
        Drive wheels with correct units no matter which PyBulletIntegration flavor:
        - old: compute_wheel_velocities(V, phi) + control_rover_velocity(omega_l, omega_r, dt)
        - new: _wheel_speeds(V, phi) + _drive(wl, wr, dt)
        """
        if hasattr(self, "compute_wheel_velocities"):
            omega_r, omega_l = self.compute_wheel_velocities(V, yaw_rate)
            if hasattr(self, "control_rover_velocity"):
                self.control_rover_velocity(omega_l, omega_r, dt)
            elif hasattr(self, "_drive"):
                self._drive(omega_l, omega_r, dt)
            else:
                raise RuntimeError("No wheel drive method found.")
        elif hasattr(self, "_wheel_speeds") and hasattr(self, "_drive"):
            wl, wr = self._wheel_speeds(V, yaw_rate)
            self._drive(wl, wr, dt)
        else:
            raise RuntimeError("Unsupported PyBulletIntegration API.")

    # ===================== PURE PURSUIT FUNCTIONALITY ======================
    # --------------------  zero-radius pivot  --------------------
    def _turn_in_place(self, target_theta: float, *, max_rate: float = 2.0, tol: float = 0.05):
        """
        Rotate the rover until its BASE heading matches *target_theta* (rad).
        Uses differential wheel speeds for pure rotation.
        """
        (x_start, y_start), th_start = self.pose()
        err = wrap_angle(target_theta - th_start)
        
        print(f"   Starting angle: {math.degrees(th_start):.1f}°")
        print(f"   Target angle: {math.degrees(target_theta):.1f}°") 
        print(f"   Error: {math.degrees(err):.1f}°")
        
        steps = 0
        max_steps = 1500  # Safety limit (6.25 seconds at 240 FPS)
        
        while abs(err) > tol and steps < max_steps:
            # For pure rotation: left and right wheels rotate in opposite directions
            # Positive error (need to turn CCW): left wheel backward, right wheel forward
            # Negative error (need to turn CW): left wheel forward, right wheel backward
            
            if err > 0:  # Turn counter-clockwise (CCW)
                left_vel = -max_rate
                right_vel = max_rate
            else:  # Turn clockwise (CW)  
                left_vel = max_rate
                right_vel = -max_rate
                
            # Apply smaller speed as we get closer to target
            speed_factor = min(1.0, abs(err) / math.radians(10))  # Slow down within 10 degrees
            left_vel *= speed_factor
            right_vel *= speed_factor
            
            self.control_rover_velocity(left_vel, right_vel)
            
            # Step simulation and update error
            p.stepSimulation()
            _, th = self.pose()
            err = wrap_angle(target_theta - th)
            steps += 1
        
        # Hard-stop wheels and ensure complete stop
        for _ in range(60):  # Apply brake for 60 steps (0.25 seconds)
            self.control_rover_velocity(0.0, 0.0)
            p.stepSimulation()
        
        # Final position check
        (x_end, y_end), th_end = self.pose()
        distance_moved = math.hypot(x_end - x_start, y_end - y_start)
        final_error = abs(wrap_angle(target_theta - th_end))
        
        print(f"   Final angle: {math.degrees(th_end):.1f}°")
        print(f"   Final error: {math.degrees(final_error):.2f}°")
        print(f"   Distance moved: {distance_moved:.4f}m")
        print(f"   Steps taken: {steps}")
        
        if steps >= max_steps:
            print(f"   ⚠️ Warning: Hit step limit, may not have reached target!")
        if distance_moved > 0.1:
            print(f"   ⚠️ Warning: Rover moved too much during turn!")
        if final_error > math.radians(5):
            print(f"   ⚠️ Warning: Turn accuracy poor!")

    # ------------------------------------------------------------------
    #  Path builder with automatic turn-in-place
    # ------------------------------------------------------------------
    def build_world_path(
            self,
            env,
            approach_duration: float = 2.0,
            approach_scale: float = 0.3,
            ds: float = 0.05,
            *,
            pivot_thresh_deg: float = 35.0,  # spin if heading error > 35 °
            pivot_rate: float = 2.5  # rad s⁻¹ (≤ self.PHI_MAX)
        ):
        """
        Returns  (approach, task_world, path_resampled)

        0.  If the base heading differs from the first task-segment
            heading by more than *pivot_thresh_deg*, spin in place first.

        1.  Bézier approach from current pose → first grid cell.

        2.  Grid trajectory → world coordinates.

        3.  Splice, prune duplicate splice point, resample to ~ds spacing.
        """
        # --- current base pose ------------------------------------------------
        # Use same robot reference as original working code
        robot_id = self.object_ids[1]  # plane[0], robot[1] in setup
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        th_now = p.getEulerFromQuaternion(quat)[2]

        # --- desired entry heading (first task segment) -----------------------
        start_grid = env.current_trajectory[0]
        next_grid = env.current_trajectory[1] if len(env.current_trajectory) > 1 else start_grid
        sx, sy = self.coord_converter.convert_2d_to_3d(*start_grid)
        nx, ny = self.coord_converter.convert_2d_to_3d(*next_grid)
        th_task = math.atan2(ny - sy, nx - sx)

        # --- 0.  Optional pivot-in-place (DISABLED FOR DEBUGGING) -------------
        hdg_err = wrap_angle(th_task - th_now)
        if False:  # DISABLED - testing original behavior first
            print(f"⟳  Pivoting {math.degrees(hdg_err):.1f}° before approach …")
            self._turn_in_place(th_task, max_rate=pivot_rate)
            # refresh pose after pivot
            pos, quat = p.getBasePositionAndOrientation(robot_id)
            th_now = p.getEulerFromQuaternion(quat)[2]

        current_pose = (pos[0], pos[1], th_now)
        start_pose = (sx, sy, th_task)

        # --- 1.  Bézier approach ---------------------------------------------
        approach = self.generate_bezier_trajectory(
            current_pose, start_pose,
            duration=approach_duration,
            scale=approach_scale
        )

        # --- 2.  Task path (grid → world) ------------------------------------
        task_world = []
        for gx, gy in env.current_trajectory:
            wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
            task_world.append((wx, wy, 0.0))

        # --- 3.  Splice → prune dupes → resample -----------------------------
        raw = prune_close(approach + task_world, min_dist=0.003)
        path = resample_polyline(raw, ds=ds)

        return approach, task_world, path

    def follow_trajectory_pure_pursuit(
        self,
        path_world,             # list[(x,y,theta)] already resampled
        *,
        dt=1/240,
        lookahead=0.10,
        v_nom=0.40,
        a_lat_max=1.2,
        yaw_slew=8.0,
        use_tcp_pose=True,      # True: track with TOOL (A). False: base (for base-shift mode).
        tcp_fwd=0.12,          # TCP forward offset
        tcp_lat=0.00,          # TCP lateral offset
        draw_tool_tick=True    # Draw tool position marker
    ):
        print(f"🚗 Pure Pursuit: {len(path_world)} samples (lookahead={lookahead:.2f} m, mode={'TCP' if use_tcp_pose else 'BASE'})")
        if len(path_world) < 2:
            return

        # Polyline arrays
        pts = np.array([(p[0], p[1]) for p in path_world], float)
        seg = pts[1:] - pts[:-1]
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        s_cum = np.concatenate([[0.0], np.cumsum(seg_len)])
        total_len = s_cum[-1]

        phi_prev = 0.0
        max_dphi = yaw_slew * dt

        t_elapsed = 0.0
        max_time = 180.0  # safety

        while t_elapsed < max_time:
            # Base pose
            if hasattr(self, "pose"):
                (x_b, y_b), th = self.pose()
            else:
                pos, quat = p.getBasePositionAndOrientation(self.object_ids[1])
                x_b, y_b = pos[0], pos[1]
                th = p.getEulerFromQuaternion(quat)[2]

            # Use TOOL or BASE for projection and body-frame errors
            if use_tcp_pose:
                x_o, y_o, _ = tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat)
                if draw_tool_tick:
                    p.addUserDebugLine([x_o, y_o, 0.01], [x_o, y_o, 0.05], [0, 1, 0], 2, lifeTime=0.2)
            else:
                x_o, y_o = x_b, y_b

            # stop near end (using the chosen origin point)
            s_here, _ = project_to_polyline(pts, s_cum, seg_len, (x_o, y_o))
            if total_len - s_here < 0.05:
                break

            # lookahead along arclength
            s_tgt = s_here + lookahead
            tx, ty = point_at_s(pts, s_cum, seg_len, s_tgt)

            # target in body frame (rotate with base heading)
            dx, dy = tx - x_o, ty - y_o
            ex = math.cos(th) * dx + math.sin(th) * dy
            ey = -math.sin(th) * dx + math.cos(th) * dy
            Ld = max(lookahead, 1e-6)

            # pure-pursuit curvature & speed scheduling
            kappa = 2.0 * ey / (Ld * Ld)
            if abs(kappa) > 1e-6:
                v_curve = math.sqrt(max(0.01, a_lat_max / abs(kappa)))
                V = min(v_nom, v_curve)
            else:
                V = v_nom
            phi_cmd = V * kappa

            # slew-limit yaw-rate to kill pulses
            dphi = phi_cmd - phi_prev
            dphi = max(-max_dphi, min(max_dphi, dphi))
            phi = phi_prev + dphi
            phi_prev = phi

            # drive wheels (units-correct & API-agnostic)
            self.set_wheel_speeds_unitsafe(V, phi, dt)
            t_elapsed += dt
        
        # Stop the robot smoothly after trajectory completion
        print("🏁 Pure Pursuit trajectory completed - stopping robot...")
        self._stop_robot_smoothly()

    def execute_pure_pursuit_trajectory(self, env, mode="TCP", **kwargs):
        """
        High-level interface for Pure Pursuit trajectory execution.
        
        Args:
            env: Environment object with current_trajectory
            mode: "TCP" or "BASE_SHIFT" 
            **kwargs: Override default Pure Pursuit parameters
        """
        # Default parameters
        config = {
            'tcp_fwd': 0.12,
            'tcp_lat': 0.00, 
            'resample_ds': 0.05,
            'v_nom': 0.40,
            'a_lat_max': 1.2,
            'yaw_slew_rate': 8.0,
            'lookahead': 0.10,
            'draw_tool_tick': True,
            'approach_duration': 2.0,
            'approach_scale': 0.3
        }
        config.update(kwargs)

        print(f"\nExecuting trajectory with PURE PURSUIT controller (mode: {mode})...")

        # Build world path (approach + task) and resample
        approach, task_world, path_world = self.build_world_path(
            env, 
            approach_duration=config['approach_duration'], 
            approach_scale=config['approach_scale'], 
            ds=config['resample_ds']
        )
        print(f"Approach: {len(approach)} pts, Task: {len(task_world)} pts, Path: {len(path_world)} pts")

        # Handle different modes
        if mode.upper() == "BASE_SHIFT":
            base_path = shift_path_along_s(path_world, s_shift=-config['tcp_fwd'])  # negative to go "back"
            path_for_controller = base_path
            use_tcp_pose = False  # base tracks shifted path
            # draw shifted path (purple)
            for i in range(len(base_path) - 1):
                p.addUserDebugLine([base_path[i][0], base_path[i][1], 0.035],
                                   [base_path[i+1][0], base_path[i+1][1], 0.035],
                                   [0.6, 0.0, 0.6], lineWidth=2.0)
        else:
            # TCP mode: controller uses the tool pose and the original resampled path
            path_for_controller = path_world
            use_tcp_pose = True

        # Visualization
        self._draw_path_visualization(approach, task_world, path_world)

        # Follow using Pure Pursuit (arc-length lookahead)
        self.follow_trajectory_pure_pursuit(
            path_for_controller,
            dt=1/240,
            lookahead=config['lookahead'],
            v_nom=config['v_nom'],
            a_lat_max=config['a_lat_max'],
            yaw_slew=config['yaw_slew_rate'],
            use_tcp_pose=use_tcp_pose,
            tcp_fwd=config['tcp_fwd'],
            tcp_lat=config['tcp_lat'],
            draw_tool_tick=config['draw_tool_tick']
        )

        # Post-trajectory: ensure environment stability and update 2D
        return self._post_trajectory_update(env)

    def _draw_path_visualization(self, approach, task_world, path_world):
        """Draw visualization for different path components."""
        # Draw approach (green) and task (blue)
        for i in range(len(approach) - 1):
            p.addUserDebugLine([approach[i][0], approach[i][1], 0.02],
                               [approach[i+1][0], approach[i+1][1], 0.02],
                               [0, 1, 0], lineWidth=3.0)
        for i in range(len(task_world) - 1):
            p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                               [task_world[i+1][0], task_world[i+1][1], 0.02],
                               [0, 0, 1], lineWidth=2.0)

        # Draw resampled path (black) to verify spacing
        for i in range(len(path_world) - 1):
            p.addUserDebugLine([path_world[i][0], path_world[i][1], 0.03],
                               [path_world[i+1][0], path_world[i+1][1], 0.03],
                               [0, 0, 0], lineWidth=2.0)

    def _stop_robot_smoothly(self, settle_time=0.8):
        """Stop the robot smoothly and let physics settle completely."""
        print("  🛑 Applying brakes and letting physics settle...")
        
        # Apply gentle braking
        print(f"    🔄 Braking for {settle_time}s...")
        for _ in range(int(settle_time * 240)):  # settle_time seconds at 240Hz
            self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)
        
        # Let physics settle completely - longer time for spillage to finish
        settle_extra = 1.0  # Full second for objects to settle after spilling
        print(f"    🔄 Letting physics settle for {settle_extra}s (spillage completion)...")
        for step in range(int(settle_extra * 240)):
            p.stepSimulation()
            time.sleep(1/240)
            
            # Check if objects are still moving significantly every 0.1s
            if step % 24 == 0:  # Every 0.1s
                total_velocity = 0
                moving_objects = 0
                for obj_id in self.object_ids[2:]:  # Skip plane and robot
                    try:
                        lin_vel, ang_vel = p.getBaseVelocity(obj_id)
                        speed = math.sqrt(lin_vel[0]**2 + lin_vel[1]**2 + lin_vel[2]**2)
                        total_velocity += speed
                        if speed > 0.01:  # Moving faster than 1cm/s
                            moving_objects += 1
                    except:
                        pass
                
                if step % 120 == 0:  # Every 0.5s
                    avg_velocity = total_velocity / len(self.object_ids[2:]) if len(self.object_ids) > 2 else 0
                    print(f"      📊 Physics status: {moving_objects} objects still moving, avg velocity: {avg_velocity:.4f} m/s")
        
        # Final check - if objects are still moving a lot, wait more
        final_moving = 0
        for obj_id in self.object_ids[2:]:
            try:
                lin_vel, ang_vel = p.getBaseVelocity(obj_id)
                speed = math.sqrt(lin_vel[0]**2 + lin_vel[1]**2 + lin_vel[2]**2)
                if speed > 0.005:  # Still moving faster than 0.5cm/s
                    final_moving += 1
            except:
                pass
                
        if final_moving > 0:
            print(f"    ⏳ {final_moving} objects still moving, waiting additional 0.5s...")
            for _ in range(int(0.5 * 240)):
                p.stepSimulation()
                time.sleep(1/240)
        
        print("  ✅ Robot stopped and physics completely settled")

    def _post_trajectory_update(self, env):
        """
        Handle post-trajectory updates:
        1. Ensure environment stability
        2. Capture current 3D object positions 
        3. Transfer to 2D environment
        4. Update 2D heatmap
        """
        print("\n📊 Post-trajectory update: transferring 3D state to 2D...")
        
        # 1. Get current 3D object positions (after physics settling)
        current_3d_positions = self.get_current_state()
        print(f"  📍 Captured {len(current_3d_positions)} 3D object positions")
        
        # Debug: Show ALL positions and check for realistic spread
        print(f"    📊 Complete 3D Position Analysis:")
        pebble_positions = []
        for i, pos in enumerate(current_3d_positions):
            if i == 0:
                obj_type = "plane"
            elif i == 1:
                obj_type = "robot"
            else:
                obj_type = f"pebble{i-2}"
                pebble_positions.append(pos)
            
            print(f"      {obj_type}: 3D({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
        
        # Analyze pebble spread to verify spillage was captured
        if pebble_positions:
            x_coords = [pos[0] for pos in pebble_positions]
            y_coords = [pos[1] for pos in pebble_positions]
            x_spread = max(x_coords) - min(x_coords)
            y_spread = max(y_coords) - min(y_coords)
            print(f"    📐 Pebble spread analysis:")
            print(f"      X range: {min(x_coords):.3f} to {max(x_coords):.3f} (spread: {x_spread:.3f}m)")
            print(f"      Y range: {min(y_coords):.3f} to {max(y_coords):.3f} (spread: {y_spread:.3f}m)")
            
            # Check if objects are actually spread out (indicating spillage occurred)
            if x_spread < 0.1 and y_spread < 0.1:
                print(f"    ⚠️  WARNING: Very small spread detected - objects may not have moved much!")
            else:
                print(f"    ✅ Good spread detected - spillage/movement captured")
        
        # 2. Filter pebble positions (skip plane and robot)
        pebble_positions_3d = current_3d_positions[2:]  # Skip plane[0] and robot[1]
        print(f"  🪨 Found {len(pebble_positions_3d)} pebbles to transfer")
        
        # 3. Convert 3D positions to 2D grid coordinates
        updated_objects_2d = []
        conversion_errors = 0
        grid_positions_summary = {}
        
        print(f"  🔄 Converting {len(pebble_positions_3d)} pebble positions to 2D grid:")
        
        for i, (x, y, z) in enumerate(pebble_positions_3d):
            try:
                # Convert world coordinates to 2D grid
                grid_x, grid_y = self.coord_converter.convert_3d_to_2d(x, y)
                updated_objects_2d.append((grid_x, grid_y))
                
                # Track grid position distribution
                grid_key = (grid_x, grid_y)
                grid_positions_summary[grid_key] = grid_positions_summary.get(grid_key, 0) + 1
                
                # Show more detailed conversion info
                if i < 10:  # Show first 10 conversions for debugging
                    print(f"    🔄 Pebble {i:2d}: 3D({x:7.4f}, {y:7.4f}) → 2D({grid_x:2d}, {grid_y:2d})")
                elif i == 10:
                    print(f"    ... (showing first 10, continuing conversion for {len(pebble_positions_3d)-10} more)")
                    
            except Exception as e:
                conversion_errors += 1
                if conversion_errors <= 5:  # Show first 5 errors 
                    print(f"    ⚠️ Warning: Could not convert pebble {i} at ({x:.4f}, {y:.4f}): {e}")
                elif conversion_errors == 6:
                    print(f"    ⚠️ ... (suppressing further conversion warnings)")
        
        # Show summary of where objects ended up
        print(f"    📊 2D Grid Distribution Summary:")
        print(f"      Total successful conversions: {len(updated_objects_2d)}")
        print(f"      Unique grid cells occupied: {len(grid_positions_summary)}")
        if conversion_errors > 0:
            print(f"      Conversion errors: {conversion_errors}/{len(pebble_positions_3d)}")
            
        # Show grid cells with most objects (spillage hotspots)
        if grid_positions_summary:
            sorted_cells = sorted(grid_positions_summary.items(), key=lambda x: x[1], reverse=True)
            print(f"      Top occupied cells:")
            for i, ((gx, gy), count) in enumerate(sorted_cells[:5]):
                print(f"        Cell ({gx:2d}, {gy:2d}): {count} objects")
                if i == 4 and len(sorted_cells) > 5:
                    print(f"        ... and {len(sorted_cells)-5} more cells")
        
        print(f"  🗂️ Successfully converted {len(updated_objects_2d)} objects to 2D coordinates")
        
        # 4. The KEY INSIGHT: Don't try to update existing 2D env, recreate it entirely!
        print("  🔄 The existing 2D environment cannot be properly updated.")
        print("      Instead, we need to signal for a complete reconstruction.")
        print("      This should be done by recreating the 2D environment with new positions.")
        
        # Store the new positions for the orchestrator to use
        self._new_object_positions_3d = pebble_positions_3d
        print(f"  📦 Stored {len(pebble_positions_3d)} new 3D positions for 2D environment recreation")
        
        print("✅ Post-trajectory update complete!\n")
        
        return {
            'total_objects': len(current_3d_positions),
            'pebbles_transferred': len(updated_objects_2d),
            '3d_positions': current_3d_positions,
            '2d_positions': updated_objects_2d,
            'needs_2d_recreation': True,
            'new_object_positions_3d': pebble_positions_3d
        }

    def get_updated_object_positions_for_2d_recreation(self):
        """Get the updated 3D positions for recreating the 2D environment."""
        if hasattr(self, '_new_object_positions_3d'):
            return self._new_object_positions_3d
        else:
            # Fallback to current state
            return self.get_current_state()[2:]  # Skip plane and robot

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def calculate_path_curvature(self, trajectory, current_index, window=3):
        """
        Calculate path curvature at current position by looking ahead.
        Returns curvature value (higher = more curved path).
        """
        if current_index + window >= len(trajectory):
            return 0.0
        
        # Get three points: current, mid, and future
        p1 = np.array(trajectory[current_index][:2])
        p2 = np.array(trajectory[min(current_index + window//2, len(trajectory)-1)][:2])
        p3 = np.array(trajectory[min(current_index + window, len(trajectory)-1)][:2])
        
        # Calculate vectors
        v1 = p2 - p1
        v2 = p3 - p2
        
        # Calculate angle between vectors
        if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
            return 0.0
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # Curvature is inversely related to the angle (smaller angle = higher curvature)
        curvature = (np.pi - angle) / np.pi
        return curvature
    
    def adaptive_gains(self, current_pos, trajectory, current_index):
        """
        Adaptive gain scheduling based on trajectory characteristics.
        
        Parameters:
        - current_pos: Current robot position (x, y, z)
        - trajectory: List of trajectory waypoints
        - current_index: Current waypoint index
        
        Returns:
        - Tuple of (K1, K2, K3, vmax) adapted gains
        """
        if not self.enable_adaptive_gains:
            return self.K1_base, self.K2_base, self.K3_base, self.vmax_base
        
        # Calculate distance to current target
        if current_index >= len(trajectory):
            return self.K1_base, self.K2_base, self.K3_base, self.vmax_base
            
        target_pos = trajectory[current_index]
        dist_to_target = np.linalg.norm(np.array(current_pos[:2]) - np.array(target_pos[:2]))
        
        # Calculate path curvature
        curvature = self.calculate_path_curvature(trajectory, current_index, self.curvature_window)
        
        # Calculate distance to end of trajectory
        final_pos = trajectory[-1]
        dist_to_end = np.linalg.norm(np.array(current_pos[:2]) - np.array(final_pos[:2]))
        
        # Phase-based adaptation using configurable thresholds
        if dist_to_end > self.approach_distance:  # Approach phase (far from end)
            K1 = self.K1_base * 0.8  # Smoother forward motion
            K2 = self.K2_base * 0.6  # Less aggressive lateral correction  
            K3 = self.K3_base * 0.6  # Less aggressive angular correction
            vmax = self.vmax_base * 0.8  # Moderate speed
            phase_name = "approach"
            
        elif dist_to_end > self.precision_distance:  # Tracking phase (middle distance)
            K1 = self.K1_base * 0.7  # Smoother forward motion
            K2 = self.K2_base * 0.8  # Moderate lateral correction
            K3 = self.K3_base * 0.8  # Moderate angular correction 
            vmax = self.vmax_base * 0.6  # Reduced speed
            phase_name = "tracking"
            
        else:  # Precision phase (close to end)
            K1 = self.K1_base * 0.4  # Very careful forward motion
            K2 = self.K2_base * 1.0  # Moderate precision
            K3 = self.K3_base * 1.2  # Moderate precision
            vmax = self.vmax_base * 0.3  # Very slow
            phase_name = "precision"
        
        # Curvature-based adaptation using configurable thresholds
        if curvature > self.high_curvature_threshold:  # High curvature (sharp turn)
            K1 *= 0.8  # Gently reduce forward speed
            K2 *= 1.1  # Gentle lateral correction increase
            K3 *= 1.2  # Gentle angular correction increase
            vmax *= 0.7  # Moderate speed reduction
            curve_name = "high"
            
        elif curvature > self.medium_curvature_threshold:  # Medium curvature
            K1 *= 0.9   # Very gentle forward speed reduction
            K2 *= 1.05  # Very gentle lateral correction increase
            K3 *= 1.1   # Very gentle angular correction increase
            vmax *= 0.85  # Very gentle speed reduction
            curve_name = "medium"
        else:
            curve_name = "low"
        
        # Distance-based fine-tuning
        if dist_to_target < 0.1:  # Very close to target
            K1 *= 0.5  # Very careful forward motion
            K2 *= 1.5  # Increase precision
            K3 *= 1.5  # Increase precision
            vmax *= 0.4  # Very slow
        
        # Ensure gains stay within reasonable bounds
        K1 = np.clip(K1, 0.2, 2.0)  # Tighter bounds to prevent oscillation
        K2 = np.clip(K2, 0.5, 3.0)  # Tighter bounds to prevent oscillation  
        K3 = np.clip(K3, 0.8, 4.0)  # Tighter bounds to prevent oscillation
        vmax = np.clip(vmax, 0.1, 0.8)
        
        # Debug output every 10 iterations to monitor gain scheduling  
        if current_index % 10 == 0:
            print(f"  Gains[{phase_name}|{curve_name}]: dist_end={dist_to_end:.2f}m, K1={K1:.2f}, K2={K2:.2f}, K3={K3:.2f}, vmax={vmax:.2f}, curve={curvature:.3f}")
            
        # Log phase transitions
        if not hasattr(self, '_last_phase') or self._last_phase != phase_name:
            print(f"  🔄 Phase transition: {getattr(self, '_last_phase', 'start')} → {phase_name} (dist_to_end={dist_to_end:.2f}m)")
            self._last_phase = phase_name
        
        return K1, K2, K3, vmax

    def set_gain_thresholds(self, approach_dist=None, precision_dist=None, high_curve=None, med_curve=None):
        """
        Adjust gain scheduling thresholds for tuning.
        
        Parameters:
        - approach_dist: Distance threshold for approach→tracking phase switch (default: 0.8m)
        - precision_dist: Distance threshold for tracking→precision phase switch (default: 0.3m)  
        - high_curve: Curvature threshold for high curvature detection (default: 0.4)
        - med_curve: Curvature threshold for medium curvature detection (default: 0.2)
        """
        if approach_dist is not None:
            self.approach_distance = approach_dist
        if precision_dist is not None:
            self.precision_distance = precision_dist
        if high_curve is not None:
            self.high_curvature_threshold = high_curve
        if med_curve is not None:
            self.medium_curvature_threshold = med_curve
            
        print(f"Gain thresholds updated: approach={self.approach_distance}m, precision={self.precision_distance}m, high_curve={self.high_curvature_threshold}, med_curve={self.medium_curvature_threshold}")

    def compute_target_velocities(self, target_pos, target_quat, current_pos, current_quat):
        """Compute target velocities for position control."""
        # Compute position error
        delta_pos = np.array(current_pos) - np.array(target_pos)
        ex = delta_pos[0]
        ey = delta_pos[1]
        
        # Compute orientation error
        current_theta = p.getEulerFromQuaternion(current_quat)[2]
        target_theta = p.getEulerFromQuaternion(target_quat)[2]
        etheta = self.normalize_angle(current_theta - target_theta)
        
        # Error vector
        e = np.array([ex, ey, etheta])
        
        # R matrix
        R = np.array([[+np.cos(target_theta), +np.sin(target_theta), 0],
                     [-np.sin(target_theta), +np.cos(target_theta), 0],
                     [0, 0, 1]])
        e_r = np.dot(R, e)
        
        # Change of coordinates
        rho = np.sqrt(e_r[0]**2 + e_r[1]**2)
        gamma = self.normalize_angle(np.arctan2(e_r[1], e_r[0]) - e_r[2] + np.pi)
        delta = self.normalize_angle(gamma + e_r[2])
        
        # Compute target velocities with epsilon to prevent division by zero
        epsilon = 1e-6
        sigma = 1 if abs(rho) > 1e-4 else 0
        
        # Linear velocity control
        target_V = self.K1 * rho * np.cos(gamma) * sigma
        
        # Angular velocity control with improved stability
        if abs(gamma) < epsilon:
            # When gamma is very small, use a simpler control law
            target_phi = self.K2 * gamma + self.K3 * delta
        else:
            # Normal control law with epsilon to prevent division by zero
            target_phi = ((self.K1 * np.sin(gamma) * np.cos(gamma)) / (gamma + epsilon)) * (gamma + self.K3 * delta) + self.K2 * gamma
        
        return target_V, target_phi, e_r

    def follow_trajectory(self, trajectory, step_time=0.1):
        """Follow a trajectory using position control."""
        # Get current robot position and orientation
        current_pos, current_quat = self.get_robot_position()
        current_theta = p.getEulerFromQuaternion(current_quat)[2]
        
        # Get the first point of the trajectory
        start = trajectory[0]
        start_world = (
            (start[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
            (start[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
            0.1  # Slightly above ground
        )

        # Calculate target orientation based on first movement
        if len(trajectory) > 1:
            next_point = trajectory[1]
            next_world = (
                (next_point[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (next_point[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1
            )
            dx = next_world[0] - start_world[0]
            dy = next_world[1] - start_world[1]
            target_theta = math.atan2(dy, dx)
        else:
            target_theta = current_theta

        # Calculate approach trajectory
        # First, calculate the distance and angle to the target
        dx = start_world[0] - current_pos[0]
        dy = start_world[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)
        
        # Create approach waypoints
        approach_trajectory = []
        
        # If we need to rotate first
        angle_diff = self.normalize_angle(angle_to_target - current_theta)
        if abs(angle_diff) > 0.1:  # If angle difference is significant
            # Add rotation waypoint
            approach_trajectory.append((
                current_pos[0],
                current_pos[1],
                angle_to_target
            ))
        
        # Add intermediate point if distance is large
        if distance > 0.5:  # If distance is significant
            mid_x = current_pos[0] + 0.5 * dx
            mid_y = current_pos[1] + 0.5 * dy
            approach_trajectory.append((
                mid_x,
                mid_y,
                angle_to_target
            ))
        
        # Add final approach point
        approach_trajectory.append((
            start_world[0],
            start_world[1],
            target_theta
        ))

        # Execute approach trajectory
        print("\nExecuting approach trajectory...")
        for i in range(len(approach_trajectory) - 1):
            start = approach_trajectory[i]
            end = approach_trajectory[i + 1]
            
            # Calculate desired orientation
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            target_theta = math.atan2(dy, dx)
            target_quat = p.getQuaternionFromEuler([0, 0, target_theta])
            
            # Set position and wait for completion
            self.set_positions((end[0], end[1], 0.1), target_quat)
            t.sleep(step_time)
        
        # Follow the main trajectory
        print("Following main trajectory...")
        for i in range(len(trajectory) - 1):
            start = trajectory[i]
            end = trajectory[i + 1]
            
            # Convert grid coordinates to world coordinates
            start_world = (
                (start[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (start[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1  # Slightly above ground
            )
            end_world = (
                (end[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (end[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1  # Slightly above ground
            )
            
            # Calculate desired orientation
            dx = end_world[0] - start_world[0]
            dy = end_world[1] - start_world[1]
            target_theta = math.atan2(dy, dx)
            target_quat = p.getQuaternionFromEuler([0, 0, target_theta])
            
            # Set position and wait for completion
            self.set_positions(end_world, target_quat)
            t.sleep(step_time)

    def set_positions(self, target_pos, target_quat):
        """Set robot position and orientation using position control."""
        reached = False
        control_iter = 0
        error_vector = []
        max_iterations = 1000  # Add a maximum iteration limit
        
        while not reached and control_iter < max_iterations:
            # Get current position and orientation
            current_pos, current_quat = self.get_robot_position()
            
            # Compute target velocities
            target_V, target_phi, e_r = self.compute_target_velocities(target_pos, target_quat, current_pos, current_quat)
            
            # Compute wheel velocities
            omega_r, omega_l = self.compute_wheel_velocities(target_V, target_phi)
            
            # Set velocities
            self.control_rover_velocity(omega_l, omega_r, self.control_dt)
            
            # Check if target reached
            control_iter += 1
            if np.linalg.norm(e_r[:-1]) < self.control_threshold and abs(np.rad2deg(e_r[-1])) < self.angle_threshold:
                reached = True
                break
            
            e_r[-1] = np.rad2deg(e_r[-1])
            error_vector.append(e_r)
            
            # Add a small delay to prevent too rapid control updates
            time.sleep(0.01)
        
        return np.array(error_vector).reshape(-1, 3), control_iter

    def get_current_state(self):
        """Get current 3D object positions with detailed debugging."""
        objects_3d = []
        print(f"🔍 DEBUG: Getting positions for {len(self.object_ids)} objects...")
        
        for i, obj_id in enumerate(self.object_ids):
            try:
                pos, quat = p.getBasePositionAndOrientation(obj_id)
                objects_3d.append((pos[0], pos[1], pos[2]))
                
                # Debug output for each object type
                if i == 0:
                    obj_type = "plane"
                elif i == 1:
                    obj_type = "robot"
                else:
                    obj_type = f"pebble{i-2}"
                
                print(f"  📍 {obj_type} (ID:{obj_id}): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                
            except Exception as e:
                print(f"⚠️ Failed to get position for object ID {obj_id}: {str(e)}")
        
        print(f"✅ Total objects captured: {len(objects_3d)}")
        return objects_3d
        
    def create_grid_overlay(self):
        """Create visual grid overlay in PyBullet using grid size/cell size from CoordinateConverter."""
        for line_id in self.grid_lines:
            p.removeBody(line_id)
        self.grid_lines = []

        grid_size = self.coord_converter.grid_size
        cell_size = self.coord_converter.cell_size
        env_radius = self.env_radius
        env_diameter = 2 * env_radius

        for i in range(grid_size + 1):
            pos = -env_radius + (i * env_diameter / grid_size)

            # Horizontal line
            line_id = p.addUserDebugLine(
                [-env_radius, pos, 0.01],
                [env_radius, pos, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

            # Vertical line
            line_id = p.addUserDebugLine(
                [pos, -env_radius, 0.01],
                [pos, env_radius, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

            
    def create_target_zone(self):
        """Create target zone in PyBullet."""
        target_zone_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=self.target_zone_radius,
            length=0.01,
            rgbaColor=[1, 0, 0, 0.3]
        )
        
        self.target_zone_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=target_zone_visual,
            basePosition=[0, 0, 0.005]
        )
        
    def generate_random_positions(self):
        """Generate random positions for pebbles."""
        np.random.seed(self.random_seed)
        positions = []
        
        while len(positions) < self.num_pebbles:
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(self.target_zone_radius, self.env_radius)
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            positions.append((x, y, 0.1))
            
        return positions
        
    def setup_scene(self, initial_robot_pose):
        """
        Reset the world, load plane, rover and pebbles, and cache handles.
        After this call:
            self.robot_id        – the rover base body unique ID
            self.left_wheel_joint,
            self.right_wheel_joint – the two drive-wheel joint indices
        """
        import os, numpy as np, pybullet as p

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240.)
        self.object_ids = []                # clear previous handles

        # ---- ground plane -------------------------------------------------
        plane_id = p.loadURDF("plane.urdf")
        self.object_ids.append(plane_id)

        # ---- rover --------------------------------------------------------
        x, y, theta = initial_robot_pose
        init_quat = p.getQuaternionFromEuler([0, 0, theta])
        robot_urdf = os.path.join(os.path.dirname(__file__), "2_wheel_rover.urdf")
        rover_id = p.loadURDF(robot_urdf, [x, y, 0.10], init_quat)
        self.object_ids.append(rover_id)
        self.robot_id = rover_id            # ☆ keep a persistent handle

        # Disable default position control on every joint
        for j in range(p.getNumJoints(rover_id)):
            p.setJointMotorControl2(rover_id, j, p.VELOCITY_CONTROL, force=0)

        # Apply friction to wheels and floor
        for j in range(p.getNumJoints(rover_id)):
            name = p.getJointInfo(rover_id, j)[1].decode().lower()
            if "wheel" in name:
                p.changeDynamics(rover_id, j, lateralFriction=1.5)
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Cache wheel joint indices for fast control
        self.left_wheel_joint = None
        self.right_wheel_joint = None
        for j in range(p.getNumJoints(rover_id)):
            name = p.getJointInfo(rover_id, j)[1].decode().lower()
            if "lwheel" in name:
                self.left_wheel_joint = j
            elif "rwheel" in name:
                self.right_wheel_joint = j

        # ---- visual helpers & pebbles (unchanged) -------------------------
        self.create_target_zone()
        self.create_grid_overlay()
        pebble_positions = self.generate_random_positions()
        for pos in pebble_positions:
            pebble_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                       self.pebbles_urdf)
            pebble_id = p.loadURDF(pebble_path, pos, useFixedBase=False)
            self.object_ids.append(pebble_id)

        print("[setup] Scene ready – plane, rover and", len(pebble_positions), "pebbles loaded.")
            
    # def reset_simulation(self):
    #     """Reset simulation to initial state."""
    #     for obj_id in self.object_ids:
    #         p.removeBody(obj_id)
    #     self.object_ids = []
    #     self.setup_scene(self.initial_robot_pose)

        
    def run(self, auto_continue=False):
        print("Setting up scene...")
        self.setup_scene(self.initial_robot_pose)

        p.resetDebugVisualizerCamera(
            cameraDistance=3.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        if auto_continue:
            # Auto-continue without waiting for key press
            print("Auto-continuing to 2D visualization...")
            return self.get_current_state()

        while True:
            p.stepSimulation()
            keys = p.getKeyboardEvents()
            if ord('q') in keys:
                break
            elif ord('c') in keys:
                return self.get_current_state()
            time.sleep(1 / 240)


    def visualize_trajectory(self, trajectory_2d):
        """Visualize a 2D trajectory in the 3D environment."""
        # Convert 2D grid coordinates to 3D world coordinates
        trajectory_3d = []
        for x, y in trajectory_2d:
            # Convert grid coordinates to world coordinates
            world_x, world_y = self.coord_converter.convert_2d_to_3d(x, y)
            trajectory_3d.append((world_x, world_y, 0.01))  # Just slightly above ground
        
        # Draw trajectory lines
        for i in range(len(trajectory_3d) - 1):
            start = trajectory_3d[i]
            end = trajectory_3d[i + 1]
            p.addUserDebugLine(
                start,
                end,
                [1, 0, 0],  # Red color
                2.0,  # Line width
                lifeTime=0  # Forever
            )                    
                
    def clear_trajectory(self):
        """Clear only trajectory visualization lines, preserving environment."""
        print("🧹 Clearing trajectory visualizations (preserving environment)...")
        
        # Note: Instead of removeAllUserDebugItems() which is too aggressive,
        # we'll let the trajectory lines fade naturally or rely on specific removal.
        # For now, we'll just recreate the essential visual elements.
        
        # Only recreate grid overlay if needed (don't recreate target zone)
        if not hasattr(self, 'grid_lines') or not self.grid_lines:
            self.create_grid_overlay()
        
        print("  ✅ Trajectory cleared, 3D environment preserved")

    def _update_2d_environment_with_new_positions(self, env, updated_objects_2d):
        """Update 2D environment grid with new object positions using main.py process."""
        print("  🔄 Updating 2D environment with new object positions...")
        
        try:
            # Convert 2D grid coordinates back to world coordinates for main.py process
            updated_objects_3d = []
            for grid_x, grid_y in updated_objects_2d:
                try:
                    world_x, world_y = self.coord_converter.convert_2d_to_3d(grid_x, grid_y)
                    updated_objects_3d.append((world_x, world_y, 0.1))  # Standard height for pebbles
                except Exception as e:
                    print(f"    ⚠️ Could not convert 2D({grid_x}, {grid_y}) back to world: {e}")
            
            print(f"    📍 Converted {len(updated_objects_3d)} positions back to world coordinates")
            
            # Method 1: Update real_objects with 3D world coordinates (like main.py expects)
            if hasattr(env, 'real_objects'):
                old_count = len(env.real_objects)
                # env.real_objects expects 3D WORLD coordinates, not 2D grid coordinates!
                env.real_objects = updated_objects_3d  # Use the 3D world coords we converted back
                print(f"    ✅ Updated env.real_objects with 3D world coords ({old_count} → {len(updated_objects_3d)} objects)")
                print(f"      Sample: 3D({updated_objects_3d[0][0]:.3f}, {updated_objects_3d[0][1]:.3f}) vs 2D({updated_objects_2d[0][0]}, {updated_objects_2d[0][1]})")
            else:
                print(f"    ⚠️ env.real_objects not found, proceeding with grid update")
            
            # Method 2: Completely rebuild the environment grid like main.py does
            if hasattr(env, 'grid') and hasattr(env, 'cells_with_objects'):
                print("    🔄 Rebuilding environment grid with new positions...")
                
                # Clear all existing object counts
                for row in env.grid:
                    for cell in row:
                        cell.num_objects = 0
                        cell.visible_cells_target = []
                        cell.distance_to_children_target = []
                
                # Add objects to new positions and rebuild cells_with_objects
                new_cells_with_objects = []
                for grid_x, grid_y in updated_objects_2d:
                    if 0 <= grid_x < env.grid_size and 0 <= grid_y < env.grid_size:
                        cell = env.grid[grid_x][grid_y]
                        cell.num_objects += 1
                        if cell not in new_cells_with_objects:
                            new_cells_with_objects.append(cell)
                
                env.cells_with_objects = new_cells_with_objects
                print(f"    ✅ Rebuilt grid with {len(new_cells_with_objects)} cells containing objects")
                
                # Reinitialize cell properties like main.py does
                print("    🔄 Reinitializing cell properties...")
                for cell in env.cells_with_objects:
                    # Calculate closest point on target zone
                    closest_point_target = env.find_closest_point_on_target((cell.x + 0.5, cell.y + 0.5))
                    
                    # Import Cell class if needed
                    try:
                        from cell import Cell
                        target_cell_target = Cell(
                            int(closest_point_target[0]), int(closest_point_target[1]), 0, env.target_zone, env.grid_size
                        )
                        
                        # Calculate visibility and distances
                        visible_cells_target, distance_to_children_target = env.calculate_visibility_simple(
                            cell, angle_tolerance=60, target_cell=target_cell_target
                        )
                        cell.visible_cells_target = visible_cells_target
                        cell.distance_to_children_target = distance_to_children_target
                    except ImportError:
                        print("      ⚠️ Could not import Cell class for visibility calculation")
                
                print("    ✅ Cell properties reinitialized")
                return
            
            print("    ⚠️ Warning: Could not find a method to update 2D object positions")
            
        except Exception as e:
            print(f"    ⚠️ Error updating 2D environment: {e}")

    def _force_2d_environment_recalculation(self, env):
        """Force complete recalculation following the exact main.py process."""
        print("  🌡️ Forcing complete 2D environment recalculation (main.py process)...")
        
        try:
            # Follow the exact same sequence as main.py lines 65-83
            
            # Step 1: Calculate potential field (main.py lines 65-68)
            print("    🔄 Calculating potential field...")
            if hasattr(env, 'calculate_potential_field'):
                env.calculate_potential_field(use_spillage_model=False, visualize=False)
                print("    ✅ Potential field calculated")
            else:
                print("    ⚠️ calculate_potential_field method not found")
            
            # Step 2: Calculate velocity field (main.py lines 70-73)
            print("    🔄 Calculating velocity field...")
            if hasattr(env, 'calculate_velocity_field'):
                env.calculate_velocity_field()
                print("    ✅ Velocity field calculated")
            else:
                print("    ⚠️ calculate_velocity_field method not found")
            
            # Step 3: Update heat map (main.py lines 75-78)
            print("    🔄 Simulating flow and updating heat map...")
            if hasattr(env, 'update_heat_map'):
                env.update_heat_map()
                print("    ✅ Heat map updated")
            else:
                print("    ⚠️ update_heat_map method not found")
            
            # Step 4: Calculate paths to highways (main.py lines 80-83)
            print("    🔄 Calculating paths to highways for low-potential cells...")
            if hasattr(env, 'calculate_path_to_highway'):
                env.calculate_path_to_highway()
                print("    ✅ Paths to highways calculated")
            else:
                print("    ⚠️ calculate_path_to_highway method not found")
            
            print("    ✅ Complete 2D environment recalculation finished!")
            
        except Exception as e:
            print(f"    ⚠️ Error during main.py-style recalculation: {e}")
            
            # Fallback: Try basic update methods
            print("    🔄 Attempting fallback recalculation...")
            try:
                if hasattr(env, 'update_environment'):
                    # Set affected_cells to all cells to force full recalculation
                    if hasattr(env, 'affected_cells') and hasattr(env, 'cells_with_objects'):
                        env.affected_cells = list(env.cells_with_objects)
                    env.update_environment()
                    print("    ✅ Fallback: Used env.update_environment()")
                else:
                    print("    ⚠️ No fallback recalculation methods available")
            except Exception as fallback_error:
                print(f"    ⚠️ Fallback recalculation also failed: {fallback_error}")

    def simulate(self, sim_time=0.1, step=1/240):
        """Simulate environment for given time."""
        for _ in range(int(sim_time/step)):
            p.stepSimulation()
            time.sleep(step)

    def close_environment(self):
        """Close PyBullet environment."""
        p.disconnect()

    def generate_bezier_trajectory(self, start_pose, end_pose, duration=3.0, step_time=0.1, scale=0.5):
        """
        Generate a smooth cubic Bézier trajectory from start_pose to end_pose.
        :param start_pose: (x0, y0, theta0)
        :param end_pose: (x1, y1, theta1)
        :param duration: total path duration in seconds
        :param step_time: time step for sampling
        :param scale: distance along heading for control points
        :return: list of (x, y, heading) waypoints
        """
        x0, y0, theta0 = start_pose
        x3, y3, theta3 = end_pose

        # Control points for cubic Bézier
        p0 = np.array([x0, y0])
        p1 = p0 + scale * np.array([np.cos(theta0), np.sin(theta0)])
        p3 = np.array([x3, y3])
        p2 = p3 - scale * np.array([np.cos(theta3), np.sin(theta3)])

        num_steps = int(duration / step_time)
        trajectory = []

        for i in range(num_steps):
            t = i / (num_steps - 1)
            # Cubic Bézier formula
            point = (
                (1 - t) ** 3 * p0 +
                3 * (1 - t) ** 2 * t * p1 +
                3 * (1 - t) * t ** 2 * p2 +
                t ** 3 * p3
            )
            # Tangent (derivative of Bézier curve)
            dp_dt = (
                3 * (1 - t) ** 2 * (p1 - p0) +
                6 * (1 - t) * t * (p2 - p1) +
                3 * t ** 2 * (p3 - p2)
            )
            heading = np.arctan2(dp_dt[1], dp_dt[0])
            trajectory.append((point[0], point[1], heading))

        return trajectory

    def follow_smooth_trajectory(
        self,
        trajectory,
        control_dt      = 1 / 240,
        pos_tol         = 0.05,      # metres
        angle_tol       = 0.20,      # rad  ≈ 11°
        lookahead       = 1,         # how many way-points to peek ahead
        min_speed       = 0.04       # m/s we apply while pivoting so we never stall
    ):
        """
        Closed-loop tracking of a list of (x, y, θ) way-points.

        * Steps through the list in order, but if we are already closer than
          `pos_tol` to the current target, we **immediately advance** to the next
          one (no matter the heading).  This prevents the “stuck turning in place”
          problem once the rover is on top of a point but still mis-aligned.

        * Keeps a small forward component `min_speed` even while it is mostly
          correcting heading, so static friction (or a pebble nudge) can’t freeze
          the wheels.

        * `lookahead>0` lets the robot aim slightly further down the path for
          smoother motion on very dense trajectories.
        """
        idx = 0
        finished = False   
        max_iter_0 = 200  # hard safety stop     
        max_iter = max_iter_0     # hard safety stop
        
        while not finished:                        

            # ── Current pose ──────────────────────────────────────────────
            (x, y, _), q = self.get_robot_position()
            theta = p.getEulerFromQuaternion(q)[2]

            # ── Which point are we chasing? ──────────────────────────────
            #   Skip points that are already within the distance tolerance,
            #   regardless of the current heading.
            while idx < len(trajectory):
                dx = trajectory[idx][0] - x
                dy = trajectory[idx][1] - y
                max_iter -= 1
                if math.hypot(dx, dy) > pos_tol and max_iter > 0:
                    break                      # this one still matters
                max_iter = max_iter_0        # hard safety stop
                idx += 1                       # already “there” → next

            if idx >= len(trajectory):
                finished = True
                break

            # Optional look-ahead
            tgt_idx  = min(idx + lookahead, len(trajectory) - 1)
            x_t, y_t, th_t = trajectory[tgt_idx]

            # ── Errors in world frame ────────────────────────────────────
            dx   = x_t - x
            dy   = y_t - y
            dθ   = self.normalize_angle(th_t - theta)

            dist = math.hypot(dx, dy)
            ang  = abs(dθ)

            # ── Debug line (your “sanity check”) ─────────────────────────
            print(f"idx={idx:3d}  dist={dist:5.3f}  ang={ang:5.3f}  iter={max_iter:4d}")

            # ── Transform position error to body frame ───────────────────
            ex =  math.cos(theta) * dx + math.sin(theta) * dy   # fwd
            ey = -math.sin(theta) * dx + math.cos(theta) * dy   # left

            # ── Adaptive gain scheduling based on trajectory phase ────────
            current_pos = (x, y, 0)
            K1_adaptive, K2_adaptive, K3_adaptive, vmax_adaptive = self.adaptive_gains(
                current_pos, trajectory, idx
            )
            
            # ── Adaptive lookahead based on trajectory phase ──────────────
            dist_to_end = math.hypot(trajectory[-1][0] - x, trajectory[-1][1] - y)
            if dist_to_end > 0.8:  # Approach phase - use larger lookahead
                adaptive_lookahead = min(lookahead + 2, 4)
            elif dist_to_end > 0.3:  # Tracking phase - normal lookahead  
                adaptive_lookahead = lookahead
            else:  # Precision phase - smaller lookahead for precise control
                adaptive_lookahead = max(lookahead - 1, 0)
            
            # Update target if needed based on adaptive lookahead
            if adaptive_lookahead != lookahead:
                new_tgt_idx = min(idx + adaptive_lookahead, len(trajectory) - 1)
                if new_tgt_idx != tgt_idx:
                    tgt_idx = new_tgt_idx
                    x_t, y_t, th_t = trajectory[tgt_idx]
                    dx = x_t - x
                    dy = y_t - y  
                    dθ = self.normalize_angle(th_t - theta)
                    ex =  math.cos(theta) * dx + math.sin(theta) * dy
                    ey = -math.sin(theta) * dx + math.cos(theta) * dy

            # ── Adaptive controller with scheduled gains ──────────────────
            V   = K1_adaptive * ex
            φ   = K2_adaptive * ey + K3_adaptive * dθ
            
            # Apply velocity limits based on adaptive gains
            V = np.clip(V, -vmax_adaptive, vmax_adaptive)

            ω_r, ω_l = self.compute_wheel_velocities(V, φ)
            self.control_rover_velocity(ω_l, ω_r, control_dt)

    def draw_trajectory(self, trajectory, color=[1, 0, 0], life_time=0):
        """
        Draws a trajectory in PyBullet using debug lines.
        
        :param trajectory: List of waypoints, either (x, y) or (x, y, theta)
        :param color: RGB list, default red
        :param life_time: Duration to keep the lines (0 = forever)
        """
        for i in range(len(trajectory) - 1):
            # Handle both 2D and 3D points
            if len(trajectory[i]) == 2:
                x0, y0 = trajectory[i]
                x1, y1 = trajectory[i + 1]
            else:
                x0, y0, _ = trajectory[i]
                x1, y1, _ = trajectory[i + 1]
                
            p.addUserDebugLine(
                [x0, y0, 0.05],  # Start point
                [x1, y1, 0.05],  # End point
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=life_time
            )





def main():
    integration = PyBulletIntegration()
    integration.run()

if __name__ == "__main__":
    main()