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

def _curvature_of_three_pts(a, b, c):
    ax, ay = a; bx, by = b; cx, cy = c
    A = np.array([ax, ay]); B = np.array([bx, by]); C = np.array([cx, cy])
    a_len = np.linalg.norm(B - C)
    b_len = np.linalg.norm(C - A)
    c_len = np.linalg.norm(A - B)
    area2 = abs((B[0]-A[0])*(C[1]-A[1]) - (B[1]-A[1])*(C[0]-A[0]))  # 2*triangle area
    denom = a_len * b_len * c_len
    if denom < 1e-9:
        return 0.0
    kappa = 2.0 * area2 / denom
    # sign from local turn (z component of cross)
    sgn = np.sign((B[0]-A[0])*(C[1]-B[1]) - (B[1]-A[1])*(C[0]-B[0]))
    return sgn * kappa

def _curvature_at_s(pts, s_cum, seg_len, s, delta=0.08):
    """Curvature of the polyline around arclength s using three samples."""
    p0 = point_at_s(pts, s_cum, seg_len, s - delta)
    p1 = point_at_s(pts, s_cum, seg_len, s)
    p2 = point_at_s(pts, s_cum, seg_len, s + delta)
    return _curvature_of_three_pts(p0, p1, p2)

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

def _ang_mean(angles):
    """Circular mean of angles (radians)."""
    if not angles:
        return 0.0
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)

def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def _ang_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))

def _rot90(v):  # left-normal
    return np.array([-v[1], v[0]])

def _splice_with_fillet(approach, task, R=0.12, angle_thresh_deg=20.0):
    """Return a single combined list where the corner between approach end and task start
       is replaced by a circular arc of radius R when the turn is sharp."""
    if len(approach) < 2 or len(task) < 2:
        return approach + task
    A1 = np.array(approach[-2][:2]);
    A2 = np.array(approach[-1][:2])
    B1 = np.array(task[0][:2]);
    B2 = np.array(task[1][:2])

    # If the lists don't actually meet, just join them
    if np.linalg.norm(A2 - B1) > 0.03:
        return approach + task

    u1 = A2 - A1;
    u2 = B2 - B1
    if np.linalg.norm(u1) < 1e-6 or np.linalg.norm(u2) < 1e-6:
        return approach + task
    u1 = u1 / np.linalg.norm(u1);
    u2 = u2 / np.linalg.norm(u2)

    dot = float(np.clip(np.dot(u1, u2), -1.0, 1.0))
    ang = math.acos(dot)
    if math.degrees(ang) < angle_thresh_deg:
        return approach + task  # already smooth

    # left/right turn
    sgn = np.sign(u1[0] * u2[1] - u1[1] * u2[0])
    n1 = sgn * _rot90(u1)
    n2 = sgn * _rot90(u2)

    # distances from vertex to tangent points
    d = R * math.tan(ang / 2.0)
    L1 = min(np.linalg.norm(A2 - A1) - 1e-3, d)
    L2 = min(np.linalg.norm(B2 - B1) - 1e-3, d)
    if L1 <= 1e-3 or L2 <= 1e-3:
        return approach + task

    T1 = A2 - u1 * L1  # exit tangent point
    T2 = B1 + u2 * L2  # entry tangent point

    # circle center is at intersection of lines T1 + a*n1 and T2 + b*n2
    M = np.column_stack((n1, -n2))
    try:
        a_b = np.linalg.solve(M, (T2 - T1))
    except np.linalg.LinAlgError:
        return approach + task
    C = T1 + a_b[0] * n1

    # angles at center
    a1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
    a2 = math.atan2(T2[1] - C[1], T2[0] - C[0])

    # sweep in correct direction
    def ang_diff(a, b):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        return d

    sweep = ang_diff(a1, a2)
    if sgn > 0 and sweep < 0:
        sweep += 2 * math.pi
    if sgn < 0 and sweep > 0:
        sweep -= 2 * math.pi

    steps = max(6, int(abs(sweep) * R / 0.02))
    arc_pts = []
    for i in range(1, steps):
        t = i / steps
        a = a1 + t * sweep
        arc_pts.append((C[0] + R * math.cos(a), C[1] + R * math.sin(a), 0.0))

    # Combine
    return approach[:-1] + [(T1[0], T1[1], 0.0)] + arc_pts + [(T2[0], T2[1], 0.0)] + task[1:]

# ===================== ALIGNMENT GATE HELPERS ======================

def _tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat=0.0):
    """Calculate tool pose from base pose. Wrapper for existing tool_pose_from_base."""
    return tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat)

def _alignment_metrics(integration_obj, S0, S1, *, use_tcp_pose=True, tcp_fwd=0.12, tcp_lat=0.0):
    """Return (dot_align, behind, dist_to_S0) measured at TCP or base."""
    # Get robot pose
    robot_id = integration_obj.object_ids[1]
    pos, quat = p.getBasePositionAndOrientation(robot_id) 
    th = p.getEulerFromQuaternion(quat)[2]
    x_b, y_b = pos[0], pos[1]
    
    if use_tcp_pose:
        x_r, y_r, _ = _tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat)
    else:
        x_r, y_r = x_b, y_b

    # unit vector from robot to S0
    dx0, dy0 = (S0[0] - x_r), (S0[1] - y_r)
    d = math.hypot(dx0, dy0) + 1e-9
    vx0, vy0 = dx0 / d, dy0 / d

    # unit path direction S0->S1
    vxp, vyp = (S1[0] - S0[0]), (S1[1] - S0[1])
    npth = math.hypot(vxp, vyp) + 1e-9
    vxp /= npth; vyp /= npth

    # alignment (cosine of angle)
    dot_align = max(-1.0, min(1.0, vx0 * vxp + vy0 * vyp))
    # behind if (r - S0)·v_path < 0  <=> (S0 - r)·v_path > 0
    behind = ((x_r - S0[0]) * vxp + (y_r - S0[1]) * vyp) < 0.0
    return dot_align, behind, d

def _goto_point(integration_obj, goal_xy, *,
                use_tcp_pose=True, tcp_fwd=0.12, tcp_lat=0.0,
                v_nom=0.25, yaw_slew=8.0, tol=0.02, max_time=8.0, dt=1/240):
    """Small steering loop to drive to a single (x,y) until within tol."""
    t = 0.0; e_prev = 0.0
    robot_id = integration_obj.object_ids[1]
    
    while t < max_time:
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        th = p.getEulerFromQuaternion(quat)[2]
        x_b, y_b = pos[0], pos[1]
        
        if use_tcp_pose:
            x_o, y_o, _ = _tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat)
        else:
            x_o, y_o = x_b, y_b

        dx, dy = goal_xy[0] - x_o, goal_xy[1] - y_o
        dist = math.hypot(dx, dy)
        if dist <= tol:
            break

        hdg = math.atan2(dy, dx)
        e = wrap_angle(hdg - th)

        V = v_nom * float(np.clip(dist / 0.25, 0.25, 1.0))
        if abs(e) > math.radians(45):
            V = min(V, 0.15)

        k_yaw = 3.0
        yaw_cmd = k_yaw * e + 2.0 * (e - e_prev) / dt
        yaw_cmd = float(np.clip(yaw_cmd, -yaw_slew, yaw_slew))

        integration_obj.set_wheel_speeds_unitsafe(V, yaw_cmd, dt)
        e_prev = e; t += dt

    integration_obj._stop_robot_smoothly()

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

    def _extend_path_backwards(self, grid_waypoints, extension_config):
        """
        Extend the 2D path backwards from the starting waypoint for smoother approach.
        
        Extension direction is determined purely by the 2D path direction (first few waypoints),
        extending backwards in the opposite direction of the natural path flow.
        
        Args:
            grid_waypoints: List of (grid_x, grid_y) tuples from 2D path planning
            extension_config: Dictionary with extension parameters
                - 'extension_length': How far to extend backwards (meters)
        
        Returns:
            List of (grid_x, grid_y) tuples with extended path
        """
        if not extension_config.get('enable_extension', False) or len(grid_waypoints) < 2:
            return grid_waypoints
            
        print(f"🛤️ Extending path backwards by {extension_config['extension_length']:.2f}m...")
        
        import math
        
        # Get extension parameters
        extension_length = extension_config.get('extension_length', 1.0)
        
        # Analyze first 2-3 waypoints to determine natural path direction
        start_grid = grid_waypoints[0]
        
        # Calculate average direction from first few segments
        directions = []
        for i in range(min(3, len(grid_waypoints) - 1)):
            current = grid_waypoints[i]
            next_point = grid_waypoints[i + 1]
            
            # Convert to world coordinates for accurate direction calculation
            current_world = self.coord_converter.convert_2d_to_3d(*current)
            next_world = self.coord_converter.convert_2d_to_3d(*next_point)
            
            direction = math.atan2(next_world[1] - current_world[1], 
                                 next_world[0] - current_world[0])
            directions.append(direction)
        
        # Average the directions for smooth extension
        if directions:
            # Calculate circular mean for angles
            sum_cos = sum(math.cos(angle) for angle in directions)
            sum_sin = sum(math.sin(angle) for angle in directions)
            avg_direction = math.atan2(sum_sin, sum_cos)
        else:
            avg_direction = 0.0
        
        # Extension direction is opposite to the path direction
        extension_direction = avg_direction + math.pi
        
        start_world = self.coord_converter.convert_2d_to_3d(*start_grid)
        
        print(f"  📍 2D path starts at Cell({start_grid[0]}, {start_grid[1]}) → World({start_world[0]:.3f}, {start_world[1]:.3f})")
        print(f"  🧭 Path direction: {math.degrees(avg_direction):.1f}°")
        print(f"  ⬅️ Extension direction (opposite): {math.degrees(extension_direction):.1f}°")
        
        # Create extension points
        cell_size = self.coord_converter.cell_size
        num_extension_points = max(2, int(extension_length / cell_size))
        
        extended_waypoints = []
        
        # Generate extension points backwards from start
        for i in range(num_extension_points, 0, -1):
            # Distance from start point (evenly spaced)
            distance = (i / num_extension_points) * extension_length
            
            # World coordinates of extension point  
            ext_world_x = start_world[0] + distance * math.cos(extension_direction)
            ext_world_y = start_world[1] + distance * math.sin(extension_direction)
            
            # Convert back to grid coordinates
            try:
                ext_grid_x, ext_grid_y = self.coord_converter.convert_3d_to_2d(ext_world_x, ext_world_y)
                extended_waypoints.append((ext_grid_x, ext_grid_y))
                print(f"    Extension Point {len(extended_waypoints)}: Cell({ext_grid_x}, {ext_grid_y}) at {distance:.2f}m")
            except Exception as e:
                print(f"    ⚠️ Extension point outside grid bounds: {e}")
                break
        
        # Combine extension + original path
        final_waypoints = extended_waypoints + grid_waypoints
        
        print(f"  ✅ Path extended: {len(extended_waypoints)} extension points + {len(grid_waypoints)} original = {len(final_waypoints)} total")
        print(f"    📏 Extension covers: {len(extended_waypoints) * cell_size:.2f}m")
        
        return final_waypoints



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
    def _turn_in_place(self, target_theta: float, *, max_rate: float = 1.5, tol: float = 0.02):
        """
        Accurate rotation with PID-like control to minimize overshoot.
        Uses differential wheel speeds for pure rotation.
        """
        (x_start, y_start), th_start = self.pose()
        err = wrap_angle(target_theta - th_start)
        
        print(f"   Starting angle: {math.degrees(th_start):.1f}°")
        print(f"   Target angle: {math.degrees(target_theta):.1f}°") 
        print(f"   Error: {math.degrees(err):.1f}°")
        
        steps = 0
        max_steps = 2000  # Safety limit
        prev_err = err
        integral_err = 0.0
        
        # PID-like control parameters
        kp = 3.0  # Proportional gain
        ki = 0.1  # Integral gain  
        kd = 0.8  # Derivative gain
        
        while abs(err) > tol and steps < max_steps:
            # PID-like control calculation
            integral_err += err
            derivative_err = err - prev_err
            
            # Control signal
            control_signal = kp * err + ki * integral_err + kd * derivative_err
            
            # Limit control signal
            control_signal = max(-max_rate, min(max_rate, control_signal))
            
            # Multi-stage speed ramping for accuracy
            if abs(err) > math.radians(30):  # Large error - normal speed
                speed_factor = 1.0
            elif abs(err) > math.radians(15):  # Medium error - reduce speed
                speed_factor = 0.6
            elif abs(err) > math.radians(5):   # Small error - slow speed
                speed_factor = 0.3
            else:  # Very small error - very slow
                speed_factor = 0.15
            
            # Apply speed factor
            control_signal *= speed_factor
            
            # Drive by yaw-rate only; helper converts to correct wheel speeds & steps once
            self.set_wheel_speeds_unitsafe(0.0, control_signal, dt=1/240)
            # (No extra p.stepSimulation here - set_wheel_speeds_unitsafe handles stepping)
            _, th = self.pose()
            prev_err = err
            err = wrap_angle(target_theta - th)
            steps += 1
            
            # Prevent integral windup
            if abs(integral_err) > 5.0:
                integral_err *= 0.9
        
        # FIXED: Proper stopping with active braking (no motor disable)
        print(f"   🛑 Executing active braking sequence...")
        
        # Step 1: Actively ramp yaw-rate down to zero (real braking)
        print(f"   🔧 Gradual velocity reduction...")
        # Store the last control_signal from the loop above for proper ramp-down
        cmd_prev = control_signal if 'control_signal' in locals() else 0.0
        
        for i in range(40):
            factor = (40 - i) / 40.0
            # Gradually reduce the yaw rate command to zero
            self.set_wheel_speeds_unitsafe(0.0, cmd_prev * factor, dt=1/240)
        
        # Step 2: Final active stop + settle (uses velocity=0 with motor force to hold)
        print(f"   🔒 Active stop and settle...")
        for _ in range(120):  # 0.5 seconds of active holding at zero velocity
            self.set_wheel_speeds_unitsafe(0.0, 0.0, dt=1/240)
        
        # Step 3: Verification
        robot_id = self.object_ids[1]
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        base_speed = math.hypot(lin_vel[0], lin_vel[1])
        
        left_joint = self.get_joint_index_by_name("base_to_lwheel")
        right_joint = self.get_joint_index_by_name("base_to_rwheel")
        left_vel = p.getJointState(robot_id, left_joint)[1]
        right_vel = p.getJointState(robot_id, right_joint)[1]
        
        print(f"   ✅ Final base velocity: {base_speed:.6f} m/s")
        print(f"   ✅ Final wheel velocities: L={left_vel:.6f}, R={right_vel:.6f} rad/s")
        
        if base_speed > 0.001:  # 1mm/s threshold
            print(f"   ⚠️  WARNING: Residual movement {base_speed:.6f} m/s detected!")
        else:
            print(f"   🎯 Turn-in-place stopped successfully - motors actively holding position")
        
        # Final accuracy check
        (x_end, y_end), th_end = self.pose()
        distance_moved = math.hypot(x_end - x_start, y_end - y_start)
        final_error = abs(wrap_angle(target_theta - th_end))
        
        # Motors are already in correct state from set_wheel_speeds_unitsafe
        # No additional motor setup needed
        
        print(f"   Final angle: {math.degrees(th_end):.1f}°")
        print(f"   Final error: {math.degrees(final_error):.2f}°")
        print(f"   Distance moved: {distance_moved:.4f}m")
        print(f"   Steps taken: {steps}")
        
        if distance_moved > 0.05:  # 5cm tolerance
            print(f"   ⚠️  WARNING: Rover translated {distance_moved:.4f}m during turn!")
        if final_error > math.radians(5):  # 5° tolerance
            print(f"   ⚠️  WARNING: Turn accuracy error {math.degrees(final_error):.2f}°!")
        
        print(f"   ✅ Turn-in-place function completed successfully")
        
        # Warnings for poor performance
        if steps >= max_steps:
            print(f"   ⚠️ Warning: Hit step limit, may not have reached target!")
        if distance_moved > 0.08:  # Tighter tolerance
            print(f"   ⚠️ Warning: Rover moved too much during turn!")
        if final_error > math.radians(3):  # Tighter accuracy requirement
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
            pivot_thresh_deg: float = 35.0,
            pivot_rate: float = 2.5,
            entry_fillet_radius: float = 0.12,
            entry_fillet_min_turn_deg: float = 20.0,
            extension_config=None
    ):
        """
        Returns (approach, task_world, path_resampled)
          • Optional pivot-in-place
          • Bézier approach with distance-aware scale, heading averaged from first segments
          • Add a circular entry fillet if the turn is sharp, then prune & resample
        """
        # --- current base pose ------------------------------------------------
        robot_id = self.object_ids[1]
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        th_now = p.getEulerFromQuaternion(quat)[2]

        # --- entry heading: average first 2–3 segments ------------------------
        g = env.current_trajectory
        start_grid = g[0]
        sx, sy = self.coord_converter.convert_2d_to_3d(*start_grid)
        th_list = []
        for i in range(0, min(3, len(g) - 1)):
            (g0, g1) = g[i], g[i + 1]
            w0 = self.coord_converter.convert_2d_to_3d(*g0)
            w1 = self.coord_converter.convert_2d_to_3d(*g1)
            th_list.append(math.atan2(w1[1] - w0[1], w1[0] - w0[0]))
        th_task = _ang_mean(th_list) if th_list else th_now

        # Optional pivot (leave disabled if you prefer external call)
        hdg_err = wrap_angle(th_task - th_now)
        if False and abs(math.degrees(hdg_err)) > pivot_thresh_deg:
            self._turn_in_place(th_task, max_rate=pivot_rate)
            pos, quat = p.getBasePositionAndOrientation(robot_id)
            th_now = p.getEulerFromQuaternion(quat)[2]

        current_pose = (pos[0], pos[1], th_now)

        # 1) Build task trajectory first (may include extension points at the front)
        task_world = self._create_smooth_task_trajectory(g, extension_config=extension_config)
        
        # 2) Choose approach target: 
        # If extension is enabled and task_world starts before (sx, sy), aim for the first task point
        target_x, target_y = task_world[0][0], task_world[0][1]
        
        # Check if extension was actually added by comparing first task point with original start
        extension_detected = (abs(target_x - sx) > 0.01 or abs(target_y - sy) > 0.01)
        
        if extension_detected:
            print(f"🔗 Extension detected: Targeting approach to extended start ({target_x:.3f}, {target_y:.3f}) instead of original start ({sx:.3f}, {sy:.3f})")
        else:
            print(f"🔗 No extension: Targeting approach to original start ({sx:.3f}, {sy:.3f})")

        # 3) Build approach to the chosen target (keep same duration/scale logic)
        dist0 = math.hypot(pos[0] - target_x, pos[1] - target_y)
        scale_eff = np.clip(0.35 * dist0 + 0.20, 0.25, 0.60)
        
        start_pose = (target_x, target_y, th_task)
        approach = self.generate_bezier_trajectory(
            current_pose, start_pose,
            duration=approach_duration,
            scale=scale_eff
        )

        # 4) When extension is active, skip fillet and use simple concatenation
        # When no extension, use the original fillet logic
        if extension_detected:
            print(f"🔗 Using simple concatenation for extended path")
            raw = prune_close(approach + task_world, min_dist=0.003)
        else:
            print(f"🔗 Using fillet logic for non-extended path")
            # splice with circular fillet, then prune & resample
            raw_spliced = _splice_with_fillet(
                approach, task_world,
                R=entry_fillet_radius,
                angle_thresh_deg=entry_fillet_min_turn_deg
            )
            raw = prune_close(raw_spliced, min_dist=0.003)

        path = resample_polyline(raw, ds=ds)
        return approach, task_world, path

    def build_simple_world_path(
            self,
            env,
            ds: float = 0.05,
            turn_in_place_threshold_deg: float = 15.0
    ):
        """
        Simple trajectory approach:
        1. Go directly to the starting point of task trajectory
        2. Turn in place to correct orientation
        3. Execute task trajectory
        
        Returns (approach, task_world, path_resampled)
        """
        # Get current robot pose
        robot_id = self.object_ids[1]
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        th_now = p.getEulerFromQuaternion(quat)[2]
        current_pose = (pos[0], pos[1], th_now)
        
        # Get task trajectory start point and heading
        g = env.current_trajectory
        start_grid = g[0]
        sx, sy = self.coord_converter.convert_2d_to_3d(*start_grid)
        
        # Calculate task heading from first few segments
        th_list = []
        for i in range(0, min(3, len(g) - 1)):
            (g0, g1) = g[i], g[i + 1]
            w0 = self.coord_converter.convert_2d_to_3d(*g0)
            w1 = self.coord_converter.convert_2d_to_3d(*g1)
            th_list.append(math.atan2(w1[1] - w0[1], w1[0] - w0[0]))
        th_task = _ang_mean(th_list) if th_list else th_now
        
        print(f"🚗 Simple approach: Current pose ({pos[0]:.3f}, {pos[1]:.3f}, {math.degrees(th_now):.1f}°)")
        print(f"🎯 Task start: ({sx:.3f}, {sy:.3f}, {math.degrees(th_task):.1f}°)")
        
        # Phase 1: Create direct approach to task start point
        approach = [(current_pose[0], current_pose[1]), (sx, sy)]
        approach = resample_polyline(approach, ds=ds)
        
        print(f"📍 Approach phase: {len(approach)} points")
        
        # Phase 2: Check if turn-in-place is needed
        hdg_err = wrap_angle(th_task - th_now)
        if abs(math.degrees(hdg_err)) > turn_in_place_threshold_deg:
            print(f"🔄 Turn-in-place needed: {math.degrees(hdg_err):.1f}° error")
            # The turn will be handled by the orchestrator after approach
        
        # Phase 3: Build task trajectory
        task_world = []
        for i, (gx, gy) in enumerate(g):
            wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
            task_world.append((wx, wy))
        
        print(f"📋 Task phase: {len(task_world)} points")
        
        # Combine approach + task and resample
        raw = approach + task_world
        path = resample_polyline(raw, ds=ds)
        
        print(f"🛤️ Total path: {len(path)} points")
        
        return approach, task_world, path, {
            'turn_needed': abs(math.degrees(hdg_err)) > turn_in_place_threshold_deg, 
            'turn_angle': hdg_err,
            'target_heading': th_task  # Absolute task heading for proper pivot
        }

    def build_world_path_with_turn_extension(
            self,
            env,
            approach_duration: float = 2.0,
            approach_scale: float = 0.3,
            ds: float = 0.05,
            *,
            pivot_thresh_deg: float = 35.0,
            pivot_rate: float = 2.5,
            entry_fillet_radius: float = 0.12,
            entry_fillet_min_turn_deg: float = 20.0,
            enable_turn_in_place: bool = True  # NEW: Manual control for turn-in-place
    ):
        """
        Enhanced path builder that extends task trajectory backwards for turn-in-place operations.
        
        When sharp turns (>pivot_thresh_deg) are detected between approach and task:
        1. Extends task trajectory backwards by vehicle length + safety margin
        2. Creates smooth approach to extended pre-task position  
        3. Rover turns-in-place at extended position (outside collection cells)
        4. Continues with original task trajectory
        
        Returns (approach, task_world, path_resampled, turn_in_place_info)
        """
        
        # --- Rover dimensions from URDF analysis ---
        # Distance from rear wheels (-0.12m) to shovel tip (+0.17m) = 0.29m
        WHEELS_TO_SHOVEL = 0.29  # Distance from wheels to shovel tip
        EXTENSION_DISTANCE = WHEELS_TO_SHOVEL + 0.15  # ~0.44m (rover length + small safety margin)
        
        # --- current base pose ------------------------------------------------
        robot_id = self.object_ids[1]
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        th_now = p.getEulerFromQuaternion(quat)[2]
        current_pose = (pos[0], pos[1], th_now)

        # --- Calculate task trajectory heading from first segments -------------
        g = env.current_trajectory
        start_grid = g[0]
        sx, sy = self.coord_converter.convert_2d_to_3d(*start_grid)
        
        th_list = []
        for i in range(0, min(3, len(g) - 1)):
            (g0, g1) = g[i], g[i + 1]
            w0 = self.coord_converter.convert_2d_to_3d(*g0)
            w1 = self.coord_converter.convert_2d_to_3d(*g1)
            th_list.append(math.atan2(w1[1] - w0[1], w1[0] - w0[0]))
        th_task = _ang_mean(th_list) if th_list else th_now

        # --- Check if we need backward extension for turn-in-place ------------
        hdg_err = wrap_angle(th_task - th_now)
        needs_extension = enable_turn_in_place and abs(math.degrees(hdg_err)) > pivot_thresh_deg
        
        turn_in_place_info = {
            'needed': needs_extension,
            'target_angle': th_task,
            'heading_error_deg': math.degrees(hdg_err),
            'turn_position': None,
            'extension_distance': EXTENSION_DISTANCE if needs_extension else 0.0,
            'enabled': enable_turn_in_place,  # Track if feature is enabled
            'would_trigger': abs(math.degrees(hdg_err)) > pivot_thresh_deg  # Would it trigger if enabled?
        }
        
        if needs_extension:
            print(f"🔄 Sharp turn detected ({math.degrees(hdg_err):.1f}°) - extending task trajectory backwards")
            
            # --- Extend task trajectory backwards along its own path ---
            # Convert first few task segments to world coordinates
            task_points_world = []
            for i in range(min(3, len(g))):  # Use first 3 points for direction
                wx, wy = self.coord_converter.convert_2d_to_3d(g[i][0], g[i][1])
                task_points_world.append((wx, wy))
            
            if len(task_points_world) >= 2:
                # Calculate direction of FIRST task segment (from point 0 to point 1)
                p0, p1 = task_points_world[0], task_points_world[1]
                task_segment_direction = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                
                # Extend backwards along the REVERSE of first task segment
                extension_direction = task_segment_direction + math.pi
                
                extended_x = sx + EXTENSION_DISTANCE * math.cos(extension_direction)
                extended_y = sy + EXTENSION_DISTANCE * math.sin(extension_direction)
            else:
                # Fallback if only one task point
                extended_x = sx - EXTENSION_DISTANCE * math.cos(th_task)
                extended_y = sy - EXTENSION_DISTANCE * math.sin(th_task)
            
            # The extended position should have a heading that minimizes turn-in-place rotation
            # We want to arrive at extended position, then turn minimally to align with task
            # For now, let approach trajectory determine the arrival heading naturally
            extended_pose = (extended_x, extended_y, current_pose[2])  # Keep current heading for approach
            
            turn_in_place_info['turn_position'] = (extended_x, extended_y)
            
            print(f"  📍 Extended position: ({extended_x:.2f}, {extended_y:.2f})")
            print(f"  📍 Extension distance: {EXTENSION_DISTANCE:.2f}m")
            print(f"  📍 Task segment direction: {math.degrees(task_segment_direction):.1f}°")
            print(f"  📍 Extension direction: {math.degrees(extension_direction):.1f}°")
            
            # --- Create smooth approach to extended position ---
            # Approach should arrive at extended position ready to turn to task direction
            dist_to_extended = math.hypot(pos[0] - extended_x, pos[1] - extended_y)
            scale_eff = np.clip(0.35 * dist_to_extended + 0.20, 0.25, 0.60)
            
            approach = self.generate_bezier_trajectory(
                current_pose, extended_pose,
                duration=approach_duration,
                scale=scale_eff
            )
            
            # --- Create extended task trajectory ---
            # Start with extended position, then follow original task
            extended_task_world = [extended_pose]
            for gx, gy in g:
                wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
                extended_task_world.append((wx, wy, 0.0))
            
            task_world = extended_task_world
            
            # --- Create combined path (no fillet needed, turn-in-place handles transition) ---
            raw = prune_close(approach + task_world, min_dist=0.003)
            path = resample_polyline(raw, ds=ds)
            
            print(f"  ✅ Extended trajectory created: approach={len(approach)} + extended_task={len(task_world)} pts")
            
        else:
            # --- Use original approach for moderate turns or when disabled ---
            is_sharp_turn = abs(math.degrees(hdg_err)) > pivot_thresh_deg
            
            # Initialize extension variables for scope
            extended_x, extended_y = sx, sy  # Default to original task start
            SMOOTH_EXTENSION_DISTANCE = 0.0
            
            if not enable_turn_in_place and is_sharp_turn:
                print(f"🌊 Sharp turn detected ({math.degrees(hdg_err):.1f}°) but turn-in-place is DISABLED - using SMOOTH approach with extension")
                # SMOOTH APPROACH MODE: Create gentler curves by extending task trajectory backwards
                
                # --- Extend task trajectory backwards just like turn-in-place mode ---
                # Calculate direction of FIRST task segment (from point 0 to point 1)
                task_points_world = []
                for i in range(min(3, len(g))):  # Use first 3 points for direction
                    wx, wy = self.coord_converter.convert_2d_to_3d(g[i][0], g[i][1])
                    task_points_world.append((wx, wy))
                
                if len(task_points_world) >= 2:
                    # Calculate direction of FIRST task segment
                    p0, p1 = task_points_world[0], task_points_world[1]
                    task_segment_direction = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                    
                    # Extend backwards along the REVERSE of first task segment
                    extension_direction = task_segment_direction + math.pi
                    
                    # Use smaller extension for smooth mode (no turn-in-place needed)
                    SMOOTH_EXTENSION_DISTANCE = EXTENSION_DISTANCE * 0.6  # 60% of turn-in-place extension
                    
                    extended_x = sx + SMOOTH_EXTENSION_DISTANCE * math.cos(extension_direction)
                    extended_y = sy + SMOOTH_EXTENSION_DISTANCE * math.sin(extension_direction)
                else:
                    # Fallback if only one task point
                    SMOOTH_EXTENSION_DISTANCE = EXTENSION_DISTANCE * 0.6
                    extended_x = sx - SMOOTH_EXTENSION_DISTANCE * math.cos(th_task)
                    extended_y = sy - SMOOTH_EXTENSION_DISTANCE * math.sin(th_task)
                
                print(f"  📍 Extended smooth position: ({extended_x:.2f}, {extended_y:.2f})")
                print(f"  📍 Extension distance: {SMOOTH_EXTENSION_DISTANCE:.2f}m (60% of turn-in-place)")
                print(f"  📍 Extension direction: {math.degrees(extension_direction):.1f}°")
                
                # --- Create smooth approach to extended position ---
                extended_pose = (extended_x, extended_y, th_task)  # Target heading at extended position
                
                # Distance-aware Bézier scale with larger scale for smoother curves
                dist_to_extended = math.hypot(pos[0] - extended_x, pos[1] - extended_y)
                smooth_scale_factor = 1.5
                scale_eff = np.clip(smooth_scale_factor * (0.35 * dist_to_extended + 0.20), 0.40, 0.90)
                
                # Use longer duration for more gradual approach
                smooth_duration = approach_duration * 1.3  # 30% longer for smoother curves
                
                print(f"  📊 Smooth approach parameters: scale={scale_eff:.2f}, duration={smooth_duration:.1f}s")
                
                approach = self.generate_smooth_approach_trajectory(
                    current_pose, extended_pose,
                    duration=smooth_duration,
                    scale=scale_eff
                )
            else:
                if is_sharp_turn:
                    print(f"ℹ️ Sharp turn ({math.degrees(hdg_err):.1f}°) - would use turn-in-place but testing standard approach")
                else:
                    print(f"ℹ️ Moderate turn ({math.degrees(hdg_err):.1f}°) - using standard approach with fillet")
                
                # Distance-aware Bézier scale (original behavior)
                dist0 = math.hypot(pos[0] - sx, pos[1] - sy)
                scale_eff = np.clip(0.35 * dist0 + 0.20, 0.25, 0.60)

                start_pose = (sx, sy, th_task)
                approach = self.generate_bezier_trajectory(
                    current_pose, start_pose,
                    duration=approach_duration,
                    scale=scale_eff
                )

            # Task trajectory creation based on mode
            if not enable_turn_in_place and is_sharp_turn:
                # SMOOTH MODE: Create extended task trajectory starting from extended position
                print(f"  🌊 SMOOTH MODE: Creating extended task trajectory")
                task_world = [(extended_x, extended_y, th_task)]  # Start with extended position
                for gx, gy in g:
                    wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
                    task_world.append((wx, wy, 0.0))
                
                # Skip fillet entirely for gentler overall trajectory
                print(f"  🌊 SMOOTH MODE: Skipping fillet to maintain gentle curvature")
                raw = prune_close(approach + task_world, min_dist=0.003)
                path = resample_polyline(raw, ds=ds)
            else:
                # STANDARD MODE: Use original task trajectory with fillet
                task_world = []
                for gx, gy in g:
                    wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
                    task_world.append((wx, wy, 0.0))
                
                # Use fillet for transition smoothing
                raw_spliced = _splice_with_fillet(
                    approach, task_world,
                    R=entry_fillet_radius,
                    angle_thresh_deg=entry_fillet_min_turn_deg
                )
                raw = prune_close(raw_spliced, min_dist=0.003)
                path = resample_polyline(raw, ds=ds)
        
        return approach, task_world, path, turn_in_place_info

    def execute_enhanced_pure_pursuit_trajectory(self, env, mode="TCP", enable_turn_in_place=True, **kwargs):
        """
        Enhanced trajectory execution that handles turn-in-place at extended positions.
        
        Uses build_world_path_with_turn_extension() for intelligent trajectory planning:
        - Sharp turns (>35°): Extends trajectory backwards + turn-in-place at safe location
        - Moderate turns: Uses fillet approach for smooth transitions
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
            'approach_scale': 0.3,
            'pivot_thresh_deg': 35.0,
            'pivot_rate': 2.5
        }
        config.update(kwargs)

        print(f"\n🚗 Enhanced Pure Pursuit Execution (mode: {mode})...")

        # Build enhanced world path with turn extension capability
        approach, task_world, path_world, turn_info = self.build_world_path_with_turn_extension(
            env, 
            approach_duration=config['approach_duration'], 
            approach_scale=config['approach_scale'], 
            ds=config['resample_ds'],
            pivot_thresh_deg=config['pivot_thresh_deg'],
            pivot_rate=config['pivot_rate'],
            enable_turn_in_place=enable_turn_in_place  # Pass through control parameter
        )
        print(f"📊 Path components: Approach={len(approach)}, Task={len(task_world)}, Total={len(path_world)}")

        # Handle different control modes
        if mode.upper() == "BASE_SHIFT":
            from pybullet_integration import shift_path_along_s
            base_path = shift_path_along_s(path_world, s_shift=-config['tcp_fwd'])
            path_for_controller = base_path
            use_tcp_pose = False
            # Draw shifted path (purple)
            for i in range(len(base_path) - 1):
                p.addUserDebugLine([base_path[i][0], base_path[i][1], 0.035],
                                   [base_path[i+1][0], base_path[i+1][1], 0.035],
                                   [0.6, 0.0, 0.6], lineWidth=2.0)
        else:
            path_for_controller = path_world
            use_tcp_pose = True

        # Enhanced visualization
        self._draw_enhanced_path_visualization(approach, task_world, path_world, turn_info)

        # Execute trajectory with turn-in-place handling
        self._execute_trajectory_with_turn_handling(
            path_for_controller, turn_info, config, use_tcp_pose
        )

        # Post-trajectory processing with turn-around
        return self._post_trajectory_update_with_turnaround(env)

    def _draw_enhanced_path_visualization(self, approach, task_world, path_world, turn_info):
        """Enhanced visualization that shows extended trajectories and turn positions."""
        
        # Draw approach (green) and task (blue)
        for i in range(len(approach) - 1):
            p.addUserDebugLine([approach[i][0], approach[i][1], 0.02],
                               [approach[i+1][0], approach[i+1][1], 0.02],
                               [0, 1, 0], lineWidth=3.0)
        
        # Draw task trajectory with special coloring for extended part
        if turn_info['needed']:
            # Draw extended pre-task section in orange
            p.addUserDebugLine([task_world[0][0], task_world[0][1], 0.02],
                               [task_world[1][0], task_world[1][1], 0.02],
                               [1, 0.5, 0], lineWidth=4.0)  # Orange for extension
            
            # Mark turn-in-place position
            turn_pos = turn_info['turn_position']
            p.addUserDebugText("TURN", [turn_pos[0], turn_pos[1], 0.1], 
                              textColorRGB=[1, 0, 0], textSize=2.0, lifeTime=30.0)
            
            # Draw circle around turn position
            for angle in range(0, 360, 30):
                rad = math.radians(angle)
                next_rad = math.radians(angle + 30)
                x1, y1 = turn_pos[0] + 0.1 * math.cos(rad), turn_pos[1] + 0.1 * math.sin(rad)
                x2, y2 = turn_pos[0] + 0.1 * math.cos(next_rad), turn_pos[1] + 0.1 * math.sin(next_rad)
                p.addUserDebugLine([x1, y1, 0.03], [x2, y2, 0.03], [1, 0, 0], lineWidth=2.0, lifeTime=30.0)
            
            # Draw actual collection task in blue (starting from task_world[1])
            for i in range(1, len(task_world) - 1):
                p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                                   [task_world[i+1][0], task_world[i+1][1], 0.02],
                                   [0, 0, 1], lineWidth=2.0)
        else:
            # Standard task visualization
            for i in range(len(task_world) - 1):
                p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                                   [task_world[i+1][0], task_world[i+1][1], 0.02],
                                   [0, 0, 1], lineWidth=2.0)

        # Draw resampled path (black) to verify spacing
        for i in range(len(path_world) - 1):
            p.addUserDebugLine([path_world[i][0], path_world[i][1], 0.03],
                               [path_world[i+1][0], path_world[i+1][1], 0.03],
                               [0, 0, 0], lineWidth=1.0)

    def _execute_trajectory_with_turn_handling(self, path_world, turn_info, config, use_tcp_pose):
        """Execute trajectory with intelligent turn-in-place handling."""
        
        if turn_info['needed']:
            print(f"🔄 Executing enhanced trajectory with turn-in-place at extended position")
            print(f"  📍 Turn position: {turn_info['turn_position']}")
            print(f"  📐 Target angle: {math.degrees(turn_info['target_angle']):.1f}°")
            
            # Execute approach to extended position (no turn yet)
            self.follow_trajectory_pure_pursuit(
                path_world,
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
            
            # At extended position - perform turn-in-place
            print(f"🔄 Performing turn-in-place at extended position...")
            self._turn_in_place(
                turn_info['target_angle'], 
                max_rate=config['pivot_rate'], 
                tol=0.05
            )
            
            print(f"✅ Turn-in-place completed - rover ready for collection task")
            
        else:
            # Standard trajectory execution for moderate turns
            print(f"➡️ Executing standard trajectory with fillet smoothing")
            self.follow_trajectory_pure_pursuit(
                path_world,
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

    def follow_trajectory_pure_pursuit(
            self,
            path_world,  # list[(x,y,theta)] already resampled
            *,
            dt=1 / 240,
            lookahead=0.15,  # ↑ bit larger default
            v_nom=0.35,  # comfortable base speed
            a_lat_max=1.0,  # cap lateral accel → calmer in bends
            yaw_slew=4.0,  # ↓ slower yaw slew (rad/s^2)
            use_tcp_pose=True,
            tcp_fwd=0.12,
            tcp_lat=0.00,
            draw_tool_tick=True
    ):
        """
        Pure-Pursuit++: curvature-aware lookahead + speed cap + feedforward curvature + Stanley cross-track.
        """
        print(f"🚗 Pure Pursuit++ (enhanced): N={len(path_world)}, Ld_nom~{lookahead:.2f}, mode={'TCP' if use_tcp_pose else 'BASE'}")
        
        # ---------- prep polyline ----------
        if len(path_world) < 2:
            return

        pts = np.array([(p[0], p[1]) for p in path_world], float)
        seg = pts[1:] - pts[:-1]
        seg_len = np.hypot(seg[:,0], seg[:,1])
        s_cum = np.concatenate([[0.0], np.cumsum(seg_len)])
        total = s_cum[-1]
        if total < 1e-6:
            return

        # helper closures
        def _point_at_s(ss):
            return point_at_s([(p[0], p[1]) for p in pts], s_cum, seg_len, ss)

        def _curv_at_s(ss):
            return _curvature_at_s([(p[0], p[1]) for p in pts], s_cum, seg_len, ss, delta=0.08)

        # controller constants (mild defaults; tweak if needed)
        L_MIN = max(0.5 * np.mean(np.diff(s_cum)) if len(s_cum)>1 else 0.03, 0.04)  # allow small gaze
        L_MAX = max(lookahead, 0.10)
        K_LD  = 0.35         # how aggressively curvature shrinks Ld
        K_HD  = 1.8          # heading error gain
        K_EY  = 1.2          # Stanley cross-track gain
        V_FLOOR = 0.10       # don't crawl too much
        V_EPS   = 0.05       # prevent division by zero in Stanley
        STOP_DIST = 0.03     # stop when within 3cm of path end
        T_SETTLE  = 0.5

        # slew state and tracking variables
        last_yaw_cmd = 0.0
        s_prev = 0.0  # Initialize progress tracking
        Ld_prev = L_MAX  # Initialize lookahead
        ey_prev = 0.0  # Initialize cross-track error
        
        # Missing constants from old algorithm (compatibility)
        K_GAIN = 0.35  # how strongly curvature shrinks Ld
        Ld_alpha = 0.6  # LPF on lookahead (0..1) higher=smoother
        kappa_max = 3.0  # clamp commanded curvature
        ey_deadband = 0.010  # ignore tiny cross-track errors
        phi_smooth = 0.35  # extra low-pass on yaw-rate command
        
        # Additional tracking variables
        phi_prev = 0.0
        max_dphi = yaw_slew * dt
        t_elapsed = 0.0
        total_len = total  # Alias for compatibility

        t_settle = 0.0
        while True:
            # Base pose
            if hasattr(self, "pose"):
                (x_b, y_b), th = self.pose()
            else:
                pos, quat = p.getBasePositionAndOrientation(self.object_ids[1])
                x_b, y_b = pos[0], pos[1]
                th = p.getEulerFromQuaternion(quat)[2]

            # Use TOOL or BASE as control origin
            if use_tcp_pose:
                x_r, y_r, _ = tool_pose_from_base(x_b, y_b, th, tcp_fwd, tcp_lat)
                if draw_tool_tick:
                    p.addUserDebugLine([x_r, y_r, 0.01], [x_r, y_r, 0.05], [0, 1, 0], 2, lifeTime=0.2)
            else:
                x_r, y_r = x_b, y_b

            # Project origin to path (monotonic arclength to avoid jumps)
            s_here, _ = project_to_polyline(pts, s_cum, seg_len, (x_r, y_r))
            s_here = max(s_prev, s_here)  # never go backward
            if total_len - s_here < 0.05:
                break

            # Curvature-adaptive lookahead (smoothed)
            k_geom = abs(_curvature_at_s(pts, s_cum, seg_len, s_here, delta=max(0.5 * Ld_prev, 0.04)))
            Ld_raw = np.clip(lookahead / (1.0 + K_GAIN * k_geom), L_MIN, L_MAX)
            Ld = Ld_alpha * Ld_prev + (1.0 - Ld_alpha) * Ld_raw

            # Target point at s_here + Ld
            s_tgt = s_here + Ld
            tx, ty = point_at_s([(p[0], p[1]) for p in path_world], s_cum, seg_len, s_tgt)

            # Body-frame errors from control origin
            dx, dy = tx - x_r, ty - y_r
            ex = math.cos(th) * dx + math.sin(th) * dy
            ey = -math.sin(th) * dx + math.cos(th) * dy

            # Tiny cross-track deadband & zero-crossing damping
            if abs(ey) < ey_deadband:
                ey = 0.0
            elif ey_prev * ey < 0 and abs(ey) < 3 * ey_deadband:
                ey = 0.0  # suppress ping-pong when crossing the line
            ey_prev = ey

            # Pure-pursuit curvature and speed schedule
            kappa = 2.0 * ey / max(Ld * Ld, 1e-9)
            kappa = float(np.clip(kappa, -kappa_max, kappa_max))

            if abs(kappa) > 1e-6:
                v_curve = math.sqrt(max(0.05, a_lat_max / abs(kappa)))
                V = min(v_nom, v_curve)
            else:
                V = v_nom

            # Slow near the end
            if total_len - s_here < 0.25:
                V = min(V, 0.25)

            # Desired yaw-rate, then slew-limit and low-pass it
            phi_cmd = V * kappa
            dphi = phi_cmd - phi_prev
            dphi = max(-max_dphi, min(max_dphi, dphi))
            phi_step = phi_prev + dphi
            phi = (1.0 - phi_smooth) * phi_prev + phi_smooth * phi_step

            # Drive
            self.set_wheel_speeds_unitsafe(V, phi, dt)

            # Update state
            phi_prev = phi
            Ld_prev = Ld
            s_prev = s_here
            t_elapsed += dt

        print("🏁 Pure Pursuit++ done – stopping robot…")
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

    def execute_simple_trajectory(self, env, mode="TCP", **kwargs):
        """
        Simple trajectory execution:
        1. Go directly to starting point of task trajectory  
        2. Turn in place to correct orientation
        3. Execute task trajectory with Pure Pursuit
        
        Args:
            env: Environment object with current_trajectory
            mode: "TCP" or "BASE_SHIFT"
            **kwargs: Override default parameters
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
            'turn_threshold_deg': 15.0,
            'turn_rate': 2.5,
            'draw_tool_tick': True
        }
        config.update(kwargs)

        print(f"\n🚗 Simple trajectory execution (mode: {mode})...")

        # Build simple world path (approach + task) 
        approach, task_world, path_world, turn_info = self.build_simple_world_path(
            env, 
            ds=config['resample_ds'],
            turn_in_place_threshold_deg=config['turn_threshold_deg']
        )

        print(f"Approach: {len(approach)} pts, Task: {len(task_world)} pts, Path: {len(path_world)} pts")

        # Phase 1: Execute approach to starting point
        if len(approach) > 1:
            print("🚗 Phase 1: Moving to task starting point...")
            
            # Handle different modes for approach
            if mode.upper() == "BASE_SHIFT":
                approach_shifted = shift_path_along_s(approach, s_shift=-config['tcp_fwd'])
                approach_for_controller = approach_shifted
                use_tcp_pose = False
            else:
                approach_for_controller = approach
                use_tcp_pose = True

            # Visualization for approach
            for i in range(len(approach) - 1):
                p.addUserDebugLine([approach[i][0], approach[i][1], 0.02],
                                   [approach[i+1][0], approach[i+1][1], 0.02],
                                   [0, 1, 0], lineWidth=3.0)  # Green for approach

            # Follow approach path
            self.follow_trajectory_pure_pursuit(
                approach_for_controller,
                dt=1/240,
                lookahead=config['lookahead'],
                v_nom=config['v_nom'],
                a_lat_max=config['a_lat_max'],
                yaw_slew=config['yaw_slew_rate'],
                use_tcp_pose=use_tcp_pose,
                tcp_fwd=config['tcp_fwd'],
                tcp_lat=config['tcp_lat'],
                draw_tool_tick=False  # No tool tick for approach
            )
            
            print("✅ Phase 1 complete: Reached task starting point")

        # Phase 2: Turn in place if needed
        if turn_info['turn_needed']:
            print(f"🔄 Phase 2: Turning in place to align with task heading...")
            
            # Get current heading (kept for fallback/logging)
            robot_id = self.object_ids[1]
            _, quat = p.getBasePositionAndOrientation(robot_id)
            current_heading = p.getEulerFromQuaternion(quat)[2]
            
            # Prefer the absolute task heading we computed earlier
            if 'target_heading' in turn_info:
                target_heading = turn_info['target_heading']
            elif len(task_world) >= 2:
                # Fallback: derive from first segment of task
                dx = task_world[1][0] - task_world[0][0]
                dy = task_world[1][1] - task_world[0][1]
                target_heading = math.atan2(dy, dx)
            else:
                # Last resort: use old method
                target_heading = current_heading + turn_info.get('turn_angle', 0.0)
            
            print(f"  📍 Current heading: {math.degrees(current_heading):.1f}°")
            print(f"  🎯 Target heading: {math.degrees(target_heading):.1f}°") 
            print(f"  🔄 Turn needed: {math.degrees(wrap_angle(target_heading - current_heading)):.1f}°")
            
            self._turn_in_place(target_heading, max_rate=config['turn_rate'])
            print("✅ Phase 2 complete: Oriented for task execution")
        else:
            print("⏭️ Phase 2 skipped: No significant turn needed")

        # Phase 3: Execute task trajectory  
        print("🚗 Phase 3: Executing task trajectory...")
        
        # Handle different modes for task
        if mode.upper() == "BASE_SHIFT":
            task_shifted = shift_path_along_s(task_world, s_shift=-config['tcp_fwd'])
            task_for_controller = task_shifted
            use_tcp_pose = False
        else:
            task_for_controller = task_world
            use_tcp_pose = True

        # Visualization for task
        for i in range(len(task_world) - 1):
            p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                               [task_world[i+1][0], task_world[i+1][1], 0.02],
                               [0, 0, 1], lineWidth=2.0)  # Blue for task

        # Follow task path
        self.follow_trajectory_pure_pursuit(
            task_for_controller,
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
        
        print("✅ Phase 3 complete: Task trajectory executed")
        print("🏁 Simple trajectory execution complete!")

        # Brief settling period after trajectory completion
        print("🛑 Brief settling after trajectory completion...")
        for _ in range(30):  # Brief stop for 30 steps (0.125 seconds)
            self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)

        # Note: Post-trajectory processing (including turn-around) is handled by caller
        print("📋 Simple trajectory complete - awaiting post-processing instructions...")

    def execute_alignment_gate_pivot_trajectory(self, env, mode="TCP", **kwargs):
        """
        Alignment gate approach:
        - Approach from behind S0 
        - When aligned (dot>=align_dot) & behind & in distance band, pivot early to path heading
        - Then follow the task path normally
        """
        cfg = {
            'tcp_fwd': 0.12, 'tcp_lat': 0.00,
            'resample_ds': 0.05,
            'v_nom': 0.35, 'a_lat_max': 1.0, 'yaw_slew_rate': 6.0,
            'lookahead': 0.12,
            'pivot_rate': 8.0,
            # alignment gate
            'align_dot': 0.93,        # cos(theta) >= 0.93  -> theta <= ~21°
            'gate_back': 0.35,        # place gate 35 cm behind S0 along -v_path
            'dist_band_min': 0.20,    # only pivot if distance to S0 is within [min,max]
            'dist_band_max': 0.60,
            'draw_tool_tick': True,
        }
        cfg.update(kwargs or {})

        g = env.current_trajectory
        if not g or len(g) < 2:
            print("⚠️ No task trajectory.")
            return

        print(f"\n🎯 Alignment Gate Pivot Trajectory Execution (mode: {mode})...")
        print(f"  📊 Gate parameters: align_dot={cfg['align_dot']:.2f}, gate_back={cfg['gate_back']:.2f}m")
        print(f"  📏 Distance band: {cfg['dist_band_min']:.2f}m - {cfg['dist_band_max']:.2f}m")

        # Build smooth world task (no approach blending)
        task_world = self._create_smooth_task_trajectory(g)
        S0 = (task_world[0][0], task_world[0][1])
        S1 = (task_world[1][0], task_world[1][1])

        # path heading
        th_task = math.atan2(S1[1] - S0[1], S1[0] - S0[0])

        # path unit vector and gate point behind S0
        vpx, vpy = (S1[0] - S0[0]), (S1[1] - S0[1])
        nrm = math.hypot(vpx, vpy) + 1e-9
        vpx /= nrm; vpy /= nrm
        G = (S0[0] - cfg['gate_back'] * vpx, S0[1] - cfg['gate_back'] * vpy)

        print(f"  🎯 S0 (task start): ({S0[0]:.3f}, {S0[1]:.3f})")
        print(f"  🚪 Gate point G: ({G[0]:.3f}, {G[1]:.3f})")
        print(f"  📐 Task heading: {math.degrees(th_task):.1f}°")

        # Debug visualization lines
        try:
            p.addUserDebugLine([S0[0], S0[1], 0.03], [S1[0], S1[1], 0.03], [0,0,1], lineWidth=2)  # Blue: path direction
            p.addUserDebugLine([S0[0], S0[1], 0.03], [G[0],  G[1],  0.03], [1,0,0], lineWidth=2)  # Red: gate line
            p.addUserDebugText("S0", [S0[0], S0[1], 0.05], [0,0,0], textSize=1.0)
            p.addUserDebugText("G", [G[0], G[1], 0.05], [1,0,0], textSize=1.0)
        except Exception:
            pass

        use_tcp = (mode.upper() != "BASE_SHIFT")

        print("🚗 Phase A1: Moving base to G point...")

        # Phase A1: Navigate BASE LINK to G point (no alignment checking yet)
        _goto_point(self, G, 
                   use_tcp_pose=False,  # Use BASE for navigation to G
                   tcp_fwd=cfg['tcp_fwd'], tcp_lat=cfg['tcp_lat'],
                   v_nom=cfg['v_nom'] * 0.8,  # Slightly slower for precision
                   tol=0.05, max_time=10.0)

        # Verify we reached G
        robot_id = self.object_ids[1]
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        x_b, y_b = pos[0], pos[1]
        dist_to_G = math.hypot(G[0] - x_b, G[1] - y_b)
        print(f"✅ Phase A1 complete: Base at ({x_b:.3f}, {y_b:.3f}), distance to G: {dist_to_G:.3f}m")

        print("🚗 Phase A2: Monitoring alignment for pivot trigger...")

        # Phase A2: Move from G towards S0 while monitoring alignment gate
        dt = 1/240; t = 0.0; max_time = 8.0; e_prev = 0.0

        while t < max_time:
            # Get current robot pose (base link)
            pos, quat = p.getBasePositionAndOrientation(robot_id)
            th = p.getEulerFromQuaternion(quat)[2]
            x_b, y_b = pos[0], pos[1]
            
            # Check alignment using TCP pose (for gate triggering)
            dot_align, behind, dist = _alignment_metrics(
                self, S0, S1,
                use_tcp_pose=True, tcp_fwd=cfg['tcp_fwd'], tcp_lat=cfg['tcp_lat']  # Always use TCP for alignment check
            )
            
            # Check alignment gate condition (based on TCP)
            if (dot_align >= cfg['align_dot'] and behind and
                cfg['dist_band_min'] <= dist <= cfg['dist_band_max']):
                print(f"✅ Alignment gate triggered!")
                print(f"  📐 TCP Alignment: {dot_align:.3f} (≥{cfg['align_dot']:.3f}) ✓")
                print(f"  📏 TCP Distance: {dist:.3f}m ({cfg['dist_band_min']:.2f}-{cfg['dist_band_max']:.2f}m) ✓")
                print(f"  ⬅️ TCP Behind S0: {behind} ✓")
                break

            # Navigate base towards S0
            hdg = math.atan2(S0[1] - y_b, S0[0] - x_b)  # Base to S0 heading
            e = wrap_angle(hdg - th)

            V = cfg['v_nom'] * float(np.clip(dist / 0.35, 0.3, 1.0))
            if abs(e) > math.radians(45): 
                V = min(V, 0.18)
            
            k_yaw = 3.0
            yaw_cmd = float(np.clip(k_yaw * e + 2.0 * (e - e_prev) / dt,
                                    -cfg['yaw_slew_rate'], cfg['yaw_slew_rate']))

            self.set_wheel_speeds_unitsafe(V, yaw_cmd, dt)
            e_prev = e; t += dt

        self._stop_robot_smoothly()

        # Phase B: pivot to face S0 (not the S0->S1 direction!)
        robot_id = self.object_ids[1]
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        x_b, y_b = pos[0], pos[1]
        
        # Calculate heading from current base position to S0
        th_to_S0 = math.atan2(S0[1] - y_b, S0[0] - x_b)
        
        print(f"🔄 Phase B: Early pivot to face S0...")
        print(f"  📍 Base position: ({x_b:.3f}, {y_b:.3f})")
        print(f"  🎯 S0 position: ({S0[0]:.3f}, {S0[1]:.3f})")
        print(f"  📐 Turn to heading: {math.degrees(th_to_S0):.1f}° (facing S0)")
        
        self._turn_in_place(th_to_S0, max_rate=cfg['pivot_rate'], tol=0.03)
        print("✅ Phase B complete: Now facing S0 for approach")

        # Phase C: follow the task path
        print("🚗 Phase C: Following task trajectory...")
        
        if mode.upper() == "BASE_SHIFT":
            path_for_controller = shift_path_along_s(task_world, s_shift=-cfg['tcp_fwd'])
            use_tcp_pose = False
            # Visualize shifted path
            for i in range(len(path_for_controller) - 1):
                p.addUserDebugLine([path_for_controller[i][0], path_for_controller[i][1], 0.025],
                                   [path_for_controller[i+1][0], path_for_controller[i+1][1], 0.025],
                                   [0.6, 0.0, 0.6], lineWidth=2.0)  # Purple for shifted
        else:
            path_for_controller = task_world
            use_tcp_pose = True

        # Visualize task trajectory
        for i in range(len(task_world) - 1):
            p.addUserDebugLine([task_world[i][0], task_world[i][1], 0.02],
                               [task_world[i+1][0], task_world[i+1][1], 0.02],
                               [0, 0, 1], lineWidth=2.0)  # Blue for task

        self.follow_trajectory_pure_pursuit(
            path_for_controller,
            dt=1/240,
            lookahead=cfg['lookahead'],
            v_nom=cfg['v_nom'],
            a_lat_max=cfg['a_lat_max'],
            yaw_slew=cfg['yaw_slew_rate'],
            use_tcp_pose=use_tcp_pose,
            tcp_fwd=cfg['tcp_fwd'],
            tcp_lat=cfg['tcp_lat'],
            draw_tool_tick=cfg['draw_tool_tick']
        )
        
        print("✅ Phase C complete: Task trajectory executed")
        print("🏁 Alignment gate pivot trajectory execution complete!")

        # Brief settling period after trajectory completion
        print("🛑 Brief settling after trajectory completion...")
        for _ in range(30):  # Brief stop for 30 steps (0.125 seconds)
            self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)

        # Note: Post-trajectory processing (including turn-around) is handled by caller
        print("📋 Alignment gate trajectory complete - awaiting post-processing instructions...")

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

    def _post_trajectory_update_with_turnaround(self, env):
        """
        Enhanced post-trajectory processing that includes turn-around functionality.
        
        After completing a trajectory that delivers objects to the target zone:
        1. Execute backup movement to avoid pushing objects out of target
        2. Perform 180° turn-in-place to face away from target zone
        3. Standard post-trajectory processing (object position updates)
        """
        print("\n🔄 Post-trajectory processing with turn-around...")
        
        # Check if this trajectory involved target zone delivery
        trajectory = env.current_trajectory if hasattr(env, 'current_trajectory') else []
        print(f"  📍 Trajectory info: {len(trajectory)} waypoints" if trajectory else "  ⚠️ No trajectory found!")
        
        if trajectory:
            print(f"    Start: {trajectory[0]} → End: {trajectory[-1]}")
        
        target_zone_delivery = self._check_if_target_zone_delivery(trajectory, env)
        print(f"  🎯 Target zone delivery detection: {'YES' if target_zone_delivery else 'NO'}")
        
        if target_zone_delivery:
            print("🎯 Target zone delivery detected - performing turn-around sequence")
            try:
                self._execute_post_delivery_turnaround()
                print("✅ Turn-around sequence completed successfully")
            except Exception as e:
                print(f"❌ Turn-around sequence failed: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("📦 Collection trajectory - no turn-around needed")
        
        # Now perform standard post-trajectory processing
        return self._post_trajectory_update(env)

    def _check_if_target_zone_delivery(self, trajectory, env):
        """
        Check if the completed trajectory was delivering objects to target zone.
        This checks if the final destination is within the target zone.
        """
        print(f"    🔍 Checking target zone delivery...")
        
        if not trajectory or len(trajectory) < 1:
            print(f"    ❌ No trajectory to check")
            return False
            
        # Get the final destination in the trajectory
        final_grid = trajectory[-1]
        print(f"    📍 Final grid position: {final_grid}")
        
        try:
            final_world_x, final_world_y = self.coord_converter.convert_2d_to_3d(*final_grid)
        except Exception as e:
            print(f"    ❌ Failed to convert grid to world: {e}")
            return False
        
        # Check distance to target zone center (0, 0)
        distance_to_target = math.hypot(final_world_x, final_world_y)
        
        # Consider it target delivery if within target zone radius + small margin
        target_zone_radius = getattr(env, 'target_zone_radius', 0.3)
        threshold = target_zone_radius + 0.1  # Add 10cm margin
        is_target_delivery = distance_to_target <= threshold
        
        print(f"    📍 Final world position: ({final_world_x:.3f}, {final_world_y:.3f})")
        print(f"    📏 Distance to target center: {distance_to_target:.3f}m")
        print(f"    🎯 Target zone radius: {target_zone_radius:.3f}m")
        print(f"    📐 Threshold (with margin): {threshold:.3f}m")
        print(f"    ✅ Verdict: {'TARGET DELIVERY' if is_target_delivery else 'COLLECTION TASK'}")
            
        return is_target_delivery

    def _execute_post_delivery_turnaround(self):
        """
        Execute the post-delivery turn-around sequence:
        1. Stop rover completely at current position
        2. Calculate direction away from target center (0, 0) 
        3. Use existing _turn_in_place function to rotate precisely
        4. Ensure rover stops completely after turn
        """
        print("🔄 Executing post-delivery turn-around sequence...")
        
        # STEP 1: Stop the rover completely and settle physics
        print("  🛑 Stopping rover completely...")
        for _ in range(60):  # Stop for 60 steps (0.25 seconds)
            self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)
        
        # Let physics settle briefly
        for _ in range(30):
            p.stepSimulation()
            time.sleep(1/240)
        
        # STEP 2: Get ACTUAL current position and heading from rover
        (x_current, y_current), theta_current = self.pose()
        print(f"  📍 Current position: ({x_current:.3f}, {y_current:.3f})")
        print(f"  🧭 Current heading: {math.degrees(theta_current):.1f}°")
        
        # STEP 3: Calculate direction FROM current position TO target center (0, 0)
        direction_to_target = math.atan2(-y_current, -x_current)
        print(f"  🎯 Direction to target center: {math.degrees(direction_to_target):.1f}°")
        
        # Target heading should be OPPOSITE direction (away from target center)
        target_theta = wrap_angle(direction_to_target + math.pi)
        print(f"  🔄 Target outward heading: {math.degrees(target_theta):.1f}°")
        
        # Calculate how much we need to turn
        turn_needed = wrap_angle(target_theta - theta_current)
        print(f"  ↻ Turn needed: {math.degrees(turn_needed):.1f}°")
        
        # STEP 4: Use the existing _turn_in_place function (just like in test_turn_in_place.py)
        print(f"🔄 Executing turn-in-place to target angle...")
        try:
            self._turn_in_place(
                target_theta,        # Target angle to face away from target center
                max_rate=2.0,        # Same as in test_turn_in_place.py
                tol=0.05            # Same tolerance as test (3° accuracy)
            )
            print("  ✅ Turn-in-place completed successfully")
        except Exception as e:
            print(f"  ❌ Turn-in-place failed: {e}")
        
        # STEP 5: CRITICAL - Stop rover completely after turn to prevent spurious movement
        print("  🛑 Final stop - ensuring no spurious movement...")
        for _ in range(120):  # Stop for 120 steps (0.5 seconds) - longer stop
            self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)
        
        # Additional physics settling to ensure complete stop
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1/240)
        
        # STEP 6: Verify rover is completely stopped and check velocities
        (x_final, y_final), theta_final = self.pose()
        
        # Get current velocities to check if rover is actually stopped
        robot_id = self.object_ids[1]
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        speed = math.hypot(lin_vel[0], lin_vel[1])
        
        print(f"✅ Turn-around completed:")
        print(f"  📍 Final position: ({x_final:.3f}, {y_final:.3f})")
        print(f"  🧭 Final heading: {math.degrees(theta_final):.1f}°")
        print(f"  🏃 Current linear velocity: {speed:.4f} m/s")
        print(f"  🔄 Current angular velocity: {abs(ang_vel[2]):.4f} rad/s")
        
        # Apply emergency braking if rover is still moving
        if speed > 0.01:  # 1cm/s tolerance
            print(f"  ⚠️  WARNING: Rover still moving with speed {speed:.4f} m/s!")
            print("      Applying emergency brake for 2 seconds...")
            # Emergency brake - more aggressive stopping
            for _ in range(480):  # Stop for 480 steps (2 seconds at 240Hz)
                self.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)
            
            # Check velocity again after emergency brake
            lin_vel, ang_vel = p.getBaseVelocity(robot_id)
            final_speed = math.hypot(lin_vel[0], lin_vel[1])
            print(f"      After emergency brake: speed = {final_speed:.4f} m/s")
        
        # Verify the rover is facing away from target
        final_direction_to_target = math.atan2(-y_final, -x_final)
        expected_outward = wrap_angle(final_direction_to_target + math.pi)
        heading_error = abs(wrap_angle(theta_final - expected_outward))
        
        print(f"  🎯 Expected outward heading: {math.degrees(expected_outward):.1f}°")
        print(f"  📐 Heading accuracy: {math.degrees(heading_error):.1f}° error")
        print(f"  🛑 Rover is STOPPED and facing away from target center!")
        print(f"  ⏸️ Ready for next assignment...")
        print()

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

    def _create_smooth_task_trajectory(self, grid_waypoints, extension_config=None):
        """
        Create a smooth B-spline trajectory that passes through object cell centers.
        Optionally extends the path backwards for smoother approach.
        
        Args:
            grid_waypoints: List of (grid_x, grid_y) tuples from 2D path planning
            extension_config: Optional path extension configuration
            
        Returns:
            List of (world_x, world_y, z) tuples for smooth 3D trajectory
        """
        # Apply path extension if configured
        if extension_config:
            extended_waypoints = self._extend_path_backwards(grid_waypoints, extension_config)
        else:
            extended_waypoints = grid_waypoints
            
        if len(extended_waypoints) < 2:
            # Single point - just convert directly
            if len(extended_waypoints) == 1:
                wx, wy = self.coord_converter.convert_2d_to_3d(*extended_waypoints[0])
                return [(wx, wy, 0.0)]
            return []
        
        total_points = len(extended_waypoints)
        original_points = len(grid_waypoints)
        extension_points = total_points - original_points
        
        if extension_points > 0:
            print(f"🛤️ Creating smooth trajectory: {extension_points} extension + {original_points} original = {total_points} total points")
        else:
            print(f"🛤️ Creating smooth B-spline trajectory through {total_points} object cells...")
        
        # Convert grid points to world coordinates (cell centers) - these are waypoints to pass through
        waypoints = []
        for gx, gy in extended_waypoints:
            wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
            waypoints.append([wx, wy])
            
        # Debug: Show waypoints the trajectory will pass through
        for i, (gx, gy) in enumerate(extended_waypoints):
            wx, wy = waypoints[i]
            if i < extension_points:
                print(f"    Extension Point {i+1}: Cell({gx}, {gy}) → World({wx:.3f}, {wy:.3f})")
            else:
                print(f"    Target Waypoint {i-extension_points+1}: Cell({gx}, {gy}) → World({wx:.3f}, {wy:.3f})")
        
        # Create smooth trajectory using cubic spline interpolation at target spacing
        # Use 0.03m (3cm) spacing to match resampling parameter for seamless integration
        smooth_trajectory = self._generate_smooth_spline_path(waypoints, target_spacing=0.03)
        
        print(f"  📍 Generated smooth spline: {len(waypoints)} waypoints → {len(smooth_trajectory)} trajectory points")
        print(f"  🎯 Path: Cell({grid_waypoints[0]}) → Cell({grid_waypoints[-1]})")
            
        return smooth_trajectory
    
    def _generate_smooth_spline_path(self, waypoints, target_spacing=0.03):
        """
        Generate smooth spline path that passes through all waypoints with specified spacing.
        Uses Catmull-Rom spline for smooth interpolation that maintains curves after path building.
        
        Args:
            waypoints: List of [x, y] waypoints to interpolate
            target_spacing: Desired spacing between points (meters) - matches resampling spacing
        """
        import numpy as np
        import math
        
        waypoints = np.array(waypoints)
        
        if len(waypoints) < 2:
            return [(waypoints[0][0], waypoints[0][1], 0.0)]
        
        # Calculate total path length to determine point density
        total_length = 0.0
        for i in range(1, len(waypoints)):
            total_length += math.hypot(waypoints[i][0] - waypoints[i-1][0], 
                                     waypoints[i][1] - waypoints[i-1][1])
        
        if len(waypoints) == 2:
            # Linear interpolation for 2 points at target spacing
            p0, p1 = waypoints[0], waypoints[1]
            num_points = max(2, int(total_length / target_spacing) + 1)
            trajectory = []
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0
                x = p0[0] + t * (p1[0] - p0[0])
                y = p0[1] + t * (p1[1] - p0[1])
                trajectory.append((x, y, 0.0))
            return trajectory
        
        # For 3+ points, use Catmull-Rom spline with adaptive point density
        trajectory = []
        
        # Add extra points at start and end for proper Catmull-Rom behavior
        extended_points = np.zeros((len(waypoints) + 2, 2))
        extended_points[1:-1] = waypoints
        # Extend first and last points
        extended_points[0] = 2 * waypoints[0] - waypoints[1]
        extended_points[-1] = 2 * waypoints[-1] - waypoints[-2]
        
        # Generate smooth curve segments with target spacing
        for i in range(1, len(extended_points) - 2):
            p0, p1, p2, p3 = extended_points[i-1:i+3]
            
            # Estimate segment length for point density
            segment_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            points_this_segment = max(2, int(segment_length / target_spacing))
            
            # Catmull-Rom spline interpolation
            for j in range(points_this_segment):
                t = j / points_this_segment
                t2 = t * t
                t3 = t2 * t
                
                # Catmull-Rom formula
                x = 0.5 * ((2 * p1[0]) + 
                          (-p0[0] + p2[0]) * t +
                          (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                          (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
                
                y = 0.5 * ((2 * p1[1]) + 
                          (-p0[1] + p2[1]) * t +
                          (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                          (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                
                trajectory.append((x, y, 0.0))
        
        # Always end exactly at the final waypoint
        final_point = waypoints[-1]
        trajectory.append((final_point[0], final_point[1], 0.0))
        
        return trajectory

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

    def generate_smooth_approach_trajectory(self, start_pose, end_pose, duration=3.0, step_time=0.1, scale=0.5):
        """
        Generate a smoother cubic Bézier trajectory optimized for reduced curvature.
        Uses advanced control point positioning and curvature management for gentler approaches.
        
        :param start_pose: (x0, y0, theta0)
        :param end_pose: (x1, y1, theta1)
        :param duration: total path duration in seconds  
        :param step_time: time step for sampling
        :param scale: base distance for control points (will be adaptively adjusted)
        :return: list of (x, y, heading) waypoints with reduced curvature
        """
        x0, y0, theta0 = start_pose
        x3, y3, theta3 = end_pose
        
        # Calculate total distance for adaptive scaling
        total_dist = math.hypot(x3 - x0, y3 - y0)
        
        # Enhanced control point calculation for smoother curves
        p0 = np.array([x0, y0])
        p3 = np.array([x3, y3])
        
        # Adaptive scale based on distance and angular difference
        angle_diff = abs(wrap_angle(theta3 - theta0))
        
        # For sharp angular differences, use longer control arms to create smoother curves
        adaptive_scale_start = scale * (1.0 + 0.5 * min(angle_diff / math.pi, 1.0))
        adaptive_scale_end = scale * (1.0 + 0.3 * min(angle_diff / math.pi, 1.0))
        
        # Scale control arms based on total distance (longer distances = longer arms)
        dist_factor = min(total_dist / 2.0, 1.0)  # Cap at reasonable level
        adaptive_scale_start *= (0.5 + 0.5 * dist_factor)
        adaptive_scale_end *= (0.5 + 0.5 * dist_factor)
        
        # Enhanced control points with intermediate direction consideration
        start_dir = np.array([np.cos(theta0), np.sin(theta0)])
        end_dir = np.array([np.cos(theta3), np.sin(theta3)])
        
        # Create intermediate target that helps reduce sharp curvature
        midpoint = (p0 + p3) / 2
        to_mid_from_start = midpoint - p0
        to_mid_from_end = midpoint - p3
        
        # Adjust control points to naturally guide toward intermediate direction
        p1 = p0 + adaptive_scale_start * start_dir
        p2 = p3 - adaptive_scale_end * end_dir
        
        # Apply gentle mid-point influence to reduce maximum curvature
        mid_influence = 0.15  # Small influence to maintain start/end directions
        p1 += mid_influence * to_mid_from_start / max(np.linalg.norm(to_mid_from_start), 0.1)
        p2 += mid_influence * to_mid_from_end / max(np.linalg.norm(to_mid_from_end), 0.1)
        
        num_steps = int(duration / step_time)
        trajectory = []
        
        # Generate trajectory with curvature monitoring
        max_curvature = 0.0
        
        for i in range(num_steps):
            t = i / (num_steps - 1) if num_steps > 1 else 0
            
            # Smooth velocity profile - slower at endpoints for better curvature control
            # Use cosine-based velocity profile for natural acceleration/deceleration
            velocity_factor = 0.5 * (1 - math.cos(math.pi * t))
            
            # Cubic Bézier formula with smooth parameterization
            point = (
                (1 - t) ** 3 * p0 +
                3 * (1 - t) ** 2 * t * p1 +
                3 * (1 - t) * t ** 2 * p2 +
                t ** 3 * p3
            )
            
            # First derivative (velocity)
            dp_dt = (
                3 * (1 - t) ** 2 * (p1 - p0) +
                6 * (1 - t) * t * (p2 - p1) +
                3 * t ** 2 * (p3 - p2)
            )
            
            # Second derivative (acceleration) for curvature calculation
            d2p_dt2 = (
                6 * (1 - t) * (p2 - 2 * p1 + p0) +
                6 * t * (p3 - 2 * p2 + p1)
            )
            
            # Calculate heading from velocity direction
            heading = np.arctan2(dp_dt[1], dp_dt[0])
            
            # Monitor curvature for quality assessment
            speed = np.linalg.norm(dp_dt)
            if speed > 1e-6:  # Avoid division by zero
                curvature = abs(np.cross(dp_dt, d2p_dt2)) / (speed ** 3)
                max_curvature = max(max_curvature, curvature)
            
            trajectory.append((point[0], point[1], heading))
        
        print(f"    🌊 Smooth trajectory generated: {len(trajectory)} points, max_curvature={max_curvature:.3f}")
        print(f"    📊 Control scales: start={adaptive_scale_start:.2f}, end={adaptive_scale_end:.2f}")
        
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