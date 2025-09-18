# orchestrator_pure_pursuit.py  —  TCP-aware Pure Pursuit (TCP tracking + base path pre-offset)
# ---------------------------------------------------------------------------------------------
from main import run_2d_env
import pygame
import math
import pybullet as p
from pybullet_integration import PyBulletIntegration
import numpy as np

# ===================== CONFIG =====================
# Tool Center Point (shovel) offset in the robot BODY frame (meters)
TCP_FWD = 0.12   # forward offset (+X, from base origin to shovel center)
TCP_LAT = 0.00   # lateral offset (+Y left). Set small +/- if shovel is off-center.

# Choose control mode:
#   "TCP"        – track the original path using the tool (shovel) pose (A)
#   "BASE_SHIFT" – shift the path backward by TCP_FWD; base follows shifted path (B)
MODE = "TCP"  # or "BASE_SHIFT"

# Resample spacing and controller defaults
RESAMPLE_DS   = 0.06   # ~6 cm path re-sample for stable lookahead
LOOKAHEAD     = 0.12   # 12 cm lookahead along arc-length
V_NOM         = 0.20   # nominal forward speed (m/s)
A_LAT_MAX     = 0.70   # lateral acceleration cap (m/s^2) for cornering speed
YAW_SLEW_RATE = 6.0    # rad/s/sec limit to remove yaw pulses

# Draw the tool point tick as a sanity marker
DRAW_TOOL_TICK = True


# ================== PATH UTILITIES =================

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

    def point_at_s(ss):
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
        x, y = point_at_s(s_target)
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


def build_world_path(integration, env, approach_duration=2.0, approach_scale=0.3, ds=RESAMPLE_DS):
    """
    World path builder:
      1) Bezier approach from current pose to first grid cell
      2) 2D grid trajectory converted to world
      3) prune duplicates at the splice and resample uniformly
    """
    # current robot pose
    robot_id = integration.object_ids[1]  # plane[0], robot[1] in your setup
    pos, quat = p.getBasePositionAndOrientation(robot_id)
    theta = p.getEulerFromQuaternion(quat)[2]
    current_pose = (pos[0], pos[1], theta)

    # first task heading from first two grid cells
    start_grid = env.current_trajectory[0]
    next_grid = env.current_trajectory[1] if len(env.current_trajectory) > 1 else start_grid
    start_world = integration.coord_converter.convert_2d_to_3d(*start_grid)
    next_world = integration.coord_converter.convert_2d_to_3d(*next_grid)
    target_theta = math.atan2(next_world[1] - start_world[1], next_world[0] - start_world[0])
    start_pose = (start_world[0], start_world[1], target_theta)

    # approach
    approach = integration.generate_bezier_trajectory(current_pose, start_pose,
                                                      duration=approach_duration,
                                                      scale=approach_scale)
    # task → world
    task_world = []
    for gx, gy in env.current_trajectory:
        wx, wy = integration.coord_converter.convert_2d_to_3d(gx, gy)
        task_world.append((wx, wy, 0.0))

    # splice → prune → resample
    raw = prune_close(approach + task_world, min_dist=0.003)
    path = resample_polyline(raw, ds=ds)
    return approach, task_world, path


# ============== Controller utilities (API-agnostic) ==============

def set_wheel_speeds_unitsafe(integration, V, yaw_rate, dt=1/240):
    """
    Drive wheels with correct units no matter which PyBulletIntegration flavor:
    - old: compute_wheel_velocities(V, phi) + control_rover_velocity(omega_l, omega_r, dt)
    - new: _wheel_speeds(V, phi) + _drive(wl, wr, dt)
    """
    if hasattr(integration, "compute_wheel_velocities"):
        omega_r, omega_l = integration.compute_wheel_velocities(V, yaw_rate)
        if hasattr(integration, "control_rover_velocity"):
            integration.control_rover_velocity(omega_l, omega_r, dt)
        elif hasattr(integration, "_drive"):
            integration._drive(omega_l, omega_r, dt)
        else:
            raise RuntimeError("No wheel drive method found.")
    elif hasattr(integration, "_wheel_speeds") and hasattr(integration, "_drive"):
        wl, wr = integration._wheel_speeds(V, yaw_rate)
        integration._drive(wl, wr, dt)
    else:
        raise RuntimeError("Unsupported PyBulletIntegration API.")


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ===================== Pure Pursuit (arc-length) =====================

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


def follow_trajectory_pure_pursuit(
    integration,
    path_world,             # list[(x,y,theta)] already resampled
    *,
    dt=1/240,
    lookahead=LOOKAHEAD,
    v_nom=V_NOM,
    a_lat_max=A_LAT_MAX,
    yaw_slew=YAW_SLEW_RATE,
    use_tcp_pose=True,      # True: track with TOOL (A). False: base (for base-shift mode).
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
        if hasattr(integration, "pose"):
            (x_b, y_b), th = integration.pose()
        else:
            pos, quat = p.getBasePositionAndOrientation(integration.object_ids[1])
            x_b, y_b = pos[0], pos[1]
            th = p.getEulerFromQuaternion(quat)[2]

        # Use TOOL or BASE for projection and body-frame errors
        if use_tcp_pose:
            x_o, y_o, _ = tool_pose_from_base(x_b, y_b, th, TCP_FWD, TCP_LAT)
            if DRAW_TOOL_TICK:
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
        set_wheel_speeds_unitsafe(integration, V, phi, dt)
        t_elapsed += dt


# ====================== Event / main loop wiring ======================

def handle_2d_events(visualizer, env, integration):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            clicked_cell = visualizer.handle_click_event(pos)
            if clicked_cell:
                print(f"\nClicked cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects.")

                choice = "target"
                trajectory = env.get_trajectory(clicked_cell, choice, True)

                env.current_trajectory = trajectory
                env.current_cell = clicked_cell
                env.current_path_type = choice
                env.current_use_spillage = True

                print("\nVisualizing trajectory...")
                visualizer.set_trajectory(trajectory)
                integration.visualize_trajectory(trajectory)

                print("\nExecuting trajectory with PURE PURSUIT controller...")

                # Build world path (approach + task) and resample
                approach, task_world, path_world = build_world_path(
                    integration, env, approach_duration=2.0, approach_scale=0.3, ds=RESAMPLE_DS
                )
                print(f"Approach: {len(approach)} pts, Task: {len(task_world)} pts, Path: {len(path_world)} pts")

                # If using BASE_SHIFT mode, shift the path backward by TCP_FWD so base can track it
                if MODE.upper() == "BASE_SHIFT":
                    base_path = shift_path_along_s(path_world, s_shift=-TCP_FWD)  # negative to go "back"
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

                # Follow using Pure Pursuit (arc-length lookahead)
                follow_trajectory_pure_pursuit(
                    integration, path_for_controller,
                    dt=1/240,
                    lookahead=LOOKAHEAD,
                    v_nom=V_NOM,
                    a_lat_max=A_LAT_MAX,
                    yaw_slew=YAW_SLEW_RATE,
                    use_tcp_pose=use_tcp_pose
                )

                # Optional: execute the 2D path action (your original logic)
                env.execute_path(env.current_cell, env.current_path_type, use_spillage=env.current_use_spillage)
                env.update_environment()

                integration.clear_trajectory()
                visualizer.clear_trajectory()

                env.current_trajectory = None
                env.current_cell = None
                env.current_path_type = None
                env.current_use_spillage = None

    return True


if __name__ == "__main__":
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 40
    random_seed = 10
    initial_robot_pose = (0.0, -2.0, math.pi / 2)  # keep inside arena

    integration = PyBulletIntegration(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        num_pebbles=num_pebbles,
        random_seed=random_seed,
        gui=True,
        initial_robot_pose=initial_robot_pose
    )

    if hasattr(integration, "set_robot_dim"):
        integration.set_robot_dim(L=0.2, R=0.07)

    print("\nPress 'c' in the PyBullet window to continue to 2D visualization...")
    objects_3d = integration.run()
    shovel_width = integration.shovel_width

    env, visualizer = run_2d_env(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width,
        real_objects=objects_3d,
        manual_mode=False
    )

    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))
        visualizer.draw_elements()
        pygame.display.flip()
        visualizer.clock.tick(30)
        running = handle_2d_events(visualizer, env, integration)

    pygame.quit()
    integration.close_environment()
