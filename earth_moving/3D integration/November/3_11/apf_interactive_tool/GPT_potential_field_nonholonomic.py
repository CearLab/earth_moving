import math
import numpy as np
import matplotlib.pyplot as plt


# =========================
# Potential field functions
# =========================

def attractive_p(p, goal, zeta=1.0):
    p = np.asarray(p)
    goal = np.asarray(goal)
    return 0.5 * zeta * np.sum((p - goal) ** 2)


def attractive_grad(p, goal, zeta=1.0):
    p = np.asarray(p)
    goal = np.asarray(goal)
    return zeta * (p - goal)


def distance_to_obstacle(p, obs):
    p = np.asarray(p)
    c = np.asarray(obs["center"])
    r = obs["radius"]
    return np.linalg.norm(p - c) - r


def repulsive_p(p, obs, eta=5.0, rho0=1.0):
    rho = distance_to_obstacle(p, obs)
    if rho > rho0:
        return 0.0
    if rho <= 0.0:
        rho = 1e-3  # inside obstacle → clamp to avoid division by zero
    return 0.5 * eta * (1.0 / rho - 1.0 / rho0) ** 2


def repulsive_grad(p, obs, eta=5.0, rho0=1.0):
    p = np.asarray(p)
    c = np.asarray(obs["center"])
    r = obs["radius"]

    diff = p - c
    dist_center = np.linalg.norm(diff)
    if dist_center < 1e-6:
        return np.zeros(2)

    rho = dist_center - r
    if rho > rho0:
        return np.zeros(2)
    if rho <= 0.0:
        rho = 1e-3

    d_rho_dp = diff / dist_center
    common = eta * (1.0 / rho - 1.0 / rho0) * (-1.0 / (rho ** 2))
    return common * d_rho_dp


def total_p(p, goal, obstacles, zeta=1.0, eta=5.0, rho0=1.0):
    U = attractive_p(p, goal, zeta)
    for obs in obstacles:
        U += repulsive_p(p, obs, eta, rho0)
    return U


def total_grad(p, goal, obstacles, zeta=1.0, eta=5.0, rho0=1.0):
    grad = attractive_grad(p, goal, zeta)
    for obs in obstacles:
        grad += repulsive_grad(p, obs, eta, rho0)
    return grad


# =========================
# Robot model + simulation
# =========================

def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def simulate_nonholonomic(start_state, goal, obstacles,
                          dt=0.05, max_steps=4000,
                          v_max=1.0, w_max=3.0,
                          zeta=1.0, eta=8.0, rho0=1.0,
                          k_omega=6.0,
                          goal_tol=0.2,
                          stuck_window=80,
                          stuck_tol=1e-3,
                          small_grad=1e-3,
                          escape_horizon=80):
    """
    Non-holonomic potential-field navigation with a simple local-minimum escape:
    - normally follow -grad(U)
    - if progress stalls or |grad(U)| is tiny → move tangentially to the level set
      (orthogonal to grad) for 'escape_horizon' steps.
    """

    x, y, theta = start_state
    path_x = [x]
    path_y = [y]

    p = np.array([x, y])
    best_dist = np.linalg.norm(p - goal)
    stuck_counter = 0
    escape_timer = 0

    for k in range(max_steps):
        p = np.array([x, y])
        gradU = total_grad(p, goal, obstacles, zeta, eta, rho0)
        dist_to_goal = np.linalg.norm(p - goal)

        # reached goal
        if dist_to_goal < goal_tol:
            break

        # ---- stuck detection (no improvement in distance) ----
        if dist_to_goal < best_dist - stuck_tol:
            best_dist = dist_to_goal
            stuck_counter = 0
        else:
            stuck_counter += 1

        # trigger escape mode if stuck or gradient is too small
        if stuck_counter > stuck_window or np.linalg.norm(gradU) < small_grad:
            escape_timer = escape_horizon
            stuck_counter = 0

        # ---- choose direction ----
        if escape_timer > 0:
            # move tangential to the potential contour: direction ⟂ gradU
            g_norm = np.linalg.norm(gradU)
            if g_norm < 1e-6:
                d = goal - p  # degenerate: just aim at goal
            else:
                g_dir = gradU / g_norm
                tangent = np.array([-g_dir[1], g_dir[0]])  # rotate +90°
                to_goal = goal - p
                if np.dot(tangent, to_goal) < 0:
                    tangent = -tangent  # choose direction roughly toward the goal
                d = tangent
            escape_timer -= 1
        else:
            # normal steepest descent
            d = -gradU

        # normalize direction
        norm_d = np.linalg.norm(d)
        if norm_d < 1e-8:
            break
        d /= norm_d

        desired_heading = math.atan2(d[1], d[0])
        alpha = wrap_angle(desired_heading - theta)

        # unicycle control
        if abs(alpha) < math.pi / 2:
            v = v_max * math.cos(alpha)
        else:
            v = 0.0
        w = k_omega * alpha

        v = max(min(v, v_max), -v_max)
        w = max(min(w, w_max), -w_max)

        # integrate unicycle model
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        theta = wrap_angle(theta)

        path_x.append(x)
        path_y.append(y)

    return np.array(path_x), np.array(path_y)


# =========================
# Main demo
# =========================

def main():
    # World bounds
    x_min, x_max = -5.0, 5.0
    y_min, y_max = -5.0, 5.0

    # Start and goal
    goal = np.array([4.0, 4.0])
    start_state = np.array([-4.0, -4.0, 0.0])

    # Dense obstacle configuration
    obstacles = [
        {"center": np.array([-2.5, -1.5]), "radius": 0.6},
        {"center": np.array([-2.0,  1.0]), "radius": 0.7},
        {"center": np.array([-1.0,  1.0]), "radius": 0.7},
        {"center": np.array([-0.5, -0.5]), "radius": 0.6},
        {"center": np.array([ 0.0,  0.8]), "radius": 0.5},
        {"center": np.array([ 0.5,  2.0]), "radius": 0.6},
        {"center": np.array([ 1.0, -1.5]), "radius": 0.9},
        {"center": np.array([ 1.5,  0.0]), "radius": 0.7},
        {"center": np.array([ 2.0,  2.5]), "radius": 0.6},
        {"center": np.array([-1.5,  3.0]), "radius": 0.6},
        {"center": np.array([ 3.0, -0.5]), "radius": 0.7},
        {"center": np.array([ 2.5, -2.0]), "radius": 0.6},
    ]

    # Simulate robot
    path_x, path_y = simulate_nonholonomic(
        start_state, goal, obstacles,
        dt=0.05, max_steps=4000,
        v_max=1.0, w_max=3.0,
        zeta=1.0, eta=8.0, rho0=1.0,
        k_omega=6.0,
        goal_tol=0.2
    )

    # =========================
    # Potential field heatmap
    # =========================
    nx, ny = 150, 150
    xs = np.linspace(x_min, x_max, nx)
    ys = np.linspace(y_min, y_max, ny)
    X, Y = np.meshgrid(xs, ys)
    U = np.zeros_like(X)

    for i in range(nx):
        for j in range(ny):
            p = np.array([X[j, i], Y[j, i]])
            U[j, i] = total_p(p, goal, obstacles, zeta=1.0, eta=8.0, rho0=1.0)

    fig, ax = plt.subplots(figsize=(7, 6))

    # Heatmap of potential
    hm = ax.contourf(X, Y, U, levels=40)
    cbar = fig.colorbar(hm, ax=ax)
    cbar.set_label("Potential U(x, y)")

    # Obstacles
    for obs in obstacles:
        circle = plt.Circle(obs["center"], obs["radius"],
                            color="black", alpha=0.5)
        ax.add_patch(circle)

    # Path, start, goal
    ax.plot(path_x, path_y, "w-", linewidth=2, label="Robot path")
    ax.plot(start_state[0], start_state[1], "go", label="Start")
    ax.plot(goal[0], goal[1], "rx", markersize=10, label="Goal")

    ax.set_title("Potential field navigation with local-minimum escape")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.legend(loc="upper left")

    plt.show()


if __name__ == "__main__":
    main()
