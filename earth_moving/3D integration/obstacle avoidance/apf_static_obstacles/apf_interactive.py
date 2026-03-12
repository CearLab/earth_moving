"""
APF Interactive Navigation Tool (Aggressive Parameters)

This is a simplified and optimized version of the APF navigation tester.
It uses AGGRESSIVE parameters which have been validated as the best-performing configuration.

Features:
- Quick obstacle configuration
- Interactive goal position selection
- Real-time APF navigation with proven aggressive parameters
- Comparison testing

Usage:
    python apf_interactive.py

Aggressive Parameters (OPTIMIZED):
    - k_rep = 0.05 (very low repulsion - allows close navigation)
    - d0 = 0.45m (short range influence)
    - k_att = 1.5 (moderate attraction to goal)
    - v_max = 1.3 m/s (very fast)
"""

import numpy as np
import time
import pybullet as p
from pybullet_integration_apf import PyBulletIntegration, _goto_point
import apf_nav

# Try to import matplotlib for potential field visualization
try:
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    import matplotlib.cm as cm
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib not installed - heatmap visualization disabled")


def create_obstacle_field():
    """
    Create obstacle field configuration.

    Returns:
        List of (x, y) obstacle positions
    """
    print("\n" + "="*60)
    print("OBSTACLE CONFIGURATION")
    print("="*60)
    print("\nChoose obstacle setup:")
    print("  1. Corridor (obstacles on sides)")
    print("  2. Wall (horizontal barrier)")
    print("  3. Scattered (random obstacles)")
    print("  4. Dense cluster (center)")
    print("  5. Custom positions")

    choice = input("\nSelect (1-5): ").strip()

    if choice == "1":
        # Corridor: obstacles along sides
        obstacles = []
        for i in range(-5, 6):
            obstacles.append((0.5, i * 0.3))   # Right side
            obstacles.append((-0.5, i * 0.3))  # Left side
        print(f"[OK] Created corridor with {len(obstacles)} obstacles")
        return obstacles

    elif choice == "2":
        # Wall: horizontal barrier
        obstacles = []
        for i in range(-5, 6):
            obstacles.append((i * 0.15, 0.0))
        print(f"[OK] Created wall with {len(obstacles)} obstacles")
        return obstacles

    elif choice == "3":
        # Scattered random
        num_obs = int(input("Number of obstacles (10-100): ").strip())
        radius = float(input("Scatter radius in meters (0.5-1.5): ").strip())
        obstacles = []
        for i in range(num_obs):
            r = radius * np.sqrt(np.random.random())
            theta = 2 * np.pi * np.random.random()
            obstacles.append((r * np.cos(theta), r * np.sin(theta)))
        print(f"[OK] Created {len(obstacles)} scattered obstacles")
        return obstacles

    elif choice == "4":
        # Dense center
        num_obs = int(input("Number of obstacles (10-100): ").strip())
        cluster_radius = 0.5
        obstacles = []
        for i in range(num_obs):
            r = cluster_radius * np.sqrt(np.random.random())
            theta = 2 * np.pi * np.random.random()
            obstacles.append((r * np.cos(theta), r * np.sin(theta)))
        print(f"[OK] Created {len(obstacles)} obstacles in center")
        return obstacles

    elif choice == "5":
        # Custom manual entry
        obstacles = []
        print("\nEnter obstacle positions (x, y) one per line.")
        print("Type 'done' when finished.")
        while True:
            inp = input(f"Obstacle {len(obstacles)+1} (x, y) or 'done': ").strip()
            if inp.lower() == 'done':
                break
            try:
                x, y = map(float, inp.split(','))
                obstacles.append((x, y))
                print(f"  [+] Added at ({x:.3f}, {y:.3f})")
            except:
                print("  [-] Invalid format. Use: x, y")
        print(f"[OK] Created {len(obstacles)} custom obstacles")
        return obstacles

    else:
        print("Invalid choice. Using scattered obstacles.")
        return create_obstacle_field()


def setup_environment(num_pebbles=0, obstacle_positions=None, random_seed=42):
    """
    Setup PyBullet environment with obstacles.

    Args:
        num_pebbles: Number of random pebbles (if obstacle_positions is None)
        obstacle_positions: List of (x, y) positions
        random_seed: Random seed for reproducibility

    Returns:
        PyBulletIntegration instance
    """
    if obstacle_positions is not None:
        num_pebbles = len(obstacle_positions)

    integration = PyBulletIntegration(
        env_radius=2.0,
        target_zone_radius=0.3,
        num_pebbles=num_pebbles,
        random_seed=random_seed,
        gui=True,
        initial_robot_pose=(0, 0, 0)
    )

    # Set robot dimensions
    integration.set_robot_dim(L=0.2, R=0.07)

    # Setup scene
    integration.setup_scene(integration.initial_robot_pose)

    # Position obstacles if provided
    if obstacle_positions is not None:
        print(f"\n[LOC] Positioning {len(obstacle_positions)} obstacles...")
        for i, (x, y) in enumerate(obstacle_positions):
            if i + 2 < len(integration.object_ids):
                obj_id = integration.object_ids[i + 2]
                _, quat = p.getBasePositionAndOrientation(obj_id)
                p.resetBasePositionAndOrientation(obj_id, [x, y, 0.025], quat)

        for _ in range(100):
            p.stepSimulation()

        print("[OK] Obstacles positioned")

    # Set AGGRESSIVE parameters (proven best configuration)
    integration.apf_params = apf_nav.aggressive()
    integration.use_apf_navigation = True

    return integration


def visualize_potential_field(integration, goal, robot_start=(0, 0)):
    """
    Create and display a 2D heatmap of the potential field.

    Args:
        integration: PyBulletIntegration instance
        goal: (x, y) goal position
        robot_start: (x, y) robot starting position
    """
    if not MATPLOTLIB_AVAILABLE:
        print("[SKIP] Matplotlib not available - skipping heatmap visualization")
        return

    print("\n[INFO] Generating potential field heatmap...")

    # Get obstacle positions from PyBullet
    robot_id = integration.object_ids[1]
    obstacle_ids = integration.object_ids[2:]  # All except plane and robot

    obstacles = []
    for obs_id in obstacle_ids:
        try:
            pos, _ = p.getBasePositionAndOrientation(obs_id)
            obstacles.append((pos[0], pos[1]))
        except:
            pass

    # Create grid for potential field calculation
    grid_resolution = 0.1  # meters per cell
    x_range = np.arange(-2.2, 2.2, grid_resolution)
    y_range = np.arange(-2.2, 2.2, grid_resolution)
    X, Y = np.meshgrid(x_range, y_range)

    # Calculate potential field
    Z = np.zeros_like(X, dtype=float)

    goal_x, goal_y = goal

    # Parameters from aggressive configuration
    k_att = 1.5  # Attraction coefficient
    k_rep = 0.05  # Repulsion coefficient
    d0 = 0.45  # Repulsion distance

    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            x, y = X[i, j], Y[i, j]

            # Attractive potential (goal)
            dist_to_goal = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            V_att = 0.5 * k_att * dist_to_goal**2

            # Repulsive potential (obstacles)
            V_rep = 0
            for obs_x, obs_y in obstacles:
                dist_to_obs = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                if dist_to_obs < d0:
                    V_rep += 0.5 * k_rep * (1/dist_to_obs - 1/d0)**2

            # Total potential
            Z[i, j] = V_att + V_rep

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot heatmap
    norm = Normalize(vmin=Z.min(), vmax=np.percentile(Z, 95))
    heatmap = ax.contourf(X, Y, Z, levels=50, cmap='viridis', norm=norm)

    # Plot obstacles
    if obstacles:
        obs_x, obs_y = zip(*obstacles)
        ax.scatter(obs_x, obs_y, c='red', s=100, marker='o',
                  label='Obstacles', edgecolors='darkred', linewidth=2)

    # Plot goal
    ax.scatter([goal_x], [goal_y], c='lime', s=200, marker='*',
              label='Goal', edgecolors='darkgreen', linewidth=2)

    # Plot robot start position
    ax.scatter([robot_start[0]], [robot_start[1]], c='cyan', s=150, marker='s',
              label='Robot Start', edgecolors='darkblue', linewidth=2)

    # Environment boundary
    circle = plt.Circle((0, 0), 2.0, color='black', fill=False,
                       linewidth=2, linestyle='--', label='Boundary')
    ax.add_patch(circle)

    # Labels and formatting
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-2.2, 2.2)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('APF Potential Field Heatmap\n(Aggressive Parameters: k_rep=0.05, d0=0.45m, k_att=1.5)',
                fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)

    # Add colorbar
    cbar = plt.colorbar(heatmap, ax=ax)
    cbar.set_label('Potential Energy', fontsize=11)

    # Show the plot
    plt.tight_layout()
    plt.show(block=False)

    print("[OK] Potential field heatmap displayed!")
    print("[INFO] Blue/dark areas = low potential (good paths)")
    print("[INFO] Yellow/bright areas = high potential (obstacles or far from goal)")

    return fig


def get_goal():
    """Get goal position from user."""
    print("\n" + "="*60)
    print("GOAL POSITION")
    print("="*60)
    print("\nChoose goal:")
    print("  1. North (0, 1.5)")
    print("  2. South (0, -1.5)")
    print("  3. East (1.5, 0)")
    print("  4. West (-1.5, 0)")
    print("  5. Northeast (1.0, 1.0)")
    print("  6. Northwest (-1.0, 1.0)")
    print("  7. Southeast (1.0, -1.0)")
    print("  8. Southwest (-1.0, -1.0)")
    print("  9. Custom (x, y)")

    choice = input("\nSelect (1-9): ").strip()

    goals = {
        '1': (0, 1.5),
        '2': (0, -1.5),
        '3': (1.5, 0),
        '4': (-1.5, 0),
        '5': (1.0, 1.0),
        '6': (-1.0, 1.0),
        '7': (1.0, -1.0),
        '8': (-1.0, -1.0)
    }

    if choice in goals:
        goal = goals[choice]
        print(f"[OK] Goal: ({goal[0]:.3f}, {goal[1]:.3f})")
        return goal
    elif choice == '9':
        try:
            inp = input("Enter goal (x, y): ").strip()
            x, y = map(float, inp.split(','))
            print(f"[OK] Goal: ({x:.3f}, {y:.3f})")
            return (x, y)
        except:
            print("Invalid format. Using (1.0, 1.0)")
            return (1.0, 1.0)
    else:
        print("Invalid choice. Using (1.0, 1.0)")
        return (1.0, 1.0)


def run_navigation(integration, goal, max_time=20.0, v_nom=0.4, tol=0.1):
    """
    Run APF navigation test.

    Args:
        integration: PyBulletIntegration instance
        goal: (x, y) goal position
        max_time: Maximum navigation time
        v_nom: Nominal velocity
        tol: Goal tolerance

    Returns:
        dict: Results
    """
    goal_x, goal_y = goal

    # Visualize goal
    goal_marker = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.08,
        rgbaColor=[1, 0, 0, 0.9]
    )
    goal_body = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=goal_marker,
        basePosition=[goal_x, goal_y, 0.1]
    )

    print(f"\n{'='*60}")
    print(f"POTENTIAL FIELD VISUALIZATION")
    print(f"{'='*60}")

    # Show potential field heatmap
    visualize_potential_field(integration, goal, robot_start=(0, 0))

    print(f"\n{'='*60}")
    print(f"READY TO START NAVIGATION")
    print(f"{'='*60}")
    print(f"Goal: ({goal_x:.3f}, {goal_y:.3f})")
    print(f"Distance: {np.sqrt(goal_x**2 + goal_y**2):.3f}m")
    print(f"k_rep=0.05, d0=0.45m, v_max=1.3m/s")
    print(f"Max time: {max_time:.1f}s")
    print(f"{'='*60}")

    # Wait for user confirmation before starting navigation
    input("\nPress Enter to start navigation...")

    print(f"\n{'='*60}")
    print(f"STARTING NAVIGATION (AGGRESSIVE PARAMETERS)")
    print(f"{'='*60}\n")

    start_time = time.time()

    # Navigate
    _goto_point(
        integration, goal,
        use_tcp_pose=False,
        v_nom=v_nom,
        tol=tol,
        max_time=max_time,
        use_apf=True,
        apf_params=integration.apf_params
    )

    elapsed_time = time.time() - start_time

    # Get final position
    robot_id = integration.object_ids[1]
    final_pos, _ = p.getBasePositionAndOrientation(robot_id)
    final_distance = np.sqrt((final_pos[0] - goal_x)**2 + (final_pos[1] - goal_y)**2)

    success = final_distance <= tol

    print(f"\n{'='*60}")
    print(f"RESULT: {'SUCCESS' if success else 'INCOMPLETE'}")
    print(f"{'='*60}")
    print(f"Time: {elapsed_time:.2f}s")
    print(f"Final distance: {final_distance:.4f}m")
    print(f"Goal reached: {'YES' if success else 'NO'}")
    print(f"{'='*60}\n")

    p.removeBody(goal_body)

    return {
        'time': elapsed_time,
        'final_distance': final_distance,
        'success': success
    }


def main():
    """Main interactive navigation tool."""
    print("\n" + "="*60)
    print("APF INTERACTIVE NAVIGATION TOOL")
    print("OPTIMIZED WITH AGGRESSIVE PARAMETERS")
    print("="*60)

    while True:
        print("\n" + "="*60)
        print("MAIN MENU")
        print("="*60)
        print("1. Quick test (random obstacles)")
        print("2. Custom obstacle field")
        print("3. Comparison: APF vs Simple Steering")
        print("4. Exit")

        choice = input("\nSelect (1-4): ").strip()

        if choice == '1':
            # Quick test
            num_pebbles = int(input("\nNumber of pebbles (0-100): ").strip())
            random_seed = int(input("Random seed (any number): ").strip())

            integration = setup_environment(num_pebbles, None, random_seed)
            goal = get_goal()
            run_navigation(integration, goal)

            input("\nPress Enter to continue...")
            p.disconnect()

        elif choice == '2':
            # Custom obstacles
            obstacle_positions = create_obstacle_field()
            integration = setup_environment(0, obstacle_positions, 42)
            goal = get_goal()
            run_navigation(integration, goal)

            input("\nPress Enter to continue...")
            p.disconnect()

        elif choice == '3':
            # Comparison test
            print("\n" + "="*60)
            print("COMPARISON: APF (Aggressive) vs Simple Steering")
            print("="*60)

            num_pebbles = int(input("\nNumber of pebbles (0-100): ").strip())
            seed = int(input("Random seed: ").strip())
            goal = get_goal()

            # Test 1: APF with aggressive params
            print("\n>>> TEST 1: APF (Aggressive Parameters)")
            integration1 = setup_environment(num_pebbles, None, seed)
            result_apf = run_navigation(integration1, goal)
            input("\nPress Enter to run simple steering test...")
            p.disconnect()

            # Test 2: Simple Steering
            print("\n>>> TEST 2: Simple Steering")
            integration2 = setup_environment(num_pebbles, None, seed)
            integration2.use_apf_navigation = False
            result_simple = run_navigation(integration2, goal)

            # Comparison
            print("\n" + "="*60)
            print("COMPARISON RESULTS")
            print("="*60)
            print(f"{'Metric':<20} {'APF':<15} {'Simple':<15}")
            print("-"*60)
            print(f"{'Time (s)':<20} {result_apf['time']:<15.2f} {result_simple['time']:<15.2f}")
            print(f"{'Final Distance (m)':<20} {result_apf['final_distance']:<15.4f} {result_simple['final_distance']:<15.4f}")
            print(f"{'Success':<20} {str(result_apf['success']):<15} {str(result_simple['success']):<15}")
            print("="*60)

            input("\nPress Enter to continue...")
            p.disconnect()

        elif choice == '4':
            print("\n[BYE] Goodbye!")
            break

        else:
            print("Invalid choice. Try again.")


if __name__ == "__main__":
    main()
