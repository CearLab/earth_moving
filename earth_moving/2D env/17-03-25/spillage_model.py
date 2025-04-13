import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math

def simulate_spillage(waypoints, objects_at_cells, agent_capacity, spillage_factor, min_spillage_threshold=0.1):
    """
    Simulates object spillage along a smoothed path and returns the detailed curve and impacted cells.

    :param waypoints: List of (x, y) tuples representing the path.
    :param objects_at_cells: Dictionary mapping (x, y) -> number of objects in each cell.
    :param agent_capacity: Maximum carrying capacity of the agent.
    :param spillage_factor: Proportion of objects lost per unit curvature.
    :param min_spillage_threshold: Minimum amount of spillage to be considered valid.
    :return: (spline_points, impacted_cells, total_objects_at_target, max_possible_objects)
    """
    # ✅ Adjust waypoints to center of cells
    waypoints = np.array([(x + 0.5, y + 0.5) for x, y in waypoints])
    x, y = waypoints[:, 0], waypoints[:, 1]

    # ✅ Ensure enough waypoints for spline interpolation
    if len(x) < 2:
        print("Error: Not enough waypoints for a valid path.")
        return [], {}, 0, sum(objects_at_cells.values())  # Return empty results

    # ✅ Adjust spline degree dynamically (must be ≤ number of waypoints - 1)
    spline_degree = min(3, len(x) - 1)

    # ✅ Generate smooth spline path
    tck, _ = splprep([x, y], s=0.5, k=spline_degree)  # Avoids `m > k` error
    num_interpolated_points = 1000  # High-resolution path
    t_fine = np.linspace(0, 1, num_interpolated_points)
    smooth_x, smooth_y = splev(t_fine, tck)

    # ✅ Compute first derivative
    dx_dt, dy_dt = splev(t_fine, tck, der=1)

    # ✅ Compute curvature only if `k > 1`
    if spline_degree > 1:
        d2x_dt2, d2y_dt2 = splev(t_fine, tck, der=2)  # Second derivative
        curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt**2 + dy_dt**2)**(3/2)
        curvature[np.isnan(curvature)] = 0  # Fix NaNs
    else:
        curvature = np.zeros_like(dx_dt)  # No curvature for k=1 (linear)

    # ✅ Track objects & spillage
    impacted_cells = {}  # Stores affected cells and extra objects spilled
    total_objects = 0  # Objects currently carried
    visited_cells = set()  # Ensure each cell is collected **only once**
    spline_points = list(zip(smooth_x, smooth_y))  # Return this full path

    # ✅ Identify last cell (target zone)
    target_cell_x, target_cell_y = int(math.floor(smooth_x[-1])), int(math.floor(smooth_y[-1]))

    # ✅ Compute total possible objects (for comparison)
    max_possible_objects = sum(objects_at_cells.values())

    for i, (px, py) in enumerate(spline_points):
        # ✅ Find closest cell (rounding to nearest grid cell)
        cell_x, cell_y = int(math.floor(px)), int(math.floor(py))

        # ✅ If reaching a waypoint and not collected yet, collect objects
        if (cell_x, cell_y) in objects_at_cells and (cell_x, cell_y) not in visited_cells:
            total_objects += objects_at_cells[(cell_x, cell_y)]
            visited_cells.add((cell_x, cell_y))  # Mark as visited

        # ✅ Compute spillage
        spilled_objects = spillage_factor * curvature[i] * total_objects
        spilled_objects = min(spilled_objects, total_objects)  # Can't spill more than carried

        # ✅ Apply spillage threshold
        if spilled_objects >= min_spillage_threshold:
            impacted_cells[(cell_x, cell_y)] = impacted_cells.get((cell_x, cell_y), 0) + spilled_objects
            total_objects -= spilled_objects  # Update carried objects

    # ✅ Final update at the end (objects reaching target)
    total_objects_at_target = total_objects + impacted_cells.get((target_cell_x, target_cell_y), 0)
    impacted_cells[(target_cell_x, target_cell_y)] = total_objects_at_target  # Store final objects in target zone

    # ✅ Round values to integers
    impacted_cells = {cell: int(round(value)) for cell, value in impacted_cells.items()}

    # ✅ Remove zero-value cells
    impacted_cells = {cell: value for cell, value in impacted_cells.items() if value > 0}

    total_objects_at_target = int(round(total_objects_at_target))

    return spline_points, impacted_cells, total_objects_at_target, max_possible_objects


# ======================== #
# ==== TESTING SCRIPT ==== #
# ======================== #
if __name__ == "__main__":
    # Example path with waypoints
    waypoints = [(0, 0), (1, 1), (2, 5), (4, 5), (6, 7)]  # Example path

    # Mock object distribution at cells
    objects_at_cells = {
        (0, 0): 5,
        (1, 1): 8,
        (2, 5): 3,
        (4, 5): 6
    }

    # Simulation parameters
    agent_capacity = 50
    spillage_factor = 0.05  # Adjust based on real-world tests
    min_spillage_threshold = 0.3  # Minimum spillage to be counted

    # Run simulation
    spline_points, impacted_cells, total_objects_at_target, max_possible_objects = simulate_spillage(
        waypoints, objects_at_cells, agent_capacity, spillage_factor, min_spillage_threshold
    )

    # ✅ Print results
    print("Total spline points:", len(spline_points))
    print("Impacted Cells:", impacted_cells)
    print(f"Total Objects Reaching Target: {total_objects_at_target}/{max_possible_objects} "
          f"({(max_possible_objects - total_objects_at_target) / max_possible_objects * 100:.1f}% lost)")

    # ✅ Visualization
    plt.figure(figsize=(8, 8))
    plt.plot(*zip(*spline_points), label="Smoothed Path", color="blue")
    plt.scatter(*zip(*waypoints), label="Waypoints", color="red", marker='o')
    plt.scatter(*zip(*impacted_cells.keys()), s=50, label="Spillage Locations", color="orange", marker='x')
    plt.scatter(target_cell_x, target_cell_y, s=100, label="Target Zone", color="green", marker='D')
    plt.xlabel("X-axis (Cells)")
    plt.ylabel("Y-axis (Cells)")
    plt.title("Spillage Simulation Along Spline Path")
    plt.legend()
    plt.grid()
    plt.show()


def visualize_spillage(self, spline_points, impacted_cells, best_path):
    """
    Visualize spillage along the path.

    :param spline_points: List of spline points (smoothed path).
    :param impacted_cells: Dictionary of impacted cells and spilled objects.
    :param best_path: The original path before smoothing.
    """
    import matplotlib.pyplot as plt

    plt.figure(figsize=(8, 8))

    # Plot the spline path
    smooth_x, smooth_y = zip(*spline_points)
    plt.plot(smooth_x, smooth_y, label="Smoothed Path", color="blue")

    # Plot waypoints
    waypoints = [(c.x + 0.5, c.y + 0.5) for c in best_path]
    plt.scatter(*zip(*waypoints), label="Waypoints", color="red", marker='o')

    # Plot spillage locations
    if impacted_cells:
        plt.scatter(*zip(*impacted_cells.keys()), s=50, label="Spillage Locations", color="orange", marker='x')

    # Mark the final target cell
    final_target = best_path[-1]
    plt.scatter(final_target.x + 0.5, final_target.y + 0.5, s=100, label="Target Zone", color="green", marker='D')

    plt.xlabel("X-axis (Cells)")
    plt.ylabel("Y-axis (Cells)")
    plt.title("Spillage Visualization")
    plt.legend()
    plt.grid()
    plt.show()
