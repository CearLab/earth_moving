import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math

USE_ADVISOR_MODEL = False  # Toggle this to switch between your model and advisor's

def smooth_path_with_spline(waypoints, smoothing_factor=0.5, num_points=1000):
    waypoints = np.array([(x + 0.5, y + 0.5) for x, y in waypoints])
    x, y = waypoints[:, 0], waypoints[:, 1]

    if len(x) < 2:
        return [], [], False

    try:
        k = min(3, len(x) - 1)
        tck, _ = splprep([x, y], s=smoothing_factor, k=k)
        t_fine = np.linspace(0, 1, num_points)
        smooth_x, smooth_y = splev(t_fine, tck)
        dx, dy = splev(t_fine, tck, der=1)
        d2x, d2y = splev(t_fine, tck, der=2)
        curvature = np.abs(dx * d2y - dy * d2x) / np.power(dx**2 + dy**2, 1.5)
        curvature[np.isnan(curvature)] = 0
        return list(zip(smooth_x, smooth_y)), curvature.tolist(), True
    except Exception:
        return [], [], False

def simulate_spillage(waypoints, objects_at_cells, agent_capacity, spillage_factor, min_spillage_threshold=0.1):
    spline_points, curvature, ok = smooth_path_with_spline(waypoints)
    if not ok:
        print("Error: Not enough waypoints or spline fitting failed.")
        return [], {}, 0, sum(objects_at_cells.values())

    max_possible_objects = sum(objects_at_cells.values())

    if USE_ADVISOR_MODEL:
        return simulate_spillage_advisor(spline_points, curvature, objects_at_cells, agent_capacity, max_possible_objects)
    else:
        impacted_cells = {}
        total_objects = 0
        total_spilled = 0  # Track total spilled objects for verification
        visited_cells = set()

        for i, (px, py) in enumerate(spline_points):
            cell_x, cell_y = int(math.floor(px)), int(math.floor(py))
            if (cell_x, cell_y) in objects_at_cells and (cell_x, cell_y) not in visited_cells:
                total_objects += objects_at_cells[(cell_x, cell_y)]
                visited_cells.add((cell_x, cell_y))

            spilled_objects = spillage_factor * curvature[i] * total_objects
            spilled_objects = min(spilled_objects, total_objects)

            if spilled_objects >= min_spillage_threshold:
                impacted_cells[(cell_x, cell_y)] = impacted_cells.get((cell_x, cell_y), 0) + spilled_objects
                total_objects -= spilled_objects
                total_spilled += spilled_objects

        # Handle the final cell specially
        target_cell_x, target_cell_y = int(math.floor(spline_points[-1][0])), int(math.floor(spline_points[-1][1]))
        
        # Make sure remaining objects go to the target cell
        impacted_cells[(target_cell_x, target_cell_y)] = impacted_cells.get((target_cell_x, target_cell_y), 0) + total_objects
        
        # Verify conservation of objects
        total_distributed = sum(impacted_cells.values())
        
        # Debug information
        if abs(total_distributed - max_possible_objects) > 0.01:  # Allow for floating point errors
            print(f"WARNING: Conservation issue detected!")
            print(f"Initial objects: {max_possible_objects}")
            print(f"Distributed objects: {total_distributed}")
            print(f"Difference: {max_possible_objects - total_distributed}")
            
            # Adjust the target cell to ensure conservation
            adjustment = max_possible_objects - total_distributed
            impacted_cells[(target_cell_x, target_cell_y)] += adjustment
            print(f"Adjusted target cell by {adjustment} objects")
        
        # Round and filter out zero-value cells
        impacted_cells = {cell: int(round(value)) for cell, value in impacted_cells.items() if value > 0}
        total_objects_at_target = impacted_cells.get((target_cell_x, target_cell_y), 0)

        return spline_points, impacted_cells, total_objects_at_target, max_possible_objects

def simulate_spillage_advisor(spline_points, curvature, objects_at_cells, agent_capacity, max_possible_objects):
    """
    Placeholder for advisor's spillage model.
    :param spline_points: Smoothed path
    :param curvature: List of curvature values
    :param objects_at_cells: Map of original object locations
    :param agent_capacity: Carrying capacity
    :param max_possible_objects: Total initial objects on path
    :return: (spline_points, impacted_cells, total_objects_at_target, max_possible_objects)
    """
    # --- Advisor logic should be implemented here ---
    # For now, simulate zero spillage (all objects arrive safely)
    impacted_cells = {}
    total_objects = sum(objects_at_cells.values())
    final_cell_x, final_cell_y = int(math.floor(spline_points[-1][0])), int(math.floor(spline_points[-1][1]))
    impacted_cells[(final_cell_x, final_cell_y)] = total_objects
    return spline_points, impacted_cells, total_objects, max_possible_objects


def visualize_spillage(self, spline_points, impacted_cells, best_path):
    import matplotlib.pyplot as plt
    plt.figure(figsize=(8, 8))
    smooth_x, smooth_y = zip(*spline_points)
    plt.plot(smooth_x, smooth_y, label="Smoothed Path", color="blue")
    waypoints = [(c.x + 0.5, c.y + 0.5) for c in best_path]
    plt.scatter(*zip(*waypoints), label="Waypoints", color="red", marker='o')
    if impacted_cells:
        plt.scatter(*zip(*impacted_cells.keys()), s=50, label="Spillage Locations", color="orange", marker='x')
    final_target = best_path[-1]
    plt.scatter(final_target.x + 0.5, final_target.y + 0.5, s=100, label="Target Zone", color="green", marker='D')
    plt.xlabel("X-axis (Cells)")
    plt.ylabel("Y-axis (Cells)")
    plt.title("Spillage Visualization")
    plt.legend()
    plt.grid()
    plt.show()
