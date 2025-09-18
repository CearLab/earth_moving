import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math

USE_ADVISOR_MODEL = False  # Toggle this to switch between your model and advisor's

def smooth_path_with_spline(waypoints, smoothing_factor=0.5, num_points=1000):
    waypoints = np.array([(x + 0.5, y + 0.5) for x, y in waypoints])
    x, y = waypoints[:, 0], waypoints[:, 1]

    if len(x) < 2:
        # print(f"Warning: Only {len(x)} waypoints provided, need at least 2")
        return [], [], False

    # For exactly 2 waypoints, use linear interpolation
    if len(x) == 2:
        # print(f"Using linear interpolation for 2-waypoint path")
        return linear_interpolation_with_curvature(x, y, num_points)

    try:
        # Adjust smoothing factor based on path length - shorter paths need less smoothing
        adaptive_smoothing = max(0.0, smoothing_factor * (len(x) - 2) / 3.0)
        k = min(3, len(x) - 1)
        
        tck, _ = splprep([x, y], s=adaptive_smoothing, k=k)
        t_fine = np.linspace(0, 1, num_points)
        smooth_x, smooth_y = splev(t_fine, tck)
        dx, dy = splev(t_fine, tck, der=1)
        d2x, d2y = splev(t_fine, tck, der=2)
        curvature = np.abs(dx * d2y - dy * d2x) / np.power(dx**2 + dy**2, 1.5)
        curvature[np.isnan(curvature)] = 0
        
        # print(f"Spline fitting successful for {len(x)} waypoints (k={k}, s={adaptive_smoothing:.3f})")
        return list(zip(smooth_x, smooth_y)), curvature.tolist(), True
        
    except Exception as e:
        # print(f"Spline fitting failed for {len(x)} waypoints: {e}")
        # print("Falling back to linear interpolation with artificial curvature")
        return linear_interpolation_with_curvature(x, y, num_points)

def linear_interpolation_with_curvature(x, y, num_points):
    """
    Fallback method for when spline fitting fails.
    Creates linear interpolation with artificial curvature based on direction changes.
    """
    # Linear interpolation between waypoints
    t_waypoints = np.linspace(0, 1, len(x))
    t_fine = np.linspace(0, 1, num_points)
    
    smooth_x = np.interp(t_fine, t_waypoints, x)
    smooth_y = np.interp(t_fine, t_waypoints, y)
    
    # Calculate artificial curvature based on direction changes
    curvature = np.zeros(num_points)
    
    if len(x) >= 3:
        # For paths with 3+ waypoints, add curvature at direction change points
        for i in range(1, len(x) - 1):
            # Calculate direction change at waypoint i
            v1 = np.array([x[i] - x[i-1], y[i] - y[i-1]])
            v2 = np.array([x[i+1] - x[i], y[i+1] - y[i]])
            
            # Normalize vectors
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0 and v2_norm > 0:
                v1_unit = v1 / v1_norm
                v2_unit = v2 / v2_norm
                
                # Calculate angle between vectors (0 = straight, pi = sharp turn)
                cross_product = v1_unit[0] * v2_unit[1] - v1_unit[1] * v2_unit[0]
                angle_change = abs(cross_product)  # Approximation of angle change
                
                # Add curvature around the waypoint
                waypoint_t = i / (len(x) - 1)  # Position of waypoint in t_fine
                waypoint_idx = int(waypoint_t * (num_points - 1))
                
                # Spread curvature over nearby points
                spread_range = max(1, num_points // 20)  # 5% of points around waypoint
                for j in range(max(0, waypoint_idx - spread_range), 
                             min(num_points, waypoint_idx + spread_range + 1)):
                    distance_weight = max(0, 1 - abs(j - waypoint_idx) / spread_range)
                    curvature[j] = max(curvature[j], angle_change * distance_weight * 0.5)
    else:
        # For 2-waypoint straight lines, add minimal baseline curvature
        curvature.fill(0.01)  # Small baseline for spillage calculations
    
    # print(f"Linear interpolation created with max curvature: {np.max(curvature):.4f}")
    return list(zip(smooth_x, smooth_y)), curvature.tolist(), True

def simulate_spillage(waypoints, objects_at_cells, agent_capacity, spillage_factor, min_spillage_threshold=0.1):
    # print(f"Simulating spillage for {len(waypoints)} waypoints, {len(objects_at_cells)} object locations")
    
    spline_points, curvature, ok = smooth_path_with_spline(waypoints)
    if not ok:
        # print(f"Warning: Path smoothing failed for waypoints: {waypoints}")
        # For single-waypoint cases, simulate direct delivery to target
        if len(waypoints) == 1:
            print("Single waypoint detected - simulating direct delivery with no spillage")
            target_cell = waypoints[0]
            total_objects = sum(objects_at_cells.values())
            impacted_cells = {target_cell: total_objects}
            return [(target_cell[0] + 0.5, target_cell[1] + 0.5)], impacted_cells, total_objects, total_objects
        else:
            # print(f"Unexpected path smoothing failure for {len(waypoints)} waypoints")
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
        
        # Conservation-preserving rounding: distribute fractional parts to ensure total matches
        rounded_cells = {}
        total_fractional_loss = 0
        
        # First pass: round down and track fractional losses
        for cell, value in impacted_cells.items():
            rounded_value = int(math.floor(value))
            fractional_part = value - rounded_value
            total_fractional_loss += fractional_part
            if rounded_value > 0:
                rounded_cells[cell] = rounded_value
        
        # Second pass: distribute fractional losses to preserve conservation
        # Find cells with largest fractional parts to receive extra objects
        if total_fractional_loss >= 0.5:
            fractional_cells = [(cell, value - math.floor(value)) for cell, value in impacted_cells.items()]
            fractional_cells.sort(key=lambda x: x[1], reverse=True)  # Sort by fractional part, descending
            
            extra_objects_needed = int(round(total_fractional_loss))
            for i in range(min(extra_objects_needed, len(fractional_cells))):
                cell, _ = fractional_cells[i]
                rounded_cells[cell] = rounded_cells.get(cell, 0) + 1
        
        impacted_cells = rounded_cells
        total_objects_at_target = impacted_cells.get((target_cell_x, target_cell_y), 0)
        
        # Verify conservation after rounding
        final_total = sum(impacted_cells.values())
        if abs(final_total - max_possible_objects) > 0.01:
            print(f"WARNING: Conservation violated after rounding! Expected {max_possible_objects}, got {final_total}")
            # Force conservation by adjusting target cell
            adjustment = max_possible_objects - final_total
            if (target_cell_x, target_cell_y) in impacted_cells:
                impacted_cells[(target_cell_x, target_cell_y)] += adjustment
            else:
                impacted_cells[(target_cell_x, target_cell_y)] = adjustment
            print(f"Adjusted target cell by {adjustment} objects")

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
