
# imports
import numpy as np
import random
from scipy.interpolate import CubicSpline
import earth_moving.constants as const

def generate_trajectory(start, end, height, n_points = 1, degree = 3):
            
        # start from the left side
        bound = 0.4 * height
        middle_points = []              
        
        # middle points            
        middle_points = []
        for i in range(1, n_points + 1):
            fraction = i / (n_points + 1)
            x = start[0] + fraction * (end[0] - start[0])
            # bound = np.clip(bound * 5, -height, height)
            y = random.uniform(-bound, bound)
            middle_points.append(np.array((x, y, start[-1])))                            
        
        # generate stream
        points = [start] + middle_points + [end]
        
        # generate path
        path = generate_path_from_points(points, degree, 100 * n_points, end[0], height)
        orientation = get_tangent_orientation(path)
        return path, orientation

def generate_path_from_points(points, degree, num_points, width, height) -> list:                

        # Generate cubic spline points
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Create cubic splines for x and y
        t = np.linspace(0, 1, len(points))
        spline_x = np.poly1d(np.polyfit(t, x, deg=degree))
        spline_y = np.poly1d(np.polyfit(t, y, deg=degree))
        spline_z = np.poly1d(np.polyfit(t, z, deg=0))

        # Generate points along the spline
        t_values = np.linspace(0, 1, num_points)
        spline_points = np.array([[spline_x(t), spline_y(t), spline_z(t)] for t in t_values])        

        # Clip points to the limits
        spline_points = np.clip(spline_points, [0, -height, 0], [width, height, np.inf])

        # Remove duplicates
        path = []
        seen = set()
        for point in spline_points:
            if tuple(point) not in seen:
                path.append(point.tolist())
                seen.add(tuple(point))

        return path
    
def get_tangent_orientation(path):
    
    index = 0        
    current_point = np.array(path[index])
    orientation = []
    
    while index < len(path):
        # Check if the current point is close to the next point
        next_index = (index + 1) % len(path)
        next_point = np.array(path[next_index])                
        
        # Calculate the tangent vector
        tangent_vector = next_point - current_point
        tangent_vector /= np.linalg.norm(tangent_vector)  # Normalize
        
        yaw = np.arctan2(tangent_vector[1], tangent_vector[0])
        orientation.append([0, 0, yaw])
                        
        index += 1
        current_point = next_point        
    
    
    
    return orientation


    


    