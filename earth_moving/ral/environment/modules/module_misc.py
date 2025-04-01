import random
from scipy.interpolate import CubicSpline
import numpy as np

# scale sigma
def scale_sigma(sigma, cluster_radius) -> float:
        return (cluster_radius + sigma - 1)/sigma

def D4_neighbourhood(position, limits, step=1) -> list:
    D4_neighbourhood = [[position[0], position[1] - step],
                        [position[0] + step, position[1]],
                        [position[0], position[1] + step],
                        [position[0] - step, position[1]]]    
    
    return D4_neighbourhood

# generate D8 neighbourhood
def D8_neighbourhood(position, limits, step=1) -> list:
    D8_neighbourhood = [[position[0], position[1] - step],
                        [position[0] + step, position[1] - step],
                        [position[0] + step, position[1]],
                        [position[0] + step, position[1] + step],
                        [position[0], position[1] + step],
                        [position[0] - step, position[1] + step],
                        [position[0] - step, position[1]],
                        [position[0] - step, position[1] - step]]
    
    return D8_neighbourhood

def is_within_limits(position, width, height):
        return 0 <= position[0] <= width and 0 <= position[1] <= height

def get_coordinates_from_perimeter(width, height, point) -> list:
        # counterclockwise
        if point < width:
            point_x = point
            point_y = 0
        elif point < width + height:
            point_x = width
            point_y = point - width 
        elif point < 2*width + height:
            point_x = 2*width + height - point
            point_y = height
        else:
            point_x = 0
            point_y = 2*width + 2*height - point
        return [point_x, point_y]

def generate_path_from_points(points, width, height) -> list:                

        # Generate cubic spline points
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

        # Parameterize the points
        t = np.linspace(0, 1, len(points))
        spline_x = np.poly1d(np.polyfit(t, x, 1))
        spline_y = np.poly1d(np.polyfit(t, y, 3))
        
        # Generate points along the spline
        num_points = 500  # Number of points to generate along the spline
        t_values = np.linspace(0, 1, num_points)
        spline_points = np.array([[spline_x(t), spline_y(t)] for t in t_values])

        # Round and convert to integer coordinates
        spline_points = np.rint(spline_points).astype(int)

        # Add the generated points to the path
        path = spline_points.tolist()        
                
        # remove duplicates
        seen = set()
        path = [x for x in path if tuple(x) not in seen and not seen.add(tuple(x))]
        
        # clip points to the limits
        path = np.clip(path, 0, [width, height])        
                
        # Ensure all points in the path are neighbours
        final_path = [path[0]]
        for i in range(1, len(path)):
            prev_point = final_path[-1]
            current_point = path[i]
            neighbours = np.asarray(D4_neighbourhood(prev_point, (width, height)))
            while not any((current_point == np.array(neighbour)).all() for neighbour in neighbours):
                # Find the next neighbour closer to the current_point
                neighbours = D4_neighbourhood(prev_point, (width, height))
                next_point = min(neighbours, key=lambda p: abs(p[0] - current_point[0]) + abs(p[1] - current_point[1]))
                final_path.append(next_point)
                prev_point = next_point
            final_path.append(current_point)

        path = final_path
        
        # remove duplicates
        seen = set()
        path = [x for x in path if tuple(x) not in seen and not seen.add(tuple(x))]
        
        # clip points to the limits
        path = np.clip(path, 0, [width, height])
        
        return path
                