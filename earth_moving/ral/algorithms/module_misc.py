
# imports
import numpy as np
import random
from scipy.interpolate import CubicSpline

# general measure mode
def measure(map, position, mode):
    if mode == 'D1':
        measure_positions = [position]
    elif mode == 'D4':
        measure_positions = measure_D4(map, position)
    elif mode == 'D8':
        measure_positions = measure_D8(map, position)    
    elif mode == 'gradient_D8':
        measure_positions = measure_D8(map, position)
        gradient_matrix, values_matrix = matrix_gradient(map, measure_positions)
        measure_positions = remove_negative_gradient(measure_positions[0], gradient_matrix, values_matrix)
    elif mode == 'D12':
        measure_positions = measure_D12(map, position)
    elif mode == 'D20':
        measure_positions = measure_D20(map, position)
    elif mode == 'gradient_D20':
        measure_positions = measure_D20(map, position)
        _, values_matrix = matrix_gradient(map, measure_positions)
        measure_positions, _ = find_monotonic_indices(values_matrix, position)
    else:
        raise ValueError('Invalid mode')
    
    measure_values = []
    for pos in measure_positions:
        measure_values.append(map[pos[0],pos[1]])
    
    return measure_positions, measure_values
    
# measure D4
def measure_D4(map, position):
    limits = get_map_limits(map)
    neighbourhood = D4_neighbourhood(position, limits)
    return neighbourhood

# measure D8
def measure_D8(map, position):
    limits = get_map_limits(map)
    neighbourhood = D8_neighbourhood(position, limits)
    return neighbourhood

# measure D12
def measure_D12(map, position):
    limits = get_map_limits(map)
    neighbourhood = D12_neighbourhood(position, limits)
    return neighbourhood

# measure D20
def measure_D20(map, position):
    limits = get_map_limits(map)
    neighbourhood = D20_neighbourhood(position, limits)
    return neighbourhood

# get map limits
def get_map_limits(map):
    return np.array([[0, map.shape[0]-1], [0, map.shape[1]-1]])    
    
# D4 neighbourhood
def D4_neighbourhood(position, limits, step=1) -> list:
    neighbourhood = [[position[0], position[1]],
                     [position[0], position[1] - step],
                     [position[0] + step, position[1]],
                     [position[0], position[1] + step],
                     [position[0] - step, position[1]]]
    
    neighbourhood = [pos for pos in neighbourhood if limits[0][0] <= pos[0] <= limits[0][1] and limits[1][0] <= pos[1] <= limits[1][1]]
    
    return neighbourhood

# D8 neighbourhood
def D8_neighbourhood(position, limits, step=1) -> list:
    neighbourhood = [[position[0], position[1]],
                     [position[0], position[1] - step],
                     [position[0] + step, position[1] - step],
                     [position[0] + step, position[1]],
                     [position[0] + step, position[1] + step],
                     [position[0], position[1] + step],
                     [position[0] - step, position[1] + step],
                     [position[0] - step, position[1]],
                     [position[0] - step, position[1] - step]]
    
    neighbourhood = [pos for pos in neighbourhood if limits[0][0] <= pos[0] <= limits[0][1] and limits[1][0] <= pos[1] <= limits[1][1]]
    
    return neighbourhood

# D12 neighbourhood
def D12_neighbourhood(position, limits, step=1) -> list:
    neighbourhood = [[position[0], position[1]],
                     [position[0], position[1] - step],
                     [position[0] + step, position[1] - step],
                     [position[0] + step, position[1]],
                     [position[0] + step, position[1] + step],
                     [position[0], position[1] + step],
                     [position[0] - step, position[1] + step],
                     [position[0] - step, position[1]],
                     [position[0] - step, position[1] - step],
                     [position[0] + 2*step, position[1]],
                     [position[0], position[1] + 2*step],
                     [position[0] - 2*step, position[1]], 
                     [position[0], position[1] - 2*step]]
    
    neighbourhood = [pos for pos in neighbourhood if limits[0][0] <= pos[0] <= limits[0][1] and limits[1][0] <= pos[1] <= limits[1][1]]
    
    return neighbourhood

# D20 neighbourhood
def D20_neighbourhood(position, limits, step=1) -> list:
    neighbourhood = [[position[0], position[1]],
                     [position[0], position[1] - step],
                     [position[0] + step, position[1] - step],
                     [position[0] + step, position[1]],
                     [position[0] + step, position[1] + step],
                     [position[0], position[1] + step],
                     [position[0] - step, position[1] + step],
                     [position[0] - step, position[1]],
                     [position[0] - step, position[1] - step],
                     [position[0] + 2*step, position[1]],
                     [position[0], position[1] + 2*step],
                     [position[0] - 2*step, position[1]], 
                     [position[0], position[1] - 2*step],
                     [position[0] + 2*step, position[1] + step],
                     [position[0] + 2*step, position[1] - step],
                     [position[0] - 2*step, position[1] + step],
                     [position[0] - 2*step, position[1] - step],
                     [position[0] + step, position[1] + 2*step],
                     [position[0] - step, position[1] + 2*step],
                     [position[0] + step, position[1] - 2*step],
                     [position[0] - step, position[1] - 2*step]]
    
    neighbourhood = [pos for pos in neighbourhood if limits[0][0] <= pos[0] <= limits[0][1] and limits[1][0] <= pos[1] <= limits[1][1]]
    
    return neighbourhood

# matrix gradient
def matrix_gradient(matrix, neighbours) -> np.array:
        
    # Determine the bounds of the submatrix
    min_x = min(neighbour[0] for neighbour in neighbours)
    max_x = max(neighbour[0] for neighbour in neighbours)
    min_y = min(neighbour[1] for neighbour in neighbours)
    max_y = max(neighbour[1] for neighbour in neighbours)

    # Create a padded submatrix filled with NaN
    values_matrix_size = max(max_x - min_x + 1, max_y - min_y + 1)
    dd = values_matrix_size // 2
        
    # create the gradient matrix    
    values_matrix = np.full((values_matrix_size,values_matrix_size), np.nan)
    gradient_matrix = np.full((values_matrix_size,values_matrix_size), np.nan)
    center_x, center_y = values_matrix_size // 2, values_matrix_size // 2  # Center of the submatrix corresponds to the position     
    global_center_x, global_center_y = min_x + center_x, min_y + center_y  # Center of the matrix corresponds to the position    
    
    # populate the center of the matrix
    gradient_matrix[center_x, center_y] = 0
    values_matrix[center_x, center_y] = matrix[global_center_x, global_center_y]

    for neighbour in neighbours:
        x, y = neighbour
        if 0 <= x < matrix.shape[0] and 0 <= y < matrix.shape[1]:
            dx, dy = x - global_center_x, y - global_center_y
            if -dd <= dx <= dd and -dd <= dy <= dd:  # Ensure neighbour fits in the 3x3 matrix
                if not np.isnan(matrix[x, y]):
                    values_matrix[center_x + dx, center_y + dy] = matrix[x, y]
                    if not np.isnan(matrix[global_center_x, global_center_y]):
                        gradient_matrix[center_x + dx, center_y + dy] = matrix[x, y] - matrix[global_center_x, global_center_y]
                                            

    return gradient_matrix, values_matrix

# remove from neighbourhood the values that have a negative gradient
def remove_negative_gradient(neighbours_center, gradient_matrix, values_matrix):
        
    gradient_matrix = np.array(gradient_matrix)
    values_matrix = np.array(values_matrix)
    
    # Get the coordinates of the non-NaN values in the gradient matrix
    non_nan_coords = np.argwhere(~np.isnan(gradient_matrix))
    
    # Create a mask to filter out negative gradients
    mask = gradient_matrix[non_nan_coords[:, 0], non_nan_coords[:, 1]] >= 0
    
    # Get the filtered coordinates
    filtered_coords = non_nan_coords[mask]
    
    # Convert back to original coordinates  
    dd = values_matrix.shape[0] // 2                  
    filtered_neighbours = neighbours_center + filtered_coords - dd
    
    return filtered_neighbours.tolist()

# find monotonic indices
def find_monotonic_indices(values_matrix, center):
    
    directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    monotonic_indices = []
    monotonic_indices_global = []
    
    dd = values_matrix.shape[0] // 2  

    center_value = values_matrix[dd][dd]
    for dx, dy in directions:
        x, y = dd, dd
        prev_value = center_value
        direction_indices = []

        while True:
            x += dx
            y += dy

            # Check bounds
            if x < 0 or y < 0 or x >= values_matrix.shape[0] or y >= values_matrix.shape[1]:
                break

            current_value = values_matrix[x, y]

            # Stop if NaN or sequence breaks
            if np.isnan(current_value) or (current_value < prev_value and current_value > prev_value):
                break

            # Add to the sequence
            direction_indices.append((x, y))
            prev_value = current_value

        # Add the valid indices for this direction
        if direction_indices:
            monotonic_indices.extend(np.array(direction_indices))

    monotonic_indices.extend(np.array([[dd, dd]]))
    monotonic_indices_global = monotonic_indices + np.array(center) - dd
    monotonic_matrix = np.full(values_matrix.shape, np.nan)
    for idx in monotonic_indices:
        monotonic_matrix[idx[0]][idx[1]] = values_matrix[idx[0]][idx[1]]
        
    return monotonic_indices_global, monotonic_matrix

# scale sigma
def scale_sigma(sigma, cluster_radius) -> float:
        return (cluster_radius + sigma - 1)/sigma
    
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
        
        # Create splines for each pair of points
        splines = []
        for i in range(len(points) - 1):
            x_pair = points[i:i + 2, 0]
            y_pair = points[i:i + 2, 1]
            t_pair = np.linspace(0, 1, len(x_pair))
            spline_x = np.poly1d(np.polyfit(t_pair, x_pair, 1))
            spline_y = np.poly1d(np.polyfit(t_pair, y_pair, 1))
            splines.append((spline_x, spline_y))
            
        # Merge all splines into a single spline
        t_values = np.linspace(0, len(points) - 1, (len(points) -1) * 100)
        merged_x = []
        merged_y = []
        for i, (spline_x, spline_y) in enumerate(splines):
            t_local = np.linspace(i, i + 1, 100)
            merged_x.extend(spline_x(t_local - i))
            merged_y.extend(spline_y(t_local - i))
        
        # Create a single spline from the merged points
        spline_x = np.poly1d(np.polyfit(t_values, merged_x, 4))
        spline_y = np.poly1d(np.polyfit(t_values, merged_y, 4))

        # Parameterize the points
        # t = np.linspace(0, 1, len(points))
        # spline_x = np.poly1d(np.polyfit(t, x, 3))
        # spline_y = np.poly1d(np.polyfit(t, y, 3))
        
        # Generate points along the spline
        num_points = 500  # Number of points to generate along the spline
        t_values = np.linspace(0, len(points) - 1, num_points)
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
            limits = np.array([[0, width], [0, height]])
            neighbours = np.asarray(D4_neighbourhood(prev_point, limits))
            while not any((current_point == np.array(neighbour)).all() for neighbour in neighbours):
                # Find the next neighbour closer to the current_point
                neighbours = np.asarray(D4_neighbourhood(prev_point, limits))
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


    


    