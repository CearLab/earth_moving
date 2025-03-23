# general imports
import numpy as np

# generate D4 neighbourhood
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

def D4_neighbourhood_cycle(position, limits, step=1) -> list:
    D4_neighbourhood = [[position[0], position[1] - step],
                        [position[0] + step, position[1]],
                        [position[0], position[1] + step],
                        [position[0] - step, position[1]],
                        [position[0], position[1]]]
    
    return D4_neighbourhood

def D8_neighbourhood_cycle(position, limits, step=1) -> list:
    D8_neighbourhood = [[position[0], position[1] - step],
                        [position[0] + step, position[1] - step],
                        [position[0] + step, position[1]],
                        [position[0] + step, position[1] + step],
                        [position[0], position[1] + step],
                        [position[0] - step, position[1] + step],
                        [position[0] - step, position[1]],
                        [position[0] - step, position[1] - step],
                        [position[0], position[1]]]    
    
    return D8_neighbourhood

# matrix gradient
def matrix_gradient(matrix, neighbours, position) -> np.array:    

    gradient_matrix = np.full((3, 3), np.nan)  # Initialize a 3x3 matrix
    values_matrix = np.full((3, 3), np.nan)  # Initialize a 3x3 matrix
    center_x, center_y = 1, 1  # Center of the 3x3 matrix corresponds to the position
    
    # populate the center of the matrix
    gradient_matrix[center_x, center_y] = 0
    values_matrix[center_x, center_y] = matrix[position[0], position[1]]

    for neighbour in neighbours:
        x, y = neighbour
        if 0 <= x < matrix.shape[0] and 0 <= y < matrix.shape[1]:
            dx, dy = x - position[0], y - position[1]
            if -1 <= dx <= 1 and -1 <= dy <= 1:  # Ensure neighbour fits in the 3x3 matrix
                if not np.isnan(matrix[x, y]):
                    values_matrix[center_x + dx, center_y + dy] = matrix[x, y]
                    if not np.isnan(matrix[position[0], position[1]]):
                        gradient_matrix[center_x + dx, center_y + dy] = matrix[x, y] - matrix[position[0], position[1]]
                                            

    return gradient_matrix, values_matrix
