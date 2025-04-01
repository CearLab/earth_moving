
# imports
import numpy as np

# general measure mode
def measure(map, position, mode):
    if mode == 'D1':
        measure_positions = [position]
    elif mode == 'D4':
        measure_positions = measure_D4(map, position)
    elif mode == 'D8':
        measure_positions = measure_D8(map, position)    
    elif mode == 'D12':
        measure_positions = measure_D12(map, position)
    elif mode == 'D20':
        measure_positions = measure_D20(map, position)
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


    


    