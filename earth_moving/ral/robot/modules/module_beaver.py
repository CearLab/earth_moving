# general imports
import random 
import numpy as np

# backend imports

# module imports
import earth_moving.ral.algorithms.module_misc as module_misc

def exploration_DN(position, limits, N=4, home_base=None):
    _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)
    if home_base in _neighbourhood:
        _neighbourhood.remove(home_base)
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_random_DN(position, limits, N=4, home_base=None):
    _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)
    if home_base in _neighbourhood:
        _neighbourhood.remove(home_base)
    direction = random.randint(0, len(_neighbourhood) - 1)
    _neighbourhood = [_neighbourhood[direction]]            
    _neighbourhood_reached_flag = [False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_gradient_DN(position, limits, local_vegetation_map, N=4, home_base_store=None):
    
    local_vegetation_map = local_vegetation_map.copy()
    for home_base in home_base_store:
        if limits[0][1] >= home_base[0] and limits[1][1] >= home_base[1]:
            local_vegetation_map[home_base[0]][home_base[1]] = 0    
    
    if local_vegetation_map is None:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)        
    elif position[0] == limits[0][1] or position[1] == limits[1][1]:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)        
    else:                
        
        if N == 0:
            _neighbourhood = np.argwhere(local_vegetation_map == np.nanmax(local_vegetation_map))            
            _neighbourhood = [min(_neighbourhood, key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]            
        else:
            _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)            
            
            gradient_matrix, values_matrix = module_misc.matrix_gradient(local_vegetation_map, _neighbourhood)
        
            if np.nanmax(gradient_matrix) < 0.0:
                _neighbourhood = [min(_neighbourhood, key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]                
            else:        
                max_indices = np.argwhere(gradient_matrix == np.nanmax(gradient_matrix))
                direction = max_indices.tolist()
                
                dx = []
                dy = []
                for dir in direction:
                    dx.append(dir[1] - 1)  # column index - center column
                    dy.append(1 - dir[0])  # row index - center row
                    
                new_position = []
                for i, dir in enumerate(direction):  
                    possible_position = [position[0] + dx[i], position[1] + dy[i]]
                    if possible_position != position: 
                        new_position.append(possible_position)
                
                if new_position:
                    _neighbourhood = [min(new_position, key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]                    
                else:                
                    _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
                    
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index