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

def exploration_gradient_DN(position, limits, local_map, N=4, home_base_store=None, max_vegetation=None, eta=1, N_recovery=4):
    
    local_map = local_map.copy()
    for home_base in home_base_store:
        if limits[0][1] >= home_base[0] and limits[1][1] >= home_base[1]:
            local_map[home_base[0]][home_base[1]] = 0    
    
    if local_map is None:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
        np.random.shuffle(_neighbourhood)       
    elif position[0] == limits[0][1] or position[1] == limits[1][1]:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
        np.random.shuffle(_neighbourhood)       
    else:         
        
        if max_vegetation is None:
            max_vegetation = np.nanmax(local_map)  
        local_map = np.nan_to_num(local_map, nan=0.0)
        local_map[local_map > max_vegetation[1]] = 0.0 
        
        _neighbourhood_valid = False
        
        if N == 0:            
            _neighbourhood = np.argwhere(local_map >= eta * max_vegetation[1])            
            if len(_neighbourhood) > 1:
                _neighbourhood_valid = True
                np.random.shuffle(_neighbourhood)
                try:
                    _neighbourhood = [min((pos for pos in _neighbourhood if not np.array_equal(pos, position) \
                                        and (np.linalg.norm(np.array(pos) - np.array(position)) < 2)),
                                        key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]                    
                except:
                    pass
            else:
                N = N_recovery
                
        if not _neighbourhood_valid:
            _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)          
            
            gradient_matrix, values_matrix = module_misc.matrix_gradient(local_map, _neighbourhood)
        
            if np.nanmax(gradient_matrix) < 0.0:
                try:
                    _neighbourhood = [min(  (pos for pos in _neighbourhood if (not np.array_equal(pos, position)) \
                        and (np.linalg.norm(np.array(pos) - np.array(position)) < 2)),
                        key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]                    
                except:
                    pass
            else:        
                max_indices = np.argwhere(gradient_matrix == np.nanmax(gradient_matrix))
                direction = max_indices.tolist()
                
                dx = []
                dy = []
                cx = values_matrix.shape[0] // 2
                cy = values_matrix.shape[1] // 2
                for dir in direction:
                    dx.append(dir[1] - cx)  # column index - center column
                    dy.append(cy - dir[0])  # row index - center row
                    
                new_position = []
                for i, dir in enumerate(direction):  
                    possible_position = [position[0] + dx[i], position[1] + dy[i]]
                    if possible_position[0] != position[0] and possible_position[1] != position[1]: 
                        new_position.append(possible_position)
                
                if new_position:
                    _neighbourhood = [random.choice(new_position)]                 
                else:                
                    _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
                    np.random.shuffle(_neighbourhood)
                    
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_softmax_DN(position, limits, local_map, N=4, home_base_store=None, max_vegetation=None, eta=1, N_recovery=4):
    
    local_map = local_map.copy()
    for home_base in home_base_store:
        if limits[0][1] >= home_base[0] and limits[1][1] >= home_base[1]:
            local_map[home_base[0]][home_base[1]] = 0    
    
    if local_map is None:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
        np.random.shuffle(_neighbourhood)    
    elif position[0] == limits[0][1] or position[1] == limits[1][1]:
        _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
        np.random.shuffle(_neighbourhood)        
    else:      
        
        if max_vegetation is None:
            max_vegetation = np.nanmax(local_map)         
        local_map = np.nan_to_num(local_map, nan=0.0)
        local_map[local_map > max_vegetation[1]] = 0.0
        
        #! only in softmax
        local_map[position[0]][position[1]] = 0.0
        
        _neighbourhood_valid = False
        
        probabilities = np.exp(local_map.flatten()) / np.sum(np.exp(local_map.flatten()))
        probabilities = probabilities.reshape(local_map.shape)                     
        
        if N == 0:
            _neighbourhood = np.argwhere(probabilities >= eta * np.nanmax(probabilities))
            if len(_neighbourhood) > 1:
                _neighbourhood_valid = True
                np.random.shuffle(_neighbourhood)
                try:
                    _neighbourhood = [min((pos for pos in _neighbourhood if not np.array_equal(pos, position) \
                                        and (np.linalg.norm(np.array(pos) - np.array(position)) < 2)),
                                        key=lambda pos: np.linalg.norm(np.array(pos) - np.array(position)))]                    
                except:
                    pass
            else:
                N = N_recovery
                
        if not _neighbourhood_valid:
            
            _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)                        
            gradient_matrix, values_matrix = module_misc.matrix_gradient(probabilities, _neighbourhood)
                    
            max_indices = np.argwhere(values_matrix >= np.nanmax(values_matrix))
            direction = max_indices.tolist()
            
            dx = []
            dy = []
            cx = values_matrix.shape[0] // 2
            cy = values_matrix.shape[1] // 2
            for dir in direction:
                dx.append(dir[1] - cx)  # column index - center column
                dy.append(cy - dir[0])  # row index - center row
                
            new_position = []
            for i, dir in enumerate(direction):  
                possible_position = [position[0] + dx[i], position[1] + dy[i]]
                if possible_position[0] != position[0] and possible_position[1] != position[1]: 
                    new_position.append(possible_position)
            
            if new_position:
                _neighbourhood = [random.choice(new_position)]
            else:                
                _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
                np.random.shuffle(_neighbourhood)
                    
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index