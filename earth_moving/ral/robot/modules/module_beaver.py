# general imports
import random 
import numpy as np

# backend imports

# module imports
import ral.robot.modules.module_misc as module_misc

def exploration_D4(position, limits):
    _neighbourhood = module_misc.D4_neighbourhood_cycle(position, limits)
    _neighbourhood_reached_flag = [False, False, False, False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_D8(position, limits):
    _neighbourhood = module_misc.D8_neighbourhood_cycle(position, limits)
    _neighbourhood_reached_flag = [False, False, False, False, False, False, False, False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_D4_random(position, limits):
    D4_neighbourhood = module_misc.D4_neighbourhood(position, limits)     
    direction = random.randint(0, len(D4_neighbourhood) - 1)
    _neighbourhood = [D4_neighbourhood[direction]]            
    _neighbourhood_reached_flag = [False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_D8_random(position, limits):
    D8_neighbourhood = module_misc.D8_neighbourhood(position, limits)     
    direction = random.randint(0, len(D8_neighbourhood) - 1)
    _neighbourhood = [D8_neighbourhood[direction]]            
    _neighbourhood_reached_flag = [False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_gradient_D4(position, limits, local_vegetation_map, position_store):
    if local_vegetation_map is None:
        _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D4(position, limits)        
    elif position[0] == limits[0][1] or position[1] == limits[1][1]:
        _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D4(position, limits)
    else:
        D4_neighbourhood = module_misc.D4_neighbourhood(position, limits)     
        gradient_matrix, values_matrix = module_misc.matrix_gradient(local_vegetation_map, D4_neighbourhood, position)
        
        # get the last different position from current
        last_different_position = None
        for pos in reversed(position_store):
            if pos != position:
                last_different_position = pos
                break
            
        possible_unseen_neighbours = [
            pos for pos in D4_neighbourhood
            if pos != last_different_position and \
            pos != position
        ]
        
        if np.nanmax(gradient_matrix) < 0.0:
            if possible_unseen_neighbours:
                _neighbourhood = [random.choice(possible_unseen_neighbours)]
            else:
                _neighbourhood = [random.choiche(D4_neighbourhood)]
            _neighbourhood_reached_flag = [False]
            _neighbourhood_current_index = 0
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
                _neighbourhood = [random.choice(new_position)]
                _neighbourhood_reached_flag = [False]
                _neighbourhood_current_index = 0
            elif possible_unseen_neighbours:
                _neighbourhood = [random.choice(possible_unseen_neighbours)]
                _neighbourhood_reached_flag = [False]
                _neighbourhood_current_index = 0
            else:
                _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D4(position, limits)            
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_gradient_D8(position, limits, local_vegetation_map, position_store):
    if local_vegetation_map is None:
        _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D8(position, limits)        
    elif position[0] == limits[0][1] or position[1] == limits[1][1]:
        _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D8(position, limits)
    else:
        D8_neighbourhood = module_misc.D8_neighbourhood(position, limits)     
        gradient_matrix, values_matrix = module_misc.matrix_gradient(local_vegetation_map, D8_neighbourhood, position)
        
        # get the last different position from current
        last_different_position = None
        for pos in reversed(position_store):
            if pos != position:
                last_different_position = pos
                break
            
        possible_unseen_neighbours = [
            pos for pos in D8_neighbourhood
            if pos != last_different_position and \
            pos != position
        ]
        
        if np.nanmax(gradient_matrix) < 0.0:
            if possible_unseen_neighbours:
                _neighbourhood = [random.choice(possible_unseen_neighbours)]
            else:
                _neighbourhood = [random.choiche(D8_neighbourhood)]
            _neighbourhood_reached_flag = [False]
            _neighbourhood_current_index = 0
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
                _neighbourhood = [random.choice(new_position)]
                _neighbourhood_reached_flag = [False]
                _neighbourhood_current_index = 0
            elif possible_unseen_neighbours:
                _neighbourhood = [random.choice(possible_unseen_neighbours)]
                _neighbourhood_reached_flag = [False]
                _neighbourhood_current_index = 0
            else:
                _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index = exploration_D8(position, limits)
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index