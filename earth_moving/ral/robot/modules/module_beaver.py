"""
Beaver Robot Exploration Strategies Module
==========================================

This module implements various exploration strategies for beaver-like autonomous robots
performing vegetation management tasks. The strategies balance exploration-exploitation
trade-offs while considering environmental constraints and resource optimization.

The module provides four distinct exploration algorithms:
1. Deterministic Neighborhood (DN) - Systematic geometric exploration
2. Random Deterministic (random_DN) - Randomized systematic exploration  
3. Gradient-based (gradient_DN) - Quality-driven exploration with gradient ascent
4. Softmax (softmax_DN) - Probabilistic exploration using softmax distributions

Each strategy generates a neighborhood of target positions for robot exploration,
along with management flags for tracking visit status and execution order.

Key Features:
- Home base avoidance to prevent unnecessary revisits to storage locations
- Boundary handling for edge cases and environment limits
- Adaptive fallback mechanisms when primary strategies fail
- Configurable neighborhood sizes for different exploration scales
- Quality-based optimization for resource-rich environment exploitation

Dependencies:
- numpy: Numerical computations and array operations
- random: Stochastic decision making
- module_misc: Utility functions for neighborhood generation and gradient calculations

Usage:
------
>>> position = [10, 10]
>>> limits = [[0, 50], [0, 50]]
>>> targets, flags, index = exploration_DN(position, limits, N=8)
>>> print(f"Generated {len(targets)} exploration targets")

>>> # For gradient-based exploration
>>> local_map = np.random.rand(50, 50)  # Example vegetation quality map
>>> targets, flags, index = exploration_gradient_DN(
...     position, limits, local_map, N=8, 
...     max_vegetation=[0.3, 0.8], eta=0.5
... )

Notes:
------
All exploration functions return a consistent tuple format:
- neighbourhood: List of [x, y] coordinate pairs for exploration targets
- neighbourhood_reached_flag: Boolean list indicating visit status  
- neighbourhood_current_index: Starting index for systematic progression

The strategies are designed to work with the BeaversRobotBackend class and
integrate seamlessly with the robot's finite state machine architecture.
"""

# general imports
import random 
import numpy as np

# backend imports

# module imports
import earth_moving.ral.algorithms.module_misc as module_misc

def exploration_gradient_DN(position, limits, local_map, N=4, home_base_store=None, eta=1, N_recovery=4, step=1):
    """
    Generate gradient-based exploration targets optimized for vegetation quality.
    
    This function implements intelligent exploration using gradient ascent on vegetation
    quality maps. It directs robots toward areas with higher vegetation density while
    avoiding over-exploited regions and home base locations.
    
    The strategy combines local gradient information with neighborhood constraints to
    find optimal exploration directions that maximize resource discovery potential.
    
    Parameters
    ----------
    position : list[int, int]
        Current robot position as [x, y] coordinates
    limits : list[list[int, int], list[int, int]]
        Environment boundaries as [[x_min, x_max], [y_min, y_max]]
    local_map : numpy.ndarray
        2D array representing vegetation quality distribution
        Values typically range from 0 (no vegetation) to 1 (high quality)
        NaN values indicate unexplored areas
    N : int, optional
        Neighborhood size for gradient calculation (default: 4)
        Valid values: {0, 4, 8, 24, 40}
        - 0: Global search mode for high-quality areas
        - 4-40: Local gradient-based exploration
    home_base_store : list[list[int, int]], optional
        List of home base positions to exclude from exploration (default: None)
        Prevents exploration of storage/depot locations
    max_vegetation : list[float, float], optional
        [min_threshold, max_threshold] for vegetation quality filtering (default: None)
        Areas above max_threshold are set to 0 (over-exploited)
        If None, uses nanmax of local_map
    eta : float, optional
        Softmax temperature parameter controlling exploration vs exploitation (default: 1)
        Low values (0.1-0.5): More exploratory, uniform probability distribution
        High values (1.5-3.0): More exploitative, concentrated on best gradients
        eta = 1.0: Balanced behavior between exploration and exploitation
    N_recovery : int, optional
        Fallback neighborhood size when gradient search fails (default: 4)
        Used when N=0 mode doesn't find suitable targets
        
    Returns
    -------
    neighbourhood : list[list[int, int]]
        Single-element list with probabilistically selected [x, y] exploration target
        Selection probability proportional to gradient magnitude and eta temperature
    neighbourhood_reached_flag : list[bool]
        Single-element boolean list for tracking target visit status
    neighbourhood_current_index : int
        Always 0 since single target is selected
        
    Notes
    -----
    Gradient-Based Algorithm:
    
    1. **Map Preprocessing**:
       - Copy local map to avoid modifying original
       - Set home base positions to 0 (avoid revisiting)
       - Handle NaN values and apply vegetation thresholds
       - Filter out over-exploited areas (> max_vegetation[1])
    
    2. **Mode Selection**:
       - N=0: Global search for high-quality areas (≥ eta * max_threshold)
       - N>0: Local gradient-based neighborhood exploration with probabilistic selection
    
    3. **Gradient Calculation** (N>0 mode):
       - Generate neighborhood around current position
       - Compute gradient matrix using module_misc.matrix_gradient()
       - Consider ALL positive gradient directions (no threshold filtering)
       - Convert gradient directions to new position candidates
    
    4. **Probabilistic Target Selection**:
       - Convert gradient magnitudes to selection probabilities using softmax
       - Apply eta as temperature: P(direction) ∝ exp(eta * gradient_magnitude)
       - Low eta: More uniform probabilities (exploratory)
       - High eta: More concentrated probabilities (exploitative)
       - Probabilistic sampling weighted by gradient strength and temperature
       - Fallback to random deterministic exploration if no positive gradients
    
    Fallback Mechanisms:
    - No local map: Random deterministic exploration
    - Edge position: Random deterministic exploration  
    - No valid gradients: Random selection from basic neighborhood
    - Global mode failure: Switches to N_recovery local mode
    
    Quality Optimization:
    The strategy balances exploitation (following gradients to high-quality areas)
    with exploration (maintaining spatial coverage) through probabilistic selection.
    The eta parameter serves as a softmax temperature controlling this balance:
    - Low eta (0.1-0.5): Uniform exploration across all positive gradients
    - High eta (1.5-3.0): Strong preference for steepest gradients
    - eta = 1.0: Natural exponential weighting of gradient magnitudes
    
    Examples
    --------
    >>> # Exploratory gradient behavior (low temperature)
    >>> position = [10, 10]
    >>> limits = [[0, 50], [0, 50]]
    >>> veg_map = np.random.rand(50, 50)  # Example vegetation map
    >>> home_bases = [[25, 25], [30, 30]]
    >>> targets, flags, idx = exploration_gradient_DN(
    ...     position, limits, veg_map, N=8,
    ...     home_base_store=home_bases,
    ...     max_vegetation=[0.3, 0.8],
    ...     eta=0.3  # Low temperature = more exploration
    ... )
    >>> print(f"Exploratory target: {targets[0]}")
    
    >>> # Exploitative gradient behavior (high temperature)
    >>> targets, flags, idx = exploration_gradient_DN(
    ...     position, limits, veg_map, N=8,
    ...     home_base_store=home_bases,
    ...     max_vegetation=[0.3, 0.8],
    ...     eta=2.0  # High temperature = focus on best gradients
    ... )
    >>> print(f"Exploitative target: {targets[0]}")
    
    >>> # Global search mode for high-quality areas
    >>> targets, flags, idx = exploration_gradient_DN(
    ...     position, limits, veg_map, N=0,
    ...     eta=0.9,  # Still used for global search threshold
    ...     N_recovery=8
    ... )
    
    See Also
    --------
    exploration_softmax_DN : Probabilistic exploration based on vegetation quality
    exploration_DN : Basic deterministic exploration
    module_misc.matrix_gradient : Gradient calculation utility
    """
    
    local_map = local_map.copy()
    for home_base in home_base_store:
        if limits[0][1] >= home_base[0] and limits[1][1] >= home_base[1]:
            local_map[home_base[0]][home_base[1]] = 0    
    
    max_vegetation = np.nanmax(local_map)  
    local_map = np.nan_to_num(local_map, nan=0.0)
    local_map[local_map > max_vegetation] = 0.0

    _neighbourhood_valid = False

    if N == 0:
        _neighbourhood = np.argwhere(local_map >= 0.9 * max_vegetation)
        if len(_neighbourhood) > 1:
            _neighbourhood_valid = True            
            selected_idx = np.random.choice(len(_neighbourhood))
            _neighbourhood = [_neighbourhood[selected_idx]]
        else:
            N = N_recovery
            
    if not _neighbourhood_valid:
        _neighbourhood = module_misc.DN_neighbourhood(position, limits, N, step=step)

        #! GRADIENT CALCULATION
        gradient_matrix, values_matrix = module_misc.matrix_gradient(local_map, _neighbourhood)
        
        # Remove threshold filtering - consider ALL positive gradients
        all_indices = np.argwhere(~np.isnan(gradient_matrix))
        direction = all_indices.tolist()
        
        #! CREATE A PROBABILISTIC DISRIBUTION BASED ON GRADIENT MAGNITUDE
        dir_matrix = [[[] for _ in range(values_matrix.shape[1])] for _ in range(values_matrix.shape[0])]
        cx = values_matrix.shape[0] // 2
        cy = values_matrix.shape[1] // 2
        for dir in direction:
            dir_matrix[dir[0]][dir[1]] = [dir[1] - cx, cy - dir[0]]  # column index - center column, row index - center row                
            
        new_position_matrix = [[[] for _ in range(values_matrix.shape[1])] for _ in range(values_matrix.shape[0])]
        new_position = []
        gradient_values = []  # Store corresponding gradient values for probabilistic selection
        
        for i in range(values_matrix.shape[0]):
            for j in range(values_matrix.shape[1]):
                if dir_matrix[i][j]:  # Check if the direction is not empty
                    possible_position = [position[0] + dir_matrix[i][j][0], position[1] + dir_matrix[i][j][1]]
                    if possible_position[0] != position[0] or possible_position[1] != position[1] : 
                        new_position_matrix[i][j] = possible_position
                        new_position.append(possible_position)
                        gradient_values.append(gradient_matrix[i][j])  # Store gradient magnitude            
        
        if new_position:
            # Probabilistic selection based on gradient magnitudes
            if len(gradient_values) > 1:                
                   
                # Convert gradient magnitudes to probabilities using softmax
                scaled_values = eta * np.array(gradient_values)                
                scaled_values = scaled_values - np.max(scaled_values)                
                gradient_probs = np.exp(scaled_values)
                gradient_probs = gradient_probs / np.sum(gradient_probs)                                
                selected_idx = np.random.choice(len(new_position), p=gradient_probs)
                
                # Select the closest position among those with maximum gradient value
                # max_value = np.max(gradient_values)
                # max_indices = [i for i, v in enumerate(gradient_values) if v == max_value]
                # max_positions = [new_position[i] for i in max_indices]
                # distances = [np.linalg.norm(np.array(pos) - np.array(position)) for pos in max_positions]
                # min_distance = np.min(distances)
                # closest_indices = [i for i, d in enumerate(distances) if d == min_distance]
                # selected_idx_in_max = np.random.choice(closest_indices)
                # selected_idx = max_indices[selected_idx_in_max]
                
                # def _neighbourhood
                _neighbourhood = [new_position[selected_idx]]
            else:
                # Select the neighbourhood position closest to current position
                distances = [np.linalg.norm(np.array(pos) - np.array(position)) for pos in _neighbourhood]
                min_distance = np.min(distances)
                closest_indices = [i for i, d in enumerate(distances) if d == min_distance]
                selected_idx = np.random.choice(closest_indices)
                _neighbourhood = [_neighbourhood[selected_idx]]        
            
                
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
        
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index