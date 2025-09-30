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

def exploration_DN(position, limits, N=4, home_base=None):
    """
    Generate deterministic neighborhood exploration targets.
    
    This function implements systematic exploration using geometric neighborhood patterns.
    It creates a deterministic set of exploration targets around the current position,
    ensuring complete local coverage without randomization.
    
    The strategy generates neighbors in a predictable order, making it ideal for
    systematic surveys and ensuring no local areas are missed during exploration.
    
    Parameters
    ----------
    position : list[int, int]
        Current robot position as [x, y] coordinates
    limits : list[list[int, int], list[int, int]]
        Environment boundaries as [[x_min, x_max], [y_min, y_max]]
    N : int, optional
        Neighborhood size determining exploration pattern (default: 4)
        Valid values: {0, 4, 8, 24, 40}
        - 0: Current position only
        - 4: Von Neumann neighborhood (4-connected)
        - 8: Moore neighborhood (8-connected)
        - 24: Extended local area
        - 40: Large neighborhood for comprehensive coverage
    home_base : list[int, int], optional
        Home base position to exclude from exploration targets (default: None)
        Used to avoid unnecessary revisits to storage locations
        
    Returns
    -------
    neighbourhood : list[list[int, int]]
        List of [x, y] coordinate pairs representing exploration targets
        Ordered systematically based on geometric neighborhood pattern
    neighbourhood_reached_flag : list[bool]
        Boolean flags indicating visit status for each target
        Initially all False, updated as targets are reached
    neighbourhood_current_index : int
        Starting index for systematic target progression (always 0)
        
    Notes
    -----
    Deterministic Exploration Characteristics:
    - Predictable and reproducible exploration patterns
    - Ensures complete local coverage without gaps
    - No randomization - same position always generates same targets
    - Suitable for methodical mapping and systematic surveys
    
    Home Base Handling:
    If home_base is specified and appears in the generated neighborhood,
    it is removed to prevent unnecessary exploration of storage locations.
    
    Boundary Clipping:
    Target coordinates are automatically clipped to environment boundaries
    by the underlying DN_neighbourhood function in module_misc.
    
    Examples
    --------
    >>> # Basic 4-neighborhood exploration
    >>> position = [10, 10]
    >>> limits = [[0, 20], [0, 20]]
    >>> targets, flags, index = exploration_DN(position, limits, N=4)
    >>> print(f"Targets: {targets}")  # [[9,10], [11,10], [10,9], [10,11]]
    
    >>> # Large neighborhood with home base exclusion
    >>> home = [12, 12]
    >>> targets, flags, index = exploration_DN(position, limits, N=8, home_base=home)
    >>> print(f"Generated {len(targets)} targets, home base excluded")
    
    See Also
    --------
    exploration_random_DN : Randomized version of deterministic exploration
    exploration_gradient_DN : Quality-driven exploration strategy
    module_misc.DN_neighbourhood : Underlying neighborhood generation function
    """
    _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)
    if home_base in _neighbourhood:
        _neighbourhood.remove(home_base)
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_random_DN(position, limits, N=4, home_base_store=None):
    """
    Generate randomized single-target exploration from deterministic neighborhood.
    
    This function implements stochastic exploration by randomly selecting one target
    from a deterministic neighborhood pattern. It combines the coverage benefits of
    systematic exploration with unpredictability to avoid behavioral patterns.
    
    The strategy generates the full deterministic neighborhood and then randomly
    selects one direction for exploration, making robot behavior less predictable
    while maintaining geometric locality.
    
    Parameters
    ----------
    position : list[int, int]
        Current robot position as [x, y] coordinates
    limits : list[list[int, int], list[int, int]]
        Environment boundaries as [[x_min, x_max], [y_min, y_max]]
    N : int, optional
        Neighborhood size for random selection (default: 4)
        Valid values: {0, 4, 8, 24, 40}
        Larger N provides more direction options for random selection
    home_base : list[int, int], optional
        Home base position to exclude from selection (default: None)
        Prevents random selection of storage locations
        
    Returns
    -------
    neighbourhood : list[list[int, int]]
        Single-element list containing one randomly selected [x, y] target
        Length is always 1 (unless no valid targets exist)
    neighbourhood_reached_flag : list[bool]
        Single-element boolean list for the selected target (initially [False])
    neighbourhood_current_index : int
        Always 0 since only one target is selected
        
    Notes
    -----
    Random Selection Characteristics:
    - Unpredictable exploration direction from current position
    - Maintains local neighborhood constraint (no long-distance jumps)
    - Prevents systematic biases in multi-robot scenarios
    - Good for distributed exploration with multiple agents
    
    Algorithm Steps:
    1. Generate full deterministic neighborhood using exploration_DN logic
    2. Remove home base if it exists in the neighborhood
    3. Randomly select one direction from available options
    4. Return single-target exploration setup
    
    Use Cases:
    - Multi-robot systems to prevent clustering
    - Avoiding predictable exploration patterns
    - Adding stochasticity to systematic exploration
    - Breaking ties when multiple equally good options exist
    
    Examples
    --------
    >>> # Random direction from 4-neighborhood
    >>> position = [5, 5]
    >>> limits = [[0, 10], [0, 10]]
    >>> targets, flags, index = exploration_random_DN(position, limits, N=4)
    >>> print(f"Selected target: {targets[0]}")  # One of: [4,5], [6,5], [5,4], [5,6]
    
    >>> # Random selection from larger neighborhood
    >>> targets, flags, index = exploration_random_DN(position, limits, N=8)
    >>> print(f"Random target from 8-neighborhood: {targets[0]}")
    
    See Also
    --------
    exploration_DN : Deterministic version with all neighborhood targets
    exploration_gradient_DN : Quality-biased exploration strategy
    """
    _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)
    if home_base_store in _neighbourhood:
        _neighbourhood.remove(home_base_store)
    direction = random.randint(0, len(_neighbourhood) - 1)
    _neighbourhood = [_neighbourhood[direction]]            
    _neighbourhood_reached_flag = [False]
    _neighbourhood_current_index = 0
    
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_gradient_DN(position, limits, local_map, N=4, home_base_store=None, max_vegetation=None, eta=1, N_recovery=4):
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
                _neighbourhood = [random.choice(_neighbourhood)]
            else:
                N = N_recovery
                
        if not _neighbourhood_valid:
            _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)
              
            #! GRADIENT CALCULATION
            gradient_matrix, values_matrix = module_misc.matrix_gradient(local_map, _neighbourhood)
            
            # Remove threshold filtering - consider ALL positive gradients
            all_indices = np.argwhere(gradient_matrix > 0)  # Only positive gradients                
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
                    # eta now serves purely as temperature: low eta = explore, high eta = exploit
                    
                    # Apply numerical stability to prevent overflow
                    scaled_values = eta * np.array(gradient_values)
                    # Subtract the maximum to prevent overflow (standard softmax trick)
                    scaled_values = scaled_values - np.max(scaled_values)
                    
                    gradient_probs = np.exp(scaled_values)
                    gradient_probs = gradient_probs / np.sum(gradient_probs)
                    
                    # Probabilistic selection weighted by gradient magnitude                    
                    selected_idx = np.random.choice(len(new_position), p=gradient_probs)                    
                    _neighbourhood = [new_position[selected_idx]]
                else:
                    # Only one option available
                    _neighbourhood = [new_position[0]]
            else:                
                _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
                _neighbourhood = [random.choice(_neighbourhood)]
                    
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index

def exploration_softmax_DN(position, limits, local_map, N=4, home_base_store=None, max_vegetation=None, eta=1, N_recovery=4):
    """
    Generate probabilistic exploration targets using softmax distribution on vegetation quality.
    
    This function implements stochastic exploration using softmax probability distributions
    computed from vegetation quality maps. It provides a probabilistic approach to quality-based
    exploration, balancing exploitation of high-quality areas with exploration diversity.
    
    The strategy converts vegetation quality values into probability distributions and uses
    these to guide exploration decisions, providing more nuanced behavior than hard gradient
    thresholds while maintaining quality-awareness.
    
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
        Neighborhood size for probability calculation (default: 4)
        Valid values: {0, 4, 8, 24, 40}
        - 0: Global probabilistic search mode
        - 4-40: Local probability-based neighborhood exploration
    home_base_store : list[list[int, int]], optional
        List of home base positions to exclude from exploration (default: None)
        These positions are set to 0 probability
    max_vegetation : list[float, float], optional
        [min_threshold, max_threshold] for vegetation quality filtering (default: None)
        Areas above max_threshold are set to 0 (over-exploited)
        If None, uses nanmax of local_map
    eta : float, optional
        Currently unused in probabilistic sampling implementation (default: 1)
        Maintained for API compatibility with gradient-based exploration
        In future versions, could be used as temperature parameter for softmax
    N_recovery : int, optional
        Fallback neighborhood size when probabilistic search fails (default: 4)
        Used when N=0 mode doesn't find suitable targets
        
    Returns
    -------
    neighbourhood : list[list[int, int]]
        Single-element list with probabilistically selected [x, y] exploration target
        Selection probability proportional to vegetation quality
    neighbourhood_reached_flag : list[bool]
        Single-element boolean list for tracking target visit status
    neighbourhood_current_index : int
        Always 0 since single target is selected
        
    Notes
    -----
    Softmax Probability Algorithm:
    
    1. **Map Preprocessing**:
       - Copy local map and exclude home base positions (set to 0)
       - Handle NaN values and apply vegetation thresholds
       - Set current position to 0 to avoid self-selection
       - Filter out over-exploited areas (> max_vegetation[1])
    
    2. **Probability Distribution**:
       - Compute softmax: P(x,y) = exp(quality(x,y)) / Σ exp(quality(i,j))
       - Creates probability distribution over entire map
       - Higher quality areas get higher selection probability
    
    3. **Mode Selection**:
       - N=0: Global probabilistic sampling from entire map according to softmax distribution
       - N>0: Local probabilistic sampling from neighborhood according to local probabilities
    
    4. **Local Exploration** (N>0 mode):
       - Generate neighborhood around current position
       - Extract probability values at neighborhood positions
       - Normalize neighborhood probabilities to sum to 1
       - Sample position according to normalized neighborhood distribution
    
    5. **Target Selection**:
       - True probabilistic sampling where selection probability ∝ vegetation quality
       - Higher quality areas have higher selection probability (not threshold-based)
       - Fallback to deterministic exploration if no valid probabilities exist
    
    Key Differences from Gradient Method:
    - Uses probability distributions instead of raw gradients
    - Softmax normalization provides global context awareness
    - More stochastic behavior with quality-proportional selection
    - Better handling of sparse high-quality areas
    
    Probability Temperature:
    The softmax naturally provides temperature-like behavior where:
    - High vegetation quality → High selection probability
    - Low vegetation quality → Low (but non-zero) selection probability
    - Zero vegetation quality → Zero selection probability
    
    Examples
    --------
    >>> # Probabilistic exploration with quality bias
    >>> position = [15, 15]
    >>> limits = [[0, 50], [0, 50]]
    >>> veg_map = np.random.rand(50, 50)
    >>> home_bases = [[25, 25]]
    >>> targets, flags, idx = exploration_softmax_DN(
    ...     position, limits, veg_map, N=8,
    ...     home_base_store=home_bases,
    ...     max_vegetation=[0.2, 0.9],
    ...     eta=0.8
    ... )
    >>> print(f"Probabilistically selected target: {targets[0]}")
    
    >>> # Global probabilistic search
    >>> targets, flags, idx = exploration_softmax_DN(
    ...     position, limits, veg_map, N=0,
    ...     eta=0.95,  # Very selective for high-quality areas
    ...     N_recovery=8
    ... )
    
    See Also
    --------
    exploration_gradient_DN : Deterministic gradient-based exploration
    exploration_random_DN : Pure random exploration strategy
    numpy.exp : Exponential function used in softmax calculation
    """
    
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
            # Global probabilistic sampling from the entire map
            # Flatten probabilities and create cumulative distribution
            flat_probs = probabilities.flatten()
            if np.sum(flat_probs) > 0:  # Ensure we have valid probabilities
                # Sample according to probability distribution
                flat_indices = np.arange(len(flat_probs))
                sampled_flat_idx = np.random.choice(flat_indices, p=flat_probs)
                
                # Convert back to 2D coordinates
                sampled_row = sampled_flat_idx // probabilities.shape[1]
                sampled_col = sampled_flat_idx % probabilities.shape[1]
                _neighbourhood = [[sampled_row, sampled_col]]
                _neighbourhood_valid = True
            else:
                N = N_recovery
                
        if not _neighbourhood_valid:
            
            _neighbourhood = module_misc.DN_neighbourhood(position, limits, N)                        
            gradient_matrix, values_matrix = module_misc.matrix_gradient(probabilities, _neighbourhood)
            
            # Extract probabilities at neighborhood positions
            neighborhood_probs = []
            valid_positions = []
            
            for pos in _neighbourhood:
                if (0 <= pos[0] < probabilities.shape[0] and 
                    0 <= pos[1] < probabilities.shape[1]):
                    prob_val = probabilities[pos[0], pos[1]]
                    neighborhood_probs.append(prob_val)
                    valid_positions.append(pos)
            
            if valid_positions and np.sum(neighborhood_probs) > 0:
                # Normalize neighborhood probabilities
                neighborhood_probs = np.array(neighborhood_probs)
                neighborhood_probs = neighborhood_probs / np.sum(neighborhood_probs)
                
                # Sample according to neighborhood probability distribution
                selected_idx = np.random.choice(len(valid_positions), p=neighborhood_probs)
                _neighbourhood = [valid_positions[selected_idx]]
            else:                
                _neighbourhood, _, _ = exploration_DN(position, limits, N=4)
                _neighbourhood = [random.choice(_neighbourhood)]
                    
    _neighbourhood_reached_flag = [False] * len(_neighbourhood)
    _neighbourhood_current_index = 0
            
    return _neighbourhood, _neighbourhood_reached_flag, _neighbourhood_current_index