import numpy as np
import beaverbot.ral.algorithms.module_misc as module_misc

def exploration_gradient_DN(position, limits, local_map, N=4, home_base_store=None, eta=1, N_recovery=4, step=1):
    """Select next exploration position using gradient-based probabilistic selection (softmax with eta parameter)."""
    # Zero out home bases in local map
    local_map = local_map.copy()
    for home_base in home_base_store:
        if limits[0][1] >= home_base[0] and limits[1][1] >= home_base[1]:
            local_map[home_base[0]][home_base[1]] = 0
    
    max_vegetation = np.nanmax(local_map)
    neighbourhood_valid = False
    
    # Greedy mode: select high-quality vegetation directly
    if N == 0:
        neighbourhood = np.argwhere(local_map >= 0.9 * max_vegetation)
        if len(neighbourhood) > 1:
            neighbourhood_valid = True
            neighbourhood = [neighbourhood[np.random.choice(len(neighbourhood))]]
        else:
            N = N_recovery  # Fallback to gradient-based
    
    # Gradient-based mode: probabilistic selection using vegetation gradient
    if not neighbourhood_valid:
        neighbourhood = module_misc.DN_neighbourhood(position, limits, N, step=step)
        gradient_matrix, _ = module_misc.matrix_gradient(local_map, neighbourhood)
        
        # Convert matrix indices to direction vectors
        cx, cy = gradient_matrix.shape[0] // 2, gradient_matrix.shape[1] // 2
        directions = np.argwhere(~np.isnan(gradient_matrix))
        
        # Calculate possible positions and their gradient values
        new_positions, gradient_values = [], []
        for i, j in directions:
            dir_vec = [j - cx, cy - i]  # [dx, dy]
            pos = [position[0] + dir_vec[0], position[1] + dir_vec[1]]
            if pos != position:  # Exclude current position
                new_positions.append(pos)
                gradient_values.append(gradient_matrix[i, j])
        
        try:
            # Filter NaN values
            finite_mask = ~np.isnan(gradient_values)
            finite_values = np.array(gradient_values)[finite_mask]
            finite_positions = [new_positions[i] for i in range(len(new_positions)) if finite_mask[i]]
            
            # Softmax selection: higher gradient = higher probability
            scaled = finite_values * eta - np.max(finite_values * eta)
            probs = np.exp(scaled) / np.sum(np.exp(scaled))
            neighbourhood = [finite_positions[np.random.choice(len(finite_positions), p=probs)]]
        
        except (ValueError, IndexError):
            # Fallback: select nearby position
            distances = [np.linalg.norm(np.array(pos) - np.array(position)) for pos in neighbourhood]
            zero_idx = [i for i, d in enumerate(distances) if d == 0.0]
            if zero_idx:
                distances[zero_idx[0]] = np.inf  # Exclude current position
            max_dist = 2 * np.min(distances) if distances else 1.0
            valid_idx = [i for i, d in enumerate(distances) if 0 < d <= max_dist]
            neighbourhood = [neighbourhood[np.random.choice(valid_idx) if valid_idx else 0]]
    
    return neighbourhood, [False] * len(neighbourhood), 0