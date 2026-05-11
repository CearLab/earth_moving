import numpy as np
import matplotlib.pyplot as plt
import cv2
import copy


class PushEnvironment:
    """
    Original PushEnvironment with MCTS compatibility methods added.
    This maintains your original boundary-based physics and logic.
    """
    def __init__(self, boundary_points, delta_s_options, theta_options, push_width, initial_pos, max_pushes, coverage_thresh, occupancy_grid, total_occupied, total_pixels_in_cell=0.0, stochasticity=False, grid_size=20, action_prune=True):
        self.boundary_points = boundary_points
        self.delta_s_options = delta_s_options
        self.theta_options = theta_options
        self.push_width = push_width
        self.boundary_len = len(boundary_points)
        self.max_pushes = max_pushes
        self.coverage_thresh = coverage_thresh
        # self.BB_SIZE = 500                                 # Size of the bounding box in real-world units mm
        self.BB_SIZE = 0.5                                   # Size of the bounding box in real-world units meters
        self.grid_size = grid_size
        self.grid_res = 1.0 / grid_size
        self.grid_real_res = self.BB_SIZE * self.grid_res  # Real-world size of each grid cell
        self.grid_offset = np.array([0.0, 0.0])            # Assuming grid starts at (0,0) in world coordinates
        self.stochastic_push = stochasticity
        self.total_pixels_in_cell = total_pixels_in_cell  # Only used if stochastic_push is True
        self.action_prune = action_prune

        if self.stochastic_push:
            print("Stochastic push enabled.")
        else:
            print("Deterministic push enabled.")

        self.total_occupied = total_occupied

        self.initial_state = {
            "s_idx": initial_pos,
            "grid_mask": occupancy_grid.copy(),
            "num_pushes": 0,
            "length": 0.0,
            "len_pushes": 0.0,
            "len_path": 0.0
        }

        self.shovel_type = 'flat'  # Default shovel type
        # self.shovel_type = 'angled_left_25'
        self.current_state = self.initial_state.copy()
        self.action_space = [(delta_s, theta) for delta_s in delta_s_options for theta in theta_options]

    def reset(self):
        self.current_state = self.initial_state.copy()
        return self.current_state

    # MCTS Compatibility Methods
    def get_copy(self):
        """Create independent copy for MCTS simulation.
        Only copies mutable state; shares immutable data for speed.
        """
        new_env = object.__new__(PushEnvironment)
        
        # Share immutable/read-only data (no deepcopy needed)
        new_env.boundary_points = self.boundary_points
        new_env.delta_s_options = self.delta_s_options
        new_env.theta_options = self.theta_options
        new_env.push_width = self.push_width
        new_env.boundary_len = self.boundary_len
        new_env.max_pushes = self.max_pushes
        new_env.coverage_thresh = self.coverage_thresh
        new_env.BB_SIZE = self.BB_SIZE
        new_env.grid_size = self.grid_size
        new_env.grid_res = self.grid_res
        new_env.grid_real_res = self.grid_real_res
        new_env.grid_offset = self.grid_offset
        new_env.stochastic_push = self.stochastic_push
        new_env.total_pixels_in_cell = self.total_pixels_in_cell
        new_env.total_occupied = self.total_occupied
        new_env.initial_state = self.initial_state
        new_env.shovel_type = self.shovel_type
        new_env.action_space = self.action_space
        
        # Copy mutable state (only grid_mask array needs deep copy)
        new_env.current_state = {
            's_idx': self.current_state['s_idx'],
            'grid_mask': self.current_state['grid_mask'].copy(),
            'num_pushes': self.current_state['num_pushes'],
            'length': self.current_state['length'],
            'len_pushes': self.current_state['len_pushes'],
            'len_path': self.current_state['len_path']
        }
        
        return new_env

    def legal_actions(self):
        """Get list of legal action indices for MCTS."""
        if self.action_prune:
            return self.legal_actions_prune()
        return list(range(len(self.action_space)))
    
    def get_available_actions(self):
        return self.legal_actions()

    def legal_actions_prune(self):
        """Get list of legal action indices for MCTS with dynamic pruning."""
        # print("Legal actions called")
        if self.current_state['num_pushes'] >= self.max_pushes:
            return []

        legal = []
        s_idx = self.current_state["s_idx"]
        grid_mask = self.current_state["grid_mask"]

        for idx, (delta_s, theta) in enumerate(self.action_space):
            new_s_idx = (s_idx + delta_s) % self.boundary_len
            start_point = np.array(self.boundary_points[new_s_idx])
            alpha = self.transform_theta_to_absolute(theta, start_point)
            direction = np.array([np.cos(alpha), np.sin(alpha)])
            t_max = self._compute_ray_box_intersection(start_point, direction)

            if t_max <= 0:
                continue  # ray doesn't enter the domain

            # --- check quickly if this push rectangle overlaps particles ---
            if self._push_hits_particles(grid_mask, start_point, direction, t_max):
                legal.append(idx)

        return legal
    
    def get_refined_actions(self, coarse_action):
        """Get refined actions from a coarse action."""
        print("Get refined actions called")
        if isinstance(coarse_action, (int, np.integer)):
            delta_s_coarse, theta_coarse = self.action_space[coarse_action]
        else:
            delta_s_coarse, theta_coarse = coarse_action

        refined_actions = []

        # Define refinement parameters
        delta_s_refinements = [delta_s_coarse + ds for ds in np.linspace(-1, 1, num=3, dtype=int)]
        # delta_s_refinements = [delta_s_coarse + ds for ds in np.linspace(-2, 2, num=3, dtype=int)]
        # theta_refinements = [theta_coarse + dt for dt in np.linspace(-np.pi/16, np.pi/16, num=3)]
        theta_refinements = [theta_coarse + dt for dt in np.linspace(0, np.pi/16, num=1)]
        theta_refinements.extend(list(self.theta_options))

        for delta_s in delta_s_refinements:
            for theta in theta_refinements:
                if (delta_s, theta) not in self.action_space:
                    refined_actions.append((delta_s, theta))

        return list(set(refined_actions))


    def _push_hits_particles(self, grid_mask, start_point, direction, length):
        """
        Fast check if a push hits any occupied cells.
        """
        if length <= 0:
            return False

        # Define rectangle corners
        alpha = np.arctan2(direction[1], direction[0])
        normal = np.array([-np.sin(alpha), np.cos(alpha)])
        rect_end = start_point + direction * length

        rect = np.array([
            start_point - normal * (self.push_width * 0.5),
            start_point + normal * (self.push_width * 0.5),
            rect_end + normal * (self.push_width * 0.5),
            rect_end - normal * (self.push_width * 0.5),
        ])

        # Convert rectangle to grid coords (bounding box)
        corners = [self._world_to_grid(pt) for pt in rect]
        min_i = max(0, min(c[0] for c in corners))
        max_i = min(self.grid_size - 1, max(c[0] for c in corners))
        min_j = max(0, min(c[1] for c in corners))
        max_j = min(self.grid_size - 1, max(c[1] for c in corners))

        if min_i >= self.grid_size or max_i < 0 or min_j >= self.grid_size or max_j < 0:
            return False

        # Check if there are occupied cells inside bbox
        region = grid_mask[min_i:max_i+1, min_j:max_j+1]
        return np.any(region)  # True if at least one particle inside


    def sample_action(self):
        """Sample random legal action for MCTS."""
        legal_acts = self.legal_actions()
        return np.random.choice(legal_acts) if legal_acts else 0

    def step(self, action_index):
        """
        MCTS-compatible step method.
        
        Args:
            action_index: Index into action_space
            
        Returns:
            (state, reward, done, info)
        """
        if isinstance(action_index, (int, np.integer)):
            delta_s, theta = self.action_space[action_index]
        else:
            delta_s, theta = action_index
            
        new_state, reward, done, info = self.apply_push(self.current_state, delta_s, theta)
        self.current_state = new_state
        return new_state, reward, done, info

    def is_terminal(self):
        """Check if current state is terminal."""
        coverage = self.compute_coverage(self.current_state['grid_mask'])
        return (coverage >= self.coverage_thresh or 
                self.current_state['num_pushes'] >= self.max_pushes)


    ########################################## Transformation Functions ##########################################
    
    def grid_to_particles(self, grid_mask):
        particles = []
        for i in range(grid_mask.shape[0]):
            for j in range(grid_mask.shape[1]):
                if grid_mask[i, j]:
                    x = (i + 0.5) * self.grid_res
                    y = (j + 0.5) * self.grid_res
                    particles.append((x, y))
        return np.array(particles)

    def _world_to_grid(self, pos, grid_size=None):
        if grid_size is None:
            grid_size = self.grid_size
        x, y = pos
        i = min(max(int(x * grid_size), 0), grid_size - 1)
        j = min(max(int(y * grid_size), 0), grid_size - 1)
        return i, j
    
    def _grid_to_world(self, grid_pos):
        i, j = grid_pos
        x = (i + 0.5) * self.grid_res
        y = (j + 0.5) * self.grid_res
        return x, y
          
    def transform_theta_to_absolute(self, relative_theta, start_point):
        """
        Transform relative theta to absolute theta based on the current state.
        The absolute theta is the angle from the current boundary point to the next.
        """
        if relative_theta is None:
            return None
        # Normalize relative theta to be within [0, pi)
        relative_theta = relative_theta % np.pi

        # Convert relative theta to absolute theta
        if start_point[0] == 0:
            boundary_angle = 3*np.pi / 2
        elif start_point[1] == 1:
            boundary_angle = np.pi
        elif start_point[0] == 1:
            boundary_angle = np.pi / 2
        else:
            boundary_angle = 0

        absolute_theta = boundary_angle + relative_theta
        # Normalize absolute theta to be within [0, 2*pi)
        absolute_theta = absolute_theta % (2 * np.pi)
        # Ensure absolute theta is within [0, 2*pi)
        if absolute_theta < 0:
            absolute_theta += 2 * np.pi
        if absolute_theta > 2 * np.pi:
            absolute_theta -= 2 * np.pi

        return absolute_theta
    
    def _compute_ray_box_intersection(self, origin, direction):
        """
        Compute the intersection length (t) for a ray from origin in given direction
        with the axis-aligned bounding box [0,1]^2.
        """
        tmin = float('-inf')
        tmax = float('inf')

        for i in range(2):  # x and y
            if abs(direction[i]) < 1e-8:
                if origin[i] < 0 or origin[i] > 1:
                    return 0.0  # No intersection
            else:
                t1 = (0.0 - origin[i]) / direction[i]
                t2 = (1.0 - origin[i]) / direction[i]
                t_near = min(t1, t2)
                t_far = max(t1, t2)
                tmin = max(tmin, t_near)
                tmax = min(tmax, t_far)

        if tmax >= tmin and tmax > 0:
            return tmax
        else:
            return 0.0

    def transform_s_to_normed_world(self, s_idx):
        # Get the boundary point [x,y] corresponding to s_idx in normalized world coordinates [0,1]^2
        boundary_point = np.array(self.boundary_points[s_idx])
        return boundary_point
    
    def find_boundry_corners_in_ds_path(self, prev_s_idx, delta_s):
        # Given a push from prev_s_idx to new_s_idx = (prev_s_idx + delta_s) % boundary_len, find the corner points if the path crosses corners
        prev_point = self.transform_s_to_normed_world(prev_s_idx)
        prev_side = 0
        if prev_point[0] == 0:
            prev_side = 0
        elif prev_point[1] == 1:
            prev_side = 1
        elif prev_point[0] == 1:
            prev_side = 2
        else:
            prev_side = 3
        
        curr_s_idx = (prev_s_idx + delta_s) % self.boundary_len
        curr_point = self.transform_s_to_normed_world(curr_s_idx)
        curr_side = 0
        if curr_point[0] == 0:
            curr_side = 0
        elif curr_point[1] == 1:
            curr_side = 1
        elif curr_point[0] == 1:
            curr_side = 2
        else:
            curr_side = 3
        
        corner_point2 = None
        if prev_side == curr_side:
            return None, None
        else:
        # add the corner point between prev_point and curr_point to the path
            if prev_side == 0:
                if curr_side == 1:
                    corner_point = np.array([0, 1])
                elif curr_side == 3:
                    corner_point = np.array([0, 0])
                else: # curr_side == 2
                    if delta_s > 0:
                        corner_point = np.array([0, 1])
                        corner_point2 = np.array([1, 1])
                    else:
                        corner_point = np.array([0, 0])
                        corner_point2 = np.array([1, 0])
            elif prev_side == 1:
                if curr_side == 2:
                    corner_point = np.array([1, 1])
                elif curr_side == 0:
                    corner_point = np.array([0, 1])
                else: # curr_side == 3
                    if delta_s > 0:
                        corner_point = np.array([1, 1])
                        corner_point2 = np.array([1, 0])
                    else:
                        corner_point = np.array([0, 1])
                        corner_point2 = np.array([0, 0])
            elif prev_side == 2:
                if curr_side == 3:
                    corner_point = np.array([1, 0])
                elif curr_side == 1:
                    corner_point = np.array([1, 1])
                else: # curr_side == 0
                    if delta_s > 0:
                        corner_point = np.array([1, 0])
                        corner_point2 = np.array([0, 0])
                    else:
                        corner_point = np.array([1, 1])
                        corner_point2 = np.array([0, 1])
            else: # prev_side == 3
                if curr_side == 0:
                    corner_point = np.array([0, 0])
                elif curr_side == 2:
                    corner_point = np.array([1, 0])
                else: # curr_side == 1
                    if delta_s > 0:
                        corner_point = np.array([0, 0])
                        corner_point2 = np.array([0, 1])
                    else:
                        corner_point = np.array([1, 0])
                        corner_point2 = np.array([1, 1])
        return corner_point, corner_point2
    
    def transform_normed_world_to_global(self, point):
        # Convert a point from normalized world coordinates [0,1]^2 to global world coordinates using the bounding box size and offset
        global_point = point * self.BB_SIZE + self.grid_offset
        return global_point
    
    def transform_push_to_normed_world(self, s_idx, delta_s, theta):

        new_s_idx = (s_idx + delta_s) % self.boundary_len
        start_point = np.array(self.boundary_points[new_s_idx])
        alpha = self.transform_theta_to_absolute(theta, start_point)
        direction = np.array([np.cos(alpha), np.sin(alpha)])
        t_max = self._compute_ray_box_intersection(start_point, direction)
        rect_end = start_point + direction * t_max

        return start_point, rect_end, alpha
    
    def transform_push_to_global_world(self, s_idx, delta_s, theta):

        start_point, rect_end, alpha = self.transform_push_to_normed_world(s_idx, delta_s, theta)
        # Convert to global world coordinates
        start_point_global = self.transform_normed_world_to_global(start_point)
        rect_end_global = self.transform_normed_world_to_global(rect_end)
        alpha_global = alpha

        return start_point_global, rect_end_global, alpha_global
    
    ############################################################################################################

    def apply_push(self, state, delta_s, theta, print_push=0):
        done = False
        reward = 0.0
        info = {}

        # Check if the action is valid
        if state['num_pushes'] >= self.max_pushes:
            done = True
            coverage = self.compute_coverage(state['grid_mask'])
            info['t_max'] = 0
            info['coverage'] = coverage
            # Compute terminal reward
            reward = self.get_terminal_reward(state, state, coverage)
            if print_push:
                print("Maximum number of pushes reached.")
            return state, reward, done, info  # No change in state

        length = state['length']
        len_pushes = state['len_pushes']
        len_path = state['len_path']

        # Update length and len_path
        len_path += abs(delta_s)
        length += abs(delta_s)

        # s_(i+1)' = s_(i) + delta_s (move along the boundary to next push start point)
        new_s_idx = (state['s_idx'] + delta_s) % self.boundary_len

        start_point = np.array(self.boundary_points[new_s_idx])
        alpha = self.transform_theta_to_absolute(theta, start_point)
        direction = np.array([np.cos(alpha), np.sin(alpha)])
        t_max = self._compute_ray_box_intersection(start_point, direction)

        # s_(i+1) = s_(i+1)'+ direction * t_max (end of the push rectangle)
        rect_end = start_point + direction * t_max
        # Find the closest point on the boundary to the end of the push rectangle
        dists = np.linalg.norm(rect_end - self.boundary_points, axis=1)
        new_s_idx = int(np.argmin(dists))
        
        len_pushes += t_max * float(self.grid_size)  # Convert to grid units
        length += t_max * float(self.grid_size)  # Convert to grid units

        if print_push:
            print(f"Push from {start_point} to {rect_end}")
            print(f"New start idx: {new_s_idx}")

        if self.stochastic_push:
            new_mask = self.apply_stochastic_rect_push(state['grid_mask'].copy(), self.shovel_type, start_point, direction, t_max, plot=print_push)
        else:
            new_mask = self.apply_rect_push(state['grid_mask'].copy(), start_point, direction, t_max)

        new_state = {
            "s_idx": new_s_idx,
            "grid_mask": new_mask,
            "num_pushes": state['num_pushes'] + 1,
            "length": length,
            "len_pushes": len_pushes,
            "len_path": len_path
        }

        # Compute coverage
        new_coverage = self.compute_coverage(new_mask)
        if print_push:
            print(f"Coverage after push: {new_coverage:.3f}")
        
        # Check if the coverage is sufficient or if maximum pushes reached
        if new_coverage >= 0.999:
            done = True
            # Compute terminal reward
            reward = self.get_terminal_reward(new_state, state, new_coverage)
            if print_push:
                print("All particles covered!")
        elif new_state['num_pushes'] == self.max_pushes:
            done = True
            # Compute terminal reward
            reward = self.get_terminal_reward(new_state, state, new_coverage)
            if print_push:
                print("Maximum number of pushes reached.")
        else:
            # Not done yet, compute intermediate reward
            reward = self.get_reward(new_state, state, new_coverage)

        info['t_max'] = t_max
        info['coverage'] = new_coverage

        return new_state, reward, done, info
    
    def compute_coverage(self, grid_mask):
        if self.stochastic_push:
            uncovered = np.sum(grid_mask)
        else:
            uncovered = np.count_nonzero(grid_mask)
        return 1.0 - uncovered / self.total_occupied
    
    def potential_function(self, state, coverage, lambd=0.8):
        """
        Compute the potential function based on the current state.
        """
        phi = (lambd * coverage * self.total_occupied - (1-lambd) * (state["length"] * self.push_width * self.grid_size)) * (self.grid_res)

        return phi
    
    def get_reward(self, new_state, state, new_coverage):
        """
        Compute the reward based on the current state.
        The reward is negative based on the uncovered area.
        If all particles are covered, the reward is zero.
        """
        reward = 0.0
        w1 = 2.5
        w2 = 1.0
        # alpha = 0.85

        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        #     if new_coverage < 0.9:
        #         # Penalize for uncovered area
        #         reward += -(((new_state['length'] * self.push_width * self.grid_size) * 20.0) / ((new_coverage) * self.total_occupied))
        #     else:
        #         # Reward for good coverage
        #         reward += ((new_coverage * self.total_occupied) / ((new_state['length'] * self.push_width * self.grid_size) * 20.0))

        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        #     reward = ((new_coverage) * self.total_occupied * self.grid_res) / (((new_state['len_pushes'] * self.push_width * self.grid_size) + new_state['len_path']) * self.grid_res)
        

        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        #     reward = ((new_coverage) * self.total_occupied * self.grid_res) - 0.2 * (new_state['length'] * self.push_width * self.grid_size * self.grid_res)



        # potential based reward
        # pre_coverage = self.compute_coverage(state['grid_mask'])
        # reward += self.potential_function(new_state, new_coverage) - self.potential_function(state, pre_coverage)
        # max_coverage = self.total_occupied

        max_length = (np.sqrt(2) * self.grid_size + max(self.delta_s_options))
        delta_length = new_state['length'] - state['length']
        delta_coverage_normed = new_coverage - self.compute_coverage(state['grid_mask'])

        if delta_coverage_normed <= 0.0:
            return -0.5    # Heavy penalty for no coverage improvement
        delta_length_normed = delta_length / max_length

        reward = w1 * delta_coverage_normed - w2 * delta_length_normed

        # uncovered = 1.0 - new_coverage
        # if delta_length:
        #     inv_delta_length = 1.0 / (delta_length)  # Avoid division by zero
        # else:
        #     inv_delta_length = 0.0
        # if inv_delta_length > 1.0:
        #     print("Warning: inv_delta_length > 1.0,", inv_delta_length, "delta_length:", delta_length, "new_state['length']:", new_state['length'], "state['length']:", state['length'])
        
        # delta_coverage = (new_coverage - self.compute_coverage(state['grid_mask'])) * self.total_occupied
        # norm_term = max_length + max_coverage

        # delta_uncovered = (1-new_coverage) - (1-self.compute_coverage(state['grid_mask'])) * self.total_occupied
        # epsilon = 0.05 * max_length
        # epsilon = 1.0
        # length_normed = new_state['length'] / max_length_tot
        # reward = - uncovered - length_normed

        # if new_coverage >= self.coverage_thresh:
        #     reward = new_coverage / length_normed
        # max_coverage = self.total_occupied
        # delta_length = new_state['length'] - state['length']

        # # if delta_length:
        # #     inv_delta_length = 1.0 / (delta_length)  # Avoid division by zero
        # # else:
        # #     inv_delta_length = 0.0
        # # if inv_delta_length > 1.0:
        # #     print("Warning: inv_delta_length > 1.0,", inv_delta_length, "delta_length:", delta_length, "new_state['length']:", new_state['length'], "state['length']:", state['length'])
        
        # # delta_coverage = (new_coverage - self.compute_coverage(state['grid_mask'])) * self.total_occupied
        # # norm_term = max_length + max_coverage

        # # delta_uncovered = (1-new_coverage) - (1-self.compute_coverage(state['grid_mask'])) * self.total_occupied

        # delta_coverage_normed = new_coverage - self.compute_coverage(state['grid_mask'])
        # epsilon = 0.05 * max_length 
        # # epsilon = 1.0

        # if delta_length <= epsilon:
        #     # print("Warning: delta_length <= epsilon, setting to epsilon", "delta_length:", delta_length, "new_state['length']:", new_state['length'], "state['length']:", state['length'])
        #     delta_length = epsilon

        # delta_length_normed = (epsilon * (max_length - delta_length)) / (delta_length * (max_length - epsilon))

        # reward = (alpha * delta_coverage_normed + (1 - alpha) * delta_length_normed)

        # reward = delta_uncovered - (delta_length * self.push_width * self.grid_size)
        # reward = (alpha * (delta_coverage / norm_term) + (1-alpha) * (inv_delta_length / norm_term)) * 100.0
        # reward = (alpha * (delta_coverage / norm_term) - (1-alpha) * (delta_length / norm_term)) * 100.0

        # if reward > 1.0:
        #     print("Warning: reward > 1.0,", reward, "new_coverage:", new_coverage, "new_state['length']:", new_state['length'], "state['length']:", state['length'])

        return reward
    
    def get_terminal_reward(self, new_state, state, new_coverage):
        """
        Compute the terminal reward based on the coverage and length of the path.
        """
        reward = 0.0
        # alpha = 0.85
        w1 = 2.5
        w2 = 1.0

        # Compute potential-based reward
        # pre_coverage = self.compute_coverage(state['grid_mask'])
        # reward += self.potential_function(new_state, new_coverage) - self.potential_function(state, pre_coverage)

        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        #     if new_coverage < 0.9:
        #         # Penalize for uncovered area
        #         reward += - (((new_state['length'] * self.push_width * self.grid_size) * 30.0) / ((new_coverage) * self.total_occupied)) * 10.0
        #     else:
        #         # Reward for good coverage
        #         reward += ((new_coverage * self.total_occupied) / ((new_state['length'] * self.push_width * self.grid_size) * 30.0)) * 10.0

        max_length_tot = (np.sqrt(2) * self.grid_size + max(self.delta_s_options)) * new_state['num_pushes']
        max_length = (np.sqrt(2) * self.grid_size + max(self.delta_s_options))
        delta_length = new_state['length'] - state['length']
        delta_coverage_normed = new_coverage - self.compute_coverage(state['grid_mask'])
        delta_length_normed = delta_length / max_length

        if delta_coverage_normed <= 0.0:
            return -0.5    # Heavy penalty for no coverage improvement

        reward = w1 * delta_coverage_normed - w2 * delta_length_normed
        # uncovered = 1.0 - new_coverage
        # if delta_length:
        #     inv_delta_length = 1.0 / (delta_length)  # Avoid division by zero
        # else:
        #     inv_delta_length = 0.0
        # if inv_delta_length > 1.0:
        #     print("Warning: inv_delta_length > 1.0,", inv_delta_length, "delta_length:", delta_length, "new_state['length']:", new_state['length'], "state['length']:", state['length'])
        
        # delta_coverage = (new_coverage - self.compute_coverage(state['grid_mask'])) * self.total_occupied
        # norm_term = max_length + max_coverage

        # delta_uncovered = (1-new_coverage) - (1-self.compute_coverage(state['grid_mask'])) * self.total_occupied
        # delta_coverage_normed = new_coverage - self.compute_coverage(state['grid_mask'])
        # epsilon = 0.05 * max_length
        # epsilon = 1.0
        length_normed = (new_state['length']) / max_length_tot
        # reward = - uncovered - length_normed

        if new_coverage >= self.coverage_thresh:
            reward += (w1 * new_coverage) / (length_normed)
        # else:
        #     reward = w1 * new_coverage - w2 * length_normed

        # reward = w1 * new_coverage - w2 * length_normed
        # if delta_length <= epsilon:
        #     # print("Warning: delta_length <= epsilon, setting to epsilon", "delta_length:", delta_length, "new_state['length']:", new_state['length'], "state['length']:", state['length'])
        #     delta_length = epsilon

        # delta_length_normed = (epsilon * (max_length - delta_length)) / (delta_length * (max_length - epsilon))

        # reward = (alpha * delta_coverage_normed + (1 - alpha) * delta_length_normed)

        # reward = delta_uncovered - (delta_length * self.push_width * self.grid_size)

        # reward = (alpha * (delta_coverage / norm_term) + (1-alpha) * (inv_delta_length / norm_term)) * 100.0
        # reward = (alpha * (delta_coverage / norm_term) - (1-alpha) * (delta_length / norm_term)) * 100.0

        # if reward > 1.0:
        #     print("Warning: reward > 1.0,", reward, "new_coverage:", new_coverage, "new_state['length']:", new_state['length'], "state['length']:", state['length'])


        # if new_state['length'] > 0.0 and new_coverage > 0.0:
        #     reward = ((new_coverage) * self.total_occupied * self.grid_res) - 0.2 * (new_state['length'] * self.push_width * self.grid_size * self.grid_res)

        # if new_coverage >= 0.99:
        #     # Success reward
        #     # reward += 10.0 * (new_coverage * self.total_occupied * self.grid_res)
        #     reward += 10.0

        return reward
    

    def apply_rect_push(self, grid_mask, start_point, direction, length):
        """
        Apply a rectangular push on the grid mask starting from start_point in the given direction and length.
        The rectangle is defined by the push width and length.
        """
        # Early exit for zero-length pushes
        if length <= 0:
            return grid_mask.copy()
            
        # Copy current grid mask
        new_mask = grid_mask.copy()
        alpha = np.arctan2(direction[1], direction[0])  # Angle of the push direction
        normal = np.array([-np.sin(alpha), np.cos(alpha)])
        rect_start = start_point
        rect_end = start_point + direction * length
        
        rect = np.array([
            rect_start - normal * (self.push_width * 0.5),
            rect_start + normal * (self.push_width * 0.5),
            rect_end + normal * (self.push_width * 0.5),
            rect_end - normal * (self.push_width * 0.5),
        ])

        # Convert rectangle corners to grid coordinates
        c1 = self._world_to_grid(rect[0])
        c2 = self._world_to_grid(rect[1])
        c3 = self._world_to_grid(rect[2])
        c4 = self._world_to_grid(rect[3])

        min_i = min(c1[0], c2[0], c3[0], c4[0])
        max_i = max(c1[0], c2[0], c3[0], c4[0])
        min_j = min(c1[1], c2[1], c3[1], c4[1])
        max_j = max(c1[1], c2[1], c3[1], c4[1])
        
        # Early exit if bounding box is outside grid bounds
        if min_i >= self.grid_size or max_i < 0 or min_j >= self.grid_size or max_j < 0:
            return new_mask
            
        # Clamp bounding box to grid bounds
        min_i = max(0, min_i)
        max_i = min(self.grid_size - 1, max_i)
        min_j = max(0, min_j)
        max_j = min(self.grid_size - 1, max_j)
        
        # Extract occupied cells in the bounding box region
        bbox_mask = new_mask[min_i:max_i+1, min_j:max_j+1]
        occupied_indices = np.where(bbox_mask)
        
        if len(occupied_indices[0]) > 0:
            # Convert to absolute grid coordinates
            i_coords = occupied_indices[0] + min_i
            j_coords = occupied_indices[1] + min_j
            
            # Convert to world coordinates
            x_coords, y_coords = self._grid_to_world((i_coords, j_coords))
            
            # Check which particles are inside the push rectangle
            rel_pos = np.column_stack([x_coords, y_coords]) - start_point
            proj = np.dot(rel_pos, direction)
            
            perp_vectors = rel_pos - proj[:, np.newaxis] * direction
            perp_len_sq = np.sum(perp_vectors * perp_vectors, axis=1)
            push_width_half_sq = (self.push_width * 0.5) ** 2
            
            # Find cells within the push rectangle (use squared distances)
            valid_mask = (proj >= 0) & (proj <= length) & (perp_len_sq <= push_width_half_sq)
            
            # Update the grid mask for valid cells
            if np.any(valid_mask):
                valid_i = i_coords[valid_mask]
                valid_j = j_coords[valid_mask]
                new_mask[valid_i, valid_j] = False

        return new_mask


    def apply_stochastic_rect_push(self, grid_mask, shovel_type, start_point, direction, length, plot=False):
        if length <= 0:
            return grid_mask.copy()

        new_mask = grid_mask.copy()
        
        # --- 1. Pre-calculations (Vectorized) ---
        alpha = np.arctan2(direction[1], direction[0])
        normal = np.array([-np.sin(alpha), np.cos(alpha)])
        push_half_width = self.push_width * 0.5
        
        # Push direction components (grid units)
        if shovel_type == 'flat':
            # push_dir = direction
            # push_normal = normal
            # fwd_prob = 0.9
            fwd_prob = 0.65
            # fwd_prob = 0.7
            remaining_split_right = 0.5
        elif shovel_type == 'angled_right_15':
            # angle_offset = np.pi / 12  # 15 degrees to the right
            # rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
            #                        [np.sin(angle_offset),  np.cos(angle_offset)]])
            # push_dir = rot_matrix @ direction
            # push_normal = np.array([-push_dir[1], push_dir[0]])
            fwd_prob = 0.78
            remaining_split_right = 0.75
        elif shovel_type == 'angled_right_20':
            # angle_offset = np.pi / 9  # 20 degrees to the right
            # rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
            #                        [np.sin(angle_offset),  np.cos(angle_offset)]])
            # push_dir = rot_matrix @ direction
            # push_normal = np.array([-push_dir[1], push_dir[0]])
            fwd_prob = 0.75
            remaining_split_right = 0.8
        elif shovel_type == 'angled_right_25':
            # angle_offset = (24 * np.pi) / 180  # 25 degrees to the right
            # rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
            #                        [np.sin(angle_offset),  np.cos(angle_offset)]])
            # push_dir = rot_matrix @ direction
            # push_normal = np.array([-push_dir[1], push_dir[0]])
            fwd_prob = 0.79 # need to check again
            remaining_split_right = 0.95
        elif shovel_type == 'angled_left_25':
            # angle_offset = (24 * np.pi) / 180  # 25 degrees to the right
            # rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
            #                        [np.sin(angle_offset),  np.cos(angle_offset)]])
            # push_dir = rot_matrix @ direction
            # push_normal = np.array([-push_dir[1], push_dir[0]])
            fwd_prob = 0.79 # need to check again
            remaining_split_right = 0.05
        elif shovel_type == 'concave':
            fwd_prob = 0.92
            remaining_split_right = 0.4

        di_f = int(round(direction[0]))
        dj_f = int(round(direction[1]))
        di_r = int(round(direction[0] + normal[0]))
        dj_r = int(round(direction[1] + normal[1]))
        di_l = int(round(direction[0] - normal[0]))
        dj_l = int(round(direction[1] - normal[1]))

        # Define Bounding Box (to avoid scanning whole grid)
        rect_end = start_point + direction * length
        pts = np.array([
            start_point - normal * push_half_width,
            start_point + normal * push_half_width,
            rect_end + normal * push_half_width,
            rect_end - normal * push_half_width
        ])
        
        min_i = max(0, int((np.min(pts[:,0]) / self.grid_res) - 1))
        max_i = min(self.grid_size, int((np.max(pts[:,0]) / self.grid_res) + 2))
        min_j = max(0, int((np.min(pts[:,1]) / self.grid_res) - 1))
        max_j = min(self.grid_size, int((np.max(pts[:,1]) / self.grid_res) + 2))
        
        # Create Meshgrid for the Push Area
        i_range = np.arange(min_i, max_i)
        j_range = np.arange(min_j, max_j)
        I_local, J_local = np.meshgrid(i_range, j_range, indexing='ij')
        
        # Convert to World Coords
        X_world = (I_local + 0.5) * self.grid_res
        Y_world = (J_local + 0.5) * self.grid_res
        
        # Calculate Projections for sorting
        RX = X_world - start_point[0]
        RY = Y_world - start_point[1]
        proj_para = RX * direction[0] + RY * direction[1]
        proj_perp = RX * normal[0] + RY * normal[1]

        # Create sub-grid view for density checks
        sub_grid = new_mask[min_i:max_i, min_j:max_j]
        
        # Identify ALL cells that will be part of the push (in the rectangle)
        # We do this once to get the geometry
        valid_geometry_mask = (proj_para >= 0) & (proj_para <= length) & \
                            (np.abs(proj_perp) <= push_half_width)

        # intersect geometry mask with actual dirt presence
        dirt_in_path_mask = valid_geometry_mask & (sub_grid > 0)

        if not np.any(dirt_in_path_mask):
            return new_mask  # Shovel moves through empty space, nothing happens

        # Find the minimum projection distance of any dirt particle
        min_dirt_dist = np.min(proj_para[dirt_in_path_mask])

        # # Calculate the starting slice index
        # # We floor the distance to ensure we include the slice containing the first particle
        # start_k = int(np.floor(min_dirt_dist / self.grid_res))
        # start_k = max(0, start_k) # Safety check

        # --- 2. SLICED EXECUTION (The Wave Effect) ---
        slice_step = self.grid_res
        current_front = np.floor(min_dirt_dist / slice_step) * slice_step
        idx = 0
        
        while current_front < length:
            idx += 1
            current_front += slice_step
            # Define the distance range for this slice (Row k)
            # dist_min = k * self.grid_res - (self.grid_res * 0.1) # buffer
            # dist_max = (k + 1) * self.grid_res + (self.grid_res * 0.1)
            
            # # Identify cells in this specific slice
            # # Note: We check new_mask > 0 dynamically because previous iterations might have filled these cells!
            # slice_mask = valid_geometry_mask & \
            #             (proj_para >= dist_min) & \
            #             (proj_para < dist_max)
            
            # Extract current state of these cells
            # We act on the sub-grid for speed
            active_slice_mask = valid_geometry_mask & (proj_para <= current_front)
            # active_slice_mask = valid_geometry_mask & (proj_para < current_front)
            # active_slice_mask = slice_mask & (sub_grid > 0)
            
            if not np.any(active_slice_mask):
                continue

            # --- A. Apply Vectorized Push to this Slice ---
            
            # Get Global Indices
            src_i = I_local[active_slice_mask]
            src_j = J_local[active_slice_mask]
            densities = sub_grid[active_slice_mask]
            
            # Clear source
            new_mask[src_i, src_j] = 0.0
            
            # Calculate Moves (Stochastic)
            n_pixels = np.round(densities * self.total_pixels_in_cell).astype(int)
            # Vectorized binomial draws - need to iterate since binomial doesn't accept array of probabilities
            p_fwd = np.clip(fwd_prob + 0.05 - 0.1 * densities, 0.0, 1.0)
            n_fwd = np.array([np.random.binomial(n, p) for n, p in zip(n_pixels, p_fwd)])
            # n_exc_fwd = np.random.binomial(n_excess, fwd_prob)
            remaining = n_pixels - n_fwd
            n_right = np.random.binomial(remaining, remaining_split_right)
            n_left = remaining - n_right
            
            d_fwd = n_fwd / self.total_pixels_in_cell
            d_right = n_right / self.total_pixels_in_cell
            d_left = n_left / self.total_pixels_in_cell
            
            # Helper to add mass
            def add_safe(idx_i, idx_j, d_vals):
                valid = (idx_i >= 0) & (idx_i < self.grid_size) & \
                        (idx_j >= 0) & (idx_j < self.grid_size) & \
                        (d_vals > 0.009)
                if np.any(valid):
                    np.add.at(new_mask, (idx_i[valid], idx_j[valid]), d_vals[valid])

            # Execute moves for this row
            add_safe(src_i + di_f, src_j + dj_f, d_fwd)
            add_safe(src_i + di_r, src_j + dj_r, d_right)
            add_safe(src_i + di_l, src_j + dj_l, d_left)
            
            # --- B. Immediate Spillover (Stabilize *before* next row) ---
            # We run a short spill loop here so the pile is ready for the next slice to pick up.
            # This mimics your inner loop logic.
            
            for _ in range(10): # Small number of iters per slice is usually enough
                saturation = 1.0
                excess_mask = new_mask > saturation
                if not np.any(excess_mask):
                    break
                    
                excess = new_mask[excess_mask] - saturation
                new_mask[excess_mask] = saturation
                
                ovf_i, ovf_j = np.where(excess_mask)
                
                # Distribute excess Forward (Simulating avalanche)
                n_excess = np.round(excess * self.total_pixels_in_cell).astype(int)
                n_exc_fwd = np.random.binomial(n_excess, fwd_prob)
                exc_remaining = n_excess - n_exc_fwd
                n_exc_right = np.random.binomial(exc_remaining, remaining_split_right)
                n_exc_left = exc_remaining - n_exc_right

                exc_fwd = n_exc_fwd / self.total_pixels_in_cell
                exc_right = n_exc_right / self.total_pixels_in_cell
                exc_left = n_exc_left / self.total_pixels_in_cell
                add_safe(ovf_i + di_f, ovf_j + dj_f, exc_fwd)
                add_safe(ovf_i + di_r, ovf_j + dj_r, exc_right)
                add_safe(ovf_i + di_l, ovf_j + dj_l, exc_left)

            if plot:
                # start_point = start_point + direction * self.grid_res
                # if plot == 2 and idx == 10:
                if plot == 2 and idx == 9:
                # if plot == 2 and idx == 21:
                # if plot == 2 and idx == 12:
                    push = (start_point, direction)
                    self.draw_grid_environment(new_mask, current_push=push)

                    return new_mask
                elif plot == 1:
                    push = (start_point, direction)
                    self.draw_grid_environment(new_mask, current_push=push)
            
        return new_mask
    
############################################### visualization functions ################################################

    def draw_environment(self, perticles, pushes=[], current_push=None):
        plt.figure(figsize=(8,8))
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)

        # Draw boundary
        boundary_x, boundary_y = zip(*self.boundary_points)
        plt.plot(boundary_x + (boundary_x[0],), boundary_y + (boundary_y[0],), 'k-')

        if len(perticles) > 0:
            plt.scatter(perticles[:,0], perticles[:,1], s=10)

        # Draw all pushes
        for push in pushes:
            # start_point, direction = push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'b-')
            plt.fill(rect[:,0], rect[:,1], 'cyan', alpha=0.2)

        # Draw current push in different color
        if current_push is not None:
            # start_point, direction = current_push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'r-')
            plt.fill(rect[:,0], rect[:,1], 'orange', alpha=0.4)

        plt.title("Aggregates Environment")
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.grid(True)
        plt.show()

    def draw_real_environment(self, perticles, pushes=[], current_push=None):
        plt.figure(figsize=(8,8))
        plt.xlim(0.0, self.BB_SIZE)
        plt.ylim(0.0, self.BB_SIZE)
        real_particles = perticles * self.BB_SIZE
        real_push_width = self.push_width * self.BB_SIZE

        # Draw boundary
        boundary_x, boundary_y = zip(*self.boundary_points)
        plt.plot(boundary_x + (boundary_x[0],), boundary_y + (boundary_y[0],), 'k-')

        if len(perticles) > 0:
            plt.scatter(real_particles[:,0], real_particles[:,1], s=10)

        # Draw all pushes
        for push in pushes:
            # start_point, direction = push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'b-')
            plt.fill(rect[:,0], rect[:,1], 'cyan', alpha=0.2)

        # Draw current push in different color
        if current_push is not None:
            # start_point, direction = current_push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'r-')
            plt.fill(rect[:,0], rect[:,1], 'orange', alpha=0.4)

        plt.title("Aggregates Environment")
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.grid(True)
        plt.show()

    def animate_grid_push(self, action):

        s_idx = self.current_state['s_idx']

        if isinstance(action, (int, np.integer)):
            delta_s, theta = self.action_space[action]
        else:
            delta_s, theta = action
            
        new_state, _, _, _ = self.apply_push(self.current_state, delta_s, theta, print_push=True)

        grid_mask = new_state['grid_mask']

        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                if value > 0.0:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        plt.title("Occupancy Grid")
        plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        plt.show()

    def animate_transition_push(self, action):

        s_idx = self.current_state['s_idx']

        if isinstance(action, (int, np.integer)):
            delta_s, theta = self.action_space[action]
        else:
            delta_s, theta = action
            
        new_state, _, _, _ = self.apply_push(self.current_state, delta_s, theta, print_push=2)

        grid_mask = new_state['grid_mask']

        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                if value > 0.0:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        plt.title("Occupancy Grid")
        plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        plt.show()

        return new_state

    def draw_grid_environment(self, grid_mask, pushes=[], current_push=None):
        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                # if value > 0.0:
                if value:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        # Draw previous pushes
        for start_point, direction in pushes:
            self._draw_push(start_point, direction, color='cyan', alpha=0.2)
        # # Draw previous pushes
        # for s_idx, delta_s, theta in pushes:
        #     self._draw_push(s_idx, delta_s, theta, color='cyan', alpha=0.2)

        # Draw current push
        if current_push is not None:
            start_point, direction = current_push
            self._draw_push(start_point, direction, color='orange', alpha=0.1)

        plt.title("Occupancy Grid")
        # plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        plt.show()

    # def _draw_push(self, s_idx, delta_s, theta, color='cyan', alpha=0.3):
    def _draw_push(self, start_point, direction, color='cyan', alpha=0.3):
        theta = np.arctan2(direction[1], direction[0])
        normal = np.array([-np.sin(theta), np.cos(theta)])

        rect_start = start_point
        t_max = self._compute_ray_box_intersection(start_point, direction)
        rect_end = rect_start + direction * t_max
        
        if self.shovel_type == 'flat':
            push_normal = normal
        elif self.shovel_type == 'angled_right_15':
            angle_offset = np.pi / 12  # 15 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_right_20':
            angle_offset = np.pi / 9  # 20 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_right_25':
            angle_offset = (5 * np.pi) / 36  # 25 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_left_25':
            angle_offset = -(5 * np.pi) / 36  # 25 degrees to the left
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'concave':
            angle_offset_right = np.pi / 12  # 15 degrees to the right
            angle_offset_left = -np.pi / 12   # 15 degrees to the left
            rot_matrix_right = np.array([[np.cos(angle_offset_right), -np.sin(angle_offset_right)],
                                   [np.sin(angle_offset_right),  np.cos(angle_offset_right)]])
            rot_matrix_left = np.array([[np.cos(angle_offset_left), -np.sin(angle_offset_left)],
                                   [np.sin(angle_offset_left),  np.cos(angle_offset_left)]])
            push_dir_right = rot_matrix_right @ direction
            push_dir_left = rot_matrix_left @ direction
            push_normal_right = np.array([-push_dir_right[1], push_dir_right[0]])
            push_normal_left = np.array([-push_dir_left[1], push_dir_left[0]])
            # For concave, we can average the normals for visualization
            push_normal = (push_normal_right + push_normal_left) / 2

        
        shovel = np.array([
            rect_start + push_normal * (self.push_width / 2),
            rect_start - push_normal * (self.push_width / 2)
        ])

        rect = np.array([
            rect_start + normal * (self.push_width / 2),
            rect_end + normal * (self.push_width / 2),
            rect_end - normal * (self.push_width / 2),
            rect_start - normal * (self.push_width / 2),
            rect_start + normal * (self.push_width / 2)
        ])

        plt.plot(rect[:, 0], rect[:, 1], color=color)
        plt.fill(rect[:, 0], rect[:, 1], color=color, alpha=alpha)
        plt.scatter(*start_point, color='red', s=30)  # Push origin
        plt.plot(shovel[:, 0], shovel[:, 1], color='brown', linewidth=4, label="Shovel Edge")

    def plot_pushes_original_space(self, pushes, boundary_points, rock_positions, shovel_width, title="Push Rectangles"):

        plt.figure(figsize=(8, 8))

        # Plot the particles
        plt.scatter(rock_positions[:, 0], rock_positions[:, 1], s=10, color='black', label="Particles")

        # # Plot boundary
        # bx, by = zip(*boundary_points)
        # plt.plot(bx + (bx[0],), by + (by[0],), 'gray', linestyle='--', label="Boundary")

        # Rectangle colors
        colors_rect = ['r', 'g', 'b', 'orange', 'purple', 'cyan', 'magenta', 'lime', 'gold', 'teal']

        # for i, (s_idx, delta_s, theta) in enumerate(pushes):
        # for i, (start_point, direction) in enumerate(pushes):
        for i, (start_point, end_point, alpha) in enumerate(pushes):
            color = colors_rect[i % len(colors_rect)]
            # start_point = np.array(boundary_points[s_idx])
            # new_start_idx = (s_idx + delta_s) % len(boundary_points)
            # start_point = np.array(boundary_points[new_start_idx])
            # Transform theta to absolute angle
            # alpha = self.transform_theta_to_absolute(theta, start_point)
            # direction = np.array([np.cos(alpha), np.sin(alpha)])
            # alpha = np.arctan2(direction[1], direction[0])
            normal = np.array([-np.sin(alpha), np.cos(alpha)])

            rect_start = start_point
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            rect_end = end_point

            rect = np.array([
                rect_start + normal * (shovel_width / 2),
                rect_end + normal * (shovel_width / 2),
                rect_end - normal * (shovel_width / 2),
                rect_start - normal * (shovel_width / 2),
                rect_start + normal * (shovel_width / 2)
            ])

            # Plot the rectangle
            plt.plot(rect[:, 0], rect[:, 1], color=color, linewidth=2, label=f"Push {i+1}")
            plt.fill(rect[:, 0], rect[:, 1], color=color, alpha=0.2)
            plt.scatter(*start_point, color='red', s=30)  # Push origin

        plt.title(title)
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.xlim(0, 1)
        # plt.ylim(0, 1)
        plt.gca().set_aspect("equal")
        plt.legend()
        plt.show()
