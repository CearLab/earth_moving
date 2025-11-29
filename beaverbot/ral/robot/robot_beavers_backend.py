# Backend imports
from beaverbot.ral.robot.robot_backend import BaseRobotBackend

# Module imports
from beaverbot.ral.robot.modules.module_control import Controller, Dynamics
import beaverbot.ral.algorithms.module_misc as module_misc
import beaverbot.ral.robot.modules.module_beaver as module_beaver


class BeaversRobotBackend(BaseRobotBackend):
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs) 
                
    def initiate_robot(self, **kwargs):
        
        super().initiate_robot(**kwargs)
        
        # Maximum load configuration
        self._maximum_load = self._robot.get('maximum_load') 
        if self._maximum_load is None:
            self._maximum_load = self.np.inf
        self._maximum_load_init = self._maximum_load
        
        # Harvesting and exploration parameters
        self._harvest_threshold = self._robot.get('harvest_threshold')      
        self._exploration_eta = self._robot.get('exploration_eta')
        self._exploration_map = self._robot.get('exploration_map')
        self._exploration_N_recovery = self._robot.get('exploration_N_recovery', 8)           
        self._print = self._robot.get('print')
        self._role = self._robot.get('role', 'explorer')  # Default role is explorer
        
        # Spatial and movement attributes
        self._range_x = self._robot.get('range_x')
        self._range_y = self._robot.get('range_y')
        self._exploration_mode = self._robot.get('exploration_mode')
        
        # Harvesting and measurement parameters
        self._vegetation_removal = self._robot.get('vegetation_removal')
        self._measurement_mode = self._robot.get('measurement_mode')
        self._measure_step = self._robot.get('measure_step', 1)
        
        # Home base configuration (beaver's lodge)
        self._home_base_position = list(self._robot.get('home_base_position'))[0]
        self._home_base_position_store = list(self._robot.get('home_base_position'))
        
        # Adaptive behavior parameters
        self._n_traces = self._robot.get('n_traces')
        self._decay_values = self._robot.get('decay_values')

        # Internal state variables
        self._current_time = 0  # Time counter (agent does not compute hour/time - provided by environment)
        self._map_quality_measure = None
        self._map_quality_measure_position = None
        self._map_quality_update = False  # Flag to update vegetation quality in environment
        self._motion_destination = None
        self._neighbourhood = None        
        self._neighbourhood_reached_flag = None
        self._neighbourhood_current_index = None
        self._local_map = None
        self._local_map_visits = None
        self._harvesting_actions_counter = 0
        self._in_water_counter = 0
        self._reset_integral = 50
        self._min_home_distance = 500
        
        # Environment data (will be provided by the environment backend)
        self._vegetation_quality_range = None        
        
        # Position initialization
        position = self._robot.get('position')
        if position == 'random':
            self._position = [self.random.randint(self._range_x[0], self._range_x[1] - 1), 
                              self.random.randint(self._range_y[0], self._range_y[1] - 1)]
        elif position == 'home':
            self._position = self._home_base_position            
        elif position == 'random_home':
            # Choose a random home base from the list
            if isinstance(self._home_base_position_store, list) and len(self._home_base_position_store) > 0:
                selected_base = list(self.random.choice(self._home_base_position_store))
                # Randomly select a position around the chosen home base within the allowed range
                self._position = [
                    selected_base[0] + self.random.randint(self._range_x[0], self._range_x[1] - 1),
                    selected_base[1] + self.random.randint(self._range_y[0], self._range_y[1] - 1)
                ]
            else:
                raise ValueError('home_base_position must be a non-empty list for random_home initialization')
        elif position is None:
            self._position = [0, 0]
        elif isinstance(position, list):
            self._position = position
            self._position[0] = self.np.clip(self._position[0], self._range_x[0], self._range_x[1])
            self._position[1] = self.np.clip(self._position[1], self._range_y[0], self._range_y[1])
        else:
            raise ValueError('Invalid position value: {}'.format(position))
        
        # Physical state initialization
        self._load = 0
        
        # Motion control initialization                      
        self._controller = Controller(**self._robot)
        initial_state = self.np.array([self._position, self.np.zeros(self._controller._dimension)])
        self._dynamics = Dynamics(initial_state, **self._robot)        
        self._local_map_control = None        
        
        # Status flags
        self._status_robot = 'IDLE'      
        self._status_task = 'IDLE'  
        self._status_motion = 'IDLE'
        self._current_task = None
        self._current_action = None
        
        # Data storage arrays
        self._destination_store = []
        self._action_store = []
        self._position_store = []
        self._error_store = []
        self._load_store = []
        self._task_store = []
        self._time_store = []
        self._exploration_eta_store = []        
        self._harvest_threshold_store = []
        self._maximum_load_store = []
        
        # Store initial parameter values for resets
        self._exploration_eta_init = self._exploration_eta
        self._harvest_threshold_init = self._harvest_threshold.copy()
        self._wait_b4_explore_init = 1 * 24
        self._wait_b4_explore = 0
        return self
    
    def step_beaver(self, dt, time_of_day, map_quality, limits, misc=None) -> None:
    
        # Update internal clock
        self._current_time += dt
        
        # Set integration time for controller and dynamics
        self._controller._dt = dt                
        self._dynamics._dt = dt     
                        
        # Gather observations from environment
        self._map_quality_measure = map_quality 
        # Quality at the current position (see module_misc in the visualizer backend)
        self._map_quality_measure_position = self._map_quality_measure[1][-1] 
        
        # Update the local_map according to the measurements
        self.update_local_map(self._map_quality_measure, misc)
        self._map_quality_update = False  # Reset the flag
        
        # Reset integral error periodically
        if self._current_time % self._reset_integral == 0:
            self._controller._error_integral = 0.0
            
        # Decide the goal (task policy)
        self.decide_task(time_of_day, limits, misc)
        
        # Execute the task
        self.do_task(time_of_day, limits, misc)
        
        # Time-dependent decay of the explore/harvest parameters to avoid stagnation
        # Decay order parameters: exploration_eta, harvest_threshold, maximum_load
        self.select_exploration_eta(dt=self._wait_b4_explore_init, dt_percentage=0.25, decay=self._decay_values)

        # Determine if environment should be actuated/updated
        # Only the harvest and store tasks modify the environment
        if (self._current_task == 'harvest' and self._status_task == 'FINISHED') or \
            (self._current_task == 'store' and \
                (self._status_task == 'FINISHED') or (self._status_task == 'INPROGRESS' and self._status_motion == 'FINISHED')):
            self._map_quality_update = True
        else:                        
            self._map_quality_update = False                       
            
        # Update the local_map according to the action
        if self._map_quality_update == True:
            self._map_quality_measure = [[self._position], [self._map_quality_measure_position]]
            self.update_local_map(self._map_quality_measure, misc)            
        self._map_quality_measure_position = self._local_map[self._position[0], self._position[1]]
            
        # Store the data for analysis
        self._destination_store.append(self._motion_destination)
        self._action_store.append(self._current_action)
        self._position_store.append(self._position)
        self._error_store.append(self._controller._error)
        self._load_store.append(self._load)
        self._task_store.append(self._current_task)
        self._time_store.append(time_of_day)
        self._exploration_eta_store.append(self._exploration_eta)
        self._harvest_threshold_store.append(self._harvest_threshold.copy())
        self._maximum_load_store.append(self._maximum_load)
        
        # Debug printing
        if self._print:
            print('t= {}: Agent {} is doing {}'.format(self._current_time, self.unique_id, self._current_action))
            
    ############################################################
    # POLICIES AND IMPLEMENTATIONS
    ############################################################

    def decide_task(self, time_of_day, limits, misc=None) -> None:
                    
        # Close the FSM loop if the current task is finished
        if self._status_task == 'FINISHED':
            self._status_task = 'IDLE'                        
        
        # Atomic implementation of the tasks (default)
        cond_atomic = True

        # Decide the task based on current state
        if cond_atomic and \
            (self._load >= self.np.floor(self._maximum_load) or \
                self._maximum_load <= 0.1 * self._maximum_load_init):
            self._current_task = 'store'                
        elif cond_atomic and \
            self._map_quality_measure_position > 1 * self._harvest_threshold[0] and \
            self._map_quality_measure_position < self._harvest_threshold[1] and \
            (not any(self._position[0] == pos[0] and self._position[1] == pos[1] for pos in self._home_base_position_store)):
            self._current_task = 'harvest'        
        else:
            self._harvesting_actions_counter = 0
            self._current_task = 'explore'        
        
        # Set the task status
        self._status_task = 'STARTING'
       
    def do_task(self, time_of_day=None, limits=None, misc=None) -> None:
        """Execute the current task based on its type."""
        if self._current_task == 'explore':
            # First time: get the neighbors and set the status to INPROGRESS
            if self._status_task == 'STARTING':
                self.get_neighbourhood()
                self._status_task = 'INPROGRESS'
            # Call the explore method
            self.explore(time_of_day, limits, misc)
        elif self._current_task == 'harvest':
            # First time: set the status to INPROGRESS
            if self._status_task == 'STARTING':                
                self._status_task = 'INPROGRESS'
            # Call the harvest method
            self.harvest(time_of_day, limits, misc)
        elif self._current_task == 'store':
            # First time: set the status to INPROGRESS
            if self._status_task == 'STARTING':   
                self.set_home_base_position()             
                self._status_task = 'INPROGRESS'
            # Call the store method
            self.store(time_of_day, limits, misc)
        else:
            raise ValueError('Invalid task: {}'.format(self._current_task))
    
    def do_action(self, time_of_day=None, limits=None, misc=None) -> bool:
        """Execute the current action and return success status."""
        requested_action = self._current_action
        
        # Execute the action based on time of day
        if time_of_day == 'day':
            if self._current_action == 'move':
                self._status_robot = 'ACTING'
                self.move(time_of_day, limits, misc)
            elif self._current_action == 'remove_vegetation':
                self._status_robot = 'ACTING'
                self.remove_vegetation(time_of_day, limits, misc)
            elif self._current_action == 'idle':
                self._status_robot = 'IDLE'
        else:
            raise ValueError('Invalid time_of_day: {}'.format(time_of_day))
        
        return requested_action == self._current_action
        
    ############################################################
    # TASKS
    ############################################################
    
    def harvest(self, time_of_day=None, limits=None, misc=None) -> None:
        """Task: Remove vegetation at current position."""
        self._current_action = 'remove_vegetation'
        success = self.do_action(time_of_day, limits, misc)
        
        if success:
            self._status_task = 'FINISHED'                    

    def explore(self, time_of_day=None, limits=None, misc=None) -> None:
        """Task: Explore the environment by visiting neighborhood cells."""
        
        # Decide the action based on motion status
        if self._status_motion == 'IDLE':
            try:
                self._neighbourhood_current_index = self._neighbourhood_reached_flag.index(False)
            except ValueError:
                # All cells selected in the neighborhood (get_neighbourhood method) have been explored
                self._neighbourhood_current_index = None 
                    
            if self._neighbourhood_current_index is not None:
                self._motion_destination = [self._neighbourhood[self._neighbourhood_current_index][0], 
                                            self._neighbourhood[self._neighbourhood_current_index][1]]
                # Clip the destination to limits
                if limits is not None:
                    self._motion_destination[0] = self.np.clip(self._motion_destination[0], limits[0][0], limits[0][1])
                    self._motion_destination[1] = self.np.clip(self._motion_destination[1], limits[1][0], limits[1][1])
                    
                self._current_action = 'move'
                self._status_task = 'INPROGRESS'           
            else:
                self._motion_destination = None   
                self._current_action = 'idle'           
                self._status_task = 'FINISHED'
                
        elif self._status_motion == 'ACTIVE':
            self._current_action = 'move'           
            self._status_task = 'INPROGRESS'
            
        elif self._status_motion == 'FINISHED':
            # Mark the cell as explored
            self._neighbourhood_reached_flag[self._neighbourhood_current_index] = True
            self._controller._error_integral = 0.0
            self._controller._error_old = 0.0
            
            self._current_action = 'move'           
            self._status_task = 'FINISHED'
        else:
            raise ValueError('Invalid status_motion: {}'.format(self._status_motion))
            
        # Execute the action
        success = self.do_action(time_of_day, limits, misc)
        
    def store(self, time_of_day=None, limits=None, misc=None) -> None:
        """Task: Move to home base and store collected vegetation."""
        self._current_action = 'move'
        
        # Temporarily reduce control gains for smoother approach to home
        self._controller._Kp = 0.5 * self._controller._Kp_init
        self._controller._Kd = 0.5 * self._controller._Kd_init
        self._controller._Ki = 0.5 * self._controller._Ki_init
        success = self.do_action(time_of_day, limits, misc)
        
        # Transfer the load to the home base when reached
        if success and self._status_motion == 'FINISHED':                
            self.store_vegetation(time_of_day, limits)
            
            # Restore original control gains
            self._controller._Kp = self._controller._Kp_init
            self._controller._Kd = self._controller._Kd_init
            self._controller._Ki = self._controller._Ki_init
            
            self._status_task = 'FINISHED'
            self._current_action = 'idle' 
            self._in_water_counter = 0
            self._maximum_load = self._maximum_load_init

    ############################################################
    # ACTIONS
    ############################################################
    
    def move(self, time_of_day=None, limits=None, misc=None) -> None:
        """Action: Move towards destination using selected controller."""
        if self._controller._name == 'P':
            setpoint = self._motion_destination
            self._controller.step(setpoint, self._position)
            
        elif self._controller._name == 'P_repulsive':            
            setpoint = self._motion_destination
            _neighbourhood = module_misc.DN_neighbourhood(self._position, limits, N=self._controller._neighbourhood_size, step=self._measure_step)
            
            # Select which map to use for repulsion
            if self._controller._map_repulsive == 'vegetation_quality':
                map_repulsive = self._local_map.copy()
            elif self._controller._map_repulsive == 'vegetation_visits':
                map_repulsive = self._local_map.copy() / (1 + self._local_map_visits.copy())
            else:
                raise ValueError('Invalid map_repulsive value: {}'.format(self._controller._map_repulsive))
            
            self._local_map_control = map_repulsive

            # Get base neighborhood values for control
            _neighbourhood_values = [self._local_map_control[int(pos[0]), int(pos[1])] for pos in _neighbourhood]                        
            
            # Apply flow dynamics to water cells
            flow_direction = self.np.array(misc.get('direction', [1, 0]))
            flow_strength = misc.get('strength', 1.0)
            
            for i, pos in enumerate(_neighbourhood):
                cell_x, cell_y = int(pos[0]), int(pos[1])
                
                # Check if this neighborhood cell is water
                if self._local_map[cell_x, cell_y] < 0:
                    # Vector from robot position to this neighborhood cell
                    direction_to_cell = self.np.array([cell_x - self._position[0], 
                                                      cell_y - self._position[1]])
                    
                    if self.np.linalg.norm(direction_to_cell) > 0:
                        # Normalize direction vector
                        direction_to_cell = direction_to_cell / self.np.linalg.norm(direction_to_cell)
                        
                        # Calculate alignment with flow direction (-1 to 1)
                        # +1 = same direction as flow (easier), -1 = against flow (harder)
                        flow_alignment = self.np.dot(direction_to_cell, flow_direction) / self.np.linalg.norm(flow_direction)

                        # Apply flow bias: reduce cost for downstream movement, increase for upstream
                        flow_modifier = -flow_strength * flow_alignment
                        if self.np.isnan(flow_modifier):
                            flow_modifier = 0
                        _neighbourhood_values[i] += flow_modifier
            
            self._controller.step(setpoint, self._position, [_neighbourhood, _neighbourhood_values])
        else:
            raise ValueError('Invalid controller name: {}'.format(self._controller._name))

        # Update dynamics and position
        self._dynamics.step(self._controller._output)
        self._position = list([int(coord) for coord in self._dynamics._output[0]])  
        
        # Clip the position to valid range
        self._position[0] = self.np.clip(self._position[0], limits[0][0], limits[0][1])
        self._position[1] = self.np.clip(self._position[1], limits[1][0], limits[1][1])
                    
        # Update motion status            
        self._status_motion = self._controller._status
        
    def remove_vegetation(self, time_of_day=None, limits=None, misc=None) -> None:
        """Action: Remove vegetation at current position and add to load."""
        # Cannot harvest from water        
        if self._map_quality_measure_position < 0:
            return
            
        potential_new_value = self._map_quality_measure_position - self._vegetation_removal
        
        # Check if removal would result in negative values (water creation)
        if potential_new_value < 0:
            # Check if there's a river (negative value) in D8 neighborhood
            if limits is not None:
                # Get D8 neighborhood around current position
                d8_neighborhood = module_misc.DN_neighbourhood(self._position, limits, N=2, step=self._measure_step)

                # Check if any neighboring cell has negative values (is water/river)
                river_nearby = False
                for neighbor_pos in d8_neighborhood:
                    neighbor_x, neighbor_y = int(neighbor_pos[0]), int(neighbor_pos[1])
                    
                    # Check bounds to avoid index errors
                    if (0 <= neighbor_x < self._local_map.shape[0] and 
                        0 <= neighbor_y < self._local_map.shape[1]):
                        
                        if self._local_map[neighbor_x, neighbor_y] < 0:
                            river_nearby = True
                            break
                
                # Only allow digging to negative values if river is nearby
                if not river_nearby:
                    # Limit removal to prevent going below 0
                    max_allowed_removal = max(0, self._map_quality_measure_position)
                    if max_allowed_removal > 0:
                        actual_removal = min(self._vegetation_removal, max_allowed_removal)
                        self._harvesting_actions_counter += 1
                        self._map_quality_measure_position -= actual_removal
                        self._load += actual_removal
                    return
        
        # Normal harvesting (either won't go negative, or river is nearby)
        self._harvesting_actions_counter += 1        
        self._map_quality_measure_position -= self._vegetation_removal
        self._load += self._vegetation_removal        

    def store_vegetation(self, time_of_day=None, limits=None, misc=None) -> bool:
        """Action: Store collected vegetation at home base position."""
        if self._load > 0:
            available_space = self.np.inf * self._vegetation_quality_range[1] - self.np.ceil(self._map_quality_measure_position)
            
            if available_space > 0:
                removed_load = min(available_space, self._load)
                self._map_quality_measure_position += removed_load
                self._map_quality_measure_position = self.np.clip(self._map_quality_measure_position, 
                                                                 self._vegetation_quality_range[0],
                                                                 self._vegetation_quality_range[1])
                self._load -= removed_load
                return True
            else:
                return False       
        
    ############################################################
    # UTILITIES
    ############################################################
        
    def get_neighbourhood(self) -> None:
        """Compute the neighborhood cells to explore based on exploration strategy."""

        if self.np.any(self._neighbourhood_reached_flag) == False and self._neighbourhood is not None:
            # Already have a neighborhood, no need to recompute
            return
        
        # Current position                 
        position = self._position

        # Behavioral logic model
        eps = 1e0      
        threshold_mask = (self._local_map >= self._harvest_threshold[0]) & (self._local_map <= self._harvest_threshold[1])
        local_map = self._local_map.copy()
        local_visits_map = self._local_map_visits.copy()
        
        # Rescale local_visits_map between 0 and self._vegetation_quality_range[1]
        visits_min = self.np.nanmin(local_visits_map)
        visits_max = self.np.nanmax(local_visits_map)
        if visits_max > visits_min:  # Avoid division by zero
            local_visits_map = (local_visits_map - visits_min) / (visits_max - visits_min) * self._vegetation_quality_range[1]
        else:
            local_visits_map = self.np.zeros_like(local_visits_map)
            
        # Apply exploration strategy based on map type
        if self._exploration_map == 'vegetation_quality':                        
            local_map[~threshold_mask] = self.np.nan
            if self._role == 'explorer':
                local_map[threshold_mask] = (eps + local_map[threshold_mask])**2
            elif self._role == 'builder':
                local_map[threshold_mask] = 1/(eps + local_map[threshold_mask])**2
        elif self._exploration_map == 'vegetation_visits':            
            local_map[~threshold_mask] = self.np.nan
            if self._role == 'explorer':
                local_map[threshold_mask] = (eps + local_visits_map[threshold_mask])**2 * (eps + local_map[threshold_mask])**2
            elif self._role == 'builder':
                local_map[threshold_mask] = (eps + local_visits_map[threshold_mask])**2/((eps + local_map[threshold_mask])**2)
        else:
            raise ValueError('Invalid exploration_map value: {}'.format(self._exploration_map))
          
        if self._local_map is not None:
            limits = [[0, self._local_map.shape[0] - 1], [0, self._local_map.shape[1] - 1]]
        else:
            limits = [[self._position[0], self._position[0]], [self._position[1], self._position[1]]]                    
            
        # Split exploration_mode into two parts: prefix and suffix
        if len(self._exploration_mode) > 2:
            exploration_prefix = self._exploration_mode[:-3]
            exploration_suffix = int(self._exploration_mode[-3:])
        else:
            exploration_prefix = self._exploration_mode
            exploration_suffix = None                    
            
        if exploration_prefix == 'gradient_D':
            N, NF, NI = module_beaver.exploration_gradient_DN(position, limits, local_map, N=exploration_suffix, \
                home_base_store=self._home_base_position_store, eta=self._exploration_eta, \
                N_recovery=self._exploration_N_recovery, step=self._measure_step)        
        else:
            raise ValueError('Invalid exploration mode: {}'.format(self._exploration_mode))            
        
        self._neighbourhood = N
        self._neighbourhood_reached_flag = NF
        self._neighbourhood_current_index = NI
        
        
    def update_local_map(self, map_quality, misc=None) -> None:
        """Update the local map with new measurements from the environment."""
        # Ensure the local map is initialized
        if self._local_map is None:
            self._local_map = self.np.ones((1, 1)) * self.np.nan            
            
        # Extract positions and values
        measure_positions = map_quality[0]
        measure_values = map_quality[1]

        # Calculate visit increase based on load changes
        bias = 1
        delta_store = self.np.diff(self._load_store[-(self._n_traces+1):-1]) if len(self._load_store) > self._n_traces else 0
        delta_load = bias + self.np.sum(delta_store)
        increase = delta_load
            
        # If seeing the whole map
        if map_quality[0] == 'all':            
            self._local_map = measure_values[0]
            global_map_visits = misc.get('visits', None)
            self._local_map_visits = global_map_visits                                
            self._local_map_visits[self._position[0], self._position[1]] += increase            
            return                    

        # Expand the matrix if the position is out of bounds
        x = self.np.max([pos[0] for pos in measure_positions])
        y = self.np.max([pos[1] for pos in measure_positions])
        if x >= self._local_map.shape[0]:
            # local map
            self._local_map = self.np.pad(self._local_map, ((0, x - self._local_map.shape[0] + 1), (0, 0)), 
                                                mode='constant', constant_values=self.np.nan)                        
            
        if y >= self._local_map.shape[1]:
            # local map
            self._local_map = self.np.pad(self._local_map, ((0, 0), (0, y - self._local_map.shape[1] + 1)), 
                                                mode='constant', constant_values=self.np.nan)                        

        # Update the vegetation quality at the current position        
        for pos, val in zip(measure_positions, measure_values):            
            self._local_map[pos[0], pos[1]] = val        
        
        # Track how much time was spent in water
        if self._local_map[self._position[0], self._position[1]] < 0:
            self._in_water_counter += 1
        else:
            self._in_water_counter = 0
        
    def set_home_base_position(self) -> None:
        """Set the motion destination to the closest home base."""
        # Get distance to each home base
        distances_to_home = [
            self.np.sqrt(
                (self._position[0] - home_pos[0]) ** 2 +
                (self._position[1] - home_pos[1]) ** 2
            )
            for home_pos in self._home_base_position_store
        ]
                
        # Define the motion destination as the closest home base position
        self._motion_destination = min(
            self._home_base_position_store,
            key=lambda pos: self.np.sqrt(
                (self._position[0] - pos[0]) ** 2 +
                (self._position[1] - pos[1]) ** 2
            )
        )

    def select_exploration_eta(self, dt, dt_percentage, decay):
        """Adaptively adjust exploration parameters based on recent performance."""
        decay_eta = decay[0]        
        harvest_decay = decay[1]
        load_decay = decay[2]
        
        load_derivative = self.np.diff(self._load_store[-(dt+1):-1])
        self._wait_b4_explore += 1
        
        if len(load_derivative) >= dt-1 and self._wait_b4_explore >= dt:
            self._wait_b4_explore = 0            
            if (load_derivative > 0).sum() < dt_percentage * dt and self._load < self._maximum_load:
                # Decay if not making progress
                self._exploration_eta = (1-decay_eta) * self._exploration_eta
                self._maximum_load = (1-load_decay) * self._maximum_load
                self._harvest_threshold = [(1-harvest_decay) * self._harvest_threshold[0], (1+harvest_decay) * self._harvest_threshold[1]]
            else:
                # Reset to initial values if load is decreasing
                self._exploration_eta = self._exploration_eta_init
                self._harvest_threshold = self._harvest_threshold_init
                self._maximum_load = self._maximum_load_init

            # Clip values to valid ranges
            self._exploration_eta = self.np.clip(self._exploration_eta, 1e-2, 10)
            self._maximum_load = self.np.clip(self._maximum_load, 0.0, self.np.inf)
            self._harvest_threshold[0] = self.np.clip(self._harvest_threshold[0], 0.0, self._vegetation_quality_range[1])
            self._harvest_threshold[1] = self.np.clip(self._harvest_threshold[1], 0.0, self._vegetation_quality_range[1])
            self._harvest_threshold = sorted(self._harvest_threshold)