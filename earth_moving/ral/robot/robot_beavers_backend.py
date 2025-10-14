# general imports

# backend imports
from earth_moving.ral.robot.robot_backend import BaseRobotBackend

# module imports
from earth_moving.ral.robot.modules.module_control import Controller
from earth_moving.ral.robot.modules.module_control import Dynamics
import earth_moving.ral.algorithms.module_misc as module_misc
import earth_moving.ral.robot.modules.module_beaver as module_beaver


class BeaversRobotBackend(BaseRobotBackend):
    """
    Robot backend implementation for beaver-like agents that perform vegetation management tasks.
    
    This class implements a finite state machine (FSM) based robot that can explore environments,
    harvest vegetation, and transport materials to storage locations. The robot operates with
    three main behavioral states: exploring, harvesting, and storing.
    
    The robot simulates beaver-like behavior including:
    - Autonomous exploration of the environment using various strategies
    - Vegetation harvesting based on quality thresholds
    - Material transport and storage at designated home base locations
    - Energy management with consumption and recovery mechanics
    - Adaptive pathfinding with obstacle avoidance
    
    State Management:
    ----------------
    The robot maintains three status indicators:
    
    - status_robot: Physical state of the robot
        * IDLE: Not performing any action
        * ACTING: Currently executing an action
        * TIRED: Insufficient energy to perform requested action
    
    - status_task: High-level task execution state
        * IDLE: Waiting for a new task
        * STARTING: Beginning task execution
        * INPROGRESS: Currently executing task
        * FINISHED: Task completed successfully
    
    - status_motion: Movement state
        * IDLE: Not moving
        * ACTIVE: Currently in motion
        * FINISHED: Reached destination
    
    Actions:
    --------
    - idle: No action, constant energy consumption
    - sleep: Energy recovery mode (typically during night)
    - move: Navigate to destination, consumes energy based on load
    - remove_vegetation: Harvest vegetation, increases load
    - store_vegetation: Deposit materials at home base, decreases load
    
    Tasks:
    ------
    - explore: Search for new areas using configurable exploration strategies
    - harvest: Collect vegetation when quality thresholds are met
    - store: Transport collected materials to home base locations
    
    Exploration Strategies:
    ----------------------
    - D[N]: Deterministic neighborhood exploration (N=0,4,8,24,40)
    - random_D[N]: Randomized neighborhood exploration
    - gradient_D[N]: Gradient-based exploration favoring high vegetation quality
    - softmax_D[N]: Probabilistic exploration using softmax distribution
    
    Energy Management:
    -----------------
    Energy consumption varies by action:
    - Motion: Base consumption + load-dependent penalty
    - Harvesting: Fixed consumption per harvest action
    - Sleep: Energy recovery at configurable rate
    
    Attributes:
    -----------
    _position : list[int, int]
        Current [x, y] coordinates in the environment
    _energy : int
        Current energy level (0-100)
    _load : int
        Current carried load amount
    _local_map : numpy.ndarray
        Agent's knowledge of vegetation quality distribution
    _local_map_visits : numpy.ndarray
        Visit frequency tracking for exploration optimization
    _home_base_position_store : list
        Locations of established home base positions
    
    Examples:
    ---------
    >>> robot = BeaversRobotBackend()
    >>> robot.initiate_robot(
    ...     position='home',
    ...     initial_energy=100,
    ...     exploration_mode='gradient_D8',
    ...     harvest_threshold=[0.3, 0.8]
    ... )
    >>> robot.step_beaver(dt=1.0, time_of_day='day', map_quality=quality_data, limits=bounds)
    
    Notes:
    ------
    The implementation follows ecological principles of beaver behavior including
    territorial establishment, resource optimization, and adaptive foraging strategies.
    The robot maintains persistent knowledge of the environment and can establish
    multiple home base locations for efficient resource management.
    """
    
    def __init__(self, **kwargs) -> None:
        """
        Initialize the BeaversRobotBackend.
        
        Sets up the robot's internal state machine and behavioral framework.
        The robot operates on a task-action hierarchy where high-level tasks
        (explore, harvest, store) are decomposed into specific actions
        (move, remove_vegetation, store_vegetation, sleep, idle).
        
        The initialization establishes the foundational state tracking for:
        - Robot status management (IDLE/ACTING/TIRED)
        - Task execution flow (IDLE/STARTING/INPROGRESS/FINISHED)  
        - Motion control state (IDLE/ACTIVE/FINISHED)
        
        Parameters
        ----------
        **kwargs : dict
            Keyword arguments passed to parent BaseRobotBackend constructor
            
        Notes
        -----
        This method only sets up the state machine framework. Actual robot
        parameters and physical attributes are configured in initiate_robot().
        
        The state machine design ensures proper task sequencing and prevents
        conflicting actions during execution.
        """
        super().__init__(**kwargs) 
        
        #TODO store total trail lenght
        #TODO store total canal lenght
        #TODO store total vegetation of high quality
        #TODO if not already done, initialize a trail
        
        # TASK RECAP
        # Exploring: decide a (set) of destination(s) to explore
        #   - move(destination)
        #   - read(map_quality)
        # Harvesting: harvest vegetation where you are
        #   - remove_vegetation()
        # Storing: store the harvested vegetation in the home base
        #   - move(destination)
        #   - store_vegetation()

        # ACTION RECAP
        # Idle: do nothing                          -> constant energy, constant load
        # Sleep: recover energy                     -> increase energy, constant load
        # Move: move to a destination               -> consume energy, constant load
        # Remove_vegetation: harvest vegetation     -> consume energy, increase load
        # Store_vegetation: store vegetation        -> consume energy, decrease load
        
        # STATE RECAP
        # status_robot: #! changed in do_action
        #   IDLE: the agent is not doing anything (action: idle)
        #   ACTING: the agent is doing an action        
        #   TIRED: the agent is tired (action: idle because it does not have enough energy)
        # status_task: #! changed in decide_task, do_task
        #   IDLE: waiting for a task -> STARTING
        #   STARTING: starting a task -> INPROGRESS
        #   INPROGRESS: doing a task -> FINISHED
        #   FINISHED: just finished a task -> IDLE
        # status_motion: #! changed in move
        #   IDLE: the agent is not moving
        #   ACTIVE: the agent is moving
        #   FINISHED: the agent has reached the destination
        
        # OBSERVATION RECAP
        # time_of_day: day or night (provided by the environment)
        # map_quality: quality of the vegetation at the current position (provided by the environment)
        # limits: limits of the environment (provided by the environment)
        
        # ACTUATION RECAP
        # update_map_quality: update the vegetation quality at the current position (actuate the environment)
        
    def initiate_robot(self, **kwargs):
        """
        Configure and initialize all robot parameters and subsystems.
        
        This method performs comprehensive robot setup including physical attributes,
        behavioral parameters, control systems, and internal state initialization.
        It must be called after __init__ but before any robot operations.
        
        Parameters
        ----------
        **kwargs : dict
            Robot configuration parameters. Key parameters include:
            
            Physical Parameters:
            - maximum_load : int, optional
                Maximum carrying capacity (default: infinite)
            - position : str or list, optional  
                Initial position: 'random', 'home', [x,y] coordinates (default: [0,0])
            - initial_energy : str or int, optional
                Starting energy: 'random' or 0-100 value (default: 100)
            - initial_load : str or int, optional
                Starting load: 'random' or value up to maximum_load (default: 0)
                
            Environment Parameters:
            - range_x, range_y : list[int, int]
                Valid coordinate ranges for robot movement
            - home_base_position : list[int, int]
                Primary home base coordinates
                
            Behavioral Parameters:
            - exploration_mode : str
                Exploration strategy: 'D[N]', 'random_D[N]', 'gradient_D[N]', 'softmax_D[N]'
                where N ∈ {0,4,8,24,40} specifies neighborhood size
            - exploration_map : str
                Map type for exploration: 'vegetation_quality' or 'vegetation_visits'
            - exploration_eta : float
                Learning rate for gradient-based exploration
            - exploration_N_recovery : int
                Visit count reset factor
                
            Energy Parameters:
            - motion_consumption : float
                Base energy cost for movement
            - load_consumption : float  
                Additional energy cost per unit load during movement
            - harvest_consumption : float
                Energy cost for vegetation harvesting
            - sleep_recovery : float
                Energy gained per sleep action
                
            Harvesting Parameters:
            - harvest_threshold : list[float, float]
                [min, max] vegetation quality range for harvesting
            - vegetation_removal : float
                Amount of vegetation harvested per action
            - harvesting_actions_limit : int
                Maximum consecutive harvesting actions (default: 4)
                
            Control Parameters:
            - measurement_mode : str
                Sensor configuration mode
            - print : bool, optional
                Enable debug output (default: False)
        
        Returns
        -------
        self : BeaversRobotBackend
            Returns self for method chaining
            
        Raises
        ------
        ValueError
            If invalid values provided for position, initial_energy, or initial_load
            
        Notes
        -----
        The method initializes several key subsystems:
        
        1. Controller: Handles pathfinding and motion control
        2. Dynamics: Manages physical movement simulation  
        3. Local mapping: Tracks environmental knowledge
        4. State machines: Sets up status tracking
        5. Data storage: Initializes history tracking arrays
        
        Position initialization supports multiple modes:
        - 'random': Random location within specified ranges
        - 'home': Start at designated home base
        - [x,y]: Specific coordinates (clipped to valid range)
        
        Energy and load can be initialized randomly or to specific values,
        with automatic clipping to valid ranges.
        
        Examples
        --------
        >>> robot = BeaversRobotBackend()
        >>> robot.initiate_robot(
        ...     position='home',
        ...     initial_energy=100,
        ...     maximum_load=50,
        ...     exploration_mode='gradient_D8',
        ...     harvest_threshold=[0.3, 0.8],
        ...     motion_consumption=2.0,
        ...     harvest_consumption=5.0
        ... )
        """
        super().initiate_robot(**kwargs)                
        
        # parse _robot
        self._maximum_load = self._robot.get('maximum_load') 
        if self._maximum_load is None:
            self._maximum_load = self.np.inf
        self._maximum_load_init = self._maximum_load
        self._print = self._robot.get('print')
        
        # custom attributes                        
        self._range_x = self._robot.get('range_x')
        self._range_y = self._robot.get('range_y')
        self._exploration_mode = self._robot.get('exploration_mode')
        self._exploration_map = self._robot.get('exploration_map')
        self._exploration_eta = self._robot.get('exploration_eta')
        self._exploration_N_recovery = self._robot.get('exploration_N_recovery')
        self._motion_consumption = self._robot.get('motion_consumption')
        self._load_consumption = self._robot.get('load_consumption')
        self._harvest_consumption = self._robot.get('harvest_consumption')        
        self._sleep_recovery = self._robot.get('sleep_recovery')
        self._vegetation_removal = self._robot.get('vegetation_removal')
        self._measurement_mode = self._robot.get('measurement_mode')
        self._measure_step = self._robot.get('measure_step', 1)
        self._home_base_position = list(self._robot.get('home_base_position'))[0]
        self._home_base_position_store = list(self._robot.get('home_base_position'))
        self._stomping_interval = self._robot.get('stomping_interval')
        self._stomping_removal = self._robot.get('stomping_removal')
        self._n_traces = self._robot.get('n_traces')
        self._decay_values = self._robot.get('decay_values')
        
        self._harvest_threshold_list = self._robot.get('harvest_threshold')
        means = self.np.mean(self._harvest_threshold_list, axis=0)
        if isinstance(self._harvest_threshold_list, list) and len(self._harvest_threshold_list) > 0:
                base = list(self.random.choice(self._harvest_threshold_list))
                # Randomly select a position around the chosen home base within the allowed range
                self._harvest_threshold = base
                if base[0] > means[0]:
                    self._role = 'explorer'
                else:
                    self._role = 'expander'
        else:
            raise ValueError('harvest_threshold must be a non-empty list for random_home initialization')

        # other attributes
        self._current_time = 0 #! this is a counter. Agent does not compute the hour/time of the day. It will be provided by the environment        
        self._map_quality_measure = None
        self._map_quality_measure_position = None
        self._map_quality_update = False #! this is a flag to update the vegetation quality in the environment     
        self._motion_destination = None
        self._neighbourhood = None        
        self._neighbourhood_reached_flag = None
        self._neighbourhood_current_index = None
        self._local_map = None
        self._local_map_visits = None
        self._harvesting_actions_counter = 0
        self._harvesting_actions_limit = 1
        self._in_water_counter = 0
        self._reset_integral = 50
        self._min_home_distance = 500
        
        # from environment
        self._vegetation_quality_range = None        
        
        # physical attributes
        # position
        position = self._robot.get('position')
        if position is 'random':
            self._position = [self.random.randint(self._range_x[0], self._range_x[1] - 1), 
                              self.random.randint(self._range_y[0], self._range_y[1] - 1)]
        elif position is 'home':
            self._position = self._home_base_position            
        elif position is 'random_home':
            # Choose a random home base from the list
            if isinstance(self._home_base_position_store, list) and len(self._home_base_position_store) > 0:
                base = list(self.random.choice(self._home_base_position_store))
                # Randomly select a position around the chosen home base within the allowed range
                self._position = [
                    base[0] + self.random.randint(self._range_x[0], self._range_x[1] - 1),
                    base[1] + self.random.randint(self._range_y[0], self._range_y[1] - 1)
                ]
            else:
                raise ValueError('home_base_position must be a non-empty list for random_home initialization')
        elif position is None:
            self._position = [0,0]
        elif isinstance(position,list):
            self._position = position
            self._position[0] = self.np.clip(self._position[0], self._range_x[0], self._range_x[1])
            self._position[1] = self.np.clip(self._position[1], self._range_y[0], self._range_y[1])
        else:
            raise ValueError('Invalid position value: {}'.format(position))
        
        # energy
        initial_energy = self._robot.get('initial_energy')
        if initial_energy is 'random':
            self._energy = self.random.randint(0,100)
        elif initial_energy is None:
            self._energy = 100
        elif isinstance(initial_energy,int):
            self._energy = self.np.clip(initial_energy, 0, 100)
        else:
            raise ValueError('Invalid initial_energy value: {}'.format(initial_energy))
        
        # load
        initial_load = self._robot.get('initial_load')
        if initial_load is 'random':
            self._load = self.random.randint(0,self._maximum_load)
        elif initial_load is None:
            self._load = 0
        elif isinstance(initial_load,int):
            self._load = self.np.clip(initial_load, 0, self._maximum_load)
        else:
            raise ValueError('Invalid initial_load value: {}'.format(initial_load))                                
        
        # motion policy                        
        self._controller = Controller(**self._robot)
        initial_state = self.np.array([self._position, self.np.zeros(self._controller._dimension)])
        self._dynamics = Dynamics(initial_state, **self._robot)        
        self._local_map_control = None        
        
        self._status_robot = 'IDLE'      
        self._status_task = 'IDLE'  
        self._status_motion = 'IDLE'
        self._current_task = None
        self._current_action = None
        
        # storage
        self._destination_store = []
        self._action_store = []
        self._position_store = []
        self._error_store = []
        self._energy_store = []
        self._load_store = []
        self._task_store = []
        self._time_store = []
        self._exploration_eta_store = []        
        self._harvest_threshold_store = []
        self._maximum_load_store = []
        self._exploration_eta_init = self._exploration_eta
        self._harvest_threshold_init = self._harvest_threshold.copy()
        self._wait_b4_explore_init = 1*24
        self._wait_b4_explore = 0
        return self
    
    def step_beaver(self, dt, time_of_day, map_quality, limits, misc=None) -> None:
        """
        Execute one simulation step of the beaver robot behavior.
        
        This is the main execution loop that advances the robot's state by one time step.
        It processes environmental observations, makes behavioral decisions, executes
        actions, and updates internal state. The method implements the complete
        sense-plan-act cycle for autonomous robot operation.
        
        The execution sequence follows this pipeline:
        1. Update internal time and system parameters
        2. Process environmental observations (map quality, time of day)
        3. Update local environmental knowledge
        4. Execute task policy (decide what to do)
        5. Execute task implementation (how to do it)
        6. Update energy based on actions performed
        7. Determine environmental actuation needs
        8. Store historical data for analysis
        
        Parameters
        ----------
        dt : float
            Time step duration for integration and energy calculations
        time_of_day : str
            Current time period: 'day' or 'night'
            Affects available actions (sleep enforced during night)
        map_quality : list
            Environmental sensor data in format:
            [measure_positions, measure_values] where:
            - measure_positions: [[x1,y1], [x2,y2], ..., [xn,yn]] or 'all'
            - measure_values: [vegetation_values, current_position_value]
        limits : list
            Environment boundaries [[x_min, x_max], [y_min, y_max]]
            Used for position clipping and motion planning
            
        Notes
        -----
        Environmental Actuation:
        The robot can modify the environment through vegetation harvesting
        and material deposition. The _map_quality_update flag signals when
        environmental changes should be applied.
        
        Time of Day Effects:
        - 'day': All actions available based on energy and task requirements
        - 'night': Forces sleep action for energy recovery
        
        Energy Management:
        Energy consumption depends on current action:
        - Movement: Base cost + load penalty
        - Harvesting: Fixed cost per action
        - Sleep: Energy recovery
        - Idle: Minimal/no consumption
        
        State Persistence:
        All robot states, actions, and positions are stored in history arrays
        for post-simulation analysis and debugging.
        
        Map Quality Format:
        - Standard: [[[x1,y1],[x2,y2],...], [v1,v2,...,v_current]]
        - Global: ['all', [full_map_array, current_position_value]]
        
        Examples
        --------
        >>> # Single step execution
        >>> quality_data = [[[5,5], [6,6]], [0.7, 0.5]]
        >>> bounds = [[0, 100], [0, 100]]
        >>> robot.step_beaver(
        ...     dt=1.0,
        ...     time_of_day='day', 
        ...     map_quality=quality_data,
        ...     limits=bounds
        ... )
        
        >>> # Night time step (forces sleep)
        >>> robot.step_beaver(
        ...     dt=1.0,
        ...     time_of_day='night',
        ...     map_quality=quality_data, 
        ...     limits=bounds
        ... )
        """
        # update your internal clock
        self._current_time += dt
        # set integration time
        self._controller._dt = dt                
        self._dynamics._dt = dt     
                        
        # gather information (OBSERVATIONS)
        #! map quality = [measure_positions, measure_values]
        #! measure_positions = [[x1,y1],[x2,y2],...,[xn,yn]]
        #! measure_values = [[v1,v2,...,vn], vposition]  
        #? Do I read the measurements also at night?
        self._map_quality_measure = map_quality 
        #! this is the quality at the current position (see module_misc in the visualizer)
        self._map_quality_measure_position = self._map_quality_measure[1][-1] 
        
        # update the local_map according to the measurements
        self.update_local_map(self._map_quality_measure, misc)
        self._map_quality_update = False #! reset the flag
        
        if self._current_time % self._reset_integral == 0:
            self._controller._error_integral = 0.0
        # decide the goal (TASK POLICY)
        self.decide_task(time_of_day, limits, misc)
        # do the task (TASK IMPLEMENTATION)
        self.do_task(time_of_day, limits, misc)
        # update energy
        self.update_energy()
        #! decay order parameter: exploration_eta, harvest_threshold, maximum_load
        self.select_exploration_eta(dt=self._wait_b4_explore_init, dt_percentage=0.1, decay=self._decay_values)

        #! this is where we change/actuate the environment
        if (self._current_task == 'harvest' and self._status_task == 'FINISHED') or \
            (self._current_task == 'store' and \
                (self._status_task == 'FINISHED') or (self._status_task == 'INPROGRESS' and self._status_motion == 'FINISHED')):
            self._map_quality_update = True
        else:                        
            self._map_quality_update = False                       
            
        # update the local_map according to the action
        if self._map_quality_update == True:
            self._map_quality_measure = [[self._position], [self._map_quality_measure_position]]
            self.update_local_map(self._map_quality_measure, misc)
        #! here I also account for the stomping effect
        if self._local_map[self._position[0], self._position[1]] > self._stomping_interval[0] \
            and self._local_map[self._position[0], self._position[1]] < self._stomping_interval[1]:
            stomped_val = max(0,self._local_map[self._position[0], self._position[1]] - self._stomping_removal)
            self._local_map[self._position[0], self._position[1]] = stomped_val            
        self._map_quality_measure_position = self._local_map[self._position[0], self._position[1]]
            

        # store the data
        self._destination_store.append(self._motion_destination)
        self._action_store.append(self._current_action)
        self._position_store.append(self._position)
        self._error_store.append(self._controller._error)
        self._energy_store.append(self._energy)
        self._load_store.append(self._load)
        self._task_store.append(self._current_task)
        self._time_store.append(time_of_day)
        self._exploration_eta_store.append(self._exploration_eta)
        self._harvest_threshold_store.append(self._harvest_threshold.copy())
        self._maximum_load_store.append(self._maximum_load)
        
        # prints
        if self._print:
            print('t= {}: Agent {} is doing {}'.format(self._current_time, self.unique_id, self._current_action))
            
    ############################################################
    # POLICIES and IMPLEMENTATIONS
    ############################################################

    def decide_task(self, time_of_day, limits, misc=None) -> None:
        """
        Implement the high-level task selection policy.
        
        This method contains the core behavioral logic that determines what
        the robot should do based on its current state and environmental
        conditions. It implements a priority-based decision system with
        atomic task execution to prevent task switching mid-execution.
        
        Decision Priority (highest to lowest):
        1. Store materials when load reaches maximum capacity
        2. Harvest vegetation when quality and conditions are suitable
        3. Explore environment (default fallback behavior)
        
        The policy considers multiple factors:
        - Current load vs. maximum capacity
        - Vegetation quality at current position vs. harvest thresholds
        - Harvesting action limits to prevent over-exploitation
        - Home base proximity to avoid harvesting at storage locations
        
        Parameters
        ----------
        time_of_day : str
            Current time period ('day' or 'night')
            Currently not used in decision logic but available for extensions
        limits : list
            Environment boundaries [[x_min, x_max], [y_min, y_max]]
            Used for spatial reasoning in task decisions
            
        Notes
        -----
        Task Selection Logic:
        
        1. **Storage Priority**: Triggered when load ≥ floor(maximum_load)
           Forces immediate transport to home base to prevent overloading
           
        2. **Harvesting Conditions**: All must be met:
           - Vegetation quality within harvest_threshold range
           - Current load < maximum_load (space available)
           - Harvesting action counter < limit (prevents over-exploitation)
           - Not currently at a home base position
           
        3. **Exploration Default**: When other conditions not met
           Resets harvesting counter to allow future harvesting
           
        Atomic Execution:
        Tasks execute atomically (cond_atomic check) to prevent interruption
        during critical operations. This ensures task completion before
        policy re-evaluation.
        
        State Management:
        - Closes FSM loop by transitioning FINISHED → IDLE
        - Sets task status to STARTING for new task initiation
        - Updates _current_task for execution systems
        
        The harvesting action counter prevents robots from repeatedly
        harvesting the same location, encouraging spatial exploration
        and sustainable resource management.
        
        Examples
        --------
        Typical decision sequence:
        1. Robot explores until finding high-quality vegetation
        2. Harvests until load limit or action limit reached  
        3. Stores materials at home base
        4. Returns to exploration
        """                
                    
        # close the FSM loop
        if self._status_task is 'FINISHED':
            self._status_task = 'IDLE'                        
        
        # Only proceed with a new task decision if the previous task is finished 
        # (atomic execution)
        cond_atomic = self._status_task == 'IDLE'
        # Uncomment this to always allow new task decisions (for debugging or testing)        
        # cond_atomic = True
        
        """
        TASK SELECTION FINITE STATE MACHINE (FSM)
        ==========================================
        
        This FSM implements a priority-based behavioral hierarchy for beaver-like resource management.
        The robot transitions between three main behavioral states based on internal conditions and 
        environmental observations.
        
        FSM State Diagram:
        ┌─────────┐    load ≥ max_load    ┌───────┐
        │EXPLORE  │────────────────────────→│ STORE │
        │         │←──────────────────────┬─│       │
        │ (default│    load < max_load    │ └───────┘
        │  state) │                       │     │
        │         │  quality ∈ [min,max]  │     │ load deposited
        │         │  ∧ load < max_load    │     │
        │         │  ∧ actions < limit    │     │
        │         │  ∧ not_at_home_base   │     │
        │         │                       │     │
        │         │◄──────────────────────┴─────┘
        └─────────┘          ▲
             │               │
             │ quality ∈     │ actions ≥ limit
             │ [min,max]     │ ∨ load ≥ max_load
             │               │ ∨ at_home_base
             ▼               │
        ┌─────────┐          │
        │ HARVEST │──────────┘
        │         │
        └─────────┘
        
        STATE DESCRIPTIONS:
        
        1. STORE (Highest Priority)
           Triggered when: load ≥ floor(maximum_load)
           Behavior: Navigate to nearest home base and deposit materials
           Transitions to: EXPLORE (after successful storage)
           
        2. HARVEST (Medium Priority) 
           Triggered when ALL conditions met:
           - Vegetation quality within harvest thresholds [min, max]
           - Available carrying capacity (load < maximum_load)
           - Harvesting action counter below limit (prevents over-exploitation)
           - Not currently at a home base position
           Behavior: Collect vegetation at current location
           Transitions to: STORE (when load full) or EXPLORE (when conditions no longer met)
           
        3. EXPLORE (Default/Fallback State)
           Triggered when: Neither STORE nor HARVEST conditions are met
           Behavior: Search environment using configured exploration strategy
           Side effect: Resets harvesting action counter to 0
           Transitions to: HARVEST (when suitable vegetation found) or STORE (when load full)
           
        ATOMIC EXECUTION:
        The cond_atomic flag ensures tasks execute atomically - once a task begins,
        it must complete before policy re-evaluation. This prevents task switching
        mid-execution and ensures behavioral consistency.
        
        HARVEST COUNTER RESET:
        The harvesting counter is reset to 0 in the EXPLORE state, allowing
        future harvesting opportunities while preventing robots from repeatedly
        harvesting the same location during a single exploration cycle.
        """
        
        # decide the task
        if  cond_atomic and \
            (self._load >= self.np.floor(self._maximum_load) or self._in_water_counter > 30):
            self._current_task = 'store'                
        elif cond_atomic and \
            self._vegetation_removal > 0 and \
            self._map_quality_measure_position > 1 * self._harvest_threshold[0] and \
            self._map_quality_measure_position < self._harvest_threshold[1] and \
            self._harvesting_actions_counter < self._harvesting_actions_limit and \
            (not any(self._position[0] == pos[0] and self._position[1] == pos[1] for pos in self._home_base_position_store)):
            self._current_task = 'harvest'        
        else:
            self._harvesting_actions_counter = 0
            self._current_task = 'explore'        
        
                        
        # set the task status
        self._status_task = 'STARTING'
            
        # if you get here you're INPROGRESS
       
    # this is the task implementation
    def do_task(self, time_of_day=None, limits=None, misc=None) -> None:
        if self._current_task == 'explore':
            # first time you get the neighbors and set the status to INPROGRESS
            if self._status_task is 'STARTING':
                self.get_neighbourhood()
                self._status_task = 'INPROGRESS'
            self.explore(time_of_day, limits, misc)
        elif self._current_task == 'harvest':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':                
                self._status_task = 'INPROGRESS'
            self.harvest(time_of_day, limits, misc)
        elif self._current_task == 'store':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':   
                self.set_home_base_position()             
                self._status_task = 'INPROGRESS'
            self.store(time_of_day, limits, misc)
        elif self._current_task == 'sleep':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':
                self._status_task = 'INPROGRESS'
            self.sleep(time_of_day, limits, misc)        
        else:
            raise ValueError('Invalid task: {}'.format(self._current_task))
        
    """
    ACTION EXECUTION FINITE STATE MACHINE (FSM)
    ============================================
    
    This FSM implements the low-level action execution layer that translates high-level
    task decisions into specific robot actions. It manages energy constraints, circadian
    rhythms, and robot status transitions during action execution.
    
    FSM Flow Diagram:
    ┌─────────────┐    energy < required    ┌─────────┐
    │ REQUESTED   │─────────────────────────→│  TIRED  │
    │   ACTION    │                          │ (idle)  │
    │             │◄─────────────────────────│         │
    └─────────────┘    energy recovered      └─────────┘
           │
           │ energy ≥ required
           ▼
    ┌─────────────┐
    │   TIME OF   │
    │ DAY CHECK   │
    └─────────────┘
           │
           ├─── NIGHT ───► SLEEP (forced override)
           │                  │
           │                  ▼
           │            ┌─────────────┐
           │            │   ACTING    │
           │            │  (sleep)    │
           │            └─────────────┘
           │
           └─── DAY ────► ACTION DISPATCH
                              │
                              ├─── move ────► ACTING (move)
                              │                   │
                              ├─── remove_veg ──► ACTING (harvest)
                              │                   │
                              └─── idle ────────► IDLE
                                                  │
                                                  ▼
                                            ┌─────────────┐
                                            │   SUCCESS   │
                                            │ VALIDATION  │
                                            └─────────────┘
                                                  │
                                                  ├─── True ────► Action executed successfully
                                                  │
                                                  └─── False ───► Action was overridden/failed
    
    STATE DESCRIPTIONS:
    
    1. ENERGY VALIDATION (Entry Point)
       - Checks if robot has sufficient energy for requested action
       - If insufficient: Forces 'idle' action, sets status to 'TIRED', returns False
       - If sufficient: Proceeds to circadian rhythm check
       
    2. CIRCADIAN RHYTHM ENFORCEMENT
       - NIGHT: Overrides any requested action with 'sleep' for energy recovery
         * Forces sleep regardless of task requirements
         * Sets robot status to 'ACTING'
         * Biological constraint simulation
       - DAY: Proceeds with requested action dispatch
       
    3. ACTION DISPATCH (Day Only)
       Available actions during day:
       - 'move': Navigate to destination
         * Updates robot status to 'ACTING'
         * Calls move() for motion control execution
       - 'remove_vegetation': Harvest vegetation
         * Updates robot status to 'ACTING'  
         * Calls remove_vegetation() for harvesting
       - 'idle': No action (implicit)
         * Maintains current robot status
       
    4. SUCCESS VALIDATION (Exit Point)
       - Compares final action with originally requested action
       - Returns True: Action executed as requested
       - Returns False: Action was overridden (energy/time constraints)
    
    ROBOT STATUS TRANSITIONS:
    
    Initial Status → Energy Check → Time Check → Final Status
    
    ANY → TIRED:     Insufficient energy forces idle state
    ANY → ACTING:    Sufficient energy + valid action execution
    
    ENERGY MANAGEMENT:
    - Energy requirements calculated by get_required_energy_to_action()
    - Different actions have different energy costs:
      * move: Base cost + load penalty
      * remove_vegetation: Fixed harvesting cost
      * sleep: No cost (recovery action)
      * idle: Minimal/no cost
    
    OVERRIDE BEHAVIORS:
    
    1. Energy Override (Highest Priority)
       - Any action requiring more energy than available → 'idle'
       - Robot status → 'TIRED'
       - Return False (action not executed as requested)
    
    2. Circadian Override (Medium Priority)  
       - Any action during night → 'sleep'
       - Robot status → 'ACTING'
       - Return depends on whether sleep was originally requested
    
    3. Invalid Time Override (Error Condition)
       - Raises ValueError for invalid time_of_day values
       - Ensures robust error handling
    
    RETURN VALUE SEMANTICS:
    - True: Action executed successfully as requested
    - False: Action was overridden due to constraints
    
    This return value allows calling code to detect when actions
    couldn't be executed as planned and adjust behavior accordingly.
    """
    
    # this is the action implementation
    def do_action(self, time_of_day=None, limits=None, misc=None) -> bool:

        requested_action = self._current_action
        
        # required energy
        required_energy = self.get_required_energy_to_action()
        
        if self._energy < required_energy:
            self._current_action = 'idle'
            self._status_robot = 'TIRED'
            return False
        
        if time_of_day is 'night':
            self._current_action = 'sleep'
            self._status_robot = 'ACTING'
            self.sleep(time_of_day, limits, misc)
        elif time_of_day is 'day':
            if self._current_action == 'move':
                self._status_robot = 'ACTING'
                self.move(time_of_day, limits, misc)
            elif self._current_action == 'remove_vegetation':
                self._status_robot = 'ACTING'
                self.remove_vegetation(time_of_day, limits, misc)
        else:
            raise ValueError('Invalid time_of_day: {}'.format(time_of_day))
            pass
        
        if requested_action == self._current_action:
            return True
        else:
            return False
        
    # this is the energy update policy
    def update_energy(self) -> None:
        if self._current_action == 'move':
            if self._status_motion == 'ACTIVE':
                self._energy -= (self._motion_consumption + self._load_consumption*self._load)
            else:
                pass
        elif self._current_action == 'remove_vegetation':
            self._energy -= self._harvest_consumption
        elif self._current_action == 'sleep':
            self._energy += self._sleep_recovery
        else:
            pass  
        
        self._energy = self.np.clip(self._energy, 0, 100)
        
    ############################################################
    # TASKS
    ############################################################
    
    # TASK: harvest
    def harvest(self, time_of_day=None, limits=None, misc=None) -> None:
        # decide the action
        # nothing to do here, the action is already decided
        
        # do the action
        self._current_action = 'remove_vegetation'
        success = self.do_action(time_of_day, limits, misc)
        
        # close the task
        if success:
            self._status_task = 'FINISHED'                    

    def explore(self, time_of_day=None, limits=None, misc=None) -> None:
        """
        Implement exploration task behavior using configurable strategies.
        
        This method executes the exploration task by managing a neighborhood
        of target locations and systematically visiting them. The exploration
        strategy determines how these targets are selected and prioritized.
        
        The exploration operates in three motion states:
        1. IDLE: Select next unvisited target destination
        2. ACTIVE: Continue moving toward current destination  
        3. FINISHED: Mark current target as visited, prepare for next
        
        Exploration completes when all neighborhood targets have been visited,
        transitioning the task status to FINISHED.
        
        Parameters
        ----------
        time_of_day : str, optional
            Current time period, passed to action execution
        limits : list, optional
            Environment boundaries for destination clipping
            Format: [[x_min, x_max], [y_min, y_max]]
            
        Notes
        -----
        Neighborhood Management:
        The method maintains a neighborhood list with corresponding flags
        indicating which targets have been reached. Target selection uses
        the first unvisited location (FIFO order).
        
        Motion State Handling:
        
        - **IDLE State**: 
          * Finds next unvisited target using index search
          * Sets destination with boundary clipping
          * Initiates movement action
          * None index indicates all targets visited
          
        - **ACTIVE State**:
          * Continues current movement action
          * Maintains INPROGRESS task status
          
        - **FINISHED State**:
          * Marks current target as visited (reached_flag = True)
          * Transitions task to FINISHED status
          * Prepares for next exploration cycle
        
        Destination Clipping:
        Target coordinates are clipped to environment boundaries to ensure
        valid navigation goals and prevent out-of-bounds movement attempts.
        
        Action Management:
        Sets _current_action appropriately:
        - 'move': When valid destination exists
        - 'idle': When no unvisited targets remain
        
        Task Status Updates:
        - INPROGRESS: While targets remain and robot is active
        - FINISHED: When motion completes or no targets remain
        
        The exploration integrates with the broader behavioral system
        through the do_action() method, which handles energy management
        and low-level motion control.
        
        Examples
        --------
        Typical exploration cycle:
        1. get_neighbourhood() generates target list
        2. explore() systematically visits each target
        3. Motion controller handles pathfinding
        4. Targets marked visited upon arrival
        5. Task completes when all targets visited
        """
        # decide the action
        # if you're not moving, find a new destination
        if self._status_motion is 'IDLE':
            try:
                self._neighbourhood_current_index = self._neighbourhood_reached_flag.index(False)
            except ValueError:
                #! this means that all cells have been explored
                self._neighbourhood_current_index = None 
                    
            if self._neighbourhood_current_index is not None:
                self._motion_destination = [self._neighbourhood[self._neighbourhood_current_index][0], 
                                            self._neighbourhood[self._neighbourhood_current_index][1]] #! set the destination
                if limits is not None: #! clip the destination
                    self._motion_destination[0] = self.np.clip(self._motion_destination[0], limits[0][0], limits[0][1])
                    self._motion_destination[1] = self.np.clip(self._motion_destination[1], limits[1][0], limits[1][1])
                    
                # set the action and update task status
                self._current_action = 'move'
                self._status_task = 'INPROGRESS'           
            else:
                self._motion_destination = None   
                
                # set the action and update task status  
                self._current_action = 'idle'           
                self._status_task = 'FINISHED'
        elif self._status_motion is 'ACTIVE':
            # set the action and update task status  
            self._current_action = 'move'           
            self._status_task = 'INPROGRESS'
        elif self._status_motion is 'FINISHED':
            self._neighbourhood_reached_flag[self._neighbourhood_current_index] = True #! mark the cell as explored
            self._controller._error_integral = 0.0
            self._controller._error_old = 0.0
            
            # set the action and update task status  
            self._current_action = 'move'           
            self._status_task = 'FINISHED'
        else:
            raise ValueError('Invalid status_motion: {}'.format(self._status_motion))
            
        # do the action
        success = self.do_action(time_of_day, limits, misc)
        
    def store(self, time_of_day=None, limits=None, misc=None) -> None:
        # decide the action
        # nothing to do here, the action is already decided                
        
        # do the action        
        self._current_action = 'move'
        self._controller._Kp = 0.5 * self._controller._Kp_init
        self._controller._Kd = 0.5 * self._controller._Kd_init
        self._controller._Ki = 0.5 * self._controller._Ki_init
        success = self.do_action(time_of_day, limits, misc)
        
        # close the task
        if success:
            # transfer the load to the home base
            if self._status_motion == 'FINISHED':                
                self.store_vegetation(time_of_day, limits)                                
                self._controller._Kp = self._controller._Kp_init
                self._controller._Kd = self._controller._Kd_init
                self._controller._Ki = self._controller._Ki_init
                self._status_task = 'FINISHED'
                self._current_action = 'idle'                
        
    ############################################################
    # ACTIONS
    ############################################################
    
    # action: sleep
    def sleep(self, time_of_day=None, limits=None, misc=None) -> None:
        pass                        
             
    # action: move
    def move(self, time_of_day=None, limits=None, misc=None) -> None:

        if self._controller._name == 'P':
            setpoint = self._motion_destination
            self._controller.step(setpoint, self._position)
        elif self._controller._name == 'P_repulsive':            
            setpoint = self._motion_destination
            _neighbourhood = module_misc.DN_neighbourhood(self._position, limits, N=self._controller._neighbourhood_size, step=self._measure_step)                        

            map_repulsive = self._local_map.copy()            
            map_repulsive = map_repulsive / self.np.max(map_repulsive)

            # explore farther from home base
            distance_from_home = self.np.linalg.norm(self.np.array(self._position) - self.np.array(self._home_base_position))
            
            if self._controller._map_repulsive is 'vegetation_quality':
                self._local_map_control = map_repulsive
            elif self._controller._map_repulsive is 'vegetation_visits':                
                self._local_map_control = 1 / (1 + (self._local_map_visits)/self.np.max(self._local_map_visits))            
            else:
                raise ValueError('Invalid map_repulsive value: {}'.format(self._controller._map_repulsive))                                                                    

            # control - get base neighborhood values
            _neighbourhood_values = [self._local_map_control[int(pos[0]), int(pos[1])] for pos in _neighbourhood]                        
            
            flow_direction = self.np.array(misc.get('direction', [1,0]))
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
                        _neighbourhood_values[i] += flow_modifier
            
            self._controller.step(setpoint, self._position, [_neighbourhood, _neighbourhood_values])
        else:
            raise ValueError('Invalid controller name: {}'.format(self._controller._name))

        self._dynamics.step(self._controller._output)
        self._position = list([int(coord) for coord in self._dynamics._output[0]])  
        
        # clip the position
        self._position[0] = self.np.clip(self._position[0], limits[0][0], limits[0][1])
        self._position[1] = self.np.clip(self._position[1], limits[1][0], limits[1][1])
                    
        # update status            
        self._status_motion = self._controller._status
        
    # action: remove_vegetation
    def remove_vegetation(self, time_of_day=None, limits=None, misc=None) -> None:
        # Check if removal would result in negative values (water creation)
        potential_new_value = self._map_quality_measure_position - self._vegetation_removal
        
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
                    # If max_allowed_removal is 0, do nothing (can't harvest)
                    return
        
        # Normal harvesting (either won't go negative, or river is nearby)
        self._harvesting_actions_counter += 1
        if self._map_quality_measure_position >= 1:
            self._map_quality_measure_position -= self._vegetation_removal
            self._load += self._vegetation_removal
        else:
            self._map_quality_measure_position -= 0.1 * self._vegetation_removal
            self._load += 0.1 * self._vegetation_removal

    # action: store_vegetation
    def store_vegetation(self, time_of_day=None, limits=None, misc=None) -> bool:
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
    # UTILS
    ############################################################
    
    # this is the energy policy
    def get_required_energy_to_action(self) -> float:
        if self._current_action == 'move':
            required_energy =  self._motion_consumption + self._load_consumption*self._load
        elif self._current_action == 'remove_vegetation':
            required_energy = self._harvest_consumption
        elif self._current_action == 'sleep':
            required_energy = 0
        else:
            required_energy = 0
            
        return required_energy
        
    def get_neighbourhood(self) -> None:
        """
        Generate exploration targets using the configured exploration strategy.
        
        This method computes a set of neighborhood positions for exploration
        based on the robot's current location, environmental knowledge, and
        selected exploration algorithm. The generated targets guide subsequent
        exploration behavior.
        
        The method supports multiple exploration strategies that balance
        different objectives like coverage, efficiency, and resource optimization:
        
        - **Deterministic (D[N])**: Systematic geometric patterns
        - **Random (random_D[N])**: Stochastic target selection  
        - **Gradient (gradient_D[N])**: Quality-driven exploration
        - **Softmax (softmax_D[N])**: Probabilistic quality-based selection
        
        Parameters
        ----------
        None
            Uses instance attributes for configuration:
            - _exploration_mode: Strategy and neighborhood size
            - _position: Current robot location
            - _local_map: Environmental knowledge
            - _exploration_eta: Learning rate for gradient methods
            - _exploration_N_recovery: Visit count recovery factor
            
        Sets
        ----
        _neighbourhood : list
            List of [x, y] coordinate pairs for exploration targets
        _neighbourhood_reached_flag : list
            Boolean flags indicating which targets have been visited
        _neighbourhood_current_index : int
            Index of current/next target for systematic progression
            
        Raises
        ------
        ValueError
            If exploration_mode contains invalid strategy name or neighborhood size
            Valid sizes: {0, 4, 8, 24, 40}
            Valid strategies: {'D', 'random_D', 'gradient_D', 'softmax_D'}
            
        Notes
        -----
        Exploration Strategy Details:
        
        **Deterministic (D[N])**:
        - Generates systematic geometric neighborhood patterns
        - Ensures complete coverage of local area
        - Predictable and reproducible exploration paths
        
        **Random Deterministic (random_D[N])**:
        - Randomizes order of deterministic neighborhood
        - Maintains coverage while adding unpredictability
        - Useful for avoiding systematic biases
        
        **Gradient-based (gradient_D[N])**:
        - Prioritizes targets based on vegetation quality gradients
        - Uses eta parameter to balance exploitation vs exploration
        - Considers harvest thresholds and recovery factors
        - Optimizes for resource-rich areas
        
        **Softmax (softmax_D[N])**:
        - Probabilistic selection based on quality distributions
        - Temperature parameter (eta) controls exploration/exploitation
        - Provides stochastic but quality-biased target selection
        
        Neighborhood Sizes:
        - N=0: Current position only
        - N=4: Von Neumann neighborhood (4-connected)
        - N=8: Moore neighborhood (8-connected)  
        - N=24: Extended local area
        - N=40: Large neighborhood for long-range planning
        
        Map Selection:
        The method uses different maps based on _exploration_map setting:
        - 'vegetation_quality': Raw quality values
        - 'vegetation_visits': Quality normalized by visit frequency
        
        Home Base Avoidance:
        All strategies consider home base positions to avoid unnecessary
        revisiting of storage locations during exploration phases.
        
        Examples
        --------
        >>> # Configure gradient-based exploration with 8-neighborhood
        >>> robot._exploration_mode = 'gradient_D8'
        >>> robot._exploration_eta = 0.1
        >>> robot.get_neighbourhood()
        >>> print(len(robot._neighbourhood))  # 8 targets
        
        >>> # Random exploration with large neighborhood  
        >>> robot._exploration_mode = 'random_D24'
        >>> robot.get_neighbourhood()
        >>> print(len(robot._neighbourhood))  # 24 targets
        """
        
        #! step to be used in the neighbourhood, it makes sense to have 
        #! it = 1 as long as the current cell is the only one observed by the beaver
                
        position = self._position

        #! BEHAVIORAL LOGIC MODEL 
        eps = 1e0       
        threshold_mask = (self._local_map >= self._harvest_threshold[0]) & (self._local_map <= self._harvest_threshold[1])        
        if self._exploration_map is 'vegetation_quality':            
            local_map = 1/(eps + self._local_map.copy())**2
            local_map[~threshold_mask] = 0.0
        elif self._exploration_map is 'vegetation_visits':
            local_map = (eps + self._local_map_visits.copy()) * (eps + self._local_map.copy())**2
            local_map[~threshold_mask] = 0.0
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
        
        
    def update_local_map(self,map_quality, misc=None) -> None:                

        # Ensure the local map is initialized
        if self._local_map is None:
            self._local_map = self.np.ones((1, 1)) * self.np.nan            
            
        # positions
        measure_positions = map_quality[0]
        measure_values = map_quality[1]

        # reset visits
        bias = 1
        delta_store = self.np.diff(self._load_store[-(self._n_traces+1):-1]) if len(self._load_store) > self._n_traces else 0
        delta_load = bias + self.np.sum(delta_store)
        increase = delta_load
            
        # if I see the whole map
        if map_quality[0] is 'all':            
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
        
        # remember how much I was in the water
        if self._local_map[self._position[0], self._position[1]] < 0:
            self._in_water_counter += 1
        else:
            self._in_water_counter = 0
        
    def set_home_base_position(self) -> None:
        # get distance to the home base
        distances_to_home = [
            self.np.sqrt(
                (self._position[0] - home_pos[0]) ** 2 +
                (self._position[1] - home_pos[1]) ** 2
            )
            for home_pos in self._home_base_position_store
        ]
        distance_to_home = min(distances_to_home)
        
        # if too far, find a new home base position        
        if distance_to_home > self._min_home_distance:    
            _limits = [[0, self._local_map.shape[0] - 1], [0, self._local_map.shape[1] - 1]]
            _local_neighbourhood = module_misc.DN_neighbourhood(self._position, _limits, N=80, step=self._measure_step)
            valid_positions = [
                pos for pos in _local_neighbourhood
                if (self._local_map[int(pos[0] - 1), int(pos[1] - 1)] <= 0.5 * self._vegetation_quality_range[1]) \
                    and (self._local_map[int(pos[0] - 1), int(pos[1] - 1)] >= 0.0)
            ]
            if valid_positions:        
                self._home_base_position_store.append(self.random.choice(valid_positions))
                
        # define the motion destination as the closest home base position
        self._motion_destination = min(
            self._home_base_position_store,
            key=lambda pos: self.np.sqrt(
                (self._position[0] - pos[0]) ** 2 +
                (self._position[1] - pos[1]) ** 2
            )
        )

    def select_exploration_eta(self, dt, dt_percentage, decay):
        
        decay_eta = decay[0]        
        harvest_decay = decay[1]
        load_decay = decay[2]
        
        load_derivative = self.np.diff(self._load_store[-(dt+1):-1])
        self._wait_b4_explore += 1
        if len(load_derivative) >= dt-1 and self._wait_b4_explore >= dt:
            self._wait_b4_explore = 0            
            if any(load_derivative < 0) :
                self._exploration_eta = self._exploration_eta_init
                self._harvest_threshold = self._harvest_threshold_init
                self._maximum_load = self._maximum_load_init
            elif (load_derivative > 0).sum() < dt_percentage * dt:
                self._exploration_eta = (1-decay_eta) * self._exploration_eta
                self._maximum_load = (1-load_decay) * self._maximum_load
                self._harvest_threshold = [(1-harvest_decay) * self._harvest_threshold[0], (1+harvest_decay) * self._harvest_threshold[1]]
            else:
                self._exploration_eta = (1+decay_eta) * self._exploration_eta
                self._maximum_load = (1+load_decay) * self._maximum_load
                self._harvest_threshold = [(1+harvest_decay) * self._harvest_threshold[0], (1-harvest_decay) * self._harvest_threshold[1]]

            self._exploration_eta = self.np.clip(self._exploration_eta, 1e-2, 10)
            self._maximum_load = self.np.clip(self._maximum_load, 0.0, self.np.inf)
            self._harvest_threshold[0] = self.np.clip(self._harvest_threshold[0], 0.0, self._vegetation_quality_range[1])
            self._harvest_threshold[1] = self.np.clip(self._harvest_threshold[1], 0.0, self._vegetation_quality_range[1])
            self._harvest_threshold = sorted(self._harvest_threshold)