# general imports

# backend imports
from earth_moving.ral.robot.robot_backend import BaseRobotBackend

# module imports
from earth_moving.ral.robot.modules.module_control import Controller
from earth_moving.ral.robot.modules.module_control import Dynamics
import earth_moving.ral.algorithms.module_misc as module_misc
import earth_moving.ral.robot.modules.module_beaver as module_beaver
class BeaversRobotBackend(BaseRobotBackend):
    
    def __init__(self, **kwargs) -> None:
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
        
        super().initiate_robot(**kwargs)                
        
        # parse _robot
        self._maximum_load = self._robot.get('maximum_load') 
        if self._maximum_load is None:
            self._maximum_load = self.np.inf   
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
        self._harvest_threshold = self._robot.get('harvest_threshold')
        self._sleep_recovery = self._robot.get('sleep_recovery')
        self._vegetation_removal = self._robot.get('vegetation_removal')
        self._measurement_mode = self._robot.get('measurement_mode')
        self._home_base_position = self._robot.get('home_base_position')         
        
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
        self._harvesting_actions_limit = 4
        
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
        self._energy_store = []
        self._load_store = []
        self._task_store = []
        self._time_store = []
        self._home_base_position_store = [list(self._home_base_position)]
        
        return self
    
    #! remark: the vegetation quality and the time_of_day are OBSERVATIONS
    def step_beaver(self, dt, time_of_day, map_quality, limits) -> None:
        # update your internal clock
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the agent is not aware of the time flow        
        # set integration time
        self._controller._dt = dt                
        self._dynamics._dt = dt     
                        
        # gather information (OBSERVATIONS)
        #! map quality = [measure_positions, measure_values]
        #! measure_positions = [[x1,y1],[x2,y2],...,[xn,yn]]
        #! measure_values = [[v1,v2,...,vn], vposition]  
        self._map_quality_measure = map_quality #? Do I read the measurements also at night? It doesn't hurt
        self._map_quality_measure_position = self._map_quality_measure[1][-1] #! this is the quality at the current position (see module_misc in the visualizer)                
        
        # update the local_map according to the measurements
        self.update_local_map(self._map_quality_measure)
        self._map_quality_update = False #! reset the flag
                
        # decide the goal (TASK POLICY)
        self.decide_task(time_of_day, limits)        
        # do the task (TASK IMPLEMENTATION)
        self.do_task(time_of_day, limits)
        # update energy
        self.update_energy()                 
        
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
            self.update_local_map(self._map_quality_measure)
        
        # store the data
        self._destination_store.append(self._motion_destination)
        self._action_store.append(self._current_action)
        self._position_store.append(self._position)
        self._energy_store.append(self._energy)
        self._load_store.append(self._load)
        self._task_store.append(self._current_task)
        self._time_store.append(time_of_day)
        
        # prints
        if self._print:
            print('t= {}: Agent {} is doing {}'.format(self._current_time, self.unique_id, self._current_action))
            
    ############################################################
    # POLICIES and IMPLEMENTATIONS
    ############################################################
    
    # this is the task policy
    def decide_task(self, time_of_day, limits) -> None:                
                    
        # close the FSM loop
        if self._status_task is 'FINISHED':
            self._status_task = 'IDLE'            
                
        # task policy
        # if self._status_task is 'IDLE':
        
        # define interrupt or atomic schedule
        cond_atomic = self._status_task == 'IDLE'
        # cond_atomic = True
        
        # decide the task
        if  cond_atomic and \
            self._load >= self.np.floor(self._maximum_load):
            self._current_task = 'store'        
        # self._current_task == 'explore' and self._status_task == 'IDLE' and \
        elif cond_atomic and \
            self._map_quality_measure_position >= self._harvest_threshold[0] and \
            self._map_quality_measure_position <= self._harvest_threshold[1] and \
            self._load < self._maximum_load and \
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
    def do_task(self, time_of_day=None, limits=None) -> None:
        if self._current_task == 'explore':
            # first time you get the neighbors and set the status to INPROGRESS
            if self._status_task is 'STARTING':
                self.get_neighbourhood()
                self._status_task = 'INPROGRESS'
            self.explore(time_of_day, limits)
        elif self._current_task == 'harvest':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':                
                self._status_task = 'INPROGRESS'
            self.harvest(time_of_day, limits)
        elif self._current_task == 'store':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':   
                self.set_home_base_position()             
                self._status_task = 'INPROGRESS'
            self.store(time_of_day, limits)
        elif self._current_task == 'sleep':
            # first time you set the status to INPROGRESS
            if self._status_task is 'STARTING':
                self._status_task = 'INPROGRESS'
            self.sleep(time_of_day, limits)        
        else:
            raise ValueError('Invalid task: {}'.format(self._current_task))
        
    # this is the action implementation
    def do_action(self, time_of_day=None, limits=None) -> bool:
        
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
            self.sleep(time_of_day, limits)
        elif time_of_day is 'day':
            if self._current_action == 'move':
                self._status_robot = 'ACTING'
                self.move(time_of_day, limits)
            elif self._current_action == 'remove_vegetation':
                self._status_robot = 'ACTING'
                self.remove_vegetation(time_of_day, limits)
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
    def harvest(self, time_of_day=None, limits=None) -> None:
        # decide the action
        # nothing to do here, the action is already decided
        
        # do the action
        self._current_action = 'remove_vegetation'
        success = self.do_action(time_of_day, limits)
        
        # close the task
        if success:
            self._status_task = 'FINISHED'                    
        
    # TASK: explore
    def explore(self, time_of_day=None, limits=None) -> None:
        # decide the action
        # if you're not moving, find a new destination
        if self._status_motion is 'IDLE':
            try:
                self._neighbourhood_current_index = self._neighbourhood_reached_flag.index(False)
            except ValueError:
                self._neighbourhood_current_index = None #! this means that all cells have been explored                
                    
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
            
            # set the action and update task status  
            self._current_action = 'move'           
            self._status_task = 'FINISHED'
        else:
            raise ValueError('Invalid status_motion: {}'.format(self._status_motion))
            
        # do the action
        success = self.do_action(time_of_day, limits)
        
    def store(self, time_of_day=None, limits=None) -> None:
        # decide the action
        # nothing to do here, the action is already decided                
        
        # do the action        
        self._current_action = 'move'
        success = self.do_action(time_of_day, limits)
        
        # close the task
        if success:
            # transfer the load to the home base
            if self._status_motion == 'FINISHED':                
                self.store_vegetation(time_of_day, limits)                                
                self._status_task = 'FINISHED'
                self._current_action = 'idle'                
        
    ############################################################
    # ACTIONS
    ############################################################
    
    # action: sleep
    def sleep(self, time_of_day=None, limits=None) -> None:
        pass                        
             
    # action: move
    def move(self, time_of_day=None, limits=None) -> None:
                     
        if self._controller._name == 'P':
            setpoint = self._motion_destination
            self._controller.step(setpoint, self._position)
        elif self._controller._name == 'P_repulsive':            
            setpoint = self._motion_destination
            _neighbourhood = module_misc.DN_neighbourhood(self._position, limits, N=self._controller._neighbourhood_size)
            
            map_repulsive = self.np.zeros(self._local_map.shape)
            map_repulsive[self.np.isnan(map_repulsive)] = 0
            map_repulsive[self._local_map >= self._harvest_threshold[0]] = self._controller._vegetation_barrier[0]
            map_repulsive[self._local_map > self._harvest_threshold[1]] = self._controller._vegetation_barrier[1]
            map_repulsive[self._local_map < 0] = self._controller._river_barrier
            
            score = 1
            if self._controller._map_repulsive is 'vegetation_quality':
                self._local_map_control = map_repulsive + score * self._local_map
            elif self._controller._map_repulsive is 'vegetation_visits':                
                self._local_map_control = map_repulsive + score * self._local_map_visits
            else:
                raise ValueError('Invalid map_repulsive value: {}'.format(self._controller._map_repulsive))                                
                        
            # control
            _neighbourhood_values = [self._local_map_control[int(pos[0]), int(pos[1])] for pos in _neighbourhood]
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
    def remove_vegetation(self, time_of_day=None, limits=None) -> None:
        if self._load <= self._maximum_load - self._vegetation_removal and \
            self._harvesting_actions_counter < self._harvesting_actions_limit:
                self._harvesting_actions_counter += 1
                self._map_quality_measure_position  -= self._vegetation_removal
                self._load += self._vegetation_removal            
            
    # action: store_vegetation
    def store_vegetation(self, time_of_day=None, limits=None) -> bool:
        if self._load > 0:
            available_space = self.np.inf * self._vegetation_quality_range[1] - self.np.ceil(self._map_quality_measure_position)
            
            if available_space > 0:
                removed_load = min(available_space, self._load)
                self._map_quality_measure_position += removed_load
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
        
    # get the neighbourhood
    def get_neighbourhood(self) -> None:
        
        #! step to be used in the neighbourhood, it makes sense to have 
        #! it = 1 as long as the current cell is the only one observed by the beaver    
        
        position = self._position   
        
        # exploration_map
        if self._exploration_map is 'vegetation_quality':
            local_map = self._local_map   
        elif self._exploration_map is 'vegetation_visits':     
            local_map = self._local_map / (1 + self._local_map_visits)
        else:
            raise ValueError('Invalid exploration_map value: {}'.format(self._exploration_map))
        
        if self._local_map is not None:
            limits = [[0, self._local_map.shape[0] - 1], [0, self._local_map.shape[1] - 1]]
        else:
            limits = [[self._position[0], self._position[0]], [self._position[1], self._position[1]]]                    
            
        # Split exploration_mode into two parts: prefix and suffix
        if len(self._exploration_mode) > 2:
            exploration_prefix = self._exploration_mode[:-2]
            exploration_suffix = int(self._exploration_mode[-2:])
        else:
            exploration_prefix = self._exploration_mode
            exploration_suffix = None
            
        # Check if the suffix is a valid number
        if exploration_suffix not in [0, 4, 8, 24, 40]:
            raise ValueError('Invalid exploration mode: {}'.format(self._exploration_mode))                
            
        if exploration_prefix == 'D':
            N, NF, NI = module_beaver.exploration_DN(position, limits, N=exploration_suffix, home_base_store=self._home_base_position_store)
        elif exploration_prefix == 'random_D':
            N, NF, NI = module_beaver.exploration_random_DN(position, limits, N=exploration_suffix, home_base_store=self._home_base_position_store)
        elif exploration_prefix == 'gradient_D':
            N, NF, NI = module_beaver.exploration_gradient_DN(position, limits, local_map, N=exploration_suffix, \
                home_base_store=self._home_base_position_store, max_vegetation=self._harvest_threshold, eta=self._exploration_eta, \
                N_recovery=self._exploration_N_recovery)
        elif exploration_prefix == 'softmax_D':            
            N, NF, NI = module_beaver.exploration_softmax_DN(position, limits, local_map, N=exploration_suffix, \
                home_base_store=self._home_base_position_store, max_vegetation=self._harvest_threshold, eta=self._exploration_eta,
                N_recovery=self._exploration_N_recovery)
        else:
            raise ValueError('Invalid exploration mode: {}'.format(self._exploration_mode))            
        
        self._neighbourhood = N
        self._neighbourhood_reached_flag = NF
        self._neighbourhood_current_index = NI
        
        
    def update_local_map(self,map_quality) -> None:
        
        # Ensure the local map is initialized
        if self._local_map is None:
            self._local_map = self.np.ones((1, 1)) * self.np.nan            
            if self._local_map_visits is None:
                self._local_map_visits = self.np.zeros((1, 1))
            
        # positions
        measure_positions = map_quality[0]
        measure_values = map_quality[1]
        
        # reset visits
        N_reset = self._controller._visits_reset
            
        # if I see the whole map
        if map_quality[0] is 'all':
            self._local_map = map_quality[1][0]                              
                      
            if self._local_map_visits.shape != self._local_map.shape:
                self._local_map_visits = self.np.zeros(self._local_map.shape)
            self._local_map_visits[self._position[0], self._position[1]] += 1
            self._local_map_visits = self._local_map_visits * N_reset
            return                    

        # Expand the matrix if the position is out of bounds
        x = self.np.max([pos[0] for pos in measure_positions])
        y = self.np.max([pos[1] for pos in measure_positions])
        if x >= self._local_map.shape[0]:
            # local map
            self._local_map = self.np.pad(self._local_map, ((0, x - self._local_map.shape[0] + 1), (0, 0)), 
                                                mode='constant', constant_values=self.np.nan)
            
            # local map visits
            self._local_map_visits = self.np.pad(self._local_map_visits, ((0, x - self._local_map_visits.shape[0] + 1), (0, 0)), 
                                                mode='constant', constant_values=0)                        
            
        if y >= self._local_map.shape[1]:
            # local map
            self._local_map = self.np.pad(self._local_map, ((0, 0), (0, y - self._local_map.shape[1] + 1)), 
                                                mode='constant', constant_values=self.np.nan)
            
            # local map visits
            self._local_map_visits = self.np.pad(self._local_map_visits, ((0, 0), (0, y - self._local_map_visits.shape[1] + 1)), 
                                                mode='constant', constant_values=0)                        

        # Update the vegetation quality at the current position        
        for pos, val in zip(measure_positions, measure_values):
            self._local_map[pos[0], pos[1]] = val            
        self._local_map_visits[self._position[0], self._position[1]] += 1     
        self._local_map_visits = self._local_map_visits * N_reset
        
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
        if distance_to_home > 30:    
            _limits = [[0, self._local_map.shape[0] - 1], [0, self._local_map.shape[1] - 1]]
            _local_neighbourhood = module_misc.DN_neighbourhood(self._position, _limits, N=40) 
            valid_positions = [
                pos for pos in _local_neighbourhood
                if self._local_map[int(pos[0] - 1), int(pos[1] - 1)] <= self._vegetation_quality_range[0]
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