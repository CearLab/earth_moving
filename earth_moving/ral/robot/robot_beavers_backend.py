# general imports

# backend imports
from ral.robot.robot_backend import BaseRobotBackend

# module imports
from ral.robot.modules.module_control import Controller
from ral.robot.modules.module_control import Dynamics
import ral.robot.modules.module_misc as module_misc
import ral.robot.modules.module_beaver as module_beaver
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

        # ACTION RECAP
        # Idle: do nothing                          -> constant energy, constant load
        # Sleep: recover energy                     -> increase energy, constant load
        # Move: move to a destination               -> consume energy, constant load
        # Remove_vegetation: harvest vegetation     -> consume energy, increase load
        
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
        self._motion_consumption = self._robot.get('motion_consumption')
        self._load_consumption = self._robot.get('load_consumption')
        self._harvest_consumption = self._robot.get('harvest_consumption')
        self._harvest_threshold = self._robot.get('harvest_threshold')
        self._sleep_recovery = self._robot.get('sleep_recovery')
        self._vegetation_removal = self._robot.get('vegetation_removal')
        self._measurement_mode = self._robot.get('measurement_mode')
        
        # other attributes
        self._current_time = 0 #! this is a counter. Agent does not compute the hour/time of the day. It will be provided by the environment         
        self._map_quality = None
        self._map_quality_measure = None
        self._map_quality_measure_position = None
        self._map_quality_update = False #! this is a flag to update the vegetation quality in the environment
        self._motion_destination = None
        self._neighbourhood = None        
        self._neighbourhood_reached_flag = None
        self._neighbourhood_current_index = None
        self._local_map = None
        
        # physical attributes
        # position
        position = self._robot.get('position')
        if position is 'random':
            self._position = [self.random.randint(self._range_x[0], self._range_x[1]), 
                              self.random.randint(self._range_y[0], self._range_y[1])]
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
        
        return self
    
    #! remark: the vegetation quality and the time_of_day are OBSERVATIONS
    def step_beaver(self, dt, time_of_day, map_quality, limits) -> None:
        # update your internal clock
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the agent is not aware of the time flow        
        # set integration time
        self._controller._dt = dt                
        self._dynamics._dt = dt     
                        
        # gather information (OBSERVATIONS)
        #! time_of_day is provided, no need to store it
        self._map_quality_measure = map_quality #? Do I read the measurements also at night? It doesn't hurt
        self._map_quality_measure_position = self._map_quality_measure[1][0] #! this is the quality at the current position (see module_misc in the visualizer)
        self._map_quality_update = False #! reset the flag
        
        # decide the goal (TASK POLICY)
        self.decide_task(time_of_day, limits)        
        # do the task (TASK IMPLEMENTATION)
        self.do_task(time_of_day, limits)
        # update energy
        self.update_energy()                        
        
        #! this is where we change/actuate the environment
        if self._current_task == 'harvest' and self._status_task == 'FINISHED':               
            self._map_quality_update = True
        else:                        
            self._map_quality_update = False
            
        # update the local_map according to the action 
        self._map_quality = self._map_quality_measure_position
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
        if self._status_task is 'IDLE':
            if self._map_quality_measure_position > self._harvest_threshold and self._load < self._maximum_load:
                self._current_task = 'harvest'
            else:
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
                self._neighbourhood_current_index = self._neighbourhood_reached_flag.index(False) #! explore the next first unexplored cell
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
            self._status_task = 'INPROGRESS'        
        else:
            raise ValueError('Invalid status_motion: {}'.format(self._status_motion))
            
        # do the action
        success = self.do_action(time_of_day, limits)
        
    ############################################################
    # ACTIONS
    ############################################################
    
    # action: sleep
    def sleep(self, time_of_day=None, limits=None) -> None:
        pass                        
             
    # action: move
    def move(self, time_of_day=None, limits=None) -> None:
                     
        self._controller.step(self._motion_destination, self._position)
        self._dynamics.step(self._controller._output)
        self._position = list([int(coord) for coord in self._dynamics._output[0]])  
        
        # clip the position
        self._position[0] = self.np.clip(self._position[0], limits[0][0], limits[0][1])
        self._position[1] = self.np.clip(self._position[1], limits[1][0], limits[1][1])
                    
        # update status            
        self._status_motion = self._controller._status
        
    # action: remove_vegetation
    def remove_vegetation(self, time_of_day=None, limits=None) -> None:
        if self._load < self._maximum_load - self._vegetation_removal:
            self._map_quality_measure_position  -= self._vegetation_removal
            self._load += self._vegetation_removal
        
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
        local_map = self._local_map
        position_store = self._position_store
        if self._local_map is not None:
            limits = [[0,0], [self._local_map.shape[0] - 1, self._local_map.shape[1] - 1]]
        else:
            limits = [[self._position[0], self._position[1]], [self._position[0], self._position[1]]]                        
        
        # This is the D4 exploration
        if self._exploration_mode == 'D4':            
           N , NF, NI = module_beaver.exploration_D4(position, limits)
            
        # This is the D8 exploration
        elif self._exploration_mode == 'D8':            
            N , NF, NI = module_beaver.exploration_D8(position, limits)
                
        # Generate a single random position in the D4 neighborhood
        elif self._exploration_mode == 'random_D4':
            N , NF, NI = module_beaver.exploration_D4_random(position, limits)
            
        # Generate a single random position in the D8 neighborhood
        elif self._exploration_mode == 'random_D8':  
            N , NF, NI = module_beaver.exploration_D8_random(position, limits)
               
        elif self._exploration_mode == 'gradient_D4':
            N , NF, NI = module_beaver.exploration_gradient_D4(position, limits, local_map, position_store)
            
        elif self._exploration_mode == 'gradient_D8':
            N , NF, NI = module_beaver.exploration_gradient_D8(position, limits, local_map, position_store)
        
        else:
            raise ValueError('Invalid exploration mode: {}'.format(self._exploration_mode))
        
        self._neighbourhood = N
        self._neighbourhood_reached_flag = NF
        self._neighbourhood_current_index = NI
        
        
    def update_local_map(self,map_quality) -> None:
        
        # Ensure the local map is initialized
        if self._local_map is None:
            self._local_map = self.np.ones((1, 1)) * self.np.nan
            
        # positions
        measure_positions = map_quality[0]
        measure_values = map_quality[1]

        # Expand the matrix if the position is out of bounds
        x = self.np.max([pos[0] for pos in measure_positions])
        y = self.np.max([pos[1] for pos in measure_positions])
        if x >= self._local_map.shape[0]:
            self._local_map = self.np.pad(self._local_map, ((0, x - self._local_map.shape[0] + 1), (0, 0)), 
                                                mode='constant', constant_values=self.np.nan)
        if y >= self._local_map.shape[1]:
            self._local_map = self.np.pad(self._local_map, ((0, 0), (0, y - self._local_map.shape[1] + 1)), 
                                                mode='constant', constant_values=self.np.nan)

        # Update the vegetation quality at the current position
        for pos, val in zip(measure_positions, measure_values):
            self._local_map[pos[0], pos[1]] = val