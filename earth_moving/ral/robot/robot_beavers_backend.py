import numpy as np

from ral.robot.robot_backend import BaseRobotBackend

from ral.robot.modules.robot_beavers_module import Controller
from ral.robot.modules.robot_beavers_module import Dynamics
class BeaversRobotBackend(BaseRobotBackend):
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)              
        
    def initiate_robot(self, **kwargs):
        
        super().initiate_robot(**kwargs)                
        
        # parse _robot
        self._maximum_load = self._robot.get('maximum_load')     
        self._print = self._robot.get('print')
        
        # custom attributes        
        self._range_x = self._robot.get('range_x')
        self._range_y = self._robot.get('range_y')        
        self._motion_consumption = self._robot.get('motion_consumption')
        self._load_consumption = self._robot.get('load_consumption')
        self._sleep_recovery = self._robot.get('sleep_recovery')
        self._exploration_mode = self._robot.get('exploration_mode')
        
        # other attributes
        self._current_time = 0 #! this is a counter. Agent does not compute the hour/time of the day. It will be provided by the environment        
        self._vegetation_quality = 0 #! could have set to None but don't want to deal with it in printing
        self._required_energy = 0
        self._motion_destination = None
        self._neighbourhood = None        
        self._neighbourhood_reached_flag = None
        self._neighbourhood_current_index = None
        self._local_map_vegetation = None
        
        # physical attributes
        # position
        position = self._robot.get('position')
        if position is 'random':
            self._position = [np.random.randint(self._range_x[0], self._range_x[1]), 
                              np.random.randint(self._range_y[0], self._range_y[1])]
        elif position is None:
            self._position = [0,0]
        elif isinstance(position,list):
            self._position = position
            self._position[0] = np.clip(self._position[0], self._range_x[0], self._range_x[1])
            self._position[1] = np.clip(self._position[1], self._range_y[0], self._range_y[1])
        else:
            raise ValueError('Invalid position value: {}'.format(position))
        
        # energy
        initial_energy = self._robot.get('initial_energy')
        if initial_energy is 'random':
            self._energy = np.random.randint(0,100)
        elif initial_energy is None:
            self._energy = 100
        elif isinstance(initial_energy,int):
            self._energy = np.clip(initial_energy, 0, 100)
        else:
            raise ValueError('Invalid initial_energy value: {}'.format(initial_energy))
        
        # load
        initial_load = self._robot.get('initial_load')
        if initial_load is 'random':
            self._load = np.random.randint(0,self._maximum_load)
        elif initial_load is None:
            self._load = 0
        elif isinstance(initial_load,int):
            self._load = np.clip(initial_load, 0, self._maximum_load)
        else:
            raise ValueError('Invalid initial_load value: {}'.format(initial_load))                                
        
        # motion policy                        
        self._controller = Controller(**self._robot)
        initial_state = np.array([self._position, np.zeros(self._controller._dimension)])
        self._dynamics = Dynamics(initial_state, **self._robot)                     
            
        # state machine
        # idle: the agent is not doing anything             -> constant energy, constant load
        # sleeping: the agent is sleeping                   -> energy recovery, constant load
        # exploring: the agent is exploring the environment -> energy consumption, constant load
        
        # returning: the agent is returning to the lodge    -> energy consumption, constant load                
        # building: the agent is building something         -> energy consumption, load consumption
        # harvesting: the agent is harvesting vegetation    -> energy consumption, load recovery
        self._status_robot = 'IDLE'      
        self._status_task = 'IDLE'  
        self._status_motion = 'IDLE'
        self._current_task = None
        self._current_action = None
        
        return self
    
    #! remark: the vegetation quality and the time_of_day are OBSERVATIONS
    def step_beaver(self, dt, time_of_day, vegetation_quality, limits) -> None:
        # update your internal clock
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the agent is not aware of the time flow        
        # set integration time
        self._controller._dt = dt                
        self._dynamics._dt = dt                     
        # gather information (OBSERVATIONS)
        #! time_of_day is provided, no need to store it
        self.update_vegetation_quality(vegetation_quality, time_of_day)        
        
        #TODO store carrying load            
        #TODO get total vegetation
        #TODO store total vegetation
        #TODO store total trail lenght
        #TODO store total canal lenght
        #TODO store total vegetation of high quality
        #TODO store total energy
        
        #TODO if not already done, initialize a trail
        
        # Exploring
        # - look around and set the quality of the vegetation. It's a local map, each agent has one        
        
        # decide the goal (TASK POLICY)
        self.decide_task(time_of_day, limits)        
        # do the task (TASK IMPLEMENTATION)
        self.do_task(time_of_day, limits)        
        # decide what to do (ACTION POLICY)
        self.decide_action(time_of_day, limits)                                  
        # do the action (ACTION IMPLEMENTATION)
        self.do_action(time_of_day, limits)        
        # update energy
        self.update_energy()                                        
        
        # prints
        if self._print:
            print('t= {}: Agent {} is doing {}'.format(self._current_time, self.unique_id, self._current_action))
            
    # this is the task policy
    def decide_task(self, time_of_day, limits) -> None:
        if self._status_robot is not 'IDLE':
            self._status_task = 'INPROGRESS'
            return
        
        # choice policy
        self._current_task = 'explore' 
        self._status_task = 'IDLE'       
        
    def do_task(self, time_of_day=None, limits=None) -> None:
        if self._current_task == 'explore':
            self.explore(limits)
        else:
            raise ValueError('Invalid task: {}'.format(self._current_task))
        
    # this is the policy
    def decide_action(self, time_of_day=None, limits=None) -> None:                
        
        if self._status_task is not 'INPROGRESS':
            self._current_action = 'idle'
            self._status_robot = 'IDLE'
            return
        
        # choice policy
        if time_of_day == 'day':            
            self._current_action = 'move'
            self._status_robot = 'MOVING'            
        elif time_of_day == 'night':
            self._current_action = 'sleep'
            self._status_robot = 'SLEEP'
        else:
            self._current_action = 'idle'
            self._status_robot = 'IDLE'
            raise ValueError('Invalid time_of_day: {}'.format(time_of_day))            
        
        # check if I have enough energy to do the action
        self.get_required_energy_to_action()
        if self._energy < self._required_energy:
            self._current_action = 'idle'
            self._status_robot = 'TIRED'                    
        
    # this is the energy policy
    def get_required_energy_to_action(self) -> None:
        if self._current_action == 'move':
            self._required_energy =  self._motion_consumption + self._load_consumption*self._load
        elif self._current_action == 'sleep':
            self._required_energy = 0
        else:
            self._required_energy = 0
            
    # this is the action implementation   
    def do_action(self, time_of_day=None, limits=None) -> None:
        if self._current_action == 'move':
            self.move(limits)
        elif self._current_action == 'sleep':
            self.sleep()
        elif self._current_action == 'idle':
            pass
        else:
            raise ValueError('Invalid action: {}'.format(self._current_action))
            
    # this is the energy update policy
    def update_energy(self) -> None:
        if self._current_action == 'move':
            if self._status_motion == 'ACTIVE':
                self._energy -= (self._motion_consumption + self._load_consumption*self._load)
            else:
                pass
        elif self._current_action == 'sleep':
            self._energy += self._sleep_recovery
        else:
            pass  
        
        self._energy = np.clip(self._energy, 0, 100)                    
        
    # action: sleep
    def sleep(self) -> None:
        pass                        
             
    # action: move
    def move(self, limits) -> None:                
                     
        self._controller.step(self._motion_destination, self._position)
        self._dynamics.step(self._controller._output)
        self._position = list([int(coord) for coord in self._dynamics._output[0]])  
        
        # clip the position
        self._position[0] = np.clip(self._position[0], limits[0][0], limits[0][1])
        self._position[1] = np.clip(self._position[1], limits[1][0], limits[1][1])
                    
        # update status            
        self._status_motion = self._controller._status
        
    # TASK: explore
    def explore(self, limits=None) -> None:
        
        # it's the first time you enter the explore state
        if self._status_task is not 'INPROGRESS':
            # define the neighborhood
            self.get_neighbourhood()        
        
        # if you're not moving, find a new destination
        if self._status_motion is 'IDLE':
            # find first unexplored cell
            try:
                self._neighbourhood_current_index = self._neighbourhood_reached_flag.index(False) #! explore the next first unexplored cell
            except ValueError:
                self._neighbourhood_current_index = None #! this means that all cells have been explored                
                    
            if self._neighbourhood_current_index is not None:
                self._motion_destination = [self._neighbourhood[self._neighbourhood_current_index][0], 
                                            self._neighbourhood[self._neighbourhood_current_index][1]]
                if limits is not None:
                    self._motion_destination[0] = np.clip(self._motion_destination[0], limits[0][0], limits[0][1])
                    self._motion_destination[1] = np.clip(self._motion_destination[1], limits[1][0], limits[1][1])
                self._status_task = 'INPROGRESS'
            else:
                self._motion_destination = None                
                self._status_task = 'IDLE'
        elif self._status_motion is 'FINISHED':
            self._neighbourhood_reached_flag[self._neighbourhood_current_index] = True
        else:
            pass
        
    # get the neighbourhood
    def get_neighbourhood(self) -> None:
        
        step = 3
        
        D4_neighbourhood = [[self._position[0] + step, self._position[1]],
                                [self._position[0] - step, self._position[1]],
                                [self._position[0], self._position[1] + step],
                                [self._position[0], self._position[1] - step]]
        
        D8_neighbourhood = [[self._position[0] + step, self._position[1]],
                                [self._position[0] - step, self._position[1]],
                                [self._position[0], self._position[1] + step],
                                [self._position[0], self._position[1] - step],
                                [self._position[0] + step, self._position[1] + step],
                                [self._position[0] + step, self._position[1] - step],
                                [self._position[0] - step, self._position[1] + step],
                                [self._position[0] - step, self._position[1] - step]]
        
        # This is the D4 exploration
        if self._exploration_mode == 'D4':            
            self._neighbourhood = D4_neighbourhood
            self._neighbourhood_reached_flag = [False, False, False, False]
            self._neighbourhood_current_index = 0
            
        # This is the D8 exploration
        elif self._exploration_mode == 'D8':            
            self._neighbourhood = D8_neighbourhood
            self._neighbourhood_reached_flag = [False, False, False, False, False, False, False, False]
            self._neighbourhood_current_index = 0
        
        # This is the random exploration
        # Generate a single random position in the D4 neighborhood
        elif self._exploration_mode == 'random_D4':            
            direction = np.random.randint(0, len(D4_neighbourhood))
            self._neighbourhood = [D4_neighbourhood[direction]]            
            self._neighbourhood_reached_flag = [False]
            self._neighbourhood_current_index = 0
            
        # Generate a single random position in the D8 neighborhood
        elif self._exploration_mode == 'random_D8':            
            direction = np.random.randint(0, len(D8_neighbourhood))
            self._neighbourhood = [D8_neighbourhood[direction]]            
            self._neighbourhood_reached_flag = [False]
            self._neighbourhood_current_index = 0
        
        else:
            raise ValueError('Invalid exploration mode: {}'.format(self._exploration_mode))
        
        
    def update_vegetation_quality(self,vegetation_quality,time_of_day) -> None:
        
        if time_of_day == 'day':
            # read the vegetation quality
            self.set_vegetation_quality(vegetation_quality) #! this is stored 
            
            # Ensure the local map is initialized
            if self._local_map_vegetation is None:
                self._local_map_vegetation = np.ones((1, 1)) * np.nan

            # Expand the matrix if the position is out of bounds
            x, y = self._position
            if x >= self._local_map_vegetation.shape[0]:
                self._local_map_vegetation = np.pad(self._local_map_vegetation, ((0, x - self._local_map_vegetation.shape[0] + 1), (0, 0)), 
                                                    mode='constant', constant_values=np.nan)
            if y >= self._local_map_vegetation.shape[1]:
                self._local_map_vegetation = np.pad(self._local_map_vegetation, ((0, 0), (0, y - self._local_map_vegetation.shape[1] + 1)), 
                                                    mode='constant', constant_values=np.nan)

            # Update the vegetation quality at the current position
            self._local_map_vegetation[x, y] = vegetation_quality
    
    # this is the observation
    def set_vegetation_quality(self,quality=None) -> None:
        if quality is None:
            quality = self._vegetation_quality
        self._vegetation_quality = quality        
        
                    