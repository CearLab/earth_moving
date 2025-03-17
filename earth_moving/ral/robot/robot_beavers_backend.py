import numpy as np
from ral.robot.robot_backend import BaseRobotBackend
class BeaversRobotBackend(BaseRobotBackend):
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)              
        
    def initiate_robot(self, **kwargs):
        
        super().initiate_robot(**kwargs)                
        
        # parse _robot
        position = self._robot.get('position')
        if position is 'random':
            range_x = self._robot.get('range_x')
            range_y = self._robot.get('range_y')
            self._position = [np.random.randint(range_x[0], range_x[1]), np.random.randint(range_y[0], range_y[1])]
        elif position is None:
            self._position = [0,0]
        elif isinstance(position,list):
            self._position = position
        else:
            raise ValueError('Invalid position value: {}'.format(position))
        self._vegetation_quality_range = self._robot.get('vegetation_quality_range')        
        self._print = self._robot.get('print')
        
        # custom attributes
        self._current_time = 0 #! this is a counter. Agent does not compute the hour/time of the day. It will be provided by the environment
        self._current_action = None
        self._vegetation_quality = self._vegetation_quality_range[0] #TODO could have set to None but don't want to deal with it in printing
        
        return self
    
    #! remark: the vegetation quality and the time_of_day are OBSERVATIONS
    def step_beaver(self, dt, time_of_day, vegetation_quality) -> None:
        # update your internal clock
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the agent is not aware of the time flow
        
        # gather information (OBSERVATIONS)
        #! time_of_day is provided, no need to store it
        self.set_vegetation_quality(vegetation_quality) #! this is stored 
        
        #TODO store carrying load
        #TODO get total vegetation
        #TODO store total vegetation
        #TODO store total trail lenght
        #TODO store total canal lenght
        #TODO store total vegetation of high quality
        #TODO store total energy
        
        # decide what to do (ACTION POLICY)
        self._current_action = self.decide_action(time_of_day)        
        
        # do the action (ACTION IMPLEMENTATION)
        self.do_action()
        
        # prints
        if self._print:
            print('t= {}: Agent {} is doing {}'.format(self._current_time, self.unique_id, self._current_action))
        
    # this is the policy
    def decide_action(self, time_of_day) -> str:
        if time_of_day == 'day':
            return 'move'
        elif time_of_day == 'night':
            return 'sleep'
    
    # this is the action implementation   
    def do_action(self) -> None:
        if self._current_action == 'move':
            self.move()
        elif self._current_action == 'sleep':
            self.sleep()
        else:
            raise ValueError('Invalid action: {}'.format(self._current_action))
    
    # this is the observation
    def set_vegetation_quality(self,quality=None) -> None:
        if quality is None:
            quality = self._vegetation_quality
        self._vegetation_quality = quality    
        
    # action: move
    def move(self) -> None:
        x, y = self._position
        dx, dy = np.random.randint(-1,2), np.random.randint(-1,2)
        x += dx
        y += dy
        self._position = [x,y]
        
    # action: sleep
    def sleep(self) -> None:
        pass