import numpy as np
from ral.robot.robot_backend import BaseRobotBackend

import random
class BeaversRobotBackend(BaseRobotBackend):
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)      
        # set seed
        random.seed(self._seed)  
        
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
        self._vegetation_quality = self._robot.get('vegetation_quality')        
        
        # custom attributes
        self._current_action = None
        
        return self
    
    def step_beaver(self) -> None:
        self._current_action = self.decide_action()        
        if self._current_action == 'move':
            self.move()
        elif self._current_action == 'act':
            self.set_vegetation_quality()
        else:
            raise ValueError('Invalid action: {}'.format(self._current_action))
        
    def decide_action(self) -> str:
        return np.random.choice(['move','act'],p=[1.0,0.0])
    
    def set_vegetation_quality(self,quality=None) -> None:
        if quality is None:
            quality = self._vegetation_quality
        self._vegetation_quality = quality            
        
    def move(self) -> None:
        x, y = self._position
        dx, dy = np.random.randint(-1,2), np.random.randint(-1,2)
        x += dx
        y += dy
        self._position = [x,y]    