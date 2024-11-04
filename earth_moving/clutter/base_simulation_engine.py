from enum import Enum

class ControlMode(Enum):
    VELOCITY_CONTROL = 'velocity_control'
    POSITION_CONTROL = 'position_control'
    

class BaseSimulationEngine:
    def __init__(self): # TODO: inputs = ?
        pass
    
    def simulate(self, time, step):
        raise NotImplementedError()
    
    # ...
    
    def control_joint(self, joint_target, control_mode=ControlMode.VELOCITY_CONTROL):
        raise NotImplementedError()


