class BaseRobotBackend(): 
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_robot(self, **kwargs) -> None:
        self._kwargs = kwargs        
        self._robot = self._kwargs.get('robot')        
        self._name = self._robot.get('name')    
        