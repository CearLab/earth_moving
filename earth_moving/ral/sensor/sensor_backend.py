import numpy as np

class BaseSensorBackend(): 
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_sensor(self, **kwargs) -> None:
        self._kwargs = kwargs        
        self._sensor = self._kwargs.get('sensor')        
        self._name = self._sensor.get('name')        
    
    def get_data(self) -> np.array: # TODO: what if numpy isn't installed?
        raise NotImplementedError()