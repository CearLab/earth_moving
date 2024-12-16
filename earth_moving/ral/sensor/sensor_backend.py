import numpy as np
from abc import ABC, abstractmethod


class BaseSensorBackend: 
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        self._name = self._kwargs.get('name')
        self._sensor = self._kwargs.get('sensor')
    
    @abstractmethod # ! Is this correct? I want Pybullet/ROS to handle the actual data fetching
    def get_data(self) -> np.array: # TODO: what if numpy isn't installed?
        pass