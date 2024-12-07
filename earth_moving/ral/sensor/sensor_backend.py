import numpy as np
from abc import ABC, abstractmethod


class BaseSensorBackend: 
    
    def __init__(self, **kwargs) -> None:
        super().__init__(kwargs)
        self._kwargs = kwargs
        # ? We don't set a default value and let errors propagate, right?
        self._name = self._kwargs.get('name')
    
    @abstractmethod # ! Is this correct? I want Pybullet/ROS to handle the actual data fetching
    def get_data(self) -> np.array: # TODO: what if numpy isn't installed?
        pass