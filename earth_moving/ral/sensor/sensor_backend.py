import numpy as np
from abc import ABC, abstractmethod


class BaseSensorBackend(ABC): 
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        # ? We don't set a default value and let errors propagate, right?
        self._data = self._kwargs.get('data')
        self._pose = self._kwargs.get('pose')
        self._name = self._kwargs.get('name')
        self._frame = self._name + '_link' # TODO: we should name the frames with this in later URDFs/publishers
        self._type = self._kwargs.get('type') # ? Here I would use ABSOLUTE/RELATIVE, see self.read_sensor
        
    @abstractmethod # ! Is this correct? I want Pybullet/ROS to handle the actual data fetching
    def get_data(self) -> np.array: # TODO: what if numpy isn't installed?
        pass
    
    @abstractmethod
    def transform(self,data) -> np.array:
        pass
    
    def read_sensor(self) -> np.array:
        tmp = self.get_data()
        if self._type == 'RELATIVE':
            self._data = self.transform(tmp)
        elif self._type == 'ABSOLUTE':
            self._data = tmp
        else:
            pass # ! is there a more elegant way for this? Maybe a boolean instead of a string sensor type?
    
    def update_pose(self):
        raise NotImplementedError()