import numpy as np
from abc import ABC, abstractmethod


class BaseSensorBackend(ABC): 
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        # ? We don't set a default value and let errors propagate, right?
        self._data = self._kwargs.get('data')
        self._pose = self._kwargs.get('pose')
        self._name = self._kwargs.get('name')
        self._frame_body = self._name + '_link' # TODO: we should name the frames with this in later URDFs/publishers
        self._frame_read = self._kwargs.get('frame_read') # ? In general we might want the data in a specific reference frame?
        
    @abstractmethod # ! Is this correct? I want Pybullet/ROS to handle the actual data fetching
    def get_data(self) -> np.array: # TODO: what if numpy isn't installed?
        pass
    
    @abstractmethod
    def transform(self,data) -> np.array:
        pass
    
    def read_sensor(self) -> np.array:
        tmp = self.get_data()
        if self._frame_body != self._frame_read:
            self._data = self.transform(tmp)
        else:
            self._data = tmp        
    
    def update_pose(self):
        raise NotImplementedError()