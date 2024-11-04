import numpy as np


class BaseSensorBackend: # TODO: move to a separate file?
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        self._pose = ...
    
    def get(self) -> np.array: # TODO: what if numpy isn't installed?
        raise NotImplementedError()
    
    def update_pose(self):
        raise NotImplementedError()


class BaseBackend:
    def __init__(self) -> None:
        pass
    
    def step(self):
        raise NotImplementedError()

    def initiate_rgb_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
