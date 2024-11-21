import numpy as np
from abc import ABC, abstractmethod
from sensor.sensor_backend import BaseSensorBackend
class BaseBackend(ABC):
    def __init__(self) -> None:
        pass
    
    def step(self):
        raise NotImplementedError()

    def initiate_rgb_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
