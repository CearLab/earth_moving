import numpy as np
from sensor.sensor_backend import BaseSensorBackend
class BaseBackend:
    def __init__(self) -> None:
        pass
    
    def step(self):
        raise NotImplementedError()

    def initiate_rgb_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
