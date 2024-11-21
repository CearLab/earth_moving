import numpy as np
from sensor.sensor_backend import BaseSensorBackend

class SensorRGBBackend(BaseSensorBackend):
    
    def __init(self, **kwargs) -> None:
        super().__init__(kwargs)
        self.imgW = self._kwargs.get('imgW')
        self.imgH = self._kwargs.get('imgH')
    
    def get_data(self):
        pass
    
    def transform(self, data) -> np.array:
        raise NotImplementedError()