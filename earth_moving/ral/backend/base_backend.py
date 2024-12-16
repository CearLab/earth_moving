from abc import ABC, abstractmethod
from ral.sensor.sensor_backend import BaseSensorBackend
class BaseBackend(ABC):
    def __init__(self,**kwargs) -> None:
        pass
        
    def initiate_backend(self,**kwargs) -> None:
        _simulation = kwargs.get('simulation')
        self._backend_type = kwargs.get('backend_type')
        if self._backend_type == 'pybullet':
            from ral.backend.pybullet_backend import PybulletBackend
            return PybulletBackend(simulation=_simulation)
            
    def step(self):
        raise NotImplementedError()

    def initiate_rgb_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
        
    def initiate_imu_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
    
    def initiate_gyro_sensor(self) -> BaseSensorBackend:
        raise NotImplementedError()
