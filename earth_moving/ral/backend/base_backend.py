from abc import ABC, abstractmethod
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
