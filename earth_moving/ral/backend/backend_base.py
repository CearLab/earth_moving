import numpy as np
from abc import ABC, abstractmethod
class BackendBase(ABC):
    def __init__(self,**kwargs) -> None:
        pass       
            
    def step(self):
        raise NotImplementedError()
    
    def spawn_object(self, object_pose, object_description):
        raise NotImplementedError()
