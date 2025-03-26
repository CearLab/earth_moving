import numpy as np
from abc import ABC, abstractmethod

class BaseEnvironmentBackend(ABC): 
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_environment(self, **kwargs):
        self._kwargs = kwargs
        self._environment = self._kwargs.get('environment')
        self._environment_name = self._environment.get('name')        
        return self                            
        