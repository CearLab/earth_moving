class BaseEnvironmentBackend(): 
    
    # imports
    import numpy as np
    import random
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_environment(self, **kwargs) -> None:
        self._kwargs = kwargs
        self._environment = self._kwargs.get('environment')        
        self._environment_name = self._environment.get('name')
                                  
        