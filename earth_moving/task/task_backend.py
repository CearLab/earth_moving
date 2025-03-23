class BaseTaskBackend(): 
    
    # imports
    import numpy as np
    import random
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_task(self, **kwargs) -> None:
        self._kwargs = kwargs        
        self._task = self._kwargs.get('task')
        self._name = self._task.get('name')        