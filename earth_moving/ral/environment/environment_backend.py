class BaseEnvironmentBackend(): 
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_environment(self, **kwargs):
        self._kwargs = kwargs
        self._environment = self._kwargs.get('environment')
        self._seed = self._kwargs.get('seed')
        self._environment_name = self._environment.get('name')        
        return self                            
        