import random
class BaseBackend():
    def __init__(self,**kwargs) -> None:
        pass
        
    def initiate_backend(self,**kwargs) -> None:
        self._kwargs = kwargs        
        _simulation = self._kwargs.get('simulation')
        
        # set seed
        self._seed = _simulation.get('seed')
        random.seed(self._seed)
        
        # set backend
        self._backend_type = self._kwargs.get('backend_type')
        if self._backend_type == 'pybullet':
            from ral.backend.pybullet_backend import PybulletBackend
            return PybulletBackend(simulation=_simulation)
        elif self._backend_type == 'beavers_visualizer':
            from ral.backend.beavers_visualizer_backend import BeaversVisualizerBackend            
            return BeaversVisualizerBackend(simulation=_simulation)
            
    def step(self):
        raise NotImplementedError()
    
    def load_aggregates(self,aggregate_positions, aggregate_urdf):
        raise NotImplementedError()
