import numpy as np
from abc import ABC, abstractmethod

class BaseEnvironmentBackend(ABC): 
    
    def __init__(self, **kwargs) -> None:
        pass
    
    def initiate_environment(self, **kwargs) -> None:
        self._kwargs = kwargs
        self._environment = self._kwargs.get('environment')
        self._environment_name = self._environment.get('name')
        return self
    
    def generate_aggregates(self, **kwargs) -> None:
        
        aggregates = self._environment.get('aggregates')        
        min_pos = np.asarray(aggregates.get('min_pos'))
        max_pos = np.asarray(aggregates.get('max_pos'))
        num_clusters = int(aggregates.get('num_clusters'))
        max_per_cluster = int(aggregates.get('max_per_cluster'))
        max_radius = float(aggregates.get('max_radius'))
        
        self._aggregate_urdf = aggregates.get('aggregate_urdf')
        
        import ral.algorithms.general_actions as general_actions
        general_actions = general_actions.GeneralActions()
        self._aggregates_positions = general_actions.generate_aggregates_in_clusters(min_pos, max_pos, num_clusters, max_per_cluster, max_radius)                            
        