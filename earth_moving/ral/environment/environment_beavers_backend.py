import numpy as np
from abc import ABC, abstractmethod
from ral.environment.environment_backend import BaseEnvironmentBackend

class BeaversEnvironmentBackend(BaseEnvironmentBackend): 
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        
    def initiate_environment(self, **kwargs):
        
        super().initiate_environment(**kwargs)
        
        # init class attributes
        self._trails = []
        self._canals = []
        self._river = []
        self._vegetation = []
        self._number_agents = [] #? should the environment be aware of the number of agents? Or should it be something the backend is aware of?
        self._trail_heatmap = {}
        self._time_of_day = 'night'
        self._current_time = 0
        
        # parse the config file
        self._width = self._environment.get('width')
        self._height = self._environment.get('height')
        self._sinuosity = self._environment.get('sinuosity')
        self._number_dams = self._environment.get('number_dams')
        self._number_vegetation_clusters = self._environment.get('number_vegetation_clusters')
        
        return self