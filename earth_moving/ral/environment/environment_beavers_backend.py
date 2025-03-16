import numpy as np
from scipy.stats import norm
import random

from ral.environment.environment_backend import BaseEnvironmentBackend
class BeaversEnvironmentBackend(BaseEnvironmentBackend): 
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)        
        
    def initiate_environment(self, **kwargs):
        
        super().initiate_environment(**kwargs)                
        
        # parse the config file        
        self._width = self._environment.get('width')
        self._height = self._environment.get('height')
        self._number_vegetation_clusters_init = self._environment.get('number_vegetation_clusters_init')
        self._number_vegetation_clusters_max = self._environment.get('number_vegetation_clusters_max')
        
        if self._number_vegetation_clusters_init > self._number_vegetation_clusters_max:
            raise ValueError("Initial number of vegetation clusters cannot be greater than the maximum number of vegetation clusters.")
        
        self._vegetation_cluster_sigma = self._environment.get('vegetation_cluster_sigma')
        self._vegetation_cluster_radius = self._environment.get('vegetation_cluster_radius')
        self._minimum_vegetation = self._environment.get('minimum_vegetation')
        self._maximum_vegetation = self._environment.get('maximum_vegetation')
        self._vegetation_growth_rate = self._environment.get('vegetation_growth_rate')
        self._vegetation_growth_frequency = self._environment.get('vegetation_growth_frequency')        
        
        # init class attributes        
        self._current_time = 0.0
        self._time_of_day = 'night'
        self._trail_usage_map = np.zeros((self._height, self._width))
        self._canal_usage_map = np.zeros((self._height, self._width))
        self._number_vegetation_clusters = 0
        self._vegetation_map = np.ones((self._height, self._width)) * self._minimum_vegetation
        
        # call init methods
        self.generate_vegetation_map()        
        
        return self        
    
    def step_environment(self) -> None:
        self._current_time += 1
        if self._current_time % 12 == 0:
            self._time_of_day = 'day' if self._time_of_day == 'night' else 'night'
        if (self._current_time % self._vegetation_growth_frequency == 0):
            self.grow_vegetation()
            
    def generate_vegetation_map(self) -> None:        

        for _ in range(self._number_vegetation_clusters_init):
            cx, cy = random.randint(0, self._width - 1), random.randint(0, self._height - 1)            
            self.generate_cluster(cx, cy)
        
    def generate_cluster(self, cx, cy):
        # generate the cluster radius and sigma
        cluster_radius = random.randint(1, self._vegetation_cluster_radius)
        sigma = (cluster_radius + self._vegetation_cluster_sigma - 1)/self._vegetation_cluster_sigma #! remark: scaling the sigma with the cluster radius        
        
        # Spread vegetation outward using a Gaussian-like distribution
        for dx in range(-cluster_radius, cluster_radius + 1):
            for dy in range(-cluster_radius, cluster_radius + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self._width and 0 <= ny < self._height:
                    distance = np.sqrt(dx**2 + dy**2)
                    #! remark: base_map is increased because vegetation can overlap when generated
                    self._vegetation_map[ny, nx] += self._maximum_vegetation * np.sqrt(2 * np.pi * sigma**2) \
                                        * norm.pdf(distance, 0.0, sigma)
                    self._vegetation_map = np.clip(self._vegetation_map, self._minimum_vegetation, self._maximum_vegetation)
        
        self._number_vegetation_clusters += 1
        
    def grow_vegetation(self) -> None:        
        if self._number_vegetation_clusters < self._number_vegetation_clusters_max:
            cx, cy = random.randint(0, self._width - 1), random.randint(0, self._height - 1)            
            self.generate_cluster(cx, cy)
            Warning("New vegetation cluster generated at ({},{})".format(cx,cy))