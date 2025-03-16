import numpy as np
from scipy.stats import norm
from ral.environment.environment_backend import BaseEnvironmentBackend

import random
class BeaversEnvironmentBackend(BaseEnvironmentBackend): 
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        # set seed
        random.seed(self._seed)
        
    def initiate_environment(self, **kwargs):
        
        super().initiate_environment(**kwargs)                
        
        # parse the config file        
        self._width = self._environment.get('width')
        self._height = self._environment.get('height')
        self._number_vegetation_clusters = self._environment.get('number_vegetation_clusters')
        self._vegetation_cluster_sigma = self._environment.get('vegetation_cluster_sigma')
        self._vegetation_cluster_radius = self._environment.get('vegetation_cluster_radius')
        self._minimum_vegetation = self._environment.get('minimum_vegetation')
        self._maximum_vegetation = self._environment.get('maximum_vegetation')
        self._vegetation_growth_rate = self._environment.get('vegetation_growth_rate')
        self._vegetation_growth_frequency = self._environment.get('vegetation_growth_frequency')
        self._cluster_growth_frequency = self._environment.get('clusters_growth_frequency')   
        
        # init class attributes        
        self._current_time = 0.0
        self._time_of_day = 'night'   
        self._vegetation_map = self.generate_vegetation_map()
        self._trail_usage_map = np.zeros((self._height, self._width))
        self._canal_usage_map = np.zeros((self._height, self._width))
        
        return self        
    
    def step_environment(self) -> None:
        self._time_of_day = 'day' if self._time_of_day == 'night' else 'night'
        if self._current_time % self._vegetation_growth_frequency == 0:
            self.grow_vegetation()
            
    def generate_vegetation_map(self) -> None:

        base_map = np.full((self._height, self._width), self._minimum_vegetation)

        for _ in range(self._number_vegetation_clusters):
            cx, cy = random.randint(0, self._width - 1), random.randint(0, self._height - 1)
            base_map[cy, cx] = self._maximum_vegetation  # Assign highest quality vegetation to cluster center

            # Spread vegetation outward using a Gaussian-like distribution
            for dx in range(-self._vegetation_cluster_radius, self._vegetation_cluster_radius + 1):
                for dy in range(-self._vegetation_cluster_radius, self._vegetation_cluster_radius + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self._width and 0 <= ny < self._height:
                        distance = np.sqrt(dx**2 + dy**2)
                        #! remark: base_map is increased because vegetation can overlap when generated
                        base_map[ny, nx] += self._maximum_vegetation * np.sqrt(2 * np.pi * self._vegetation_cluster_sigma**2) \
                                           * norm.pdf(distance, 0.0, self._vegetation_cluster_sigma)

        self._vegetation_map = np.clip(base_map, self._minimum_vegetation, self._maximum_vegetation)
        
    def grow_vegetation(self) -> None:
        # Vegetation grows back to maximum quality over time
        growth_indices = self._vegetation_map > self._minimum_vegetation
        self._vegetation_map[growth_indices] += self._vegetation_growth_rate
        self._vegetation_map = np.minimum(self._vegetation_map, self._maximum_vegetation)