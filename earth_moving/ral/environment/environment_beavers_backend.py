# general imports
from scipy.stats import norm

# backend imports
from ral.environment.environment_backend import BaseEnvironmentBackend

# module imports
import ral.environment.modules.module_misc as module_misc
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
        
        self._vegetation_cluster_sigma = self._environment.get('vegetation_cluster_sigma') #! This is an initial sigma, it will be scaled when growing the clusters
        self._vegetation_cluster_radius = self._environment.get('vegetation_cluster_radius')
        self._vegetation_cluster_radius_range = self._environment.get('vegetation_cluster_radius_range')
        self._vegetation_quality_range = self._environment.get('vegetation_quality_range')        
        self._vegetation_quality_init = self._environment.get('vegetation_quality_init')
        self._vegetation_growth_frequency = self._environment.get('vegetation_growth_frequency')        
        self._print = self._environment.get('print')
        
        # init class attributes        
        self._current_time = 0
        self._current_day = []
        self._current_hour = []
        self._time_of_day = []
        self.update_time_of_day()
        self._trail_usage_map = self.np.zeros((self._width, self._height))
        self._canal_usage_map = self.np.zeros((self._width, self._height))
        self._number_vegetation_clusters = 0
        self._vegetation_map = self.np.ones((self._width, self._height)) * self._vegetation_quality_init
        self._vegetation_clusters_store = []
        
        # call init methods
        self.generate_vegetation_map()        
        
        return self   
    
    def update_time_of_day(self) -> None:  
        self._current_hour = self._current_time % 24
        self._current_day = self._current_time // 24
        if self._current_hour > 6  and self._current_hour < 18:
            self._time_of_day = 'day' 
        else:  
            self._time_of_day = 'night'   
    
    def step_environment(self, dt) -> None: 
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the environment is not aware of the time flow
        self.update_time_of_day()
        if (self._current_time % self._vegetation_growth_frequency == 0) and (self._time_of_day == 'night'):
            self.grow_vegetation()
            
        # prints
        if self._print:
            print("Vegetation Clusters:")
            for cluster in self._vegetation_clusters_store:
                print(f"Cluster ID: {cluster[0]}, X: {cluster[1]}, Y: {cluster[2]}, Radius: {cluster[3]}")
            
    def generate_vegetation_map(self) -> None:
        for _ in range(self._number_vegetation_clusters_init):
            cx, cy = self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)            
            self.generate_cluster(cx, cy)
        
    def generate_cluster(self, cx, cy, cluster_radius=None) -> None:
        
        # generate the cluster radius and sigma
        if cluster_radius is None:
            if self._vegetation_cluster_radius is 'random':
                cluster_radius = self.random.randint(self._vegetation_cluster_radius_range[0], self._vegetation_cluster_radius_range[1])
            else:
                cluster_radius = self._vegetation_cluster_radius
        else:
            cluster_radius = self.np.clip(cluster_radius, self._vegetation_cluster_radius_range[0], self._vegetation_cluster_radius_range[1])
            
        # scale sigma
        sigma = module_misc.scale_sigma(self._vegetation_cluster_sigma, cluster_radius)      
        
        # Spread vegetation outward using a Gaussian-like distribution
        for dx in range(-cluster_radius, cluster_radius + 1):
            for dy in range(-cluster_radius, cluster_radius + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self._width and 0 <= ny < self._height:
                    distance = self.np.sqrt(dx**2 + dy**2)
                    #! remark: base_map is increased because vegetation can overlap when generated
                    self._vegetation_map[nx, ny] += self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                        * norm.pdf(distance, 0.0, sigma)
                    self._vegetation_map = self.np.clip(self._vegetation_map, self._vegetation_quality_range[0], self._vegetation_quality_range[1])
        
        # store clusters
        found = False
        for i, cluster in enumerate(self._vegetation_clusters_store):
            if cluster[1] == cx and cluster[2] == cy:
                self._vegetation_clusters_store[i] = (cluster[0], cluster[1], cluster[2], cluster_radius)
                found = True
                break
        if not found:
            self._vegetation_clusters_store.append((self._number_vegetation_clusters  + 1, cx, cy, cluster_radius))
        self._number_vegetation_clusters = len(self._vegetation_clusters_store)
        
    def grow_cluster(self, cx, cy, cluster_radius) -> None:
        
        # remove previous cluster
        sigma = module_misc.scale_sigma(self._vegetation_cluster_sigma, cluster_radius)
        for dx in range(-cluster_radius, cluster_radius + 1):
            for dy in range(-cluster_radius, cluster_radius + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self._width and 0 <= ny < self._height:
                    distance = self.np.sqrt(dx**2 + dy**2)
                    self._vegetation_map[nx, ny] -= self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                        * norm.pdf(distance, 0.0, sigma)
                    self._vegetation_map = self.np.clip(self._vegetation_map, self._vegetation_quality_range[0], self._vegetation_quality_range[1])
        
        # increase radius 
        cluster_radius += 1
        sigma = module_misc.scale_sigma(self._vegetation_cluster_sigma, cluster_radius)
        self.generate_cluster(cx, cy, cluster_radius)
        
    def grow_vegetation(self) -> None:
        
        # grow previous clusters
        for cluster in self._vegetation_clusters_store:
            cx = cluster[1]
            cy = cluster[2]
            cluster_radius = cluster[3]
            self.grow_cluster(cx, cy, cluster_radius)
            
        # generate new cluster   
        if self._number_vegetation_clusters < self._number_vegetation_clusters_max:
            cx, cy = self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)            
            self.generate_cluster(cx, cy, cluster_radius=self._vegetation_cluster_radius_range[0])            