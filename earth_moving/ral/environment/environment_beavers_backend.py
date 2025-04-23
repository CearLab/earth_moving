# general imports
from scipy.stats import norm

# backend imports
from earth_moving.ral.environment.environment_backend import BaseEnvironmentBackend

# module imports
import earth_moving.ral.algorithms.module_misc as module_misc
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
        self._vegetation_cluster_radius_range = self._environment.get('vegetation_cluster_radius_range')
        self._vegetation_quality_range = self._environment.get('vegetation_quality_range')        
        self._vegetation_quality_init_range = self._environment.get('vegetation_quality_init_range')
        self._vegetation_growth_frequency = self._environment.get('vegetation_growth_frequency')        
        if self._vegetation_growth_frequency == 'inf':
            self._vegetation_growth_frequency = self.np.inf
        self._print = self._environment.get('print')
        
        # streams
        self._streams_number = self._environment.get('streams_number')
        self._streams_width = self._environment.get('streams_width')        
        
        # init class attributes        
        self._current_time = 0
        self._current_day = []
        self._current_hour = []
        self._time_of_day = []
        self.update_time_of_day()
        self._trail_usage_map = self.np.zeros((self._width, self._height))
        self._canal_usage_map = self.np.zeros((self._width, self._height))
        self._number_vegetation_clusters = 0
        self._map = self.np.ones((self._width, self._height)) * self.np.random.randint(
            self._vegetation_quality_init_range[0], self._vegetation_quality_init_range[1], size=(self._width, self._height)
        )
        self._vegetation_clusters_store = []
        
        # call init methods
        self.generate_map()        
        
        return self   
    
    def update_time_of_day(self) -> None:  
        self._current_hour = self._current_time % 24
        self._current_day = self._current_time // 24
        if self._current_hour > 6  and self._current_hour < 18 or True:
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
                
    def generate_map(self) -> None:        
        self.generate_streams()
        self.generate_vegetation()        
            
    def generate_vegetation(self) -> None:
        for _ in range(self._number_vegetation_clusters_init):
            cx, cy = self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)            
            self.generate_cluster(cx, cy)
        
    def generate_cluster(self, cx, cy, cluster_radius=None) -> None:
        
        # generate the cluster radius and sigma
        if cluster_radius is None:
            cluster_radius = self.random.randint(self._vegetation_cluster_radius_range[0], self._vegetation_cluster_radius_range[1])            
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
                    if self._map[nx, ny] >= 0:
                        self._map[nx, ny] += self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                            * norm.pdf(distance, 0.0, sigma)
                        self._map = self.np.clip(self._map, -self._streams_width, self._vegetation_quality_range[1])
        
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
                    if self._map[nx, ny] > 0:                        
                        self._map[nx, ny] -= self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                            * norm.pdf(distance, 0.0, sigma)
                        self._map[nx, ny] = self.np.clip(self._map[nx, ny], 0.05, self._vegetation_quality_range[1])
        
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
            
    def generate_streams(self) -> None:
        for _ in range(self._streams_number):
            perimeter = 2*self._width + 2*self._height
            
            # start from the bottom
            position_start = self.random.randint(0, self._width -1)
            start = module_misc.get_coordinates_from_perimeter(self._width-1, self._height-1, position_start)
            
            # middle
            middle_1 = [self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)]
            middle_2 = [self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)]            
            
            # end in another side
            position_end = self.random.randint(self._width, perimeter)
            end = module_misc.get_coordinates_from_perimeter(self._width-1, self._height-1, position_end)
            
            # generate stream
            points = [start, middle_1, end]
            self.generate_stream(points)

    def generate_stream(self, points) -> None:
        sigma = 3
        path = module_misc.generate_path_from_points(points, self._width-1, self._height-1)        
        for position in path:
            self._map[position[0], position[1]] = -(self._streams_width + 1)
            
        extended_path = []
        for position in path:
            x, y = position
            for width in range(2, self._streams_width + 1):
                limits = self.np.array([[0, self._width - 1], [0, self._height - 1]])                
                neighbours = module_misc.DN_neighbourhood(position, limits, N=4, step=width-1)
                for neighbour in neighbours:                    
                    if not any((neighbour == self.np.array(points)).all() for points in path) and \
                       not any((neighbour == self.np.array(points)).all() for points in extended_path) and \
                       module_misc.is_within_limits(neighbour, self._width-1, self._height-1):  
                            
                            extended_path.append(neighbour)                         
                            distance = self.np.sqrt((neighbour[0] - x)**2 + (neighbour[1] - y)**2)
                            self._map[neighbour[0], neighbour[1]] = -self._streams_width * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                        * norm.pdf(distance, 0.0, sigma)
                            self._map[neighbour[0], neighbour[1]] = self.np.clip(self._map[neighbour[0], neighbour[1]], -self._streams_width, -0.05)    