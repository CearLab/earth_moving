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
        
        # map generation mode
        self._map_mode = self._environment.get('map_mode', 'generate')  # 'generate' or 'csv'                
        
        if self._map_mode == 'csv':
            # csv file path and center
            self._elevation_file_path = self._environment.get('elevation_file_path', None)
            self._latitude_file_path = self._environment.get('latitude_file_path', None)
            self._longitude_file_path = self._environment.get('longitude_file_path', None)
            self._csv_center = self._environment.get('csv_center', None)  # [x, y] coordinates for center of selection
            
            # initial map
            self._map_original = self.np.ones((self._width, self._height))
        elif self._map_mode == 'generate':            
            # vegetation clusters
            self._number_vegetation_clusters_init = self._environment.get('number_vegetation_clusters_init')
            self._number_vegetation_clusters_max = self._environment.get('number_vegetation_clusters_max')            
            if self._number_vegetation_clusters_init > self._number_vegetation_clusters_max:
                raise ValueError("Initial number of vegetation clusters cannot be greater than the maximum number of vegetation clusters.")            
            self._vegetation_cluster_sigma = self._environment.get('vegetation_cluster_sigma') #! This is an initial sigma, it will be scaled when growing the clusters        
            self._vegetation_cluster_radius_range = self._environment.get('vegetation_cluster_radius_range')                        
                        
            # vegetation growth frequency
            self._vegetation_growth_frequency = self._environment.get('vegetation_growth_frequency')        
            if self._vegetation_growth_frequency == 'inf':
                self._vegetation_growth_frequency = self.np.inf            
                
            # streams
            self._streams_number = self._environment.get('streams_number')            
            
            # init map
            self._map_original = self.np.ones((self._width, self._height)) * self.np.random.randint(
                self._vegetation_quality_init_range[0], self._vegetation_quality_init_range[1], size=(self._width, self._height)
            )
        else:
            raise ValueError(f"Invalid map mode: {self._map_mode}. Must be 'generate' or 'csv'.")
        
        # general environment parameters        
        self._vegetation_quality_init_range = self._environment.get('vegetation_quality_init_range')
        self._vegetation_quality_range = self._environment.get('vegetation_quality_range')        
        self._streams_width = self._environment.get('streams_width')
        self._print = self._environment.get('print')                
        
        # init class attributes        
        self._current_time = 0
        self._current_day = []
        self._current_hour = []
        self._time_of_day = []
        self.update_time_of_day()        
        self._number_vegetation_clusters = 0        
        self._map = self._map_original.copy()
        self._map_visits = self._map_original.copy()            
        self._vegetation_clusters_store = []
        
        # home base position store
        self._home_base_position_store = None
        
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
    
    def step_environment(self, dt, map_visits, home_base_position_store, grass_growth_interval) -> None: 
        self._current_time += dt #! I wamt to pass the timedelta from the simulation, the environment is not aware of the time flow
        self.update_time_of_day()
        
        # update the map
        self._map = self._map_original.copy()
        self._map_visits = self._map_original.copy()
        self._home_base_position_store = home_base_position_store
        
        # scale the map based on the visits
        fade_thresh = 0
        fade_speed = 1        
        self._map_visits[self._map_original > fade_thresh] = self._map_original[self._map_original > fade_thresh] / \
            (1 + fade_speed * map_visits[self._map_original > fade_thresh]) #! This is a simple scaling, it can be improved                    
            
            
        # grow grass        
        rate = 0.00107 # 2.6% growth per day, 0.00107 per hour (3 weeks to grow grass)
        rate = rate / 1
        self._map_original[(self._map_original >= grass_growth_interval[0]) & \
            (self._map_original <= grass_growth_interval[1])] = \
            self._map_original[(self._map_original >= grass_growth_interval[0]) & \
            (self._map_original <= grass_growth_interval[1])] * (1 + rate)
                    
        # grow vegetation
        if self._map_mode == 'generate':
            if (self._current_time % self._vegetation_growth_frequency == 0):
                self.grow_vegetation()                        
            
        # prints
        if self._print:
            print("Vegetation Clusters:")
            for cluster in self._vegetation_clusters_store:
                print(f"Cluster ID: {cluster[0]}, X: {cluster[1]}, Y: {cluster[2]}, Radius: {cluster[3]}")
                
    def generate_map(self) -> None:
        if self._map_mode == 'csv':
            self.load_map_from_npy()
        else:
            self.generate_streams(n_points=12)
            self.generate_vegetation()
    
    def load_map_from_npy(self) -> None:
        """Load map from NPY file and extract a portion based on center and size."""
        if self._elevation_file_path is None:
            raise ValueError("NPY file path must be provided when map_mode is 'csv'")
        if self._csv_center is None:
            raise ValueError("NPY center coordinates must be provided when map_mode is 'csv'")
        
        try:
            # Load numpy binary files
            full_map = self.np.load(self._elevation_file_path)
            
            # Load coordinate arrays if provided
            if self._longitude_file_path is not None:
                full_longitude_coords = self.np.load(self._longitude_file_path)
            else:
                # Create default longitude coordinates
                full_longitude_coords = self.np.arange(full_map.shape[1])
                
            if self._latitude_file_path is not None:
                full_latitude_coords = self.np.load(self._latitude_file_path)
            else:
                # Create default latitude coordinates
                full_latitude_coords = self.np.arange(full_map.shape[0])
                
        except Exception as e:
            raise ValueError(f"Error loading NPY files: {str(e)}")
        
        # Print x/y coordinate stats
        print("\n--- Loaded NPY coordinate info ---")
        print(f"X axis: {len(full_longitude_coords)} points, min={full_longitude_coords.min():.3f}, max={full_longitude_coords.max():.3f}")
        print(f"Y axis: {len(full_latitude_coords)} points, min={full_latitude_coords.min():.3f}, max={full_latitude_coords.max():.3f}")
        print("----------------------------------\n")
        # Get the dimensions of the full map
        full_height, full_width = full_map.shape
        
        # Extract center coordinates
        center_x, center_y = self._csv_center
        
        # Validate center coordinates
        if (
            center_x < 0 or center_x >= full_width or
            center_y < 0 or center_y >= full_height            
        ):
            raise ValueError(f"Center coordinates ({center_x}, {center_y}) are outside the NPY bounds "
                           f"({full_width}, {full_height})")
            
        if (
            center_x < self._width // 2 or
            center_x >= full_width - self._width // 2 or
            center_y < self._height // 2 or
            center_y >= full_height - self._height // 2
        ):
            raise ValueError(f"Center coordinates ({center_x}, {center_y}) are outside the selection bounds "
                           f"({self._width}, {self._height})")

        # Calculate the selection bounds
        half_width = self._width // 2
        half_height = self._height // 2

        # Calculate start and end indices
        start_x = center_x - half_width
        end_x = start_x + self._width
        start_y = center_y - half_height
        end_y = start_y + self._height

        # Ensure indices are within bounds
        start_x = max(0, start_x)
        end_x = min(full_width, end_x)
        start_y = max(0, start_y)
        end_y = min(full_height, end_y)
        
        # Extract the portion - Note: using [y, x] indexing to match np.ones(width, height)
        selected_portion = full_map[start_y:end_y, start_x:end_x]
        
        # Extract corresponding coordinate portions
        selected_longitude = full_longitude_coords[start_x:end_x]
        selected_latitude = full_latitude_coords[start_y:end_y]
        
        # Initialize map with same structure as np.ones(width, height)
        self._map_original = self.np.ones((self._width, self._height))
        
        # Get actual dimensions of selected portion
        portion_height, portion_width = selected_portion.shape
        
        # Calculate where to place the selected portion in the map
        # Center it if the selected portion is smaller than desired size
        start_row = (self._width - portion_height) // 2 if portion_height < self._width else 0
        start_col = (self._height - portion_width) // 2 if portion_width < self._height else 0
        
        # Ensure we don't exceed map boundaries
        end_row = min(start_row + portion_height, self._width)
        end_col = min(start_col + portion_width, self._height)
        
        # Adjust portion size if needed
        actual_height = end_row - start_row
        actual_width = end_col - start_col
        
        # Handle NaN values - replace with minimum vegetation quality
        portion_to_place = selected_portion[:actual_height, :actual_width].copy()
        nan_mask = self.np.isnan(portion_to_place)
        portion_to_place[nan_mask] = self._vegetation_quality_range[0]
        
        # Get min and max of the selected portion for rescaling (excluding NaN)
        valid_values = portion_to_place[~self.np.isnan(selected_portion[:actual_height, :actual_width])]
        if len(valid_values) > 0:
            min_val = self.np.min(valid_values)
            max_val = self.np.max(valid_values)
            
            # Rescale to vegetation quality range
            if max_val > min_val:  # Avoid division by zero
                portion_to_place = (portion_to_place - min_val) / (max_val - min_val) * \
                                 (self._vegetation_quality_range[1] - self._vegetation_quality_range[0]) + \
                                 self._vegetation_quality_range[0]
            else:
                # If all values are the same, set to middle of range
                portion_to_place[:] = (self._vegetation_quality_range[0] + self._vegetation_quality_range[1]) / 2
        else:
            # If no valid values, fill with minimum vegetation quality
            portion_to_place[:] = self._vegetation_quality_range[0]
        
        # Place the processed portion in the map - Note: transpose to match np.ones(width, height) structure
        self._map_original[start_row:end_row, start_col:end_col] = portion_to_place
        
        # Store coordinate axes for visualization
        # Create coordinate arrays matching the final map dimensions
        self.x_axis = self.np.zeros(self._width)
        self.y_axis = self.np.zeros(self._height)
        
        # Fill coordinate arrays with selected coordinates, padding with extrapolated values if needed
        if actual_width > 0 and actual_height > 0:
            # Place longitude coordinates (x_axis corresponds to width dimension)
            coord_start_col = start_col
            coord_end_col = min(start_col + len(selected_longitude), self._height)
            self.x_axis = selected_longitude[:coord_end_col - coord_start_col]
            
            # Place latitude coordinates (y_axis corresponds to height dimension) 
            coord_start_row = start_row
            coord_end_row = min(start_row + len(selected_latitude), self._width)
            self.y_axis = selected_latitude[:coord_end_row - coord_start_row]                        
        else:
            # Fallback to index-based coordinates
            self.x_axis = self.np.arange(self._width, dtype=float)
            self.y_axis = self.np.arange(self._height, dtype=float)

        if self._print:
            print(f"Loaded map from NPY: {self._elevation_file_path}")
            print(f"Full map size: {full_height} x {full_width}")
            print(f"Selected portion: ({start_x}, {start_y}) to ({end_x}, {end_y})")
            print(f"Final map size: {self._map_original.shape} (width x height)")
            print(f"Value range: {self.np.min(self._map_original):.3f} to {self.np.max(self._map_original):.3f}")
            print(f"X-axis range: {self.x_axis.min():.6f} to {self.x_axis.max():.6f}")
            print(f"Y-axis range: {self.y_axis.min():.6f} to {self.y_axis.max():.6f}")
            
        
            
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
                    if self._map_original[nx, ny] >= 0:
                        self._map_original[nx, ny] += self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                            * norm.pdf(distance, 0.0, sigma)
                        self._map_original = self.np.clip(self._map_original, -self._streams_width, self._vegetation_quality_range[1])
        
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
                    if self._map_original[nx, ny] > 0:                        
                        self._map_original[nx, ny] -= self._vegetation_quality_range[1] * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                            * norm.pdf(distance, 0.0, sigma)
                        self._map_original[nx, ny] = self.np.clip(self._map_original[nx, ny], 0.05, self._vegetation_quality_range[1])
        
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
            
    def generate_streams(self, n_points = 1) -> None:
        for _ in range(self._streams_number):            
            
            # start from the left side
            bound = int(0.2 * self._height)
            start = self.np.array((0, self.random.randint(bound, self._height - bound)))
            end = self.np.array((self._width - 1, self.random.randint(bound, self._height - bound)))
            
            # middle points            
            middle_points = []
            for i in range(1, n_points + 1):
                fraction = i / (n_points + 1)
                x = int(start[0] + fraction * (end[0] - start[0]))                                
                y = self.random.randint(bound, self._height - bound)
                middle_points.append(self.np.array((x, y)))                            
            
            # generate stream
            points = [start] + middle_points + [end]
            self.generate_stream(points, degree=8)

    def generate_stream(self, points, degree = 3) -> None:
        
        sigma = 3        
        num_points = 1000
        path = module_misc.generate_path_from_points(points, degree, num_points, self._width-1, self._height-1)        
        for position in path:
            self._map_original[position[0], position[1]] = -(self._streams_width + 1)
            
        extended_path = []
        for position in path:
            x, y = position
            for width in range(2, self._streams_width + 1):
                limits = self.np.array([[0, self._width - 1], [0, self._height - 1]])                
                neighbours = module_misc.DN_neighbourhood(position, limits, N=8, step=width-1)
                for neighbour in neighbours:                    
                    if not any((neighbour == self.np.array(points)).all() for points in path) and \
                       not any((neighbour == self.np.array(points)).all() for points in extended_path) and \
                       module_misc.is_within_limits(neighbour, self._width-1, self._height-1):  
                            
                            extended_path.append(neighbour)                         
                            distance = self.np.sqrt((neighbour[0] - x)**2 + (neighbour[1] - y)**2)
                            self._map_original[neighbour[0], neighbour[1]] = -self._streams_width * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                        * norm.pdf(distance, 0.0, sigma)
                            self._map_original[neighbour[0], neighbour[1]] = self.np.clip(self._map_original[neighbour[0], neighbour[1]], -self._streams_width, -0.05)    