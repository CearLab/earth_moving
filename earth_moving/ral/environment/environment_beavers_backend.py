# general imports
from scipy.stats import norm

# backend imports
from earth_moving.ral.environment.environment_backend import BaseEnvironmentBackend

# module imports
import earth_moving.ral.algorithms.module_misc as module_misc


class BeaversEnvironmentBackend(BaseEnvironmentBackend):
    """
    Beavers Environment Backend for simulating a dynamic ecosystem environment.
    
    This class implements a 2D grid-based environment simulation that models vegetation growth,
    water streams, and temporal dynamics for beaver habitat simulation. It supports both
    procedurally generated environments and loading from CSV/NPY data files.
    
    Key Features:
    - Dynamic vegetation clusters with growth simulation
    - Stream generation with configurable width and meandering patterns
    - Time-based updates with day/night cycles
    - Map loading from external data files (DEM, elevation data)
    - Vegetation quality degradation based on visitation patterns
    - Configurable environment parameters through configuration files
    
    Attributes:
        _vegetation_quality_init_range (tuple): Initial range for vegetation quality values
        _vegetation_quality_range (tuple): Full range for vegetation quality values
        _streams_width (int): Width of generated streams
        _map_mode (str): Environment generation mode ('generate' or 'csv')
        _width (int): Environment width in grid cells
        _height (int): Environment height in grid cells
        _map_original (numpy.ndarray): Original environment map without modifications
        _map (numpy.ndarray): Current environment map with dynamic updates
        _map_visits (numpy.ndarray): Map tracking visitation patterns
        _current_time (int): Current simulation time
        _vegetation_clusters_store (list): Storage for vegetation cluster information
    """ 
    
    def __init__(self, **kwargs) -> None:
        """
        Initialize the BeaversEnvironmentBackend.
        
        Args:
            **kwargs: Keyword arguments passed to the parent class
        """
        super().__init__(**kwargs)        
        
    def initiate_environment(self, **kwargs):
        """
        Initialize the environment with configuration parameters and generate/load the map.
        
        This method sets up the environment based on the provided configuration, handling
        both procedural generation and data loading modes. It initializes vegetation
        clusters, streams, and time tracking.
        
        Args:
            **kwargs: Configuration parameters including:
                - environment: Dictionary containing environment configuration
                - vegetation_quality_init_range: Initial vegetation quality range
                - vegetation_quality_range: Full vegetation quality range
                - streams_width: Width of water streams
                - map_mode: 'generate' for procedural or 'csv' for data loading
                - width/height: Environment dimensions (for generate mode)
                - elevation_file_path: Path to elevation data (for csv mode)
                
        Returns:
            self: Returns the instance for method chaining
            
        Raises:
            ValueError: If initial vegetation clusters exceed maximum allowed
            ValueError: If invalid map_mode is specified
        """
        
        super().initiate_environment(**kwargs)                                
                
        # general environment parameters             
        self._vegetation_quality_range = self._environment.get('vegetation_quality_range')
        default_init = (self._vegetation_quality_range[1]-self._vegetation_quality_range[0]) // 2
        self._vegetation_quality_init_range = self._environment.get('vegetation_quality_init_range', (default_init, default_init + 1))
        self._print = self._environment.get('print')
        
        # flow parameters
        flow_info = self._environment.get('flow_info', {'direction': [0, 0], 'strength': 0, 'streams_width': 10})
        self._flow_direction = flow_info.get('direction', [0, 0])  # [dx, dy]
        self._flow_strength = flow_info.get('strength', 0)         # scalar (0-1)
        self._streams_width = flow_info.get('streams_width', 10)   # stream width in grid cells
        self._river_growth_velocity = flow_info.get('river_growth_velocity', 0.0)
        
        # Growth rate calculation: 2.6% growth per day, 0.00107 per hour (3 weeks to grow grass)
        # self._grass_growth_rate = 1 * 0.05 * 0.00107
        self._grass_growth_rate = 1e-6 * self._vegetation_quality_range[1]
        self._grass_mode = 'additive'  # 'additive' or 'percentage'
        
        # map generation mode
        self._map_mode = self._environment.get('map_mode', 'generate')  # 'generate' or 'csv'                
        
        if self._map_mode == 'csv':
            # csv file path and coordinate files
            self._elevation_file_path = self._environment.get('elevation_file_path', None)
            self._latitude_file_path = self._environment.get('latitude_file_path', None)
            self._longitude_file_path = self._environment.get('longitude_file_path', None)            
            
            # Load map from NPY file (this sets self._width, self._height, and self._map_original)
            self.load_map_from_npy()            
            
            # maps init
            self._map = self._map_original.copy()
            self._map_visits = self._map_original.copy()            
            
        elif self._map_mode == 'generate': 
            # parse the config file        
            self._width = self._environment.get('width')
            self._height = self._environment.get('height')                        
            
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
            
            # maps init
            self._map = self._map_original.copy()
            self._map_visits = self._map_original.copy()            
            
            # generate map
            self.generate_streams(n_points=12)
            self.generate_vegetation()
        else:
            raise ValueError(f"Invalid map mode: {self._map_mode}. Must be 'generate' or 'csv'.")                
        
        # init class attributes     
        self._initial_map = self._map_original.copy()   
        self._current_time = 0
        self._current_day = []
        self._current_hour = []
        self._time_of_day = []
        self.update_time_of_day()        
        self._number_vegetation_clusters = 0                
        self._vegetation_clusters_store = []
        
        # home base position store
        self._home_base_position_store = None                
        
        return self   
    
    def update_time_of_day(self) -> None:
        """
        Update the current time tracking including day, hour, and time of day classification.
        
        Calculates the current day and hour based on the current time counter,
        and determines whether it's day or night time for the simulation.
        """
        self._current_hour = self._current_time % 24
        self._current_day = self._current_time // 24

        # Currently configured to always be 'day' time.
        if self._current_hour > 6  and self._current_hour < 18 or True:
            self._time_of_day = 'day' 
        else:  
            self._time_of_day = 'night'   
    
    def step_environment(self, dt, map_visits, home_base_position_store, grass_growth_interval) -> None:
        """
        Advance the environment simulation by one time step.
        
        Updates the environment state including time progression, vegetation degradation
        based on visits, grass growth, and vegetation cluster expansion. This method
        handles the core dynamics of the ecosystem simulation.
        
        Args:
            dt (float): Time delta for this simulation step
            map_visits (numpy.ndarray): Array tracking visit counts for each grid cell
            home_base_position_store: Storage for home base position information
            grass_growth_interval (tuple): Range of vegetation values eligible for growth
            
        Side Effects:
            - Updates internal time tracking
            - Modifies vegetation maps based on visitation patterns
            - Triggers vegetation growth at specified intervals
            - Updates vegetation cluster storage
        """ 
        self._current_time += dt
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
        self.grow_grass(grass_growth_interval, self._grass_growth_rate, mode=self._grass_mode)
            
        # grow rivers
        self.grow_rivers()
                    
        # grow vegetation clusters
        if self._map_mode == 'generate':
            if (self._current_time % self._vegetation_growth_frequency == 0):
                self.grow_vegetation()                     
            
        # prints
        if self._print:
            print("Vegetation Clusters:")
            for cluster in self._vegetation_clusters_store:
                print(f"Cluster ID: {cluster[0]}, X: {cluster[1]}, Y: {cluster[2]}, Radius: {cluster[3]}")                
    
    def load_map_from_npy(self) -> None:
        """
        Load map from NPY file and rescale data to vegetation quality range.
        
        Loads elevation data from a numpy binary file, applies coordinate system
        transformations to match the environment's expected orientation, handles
        invalid data (NaN, negative values), and rescales the data to the
        vegetation quality range.
        
        Coordinate System Transformations:
        - DEM data: rows=y, columns=x, origin=lower-left
        - Environment: expects [width, height] = [x, y], origin=top-left
        
        Raises:
            ValueError: If elevation file path is not provided
            ValueError: If there's an error loading the NPY file
            
        Side Effects:
            - Sets self._width and self._height from data dimensions
            - Creates self._map_original with rescaled elevation data
            - Invalid areas are marked with -self._streams_width
        """
        if self._elevation_file_path is None:
            raise ValueError("NPY file path must be provided when map_mode is 'csv'")
        
        try:
            # Load numpy binary file
            elevation_data = self.np.load(self._elevation_file_path)
        except Exception as e:
            raise ValueError(f"Error loading NPY file: {str(e)}")
        
        # Fix coordinate system orientation:
        # DEM data: rows=y, columns=x, origin=lower-left  
        # Environment: expects [width, height] = [x, y], origin=top-left        
        elevation_data = self.np.flipud(elevation_data)
        elevation_data = self.np.rot90(elevation_data, -1)
        
        # Get dimensions from the processed data
        self._width, self._height = elevation_data.shape
        
        # Handle NaN and -1.0 values (masked areas)
        invalid_mask = (self.np.isnan(elevation_data)) | (elevation_data < 0.0)
        valid_mask = ~invalid_mask
        
        if self.np.any(valid_mask):            
            elevation_data[invalid_mask] = self.np.nan
        else:
            # If all values are invalid, fill with minimum vegetation quality
            elevation_data[:] = self._vegetation_quality_range[0]
        
        # Rescale data to vegetation quality range
        min_val = self.np.min(elevation_data[valid_mask])
        max_val = self.np.max(elevation_data[valid_mask])

        if max_val > min_val:
            # Rescale to vegetation quality range
            self._map_original = (elevation_data - min_val) / (max_val - min_val) * \
                               (self._vegetation_quality_range[1] - self._vegetation_quality_range[0]) + \
                               self._vegetation_quality_range[0]
        else:
            # If all values are the same, set to middle of range
            self._map_original = self.np.full_like(elevation_data, 
                                                 (self._vegetation_quality_range[0] + self._vegetation_quality_range[1]) / 2)
            
        # Ensure all negative values in _map_original are set to -self._streams_width
        #! Here, the streams depth is constant. Maybe we should find a way to have variable
        #! depth based on the distance from the stream center?
        self._map_original[invalid_mask] = -self._streams_width
                
        if self._print:
            print(f"Loaded map from NPY: {self._elevation_file_path}")
            print(f"Map size: {self._width} x {self._height}")
            print(f"Original data range: {min_val:.3f} to {max_val:.3f}")
            print(f"Rescaled to vegetation quality range: {self.np.min(self._map_original):.3f} to {self.np.max(self._map_original):.3f}")
            print(f"Applied coordinate transformation: transpose + 180° rotation")
            
                    
    def generate_vegetation(self) -> None:
        """
        Generate initial vegetation clusters across the environment.
        
        Creates the specified number of initial vegetation clusters at random
        positions throughout the environment map. Each cluster is generated
        with a random radius within the configured range.
        """
        for _ in range(self._number_vegetation_clusters_init):
            cx, cy = self.random.randint(0, self._width - 1), self.random.randint(0, self._height - 1)
            self.generate_cluster(cx, cy)
            
        #! TODO: provide a way to load initial clusters from file?
        
    def generate_cluster(self, cx, cy, cluster_radius=None) -> None:
        """
        Generate a vegetation cluster at the specified coordinates using Gaussian distribution.
        
        Creates a circular vegetation cluster centered at (cx, cy) with vegetation
        quality distributed according to a Gaussian probability density function.
        The cluster affects the environment map by adding vegetation values that
        can overlap with existing vegetation.
        
        Args:
            cx (int): X-coordinate of the cluster center
            cy (int): Y-coordinate of the cluster center
            cluster_radius (int, optional): Radius of the cluster. If None,
                randomly selected from the configured radius range
                
        Side Effects:
            - Modifies self._map_original by adding vegetation values
            - Updates or adds cluster information to self._vegetation_clusters_store
            - Increments self._number_vegetation_clusters counter
            - Clips vegetation values to valid range
        """
        
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
                        self._map_original[nx, ny] = self.np.clip(self._map_original[nx, ny], self._vegetation_quality_range[0], 0.6 * self._vegetation_quality_range[1])
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
        """
        Expand an existing vegetation cluster by increasing its radius.
        
        Removes the existing cluster's vegetation contribution from the map,
        increases the cluster radius by 1, and regenerates the cluster with
        the new larger radius. This simulates natural vegetation expansion.
        
        Args:
            cx (int): X-coordinate of the cluster center
            cy (int): Y-coordinate of the cluster center
            cluster_radius (int): Current radius of the cluster
            
        Side Effects:
            - Temporarily removes vegetation from the original cluster area
            - Increases cluster radius by 1
            - Regenerates the cluster with the new radius
            - Updates cluster information in storage
        """
        
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
        """
        Trigger growth for all existing vegetation clusters and potentially create new ones.
        
        Expands all existing vegetation clusters by calling grow_cluster for each
        stored cluster. If the current number of clusters is below the maximum
        allowed, creates a new cluster at a random location with minimum radius.
        
        Side Effects:
            - Grows all existing vegetation clusters
            - May create new vegetation clusters up to the maximum limit
            - Updates vegetation cluster storage and counter
        """                
        
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
            
    def generate_streams(self, n_points=1) -> None:
        """
        Generate meandering water streams across the environment.
        
        Creates the specified number of streams that flow from the left side
        to the right side of the environment. Each stream follows a path
        defined by start, middle, and end points, creating natural-looking
        meandering waterways.
        
        Args:
            n_points (int): Number of intermediate control points for stream curvature.
                Default is 1. Higher values create more complex meandering patterns.
                
        Side Effects:
            - Modifies self._map_original by setting stream areas to negative values
            - Each stream is generated with configurable width and smoothness
        """
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

    def generate_stream(self, points, degree=3) -> None:
        """
        Generate a single stream following a path defined by control points.
        
        Creates a smooth stream path using polynomial interpolation between
        the provided control points. The stream has a main channel with
        negative stream width value and surrounding areas with Gaussian-
        distributed depth values to create realistic riverbank gradients.
        
        Args:
            points (list): List of numpy arrays representing control points
                [(x1, y1), (x2, y2), ...] that define the stream path
            degree (int): Polynomial degree for path interpolation. Default is 3
                for cubic interpolation, providing smooth curves
                
        Side Effects:
            - Sets main stream channel to -(self._streams_width + 1)
            - Sets surrounding areas to negative values with Gaussian distribution
            - All stream values are clipped to valid negative range
            - Uses module_misc.generate_path_from_points for path creation
        """
        
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

    def grow_grass(self, grass_growth_interval, rate, mode) -> None:
        """
        Apply grass growth to vegetation areas within the specified interval.
        
        Increases vegetation quality for areas within the grass growth interval
        by a fixed percentage rate. This simulates natural grass regrowth over time.
        
        Args:
            grass_growth_interval (tuple): Range of vegetation values [min, max] 
                eligible for grass growth
                
        Side Effects:
            - Modifies self._map_original by increasing vegetation values
            - Only affects areas within the specified growth interval
            - Uses percentage-based growth rate (currently 2.6% per day, 0.00107 per hour)
        """        
        
        # Apply growth to vegetation within the specified interval
        growth_mask = (self._map_original >= grass_growth_interval[0]) & \
                      (self._map_original <= grass_growth_interval[1])
        
        if self.np.any(growth_mask):
            # Increase by rate, but if value is zero, set to a small positive value
            if mode == 'percentage':
                self._map_original[growth_mask] = self._map_original[growth_mask] + (1 + rate)            
                zero_mask = growth_mask & (self._map_original == 0)
                if self.np.any(zero_mask):
                    self._map_original[zero_mask] = rate            
            # additive rate 
            elif mode == 'additive':   
                self._map_original[growth_mask] = self._map_original[growth_mask] + min(rate * self._vegetation_quality_range[1], self._vegetation_quality_range[1])
            else:
                raise ValueError(f"Invalid growth mode: {mode}. Must be 'percentage' or 'additive'.")
                            
    def grow_rivers(self) -> None:
        """
        Expand rivers by increasing the magnitude of negative values.
        
        Each negative element in the map (representing water/rivers) increases 
        in magnitude by self._river_growth_velocity percentage. This simulates
        natural river erosion and deepening over time.
        
        Side Effects:
            - Modifies self._map_original by increasing magnitude of negative values
            - Uses percentage-based growth: new_value = old_value * (1 + growth_velocity)
            - Only affects negative values (water/river areas)
            - Saturates at -self._streams_width to prevent unlimited deepening
        """
        if self._river_growth_velocity > 0:
            # Find all negative values (rivers/water)
            negative_mask = self._map_original < 0
            
            if self.np.any(negative_mask):
                # Increase magnitude of negative values by the growth velocity percentage
                # Since values are negative, we multiply by (1 + growth_velocity) to make them more negative
                self._map_original[negative_mask] = self._map_original[negative_mask] * (1 + self._river_growth_velocity)
                
                # Saturate at -streams_width to prevent unlimited deepening
                self._map_original[negative_mask] = self.np.clip(self._map_original[negative_mask], 
                                                               -self._streams_width, 0)
        