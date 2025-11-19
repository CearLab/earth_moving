# general imports
from scipy.stats import norm

# backend imports
from beaverbot.ral.environment.environment_backend import BaseEnvironmentBackend

# module imports
import beaverbot.ral.algorithms.module_misc as module_misc


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
        self._river_growth_interval = flow_info.get('river_growth_interval', [-3, 0])
        self._visits_reset = self._environment.get('visits_reset')
        
        # Growth rate calculation: 2.6% growth per day, 0.00107 per hour (3 weeks to grow grass)
        # self._grass_growth_rate = 1 * 0.01 * 0.00107
        self._grass_growth_rate = self._environment.get('grass_growth_rate', 1e-5)        
        self._grass_mode = 'additive'  # 'additive' or 'percentage'
        self._grass_growth_interval = self._environment.get('grass_growth_interval')
        
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
            self._map_visits = self.np.zeros(self._map_original.shape)
            self._map_visits_roles = self.np.zeros(self._map_original.shape)

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
            
            # generate map            
            self.generate_vegetation()
            
            # maps init
            self._map = self._map_original.copy()
            self._map_visits = self.np.zeros(self._map_original.shape)     
            self._map_visits_roles = self.np.zeros(self._map_original.shape)       
                        
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
    
    def step_environment(self, dt, map, map_visits, home_base_position_store, grass_growth_interval, misc) -> None:
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
        self._map = map.copy()
        self._map_visits = map_visits.copy()
        map_visits_roles = misc.get('map_visits_roles', self.np.zeros(self._map_original.shape))
        self._map_visits_roles = map_visits_roles.copy()
        self._home_base_position_store = home_base_position_store

        # grow grass
        self.grow_grass(grass_growth_interval, self._grass_growth_rate, mode=self._grass_mode)
            
        # grow rivers
        self.grow_rivers()                            
            
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
        
        # Replace NaN values with a default value (0.0) for consistent processing
        elevation_data = self.np.nan_to_num(elevation_data, nan=0.0)
        
        # Linear rescaling to target range
        min_val = self.np.min(elevation_data)
        max_val = self.np.max(elevation_data)
        
        target_min = -self._streams_width
        target_max = self._vegetation_quality_range[1]
        
        # Linear rescaling: (data - data_min) / (data_max - data_min) * (target_max - target_min) + target_min
        if max_val > min_val:
            self._map_original = (elevation_data - min_val) / (max_val - min_val) * (target_max - target_min) + target_min
            rescaling_info = f"Linear rescaling [{min_val:.3f}, {max_val:.3f}] -> [{target_min:.3f}, {target_max:.3f}]"
        else:
            # All values are the same
            self._map_original = elevation_data * 0 + (target_max + target_min) / 2
            rescaling_info = f"All values are the same ({min_val:.3f}) -> {(target_max + target_min) / 2:.3f}"
                
        if self._print:
            print(f"Loaded map from NPY: {self._elevation_file_path}")
            print(f"Map size: {self._width} x {self._height}")
            print(f"Original data range: {min_val:.3f} to {max_val:.3f}")
            print(f"Target range: [{target_min:.3f}, {target_max:.3f}]")
            print(f"Final map range: {self.np.min(self._map_original):.3f} to {self.np.max(self._map_original):.3f}")
            print(f"Applied coordinate transformation: transpose + 180° rotation")
            print(f"  {rescaling_info}")
            
                    
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
                        start_point = self._map_original[nx, ny].copy()
                        start_height = 0.01 * self._vegetation_quality_range[1]
                        self._map_original[nx, ny] = start_point + start_height * self.np.sqrt(2 * self.np.pi * sigma**2) \
                                            * norm.pdf(distance, 0.0, sigma)                        
                        self._map_original = self.np.clip(self._map_original, -self._streams_width, self._vegetation_quality_range[1])

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
        growth_mask = (self._map >= grass_growth_interval[0]) & \
                      (self._map <= grass_growth_interval[1])
        
        if self.np.any(growth_mask):
            # Increase by rate, but if value is zero, set to a small positive value
            if mode == 'percentage':
                self._map[growth_mask] = self._map[growth_mask] * (1 + rate)
                zero_mask = ((self._map >= 0) & (self._map <= 0.05))
                if self.np.any(zero_mask):
                    self._map[zero_mask] = 0.05
                # Clip the grown values between zero and the maximum vegetation quality
                self._map[growth_mask] = self.np.clip(
                    self._map[growth_mask], 0, 0.7 * self._vegetation_quality_range[1]
                )
            # additive rate 
            elif mode == 'additive':
                self._map[growth_mask] = self._map[growth_mask] + rate
                    # Clip the grown values between zero and the maximum vegetation quality
                self._map[growth_mask] = self.np.clip(
                    self._map[growth_mask], 0, self._vegetation_quality_range[1]
                )
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
            negative_mask = (self._map >= self._river_growth_interval[0]) & (self._map <= self._river_growth_interval[1])
            
            # Increase magnitude of negative values by the growth velocity percentage
            # Since values are negative, we multiply by (1 + growth_velocity) to make them more negative
            # self._map[negative_mask] = self._map[negative_mask] * (1 - self.np.sign(self._map[negative_mask]) * self._river_growth_velocity)
            self._map[negative_mask] = self._map[negative_mask] - self._river_growth_velocity

            # Saturate at -streams_width to prevent unlimited deepening
            self._map[negative_mask] = self.np.clip(self._map[negative_mask], 
                                                            self._river_growth_interval[0], self._river_growth_interval[1])
        