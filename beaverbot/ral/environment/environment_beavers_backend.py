from beaverbot.ral.environment.environment_backend import BaseEnvironmentBackend


class BeaversEnvironmentBackend(BaseEnvironmentBackend):
    """Backend for Beavers environment simulation."""
    
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        
    def initiate_environment(self, **kwargs):
        """Initialize environment: load terrain, setup growth parameters, initialize time tracking."""
        super().initiate_environment(**kwargs)
        
        # General parameters
        self._vegetation_quality_range = self._environment.get('vegetation_quality_range')
        self._print = self._environment.get('print')
        self._visits_reset = self._environment.get('visits_reset')
        
        # River parameters
        flow_info = self._environment.get('flow_info', {'direction': [0, 0], 'strength': 0, 'streams_width': 10})
        self._streams_width = flow_info.get('streams_width', 10)
        self._flow_direction = flow_info.get('direction', [0, 0])
        self._flow_strength = flow_info.get('strength', 0)
        self._river_growth_velocity = flow_info.get('river_growth_velocity', 0.0)
        self._river_growth_interval = flow_info.get('river_growth_interval', [-3, 0])
        
        # Grass growth parameters (2.6% per day = 0.00107/hour, 3 weeks to full growth)
        self._grass_growth_rate = self._environment.get('grass_growth_rate', 1e-5)
        self._grass_mode = 'additive'  # 'additive' or 'percentage'
        self._grass_growth_interval = self._environment.get('grass_growth_interval')
        
        # Map file path
        self._elevation_file_path = self._environment.get('elevation_file_path', None)
        
        # Load map (sets self._width, self._height, self._map_original)
        self.load_map_from_file()
        
        # Initialize map state
        self._initial_map = self._map_original.copy()
        self._map = self._map_original.copy()
        self._map_visits = self.np.zeros(self._map_original.shape)
        self._map_visits_roles = self.np.zeros(self._map_original.shape)
        
        # Time tracking
        self._current_time = 0
        self._current_day = []
        self._current_hour = []
        self._time_of_day = []
        self.update_time_of_day()
        
        # Home base
        self._home_base_position_store = None                
        
        return self   
    
    def update_time_of_day(self) -> None:
        """Update time of day (currently always 'day')."""
        self._current_hour = self._current_time % 24
        self._current_day = self._current_time // 24
        
        # Currently always 'day' (remove "or True" to enable day/night cycle)
        if self._current_hour > 6 and self._current_hour < 18 or True:
            self._time_of_day = 'day'
        else:
            self._time_of_day = 'night'   
    
    def step_environment(self, dt, map, map_visits, home_base_position_store, grass_growth_interval, misc) -> None:
        """Update environment for one time step: update maps, grow grass, deepen rivers."""
        self._current_time += dt
        self.update_time_of_day()
        
        # Update maps from agent observations
        self._map = map.copy()
        self._map_visits = map_visits.copy()
        map_visits_roles = misc.get('map_visits_roles', self.np.zeros(self._map_original.shape))
        self._map_visits_roles = map_visits_roles.copy()
        self._home_base_position_store = home_base_position_store

        # Grow grass and deepen rivers
        self.grow_grass(grass_growth_interval, self._grass_growth_rate, mode=self._grass_mode)
        self.grow_rivers()
        
        if self._print:
            print(f"Environment step at time {self._current_time}")
    
    def load_map_from_file(self) -> None:
        """Load elevation map from .npy file, transform coordinates, and rescale values."""
        if self._elevation_file_path is None:
            raise ValueError("Elevation file path must be provided")
        
        try:
            elevation_data = self.np.load(self._elevation_file_path)
        except Exception as e:
            raise ValueError(f"Error loading NPY file: {str(e)}")
        
        # Transform coordinates: DEM (rows=y, cols=x, origin=lower-left) -> Environment ([x,y], origin=top-left)
        elevation_data = self.np.flipud(elevation_data)
        elevation_data = self.np.rot90(elevation_data, -1)
        self._width, self._height = elevation_data.shape
        
        # Replace NaN with 0.0
        elevation_data = self.np.nan_to_num(elevation_data, nan=0.0)
        
        # Linear rescaling to simulation range
        min_val = self.np.min(elevation_data)
        max_val = self.np.max(elevation_data)
        target_min = -self._streams_width
        target_max = self._vegetation_quality_range[1]
        
        if max_val > min_val:
            self._map_original = (elevation_data - min_val) / (max_val - min_val) * (target_max - target_min) + target_min
            rescaling_info = f"[{min_val:.3f}, {max_val:.3f}] -> [{target_min:.3f}, {target_max:.3f}]"
        else:
            self._map_original = elevation_data * 0 + (target_max + target_min) / 2
            rescaling_info = f"Constant {min_val:.3f} -> {(target_max + target_min) / 2:.3f}"
        
        if self._print:
            print(f"Loaded map: {self._elevation_file_path}")
            print(f"Size: {self._width}x{self._height}, Range: {self.np.min(self._map_original):.3f} to {self.np.max(self._map_original):.3f}")
            print(f"Rescaling: {rescaling_info}")

    def grow_grass(self, grass_growth_interval, rate, mode) -> None:
        """Simulate vegetation growth using percentage or additive mode."""
        growth_mask = (self._map >= grass_growth_interval[0]) & (self._map <= grass_growth_interval[1])
        
        if self.np.any(growth_mask):
            if mode == 'percentage':
                self._map[growth_mask] = self._map[growth_mask] * (1 + rate)
                zero_mask = ((self._map >= 0) & (self._map <= 0.05))
                if self.np.any(zero_mask):
                    self._map[zero_mask] = 0.05
                self._map[growth_mask] = self.np.clip(self._map[growth_mask], 0, 0.7 * self._vegetation_quality_range[1])
            elif mode == 'additive':
                self._map[growth_mask] = self._map[growth_mask] + rate
                self._map[growth_mask] = self.np.clip(self._map[growth_mask], 0, self._vegetation_quality_range[1])
            else:
                raise ValueError(f"Invalid growth mode: {mode}. Must be 'percentage' or 'additive'.")

    def grow_rivers(self) -> None:
        """Simulate river deepening by subtracting growth velocity."""
        if self._river_growth_velocity > 0:
            negative_mask = (self._map >= self._river_growth_interval[0]) & (self._map <= self._river_growth_interval[1])
            self._map[negative_mask] = self._map[negative_mask] - self._river_growth_velocity
            self._map[negative_mask] = self.np.clip(
                self._map[negative_mask],
                self._river_growth_interval[0],
                self._river_growth_interval[1]
            )
        