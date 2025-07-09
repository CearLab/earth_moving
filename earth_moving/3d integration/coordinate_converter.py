import math

class CoordinateConverter:
    def __init__(self, env_radius=1.0, target_zone_radius=0.3, shovel_width=0.22):
        """
        Initialize the coordinate converter with environment parameters.
        
        Args:
            env_radius (float): Radius of the environment in meters
            target_zone_radius (float): Radius of the target zone in meters
            shovel_width (float): Width of the shovel in meters
        """
        # Store parameters
        self.env_radius = env_radius
        self.target_zone_radius = target_zone_radius
        self.shovel_width = shovel_width
        
        # Calculate cell size based on shovel width
        self.cell_size = self.shovel_width / (2 * math.sqrt(2))  # meters per cell
        
        # Calculate grid size based on environment diameter
        env_diameter = 2 * self.env_radius  # meters
        self.grid_size = math.ceil(env_diameter / self.cell_size)
        
        # Ensure grid size is odd for better centering
        if self.grid_size % 2 == 0:
            self.grid_size += 1
            
        # Calculate key dimensions in cells
        self.target_zone_cells = int(self.target_zone_radius / self.cell_size)  # Convert radius to cells
        self.shovel_cells = self.shovel_width / self.cell_size
        
        print("\nCoordinate Converter Initialized:")
        print(f"Environment radius: {self.env_radius:.3f}m")
        print(f"Target zone radius: {self.target_zone_radius:.3f}m")
        print(f"Shovel width: {self.shovel_width:.3f}m")
        print(f"Cell size: {self.cell_size:.3f}m")
        print(f"Grid size: {self.grid_size}x{self.grid_size} cells")
        print(f"Target zone size: {self.target_zone_cells} cells")
        print(f"Shovel coverage: {self.shovel_cells:.1f} cells")

    def convert_3d_to_2d(self, x, y):
        """Inverse of the above."""
        grid_x = int((x + self.env_radius) / self.cell_size)
        grid_y = int((self.env_radius - y) / self.cell_size)  # <- flip Y
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))
        return grid_x, grid_y

    def convert_2d_to_3d(self, grid_x, grid_y):
        """
        Pygame grid (0,0) = top-left.
        PyBullet world (0,0) = centre, +Y points *up* the screen.
        """
        x = (grid_x * self.cell_size) - self.env_radius
        y = self.env_radius - grid_y * self.cell_size  # <- flip Y
        return x, y

    def get_shovel_coverage_cells(self, grid_x, grid_y):
        """
        Get the grid cells covered by the shovel at a given position.
        
        Args:
            grid_x, grid_y (int): Center position of the shovel in grid coordinates
            
        Returns:
            list: List of (x, y) tuples representing covered cells
        """
        half_coverage = math.ceil(self.shovel_cells / 2)
        covered_cells = []
        
        for dx in range(-half_coverage, half_coverage + 1):
            for dy in range(-half_coverage, half_coverage + 1):
                cell_x = grid_x + dx
                cell_y = grid_y + dy
                
                # Check if cell is within grid bounds
                if (0 <= cell_x < self.grid_size and 
                    0 <= cell_y < self.grid_size):
                    covered_cells.append((cell_x, cell_y))
                    
        return covered_cells
        
    def is_in_target_zone(self, grid_x, grid_y):
        """
        Check if a grid cell is within the target zone.
        
        Args:
            grid_x, grid_y (int): Grid coordinates to check
            
        Returns:
            bool: True if the cell is within the target zone
        """
        # Convert grid coordinates to 3D
        x, y = self.convert_2d_to_3d(grid_x, grid_y)
        
        # Check distance from center
        distance = math.sqrt(x*x + y*y)
        return distance <= self.target_zone_radius
        
    def get_environment_info(self):
        """
        Get information about the environment dimensions and conversions.
        
        Returns:
            dict: Dictionary containing environment information
        """
        return {
            'env_radius': self.env_radius,
            'target_zone_radius': self.target_zone_radius,
            'shovel_width': self.shovel_width,
            'cell_size': self.cell_size,
            'grid_size': self.grid_size,
            'target_zone_cells': self.target_zone_cells,
            'shovel_cells': self.shovel_cells
        }
        
    def convert_objects_to_2d(self, objects_3d):
        """
        Convert a list of 3D object positions to 2D grid coordinates.
        
        Args:
            objects_3d (list): List of (x, y, z) tuples representing 3D positions
            
        Returns:
            list: List of (grid_x, grid_y) tuples
        """
        return [self.convert_3d_to_2d(x, y) for x, y, _ in objects_3d]
        
    def convert_agents_to_2d(self, agents_3d):
        """
        Convert a list of 3D agent positions and orientations to 2D grid coordinates.
        
        Args:
            agents_3d (list): List of (x, y, z, orientation) tuples
            
        Returns:
            list: List of (grid_x, grid_y, orientation) tuples
        """
        return [(self.convert_3d_to_2d(x, y)[0], 
                self.convert_3d_to_2d(x, y)[1], 
                orientation) for x, y, _, orientation in agents_3d] 