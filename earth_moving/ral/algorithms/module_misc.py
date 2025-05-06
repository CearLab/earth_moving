
# imports
import numpy as np
import random
from scipy.interpolate import CubicSpline
import earth_moving.constants as const

def generate_streams(n_points = 1) -> None:
            
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

def generate_path_from_points(points, degree, num_points, width, height) -> list:                

        # Generate cubic spline points
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

        # Create cubic splines for x and y
        t = np.linspace(0, 1, len(points))
        spline_x = np.poly1d(np.polyfit(t, x, deg=degree))
        spline_y = np.poly1d(np.polyfit(t, y, deg=degree))

        # Generate points along the spline
        t_values = np.linspace(0, 1, num_points)
        spline_points = np.array([[spline_x(t), spline_y(t)] for t in t_values])

        # Round and convert to integer coordinates
        spline_points = np.rint(spline_points).astype(int)

        # Clip points to the limits
        spline_points = np.clip(spline_points, [0, 0], [width, height])

        # Remove duplicates
        path = []
        seen = set()
        for point in spline_points:
            if tuple(point) not in seen:
                path.append(point.tolist())
                seen.add(tuple(point))

        return path


    


    