import os
import numpy as np
from shapely.geometry import Point
from geometry import create_convex_hull, closest_point_on_circle
from drawing import draw_line, draw_smooth_curve_from_points, draw_shapely_shape
from pebble import scatter_pebbles
from engine_new import PyBulletEnvironment
from movement import RobotMovement
from grid import Grid  # Import the Grid class

# Initialize parameters
robot_urdf = os.path.abspath('./urdf/shovel/shovelFlat.urdf')
pebble_urdf = os.path.abspath('./urdf/pebbles/pebbles.urdf')
engine = PyBulletEnvironment()

# Load shovel
startPos = [0, 0, 0]
startOrientation = engine.get_quaternion_from_euler([0, 0, 0])
engine.open_environment(robot_urdf, startPos, startOrientation)

# Get shovel link state and admissible init positions
shovelLinkState = engine.get_link_state(engine.ID[1], 2)
minpos = np.asarray(shovelLinkState[0]) - 0.5
maxpos = np.asarray(shovelLinkState[0]) + 0.5

# Scatter pebbles and collect their positions
pebble_positions = scatter_pebbles(engine, minpos+0.5, maxpos+0.5, 100, pebble_urdf)

# Collect shovel position
shovel_position = np.asarray(shovelLinkState[0])[:2]

# Define a circle position near the pebbles
pebble_positions_array = np.array(pebble_positions)
pebble_positions_mean = np.mean(pebble_positions_array, axis=0)
circle_center = pebble_positions_mean + np.array([0.55, 0.55])
circle_radius = 0.3
circle = Point(circle_center).buffer(circle_radius)

# Draw the circle in PyBullet
draw_smooth_curve_from_points(engine, list(circle.exterior.coords), color=[1, 0, 0])

# Create a grid
# Determine x_range and y_range based on pebble positions with some padding
padding = 0.5
min_x, min_y = np.min(pebble_positions_array, axis=0) - padding
max_x, max_y = np.max(pebble_positions_array, axis=0) + padding

cell_size = 0.1  # Adjust cell size as needed
x_range = [min_x, max_x]
y_range = [min_y, max_y]

# Initialize the grid
grid = Grid(cell_size, x_range, y_range)

# Assign pebbles to grid cells
grid.assign_pebbles_to_cells(pebble_positions)

# Draw the grid
grid.draw_grid()

# Create a convex hull using Shapely
convex_hull = create_convex_hull(pebble_positions, circle.exterior.coords[:-1])

# Draw the convex hull in PyBullet
draw_shapely_shape(engine, convex_hull, height=0.01)

# Find boundary cells and draw lines from these cells to the closest point on the circle
from shapely.geometry import Point

distance_threshold = 0.05  # Adjust as necessary

boundary_cells = []

cells_with_pebbles = grid.get_cells_with_pebbles()
for (x_index, y_index), pebbles_in_cell in cells_with_pebbles.items():
    cell_center = grid.get_cell_center(x_index, y_index)
    point = Point(cell_center)
    if convex_hull.boundary.distance(point) <= distance_threshold:
        boundary_cells.append(cell_center)

# Draw lines from boundary cell centers to the closest point on the circle
for cell_center in boundary_cells:
    closest_point = closest_point_on_circle(circle_center, circle_radius, cell_center)
    draw_line(engine, cell_center, closest_point, color=[0, 1, 0], width=3)

# Simulate and keep the environment running
try:
    while True:
        engine.step_simulation()
except KeyboardInterrupt:
    engine.close_environment()
