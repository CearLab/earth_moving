import os
import numpy as np
from numpy.random import seed, rand
from shapely.geometry import Polygon, MultiPoint, Point, LineString
import pybullet as p
import pybullet_data
import time
from engine2 import PyBulletEnvironment

# Function to draw a line in PyBullet with specified width
def draw_line(from_pos, to_pos, color=[0, 1, 0], width=3):
    p.addUserDebugLine(
        lineFromXYZ=[from_pos[0], from_pos[1], 0],  # Set z to 0
        lineToXYZ=[to_pos[0], to_pos[1], 0],        # Set z to 0
        lineColorRGB=color,
        lineWidth=width,
        lifeTime=0
    )

# Function to find the closest point on the circle
def closest_point_on_circle(circle_center, circle_radius, external_point):
    cx, cy = circle_center
    px, py = external_point

    # Calculate the direction vector from the circle's center to the external point
    direction = np.array([px - cx, py - cy])
    norm_direction = direction / np.linalg.norm(direction)

    # The closest point on the circle's circumference
    closest_point = circle_center + norm_direction * circle_radius

    return closest_point

# Use an absolute path for the URDF file
robot_urdf = os.path.abspath('./urdf/shovel/shovelFlat.urdf')

seed(1)
engine = PyBulletEnvironment()
# Load shovel
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the robot
engine.open_environment(robot_urdf, startPos, startOrientation)

# Admissible init positions
shovelLinkState = p.getLinkState(engine.ID[1], 2)
minpos = np.asarray(shovelLinkState[0]) - 5 * 1e-1
maxpos = np.asarray(shovelLinkState[0]) + 5 * 1e-1

# Number of pebbles
pebbleNum = 100
pebbleId = np.zeros(pebbleNum)

# Scatter pebbles and collect their positions
pebble_positions = []
for i in range(pebbleNum):
    startPos = minpos + (rand(3) * (maxpos - minpos))
    startPos[-1] = 0.2
    pebble_positions.append(startPos[:2])  # Append only x and y coordinates
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    engine.load_urdf(os.path.abspath('./urdf/pebbles/pebbles.urdf'), startPos, startOrientation)

# Collect shovel position
shovel_position = np.asarray(shovelLinkState[0])[:2]

# Define a circle position near the pebbles
circle_center = np.mean(pebble_positions, axis=0) + np.array([0.75, 0.75])
circle_radius = 0.3
circle = Point(circle_center).buffer(circle_radius)

# Draw the circle in PyBullet
engine.draw_shapely_shape(circle, height=0.01)

# Collect circle boundary points
circle_points = np.array(circle.exterior.coords[:-1])  # Exclude the repeated first/last point

# Combine all positions
all_positions = np.array(pebble_positions + [shovel_position] + list(circle_points))

# Create a convex hull using Shapely
points = MultiPoint([tuple(pos) for pos in all_positions])
convex_hull = points.convex_hull

# Draw the convex hull in PyBullet
engine.draw_shapely_shape(convex_hull, height=0.01)

# Define a distance threshold
distance_threshold = 0.05  # You can adjust this value based on your requirements

# Identify pebbles on the boundary of the convex hull or close to it
boundary_pebble_positions = []
for pos in pebble_positions:
    point = Point(pos)
    if convex_hull.boundary.distance(point) <= distance_threshold:
        boundary_pebble_positions.append(pos)

# Draw lines from boundary pebbles to the closest point on the circle
for pebble_pos in boundary_pebble_positions:
    closest_point = closest_point_on_circle(circle_center, circle_radius, pebble_pos)

    # Set z-coordinates to 0 to keep lines on the ground
    pebble_3d_pos = [pebble_pos[0], pebble_pos[1], 0]
    closest_3d_pos = [closest_point[0], closest_point[1], 0]
    draw_line(pebble_3d_pos, closest_3d_pos, color=[0, 1, 0], width=3)

# Simulate for some time
print("\nDone\n")
engine.simulate(0.5)

# Keep the simulation running
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)  # Add a small delay to avoid high CPU usage
except KeyboardInterrupt:
    # Disconnect from PyBullet on manual interruption
    engine.close_environment()
