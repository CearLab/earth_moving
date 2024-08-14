import os
import numpy as np
from shapely.geometry import Point  # Corrected import statement
from geometry import create_convex_hull, find_boundary_pebble_positions, closest_point_on_circle
from drawing import draw_line, draw_shapely_shape, draw_smooth_curve_from_points
from pebble import scatter_pebbles
from engine_new import PyBulletEnvironment
from movement import RobotMovement

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
minpos = np.asarray(shovelLinkState[0]) - 5 * 1e-1
maxpos = np.asarray(shovelLinkState[0]) + 5 * 1e-1

# Scatter pebbles and collect their positions
pebble_positions = scatter_pebbles(engine, minpos+1, maxpos+1, 100, pebble_urdf)

# Define a circle position near the pebbles
circle_center = np.mean(pebble_positions, axis=0) + np.array([0.75, 0.75])
circle_radius = 0.3
circle = Point(circle_center).buffer(circle_radius)

# Draw the circle in PyBullet
draw_smooth_curve_from_points(engine, list(circle.exterior.coords), color=[1, 0, 0])

# Collect circle boundary points
circle_points = np.array(circle.exterior.coords[:-1])  # Exclude the repeated first/last point

# Create a convex hull using Shapely
convex_hull = create_convex_hull(pebble_positions, circle_points)

# Draw the convex hull in PyBullet
draw_shapely_shape(engine, convex_hull, height=0.01)

# Find boundary pebble positions
boundary_pebble_positions = find_boundary_pebble_positions(pebble_positions, convex_hull, 0.05)

# Draw lines from boundary pebbles to the closest point on the circle
for pebble_pos in boundary_pebble_positions:
    closest_point = closest_point_on_circle(circle_center, circle_radius, pebble_pos)
    draw_line(engine, pebble_pos, closest_point, color=[0, 1, 0], width=3)

# Get the current end effector position and orientation
endEffectorPos = engine.get_link_state(engine.ID[1], 2)[0]
endEffectorOrientation = engine.get_link_state(engine.ID[1], 2)[1]

# Convert quaternion to Euler angles
endEffectorOrientationEuler = engine.get_euler_from_quaternion(endEffectorOrientation)

# Create a RobotMovement instance for different movements
movements = [
    RobotMovement(movement_type="forward", length=1, curvature=0, Nwaypoints=10),
    RobotMovement(movement_type="arc_right", length=1, curvature=0.5, Nwaypoints=10),
    RobotMovement(movement_type="arc_left", length=1, curvature=0.5, Nwaypoints=10),
    RobotMovement(movement_type="forward", length=1, curvature=0, Nwaypoints=10),
    RobotMovement(movement_type="arc_right", length=1, curvature=0.5, Nwaypoints=10),
    RobotMovement(movement_type="arc_left", length=1, curvature=0.5, Nwaypoints=10)
    # RobotMovement(movement_type="backward", length=1, curvature=0, Nwaypoints=10),
    # RobotMovement(movement_type="arc_backward_right", length=1, curvature=0.5, Nwaypoints=10),
    # RobotMovement(movement_type="arc_backward_left", length=1, curvature=0.5, Nwaypoints=10)
]

# Generate waypoints for the specified movements
all_waypoints = []
current_position = endEffectorPos
current_orientation = endEffectorOrientationEuler

for movement in movements:
    waypoints, new_orientation = movement.generate_waypoints(current_position, current_orientation)
    all_waypoints.extend(waypoints)
    current_position = waypoints[-1]
    current_orientation = new_orientation

# Draw the combined path
draw_smooth_curve_from_points(engine, all_waypoints, color=[1, 0, 1])

# Control the robot to follow the combined path
mode = "POSITION"  # Adjusted to string "POSITION"
target_velocities = [1e-3, 1e-2, 1e-2]
engine.control_waypoints(all_waypoints, target_velocities, control_mode=mode)

# Simulate and keep the environment running
try:
    while True:
        engine.step_simulation()
except KeyboardInterrupt:
    engine.close_environment()
