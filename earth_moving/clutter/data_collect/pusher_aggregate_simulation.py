import pybullet as p
import pybullet_data
import time
import random
import math
import numpy as np
import csv


# custom imports
from earth_moving.ral.algorithms import module_misc as misc

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf

# Set gravity
p.setGravity(0, 0, -9.81)

# Load a flat plane with friction
plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=0.8)

# Create the "pusher" as a simple long box
pusher_thickness = 0.05
pusher_length = 0.7
pusher_height = 0.1

pusher_collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[pusher_thickness/2, pusher_length/2, pusher_height/2]
)

pusher_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[pusher_thickness/2, pusher_length/2, pusher_height/2],
    rgbaColor=[1, 0, 0, 1]
)

pusher_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=pusher_collision,
    baseVisualShapeIndex=pusher_visual,
    basePosition=[0, 0, pusher_height/2]
)

# Create simple aggregates as small cubes
def create_aggregate(pos):
    cube_size = 0.03  # Reduced size from 0.02 to 0.015
    col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size]*3)
    vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size]*3, rgbaColor=[0.2, 0.8, 0.2, 1])
    return p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis_shape, basePosition=pos)

# Scatter aggregates randomly
num_aggregates = 100
length = 3
width = 3
scatter_range_x = [-width, width]
scatter_range_y = [-length, length]
# scatter_range_x = [-.5, .5]
# scatter_range_y = [-.5, .5]

aggregate_ids = []
for _ in range(num_aggregates):
    x = random.uniform(*scatter_range_x)
    y = random.uniform(*scatter_range_y)
    aggregate_ids.append(create_aggregate([x, y, 0.02]))

# Function to save aggregate positions and orientations to a CSV file
def save_aggregate_data(filename, aggregate_ids):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["ID", "Position_X", "Position_Y", "Position_Z", "Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W"])
        for aggregate_id in aggregate_ids:
            pos, orn = p.getBasePositionAndOrientation(aggregate_id)
            writer.writerow([aggregate_id, *pos, *orn])

# Save initial positions and orientations
save_aggregate_data("aggregates_start.csv", aggregate_ids)

trajectory_type = "random_spline"

# Configuration - random walk
if trajectory_type == "random_walk":
    delta_distance = 0.001  # Distance increment per step (meters)
    delta_angle = math.radians(1)  # Angle increment per step (radians)
    yaw_direction_rand_weights = [0.25, 0.5, 0.25]  # Weights for yaw direction randomization

# create a random path - spline
if trajectory_type == "random_spline":
    start = list(p.getBasePositionAndOrientation(pusher_id)[0])
    end = [length, start[1], start[2]]
    path, orientation = misc.generate_trajectory(start, end, scatter_range_y[1], n_points=10, degree=6)

    # add debug point for each point in path
    colors = [[0.0, 0.0, 0.0] for _ in path]  # Red color as vec3
    p.addUserDebugPoints(path, colors, pointSize=3, lifeTime=0)

step_idx = 0
total_steps = 10_000
# Initialize pusher position and orientation
pusher_position = [0, 0, pusher_height / 2]  # Initial position of the pusher
pusher_yaw = 0.0  # Initial yaw angle (radians)
current_distance = 0.0  # Track distance traveled

# Motion control loop
stop = False
while not stop:
    
    if trajectory_type == "random_walk":        
        pusher_yaw += np.random.choice(
            [-delta_angle, 0, delta_angle], 
            p=yaw_direction_rand_weights
        )
        pusher_orientation = [0, 0, pusher_yaw]
        if step_idx % 500 == 0:
            yaw_direction_rand_weights = np.random.permutation(yaw_direction_rand_weights)
        pusher_position[0] += delta_distance * math.cos(pusher_yaw)
        pusher_position[1] += delta_distance * math.sin(pusher_yaw)
        current_distance += delta_distance
        if step_idx == total_steps:
            stop = True
        
    if trajectory_type == "random_spline":
        # Move along the spline path        
        if step_idx < len(path):
            pusher_position = path[step_idx]
            pusher_orientation = orientation[step_idx]                            
        else:
            pusher_position = path[step_idx-1]
            pusher_orientation = orientation[step_idx-1]
            stop = True

    # Update pusher position and orientation
    pusher_orientation = p.getQuaternionFromEuler(pusher_orientation)
    p.resetBasePositionAndOrientation(pusher_id, pusher_position, pusher_orientation)

    # Step the simulation
    step_idx += 1
    p.stepSimulation()
    time.sleep(1. / 480.)

# Save final positions and orientations
save_aggregate_data("aggregates_end.csv", aggregate_ids)
