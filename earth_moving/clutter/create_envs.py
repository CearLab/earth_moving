import numpy as np
import random
import os
from engine_rover import *
from tqdm import tqdm

import pybullet as p
# Constants
NUM_OF_ENVS = 100
OUTPUT_FOLDER = "./environments"
ROBOT_URDF = './urdf/shovel/rover_with_shovel.urdf'
AGGREGATE_URDF = './urdf/pebbles/pebbles.urdf'
MIN_POS = np.array([0.2, -1.5, 0])
MAX_POS = np.array([3, 1.5, 0])

# Ensure the output folder exists
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

for env_index in tqdm(range(NUM_OF_ENVS)):
    # Initialize environment
    engine = PyBulletEnvironment(gui=False, vacuum_cleaner=True, real_time=True)

    # Load rover
    start_pos = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    engine.open_environment(ROBOT_URDF, start_pos, start_orientation)

    # Decide how to load aggregates (80% clusters, 20% individual)
    if random.random() < 0.8:
        # Load aggregates in clusters
        num_clusters = random.randint(15, 50)
        max_per_cluster = random.randint(20, 60)
        max_radius = random.uniform(0.2, 0.7)
        engine.load_aggregates_in_clusters(MIN_POS, MAX_POS, num_clusters, max_per_cluster, max_radius, AGGREGATE_URDF)
    else:
        # Load aggregates individually
        num_aggregates = random.randint(100, 700)
        engine.load_aggregates(MIN_POS, MAX_POS, num_aggregates, AGGREGATE_URDF)

    # Save environment state
    file_name = os.path.join(OUTPUT_FOLDER, f"env_{env_index}.pickle")
    engine.save_env_state_to_file(file_name)
    engine.close_environment()

print(f"{NUM_OF_ENVS} environments created and saved in {OUTPUT_FOLDER}.")
