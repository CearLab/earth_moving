# import gymnasium as gym
# from gymnasium import spaces
import numpy as np
import gym
from gym import spaces
from gym.envs.registration import register
# from gymnasium.envs.registration import register
import contextlib


# with contextlib.redirect_stdout(None):
from engine_rover import PyBulletEnvironment
import pybullet as p

from glob import glob

# import pybullet as p

ROBOT_URDF = '/home/yaron/Projects/earth_moving/earth_moving/earth_moving/clutter/urdf/shovel/rover_with_shovel.urdf'
AGGREGATE_URDF = '/home/yaron/Projects/earth_moving/earth_moving/earth_moving/clutter/urdf/pebbles/pebbles.urdf'

class RoverEnvVacuum(gym.Env):
    metadata = {'render.modes': ['rgb_array']}  # Supported render modes
    envs_list = glob("/home/yaron/Projects/earth_moving/earth_moving/earth_moving/clutter/environments/*.pickle")
    action_mapping = {0 : [0,0], 1: [0.05, 0], 2: [0.05, 0.1], 3: [0.05, 0.3],
                      4: [0.2, 0], 5: [0.2, 0.1], 6: [0.2, 0.3], 7: [0, 0.1], 8: [0, 0.3]}
    dt = 3
    def __init__(self):
        super(RoverEnvVacuum, self).__init__()
        # Define action space (discrete example: move left or right)
        self.action_space = spaces.Discrete(9)
        
        # Define observation space (continuous example)
        self.observation_space = spaces.Box(low=0, high=255, shape=(320,320,1), dtype=np.uint8)
        random_env_ind = np.random.randint(0, len(self.envs_list))
        self.engine = PyBulletEnvironment(gui=False, vacuum_cleaner=True, real_time=False)
        self.engine.restore_env_state_from_file(self.envs_list[random_env_ind], ROBOT_URDF, AGGREGATE_URDF)
        self.num_aggregates = self.engine.get_num_aggregates()
        self.engine.set_robot_dim()

    def reset(self):
        """Resets the environment to the initial state."""

        # super().reset()
        p.resetSimulation()
        random_env_ind = np.random.randint(0, len(self.envs_list))
        self.engine = PyBulletEnvironment(gui=False, vacuum_cleaner=True, real_time=False)
        self.engine.restore_env_state_from_file(self.envs_list[random_env_ind], ROBOT_URDF, AGGREGATE_URDF)
        self.num_aggregates = self.engine.get_num_aggregates()
        self.engine.set_robot_dim()
        return self.engine.get_top_view()

    def step(self, action):
        """Performs a step based on the given action."""
        # Update the state based on action
        target_V , target_phi = self.action_mapping[action]
        self.engine.set_velocities(target_V, target_phi, self.dt)
        reward = self.num_aggregates - self.engine.get_num_aggregates()
        self.num_aggregates = self.engine.get_num_aggregates()
        observation = self.engine.get_top_view()
        return observation, reward, self.num_aggregates == 0, {}  # Observation, reward, done, inf

    def render(self, mode='human'):
        """Renders the environment (e.g., print state)."""
        return self.engine.get_top_view(), {}

    def close(self):
        """Clean up resources (optional)."""
        self.engine.close_environment()

# Register the custom environment
register(
    id='RoverEnvVacuum-v0',  # Unique identifier
    entry_point='gym_env:RoverEnvVacuum',  # Path to the class
)

# Test the environment
if __name__ == "__main__":
    env = gym.make('RoverEnvVacuum-v0')
    state = env.reset()
    done = False
    ind = 0
    while not done and ind<4:
        action = env.action_space.sample()
        state, reward, done, info = env.step(action)
        env.render()
        print(reward)
        ind += 1
    env.close()