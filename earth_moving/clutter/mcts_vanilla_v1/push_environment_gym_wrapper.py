"""
Gymnasium wrapper for PushEnvironment without changing the core logic.

This creates a gym.Env interface around your existing PushEnvironment,
making it compatible with any RL algorithm that works with Gymnasium.
"""

import gymnasium as gym
# import gym as old_gym
import numpy as np
from gymnasium import spaces
from push_environment_v1_upd import PushEnvironment


class PushEnvironmentGym(gym.Env):
    """
    Gymnasium wrapper for PushEnvironment.
    
    This preserves all your original logic while providing a standard gym interface.
    The observation is just the actual state dictionary from your environment.
    """
    
    def __init__(self, 
                 boundary_points, 
                 delta_s_options, 
                 theta_options, 
                 push_width, 
                 initial_pos, 
                 max_pushes, 
                 coverage_thresh,
                 occupancy_grid,
                 total_occupied,
                 total_pixels_in_cell=0.0,
                 stochasticity=False,
                 grid_size=20):
        """Initialize with same parameters as your original PushEnvironment."""
        super().__init__()
        
        # Create your original environment - NO CHANGES to logic
        self.push_env = PushEnvironment(
            boundary_points=boundary_points,
            delta_s_options=delta_s_options,
            theta_options=theta_options,
            push_width=push_width,
            initial_pos=initial_pos,
            max_pushes=max_pushes,
            coverage_thresh=coverage_thresh,
            occupancy_grid=occupancy_grid,
            total_occupied=total_occupied,
            total_pixels_in_cell=total_pixels_in_cell,
            stochasticity=stochasticity,
            grid_size=grid_size
        )

        self.initial_state = self.push_env.initial_state
        # Define Gymnasium action space (discrete actions)
        # Always use the fallback calculation to ensure we have a proper Discrete space
        num_actions = len(delta_s_options) * len(theta_options)

        self.action_space = spaces.Discrete(num_actions)
        
        # Define observation space - we'll use a Dict space to match your state structure
        self.observation_space = spaces.Dict({
            's_idx': spaces.Discrete(len(boundary_points)),
            'grid_mask': spaces.Box(low=0, high=1, shape=(grid_size, grid_size), dtype=np.bool_),
            'num_pushes': spaces.Discrete(max_pushes + 1),
            'length': spaces.Box(low=0, high=np.inf, shape=(), dtype=np.float32),
            'len_pushes': spaces.Box(low=0, high=np.inf, shape=(), dtype=np.float32),
            'len_path': spaces.Box(low=0, high=np.inf, shape=(), dtype=np.float32),
        })
        
        # Store metadata
        self.metadata = {'render_modes': ['human']}

        
    def reset(self, seed=None, options=None):
        """Reset environment and return observation."""
        if seed is not None:
            np.random.seed(seed)
            
        # Use your original reset method
        state = self.push_env.reset()
        
        # The observation IS the state - no conversion needed
        observation = state
        info = {}
        
        return observation, info
    
    def step(self, action):
        """Take a step using your original step method."""
        # Use your original step method - NO CHANGES
        state, reward, done, info = self.push_env.step(action)
        
        # The observation IS the state
        observation = state
        terminated = done
        truncated = False  # Your environment handles termination internally
        
        return observation, reward, terminated, truncated, info
    
    def render(self, mode='human'):
        """Render using your original methods."""
        if mode == 'human':
            # Use your existing render logic
            state = self.push_env.current_state
            coverage = self.push_env.compute_coverage(state['grid_mask'])
            print(f"Position: {state['s_idx']}, Coverage: {coverage:.3f}, "
                  f"Pushes: {state['num_pushes']}/{self.push_env.max_pushes}")
            
            # You can also call your visualization methods here:
            # particles = self.push_env.grid_to_particles(state['grid_mask'])
            # self.push_env.draw_grid_environment(state['grid_mask'])
    
    def close(self):
        """Close the environment."""
        pass

    def get_copy(self):
        """Create independent copy for MCTS simulation.
        Only copies mutable state; shares immutable data for speed.
        """
        new_wrapper = object.__new__(PushEnvironmentGym)
        
        # Copy the inner push_env using its optimized get_copy()
        new_wrapper.push_env = self.push_env.get_copy()
        
        # Share immutable wrapper attributes (no copy needed)
        new_wrapper.initial_state = self.initial_state
        new_wrapper.action_space = self.action_space
        new_wrapper.observation_space = self.observation_space
        new_wrapper.metadata = self.metadata
        
        return new_wrapper

    def get_available_actions(self):
        """Get the available actions for the current state."""
        # return range(self.action_space.n)
        return self.push_env.get_available_actions()

    # Expose your original methods for direct access if needed
    def get_push_env(self):
        """Get the underlying PushEnvironment for direct access."""
        return self.push_env
    
    def get_refined_actions(self, coarse_action):
        """Get refined actions from the underlying environment."""
        return self.push_env.get_refined_actions(coarse_action)
    
    def compute_coverage(self, grid_mask):
        """Compute current coverage using your original method."""
        return self.push_env.compute_coverage(grid_mask)
    
    def transform_s_to_normed_world(self, s_idx):
        """ Get the boundary point [x,y] corresponding to s_idx in normalized world coordinates [0,1]^2 """
        return self.push_env.transform_s_to_normed_world(s_idx)
    
    def transform_normed_world_to_global(self, point):
        """ Convert a point from normalized world coordinates [0,1]^2 to global world coordinates using the bounding box size and offset """
        return self.push_env.transform_normed_world_to_global(point)
    
    def find_boundry_corners_in_ds_path(self, prev_s_idx, delta_s):
        """ Given a push from prev_s_idx to new_s_idx = (prev_s_idx + delta_s) % boundary_len, find the corner points if the path crosses corners """
        return self.push_env.find_boundry_corners_in_ds_path(prev_s_idx, delta_s)
    
    def transform_push_to_normed_world(self, s_idx, delta_s, theta):
        """Transform push to normalized world coordinates using your original method."""
        return self.push_env.transform_push_to_normed_world(s_idx, delta_s, theta)
    
    def transform_push_to_global_world(self, s_idx, delta_s, theta):
        """Transform push to global world coordinates using your original method."""
        return self.push_env.transform_push_to_global_world(s_idx, delta_s, theta)
    
    def draw_environment(self, particles, pushes=[], current_push=None):
        """Use your original visualization."""
        self.push_env.draw_environment(particles, pushes, current_push)
    
    def draw_real_environment(self, particles, pushes=[], current_push=None):
        """Use your original visualization."""
        self.push_env.draw_real_environment(particles, pushes, current_push)
    
    def animate_grid_push(self, action):
        """Use your original grid visualization."""
        self.push_env.animate_grid_push(action)
    
    def animate_transition_push(self, action):
        """Use your original grid visualization."""
        return self.push_env.animate_transition_push(action)
    
    def draw_grid_environment(self, grid_mask, pushes=[], current_push=None):
    # def draw_grid_environment(self, grid_mask, ax, current_push=None):
        """Use your original grid visualization."""
        self.push_env.draw_grid_environment(grid_mask, pushes, current_push)
        # self.push_env.draw_grid_environment(grid_mask, ax, current_push)

    def plot_pushes_original_space(self, pushes, boundary_points, rock_positions, shovel_width, title="Push Rectangles"):
        """Plot pushes in original space using your original method."""
        self.push_env.plot_pushes_original_space(pushes, boundary_points, rock_positions, shovel_width, title)


def test_gymnasium_environment():
    """Test the Gymnasium environment."""
    print("🧪 Testing Gymnasium Environment")
    print("=" * 40)
    
    # Create environment exactly like your original
    np.random.seed(42)
    particles = np.random.rand(10, 2) * 0.6 + 0.2
    boundary_points = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    
    # Create Gymnasium environment - same parameters as your original
    env = PushEnvironmentGym(
        particles=particles,
        boundary_points=boundary_points,
        delta_s_options=[-1, 0, 1],
        theta_options=[0, np.pi/4, np.pi/2],
        push_width=0.15,
        initial_pos=0,
        max_pushes=4,
        coverage_thresh=0.9,
        grid_size=12
    )
    
    print(f"✅ Gymnasium environment created")
    print(f"Action space: {env.action_space}")
    print(f"Observation space: {type(env.observation_space)}")
    
    # Test standard Gymnasium interface
    obs, info = env.reset()
    print(f"✅ Reset - observation keys: {obs.keys()}")
    print(f"   Coverage: {env.compute_coverage():.3f}")
    
    # Test step
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"✅ Step - action: {action}, reward: {reward:.4f}, done: {terminated}")
    print(f"   New coverage: {env.compute_coverage():.3f}")
    
    # Test render
    print("✅ Render:")
    env.render()
    
    # Test with MCTS wrapper (import from separate file)
    try:
        from mcts_wrapper import PushEnvironmentMCTSWrapper
        print("\n🚀 Testing MCTS Compatibility")
        mcts_game = PushEnvironmentMCTSWrapper(env)
        mcts_obs = mcts_game.reset()
        print(f"✅ MCTS wrapper - observation type: {type(mcts_obs)}")
        
        # Test copying
        game_copy = mcts_game.get_copy()
        print("✅ MCTS deep copy successful")
    except ImportError:
        print("⚠️ MCTS wrapper not available (mcts_wrapper.py not found)")
    
    env.close()
    print("\n🎉 All tests passed!")


if __name__ == "__main__":
    test_gymnasium_environment()
