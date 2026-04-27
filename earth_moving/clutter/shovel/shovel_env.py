import pybullet as p
import pybullet_data
import numpy as np
from shovel_controller import ShovelController, load_shovel, scatter_pebbles

class ShovelEnvironment:
    def __init__(self, working_area_bounds, gui=True):
        """
        Initialize the PyBullet environment, storing bounds and state.
        """
        self.bounds = working_area_bounds
        self.num_pebbles = 0
        self.pebble_ids = []
        self.robot_id = None
        self.controller = None
        
        # GUI
        self.gui = gui
        self.physics_client = None

    def initialize_environment(self, urdf_path="./urdf/shovel/shovelFlat.urdf", start_pos=None, start_orientation=None):
        """
        Initialize PyBullet simulation environment.

        :param physics_client: Optional existing physics client (default: create new GUI)
        :return: Physics client ID
        """
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        self.robot_id = load_shovel(urdf_path, start_pos, start_orientation)
        self.controller = ShovelController(self.robot_id)
        
        return self.physics_client
    
    def load_pebbles(self, num_pebbles=100, pebble_urdf="./urdf/pebbles/pebbles.urdf",
                   min_pos=None, max_pos=None, settle_steps=120):
        """Loads pebbles into the simulation and stores their IDs."""
        self.pebble_ids = scatter_pebbles(num_pebbles, pebble_urdf, min_pos, max_pos, settle_steps)
        self.remove_out_of_bounds_pebbles()  # Ensure we only keep pebbles that are within bounds after loading
        self.num_pebbles = len(self.pebble_ids)

        return self.pebble_ids

    def remove_out_of_bounds_pebbles(self):
        """Checks self.pebble_ids against self.bounds and removes escaped ones."""
        x_min = self.bounds.get('x_min', -float('inf'))
        x_max = self.bounds.get('x_max', float('inf'))
        y_min = self.bounds.get('y_min', -float('inf'))
        y_max = self.bounds.get('y_max', float('inf'))
        
        active_pebbles = []
        
        for pebble_id in self.pebble_ids:
            try:
                pos, _ = p.getBasePositionAndOrientation(pebble_id)
                x, y, z = pos
                
                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    active_pebbles.append(pebble_id)
                else:
                    p.removeBody(pebble_id)
            except p.error:
                pass
                
        # Update the class variable to only contain pebbles still in bounds
        self.pebble_ids = active_pebbles

    def step(self):
        """Steps the physics simulation and cleans up pebbles."""
        p.stepSimulation()
        self.remove_out_of_bounds_pebbles()

    def close(self):
        """Disconnects the PyBullet session."""
        p.disconnect(self.physics_client)