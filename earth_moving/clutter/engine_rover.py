# this file contains the PyBulletEnvironment class
# the general capabilities are opening the environment, closing the environment, and general control functions

# general imports
import pybullet as p
import pybullet_data
import time as t
import numpy as np
import math

# class definition
class PyBulletEnvironment:
    
    # constructor
    def __init__(self, gui=True, gravity=(0, 0, -10), vacuum_cleaner = False, real_time = True):
        """
        Initialize the PyBullet environment.

        :param gui: Boolean, if True the environment will be opened with a GUI.
        :param gravity: Tuple, the gravity vector.
        """
        
        # gui
        self.gui = gui
        self.physicsClient = None 
        
        # control accuracy threshold        
        self.control_threshold = 1e-2 
        
        # iter limit
        self.iter_limit = 5e2
        
        # ID list 
        self.ID = []
        # Save aggregate IDs seperatly
        self.aggregate_IDs = []
        
        # pybullet options
        self.URDF_MERGE_FIXED_LINKS = True
        self.URDF_USE_INERTIA_FROM_FILE = True
        self.URDF_USE_SELF_COLLISION = True
        self.URDF_USE_SELF_COLLISION_INCLUDE_PARENT = True
        self.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = False
        self.URDF_USE_IMPLICIT_CYLINDER = True
        self.URDF_ENABLE_SLEEPING = False
        self.URDF_INITIALIZE_SAT_FEATURES = False
        self.URDF_USE_MATERIAL_COLORS_FROM_MTL = False
        self.URDF_ENABLE_CACHED_GRAPHICS_SHAPES = False
        self.URDF_MAINTAIN_LINK_ORDER = True
        
        # put in a bitwise OR the previous flags to combine them
        self.flags =    self.URDF_MERGE_FIXED_LINKS | self.URDF_USE_INERTIA_FROM_FILE | self.URDF_USE_SELF_COLLISION | \
                        self.URDF_USE_SELF_COLLISION_INCLUDE_PARENT | self.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS | \
                        self.URDF_USE_IMPLICIT_CYLINDER | self.URDF_ENABLE_SLEEPING | self.URDF_INITIALIZE_SAT_FEATURES | \
                        self.URDF_USE_MATERIAL_COLORS_FROM_MTL | self.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | self.URDF_MAINTAIN_LINK_ORDER
        
        
        # set the gravity
        self.gravity = gravity

        # Set Vacuum Cleaner mode
        self.vacuum_cleaner = vacuum_cleaner

        # Set Real Time competabilty
        self.real_time = real_time


    # open environment
    def open_environment(self, robot_urdf="robot.urdf", init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
        """
        Opens the PyBullet environment with or without GUI based on the initialization parameter.
        """
        
        # set the gui mode
        if self.gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # Non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally set the search path
        
        # gravity
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2])
        
        # add the plane        
        self.ID.append(p.loadURDF("plane.urdf"))
        
        # add the robot
        self.ID.append(p.loadURDF(robot_urdf, init_pos, init_quat))
        self.NumJoints = p.getNumJoints(self.ID[1])
        self.NumLinks = self.NumJoints
        
        
    # load additional URDF file to the environment
    def load_urdf(self, urdf_file, init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
        """
        Loads a URDF file to the environment.

        :param urdf_file: String, the URDF file path.
        :param init_pos: Tuple, the initial position of the object.
        :param init_quat: Tuple, the initial quaternion of the object.
        """
        self.ID.append(p.loadURDF(urdf_file, init_pos, init_quat, flags=self.flags))
    

    def load_aggregates(self, min_pos, max_pos, num_aggregates, urdf_file):
        """
        Loads a specified number of aggregate objects into the PyBullet environment within a given position range.

        :param min_pos: List, the [x, y, z] coordinates of the minimum position for spawning aggregates.
        :param max_pos: List, the [x, y, z] coordinates of the maximum position for spawning aggregates.
        :param num_aggregates: Integer, the number of aggregate objects to load.
        :param urdf_file: String, the URDF file path of the aggregate object to be loaded.
        """
        for i in range (num_aggregates):
    
            # generate init pos
            start_pos = min_pos + (np.random.rand(3) * (max_pos - min_pos))
            start_pos[-1] = 0.2
            
            # set orientation
            start_orientation = p.getQuaternionFromEuler([0,0,0])
            
            # load pebble 
            self.aggregate_IDs.append(p.loadURDF(urdf_file, start_pos, start_orientation, flags=self.flags))
            
        # simulate
        self.simulate(0.5)


    # simulate for a given time
    def simulate(self, time=0.1, step=1/240):
        """
        Simulates the environment for a given time.

        :param time: Float, the simulation time.
        """
        rover_id = self.ID[1]
        for _ in range(int(time/step)):
            p.stepSimulation()
            if self.vacuum_cleaner:
                for aggregate_id in self.aggregate_IDs:
                    contact_points = p.getContactPoints(bodyA=rover_id, bodyB=aggregate_id)
                    if contact_points:  # If contact points exist, a collision has occurred
                        p.removeBody(aggregate_id)
                        self.aggregate_IDs.remove(aggregate_id) 
            
            if self.real_time:
                # If real time needed wait for the enviroment to adjust
                t.sleep(step)    


    # close environment
    def close_environment(self):
        """
        Closes the PyBullet environment.
        """
        p.disconnect()


    def get_top_view(self, pixel_width = 320, pixel_height= 320, camera_target_pos = [0.25, 0, 0],
                     camera_distance = 1.5, fov = 60, aspect_ratioe = 1, near = 0.1, far = 20):
        """
            Captures a top view of the PyBullet environment using a virtual camera.

            :param pixel_width: Integer, the width of the rendered image in pixels.
            :param pixel_height: Integer, the height of the rendered image in pixels. 
            :param camera_target_pos: List, the [x, y, z] coordinates of the target position the camera focuses on.
            :param camera_distance: Float, the distance of the camera from the target position.
            :param fov: Float, the field of view (FOV) of the camera in degrees. 
            :param aspect_ratioe: Float, the aspect ratio of the camera view (width/height).
            :param near: Float, the distance to the near clipping plane. 
            :param far: Float, the distance to the far clipping plane.

            :return: Tuple, containing the rendered RGB image, depth image, and segmentation mask.
        """
        yaw = 0
        pitch = -90.0
        roll = 0
        up_axis_index = 2

        view_matrix = p.computeViewMatrixFromYawPitchRoll(camera_target_pos, camera_distance, yaw, pitch, roll,
                                                            up_axis_index)
        
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect_ratioe, near, far)

        img_arr = p.getCameraImage(pixel_width,
                                    pixel_height,
                                    viewMatrix=view_matrix,
                                    projectionMatrix=projection_matrix,
                                    shadow=1,
                                    lightDirection=[1, 1, 1])
        return img_arr


    def control_rover(self, left_wheel_vel, right_wheel_vel, time):


        rover_id = self.ID[1]
        left_wheel_joint = 0
        right_wheel_joint = 1
        right_wheel_joint_back = 3
        left_wheel_joint_back = 2

        # Set back wheels to move without control
        p.setJointMotorControl2(rover_id, right_wheel_joint_back, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.setJointMotorControl2(rover_id, left_wheel_joint_back, p.VELOCITY_CONTROL, targetVelocity=.0, force=0)

        # Set velocity of front wheels
        p.setJointMotorControl2(rover_id, left_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=left_wheel_vel, force=0.1)
        p.setJointMotorControl2(rover_id, right_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=right_wheel_vel, force=0.1)

        self.simulate(time)



# Example usage
if __name__ == "__main__":
    env = PyBulletEnvironment(gui=True)
    env.open_environment()
    # Perform simulation tasks here
    env.close_environment()