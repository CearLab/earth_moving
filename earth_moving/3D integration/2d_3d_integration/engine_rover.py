# this file contains the PyBulletEnvironment class
# the general capabilities are opening the environment, closing the environment, and general control functions

# general imports
import pickle
import contextlib

# with contextlib.redirect_stdout(None):

import pybullet as p
import pybullet_data

import time as t
import numpy as np
import math
import cv2

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
        self.control_threshold = 1e-3
        self.angle_threshold = 2

        
        # iter limit
        self.iter_limit = 1e4
        
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
        # Set velocity control coefficents
        self.Kp_vel = 0.0001 
        self.Kphi_vel = 0.00001
        self.Kd_phi_vel = 0        
        # Set Lyapunov control coefficents
        self.K1 = 2.0
        self.K2 = 2.0
        self.K3 = 2.0

        self.K1_I = 0
        self.K2_I = 0
        self.K3_I = 0
        # vmax
        self.vmax = 1.0
        self.phimax = 1.0
        
        self.control_dt = 1/240
        self.reached = False
        self.control_iter = 0
        self.ed_i = 0
        self.etheta_i = 0

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


    # def load_urdf(self, urdf_file, init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
    #     """
    #     Loads a URDF file to the environment.

    #     :param urdf_file: String, the URDF file path.
    #     :param init_pos: Tuple, the initial position of the object.
    #     :param init_quat: Tuple, the initial quaternion of the object.
    #     """
    #     self.ID.append(p.loadURDF(urdf_file, init_pos, init_quat, flags=self.flags))
    

    # def load_aggregates(self, min_pos, max_pos, num_aggregates, urdf_file):
    #     """
    #     Loads a specified number of aggregate objects into the PyBullet environment within a given position range.

    #     :param min_pos: List, the [x, y, z] coordinates of the minimum position for spawning aggregates.
    #     :param max_pos: List, the [x, y, z] coordinates of the maximum position for spawning aggregates.
    #     :param num_aggregates: Integer, the number of aggregate objects to load.
    #     :param urdf_file: String, the URDF file path of the aggregate object to be loaded.
    #     """
    #     for i in range (num_aggregates):
    
    #         # generate init pos
    #         start_pos = min_pos + (np.random.rand(3) * (max_pos - min_pos))
    #         start_pos[-1] = 0.2
            
    #         # set orientation
    #         start_orientation = p.getQuaternionFromEuler([0,0,0])
            
    #         # load pebble 
    #         self.aggregate_IDs.append(p.loadURDF(urdf_file, start_pos, start_orientation, flags=self.flags))
            
    #     # simulate
    #     self.simulate(0.5)


    # def load_aggregates_in_clusters(self, min_pos, max_pos, num_clusters, max_per_cluster, max_radius, urdf_file):
    #     """
    #     Loads a specified number of aggregate objects into clusters within a given position range.

    #     :param min_pos: List, the [x, y, z] coordinates of the minimum position for spawning aggregates.
    #     :param max_pos: List, the [x, y, z] coordinates of the maximum position for spawning aggregates.
    #     :param num_clusters: Integer, the number of clusters to create.
    #     :param num_aggregates: Integer, the total number of aggregate objects to load.
    #     :param urdf_file: String, the URDF file path of the aggregate object to be loaded.
    #     """
    #     # Randomly generate the cluster centers within the min_pos and max_pos bounds
    #     cluster_centers = []
    #     for _ in range(num_clusters):
    #         cluster_center = min_pos + (np.random.rand(3) * (max_pos - min_pos))
    #         cluster_centers.append(cluster_center)
                
    #     # Randomly allocate aggregates to each cluster
    #     for center in cluster_centers:
    #         # Assign a random number of aggregates to this cluster, ensuring total sum is num_aggregates
    #         num_cluster_aggregates = np.random.randint(1, max_per_cluster)
            
    #         # Spread the aggregates around the cluster center
    #         for _ in range(num_cluster_aggregates):
    #             # Randomize position around the cluster center within a specified radius
    #             radius = np.random.rand() * max_radius  # Random radius between 0 and 2 meters
    #             angle = np.random.rand() * 2 * np.pi  # Random angle in radians
                
    #             # Generate random offset within the radius
    #             offset_x = radius * np.cos(angle)
    #             offset_y = radius * np.sin(angle)
                
    #             # Calculate the position of the aggregate within the cluster
    #             start_pos = np.array(center) + np.array([offset_x, offset_y, 0.5])
                            
    #             # Set orientation (no rotation)
    #             start_orientation = p.getQuaternionFromEuler([0, 0, 0])
                
    #             # Load the aggregate and store its ID
    #             self.aggregate_IDs.append(p.loadURDF(urdf_file, start_pos, start_orientation, flags=self.flags))
        
    #     # Simulate
    #     self.simulate(0.5)

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

    # def get_top_view(self, pixel_width = 320, pixel_height= 320, camera_target_pos = [1, 0, 0],
    #                  camera_distance = 3, fov = 60, aspect_ratioe = 1, near = 0.1, far = 20):
    #     """
    #         Captures a top view of the PyBullet environment using a virtual camera.

    #         :param pixel_width: Integer, the width of the rendered image in pixels.
    #         :param pixel_height: Integer, the height of the rendered image in pixels. 
    #         :param camera_target_pos: List, the [x, y, z] coordinates of the target position the camera focuses on.
    #         :param camera_distance: Float, the distance of the camera from the target position.
    #         :param fov: Float, the field of view (FOV) of the camera in degrees. 
    #         :param aspect_ratioe: Float, the aspect ratio of the camera view (width/height).
    #         :param near: Float, the distance to the near clipping plane. 
    #         :param far: Float, the distance to the far clipping plane.

    #         :return: Tuple, containing the rendered RGB image, depth image, and segmentation mask.
    #     """
    #     yaw = 0
    #     pitch = -90.0
    #     roll = 0
    #     up_axis_index = 2

    #     view_matrix = p.computeViewMatrixFromYawPitchRoll(camera_target_pos, camera_distance, yaw, pitch, roll,
    #                                                         up_axis_index)
        
    #     projection_matrix = p.computeProjectionMatrixFOV(fov, aspect_ratioe, near, far)

    #     img_arr = p.getCameraImage(pixel_width,
    #                                 pixel_height,
    #                                 viewMatrix=view_matrix,
    #                                 projectionMatrix=projection_matrix,
    #                                 shadow=1,
    #                                 lightDirection=[1, 1, 1])
    #     im = np.array(img_arr[2])
    #     im = im.reshape((img_arr[0],img_arr[1], 4))
    #     im = im[:,:,:3].astype(np.uint8)
    #     im = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    #     return np.expand_dims(im, 2)
    
    def set_positions(self, target_pos, target_quat):
        
        self.reached = False
        self.control_iter = 0
        
        error_vector = []
        while not self.reached and self.control_iter < self.iter_limit:
            # Get current position and orientation
            current_pos, current_quat = self.get_robot_position()
            
            # Compute the target velocities
            target_V, target_phi, e_r = self.compute_target_velocities(target_pos, target_quat, current_pos, current_quat)            
            
            # Compute the target wheel velocities
            omega_r, omega_l = self.compute_wheel_velocities(target_V, target_phi)
            
            # Set velocities
            self.control_rover_velocity(omega_l, omega_r, self.control_dt)          
            
            # flags
            self.control_iter += 1            
            if np.linalg.norm(e_r[:-1]) < self.control_threshold and abs(np.rad2deg(e_r[-1])) < self.angle_threshold:
                self.reached = True
                
            e_r[-1] = np.rad2deg(e_r[-1])
            error_vector.append(e_r)
                
        error_vector = np.array(error_vector).reshape(-1, 3)
        return error_vector, self.control_iter, 
    
    def get_robot_position(self):
        """
        Returns the position of the robot in the environment.

        :return: Tuple, the [x, y, z] coordinates of the robot's position.
        """
        position, orientation = p.getBasePositionAndOrientation(self.ID[1])
        return position, orientation
    
    def compute_target_velocities(self, target_pos, target_quat, current_pos, current_quat):
        
        # Compute position error
        delta_pos = np.array(current_pos) - np.array(target_pos)
        ex = delta_pos[0]
        ey = delta_pos[1]        
        
        # Compute orientation error
        current_theta = p.getEulerFromQuaternion(current_quat)[2]    
        target_theta = p.getEulerFromQuaternion(target_quat)[2]
        etheta = self.normalize_angle(current_theta - target_theta)
        
        # error vector
        e = np.array([ex, ey, etheta])
        
        # R matrix (Tagliacozzi 2.24)
        R = np.array([[+np.cos(target_theta), +np.sin(target_theta), 0],
                      [-np.sin(target_theta), +np.cos(target_theta), 0],
                      [0, 0, 1]])
        e_r = np.dot(R, e)
        
        # change of coordinates (Tagliacozzi 2.24b)
        rho = np.sqrt(e_r[0]**2 + e_r[1]**2)
        # gamma = np.arctan2(e_r[1], e_r[0]) - e_r[2] + np.pi
        # delta = gamma + e_r[2]
        gamma = self.normalize_angle(np.arctan2(e_r[1], e_r[0]) - e_r[2] + np.pi)
        delta = self.normalize_angle(gamma + e_r[2])
        
        # Compute the target velocities (Tagliacozzi 2.29-2.31)
        sigma = 1 if abs(rho) > 1e-4 else 0
        target_V = self.K1 * rho * np.cos(gamma) * sigma
        target_phi = ( (self.K1 * np.sin(gamma) * np.cos(gamma)) / gamma ) * (gamma + self.K3 * delta) + self.K2 * gamma
        
        return target_V, target_phi, e_r
    
    # def set_velocities(self, target_V, target_phi, time, return_errors = False):
        
    #     # Compute the target wheel velocities
    #     omega_r, omega_l = self.compute_wheel_velocities(target_V, target_phi)
        
    #     phi_errors = []
    #     v_errors = []
    #     prev_error_phi = 0
    #     for _ in range(int(time/ self.control_dt)):  # Run for ~4 seconds
            
    #         # control the rover
    #         self.control_rover_velocity(omega_l, omega_r, self.control_dt) 
            
    #         # update errors           
    #         actual_V, actual_phi = self.get_velocities()
    #         # Compute errors
    #         error_V = target_V - actual_V
    #         error_phi = target_phi - actual_phi

    #         # controller (Yaron)
    #         d_error_phi = (error_phi - prev_error_phi) / self.control_dt
    #         delta_omega_r = -self.Kp_vel * error_V - self.Kphi_vel * error_phi - self.Kd_phi_vel * d_error_phi
    #         delta_omega_l = -self.Kp_vel * error_V + self.Kphi_vel * error_phi + self.Kd_phi_vel * d_error_phi
    #         prev_error_phi = error_phi
            
    #         # Update wheel velocities
    #         omega_r += delta_omega_r
    #         omega_l += delta_omega_l
            
    #         # storage
    #         phi_errors.append(error_phi)
    #         v_errors.append(error_V)
        
    #     if return_errors:
    #         return v_errors, phi_errors
        
    def get_joint_index_by_name(self, joint_name):
        for i in range(p.getNumJoints(self.ID[1])):
            info = p.getJointInfo(self.ID[1], i)
            if info[1].decode("utf-8") == joint_name:
                return i
        raise ValueError(f"Joint '{joint_name}' not found")

    def compute_wheel_velocities(self, V,  phi):
            V = np.clip(V, -self.vmax, self.vmax)
            phi = np.clip(phi, -self.phimax, self.phimax)
            omega_r = - (2 * V + phi * self.L) / (2 * self.R)
            omega_l = - (2 * V - phi * self.L) / (2 * self.R)
            return omega_r, omega_l
        
    def normalize_angle(self, angle):        
        angle_normalized = (angle + np.pi) % (2 * np.pi) - np.pi        
        return angle_normalized
        # return angle

    def control_rover_velocity(self, left_wheel_vel, right_wheel_vel, time = 1/240):
        rover_id = self.ID[1]
        left_wheel_joint = self.get_joint_index_by_name("base_to_lwheel")
        right_wheel_joint = self.get_joint_index_by_name("base_to_rwheel")


        p.changeDynamics(rover_id, right_wheel_joint, lateralFriction=10.0)
        p.changeDynamics(rover_id, left_wheel_joint, lateralFriction=10.0)
        p.changeDynamics(1, 0, lateralFriction=0.0)

        # Set velocity of front and back wheels
        p.setJointMotorControl2(rover_id, left_wheel_joint, p.VELOCITY_CONTROL, 
                                targetVelocity=left_wheel_vel, force=10)
        p.setJointMotorControl2(rover_id, right_wheel_joint, p.VELOCITY_CONTROL,
                                 targetVelocity=right_wheel_vel, force=10)
        print(f"Set omega_l={left_wheel_vel:.3f}, omega_r={right_wheel_vel:.3f}")  # <-- Add here

        self.simulate(time)

    def get_velocities(self):
        # Get feedback from the base
        linear_velocity_world, angular_velocity_world = p.getBaseVelocity(self.ID[1])
        position, orientation = p.getBasePositionAndOrientation(self.ID[1])
        
        # Convert quaternion to rotation matrix
        rotation_matrix = p.getMatrixFromQuaternion(orientation)
        rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
        
        # Transform world-space linear velocity to local-space linear velocity
        linear_velocity_local = np.dot(rotation_matrix.T, linear_velocity_world)
        vx_local, vy_local, _ = linear_velocity_local
        
        # Forward velocity (V) in local frame is along x-axis
        actual_V = vx_local
        
        # Angular velocity (phi) remains the z-component in local frame
        actual_phi = angular_velocity_world[2]  # Angular velocity is already in local frame for Z-axis
        return actual_V, actual_phi
    
    def set_robot_dim(self, L = 0.2, R = 0.07):
        self.L = L
        self.R = R

    # def save_env_state_to_file(self, file_name):
    #     # Get the number of objects in the environment
    #     # Create a dictionary to store the state information
    #     env_state = {}

    #     # Save object states
    #     env_state["ID"] = self.ID
    #     env_state["ID_object_positions"] = []
    #     env_state["ID_object_orientations"] = []
    #     env_state["ID_object_velocities"] = []
        
    #     env_state["aggregate_IDs"] = self.aggregate_IDs
    #     env_state["aggreagets_object_positions"] = []
    #     env_state["aggreagets_object_orientations"] = []
    #     env_state["aggreagets_object_velocities"] = []
    #     for id in self.ID[1:]:
    #         position, orientation = p.getBasePositionAndOrientation(id)
    #         velocity = p.getBaseVelocity(id)
    #         env_state["ID_object_positions"].append(position)
    #         env_state["ID_object_orientations"].append(orientation)
    #         env_state["ID_object_velocities"].append(velocity)

    #     for id in self.aggregate_IDs:
    #         position, orientation = p.getBasePositionAndOrientation(id)
    #         velocity = p.getBaseVelocity(id)
    #         env_state["aggreagets_object_positions"].append(position)
    #         env_state["aggreagets_object_orientations"].append(orientation)
    #         env_state["aggreagets_object_velocities"].append(velocity)

    #     # Save the environment settings (e.g., gravity)
    #     env_state["gravity"] = self.gravity
    #     # Save to a file using pickle
    #     with open(file_name, 'wb') as f:
    #         pickle.dump(env_state, f)

    # def get_num_aggregates(self):
    #     return len(self.aggregate_IDs)
    
    # def restore_env_state_from_file(self, file_name, robot_urdf, aggregates_urdf):
    #     # Load the saved state from the file
        
    #     if self.gui:
    #         self.physicsClient = p.connect(p.GUI)
    #     else:
    #         self.physicsClient = p.connect(p.DIRECT)  # Non-graphical version
    #     p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally set the search path
        
    #     # gravity
        
    #     with open(file_name, 'rb') as f:
    #         env_state = pickle.load(f)

    #     # Restore gravity
    #     p.setGravity(*env_state["gravity"])
    #     self.ID = env_state["ID"]
    #     p.loadURDF("plane.urdf")
    #     for id in self.ID[1:]:
    #         p.loadURDF(robot_urdf, env_state["ID_object_positions"][id-1],
    #                    env_state["ID_object_orientations"][id-1])
    #     # Restore objects
    #     self.aggregate_IDs = env_state["aggregate_IDs"]
    #     for ind in range(len(self.aggregate_IDs)):
    #         p.loadURDF(aggregates_urdf, env_state["aggreagets_object_positions"][ind],
    #                    env_state["aggreagets_object_orientations"][ind])
    #     self.simulate(0.5)

    # def execute_and_record(self, commands, video_path, mode='position'):
    #     """
    #     Executes a list of (target_v, target_phi, time) commands, records a top-view video, 
    #     and saves it to the specified path.
        
    #     :param commands: List of tuples (target_v, target_phi, time) to execute.
    #     :param video_path: Path to save the recorded video.
    #     """
    #     width, height = 320, 320  # Adjust based on your simulation window size
    #     fps = 30
    #     time_step = 1/fps
    #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #     out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    #     if mode == 'velocity':            
    #         for target_v, target_phi, duration in commands:
    #             for i in np.arange(0,duration,time_step):
    #                 self.set_velocities(target_v, target_phi, time_step)
    #                 top_view = self.get_top_view( pixel_width = 320, pixel_height= 320)  # Assuming a function returning the top view
    #                 if top_view is not None:
    #                     out.write(cv2.cvtColor(top_view, cv2.COLOR_RGB2BGR))
    #     elif mode == 'position':
    #         for target_pos, target_quat, duration in commands:
    #             error_vector, control_iter = self.set_positions(target_pos, target_quat)
    #             for i in np.arange(0,duration,time_step):
    #                 top_view = self.get_top_view(pixel_width = 320, pixel_height= 320, \
    #                     camera_target_pos=[0, 0, 0], camera_distance=5)
    #                 if top_view is not None:
    #                     out.write(cv2.cvtColor(top_view, cv2.COLOR_RGB2BGR))
    #     else:
    #         raise ValueError("Invalid mode. Use 'position' or 'velocity'.")
        
    #     out.release()
    
    # def go_to_pose_smooth(self, start_pose, end_pose, duration=2.0, step_time=0.2, scale=0.5):
    #     """
    #     Generate and follow a smooth trajectory using cubic Hermite spline between start and end pose.

    #     :param start_pose: (x0, y0, theta0)
    #     :param end_pose: (x1, y1, theta1)
    #     :param duration: Total trajectory time
    #     :param step_time: Time between waypoints
    #     :param scale: Scalar multiplier for tangent length (affects curve shape)
    #     """
    #     x0, y0, theta0 = start_pose
    #     x1, y1, theta1 = end_pose

    #     # Tangent vectors at start and end based on heading angles
    #     dx0, dy0 = scale * np.cos(theta0), scale * np.sin(theta0)
    #     dx1, dy1 = scale * np.cos(theta1), scale * np.sin(theta1)

    #     def hermite_interp(p0, p1, m0, m1, t):
    #         h00 = 2*t**3 - 3*t**2 + 1
    #         h10 = t**3 - 2*t**2 + t
    #         h01 = -2*t**3 + 3*t**2
    #         h11 = t**3 - t**2
    #         return h00*p0 + h10*m0 + h01*p1 + h11*m1

    #     num_steps = int(duration / step_time)
    #     trajectory = []

    #     for i in range(num_steps):
    #         t = i / (num_steps - 1)
    #         x = hermite_interp(x0, x1, dx0, dx1, t)
    #         y = hermite_interp(y0, y1, dy0, dy1, t)

    #         # Compute tangent (dx/dt, dy/dt) and convert to heading
    #         dt = 1e-5
    #         x_next = hermite_interp(x0, x1, dx0, dx1, min(t + dt, 1.0))
    #         y_next = hermite_interp(y0, y1, dy0, dy1, min(t + dt, 1.0))
    #         heading = np.arctan2(y_next - y, x_next - x)

    #         trajectory.append((x, y, heading))

    #     self.draw_trajectory(trajectory)
    #     self.follow_trajectory(trajectory, step_time=step_time)



    import numpy as np

    def generate_bezier_trajectory(self, start_pose, end_pose, duration=3.0, step_time=0.1, scale=0.5):
        
        """
        Generate a smooth cubic Bézier trajectory from start_pose to end_pose.
        :param start_pose: (x0, y0, theta0)
        :param end_pose: (x1, y1, theta1)
        :param duration: total path duration in seconds
        :param step_time: time step for sampling
        :param scale: distance along heading for control points
        :return: list of (x, y, heading) waypoints
        """
        x0, y0, theta0 = start_pose
        x3, y3, theta3 = end_pose

        # Control points for cubic Bézier
        p0 = np.array([x0, y0])
        p1 = p0 + scale * np.array([np.cos(theta0), np.sin(theta0)])
        p3 = np.array([x3, y3])
        p2 = p3 - scale * np.array([np.cos(theta3), np.sin(theta3)])

        num_steps = int(duration / step_time)
        trajectory = []

        for i in range(num_steps):
            t = i / (num_steps - 1)
            # Cubic Bézier formula
            point = (
                (1 - t) ** 3 * p0 +
                3 * (1 - t) ** 2 * t * p1 +
                3 * (1 - t) * t ** 2 * p2 +
                t ** 3 * p3
            )
            # Tangent (derivative of Bézier curve)
            dp_dt = (
                3 * (1 - t) ** 2 * (p1 - p0) +
                6 * (1 - t) * t * (p2 - p1) +
                3 * t ** 2 * (p3 - p2)
            )
            heading = np.arctan2(dp_dt[1], dp_dt[0])
            trajectory.append((point[0], point[1], heading))

        return trajectory

    
    def follow_smooth_trajectory(self, trajectory, control_dt=1/240, lookahead=1):
        """
        Continuously tracks a trajectory using closed-loop control (like set_positions).
        Uses current pose to compute V and phi toward the next target.

        :param trajectory: list of (x, y, theta) poses
        :param control_dt: timestep between control updates
        :param lookahead: number of steps to skip ahead (used for smoothing)
        """
        reached = False
        i = 0
        max_iter = len(trajectory) * 100

        while not reached and i < max_iter:
            current_pos, current_quat = self.get_robot_position()
            current_index = min(i // 20, len(trajectory) - 1)
            target_index = min(current_index + lookahead, len(trajectory) - 1)
            target_pose = trajectory[target_index]
            target_pos = (target_pose[0], target_pose[1], 0)
            target_quat = p.getQuaternionFromEuler([0, 0, target_pose[2]])

            # Compute velocities
            target_V, target_phi, _ = self.compute_target_velocities(
                target_pos, target_quat, current_pos, current_quat
            )

            # Convert to wheel commands
            omega_r, omega_l = self.compute_wheel_velocities(target_V, target_phi)

            # Apply control
            self.control_rover_velocity(omega_l, omega_r, control_dt)
            i += 1

            # Check if reached final point
            final_pos = trajectory[-1]
            pos_error = np.linalg.norm(np.array(current_pos[:2]) - np.array(final_pos[:2]))
            if pos_error < self.control_threshold:
                reached = True

    
    def follow_trajectory(self, trajectory, step_time=0.1):
        """
        Follows a continuous trajectory consisting of (x, y, theta) tuples.

        :param trajectory: List of waypoints (x, y, theta in radians).
        :param step_time: Time to spend on each segment.
        """
        for i in range(len(trajectory) - 1):
            start = trajectory[i]
            end = trajectory[i + 1]

            # Calculate desired linear and angular velocity
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dtheta = self.normalize_angle(end[2] - start[2])

            distance = np.hypot(dx, dy)
            linear_velocity = distance / step_time
            angular_velocity = dtheta / step_time

            # Clamp to max velocity limits
            linear_velocity = np.clip(linear_velocity, -self.vmax, self.vmax)
            angular_velocity = np.clip(angular_velocity, -self.phimax, self.phimax)

            self.set_velocities(linear_velocity, angular_velocity, step_time)

    # def follow_trajectory_pose_control(self, trajectory, step_time=0.2):
    #     """
    #     Follows a trajectory using pose-level control (like set_positions).
    #     Each (x, y, theta) waypoint is treated as a target pose.
        
    #     :param trajectory: List of waypoints (x, y, theta)
    #     :param step_time: Time to wait (simulate) after each pose correction
    #     """
    #     for x, y, theta in trajectory:
    #         target_quat = p.getQuaternionFromEuler([0, 0, theta])
    #         self.set_positions((x, y, 0), target_quat)  # Z=0 flat ground
    #         self.simulate(step_time)

    # def generate_hermite_trajectory(self, start_pose, end_pose, duration=3.0, step_time=0.1, scale=0.5):
    #     """
    #     Generate a Hermite trajectory from start to end pose.

    #     :param start_pose: (x0, y0, theta0)
    #     :param end_pose: (x1, y1, theta1)
    #     :param duration: Total duration of the path
    #     :param step_time: Step size in seconds
    #     :param scale: Tangent magnitude scale
    #     :return: List of (x, y, theta) waypoints
    #     """
    #     x0, y0, theta0 = start_pose
    #     x1, y1, theta1 = end_pose

    #     dx0, dy0 = scale * np.cos(theta0), scale * np.sin(theta0)
    #     dx1, dy1 = scale * np.cos(theta1), scale * np.sin(theta1)

    #     def hermite_interp(p0, p1, m0, m1, t):
    #         h00 = 2*t**3 - 3*t**2 + 1
    #         h10 = t**3 - 2*t**2 + t
    #         h01 = -2*t**3 + 3*t**2
    #         h11 = t**3 - t**2
    #         return h00*p0 + h10*m0 + h01*p1 + h11*m1

    #     num_steps = int(duration / (2*step_time))
    #     trajectory = []

    #     for i in range(num_steps):
    #         t = i / (num_steps - 1)
    #         x = hermite_interp(x0, x1, dx0, dx1, t)
    #         y = hermite_interp(y0, y1, dy0, dy1, t)

    #         # Estimate heading
    #         dt = 1e-5
    #         x_next = hermite_interp(x0, x1, dx0, dx1, min(t + dt, 1.0))
    #         y_next = hermite_interp(y0, y1, dy0, dy1, min(t + dt, 1.0))
    #         heading = np.arctan2(y_next - y, x_next - x)

    #         trajectory.append((x, y, heading))

    #     return trajectory

    def draw_trajectory(self, trajectory, color=[1, 0, 0], life_time=0):
        """
        Draws a trajectory in PyBullet using debug lines.
        
        :param trajectory: List of (x, y, theta) waypoints
        :param color: RGB list, default red
        :param life_time: Duration to keep the lines (0 = forever)
        """
        for i in range(len(trajectory) - 1):
            x0, y0, _ = trajectory[i]
            x1, y1, _ = trajectory[i + 1]
            p.addUserDebugLine(
                [x0, y0, 0.05],  # Start point
                [x1, y1, 0.05],  # End point
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=life_time
            )



if __name__ == "__main__":
    env = PyBulletEnvironment(gui=True)
    env.open_environment()
    # Perform simulation tasks here
    env.close_environment()