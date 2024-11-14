# this file contains the PyBulletEnvironment class
# the general capabilities are opening the environment, closing the environment, and general control functions

# general imports
import pybullet as p
import pybullet_data
import time as t
import numpy as np
import math
from earth_moving.clutter.base_simulation_engine import BaseSimulationEngine

# class definition2
# class PyBulletEnvironment:
class PyBulletEnvironment(BaseSimulationEngine):
    
    # constructor
    def __init__(self, gui=True, gravity=(0, 0, -10)):
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
        self.Pos_gains = [1e-3, 1e-3, 1e-3]
        self.Vel_gains = [1e-3, 1e-3, 1e-3]
        self.Target_velocities = [0.01, 0.2, 0.2]
        self.maxForce = [1e3, 1e3, 1e3]
        
        # iter limit
        self.iter_limit = 5e2
        
        # ID list 
        self.ID = []
        
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
        
    # simulate for a given time
    def simulate(self, time=0.1, step=1/240):
        """
        Simulates the environment for a given time.

        :param time: Float, the simulation time.
        """
        for _ in range(int(time/step)):
            p.stepSimulation()
            t.sleep(step)    

    # close environment
    def close_environment(self):
        """
        Closes the PyBullet environment.
        """
        p.disconnect()
        
    # control a joint
    def control_joint(self, joint_target, control_mode=p.VELOCITY_CONTROL):
        # if control_joint == ControlMode.VELOCITY_CONTROL:
            # pybullet_velocty_control = p.
        """
        Controls a joint of the robot.

        :param joint_index: Integer, the index of the joint.
        :param joint_target: Float, the desired position or velocity of the joint.
        :param control_mode: Integer, the control mode (p.POSITION_CONTROL or p.VELOCITY_CONTROL).
        """
        
        # Njoints
        Njoints = p.getNumJoints(self.ID[1])
        JointList = range(Njoints)
        
        # control the joint
        if control_mode == p.POSITION_CONTROL:
            p.setJointMotorControlArray(self.ID[1], \
            JointList, \
            control_mode, \
            targetPositions=joint_target, \
            targetVelocities=self.Target_velocities, \
            forces=self.maxForce, \
            # positionGains=self.Pos_gains, \
            # velocityGains=self.Vel_gains
            )
            # if you want you can play also with 
        elif control_mode == p.VELOCITY_CONTROL:
            p.setJointMotorControlArray(self.ID[1], \
            JointList, \
            control_mode, \
            targetVelocities=joint_target, \
            forces=self.maxForce)
            
    # compute the error of the joint with respect to a target
    def compute_joint_error(self, joint_index, target):
        """
        Computes the error of a joint with respect to a target.

        :param joint_index: Integer, the index of the joint.
        :param target: Float, the target value.
        :return: Float, the error.
        """
        joint_state = p.getJointState(self.ID[1], joint_index)
        return target - joint_state[0]
    
    # control all the joints to a target position defined by an array
    def control_all_joints(self, joint_targets, control_mode=p.VELOCITY_CONTROL):
        """
        Controls all the joints of the robot.

        :param joint_targets: List, the desired positions or velocities of the joints.
        :param control_mode: Integer, the control mode (p.POSITION_CONTROL or p.VELOCITY_CONTROL).
        """
        
        # number of joints
        Njoints = p.getNumJoints(self.ID[1])
        
        # if it is velocity controlled we have the error computation and the control loop
        if control_mode == p.VELOCITY_CONTROL:
            
            # error array
            e = np.zeros(Njoints)        
            
            # error init
            for i in range(Njoints):
                
                # error init    
                e[i] = self.compute_joint_error(i,joint_targets[i])
                
            # reach flag array
            reach = np.zeros(Njoints)
            
            # iter count
            iter = 0
            
            # control loop
            while np.any(reach == 0) and iter < self.iter_limit: 
                
                # iter update
                iter = iter + 1
                
                # init target velocities
                targetVelCtrl = np.zeros(Njoints)
                
                # cycle joints
                for i in range(Njoints):
                    
                    if np.abs(e[i]) > self.control_threshold and reach[i] == 0:
                        targetVelCtrl[i] = self.Vel_gains[i]*np.sign(e[i])                        
                    else:
                        targetVelCtrl[i] = 0.0
                        reach[i] = 1                                            
                        
                    # error update
                    e[i] = self.compute_joint_error(i,joint_targets[i])
                    
                self.control_joint(targetVelCtrl, control_mode)
                    
                # simulate
                self.simulate()
                    
                # warning on iterations
                if iter >= self.iter_limit:
                    print("Warning: Iteration limit reached")
                        
        # if we are in position control, we just give the position
        elif control_mode == p.POSITION_CONTROL:            
            
            # control
            self.control_joint(joint_targets, control_mode)
                
            # simulate
            self.simulate()
            
        # something went wrong
        else:
            print("Error: Control mode not recognized")

    # give a set of waypoints, control the robot to reach them
    def control_waypoints(self, waypoints, control_mode=p.VELOCITY_CONTROL):
        """
        Controls the robot to reach a set of waypoints.

        :param waypoints: List, the desired waypoints.
        :param control_mode: Integer, the control mode (p.POSITION_CONTROL or p.VELOCITY_CONTROL).
        """
        
        # number of waypoints
        Nwaypoints = len(waypoints)
        
        # cycle waypoints
        for i in range(Nwaypoints):
            
            # test to control the EE position
            joint_pos = p.calculateInverseKinematics(self.ID[1], self.NumLinks-1, waypoints[i], maxNumIterations=1000, residualThreshold=1e-3)
            
            # control the robot to reach the waypoint
            self.control_all_joints(joint_pos, control_mode)
            self.simulate()   
            
            # debug printing with the end effector position     
            link_state = p.getLinkState(self.ID[1], self.NumLinks-1)            
            print("End Effector position: " + str(link_state[0]))
            
    # this function gets a path length and a curvature, and returns the waypoints
    def get_waypoints(self, start_point, path_length, curvature, Nwaypoints):
        """
        Computes the waypoints of a path along an arc or a straight line given the path length, curvature (radius), and number of waypoints.

        :param start_point: Tuple/List, the starting point (x, y, z).
        :param path_length: Float, the length of the arc or straight path.
        :param curvature: Float, the radius of the circle. A value of 0 indicates a straight path.
        :param Nwaypoints: Integer, the number of waypoints.
        :return: List, the waypoints.
        """
        
        waypoints = []
        z = start_point[2]  # Z-coordinate remains constant
        
        if curvature == 0:  # Straight path
            dx = path_length / (Nwaypoints - 1)
            for i in range(Nwaypoints):
                x = start_point[0] + i * dx
                y = start_point[1]
                waypoints.append([x, y, z])
        else:  # Arc path
            cx = start_point[0]  # Center x of the circle remains the same as start x
            cy = start_point[1] + curvature  # Center y is adjusted by curvature
            
            # Total central angle that the arc spans
            theta = path_length / abs(curvature)
            
            # Angle increment for each waypoint
            delta_theta = theta / (Nwaypoints - 1)
            
            # Initial angle for the arc
            initial_angle = math.atan2(start_point[1] - cy, start_point[0] - cx)
            
            for i in range(Nwaypoints):
                angle = initial_angle + delta_theta * i
                x = cx + abs(curvature) * math.cos(angle)
                y = cy + abs(curvature) * math.sin(angle)
                waypoints.append([x, y, z])
                
        return waypoints
    
    # this function draws a point in the pybullet space
    def draw_circle(self, position, radius, color=[1, 0, 0], num_segments=8):
        """
        Draws a circle in PyBullet around a specific dot in space.

        :param position: Tuple/List, the (x, y, z) coordinates of the center of the circle.
        :param radius: Float, the radius of the circle.
        :param color: List, the RGB color of the circle.
        :param num_segments: Integer, the number of segments to use for the circle.
        """
        # Calculate the angle between segments
        angle_increment = 2 * math.pi / num_segments
        
        # Previous point, initialized to the first point of the circle
        prev_point = (position[0] + radius * math.cos(0),
                    position[1] + radius * math.sin(0),
                    position[2])
        
        for i in range(1, num_segments + 1):
            # Calculate the x and y coordinates of the next point
            x = position[0] + radius * math.cos(i * angle_increment)
            y = position[1] + radius * math.sin(i * angle_increment)
            z = position[2]
            
            # Draw a line from the previous point to the current point
            p.addUserDebugLine(prev_point, (x, y, z), lineColorRGB=color, lineWidth=2)
            
            # Update the previous point
            prev_point = (x, y, z)
            
    # this function draws a set of points given as a list using the previous draw_circles
    def draw_path(self, points, radius=0.01, color=[1, 0, 0], num_segments=24):
        """
        Draws a path in PyBullet using a set of points.

        :param points: List, the list of points to draw.
        :param radius: Float, the radius of the circle.
        :param color: List, the RGB color of the circle.
        :param num_segments: Integer, the number of segments to use for the circle.
        """
        for point in points:
            self.draw_circle(point, radius, color, num_segments)
            

# Example usage
if __name__ == "__main__":
    env = PyBulletEnvironment(gui=True)
    env.open_environment()
    # Perform simulation tasks here
    env.close_environment()