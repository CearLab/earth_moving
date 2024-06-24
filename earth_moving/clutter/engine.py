# this file contains the PyBulletEnvironment class
# the general capabilities are opening the environment, closing the environment, and general control functions

# general imports
import pybullet as p
import pybullet_data
import time as t

# class definition
class PyBulletEnvironment:
    
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
        
        # ID list 
        self.ID = []
        
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
        
    # load additional URDF file to the environment
    def load_urdf(self, urdf_file, init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
        """
        Loads a URDF file to the environment.

        :param urdf_file: String, the URDF file path.
        :param init_pos: Tuple, the initial position of the object.
        :param init_quat: Tuple, the initial quaternion of the object.
        """
        self.ID.append(p.loadURDF(urdf_file, init_pos, init_quat))
        
    # simulate for a given time
    def simulate(self, time, step=0.01):
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
    def control_joint(self, joint_index ,joint_target, control_mode=p.VELOCITY_CONTROL, max_force=1000):
        """
        Controls a joint of the robot.

        :param joint_index: Integer, the index of the joint.
        :param joint_target: Float, the desired position or velocity of the joint.
        :param control_mode: Integer, the control mode (p.POSITION_CONTROL or p.VELOCITY_CONTROL).
        """
        
        if control_mode == p.POSITION_CONTROL:
            p.setJointMotorControl2(self.ID[1], joint_index, control_mode, targetPosition=joint_target, force=max_force)
        elif control_mode == p.VELOCITY_CONTROL:
            p.setJointMotorControl2(self.ID[1], joint_index, control_mode, targetVelocity=joint_target, force=max_force)   
            
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

# Example usage
if __name__ == "__main__":
    env = PyBulletEnvironment(gui=True)
    env.open_environment()
    # Perform simulation tasks here
    env.close_environment()