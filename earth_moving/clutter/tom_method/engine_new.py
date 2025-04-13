import pybullet as p
import time as t
import numpy as np
import pybullet_data

class PyBulletEnvironment:
    def __init__(self, gui=True, gravity=(0, 0, -10)):
        self.gui = gui
        self.physicsClient = None
        self.control_threshold = 1e-2
        self.iter_limit = 5e2
        self.ID = []

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

        self.flags = self.URDF_MERGE_FIXED_LINKS | self.URDF_USE_INERTIA_FROM_FILE | self.URDF_USE_SELF_COLLISION | \
                     self.URDF_USE_SELF_COLLISION_INCLUDE_PARENT | self.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS | \
                     self.URDF_USE_IMPLICIT_CYLINDER | self.URDF_ENABLE_SLEEPING | self.URDF_INITIALIZE_SAT_FEATURES | \
                     self.URDF_USE_MATERIAL_COLORS_FROM_MTL | self.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | self.URDF_MAINTAIN_LINK_ORDER

        self.gravity = gravity

    def open_environment(self, robot_urdf="robot.urdf", init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
        if self.gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2])
        self.ID.append(p.loadURDF("plane.urdf"))
        self.ID.append(p.loadURDF(robot_urdf, init_pos, init_quat))
        self.NumJoints = p.getNumJoints(self.ID[1])
        self.NumLinks = self.NumJoints

    def load_urdf(self, urdf_file, init_pos=(0, 0, 0), init_quat=(0, 0, 0, 1)):
        self.ID.append(p.loadURDF(urdf_file, init_pos, init_quat, flags=self.flags))

    def simulate(self, time=0.1, step=1 / 240):
        for _ in range(int(time / step)):
            p.stepSimulation()
            t.sleep(step)

    def close_environment(self):
        p.disconnect()

    def control_joint(self, joint_index, joint_target, control_mode="VELOCITY", max_force=1000, target_velocity=0.1):
        if control_mode == "POSITION":
            p.setJointMotorControl2(self.ID[1], joint_index, p.POSITION_CONTROL,
                                    targetPosition=joint_target, targetVelocity=target_velocity,
                                    maxVelocity=100 * target_velocity, force=max_force,
                                    positionGain=0.0001, velocityGain=0.0005)
        elif control_mode == "VELOCITY":
            p.setJointMotorControl2(self.ID[1], joint_index, p.VELOCITY_CONTROL,
                                    targetVelocity=joint_target, force=max_force)
        else:
            print("Error: Control mode not recognized")

    def compute_joint_error(self, joint_index, target):
        joint_state = p.getJointState(self.ID[1], joint_index)
        return target - joint_state[0]

    def control_all_joints(self, joint_targets, targetVel, control_mode="VELOCITY", max_force=1000):
        Njoints = len(joint_targets)
        if control_mode == "VELOCITY":
            e = np.zeros(Njoints)
            for i in range(Njoints):
                e[i] = self.compute_joint_error(i, joint_targets[i])
            reach = np.zeros(Njoints)
            iter = 0
            while np.any(reach == 0) and iter < self.iter_limit:
                iter += 1
                for i in range(Njoints):
                    if np.abs(e[i]) > self.control_threshold and reach[i] == 0:
                        targetVelCtrl = targetVel[i] * np.sign(e[i])
                        self.control_joint(i, targetVelCtrl, control_mode, max_force, targetVelCtrl)
                    else:
                        self.control_joint(i, 0, control_mode, max_force, 0)
                        reach[i] = 1
                    e[i] = self.compute_joint_error(i, joint_targets[i])
                self.simulate()
                if iter >= self.iter_limit:
                    print("Warning: Iteration limit reached")
        elif control_mode == "POSITION":
            for i in range(Njoints):
                self.control_joint(i, joint_targets[i], control_mode, max_force, targetVel[i])
            self.simulate()
        else:
            print("Error: Control mode not recognized")

    def control_waypoints(self, waypoints, targetVel, control_mode="POSITION", max_force=1000):
        Nwaypoints = len(waypoints)
        for i in range(Nwaypoints):
            # Calculate inverse kinematics for the entire robot to reach the waypoint
            joint_positions = p.calculateInverseKinematics(self.ID[1], self.NumLinks - 1, waypoints[i],
                                                           maxNumIterations=500, residualThreshold=1e-2)

            # Control all joints to achieve the desired joint positions
            if control_mode == "POSITION":
                for jointIndex in range(self.NumLinks):
                    p.setJointMotorControl2(bodyIndex=self.ID[1],
                                            jointIndex=jointIndex,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=joint_positions[jointIndex],
                                            force=max_force)
            elif control_mode == "VELOCITY":
                for jointIndex in range(self.NumLinks):
                    p.setJointMotorControl2(bodyIndex=self.ID[1],
                                            jointIndex=jointIndex,
                                            controlMode=p.VELOCITY_CONTROL,
                                            targetVelocity=targetVel[jointIndex],
                                            force=max_force)
            else:
                print("Error: Control mode not recognized")

            self.simulate()

            # Debugging: Print the current position of the end-effector
            link_state = p.getLinkState(self.ID[1], self.NumLinks - 1)
            print(f"End Effector position after waypoint {i}: {link_state[0]}")

        # Final position reached, ensure the robot is stationary
        for jointIndex in range(self.NumLinks):
            p.setJointMotorControl2(bodyIndex=self.ID[1],
                                    jointIndex=jointIndex,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=max_force)

    def get_waypoints(self, start_point, path_length, curvature, Nwaypoints):
        waypoints = []
        z = start_point[2]
        if curvature == 0:
            dx = path_length / (Nwaypoints - 1)
            for i in range(Nwaypoints):
                x = start_point[0] + i * dx
                y = start_point[1]
                waypoints.append([x, y, z])
        else:
            cx = start_point[0]
            cy = start_point[1] + curvature
            theta = path_length / abs(curvature)
            delta_theta = theta / (Nwaypoints - 1)
            initial_angle = np.arctan2(start_point[1] - cy, start_point[0] - cx)
            for i in range(Nwaypoints):
                angle = initial_angle + delta_theta * i
                x = cx + abs(curvature) * np.cos(angle)
                y = cy + abs(curvature) * np.sin(angle)
                waypoints.append([x, y, z])
        return waypoints

    def get_quaternion_from_euler(self, euler):
        return p.getQuaternionFromEuler(euler)

    def get_euler_from_quaternion(self, quaternion):
        return p.getEulerFromQuaternion(quaternion)

    def get_link_state(self, body_id, link_id):
        return p.getLinkState(body_id, link_id)

    def step_simulation(self):
        p.stepSimulation()
