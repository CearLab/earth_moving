import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time
import math
import warnings
import cv2
from coordinate_converter import CoordinateConverter

# Suppress warnings
warnings.filterwarnings("ignore", category=UserWarning, module="numpy")

class PyBulletIntegration:
    def __init__(self, env_radius=1.0, target_zone_radius=0.3, num_pebbles=50, random_seed=42, gui=True, initial_robot_pose=(0, 0, 0)):
        # Initialize PyBullet
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1./240.)
        
        # Store object IDs
        self.object_ids = []
        self.target_zone_id = None
        self.grid_lines = []
        self.shovel_coverage = None
        
        # Store URDF path for pebbles
        self.pebbles_urdf = os.path.join("clutter", "urdf", "pebbles", "pebbles.urdf")
        
        # Environment parameters
        self.env_radius = env_radius  # meters
        self.target_zone_radius = target_zone_radius  # meters
        self.num_pebbles = num_pebbles
        self.random_seed = random_seed
        self.initial_robot_pose = initial_robot_pose  # Store initial pose
        
        # Rover parameters
        self.shovel_width = 0.22  # meters
        self.shovel_height = 0.08
        self.shovel_depth = 0.01
        
        # Rover control parameters
        self.control_threshold = 1e-2
        self.angle_threshold = 5
        self.iter_limit = 1e2
        self.Kp_vel = 0.0001
        self.Kphi_vel = 0.00001
        self.Kd_phi_vel = 0
        
        # Baseline control gains (reasonable starting values)
        self.K1_base = 2.0  # Linear velocity gain
        self.K2_base = 3.0  # Angular velocity gain  
        self.K3_base = 5.0  # Delta angle gain
        self.vmax_base = 0.4  # Base max velocity
        
        # Current adaptive gains (initialized to baseline)
        self.K1 = self.K1_base
        self.K2 = self.K2_base
        self.K3 = self.K3_base
        self.vmax = self.vmax_base
        self.phimax = 1
        self.control_dt = 1/240
        
        # Adaptive gain scheduling parameters
        self.enable_adaptive_gains = True
        self.curvature_window = 5  # Number of waypoints to look ahead for curvature calculation
        
        # Coordinate converter for grid/cell calculations
        self.coord_converter = CoordinateConverter(
            env_radius=self.env_radius,
            target_zone_radius=self.target_zone_radius,
            shovel_width=self.shovel_width
        )
        
        print("\nPyBullet initialized successfully")
        print(f"Environment radius: {self.env_radius:.3f} meters")
        print(f"Target zone radius: {self.target_zone_radius:.3f} meters")
        print(f"Number of pebbles: {self.num_pebbles}")
        print(f"Random seed: {self.random_seed}")
        print(f"Shovel width: {self.shovel_width:.3f} meters")
        print(f"Environment diameter: {2*self.env_radius:.3f} meters")

    def set_robot_dim(self, L=0.2, R=0.07):
        """Set robot dimensions."""
        self.L = L
        self.R = R

    def get_joint_index_by_name(self, joint_name):
        rover_id = self.object_ids[1]  # robot is the second object
        for i in range(p.getNumJoints(rover_id)):
            info = p.getJointInfo(rover_id, i)
            if info[1].decode("utf-8") == joint_name:
                return i
        raise ValueError(f"Joint '{joint_name}' not found")


    def get_robot_position(self):
        """
        Return (pos, quat) of the rover base – NOT the plane.
        """
        return p.getBasePositionAndOrientation(self.robot_id)

    def get_velocities(self):
        """
        Forward linear velocity V [m/s] and yaw-rate φ [rad/s] measured
        in the rover’s body frame.
        """
        import numpy as np, pybullet as p

        lin_w, ang_w = p.getBaseVelocity(self.robot_id)
        _, quat = p.getBasePositionAndOrientation(self.robot_id)

        # world → body frame
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        lin_b = R.T @ lin_w            # body-frame linear velocity

        V = lin_b[0]                   # body-x component = forward speed
        phi = ang_w[2]                 # world-z is the same as body-z

        return V, phi

    def compute_wheel_velocities(self, V, phi):
        """Compute wheel velocities from linear and angular velocities."""
        V = np.clip(V, -self.vmax, self.vmax)
        phi = np.clip(phi, -self.phimax, self.phimax)
        
        omega_r = -(2 * V + phi * self.L) / (2 * self.R)
        omega_l = -(2 * V - phi * self.L) / (2 * self.R)
        
        return omega_r, omega_l


    def control_rover_velocity(self, left_wheel_vel, right_wheel_vel, time=1/240):
        rover_id = self.object_ids[1]  # assuming plane is [0], robot is [1]

        # Dynamically fetch correct joint indices
        left_joint = self.get_joint_index_by_name("base_to_lwheel")
        right_joint = self.get_joint_index_by_name("base_to_rwheel")

        # Apply lateral friction
        p.changeDynamics(rover_id, left_joint, lateralFriction=10.0)
        p.changeDynamics(rover_id, right_joint, lateralFriction=10.0)
        p.changeDynamics(self.object_ids[0], -1, lateralFriction=1.0)  # Plane

        # Apply motor control
        MAX_WHEEL_TORQUE = 150000  # N·m – good starting point
        p.setJointMotorControl2(rover_id, left_joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_wheel_vel, force=MAX_WHEEL_TORQUE)
        p.setJointMotorControl2(rover_id, right_joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_wheel_vel, force=MAX_WHEEL_TORQUE)

        self.simulate(time)



    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def calculate_path_curvature(self, trajectory, current_index, window=3):
        """
        Calculate path curvature at current position by looking ahead.
        Returns curvature value (higher = more curved path).
        """
        if current_index + window >= len(trajectory):
            return 0.0
        
        # Get three points: current, mid, and future
        p1 = np.array(trajectory[current_index][:2])
        p2 = np.array(trajectory[min(current_index + window//2, len(trajectory)-1)][:2])
        p3 = np.array(trajectory[min(current_index + window, len(trajectory)-1)][:2])
        
        # Calculate vectors
        v1 = p2 - p1
        v2 = p3 - p2
        
        # Calculate angle between vectors
        if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
            return 0.0
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # Curvature is inversely related to the angle (smaller angle = higher curvature)
        curvature = (np.pi - angle) / np.pi
        return curvature
    
    def adaptive_gains(self, current_pos, trajectory, current_index):
        """
        Adaptive gain scheduling based on trajectory characteristics.
        
        Parameters:
        - current_pos: Current robot position (x, y, z)
        - trajectory: List of trajectory waypoints
        - current_index: Current waypoint index
        
        Returns:
        - Tuple of (K1, K2, K3, vmax) adapted gains
        """
        if not self.enable_adaptive_gains:
            return self.K1_base, self.K2_base, self.K3_base, self.vmax_base
        
        # Calculate distance to current target
        if current_index >= len(trajectory):
            return self.K1_base, self.K2_base, self.K3_base, self.vmax_base
            
        target_pos = trajectory[current_index]
        dist_to_target = np.linalg.norm(np.array(current_pos[:2]) - np.array(target_pos[:2]))
        
        # Calculate path curvature
        curvature = self.calculate_path_curvature(trajectory, current_index, self.curvature_window)
        
        # Calculate distance to end of trajectory
        final_pos = trajectory[-1]
        dist_to_end = np.linalg.norm(np.array(current_pos[:2]) - np.array(final_pos[:2]))
        
        # Phase-based adaptation
        if dist_to_end > 0.8:  # Approach phase (far from end)
            K1 = self.K1_base * 1.2  # More aggressive forward motion
            K2 = self.K2_base * 0.8  # Less aggressive lateral correction
            K3 = self.K3_base * 0.8  # Less aggressive angular correction
            vmax = self.vmax_base * 1.0  # Full speed
            
        elif dist_to_end > 0.3:  # Tracking phase (middle distance)
            K1 = self.K1_base * 1.0  # Normal forward motion
            K2 = self.K2_base * 1.0  # Normal lateral correction
            K3 = self.K3_base * 1.0  # Normal angular correction
            vmax = self.vmax_base * 0.8  # Reduced speed
            
        else:  # Precision phase (close to end)
            K1 = self.K1_base * 0.6  # Careful forward motion
            K2 = self.K2_base * 1.4  # More precise lateral correction
            K3 = self.K3_base * 1.6  # More precise angular correction
            vmax = self.vmax_base * 0.5  # Slow and careful
        
        # Curvature-based adaptation
        if curvature > 0.4:  # High curvature (sharp turn)
            K1 *= 0.7  # Reduce forward speed
            K2 *= 1.3  # Increase lateral correction
            K3 *= 1.4  # Increase angular correction
            vmax *= 0.6  # Reduce max speed
            
        elif curvature > 0.2:  # Medium curvature
            K1 *= 0.85  # Slightly reduce forward speed
            K2 *= 1.15  # Slightly increase lateral correction
            K3 *= 1.2   # Slightly increase angular correction
            vmax *= 0.8  # Slightly reduce max speed
        
        # Distance-based fine-tuning
        if dist_to_target < 0.1:  # Very close to target
            K1 *= 0.5  # Very careful forward motion
            K2 *= 1.5  # Increase precision
            K3 *= 1.5  # Increase precision
            vmax *= 0.4  # Very slow
        
        # Ensure gains stay within reasonable bounds
        K1 = np.clip(K1, 0.5, 5.0)
        K2 = np.clip(K2, 1.0, 8.0)
        K3 = np.clip(K3, 2.0, 12.0)
        vmax = np.clip(vmax, 0.1, 0.8)
        
        return K1, K2, K3, vmax


    def compute_target_velocities(self, target_pos, target_quat, current_pos, current_quat):
        """Compute target velocities for position control."""
        # Compute position error
        delta_pos = np.array(current_pos) - np.array(target_pos)
        ex = delta_pos[0]
        ey = delta_pos[1]
        
        # Compute orientation error
        current_theta = p.getEulerFromQuaternion(current_quat)[2]
        target_theta = p.getEulerFromQuaternion(target_quat)[2]
        etheta = self.normalize_angle(current_theta - target_theta)
        
        # Error vector
        e = np.array([ex, ey, etheta])
        
        # R matrix
        R = np.array([[+np.cos(target_theta), +np.sin(target_theta), 0],
                     [-np.sin(target_theta), +np.cos(target_theta), 0],
                     [0, 0, 1]])
        e_r = np.dot(R, e)
        
        # Change of coordinates
        rho = np.sqrt(e_r[0]**2 + e_r[1]**2)
        gamma = self.normalize_angle(np.arctan2(e_r[1], e_r[0]) - e_r[2] + np.pi)
        delta = self.normalize_angle(gamma + e_r[2])
        
        # Compute target velocities with epsilon to prevent division by zero
        epsilon = 1e-6
        sigma = 1 if abs(rho) > 1e-4 else 0
        
        # Linear velocity control
        target_V = self.K1 * rho * np.cos(gamma) * sigma
        
        # Angular velocity control with improved stability
        if abs(gamma) < epsilon:
            # When gamma is very small, use a simpler control law
            target_phi = self.K2 * gamma + self.K3 * delta
        else:
            # Normal control law with epsilon to prevent division by zero
            target_phi = ((self.K1 * np.sin(gamma) * np.cos(gamma)) / (gamma + epsilon)) * (gamma + self.K3 * delta) + self.K2 * gamma
        
        return target_V, target_phi, e_r

    def follow_trajectory(self, trajectory, step_time=0.1):
        """Follow a trajectory using position control."""
        # Get current robot position and orientation
        current_pos, current_quat = self.get_robot_position()
        current_theta = p.getEulerFromQuaternion(current_quat)[2]
        
        # Get the first point of the trajectory
        start = trajectory[0]
        start_world = (
            (start[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
            (start[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
            0.1  # Slightly above ground
        )

        # Calculate target orientation based on first movement
        if len(trajectory) > 1:
            next_point = trajectory[1]
            next_world = (
                (next_point[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (next_point[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1
            )
            dx = next_world[0] - start_world[0]
            dy = next_world[1] - start_world[1]
            target_theta = math.atan2(dy, dx)
        else:
            target_theta = current_theta

        # Calculate approach trajectory
        # First, calculate the distance and angle to the target
        dx = start_world[0] - current_pos[0]
        dy = start_world[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)
        
        # Create approach waypoints
        approach_trajectory = []
        
        # If we need to rotate first
        angle_diff = self.normalize_angle(angle_to_target - current_theta)
        if abs(angle_diff) > 0.1:  # If angle difference is significant
            # Add rotation waypoint
            approach_trajectory.append((
                current_pos[0],
                current_pos[1],
                angle_to_target
            ))
        
        # Add intermediate point if distance is large
        if distance > 0.5:  # If distance is significant
            mid_x = current_pos[0] + 0.5 * dx
            mid_y = current_pos[1] + 0.5 * dy
            approach_trajectory.append((
                mid_x,
                mid_y,
                angle_to_target
            ))
        
        # Add final approach point
        approach_trajectory.append((
            start_world[0],
            start_world[1],
            target_theta
        ))

        # Execute approach trajectory
        print("\nExecuting approach trajectory...")
        for i in range(len(approach_trajectory) - 1):
            start = approach_trajectory[i]
            end = approach_trajectory[i + 1]
            
            # Calculate desired orientation
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            target_theta = math.atan2(dy, dx)
            target_quat = p.getQuaternionFromEuler([0, 0, target_theta])
            
            # Set position and wait for completion
            self.set_positions((end[0], end[1], 0.1), target_quat)
            t.sleep(step_time)
        
        # Follow the main trajectory
        print("Following main trajectory...")
        for i in range(len(trajectory) - 1):
            start = trajectory[i]
            end = trajectory[i + 1]
            
            # Convert grid coordinates to world coordinates
            start_world = (
                (start[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (start[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1  # Slightly above ground
            )
            end_world = (
                (end[0] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                (end[1] - self.coord_converter.grid_size/2) * self.coord_converter.cell_size,
                0.1  # Slightly above ground
            )
            
            # Calculate desired orientation
            dx = end_world[0] - start_world[0]
            dy = end_world[1] - start_world[1]
            target_theta = math.atan2(dy, dx)
            target_quat = p.getQuaternionFromEuler([0, 0, target_theta])
            
            # Set position and wait for completion
            self.set_positions(end_world, target_quat)
            t.sleep(step_time)

    def set_positions(self, target_pos, target_quat):
        """Set robot position and orientation using position control."""
        reached = False
        control_iter = 0
        error_vector = []
        max_iterations = 1000  # Add a maximum iteration limit
        
        while not reached and control_iter < max_iterations:
            # Get current position and orientation
            current_pos, current_quat = self.get_robot_position()
            
            # Compute target velocities
            target_V, target_phi, e_r = self.compute_target_velocities(target_pos, target_quat, current_pos, current_quat)
            
            # Compute wheel velocities
            omega_r, omega_l = self.compute_wheel_velocities(target_V, target_phi)
            
            # Set velocities
            self.control_rover_velocity(omega_l, omega_r, self.control_dt)
            
            # Check if target reached
            control_iter += 1
            if np.linalg.norm(e_r[:-1]) < self.control_threshold and abs(np.rad2deg(e_r[-1])) < self.angle_threshold:
                reached = True
                break
            
            e_r[-1] = np.rad2deg(e_r[-1])
            error_vector.append(e_r)
            
            # Add a small delay to prevent too rapid control updates
            time.sleep(0.01)
        
        return np.array(error_vector).reshape(-1, 3), control_iter

    def get_current_state(self):
        """Get current 3D object positions."""
        objects_3d = []
        for obj_id in self.object_ids:
            try:
                pos, _ = p.getBasePositionAndOrientation(obj_id)
                objects_3d.append((pos[0], pos[1], pos[2]))
            except Exception as e:
                print(f"Warning: Failed to get position for object {obj_id}: {str(e)}")
        return objects_3d
        
    def create_grid_overlay(self):
        """Create visual grid overlay in PyBullet using grid size/cell size from CoordinateConverter."""
        for line_id in self.grid_lines:
            p.removeBody(line_id)
        self.grid_lines = []

        grid_size = self.coord_converter.grid_size
        cell_size = self.coord_converter.cell_size
        env_radius = self.env_radius
        env_diameter = 2 * env_radius

        for i in range(grid_size + 1):
            pos = -env_radius + (i * env_diameter / grid_size)

            # Horizontal line
            line_id = p.addUserDebugLine(
                [-env_radius, pos, 0.01],
                [env_radius, pos, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

            # Vertical line
            line_id = p.addUserDebugLine(
                [pos, -env_radius, 0.01],
                [pos, env_radius, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

            
    def create_target_zone(self):
        """Create target zone in PyBullet."""
        target_zone_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=self.target_zone_radius,
            length=0.01,
            rgbaColor=[1, 0, 0, 0.3]
        )
        
        self.target_zone_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=target_zone_visual,
            basePosition=[0, 0, 0.005]
        )
        
    def generate_random_positions(self):
        """Generate random positions for pebbles."""
        np.random.seed(self.random_seed)
        positions = []
        
        while len(positions) < self.num_pebbles:
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(self.target_zone_radius, self.env_radius)
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            positions.append((x, y, 0.1))
            
        return positions
        
    def setup_scene(self, initial_robot_pose):
        """
        Reset the world, load plane, rover and pebbles, and cache handles.
        After this call:
            self.robot_id        – the rover base body unique ID
            self.left_wheel_joint,
            self.right_wheel_joint – the two drive-wheel joint indices
        """
        import os, numpy as np, pybullet as p

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240.)
        self.object_ids = []                # clear previous handles

        # ---- ground plane -------------------------------------------------
        plane_id = p.loadURDF("plane.urdf")
        self.object_ids.append(plane_id)

        # ---- rover --------------------------------------------------------
        x, y, theta = initial_robot_pose
        init_quat = p.getQuaternionFromEuler([0, 0, theta])
        robot_urdf = os.path.join(os.path.dirname(__file__), "2_wheel_rover.urdf")
        rover_id = p.loadURDF(robot_urdf, [x, y, 0.10], init_quat)
        self.object_ids.append(rover_id)
        self.robot_id = rover_id            # ☆ keep a persistent handle

        # Disable default position control on every joint
        for j in range(p.getNumJoints(rover_id)):
            p.setJointMotorControl2(rover_id, j, p.VELOCITY_CONTROL, force=0)

        # Apply friction to wheels and floor
        for j in range(p.getNumJoints(rover_id)):
            name = p.getJointInfo(rover_id, j)[1].decode().lower()
            if "wheel" in name:
                p.changeDynamics(rover_id, j, lateralFriction=1.5)
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Cache wheel joint indices for fast control
        self.left_wheel_joint = None
        self.right_wheel_joint = None
        for j in range(p.getNumJoints(rover_id)):
            name = p.getJointInfo(rover_id, j)[1].decode().lower()
            if "lwheel" in name:
                self.left_wheel_joint = j
            elif "rwheel" in name:
                self.right_wheel_joint = j

        # ---- visual helpers & pebbles (unchanged) -------------------------
        self.create_target_zone()
        self.create_grid_overlay()
        pebble_positions = self.generate_random_positions()
        for pos in pebble_positions:
            pebble_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                       self.pebbles_urdf)
            pebble_id = p.loadURDF(pebble_path, pos, useFixedBase=False)
            self.object_ids.append(pebble_id)

        print("[setup] Scene ready – plane, rover and", len(pebble_positions), "pebbles loaded.")
            
    # def reset_simulation(self):
    #     """Reset simulation to initial state."""
    #     for obj_id in self.object_ids:
    #         p.removeBody(obj_id)
    #     self.object_ids = []
    #     self.setup_scene(self.initial_robot_pose)

        
    def run(self):
        print("Setting up scene...")
        self.setup_scene(self.initial_robot_pose)

        p.resetDebugVisualizerCamera(
            cameraDistance=3.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        while True:
            p.stepSimulation()
            keys = p.getKeyboardEvents()
            if ord('q') in keys:
                break
            elif ord('c') in keys:
                return self.get_current_state()
            time.sleep(1 / 240)


    def visualize_trajectory(self, trajectory_2d):
        """Visualize a 2D trajectory in the 3D environment."""
        # Convert 2D grid coordinates to 3D world coordinates
        trajectory_3d = []
        for x, y in trajectory_2d:
            # Convert grid coordinates to world coordinates
            world_x, world_y = self.coord_converter.convert_2d_to_3d(x, y)
            trajectory_3d.append((world_x, world_y, 0.01))  # Just slightly above ground
        
        # Draw trajectory lines
        for i in range(len(trajectory_3d) - 1):
            start = trajectory_3d[i]
            end = trajectory_3d[i + 1]
            p.addUserDebugLine(
                start,
                end,
                [1, 0, 0],  # Red color
                2.0,  # Line width
                lifeTime=0  # Forever
            )                    
                
    def clear_trajectory(self):
        """Clear all trajectory visualizations."""
        # Remove only user debug items (lines and markers)
        p.removeAllUserDebugItems()
        
        # Redraw grid and target zone
        self.create_grid_overlay()
        self.create_target_zone()

    def simulate(self, sim_time=0.1, step=1/240):
        """Simulate environment for given time."""
        for _ in range(int(sim_time/step)):
            p.stepSimulation()
            time.sleep(step)

    def close_environment(self):
        """Close PyBullet environment."""
        p.disconnect()

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

    def follow_smooth_trajectory(
        self,
        trajectory,
        control_dt      = 1 / 240,
        pos_tol         = 0.05,      # metres
        angle_tol       = 0.20,      # rad  ≈ 11°
        lookahead       = 1,         # how many way-points to peek ahead
        min_speed       = 0.04       # m/s we apply while pivoting so we never stall
    ):
        """
        Closed-loop tracking of a list of (x, y, θ) way-points.

        * Steps through the list in order, but if we are already closer than
          `pos_tol` to the current target, we **immediately advance** to the next
          one (no matter the heading).  This prevents the “stuck turning in place”
          problem once the rover is on top of a point but still mis-aligned.

        * Keeps a small forward component `min_speed` even while it is mostly
          correcting heading, so static friction (or a pebble nudge) can’t freeze
          the wheels.

        * `lookahead>0` lets the robot aim slightly further down the path for
          smoother motion on very dense trajectories.
        """
        idx = 0
        finished = False   
        max_iter_0 = 200  # hard safety stop     
        max_iter = max_iter_0     # hard safety stop
        
        while not finished:                        

            # ── Current pose ──────────────────────────────────────────────
            (x, y, _), q = self.get_robot_position()
            theta = p.getEulerFromQuaternion(q)[2]

            # ── Which point are we chasing? ──────────────────────────────
            #   Skip points that are already within the distance tolerance,
            #   regardless of the current heading.
            while idx < len(trajectory):
                dx = trajectory[idx][0] - x
                dy = trajectory[idx][1] - y
                max_iter -= 1
                if math.hypot(dx, dy) > pos_tol and max_iter > 0:
                    break                      # this one still matters
                max_iter = max_iter_0        # hard safety stop
                idx += 1                       # already “there” → next

            if idx >= len(trajectory):
                finished = True
                break

            # Optional look-ahead
            tgt_idx  = min(idx + lookahead, len(trajectory) - 1)
            x_t, y_t, th_t = trajectory[tgt_idx]

            # ── Errors in world frame ────────────────────────────────────
            dx   = x_t - x
            dy   = y_t - y
            dθ   = self.normalize_angle(th_t - theta)

            dist = math.hypot(dx, dy)
            ang  = abs(dθ)

            # ── Debug line (your “sanity check”) ─────────────────────────
            print(f"idx={idx:3d}  dist={dist:5.3f}  ang={ang:5.3f}  iter={max_iter:4d}")

            # ── Transform position error to body frame ───────────────────
            ex =  math.cos(theta) * dx + math.sin(theta) * dy   # fwd
            ey = -math.sin(theta) * dx + math.cos(theta) * dy   # left

            # ── Simple two-zone controller ───────────────────────────────
            # if dist < 0.15:                       # close → emphasise rotation
            #     V   = self.K1 * ex
            #     φ   = self.K3 * dθ
            #     V   = math.copysign(max(abs(V), min_speed), V)  # keep moving
            # else:                                 # far  → move & steer
            V   = self.K1 * ex
            φ   = self.K2 * ey + self.K3 * dθ

            ω_r, ω_l = self.compute_wheel_velocities(V, φ)
            self.control_rover_velocity(ω_l, ω_r, control_dt)

    def draw_trajectory(self, trajectory, color=[1, 0, 0], life_time=0):
        """
        Draws a trajectory in PyBullet using debug lines.
        
        :param trajectory: List of waypoints, either (x, y) or (x, y, theta)
        :param color: RGB list, default red
        :param life_time: Duration to keep the lines (0 = forever)
        """
        for i in range(len(trajectory) - 1):
            # Handle both 2D and 3D points
            if len(trajectory[i]) == 2:
                x0, y0 = trajectory[i]
                x1, y1 = trajectory[i + 1]
            else:
                x0, y0, _ = trajectory[i]
                x1, y1, _ = trajectory[i + 1]
                
            p.addUserDebugLine(
                [x0, y0, 0.05],  # Start point
                [x1, y1, 0.05],  # End point
                lineColorRGB=color,
                lineWidth=2.0,
                lifeTime=life_time
            )





def main():
    integration = PyBulletIntegration()
    integration.run()

if __name__ == "__main__":
    main()