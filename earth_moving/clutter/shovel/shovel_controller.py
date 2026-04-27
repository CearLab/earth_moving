"""
ShovelController: A class for controlling a 2D Cartesian shovel robot in PyBullet.

This module provides high-level control of a shovel robot with:
- Forward and inverse kinematics
- Sequential Cartesian control
- End-effector trajectory tracking
- Workspace visualization
"""

import numpy as np
import pybullet as p
from pathlib import Path
from datetime import datetime
import cv2
import matplotlib.pyplot as plt


class ShovelController:
    """
    Control a shovel robot as a 2D Cartesian system with decoupled DOFs:
    - X motion (prismatic joint)
    - Y motion (prismatic joint)
    - Orientation (revolute joint)
    """

    def __init__(self, robot_id, urdf_path="./urdf/shovel/shovelFlat.urdf", 
                 start_pos=None, start_orientation=None):
        """
        Initialize the ShovelController.

        :param robot_id: PyBullet robot ID
        :param urdf_path: Path to shovel URDF file (for reference)
        :param start_pos: Initial position [x, y, z]
        :param start_orientation: Initial orientation quaternion
        """
        self.robotId = robot_id
        self.urdf_path = urdf_path
        
        # Initialize configuration
        self._setup_kinematic_chain()
        self._setup_control_parameters()

    def _setup_kinematic_chain(self):
        """Extract and organize joint information from the robot."""
        num_joints = p.getNumJoints(self.robotId)
        
        self.joint_info = {}
        self.prismatic_joints = []
        self.revolute_joints = []
        self.fixed_joints = []
        self.joint_limits = {}

        for i in range(num_joints):
            info = p.getJointInfo(self.robotId, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]  # 0=REVOLUTE, 1=PRISMATIC, 4=FIXED
            
            self.joint_info[joint_name] = {
                'index': i,
                'type': joint_type,
                'info': info
            }

            if joint_type == 1:  # PRISMATIC
                self.prismatic_joints.append(i)
            elif joint_type == 0:  # REVOLUTE
                self.revolute_joints.append(i)
            elif joint_type == 4:  # FIXED
                self.fixed_joints.append(i)

            # Store joint limits
            lower_limit = info[8]
            upper_limit = info[9]
            max_force = info[10]
            max_velocity = info[11]

            self.joint_limits[i] = {
                'name': joint_name,
                'lower': lower_limit,
                'upper': upper_limit,
                'max_force': max_force,
                'max_velocity': max_velocity
            }

        # Configure control joints for 2D Cartesian control
        self.joint_x_idx = self.prismatic_joints[0] if len(self.prismatic_joints) > 0 else None
        self.joint_y_idx = self.prismatic_joints[1] if len(self.prismatic_joints) > 1 else None
        self.joint_theta_idx = self.revolute_joints[-1] if len(self.revolute_joints) > 0 else None

    def _setup_control_parameters(self):
        """Set up control parameters."""
        self.control_dt = 1.0 / 240.0  # Physics simulation timestep (240 Hz)

    def print_joint_info(self):
        """Print detailed joint information."""
        print(f"\nTotal joints: {p.getNumJoints(self.robotId)}")
        print("\nJoint Information:")
        print("-" * 80)

        for i in range(p.getNumJoints(self.robotId)):
            info = p.getJointInfo(self.robotId, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            joint_type_str = {0: 'REVOLUTE', 1: 'PRISMATIC', 2: 'SPHERICAL', 
                             3: 'PLANAR', 4: 'FIXED'}.get(joint_type, 'UNKNOWN')
            
            print(f"Joint {i}: {joint_name:20s} - Type: {joint_type_str:10s}")

        print(f"\nPrismatic joints (indices): {self.prismatic_joints}")
        print(f"Revolute joints (indices): {self.revolute_joints}")
        print(f"Fixed joints (indices): {self.fixed_joints}")

        print(f"\n2D Cartesian Control Configuration:")
        if self.joint_x_idx is not None:
            print(f"  X motion joint: {self.joint_x_idx} - {self.joint_limits[self.joint_x_idx]['name']}")
        if self.joint_y_idx is not None:
            print(f"  Y motion joint: {self.joint_y_idx} - {self.joint_limits[self.joint_y_idx]['name']}")
        if self.joint_theta_idx is not None:
            print(f"  Theta joint: {self.joint_theta_idx} - {self.joint_limits[self.joint_theta_idx]['name']}")

    def print_joint_limits(self):
        """Print joint limits for all prismatic and revolute joints."""
        print("Joint Limits:")
        print("-" * 80)
        for joint_idx in self.prismatic_joints + self.revolute_joints:
            limits = self.joint_limits[joint_idx]
            print(f"Joint {joint_idx} ({limits['name']}):")
            print(f"  Limits: [{limits['lower']:.3f}, {limits['upper']:.3f}]")
            print(f"  Max Force: {limits['max_force']:.3f}, Max Velocity: {limits['max_velocity']:.3f}")

    def forward_kinematics_2d(self):
        """
        Forward kinematics: Query the current end-effector position from PyBullet.

        :return: Tuple (end_effector_pos [x, y, z], orientation_theta)
        """
        num_links = p.getNumJoints(self.robotId)
        ee_link_idx = num_links - 1

        link_state = p.getLinkState(self.robotId, ee_link_idx)
        ee_pos = np.array(link_state[0])
        ee_quat = link_state[1]

        euler = p.getEulerFromQuaternion(ee_quat)
        theta = euler[2]  # Z rotation

        return ee_pos, theta

    def inverse_kinematics_2d(self, target_x, target_y, target_theta):
        """
        Direct inverse kinematics for planar 2D Cartesian robot.
        For decoupled DOFs (X-prismatic, Y-prismatic, Z-revolute), the solution is trivial.

        :param target_x: Target X position
        :param target_y: Target Y position
        :param target_theta: Target orientation (radians)
        :return: Dictionary mapping joint indices to target positions
        """
        joint_commands = {
            self.joint_x_idx: target_x,
            self.joint_y_idx: target_y,
            self.joint_theta_idx: target_theta
        }

        return joint_commands

    def get_current_cartesian_state(self):
        """
        Get current end-effector state in Cartesian space.

        :return: Tuple (x, y, theta, z)
        """
        ee_pos, ee_theta = self.forward_kinematics_2d()

        x_pos = ee_pos[0]
        y_pos = ee_pos[1]
        z_pos = ee_pos[2]
        theta = ee_theta

        return x_pos, y_pos, theta, z_pos

    def apply_cartesian_control_sequential(self, target_x, target_y, target_theta,
                                          max_force=100, max_velocity=0.5, total_steps=1000, verbose=True):
        """
        Apply sequential position control: set joints to target, then record trajectory.

        :param target_x, target_y, target_theta: Target Cartesian position and orientation
        :param max_force: Maximum motor force in Newtons (capped by URDF limits)
        :param max_velocity: Maximum joint velocity in rad/s or m/s
        :param total_steps: Number of simulation steps to record
        :param verbose: Print progress information
        :return: Tuple (trajectory array, errors array, joint_velocities array, target_velocities)
        """
        trajectory = []
        errors = []
        z_positions = []
        joint_velocities = []
        target_velocities = [max_velocity, max_velocity, max_velocity]  # X, Y, Theta

        try:
            # Compute target joint positions
            joint_commands = self.inverse_kinematics_2d(target_x, target_y, target_theta)

            if verbose:
                print(f"IK computed target joint positions:")
                print(f"  Joint {self.joint_x_idx} (X): {joint_commands[self.joint_x_idx]:.4f}")
                print(f"  Joint {self.joint_y_idx} (Y): {joint_commands[self.joint_y_idx]:.4f}")
                print(f"  Joint {self.joint_theta_idx} (Theta): {joint_commands[self.joint_theta_idx]:.4f}")

            # Prepare arrays for batch control
            joint_indices = [self.joint_x_idx, self.joint_y_idx, self.joint_theta_idx]
            joint_names = {self.joint_x_idx: 'X', self.joint_y_idx: 'Y', self.joint_theta_idx: 'Theta'}
            
            target_positions = []
            forces = []
            for joint_idx in joint_indices:
                target_pos = joint_commands[joint_idx]
                limits = self.joint_limits[joint_idx]
                target_pos_clamped = np.clip(target_pos, limits['lower'], limits['upper'])
                target_positions.append(target_pos_clamped)
                
                # Use the specified max_force, but cap it by URDF limits
                joint_max_force = min(limits['max_force'], max_force)
                forces.append(joint_max_force)

                if verbose:
                    print(f"Joint {joint_idx} ({joint_names[joint_idx]}): target={target_pos_clamped:.4f}, force={joint_max_force:.1f}")

            # Set all joints at once using setJointMotorControlArray
            if verbose:
                print(f"\n--- Setting all joints to target positions (batch control) ---")
            
            p.setJointMotorControlArray(
                self.robotId,
                joint_indices,
                p.POSITION_CONTROL,
                targetPositions=target_positions,
                forces=forces,                
            )

            # Record the trajectory as all joints move
            if verbose:
                print(f"\nRecording trajectory over {total_steps} steps...")
            
            for step in range(total_steps):
                p.stepSimulation()

                x_cur, y_cur, theta_cur, z_cur = self.get_current_cartesian_state()

                error_x = target_x - x_cur
                error_y = target_y - y_cur
                error_theta = target_theta - theta_cur
                error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))
                pos_error = np.sqrt(error_x**2 + error_y**2)

                trajectory.append([x_cur, y_cur, theta_cur])
                z_positions.append(z_cur)
                errors.append([error_x, error_y, error_theta, pos_error])
                
                # Capture joint velocities
                vel_x = p.getJointState(self.robotId, self.joint_x_idx)[1]
                vel_y = p.getJointState(self.robotId, self.joint_y_idx)[1]
                vel_theta = p.getJointState(self.robotId, self.joint_theta_idx)[1]
                joint_velocities.append([vel_x, vel_y, vel_theta])

            trajectory = np.array(trajectory)
            z_positions = np.array(z_positions)
            errors = np.array(errors)
            joint_velocities = np.array(joint_velocities)

            if verbose:
                print(f"\n=== Sequential control complete ===")
                print(f"Z motion: min={z_positions.min():.4f}, max={z_positions.max():.4f}, "
                      f"range={z_positions.max()-z_positions.min():.4f} m")
                if z_positions.max() - z_positions.min() > 0.001:
                    print("WARNING: Significant Z-axis motion detected!")

            return trajectory, errors, joint_velocities, target_velocities

        except Exception as e:
            print(f"ERROR in apply_cartesian_control_sequential: {e}")
            import traceback
            traceback.print_exc()
            return np.array([]), np.array([]), np.array([]), [0, 0, 0]

    def apply_cartesian_control_velocity_mode(self, target_x, target_y, target_theta,
                                              max_velocity=0.5, max_force=100, 
                                              tolerance=0.01, max_steps=2000, verbose=True):
        """
        Apply velocity control to reach a target position.
        Uses VELOCITY_CONTROL mode which gives explicit velocity constraints.
        
        :param target_x, target_y, target_theta: Target Cartesian position and orientation
        :param max_velocity: Maximum joint velocity in rad/s or m/s (hard constraint)
        :param max_force: Maximum motor force in Newtons
        :param tolerance: Position tolerance to consider target reached (meters)
        :param max_steps: Maximum simulation steps before stopping
        :param verbose: Print progress information
        :return: Tuple (trajectory, errors, joint_velocities, commanded_velocities)
        """
        trajectory = []
        errors = []
        joint_velocities = []
        commanded_velocities = []
        
        try:
            joint_indices = [self.joint_x_idx, self.joint_y_idx, self.joint_theta_idx]
            joint_names = {self.joint_x_idx: 'X', self.joint_y_idx: 'Y', self.joint_theta_idx: 'Theta'}
            
            if verbose:
                print(f"Starting velocity control to target:")
                print(f"  Target: X={target_x:.4f}, Y={target_y:.4f}, Theta={np.degrees(target_theta):.2f}°")
                print(f"  Max velocity: {max_velocity:.4f} rad/s or m/s")
            
            # Get joint force limits
            forces = []
            for joint_idx in joint_indices:
                limits = self.joint_limits[joint_idx]
                joint_max_force = min(limits['max_force'], max_force)
                forces.append(joint_max_force)
            
            step = 0
            while step < max_steps:
                # Get current state
                x_cur, y_cur, theta_cur, z_cur = self.get_current_cartesian_state()
                
                # Calculate errors
                error_x = target_x - x_cur
                error_y = target_y - y_cur
                error_theta = target_theta - theta_cur
                error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))
                pos_error = np.sqrt(error_x**2 + error_y**2)
                
                trajectory.append([x_cur, y_cur, theta_cur])
                errors.append([error_x, error_y, error_theta, pos_error])
                
                # Compute desired velocities using simple proportional control
                # Velocity direction points toward target, magnitude is limited by max_velocity
                kp_vel = 1.0  # Proportional gain for velocity (can be tuned)
                kp_vel_theta = 1.0  # Proportional gain for velocity (can be tuned)
                
                vel_x = np.clip(kp_vel * error_x, -max_velocity, max_velocity)
                vel_y = np.clip(kp_vel * error_y, -max_velocity, max_velocity)
                vel_theta = np.clip(kp_vel_theta * error_theta, -max_velocity, max_velocity)
                
                target_velocities_cmd = [vel_x, vel_y, vel_theta]
                commanded_velocities.append(target_velocities_cmd)
                
                # Check if target reached
                if pos_error < tolerance and np.abs(error_theta) < np.radians(1):
                    if verbose:
                        print(f"\n✓ Target reached at step {step}")
                        print(f"  Position error: {pos_error:.6f}m")
                        print(f"  Theta error: {np.degrees(error_theta):.4f}°")
                    
                    # Record final velocities before breaking
                    vel_x_actual = p.getJointState(self.robotId, self.joint_x_idx)[1]
                    vel_y_actual = p.getJointState(self.robotId, self.joint_y_idx)[1]
                    vel_theta_actual = p.getJointState(self.robotId, self.joint_theta_idx)[1]
                    joint_velocities.append([vel_x_actual, vel_y_actual, vel_theta_actual])
                    break
                
                # Apply velocity control (not position control)
                p.setJointMotorControlArray(
                    self.robotId,
                    joint_indices,
                    p.VELOCITY_CONTROL,
                    targetVelocities=target_velocities_cmd,
                    forces=forces
                )
                
                # Step simulation
                p.stepSimulation()
                
                # Record actual joint velocities
                vel_x_actual = p.getJointState(self.robotId, self.joint_x_idx)[1]
                vel_y_actual = p.getJointState(self.robotId, self.joint_y_idx)[1]
                vel_theta_actual = p.getJointState(self.robotId, self.joint_theta_idx)[1]
                joint_velocities.append([vel_x_actual, vel_y_actual, vel_theta_actual])
                
                step += 1
            
            trajectory = np.array(trajectory)
            errors = np.array(errors)
            joint_velocities = np.array(joint_velocities)
            commanded_velocities = np.array(commanded_velocities)
            
            if verbose and step == max_steps:
                print(f"\nⓘ Max steps ({max_steps}) reached without convergence")
                print(f"  Final position error: {pos_error:.6f}m")
            
            return trajectory, errors, joint_velocities, commanded_velocities
        
        except Exception as e:
            print(f"ERROR in apply_cartesian_control_velocity_mode: {e}")
            import traceback
            traceback.print_exc()
            return np.array([]), np.array([]), np.array([]), np.array([])

    def get_top_view(self, pixel_width=320, pixel_height=320, camera_target_pos=None,
                     camera_distance=3, fov=60, aspect_ratio=1, near=0.1, far=20):
        """
        Capture a top-down view of the PyBullet environment.

        :param pixel_width: Width of rendered image (pixels)
        :param pixel_height: Height of rendered image (pixels)
        :param camera_target_pos: Target position [x, y, z]
        :param camera_distance: Distance from target
        :param fov: Field of view (degrees)
        :param aspect_ratio: Image aspect ratio
        :param near: Near clipping plane
        :param far: Far clipping plane
        :return: Numpy array (H, W, 1) grayscale image
        """
        if camera_target_pos is None:
            camera_target_pos = [1, 0, 0]

        yaw = 0
        pitch = -90.0
        roll = 0
        up_axis_index = 2

        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            camera_target_pos, camera_distance, yaw, pitch, roll, up_axis_index
        )

        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect_ratio, near, far)

        img_arr = p.getCameraImage(
            pixel_width,
            pixel_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            shadow=1,
            lightDirection=[1, 1, 1]
        )

        im = np.array(img_arr[2])
        im = im.reshape((img_arr[0], img_arr[1], 4))
        im = im[:, :, :3].astype(np.uint8)
        im = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)

        return np.expand_dims(im, 2)

    def save_top_view(self, output_dir="output", filename=None):
        """
        Capture and save a top-view workspace snapshot.

        :param output_dir: Output directory path
        :param filename: Optional custom filename (default: auto-generated timestamp)
        :return: Path to saved file
        """
        top_view_gray = self.get_top_view()

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        if filename is None:
            filename = f"workspace_top_view_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"

        snapshot_path = output_path / filename
        cv2.imwrite(str(snapshot_path), top_view_gray[:, :, 0])

        return snapshot_path

    def plot_trajectory_results(self, trajectory, errors, target, joint_velocities=None, target_velocities=None, mode='position'):
        """
        Plot trajectory results including 2D path, positions over time, orientation, errors, and joint velocities.

        :param trajectory: Trajectory array from control method (N x 3)
        :param errors: Errors array from control method (N x 4)
        :param target: Target position [x, y, theta]
        :param joint_velocities: Actual joint velocities array (N x 3)
        :param target_velocities: Target/commanded velocities array (N x 3) or list [vel_x, vel_y, vel_theta]
        :param mode: 'position' for POSITION_CONTROL or 'velocity' for VELOCITY_CONTROL
        :return: Figure object
        """
        # Determine number of subplot rows
        num_rows = 3 if joint_velocities is not None else 2
        fig, axes = plt.subplots(num_rows, 2, figsize=(12, 5*num_rows))

        # Plot 1: 2D trajectory
        ax = axes[0, 0]
        ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Trajectory')
        ax.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=10, label='Start')
        ax.plot(trajectory[-1, 0], trajectory[-1, 1], 'r*', markersize=15, label='End')
        ax.plot(target[0], target[1], 'rx', markersize=12, markeredgewidth=2, label='Target')
        ax.set_xlabel('X Position [m]')
        ax.set_ylabel('Y Position [m]')
        ax.set_title('End-Effector Trajectory in 2D Cartesian Space')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')

        # Plot 2: X and Y position over time
        ax = axes[0, 1]
        time = np.arange(len(trajectory)) * self.control_dt
        ax.plot(time, trajectory[:, 0], 'b-', label='X', linewidth=2)
        ax.plot(time, trajectory[:, 1], 'g-', label='Y', linewidth=2)
        ax.axhline(y=target[0], color='b', linestyle='--', alpha=0.5)
        ax.axhline(y=target[1], color='g', linestyle='--', alpha=0.5)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position [m]')
        ax.set_title('X and Y Positions Over Time')
        ax.grid(True, alpha=0.3)
        ax.legend()

        # Plot 3: Orientation over time
        ax = axes[1, 0]
        theta_deg = np.degrees(trajectory[:, 2])
        target_deg = np.degrees(target[2])
        ax.plot(time, theta_deg, 'r-', linewidth=2, label='Theta')
        ax.axhline(y=target_deg, color='r', linestyle='--', alpha=0.5, label='Target')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Orientation [deg]')
        ax.set_title('Orientation Over Time')
        ax.set_ylim(-180, 180)
        ax.grid(True, alpha=0.3)
        ax.legend()

        # Plot 4: Position error over time
        ax = axes[1, 1]
        ax.plot(time, errors[:, 0], 'b-', label='Error X', linewidth=2)
        ax.plot(time, errors[:, 1], 'g-', label='Error Y', linewidth=2)
        ax.plot(time, np.abs(errors[:, 2]), 'r-', label='Error Theta', linewidth=2)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Error')
        ax.set_title('Control Errors Over Time')
        ax.grid(True, alpha=0.3, which='both')
        ax.legend()

        # Plot 5 & 6: Joint velocities (if provided)
        if joint_velocities is not None:
            # Plot 5: Joint velocities over time
            ax = axes[2, 0]
            ax.plot(time, joint_velocities[:, 0], 'b-', label='Actual Vel X', linewidth=2)
            ax.plot(time, joint_velocities[:, 1], 'g-', label='Actual Vel Y', linewidth=2)
            ax.plot(time, joint_velocities[:, 2], 'r-', label='Actual Vel Theta', linewidth=2)
            
            # Add target velocity reference
            if target_velocities is not None:
                if isinstance(target_velocities, np.ndarray) and target_velocities.ndim == 2:
                    # Time-varying velocities (VELOCITY_CONTROL mode)
                    ax.plot(time, target_velocities[:, 0], 'b--', alpha=0.5, linewidth=1.5, label='Cmd Vel X')
                    ax.plot(time, target_velocities[:, 1], 'g--', alpha=0.5, linewidth=1.5, label='Cmd Vel Y')
                    ax.plot(time, target_velocities[:, 2], 'r--', alpha=0.5, linewidth=1.5, label='Cmd Vel Theta')
                else:
                    # Constant velocities (POSITION_CONTROL mode)
                    ax.axhline(y=target_velocities[0], color='b', linestyle='--', alpha=0.5, linewidth=1.5)
                    ax.axhline(y=target_velocities[1], color='g', linestyle='--', alpha=0.5, linewidth=1.5)
                    ax.axhline(y=target_velocities[2], color='r', linestyle='--', alpha=0.5, linewidth=1.5)
            
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Velocity [m/s or rad/s]')
            ax.set_title('Joint Velocities Over Time')
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

            # Plot 6: Velocity tracking error or magnitude
            ax = axes[2, 1]
            
            if target_velocities is not None and isinstance(target_velocities, np.ndarray) and target_velocities.ndim == 2:
                # VELOCITY_CONTROL: show velocity tracking error
                vel_error_x = target_velocities[:, 0] - joint_velocities[:, 0]
                vel_error_y = target_velocities[:, 1] - joint_velocities[:, 1]
                vel_error_theta = target_velocities[:, 2] - joint_velocities[:, 2]
                
                ax.plot(time, vel_error_x, 'b-', label='Error Vel X', linewidth=2)
                ax.plot(time, vel_error_y, 'g-', label='Error Vel Y', linewidth=2)
                ax.plot(time, vel_error_theta, 'r-', label='Error Vel Theta', linewidth=2)
                ax.set_ylabel('Velocity Error [m/s or rad/s]')
                ax.set_title('Velocity Tracking Error (Commanded - Actual)')
            else:
                # POSITION_CONTROL: show magnitude
                vel_magnitude = np.sqrt(joint_velocities[:, 0]**2 + joint_velocities[:, 1]**2)
                ax.plot(time, vel_magnitude, 'purple', linewidth=2, label='XY Velocity Magnitude')
                ax.plot(time, np.abs(joint_velocities[:, 2]), 'orange', linewidth=2, label='|Theta Velocity|')
                
                if target_velocities is not None:
                    target_xy_mag = target_velocities[0]
                    ax.axhline(y=target_xy_mag, color='purple', linestyle='--', alpha=0.5, linewidth=1.5)
                    ax.axhline(y=target_velocities[2], color='orange', linestyle='--', alpha=0.5, linewidth=1.5)
                
                ax.set_ylabel('Velocity Magnitude [m/s or rad/s]')
                ax.set_title('Velocity Magnitudes Over Time')
            
            ax.set_xlabel('Time [s]')
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

        plt.tight_layout()
        plt.show()

        # Print final state summary
        print(f"\nFinal State:")
        print(f"  Position: ({trajectory[-1, 0]:.4f}m, {trajectory[-1, 1]:.4f}m)")
        print(f"  Orientation: {np.degrees(trajectory[-1, 2]):.2f}°")
        if joint_velocities is not None:
            print(f"  Final Joint Velocities: X={joint_velocities[-1, 0]:.4f}, Y={joint_velocities[-1, 1]:.4f}, Theta={joint_velocities[-1, 2]:.4f}")
        if target_velocities is not None:
            if isinstance(target_velocities, np.ndarray) and target_velocities.ndim == 2:
                print(f"  Final Commanded Velocities: X={target_velocities[-1, 0]:.4f}, Y={target_velocities[-1, 1]:.4f}, Theta={target_velocities[-1, 2]:.4f}")
            else:
                print(f"  Target Velocities: X={target_velocities[0]:.4f}, Y={target_velocities[1]:.4f}, Theta={target_velocities[2]:.4f}")
        print(f"Target Position:   ({target[0]:.4f}m, {target[1]:.4f}m, {np.degrees(target[2]):.2f}°)")

        return fig


# Convenience functions for quick setup

def initialize_simulation(physics_client=None):
    """
    Initialize PyBullet simulation environment.

    :param physics_client: Optional existing physics client (default: create new GUI)
    :return: Physics client ID
    """
    if physics_client is None:
        import pybullet_data
        physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    return physics_client


def load_shovel(urdf_path="./urdf/shovel/shovelFlat.urdf", start_pos=None, start_orientation=None):
    """
    Load the shovel robot into the simulation and let it settle.

    :param urdf_path: Path to shovel URDF
    :param start_pos: Starting position [x, y, z] (default: [0, 0, 0.5])
    :param start_orientation: Starting orientation quaternion (default: identity)
    :return: PyBullet robot ID
    """
    if start_pos is None:
        start_pos = [0, 0, 0.5]
    if start_orientation is None:
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot_id = p.loadURDF(urdf_path, start_pos, start_orientation)
    
    # Simulate for 5 seconds to let robot settle
    settle_steps = int(5.0 * 240)  # 5 seconds at 240 Hz
    for _ in range(settle_steps):
        p.stepSimulation()
    
    return robot_id


def scatter_pebble_pos(pebble_urdf="./urdf/pebbles/pebbles.urdf",
                   pos=None, settle_steps=120):
    """
    Scatter pebbles randomly in the workspace.

    :param num_pebbles: Number of pebbles to scatter
    :param pebble_urdf: Path to pebble URDF
    :param min_pos: Minimum position [x, y, z]
    :param max_pos: Maximum position [x, y, z]
    :param settle_steps: Number of simulation steps for pebbles to settle
    :return: List of pebble IDs
    """
    if pos is None:
        pos = np.array([0.2, 0.0, 0.0])

    pos = np.array(pos)

    start_pos = pos
    start_pos[-1] = 0.2
    start_quat = p.getQuaternionFromEuler([0, 0, 0])
    pebble_id = p.loadURDF(pebble_urdf, start_pos, start_quat)

    for _ in range(settle_steps):
        p.stepSimulation()

    return pebble_id

def scatter_pebbles(num_pebbles=10, pebble_urdf="./urdf/pebbles/pebbles.urdf",
                   min_pos=None, max_pos=None, settle_steps=120):
    """
    Scatter pebbles randomly in the workspace.

    :param num_pebbles: Number of pebbles to scatter
    :param pebble_urdf: Path to pebble URDF
    :param min_pos: Minimum position [x, y, z]
    :param max_pos: Maximum position [x, y, z]
    :param settle_steps: Number of simulation steps for pebbles to settle
    :return: List of pebble IDs
    """
    if min_pos is None:
        min_pos = np.array([0.2, -1.5, 0.0])
    if max_pos is None:
        max_pos = np.array([3.0, 1.5, 0.0])

    min_pos = np.array(min_pos)
    max_pos = np.array(max_pos)

    pebble_ids = []
    for _ in range(num_pebbles):
        start_pos = min_pos + (np.random.rand(3) * (max_pos - min_pos))
        start_pos[-1] = 0.2
        start_quat = p.getQuaternionFromEuler([0, 0, 0])
        pebble_id = p.loadURDF(pebble_urdf, start_pos, start_quat)
        pebble_ids.append(pebble_id)

    for _ in range(settle_steps):
        p.stepSimulation()

    return pebble_ids
