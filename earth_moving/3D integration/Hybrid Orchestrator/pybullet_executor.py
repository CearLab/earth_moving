"""
pybullet_executor.py - Low-level PyBullet control for Hybrid Orchestrator

This module handles:
- PyBullet environment initialization
- Rover state reading from physics
- Wheel velocity commands (differential drive)
- Pebble spawning and position tracking
- Grid overlay and target zone visualization
- Smooth trajectory generation using spillage model
"""

import os
import math
import numpy as np
import pybullet as p
import pybullet_data

# Import spillage model for smooth trajectory generation
from spillage_model import smooth_path_with_spline


def yaw_to_quat(yaw):
    """Convert yaw angle (radians) to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def get_state_from_bullet(body_id):
    """
    Get robot state from PyBullet.

    Returns:
        np.array([x, y, yaw, v_forward, w_yaw])
    """
    pos, orn = p.getBasePositionAndOrientation(body_id)
    x, y, z = pos
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    lin_vel, ang_vel = p.getBaseVelocity(body_id)
    vx, vy, vz = lin_vel
    wz = ang_vel[2]
    v_forward = math.cos(yaw) * vx + math.sin(yaw) * vy
    w_yaw = wz
    return np.array([x, y, yaw, v_forward, w_yaw])


def find_wheel_joints(body_id):
    """
    Find left and right wheel joint indices.

    Returns:
        (left_joint, right_joint) indices
    """
    left = right = None
    n_joints = p.getNumJoints(body_id)
    for ji in range(n_joints):
        info = p.getJointInfo(body_id, ji)
        name = info[1].decode("utf-8")
        if name == "base_to_lwheel":
            left = ji
        elif name == "base_to_rwheel":
            right = ji
    if left is None or right is None:
        raise RuntimeError("Could not find base_to_lwheel / base_to_rwheel in URDF")
    return left, right


def apply_diff_drive_control(agent,
                             wheel_radius=0.07,
                             track_width=0.20,
                             max_wheel_speed=20.0,
                             max_torque=5.0):
    """
    Map (v, w) to left/right wheel angular velocities and send VELOCITY_CONTROL.

    Args:
        agent: dict with "body", "left_joint", "right_joint", "control"
        wheel_radius: wheel radius in meters
        track_width: distance between wheels in meters
        max_wheel_speed: maximum wheel angular velocity
        max_torque: maximum motor torque

    Note: Sign flip so that v_cmd>0 moves the rover forward with this URDF.
    """
    v_cmd, w_cmd = agent["control"]

    vL = v_cmd - w_cmd * track_width / 2.0
    vR = v_cmd + w_cmd * track_width / 2.0

    wL = -vL / wheel_radius
    wR = -vR / wheel_radius

    wL = max(min(wL, max_wheel_speed), -max_wheel_speed)
    wR = max(min(wR, max_wheel_speed), -max_wheel_speed)

    p.setJointMotorControl2(agent["body"], agent["left_joint"],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=wL,
                            force=max_torque)
    p.setJointMotorControl2(agent["body"], agent["right_joint"],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=wR,
                            force=max_torque)


class PyBulletExecutor:
    """
    Manages PyBullet simulation environment and low-level robot control.
    """

    def __init__(self,
                 env_radius=3.0,
                 target_zone_radius=0.3,
                 num_pebbles=50,
                 random_seed=41,
                 gui=True,
                 initial_robot_poses=None,
                 rover_urdf_path=None,
                 pebble_urdf_path=None,
                 coord_converter=None):
        """
        Initialize PyBullet executor.

        Args:
            env_radius: environment radius in meters
            target_zone_radius: target zone radius in meters
            num_pebbles: number of pebbles to spawn
            random_seed: random seed for pebble placement
            gui: whether to show GUI
            initial_robot_poses: list of (x, y, yaw) initial poses. Defaults to one rover at (0,0,0)
            rover_urdf_path: path to rover URDF (default: same folder)
            pebble_urdf_path: path to pebble URDF (default: same folder)
            coord_converter: CoordinateConverter instance for 2D/3D conversion
        """
        self.env_radius = env_radius
        self.target_zone_radius = target_zone_radius
        self.num_pebbles = num_pebbles
        self.random_seed = random_seed
        self.gui = gui
        
        if initial_robot_poses is None:
            self.initial_robot_poses = [(0.0, 0.0, 0.0)]
        else:
            self.initial_robot_poses = initial_robot_poses
            
        self.coord_converter = coord_converter

        # Default URDF paths
        here = os.path.dirname(os.path.abspath(__file__))
        self.rover_urdf_path = rover_urdf_path or os.path.join(here, "2_wheel_rover.urdf")
        self.pebble_urdf_path = pebble_urdf_path or os.path.join(here, "pebbles.urdf")

        # Simulation parameters
        self.sim_dt = 1.0 / 240.0
        self.pebble_radius = 0.05

        # State
        self.physics_client = None
        self.plane_id = None
        self.agents = []  # Changed to list of agents
        self.pebble_ids = []
        self.pebble_centers = []
        self.target_zone_id = None
        self.grid_lines = []

        # Robot shape for ORCA
        self.rover_shape = {
            "circles": [
                (+0.16, 0.0, 0.30),
                (-0.16, 0.0, 0.30),
            ]
        }
        
        # Colors for different agents
        self.agent_colors = [
            (0.1, 0.5, 1.0, 1.0),  # Blue
            (1.0, 0.5, 0.1, 1.0),  # Orange
            (0.1, 1.0, 0.5, 1.0),  # Green
            (1.0, 0.1, 0.5, 1.0),  # Pink
            (0.5, 0.1, 1.0, 1.0),  # Purple
        ]

    def initialize(self):
        """Initialize PyBullet environment and spawn objects."""
        # Connect to PyBullet
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(
            cameraDistance=5.0,
            cameraYaw=45,
            cameraPitch=-60,
            cameraTargetPosition=[0, 0, 0]
        )
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_dt)

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, lateralFriction=1.0)

        # Spawn rovers
        self._spawn_rovers()

        # Spawn pebbles
        self._spawn_pebbles()

        return self.agents, self.pebble_centers

    def _spawn_rovers(self):
        """Spawn the rovers at initial poses."""
        self.agents = []
        
        for i, (x, y, yaw) in enumerate(self.initial_robot_poses):
            body_id = p.loadURDF(
                self.rover_urdf_path,
                basePosition=[x, y, 0.02],
                baseOrientation=yaw_to_quat(yaw),
                useFixedBase=False,
            )
            left_j, right_j = find_wheel_joints(body_id)
            
            color = self.agent_colors[i % len(self.agent_colors)]
            
            agent = {
                "id": f"R{i}",
                "index": i,  # Add index for easy reference
                "state": np.array([x, y, yaw, 0.0, 0.0]),
                "goal": np.array([0.0, 0.0]),
                "control": (0.0, 0.0),
                "body": body_id,
                "left_joint": left_j,
                "right_joint": right_j,
                "shape": self.rover_shape,
                "color": color,
            }
            self.agents.append(agent)

            # Set rover color
            r, g, b, a = color
            p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, a])

            # Set friction
            p.changeDynamics(agent["body"], -1, lateralFriction=0.8)
            for link in (left_j, right_j):
                p.changeDynamics(
                    agent["body"], link,
                    lateralFriction=1.0,
                    rollingFriction=0.0,
                    spinningFriction=0.0,
                )

            # Disable default motors
            for j in (left_j, right_j):
                p.setJointMotorControl2(
                    agent["body"], j,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=0.0,
                    force=0.0
                )
        
        # Keep backwards compatibility for single-agent interface
        self.agent = self.agents[0] if self.agents else None

    def _spawn_pebbles(self):
        """Spawn pebbles randomly within environment radius."""
        np.random.seed(self.random_seed)
        self.pebble_ids = []
        self.pebble_centers = []

        for i in range(self.num_pebbles):
            r_rand = self.env_radius * math.sqrt(np.random.rand())
            phi = 2 * math.pi * np.random.rand()
            px = r_rand * math.cos(phi)
            py = r_rand * math.sin(phi)

            bid = p.loadURDF(
                self.pebble_urdf_path,
                basePosition=[px, py, 0.01],
                baseOrientation=[0, 0, 0, 1],
                useFixedBase=False,
                globalScaling=1.0
            )
            self.pebble_ids.append(bid)
            self.pebble_centers.append((px, py))
            p.changeDynamics(bid, -1, lateralFriction=0.8)

    def update_state(self, agent_idx=0):
        """Update agent state from PyBullet physics."""
        agent = self.agents[agent_idx]
        agent["state"] = get_state_from_bullet(agent["body"])
        return agent["state"]
        
    def update_all_states(self):
        """Update all agents' states from PyBullet physics."""
        states = []
        for agent in self.agents:
            agent["state"] = get_state_from_bullet(agent["body"])
            states.append(agent["state"])
        return states

    def get_pebble_positions(self):
        """Get current pebble positions from PyBullet."""
        positions = []
        for bid in self.pebble_ids:
            pos, _ = p.getBasePositionAndOrientation(bid)
            positions.append((pos[0], pos[1]))
        return positions

    def apply_control(self, v_cmd, w_cmd, agent_idx=0):
        """Apply velocity control commands to a specific rover."""
        agent = self.agents[agent_idx]
        agent["control"] = (v_cmd, w_cmd)
        apply_diff_drive_control(agent)

    def step(self):
        """Step the physics simulation."""
        p.stepSimulation()

    def visualize_goal(self, goal_pos, color=(0, 1, 0, 1)):
        """Add a goal marker in PyBullet."""
        goal_vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=0.08,
            rgbaColor=color
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=goal_vis,
            basePosition=[goal_pos[0], goal_pos[1], 0.05],
        )

    def visualize_path(self, path_points, color=(1, 1, 0), line_width=2.0, life_time=0.0):
        """Draw a path in PyBullet."""
        for i in range(len(path_points) - 1):
            a = path_points[i]
            b = path_points[i + 1]
            p.addUserDebugLine(
                [a[0], a[1], 0.03],
                [b[0], b[1], 0.03],
                color,
                lineWidth=line_width,
                lifeTime=life_time
            )

    def disconnect(self):
        """Disconnect from PyBullet."""
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None

    def is_connected(self):
        """Check if PyBullet is still connected."""
        return p.isConnected()

    def create_target_zone(self):
        """Create target zone visualization in PyBullet."""
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

    def create_grid_overlay(self, show_grid=True):
        """Create grid overlay visualization in PyBullet."""
        # Clear existing grid lines
        for line_id in self.grid_lines:
            p.removeUserDebugItem(line_id)
        self.grid_lines = []

        if not show_grid:
            return

        if self.coord_converter is None:
            print("[PyBulletExecutor] Warning: No coord_converter, cannot draw grid")
            return

        grid_size = self.coord_converter.grid_size
        cell_size = self.coord_converter.cell_size

        # Draw grid lines
        for i in range(grid_size + 1):
            # Vertical lines
            x = -self.env_radius + i * cell_size
            line_id = p.addUserDebugLine(
                [x, -self.env_radius, 0.01],
                [x, self.env_radius, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

            # Horizontal lines
            y = -self.env_radius + i * cell_size
            line_id = p.addUserDebugLine(
                [-self.env_radius, y, 0.01],
                [self.env_radius, y, 0.01],
                [0.5, 0.5, 0.5, 0.5],
                2.0
            )
            self.grid_lines.append(line_id)

    def initialize_grid_overlay(self, show_grid=True):
        """Initialize grid overlay based on configuration."""
        self.create_grid_overlay(show_grid=show_grid)

    def toggle_grid_overlay(self, show_grid):
        """Toggle grid overlay visibility."""
        self.create_grid_overlay(show_grid=show_grid)
        print(f"3D Grid visualization: {'ENABLED' if show_grid else 'DISABLED'}")

    def generate_smooth_trajectory(self, grid_waypoints, target_spacing=0.03,
                                   smoothing_factor=0.5, num_points=1000):
        """
        Generate smooth spline trajectory using the spillage model.

        Args:
            grid_waypoints: List of (grid_x, grid_y) tuples from 2D path planning
            target_spacing: Desired spacing between final trajectory points (meters)
            smoothing_factor: Spline smoothing factor (0.0=sharp, 1.0=smooth)
            num_points: Number of points for initial spline generation

        Returns:
            List of (world_x, world_y, z) tuples for smooth 3D trajectory
        """
        if self.coord_converter is None:
            print("[PyBulletExecutor] Warning: No coord_converter, returning direct conversion")
            return [(x, y, 0.0) for x, y in grid_waypoints]

        print(f"Generating smooth trajectory: {len(grid_waypoints)} waypoints")
        print(f"  Parameters: smoothing={smoothing_factor}, points={num_points}, spacing={target_spacing}m")

        # Use spillage model spline generation
        spline_points, curvature, success = smooth_path_with_spline(
            waypoints=grid_waypoints,
            smoothing_factor=smoothing_factor,
            num_points=num_points
        )

        if not success or not spline_points:
            print("  Warning: Spline generation failed, using fallback")
            trajectory = []
            for gx, gy in grid_waypoints:
                wx, wy = self.coord_converter.convert_2d_to_3d(gx, gy)
                trajectory.append((wx, wy, 0.0))
            return trajectory

        print(f"  Generated {len(spline_points)} spline points")

        # Convert spline points from 2D grid to 3D world coordinates
        world_spline_points = []
        for grid_x, grid_y in spline_points:
            world_x, world_y = self.coord_converter.convert_2d_to_3d(grid_x, grid_y)
            world_spline_points.append((world_x, world_y, 0.0))

        # Resample to target spacing
        if target_spacing > 0:
            resampled = self._resample_trajectory(world_spline_points, target_spacing)
            print(f"  Resampled to {len(resampled)} points at {target_spacing}m spacing")
            return resampled
        else:
            return world_spline_points

    def _resample_trajectory(self, trajectory_points, target_spacing):
        """Resample trajectory to achieve target spacing."""
        if len(trajectory_points) < 2:
            return trajectory_points

        # Calculate cumulative distances
        distances = [0.0]
        for i in range(1, len(trajectory_points)):
            p1 = trajectory_points[i - 1]
            p2 = trajectory_points[i]
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            distances.append(distances[-1] + dist)

        total_length = distances[-1]
        if total_length < target_spacing:
            return trajectory_points

        # Generate resampled points
        resampled = []
        num_segments = max(2, int(total_length / target_spacing) + 1)

        for i in range(num_segments):
            target_dist = i * total_length / (num_segments - 1) if num_segments > 1 else 0

            # Find segment containing target distance
            segment_idx = 0
            while segment_idx < len(distances) - 1 and distances[segment_idx + 1] < target_dist:
                segment_idx += 1

            if segment_idx >= len(trajectory_points) - 1:
                resampled.append(trajectory_points[-1])
                continue

            # Interpolate within segment
            p1 = trajectory_points[segment_idx]
            p2 = trajectory_points[segment_idx + 1]

            if distances[segment_idx + 1] == distances[segment_idx]:
                resampled.append(p1)
            else:
                t = (target_dist - distances[segment_idx]) / (distances[segment_idx + 1] - distances[segment_idx])
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                z = p1[2] + t * (p2[2] - p1[2])
                resampled.append((x, y, z))

        return resampled

    def set_wheel_speeds(self, v_cmd, w_cmd, dt):
        """
        Apply wheel speed control and step physics.

        Args:
            v_cmd: Forward velocity command
            w_cmd: Angular velocity command
            dt: Time step
        """
        self.apply_control(v_cmd, w_cmd)
        # Step physics for dt seconds
        num_steps = max(1, int(dt / self.sim_dt))
        for _ in range(num_steps):
            p.stepSimulation()

    def clear_debug_items(self):
        """Clear all debug visualization items."""
        p.removeAllUserDebugItems()
        self.grid_lines = []
