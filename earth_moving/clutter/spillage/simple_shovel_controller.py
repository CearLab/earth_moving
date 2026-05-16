import numpy as np
import pybullet as p


class SimpleShovelController:
    def __init__(self, robot_id):
        self.robotId = robot_id
        self._discover_joints()
        self.base_pos, _ = p.getBasePositionAndOrientation(self.robotId)
        self.base_pos = np.array(self.base_pos, dtype=np.float32)
        self._calibrate_world_joint_map()

    def _discover_joints(self):
        num_joints = p.getNumJoints(self.robotId)
        prismatic = []
        revolute = []
        self.joint_limits = {}

        for joint_idx in range(num_joints):
            info = p.getJointInfo(self.robotId, joint_idx)
            joint_type = info[2]
            if joint_type == p.JOINT_PRISMATIC:
                prismatic.append(joint_idx)
            elif joint_type == p.JOINT_REVOLUTE:
                revolute.append(joint_idx)

            self.joint_limits[joint_idx] = {
                "lower": info[8],
                "upper": info[9],
                "max_force": info[10],
                "max_velocity": info[11],
            }

        if len(prismatic) < 2 or len(revolute) < 1:
            raise ValueError("Expected at least two prismatic joints and one revolute joint")

        self.joint_x_idx = prismatic[0]
        self.joint_y_idx = prismatic[1]
        self.joint_theta_idx = revolute[-1]

    def _wrap_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _calibrate_world_joint_map(self):
        """Calibrate a linear map between world XY/theta and joint commands.

        The URDF uses shifted link origins, so base pose is not a reliable
        world-to-joint reference. We measure FK responses directly.
        """
        num_joints = p.getNumJoints(self.robotId)
        saved_states = [p.getJointState(self.robotId, j)[0] for j in range(num_joints)]

        def _set_controlled(qx, qy, qtheta):
            p.resetJointState(self.robotId, self.joint_x_idx, float(qx))
            p.resetJointState(self.robotId, self.joint_y_idx, float(qy))
            p.resetJointState(self.robotId, self.joint_theta_idx, float(qtheta))
            p.stepSimulation()

        try:
            _set_controlled(0.0, 0.0, 0.0)
            ee0, th0 = self.forward_kinematics_2d()

            _set_controlled(1.0, 0.0, 0.0)
            ee_x, _ = self.forward_kinematics_2d()

            _set_controlled(0.0, 1.0, 0.0)
            ee_y, _ = self.forward_kinematics_2d()

            _set_controlled(0.0, 0.0, 1.0)
            _, th1 = self.forward_kinematics_2d()

            self.world_x_origin = float(ee0[0])
            self.world_y_origin = float(ee0[1])
            self.theta_origin = float(th0)

            self.world_per_joint_x = float(ee_x[0] - ee0[0])
            self.world_per_joint_y = float(ee_y[1] - ee0[1])
            self.theta_per_joint = float(self._wrap_angle(th1 - th0))

            if abs(self.world_per_joint_x) < 1e-6:
                self.world_per_joint_x = 1.0
            if abs(self.world_per_joint_y) < 1e-6:
                self.world_per_joint_y = 1.0
            if abs(self.theta_per_joint) < 1e-6:
                self.theta_per_joint = 1.0
        finally:
            for j, q in enumerate(saved_states):
                p.resetJointState(self.robotId, j, float(q))
            p.stepSimulation()

    def world_to_joint(self, target_x, target_y, target_theta):
        return (
            float((target_x - self.world_x_origin) / self.world_per_joint_x),
            float((target_y - self.world_y_origin) / self.world_per_joint_y),
            float(self._wrap_angle(target_theta - self.theta_origin) / self.theta_per_joint),
        )

    def _command_joint_targets(self, joint_x, joint_y, joint_theta, max_force=100):
        joint_indices = [self.joint_x_idx, self.joint_y_idx, self.joint_theta_idx]
        target_positions = [joint_x, joint_y, joint_theta]
        forces = []

        for i, (joint_idx, target) in enumerate(zip(joint_indices, target_positions)):
            limits = self.joint_limits[joint_idx]
            clamped = float(np.clip(target, limits["lower"], limits["upper"]))
            target_positions[i] = clamped
            forces.append(min(limits["max_force"], max_force))

        p.setJointMotorControlArray(
            self.robotId,
            joint_indices,
            p.POSITION_CONTROL,
            targetPositions=target_positions,
            forces=forces,
        )

    def reset_world_pose(self, target_x, target_y, target_theta):
        joint_x, joint_y, joint_theta = self.world_to_joint(target_x, target_y, target_theta)
        p.resetJointState(self.robotId, self.joint_x_idx, joint_x)
        p.resetJointState(self.robotId, self.joint_y_idx, joint_y)
        p.resetJointState(self.robotId, self.joint_theta_idx, joint_theta)

    def forward_kinematics_2d(self):
        link_state = p.getLinkState(self.robotId, p.getNumJoints(self.robotId) - 1)
        ee_pos = np.array(link_state[0], dtype=np.float32)
        ee_quat = link_state[1]
        ee_theta = p.getEulerFromQuaternion(ee_quat)[2]
        return ee_pos, float(ee_theta)

    def get_current_cartesian_state(self):
        ee_pos, ee_theta = self.forward_kinematics_2d()
        return float(ee_pos[0]), float(ee_pos[1]), float(ee_theta), float(ee_pos[2])

    def apply_cartesian_control_sequential(
        self,
        target_x,
        target_y,
        target_theta,
        max_force=100,
        max_velocity=0.5,
        total_steps=240,
        verbose=True,
        step_callback=None,
        callback_context=None,
    ):
        start_pos, start_theta = self.forward_kinematics_2d()
        start = np.array([start_pos[0], start_pos[1], start_theta], dtype=np.float32)
        target = np.array([target_x, target_y, target_theta], dtype=np.float32)
        delta = target - start
        delta[2] = self._wrap_angle(target[2] - start[2])

        trajectory = []
        errors = []
        z_positions = []
        joint_velocities = []
        target_velocities = [max_velocity, max_velocity, max_velocity]

        if verbose:
            print(f"Direct controller moving to X={target_x:.4f}, Y={target_y:.4f}, Theta={target_theta:.4f}")

        step_count = max(1, int(total_steps))
        for alpha in np.linspace(0.0, 1.0, step_count):
            pose = start + alpha * delta
            joint_x, joint_y, joint_theta = self.world_to_joint(pose[0], pose[1], pose[2])
            self._command_joint_targets(joint_x, joint_y, joint_theta, max_force=max_force)
            p.stepSimulation()

            x_cur, y_cur, theta_cur, z_cur = self.get_current_cartesian_state()
            theta_err = self._wrap_angle(target[2] - theta_cur)
            pos_err = float(np.hypot(target[0] - x_cur, target[1] - y_cur))

            step_info = {
                "alpha": float(alpha),
                "target_world_x": float(target[0]),
                "target_world_y": float(target[1]),
                "target_world_theta": float(target[2]),
                "command_joint_x": float(joint_x),
                "command_joint_y": float(joint_y),
                "command_joint_theta": float(joint_theta),
                "shovel_x": float(x_cur),
                "shovel_y": float(y_cur),
                "shovel_z": float(z_cur),
                "shovel_theta": float(theta_cur),
                "error_x": float(target[0] - x_cur),
                "error_y": float(target[1] - y_cur),
                "error_theta": float(theta_err),
                "error_xy": float(pos_err),
                "joint_x_pos": float(p.getJointState(self.robotId, self.joint_x_idx)[0]),
                "joint_y_pos": float(p.getJointState(self.robotId, self.joint_y_idx)[0]),
                "joint_theta_pos": float(p.getJointState(self.robotId, self.joint_theta_idx)[0]),
                "joint_x_vel": float(p.getJointState(self.robotId, self.joint_x_idx)[1]),
                "joint_y_vel": float(p.getJointState(self.robotId, self.joint_y_idx)[1]),
                "joint_theta_vel": float(p.getJointState(self.robotId, self.joint_theta_idx)[1]),
                "base_x": float(self.base_pos[0]),
                "base_y": float(self.base_pos[1]),
                "base_z": float(self.base_pos[2]),
            }

            if callback_context is not None:
                step_info.update(callback_context)
            if step_callback is not None:
                step_callback(step_info)

            trajectory.append([x_cur, y_cur, theta_cur])
            errors.append([target[0] - x_cur, target[1] - y_cur, theta_err, pos_err])
            z_positions.append(z_cur)
            joint_velocities.append([
                p.getJointState(self.robotId, self.joint_x_idx)[1],
                p.getJointState(self.robotId, self.joint_y_idx)[1],
                p.getJointState(self.robotId, self.joint_theta_idx)[1],
            ])

        return (
            np.asarray(trajectory, dtype=np.float32),
            np.asarray(errors, dtype=np.float32),
            np.asarray(joint_velocities, dtype=np.float32),
            target_velocities,
        )