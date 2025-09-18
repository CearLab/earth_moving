import math, os
from engine_rover import PyBulletEnvironment
import pybullet as p

def quat_from(theta):
    return p.getQuaternionFromEuler([0, 0, theta])

# Initial robot drop pose
robot_pose = (-1.0, -1.0, math.pi / 2)

# Task trajectory start and end
task_start_pose = (0.0, 0.0, 0.0)
task_end_pose = (0.5, 0.5, 3 * math.pi / 4)

robot_urdf = os.path.join(os.path.dirname(__file__), "2_wheel_rover.urdf")
env = PyBulletEnvironment(gui=True)
env.open_environment(robot_urdf=robot_urdf,
                     init_pos=robot_pose[:2] + (0,),
                     init_quat=quat_from(robot_pose[2]))
env.set_robot_dim(L=0.2, R=0.05)

# 1. Generate from robot to task start
pre_traj = env.generate_bezier_trajectory(robot_pose, task_start_pose, duration=3.0)

# 2. Generate task trajectory
task_traj = env.generate_bezier_trajectory(task_start_pose, task_end_pose, duration=4.0)

# 3. Combine and execute
full_traj = pre_traj + task_traj
env.draw_trajectory(full_traj)
env.follow_smooth_trajectory(full_traj)

input("Press ENTER to exit...")
env.close_environment()
