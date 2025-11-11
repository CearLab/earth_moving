"""
Hybrid Collision Avoidance Simulation

Main script that integrates:
  - PyBullet physics simulation
  - Multi-rover state management
  - Hybrid planner (ORCA + APF)
  - Real-time visualization and logging
"""

import os
import sys
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

from hybrid_planner import HybridCollisionAvoidance, ORCAPlanner, APFNavigator
from scenarios import get_scenario, list_scenarios


# ============================================================================
#  CONFIGURATION
# ============================================================================

# Scenario selection (change this to test different scenarios)
SCENARIO_NAME = "4_crossing_obstacles"

# Simulation parameters
SIM_DT = 1.0 / 240.0           # Physics timestep (240 Hz)
PLAN_DT = 0.10                 # Planning timestep (10 Hz)
MAX_SIM_TIME = 30.0            # Maximum simulation time (seconds)
NEAR_RADIUS = 10.0             # Rover neighbor detection range (meters)

# Environment parameters
PEBBLE_RADIUS = 0.05           # Obstacle size (meters)
ROVER_SHAPE = {
    "circles": [
        (+0.16, 0.0, 0.30),    # Front wheel
        (-0.16, 0.0, 0.30),    # Rear wheel
    ]
}

# Hybrid planner parameters
ORCA_PARAMS = {
    "tau": 8.0,
    "v_pref": 0.8,
    "v_max": 1.0,
    "w_max": 3.0,
    "k_theta": 3.0,
    "turn_in_place_deg": 45.0,
    "n_speed": 6,
    "n_angle": 20,
}

APF_PARAMS = {
    "k_att": 1.5,
    "k_rep": 0.08,
    "d0": 0.55,
    "v_max": 1.2,
    "w_max": 3.5,
    "k_w": 3.0,
}


# ============================================================================
#  PYBULLET UTILITIES
# ============================================================================

def yaw_to_quat(yaw):
    """Convert yaw angle to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def get_state_from_bullet(body_id):
    """Extract rover state from PyBullet."""
    pos, orn = p.getBasePositionAndOrientation(body_id)
    x, y, z = pos
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    lin_vel, ang_vel = p.getBaseVelocity(body_id)
    vx, vy, vz = lin_vel
    wz = ang_vel[2]

    # Forward velocity in rover frame
    v_forward = math.cos(yaw) * vx + math.sin(yaw) * vy
    w_yaw = wz

    return np.array([x, y, yaw, v_forward, w_yaw])


def find_wheel_joints(body_id):
    """Find left and right wheel joints in rover URDF."""
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
        raise RuntimeError("Could not find wheel joints in URDF")

    return left, right


def apply_diff_drive_control(agent,
                             wheel_radius=0.07,
                             track_width=0.20,
                             max_wheel_speed=20.0,
                             max_torque=5.0):
    """
    Apply differential drive control to rover.

    Converts (v, omega) commands to left/right wheel velocities.
    """
    v_cmd, w_cmd = agent["control"]

    vL = v_cmd - w_cmd * track_width / 2.0
    vR = v_cmd + w_cmd * track_width / 2.0

    # Sign flip for URDF compatibility
    wL = -vL / wheel_radius
    wR = -vR / wheel_radius

    # Saturate wheel speeds
    wL = max(min(wL, max_wheel_speed), -max_wheel_speed)
    wR = max(min(wR, max_wheel_speed), -max_wheel_speed)

    # Apply velocity control
    p.setJointMotorControl2(
        agent["body"], agent["left_joint"],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=wL,
        force=max_torque
    )
    p.setJointMotorControl2(
        agent["body"], agent["right_joint"],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=wR,
        force=max_torque
    )


# ============================================================================
#  MAIN SIMULATION
# ============================================================================

class HybridSimulation:
    """Main simulation class that manages everything."""

    def __init__(self, scenario_name: str, gui: bool = True, verbose: bool = True):
        """Initialize simulation."""
        self.scenario_name = scenario_name
        self.gui = gui
        self.verbose = verbose
        self.scenario = get_scenario(scenario_name)

        # State
        self.rovers = []
        self.pebbles = []
        self.hybrid_planner = None
        self.sim_time = 0.0
        self.plan_acc = 0.0

        # Statistics
        self.stats = {
            "collisions": 0,
            "goals_reached": 0,
            "completion_times": {},
        }

    def setup(self):
        """Initialize PyBullet and simulation environment."""
        # Connect to PyBullet
        if self.gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(
            cameraDistance=12,
            cameraYaw=45,
            cameraPitch=-40,
            cameraTargetPosition=[0, 0, 0]
        )

        # Physics setup
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(SIM_DT)

        # Load plane
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        # Spawn rovers
        here = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(here, "2_wheel_rover.urdf")

        for i, cfg in enumerate(self.scenario["rovers"]):
            sx, sy, sz = cfg["start_pos"]
            th0 = cfg["start_heading"]
            gx, gy = cfg["goal_pos"]
            color = cfg.get("color", (0.7, 0.7, 0.7, 1.0))
            priority = cfg.get("priority", i)

            rover = {
                "id": f"R{i}",
                "priority": priority,
                "is_blind": False,
                "state": np.array([sx, sy, th0, 0.0, 0.0]),
                "goal": np.array([gx, gy]),
                "control": (0.0, 0.0),
                "shape": ROVER_SHAPE,
                "color": color,
            }

            # Load URDF
            body_id = p.loadURDF(
                urdf_path,
                basePosition=[sx, sy, 0.02],
                baseOrientation=yaw_to_quat(th0),
                useFixedBase=False,
            )
            left, right = find_wheel_joints(body_id)
            rover["body"] = body_id
            rover["left_joint"] = left
            rover["right_joint"] = right

            # Set color
            r, g, b, a = color
            p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, a])

            # Set friction
            p.changeDynamics(body_id, -1, lateralFriction=0.8)
            for link in (left, right):
                p.changeDynamics(
                    body_id, link,
                    lateralFriction=1.0,
                    rollingFriction=0.0,
                    spinningFriction=0.0,
                )

            self.rovers.append(rover)

        # Spawn pebbles
        here = os.path.dirname(os.path.abspath(__file__))
        pebble_urdf = os.path.join(here, "pebbles.urdf")

        for i, (px, py) in enumerate(self.scenario.get("pebbles", [])):
            pebble_id = p.loadURDF(
                pebble_urdf,
                basePosition=[px, py, 0.025],
                useFixedBase=False,
            )
            p.changeDynamics(pebble_id, -1, lateralFriction=0.5)
            self.pebbles.append({
                "id": f"P{i}",
                "body": pebble_id,
                "radius": PEBBLE_RADIUS,
                "pos": np.array([px, py]),
            })

        # Initialize hybrid planner
        self.hybrid_planner = HybridCollisionAvoidance(
            orca_params=ORCA_PARAMS,
            apf_params=APF_PARAMS,
        )

        if self.verbose:
            print(f"\n{'='*70}")
            print(f"HYBRID COLLISION AVOIDANCE SIMULATION")
            print(f"{'='*70}")
            print(f"Scenario: {self.scenario['name']}")
            print(f"Description: {self.scenario['description']}")
            print(f"Rovers: {len(self.rovers)}")
            print(f"Pebbles: {len(self.pebbles)}")
            print(f"{'='*70}\n")

    def get_obstacle_positions(self):
        """Get current pebble positions as (x, y, radius) tuples."""
        obstacles = []
        for pebble in self.pebbles:
            pos, _ = p.getBasePositionAndOrientation(pebble["body"])
            x, y = pos[0], pos[1]
            obstacles.append((x, y, PEBBLE_RADIUS))
        return obstacles

    def plan_step(self):
        """Execute one planning cycle."""
        # Update states from physics
        for rover in self.rovers:
            rover["state"] = get_state_from_bullet(rover["body"])

        # Get obstacle positions
        obstacles = self.get_obstacle_positions()

        # Plan for each rover
        for rover in self.rovers:
            ego = rover
            goal = rover["goal"]

            # Get neighboring rovers
            neighbors = []
            for other in self.rovers:
                if other["id"] == rover["id"]:
                    continue

                dx = other["state"][0] - rover["state"][0]
                dy = other["state"][1] - rover["state"][1]

                if dx * dx + dy * dy < NEAR_RADIUS ** 2:
                    neighbors.append({
                        "id": other["id"],
                        "priority": other["priority"],
                        "is_blind": other.get("is_blind", False),
                        "state": other["state"].copy(),
                        "shape": other["shape"],
                    })

            # Hybrid planning
            v_cmd, w_cmd = self.hybrid_planner.plan(
                ego=ego,
                goal=goal,
                rover_neighbors=neighbors,
                rover_shape=rover["shape"],
                static_obstacles=obstacles,
            )

            rover["control"] = (v_cmd, w_cmd)

    def control_step(self):
        """Apply control commands to all rovers."""
        for rover in self.rovers:
            apply_diff_drive_control(rover)

    def step(self, dt: float):
        """Execute one simulation step."""
        p.stepSimulation()
        self.sim_time += dt
        self.plan_acc += dt

        # Planning cycle every PLAN_DT
        if self.plan_acc >= PLAN_DT:
            self.plan_acc = 0.0
            self.plan_step()

        # Control every step
        self.control_step()

    def check_goals(self):
        """Check if rovers have reached their goals."""
        for rover in self.rovers:
            if rover["id"] in self.stats["completion_times"]:
                continue  # Already reached

            pos = rover["state"][:2]
            goal = rover["goal"]
            dist = np.linalg.norm(pos - goal)

            if dist < 0.15:  # Goal tolerance
                self.stats["completion_times"][rover["id"]] = self.sim_time
                self.stats["goals_reached"] += 1
                if self.verbose:
                    print(f"[{self.sim_time:.2f}s] {rover['id']} reached goal!")

    def print_status(self):
        """Print simulation status."""
        if self.verbose and self.sim_time % 1.0 < SIM_DT:  # ~once per second
            msg = " | ".join(
                f"{rover['id']} v,w=({rover['control'][0]:.2f},{rover['control'][1]:.2f})"
                for rover in self.rovers
            )
            print(f"[{self.sim_time:.2f}s] {msg}")

    def run(self, max_time: float = MAX_SIM_TIME):
        """Run simulation."""
        self.setup()

        print("\nStarting simulation... (close PyBullet window to stop)")

        t_start = time.time()
        while p.isConnected() and self.sim_time < max_time:
            self.step(SIM_DT)
            self.check_goals()
            self.print_status()

            time.sleep(SIM_DT)

        elapsed = time.time() - t_start

        self.print_final_results(elapsed)
        p.disconnect()

    def print_final_results(self, elapsed_time):
        """Print simulation results."""
        print(f"\n{'='*70}")
        print(f"SIMULATION COMPLETE")
        print(f"{'='*70}")
        print(f"Simulated time: {self.sim_time:.2f}s")
        print(f"Real time: {elapsed_time:.2f}s")
        print(f"Speedup: {self.sim_time / elapsed_time:.2f}x")
        print(f"\nRovers Reached Goal: {self.stats['goals_reached']}/{len(self.rovers)}")
        for rover_id, completion_time in self.stats["completion_times"].items():
            print(f"  {rover_id}: {completion_time:.2f}s")
        print(f"{'='*70}\n")


# ============================================================================
#  ENTRY POINT
# ============================================================================

def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Hybrid Collision Avoidance Simulation"
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default=SCENARIO_NAME,
        help=f"Scenario name (default: {SCENARIO_NAME})"
    )
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="List available scenarios"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI"
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=MAX_SIM_TIME,
        help=f"Maximum simulation time in seconds (default: {MAX_SIM_TIME})"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress verbose output"
    )

    args = parser.parse_args()

    if args.list_scenarios:
        list_scenarios()
        return

    sim = HybridSimulation(
        scenario_name=args.scenario,
        gui=not args.headless,
        verbose=not args.quiet,
    )
    sim.run(max_time=args.max_time)


if __name__ == "__main__":
    main()
