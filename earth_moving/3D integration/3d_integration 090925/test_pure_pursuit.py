#!/usr/bin/env python3
"""
Pure Pursuit Algorithm Testing Script with PyBullet Visualization

This script allows testing different Pure Pursuit configurations with custom waypoints
and spline generation (using the same method from spillage_model.py).

Features:
- Define custom waypoints for testing
- Generate smooth splines with curvature calculation
- Test different Pure Pursuit++ configurations
- PyBullet 3D visualization with rover model
- Real-time path following in 3D physics simulation
- Compare different tuning parameters

Usage:
python test_pure_pursuit.py
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import splprep, splev
import time
import sys
import os

# Add path to access the pybullet_integration module
sys.path.append(os.path.dirname(__file__))
from pybullet_integration import PyBulletIntegration

# Import spline generation from spillage_model if available
def smooth_path_with_spline(waypoints, smoothing_factor=0.5, num_points=1000):
    """
    Generate smooth spline path from waypoints (same as spillage_model.py)
    
    Args:
        waypoints: List of (x, y) waypoint tuples
        smoothing_factor: Spline smoothing parameter (0.0=interpolating, higher=smoother)
        num_points: Number of points in output spline
        
    Returns:
        tuple: (spline_points, curvature, success)
    """
    waypoints = np.array([(x + 0.5, y + 0.5) for x, y in waypoints])
    x, y = waypoints[:, 0], waypoints[:, 1]

    if len(x) < 2:
        return [], [], False

    # For exactly 2 waypoints, use linear interpolation
    if len(x) == 2:
        return linear_interpolation_with_curvature(x, y, num_points)

    try:
        # Adjust smoothing factor based on path length - shorter paths need less smoothing
        adaptive_smoothing = max(0.0, smoothing_factor * (len(x) - 2) / 3.0)
        k = min(3, len(x) - 1)
        
        tck, _ = splprep([x, y], s=adaptive_smoothing, k=k)
        t_fine = np.linspace(0, 1, num_points)
        smooth_x, smooth_y = splev(t_fine, tck)
        dx, dy = splev(t_fine, tck, der=1)
        d2x, d2y = splev(t_fine, tck, der=2)
        curvature = np.abs(dx * d2y - dy * d2x) / np.power(dx**2 + dy**2, 1.5)
        curvature[np.isnan(curvature)] = 0
        
        return list(zip(smooth_x, smooth_y)), curvature.tolist(), True
        
    except Exception as e:
        print(f"Spline fitting failed: {e}, falling back to linear interpolation")
        return linear_interpolation_with_curvature(x, y, num_points)

def linear_interpolation_with_curvature(x, y, num_points):
    """Fallback linear interpolation with artificial curvature"""
    t_waypoints = np.linspace(0, 1, len(x))
    t_fine = np.linspace(0, 1, num_points)
    
    smooth_x = np.interp(t_fine, t_waypoints, x)
    smooth_y = np.interp(t_fine, t_waypoints, y)
    
    # Calculate artificial curvature based on direction changes
    curvature = np.zeros(num_points)
    
    if len(x) >= 3:
        for i in range(1, len(x) - 1):
            v1 = np.array([x[i] - x[i-1], y[i] - y[i-1]])
            v2 = np.array([x[i+1] - x[i], y[i+1] - y[i]])
            
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0 and v2_norm > 0:
                v1_unit = v1 / v1_norm
                v2_unit = v2 / v2_norm
                
                cross_product = v1_unit[0] * v2_unit[1] - v1_unit[1] * v2_unit[0]
                angle_change = abs(cross_product)
                
                waypoint_t = i / (len(x) - 1)
                waypoint_idx = int(waypoint_t * (num_points - 1))
                
                spread_range = max(1, num_points // 20)
                for j in range(max(0, waypoint_idx - spread_range), 
                             min(num_points, waypoint_idx + spread_range + 1)):
                    distance_weight = max(0, 1 - abs(j - waypoint_idx) / spread_range)
                    curvature[j] = max(curvature[j], angle_change * distance_weight * 0.5)
    else:
        curvature.fill(0.01)
    
    return list(zip(smooth_x, smooth_y)), curvature.tolist(), True

class RealRoverPurePursuitTester:
    """Pure Pursuit algorithm testing using the real PyBulletIntegration rover"""
    
    def __init__(self, use_gui=True):
        """Initialize the tester with real rover integration"""
        # Default Pure Pursuit++ configuration (matching orchestrator.py)
        self.config = {
            'v_nom': 0.25,          # Nominal velocity (m/s) - from orchestrator
            'a_lat_max': 0.8,       # Maximum lateral acceleration (m/s²) - from orchestrator
            'yaw_slew_rate': 4.0,   # Maximum yaw rate change (rad/s²) - from orchestrator
            'lookahead_base': 0.08, # Base lookahead distance (m) - from orchestrator
            'lookahead_min': 0.05,  # Minimum lookahead distance (m)
            'lookahead_max': 0.20,  # Maximum lookahead distance (m)
            'dt': 1/240,            # Control timestep (s) - matching PyBullet
            'wheel_base': 0.20,     # Robot wheelbase (m)
            'tcp_fwd': 0.12,        # TCP forward offset (m) - from orchestrator
            'tcp_lat': 0.00,        # TCP lateral offset (m) - from orchestrator
            'stanley_k': 0.5,       # Stanley cross-track gain
            'feed_forward_k': 0.8,  # Feed-forward gain
            'resample_ds': 0.03,    # Path resampling distance - from orchestrator
        }
        
        # Initialize the real PyBullet integration
        print("Initializing real rover PyBullet integration...")
        self.integration = PyBulletIntegration(
            env_radius=2.0,  # Larger environment for test paths
            target_zone_radius=0.3,
            num_pebbles=10,  # Fewer pebbles for cleaner testing
            random_seed=42,
            gui=use_gui,
            initial_robot_pose=(0.0, -1.5, math.pi/2)  # Start at bottom
        )
        
        # Set rover dimensions matching orchestrator
        self.integration.set_robot_dim(L=0.2, R=0.07)
        
        # Initialize environment (load rover and world)
        print("Loading rover and environment...")
        objects_3d = self.integration.run(auto_continue=True)
        print(f"Environment initialized with {len(objects_3d)} objects")
        
        # Robot state tracking
        self.reset_robot_state()
        
        # Path visualization
        self.path_visualization = []
        
    def reset_robot_state(self):
        """Reset robot to initial state"""
        # Get current rover pose from PyBullet
        base_pos, base_orn = self.integration.get_robot_position()
        self.x = base_pos[0]
        self.y = base_pos[1]  
        # Get orientation from quaternion
        import pybullet as p
        euler = p.getEulerFromQuaternion(base_orn)
        self.theta = euler[2]  # Z rotation (yaw)
        self.v = 0.0      # Robot velocity
        self.omega = 0.0  # Robot angular velocity
        
        # Control state
        self.last_yaw_cmd = 0.0
        self.s_prev = 0.0
        self.Ld_prev = self.config['lookahead_base']
        self.ey_prev = 0.0
        self.phi_prev = 0.0
        
        # History for plotting
        self.history = {
            'x': [], 'y': [], 'theta': [], 'v': [], 'omega': [],
            'lookahead': [], 'cross_track': [], 'curvature': [], 'target_idx': []
        }
        
    def get_current_robot_pose(self):
        """Get current robot pose from the real rover"""
        base_pos, base_orn = self.integration.get_robot_position()
        import pybullet as p
        euler = p.getEulerFromQuaternion(base_orn)
        theta = euler[2]  # Z rotation (yaw)
        return base_pos[0], base_pos[1], theta
            
    def visualize_path_in_pybullet(self, path_points, color=[1, 0, 0]):
        """Visualize path as line segments using the real PyBullet integration"""
        # Use the integration's trajectory visualization
        self.integration.visualize_trajectory(path_points)
        
    def clear_path_visualization(self):
        """Clear all path visualization lines"""
        self.integration.clear_trajectory()
        
    def visualize_robot_info(self, target_idx, lookahead_dist, cross_track_error, path_points):
        """Show robot information as debug text using real PyBullet"""
        if target_idx < len(path_points):
            # Show target point using PyBullet debug lines
            target_pos = [path_points[target_idx][0], path_points[target_idx][1], 0.1]
            
            # Add debug text above rover
            info_text = f"Speed: {self.v:.2f} m/s\\nLookahead: {lookahead_dist:.3f} m\\nCross-track: {cross_track_error:.3f} m"
            import pybullet as p
            p.addUserDebugText(
                info_text,
                [self.x + 0.3, self.y, 0.2],
                textColorRGB=[0, 0, 1],
                textSize=1.0,
                lifeTime=0.1,
                physicsClientId=self.integration.physics_client
            )
            
            # Show target point as line
            p.addUserDebugLine(
                target_pos,
                [target_pos[0], target_pos[1], target_pos[2] + 0.1],
                lineColorRGB=[1, 0, 1],
                lineWidth=5.0,
                lifeTime=0.1,
                physicsClientId=self.integration.physics_client
            )
            
    def close_pybullet(self):
        """Clean up PyBullet simulation"""
        if hasattr(self, 'integration') and self.integration:
            self.integration.close_environment()
    
    def simulate_robot_dynamics(self, v_cmd, omega_cmd, dt):
        """Use the real rover's wheel control system"""
        # Convert v_cmd, omega_cmd to left/right wheel velocities
        # Differential drive kinematics: v = (v_left + v_right) / 2, omega = (v_right - v_left) / wheelbase
        wheel_base = self.config['wheel_base']
        v_left = v_cmd - omega_cmd * wheel_base / 2
        v_right = v_cmd + omega_cmd * wheel_base / 2
        
        # Apply wheel speeds to the real rover
        self.integration.set_wheel_speeds_unitsafe(v_left, v_right, dt)
        
        # Update our tracked state from the actual rover pose
        self.x, self.y, self.theta = self.get_current_robot_pose()
        self.v = v_cmd  # Store commanded velocity
        self.omega = omega_cmd  # Store commanded angular velocity
        
        # Store history
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)
        self.history['v'].append(self.v)
        self.history['omega'].append(self.omega)
    
    def find_closest_point_on_path(self, path_points):
        """Find closest point on path to current robot position"""
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path_points):
            dist = math.hypot(px - self.x, py - self.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx, min_dist
    
    def calculate_curvature_aware_lookahead(self, path_points, curvature, current_idx):
        """Calculate lookahead distance based on path curvature"""
        cfg = self.config
        
        # Base lookahead
        Ld = cfg['lookahead_base']
        
        # Look ahead in path to estimate upcoming curvature
        lookahead_samples = min(50, len(path_points) - current_idx - 1)
        if lookahead_samples > 0:
            upcoming_curvature = max(curvature[current_idx:current_idx + lookahead_samples])
            
            # Reduce lookahead for high curvature sections
            if upcoming_curvature > 0.1:  # High curvature threshold
                curvature_factor = 1.0 / (1.0 + 5.0 * upcoming_curvature)
                Ld *= curvature_factor
        
        # Clamp lookahead distance
        Ld = max(cfg['lookahead_min'], min(cfg['lookahead_max'], Ld))
        
        return Ld
    
    def pure_pursuit_control(self, path_points, curvature):
        """
        Pure Pursuit++ control algorithm
        
        Args:
            path_points: List of (x, y) path points
            curvature: List of curvature values at each point
            
        Returns:
            tuple: (v_cmd, omega_cmd, target_idx, cross_track_error, lookahead_dist)
        """
        cfg = self.config
        dt = cfg['dt']
        
        if len(path_points) < 2:
            return 0.0, 0.0, 0, 0.0, 0.0
        
        # Find closest point on path
        closest_idx, cross_track_error = self.find_closest_point_on_path(path_points)
        
        # Calculate curvature-aware lookahead distance
        lookahead_dist = self.calculate_curvature_aware_lookahead(path_points, curvature, closest_idx)
        
        # Find target point at lookahead distance
        target_idx = closest_idx
        cumulative_dist = 0.0
        
        for i in range(closest_idx, len(path_points) - 1):
            dx = path_points[i+1][0] - path_points[i][0]
            dy = path_points[i+1][1] - path_points[i][1]
            segment_dist = math.hypot(dx, dy)
            
            if cumulative_dist + segment_dist >= lookahead_dist:
                target_idx = i + 1
                break
            cumulative_dist += segment_dist
        
        # Clamp target index
        target_idx = min(target_idx, len(path_points) - 1)
        
        # Calculate target point in robot frame
        target_x, target_y = path_points[target_idx]
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Transform to robot frame
        target_x_robot = dx * math.cos(-self.theta) - dy * math.sin(-self.theta)
        target_y_robot = dx * math.sin(-self.theta) + dy * math.cos(-self.theta)
        
        # Pure pursuit calculation
        L = math.hypot(target_x_robot, target_y_robot)
        if L < 0.001:  # Avoid division by zero
            return 0.0, 0.0, target_idx, cross_track_error, lookahead_dist
            
        # Calculate curvature and angular velocity
        path_curvature = 2.0 * target_y_robot / (L * L)
        
        # Velocity calculation with lateral-g limiting
        local_curvature = curvature[min(target_idx, len(curvature) - 1)]
        if abs(local_curvature) > 0.001:
            v_curvature_limit = math.sqrt(cfg['a_lat_max'] / abs(local_curvature))
            v_cmd = min(cfg['v_nom'], v_curvature_limit)
        else:
            v_cmd = cfg['v_nom']
        
        # Angular velocity with feed-forward and Stanley correction
        omega_pure_pursuit = v_cmd * path_curvature
        
        # Feed-forward term
        omega_feedforward = cfg['feed_forward_k'] * v_cmd * local_curvature
        
        # Stanley cross-track correction
        omega_stanley = cfg['stanley_k'] * cross_track_error / (v_cmd + 0.1)
        
        # Combine control terms
        omega_cmd = omega_pure_pursuit + omega_feedforward + omega_stanley
        
        # Apply yaw slew rate limiting
        max_omega_change = cfg['yaw_slew_rate'] * dt
        omega_change = omega_cmd - self.last_yaw_cmd
        if abs(omega_change) > max_omega_change:
            omega_cmd = self.last_yaw_cmd + math.copysign(max_omega_change, omega_change)
        
        self.last_yaw_cmd = omega_cmd
        
        # Store control history
        self.history['lookahead'].append(lookahead_dist)
        self.history['cross_track'].append(cross_track_error)
        self.history['curvature'].append(local_curvature)
        self.history['target_idx'].append(target_idx)
        
        return v_cmd, omega_cmd, target_idx, cross_track_error, lookahead_dist
    
    def run_simulation(self, path_points, curvature, max_time=30.0, pybullet_visualization=True):
        """
        Run Pure Pursuit simulation on given path with PyBullet visualization
        
        Args:
            path_points: List of (x, y) waypoints
            curvature: List of curvature values
            max_time: Maximum simulation time (s)
            pybullet_visualization: Use PyBullet 3D visualization
            
        Returns:
            dict: Simulation results and metrics
        """
        print(f"Running Pure Pursuit simulation with PyBullet...")
        print(f"Path: {len(path_points)} points")
        print(f"Config: v_nom={self.config['v_nom']}, lookahead={self.config['lookahead_base']}")
        
        self.reset_robot_state()
        
        # Visualize path in PyBullet
        if pybullet_visualization:
            self.visualize_path_in_pybullet(path_points, color=[1, 0, 0])  # Red path
        
        dt = self.config['dt']
        t = 0.0
        step = 0
        
        # Place robot at start of path (for now, use current position)
        # In a real test, you might want to move the rover to the path start
        # For now, we'll start from the current rover position
        self.reset_robot_state()
        print(f"Starting from rover position: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f}°)")
        
        # Optionally, could move rover to path start (uncomment if desired):
        # if path_points:
        #     target_x, target_y = path_points[0]
        #     print(f"Moving rover to path start: ({target_x:.2f}, {target_y:.2f})")
        #     # Use a simple goto to move to start position
        #     # This would require implementing a goto function
        
        print("Starting simulation... Press 'q' in PyBullet window to quit early")
        
        # Simulation loop
        while t < max_time:
            # Pure pursuit control
            v_cmd, omega_cmd, target_idx, cross_track, lookahead = self.pure_pursuit_control(path_points, curvature)
            
            # Check if reached end of path
            end_dist = math.hypot(path_points[-1][0] - self.x, path_points[-1][1] - self.y)
            if end_dist < 0.05:  # 5cm tolerance
                print(f"Reached path end at t={t:.2f}s")
                break
            
            # Simulate robot dynamics
            self.simulate_robot_dynamics(v_cmd, omega_cmd, dt)
            
            # PyBullet visualization updates
            if pybullet_visualization and step % 5 == 0:  # Update every 5 steps for performance
                self.visualize_robot_info(target_idx, lookahead, cross_track, path_points)
                
            # Step PyBullet simulation (for visual updates) - already done by set_wheel_speeds_unitsafe
            # The integration.set_wheel_speeds_unitsafe already steps the simulation
                
            # Check for early termination (keyboard interrupt)
            import pybullet as p
            keys = p.getKeyboardEvents(physicsClientId=self.integration.physics_client)
            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("Early termination requested")
                break
                
            # Small delay for real-time visualization
            # No additional sleep needed - the rover physics runs at real time
            
            t += dt
            step += 1
        
        # Calculate performance metrics
        results = self.calculate_metrics(path_points)
        results['simulation_time'] = t
        results['steps'] = step
        
        print(f"Simulation complete: {step} steps, {t:.2f}s")
        print(f"Final position error: {results['final_position_error']:.3f}m")
        print(f"Average cross-track error: {results['avg_cross_track_error']:.3f}m")
        print(f"Max cross-track error: {results['max_cross_track_error']:.3f}m")
        
        return results
    
    def calculate_metrics(self, path_points):
        """Calculate performance metrics from simulation history"""
        if not self.history['x']:
            return {}
        
        # Final position error
        final_pos_error = math.hypot(
            path_points[-1][0] - self.history['x'][-1],
            path_points[-1][1] - self.history['y'][-1]
        )
        
        # Cross-track error statistics
        cross_track_errors = [abs(e) for e in self.history['cross_track']]
        avg_cross_track = np.mean(cross_track_errors) if cross_track_errors else 0
        max_cross_track = max(cross_track_errors) if cross_track_errors else 0
        
        # Velocity and smoothness
        avg_velocity = np.mean(self.history['v']) if self.history['v'] else 0
        velocity_std = np.std(self.history['v']) if len(self.history['v']) > 1 else 0
        
        omega_changes = [abs(self.history['omega'][i] - self.history['omega'][i-1]) 
                        for i in range(1, len(self.history['omega']))]
        avg_omega_change = np.mean(omega_changes) if omega_changes else 0
        
        return {
            'final_position_error': final_pos_error,
            'avg_cross_track_error': avg_cross_track,
            'max_cross_track_error': max_cross_track,
            'avg_velocity': avg_velocity,
            'velocity_smoothness': velocity_std,
            'avg_omega_change': avg_omega_change,
        }
    
    def __del__(self):
        """Destructor to clean up PyBullet"""
        self.close_pybullet()
    
    def plot_results(self, path_points, curvature, results):
        """Plot comprehensive simulation results"""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle(f'Pure Pursuit++ Simulation Results', fontsize=16)
        
        # Path following plot
        ax = axes[0, 0]
        path_x, path_y = zip(*path_points)
        ax.plot(path_x, path_y, 'b-', linewidth=3, label='Reference Path', alpha=0.7)
        ax.plot(self.history['x'], self.history['y'], 'g-', linewidth=2, label='Robot Trajectory')
        ax.plot(self.history['x'][0], self.history['y'][0], 'go', markersize=8, label='Start')
        ax.plot(self.history['x'][-1], self.history['y'][-1], 'ro', markersize=8, label='End')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Path Following')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        
        # Velocity profile
        ax = axes[0, 1]
        time = np.arange(len(self.history['v'])) * self.config['dt']
        ax.plot(time, self.history['v'], 'b-', linewidth=2)
        ax.axhline(self.config['v_nom'], color='r', linestyle='--', label=f'v_nom={self.config["v_nom"]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.grid(True)
        ax.legend()
        
        # Cross-track error
        ax = axes[0, 2]
        ax.plot(time, self.history['cross_track'], 'r-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Cross-track Error (m)')
        ax.set_title('Cross-track Error')
        ax.grid(True)
        
        # Lookahead distance
        ax = axes[1, 0]
        ax.plot(time, self.history['lookahead'], 'g-', linewidth=2)
        ax.axhline(self.config['lookahead_base'], color='b', linestyle='--', label=f'Base={self.config["lookahead_base"]}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Lookahead (m)')
        ax.set_title('Adaptive Lookahead')
        ax.grid(True)
        ax.legend()
        
        # Angular velocity
        ax = axes[1, 1]
        ax.plot(time, self.history['omega'], 'purple', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title('Angular Velocity')
        ax.grid(True)
        
        # Path curvature
        ax = axes[1, 2]
        path_s = np.linspace(0, 1, len(curvature))
        ax.plot(path_s, curvature, 'orange', linewidth=2)
        ax.set_xlabel('Path Progress')
        ax.set_ylabel('Curvature (1/m)')
        ax.set_title('Path Curvature')
        ax.grid(True)
        
        plt.tight_layout()
        return fig

def create_test_scenarios():
    """Create different test scenarios for Pure Pursuit tuning"""
    scenarios = {}
    
    # Scenario 1: Simple straight line
    scenarios['straight_line'] = {
        'waypoints': [(0, 0), (5, 0)],
        'description': 'Simple straight line path'
    }
    
    # Scenario 2: Simple curve
    scenarios['simple_curve'] = {
        'waypoints': [(0, 0), (2, 0), (4, 2), (4, 4)],
        'description': 'Simple curved path'
    }
    
    # Scenario 3: S-curve
    scenarios['s_curve'] = {
        'waypoints': [(0, 0), (1, 0), (2, 1), (3, 1), (4, 0), (5, 0)],
        'description': 'S-shaped curve'
    }
    
    # Scenario 4: Tight turn
    scenarios['tight_turn'] = {
        'waypoints': [(0, 0), (2, 0), (2, 2), (0, 2)],
        'description': 'Sharp 90-degree turns'
    }
    
    # Scenario 5: Complex path
    scenarios['complex_path'] = {
        'waypoints': [(0, 0), (1, 1), (3, 1), (4, 3), (2, 4), (1, 2), (0, 3)],
        'description': 'Complex path with multiple turns'
    }
    
    return scenarios

def create_config_variations():
    """Create different Pure Pursuit configuration variations for testing"""
    base_config = {
        'v_nom': 0.30,
        'a_lat_max': 1.0,
        'yaw_slew_rate': 7.0,
        'lookahead_base': 0.10,
        'lookahead_min': 0.05,
        'lookahead_max': 0.20,
        'dt': 0.02,
        'wheel_base': 0.20,
        'tcp_fwd': 0.12,
        'tcp_lat': 0.00,
        'stanley_k': 0.5,
        'feed_forward_k': 0.8,
    }
    
    variations = {}
    
    # Conservative tuning
    variations['conservative'] = base_config.copy()
    variations['conservative'].update({
        'v_nom': 0.20,
        'lookahead_base': 0.15,
        'yaw_slew_rate': 5.0,
        'stanley_k': 0.3
    })
    
    # Aggressive tuning
    variations['aggressive'] = base_config.copy()
    variations['aggressive'].update({
        'v_nom': 0.40,
        'lookahead_base': 0.08,
        'yaw_slew_rate': 10.0,
        'stanley_k': 0.8
    })
    
    # High precision tuning
    variations['precision'] = base_config.copy()
    variations['precision'].update({
        'lookahead_base': 0.06,
        'lookahead_min': 0.03,
        'stanley_k': 1.0,
        'feed_forward_k': 1.0
    })
    
    # Smooth tuning
    variations['smooth'] = base_config.copy()
    variations['smooth'].update({
        'lookahead_base': 0.12,
        'yaw_slew_rate': 4.0,
        'stanley_k': 0.2,
        'feed_forward_k': 0.5
    })
    
    return variations

def main():
    """Main testing function with PyBullet visualization"""
    print("=== Pure Pursuit++ Testing Script with PyBullet ===")
    
    # PyBullet GUI option
    use_gui = input("Use PyBullet GUI visualization? (y/n, default=y): ").strip().lower()
    use_gui = use_gui != 'n'  # Default to GUI unless explicitly no
    
    # Create test scenarios and configurations
    scenarios = create_test_scenarios()
    configs = create_config_variations()
    
    # Select scenario
    print("\\nAvailable test scenarios:")
    for name, info in scenarios.items():
        print(f"  {name}: {info['description']}")
    
    scenario_name = input("\\nSelect scenario (or press Enter for 's_curve'): ").strip()
    if not scenario_name or scenario_name not in scenarios:
        scenario_name = 's_curve'
    
    scenario = scenarios[scenario_name]
    print(f"Selected: {scenario['description']}")
    
    # Generate spline path
    print("\\nGenerating spline path...")
    waypoints = scenario['waypoints']
    spline_points, curvature, success = smooth_path_with_spline(waypoints, smoothing_factor=0.3, num_points=500)
    
    if not success:
        print("ERROR: Failed to generate spline path")
        return
    
    print(f"Generated spline with {len(spline_points)} points")
    print(f"Max curvature: {max(curvature):.4f}")
    
    # Select configuration
    print("\\nAvailable configurations:")
    for name in configs.keys():
        print(f"  {name}")
    
    config_name = input("\\nSelect configuration (or press Enter for 'conservative'): ").strip()
    if not config_name or config_name not in configs:
        config_name = 'conservative'
    
    print(f"Selected: {config_name} configuration")
    
    # Initialize Real Rover tester
    print(f"\\nInitializing Real Rover PyBullet environment (GUI: {use_gui})...")
    tester = RealRoverPurePursuitTester(use_gui=use_gui)
    tester.config.update(configs[config_name])
    
    try:
        # Run simulation with PyBullet visualization
        results = tester.run_simulation(spline_points, curvature, max_time=20.0, pybullet_visualization=True)
        
        # Generate analysis plots
        generate_plots = input("\\nGenerate analysis plots? (y/n): ").strip().lower()
        if generate_plots == 'y':
            print("\\nGenerating plots...")
            fig = tester.plot_results(spline_points, curvature, results)
            plt.show()
        
        # Option to test multiple configurations
        test_all = input("\\nTest all configurations in PyBullet? (y/n): ").strip().lower()
        if test_all == 'y':
            print("\\nTesting all configurations...")
            comparison_results = {}
            
            for test_config_name, config in configs.items():
                if test_config_name == config_name:
                    # Use existing results
                    comparison_results[test_config_name] = results
                    continue
                    
                print(f"\\nTesting {test_config_name}...")
                tester.config.update(config)
                test_results = tester.run_simulation(spline_points, curvature, max_time=20.0, pybullet_visualization=True)
                comparison_results[test_config_name] = test_results
                
                # Brief pause between tests
                if use_gui:
                    input("Press Enter to continue to next configuration...")
            
            # Print comparison
            print("\\n=== CONFIGURATION COMPARISON ===")
            print(f"{'Config':<12} {'Pos Error':<10} {'Avg CrossT':<11} {'Max CrossT':<11} {'Avg Vel':<8} {'Time':<6}")
            print("-" * 70)
            for name, res in comparison_results.items():
                print(f"{name:<12} {res['final_position_error']:<10.3f} {res['avg_cross_track_error']:<11.3f} "
                      f"{res['max_cross_track_error']:<11.3f} {res['avg_velocity']:<8.2f} {res['simulation_time']:<6.1f}")
        
    except KeyboardInterrupt:
        print("\\nSimulation interrupted by user")
    except Exception as e:
        print(f"\\nError during simulation: {e}")
    finally:
        # Clean up PyBullet
        print("\\nCleaning up...")
        tester.close_pybullet()
    
    print("\\nTesting complete!")

if __name__ == "__main__":
    main()