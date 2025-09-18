import math
import time
import pybullet as p
from pybullet_integration import PyBulletIntegration

def debug_turn_drift():
    """
    Comprehensive debug test to identify why rover drifts forward after turn-in-place.
    Tests motor states, physics properties, and control modes.
    """
    print("🔍 DEBUG: Turn-in-Place Drift Analysis")
    print("=" * 60)
    
    # Initialize PyBullet integration
    integration = PyBulletIntegration(
        env_radius=1.0,
        target_zone_radius=0.3,
        num_pebbles=5,  # Minimal pebbles for cleaner testing
        random_seed=42,
        gui=True,
        initial_robot_pose=(0.0, 0.0, 0.0)  # Start facing East (0°)
    )
    
    integration.set_robot_dim(L=0.2, R=0.07)
    
    print("Setting up 3D environment...")
    objects_3d = integration.run(auto_continue=True)
    
    robot_id = integration.object_ids[1]
    left_joint = integration.get_joint_index_by_name("base_to_lwheel")
    right_joint = integration.get_joint_index_by_name("base_to_rwheel")
    
    def get_detailed_state(label=""):
        """Get comprehensive rover state for analysis"""
        (x, y), theta = integration.pose()
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        base_speed = math.hypot(lin_vel[0], lin_vel[1])
        
        left_state = p.getJointState(robot_id, left_joint)
        right_state = p.getJointState(robot_id, right_joint)
        
        left_info = p.getJointInfo(robot_id, left_joint)
        right_info = p.getJointInfo(robot_id, right_joint)
        
        print(f"\n📊 {label} ROVER STATE:")
        print(f"  📍 Position: ({x:.4f}, {y:.4f})")
        print(f"  🧭 Heading: {math.degrees(theta):.2f}°")
        print(f"  🏃 Base velocity: {base_speed:.6f} m/s")
        print(f"  🔄 Angular velocity: {abs(ang_vel[2]):.6f} rad/s")
        print(f"  🔧 Left wheel: pos={left_state[0]:.4f}, vel={left_state[1]:.6f} rad/s")
        print(f"  🔧 Right wheel: pos={right_state[0]:.4f}, vel={right_state[1]:.6f} rad/s")
        
        # Check joint dynamics
        left_dynamics = p.getDynamicsInfo(robot_id, left_joint)
        right_dynamics = p.getDynamicsInfo(robot_id, right_joint)
        
        print(f"  ⚙️  Left wheel dynamics: friction={left_dynamics[1]:.2f}, damping={left_dynamics[6]:.4f}")
        print(f"  ⚙️  Right wheel dynamics: friction={right_dynamics[1]:.2f}, damping={right_dynamics[6]:.4f}")
        
        return (x, y), theta, base_speed
    
    # Test 1: Baseline state
    initial_pos, initial_theta, initial_speed = get_detailed_state("INITIAL")
    
    print(f"\n🎯 TEST: 90° turn-in-place")
    target_angle = initial_theta + math.radians(90)
    
    # Execute turn
    print(f"Executing turn from {math.degrees(initial_theta):.1f}° to {math.degrees(target_angle):.1f}°...")
    start_time = time.time()
    
    try:
        integration._turn_in_place(target_angle, max_rate=2.0, tol=0.05)
        end_time = time.time()
        
        # Immediate post-turn state
        post_turn_pos, post_turn_theta, post_turn_speed = get_detailed_state("IMMEDIATE POST-TURN")
        
        print(f"\n⏱️  Turn completed in {end_time - start_time:.2f}s")
        
        # Wait and monitor for drift
        print(f"\n🔍 MONITORING FOR DRIFT (10 seconds)...")
        
        drift_positions = []
        drift_times = []
        
        for i in range(100):  # Monitor for 10 seconds (100 * 0.1s)
            time.sleep(0.1)
            
            current_pos, current_theta, current_speed = integration.pose()[0], integration.pose()[1], 0
            lin_vel, ang_vel = p.getBaseVelocity(robot_id)
            current_speed = math.hypot(lin_vel[0], lin_vel[1])
            
            drift_positions.append(current_pos)
            drift_times.append(i * 0.1)
            
            # Log every 2 seconds
            if i % 20 == 0:
                distance_drifted = math.hypot(current_pos[0] - post_turn_pos[0][0], 
                                            current_pos[1] - post_turn_pos[0][1])
                print(f"  T+{i*0.1:.1f}s: pos=({current_pos[0]:.4f}, {current_pos[1]:.4f}), "
                      f"drift={distance_drifted:.4f}m, speed={current_speed:.6f} m/s")
                
                # Check joint states
                left_vel = p.getJointState(robot_id, left_joint)[1]
                right_vel = p.getJointState(robot_id, right_joint)[1]
                print(f"         wheel_vels: L={left_vel:.6f}, R={right_vel:.6f} rad/s")
        
        # Final analysis
        final_pos, final_theta, final_speed = get_detailed_state("FINAL (after 10s)")
        
        total_drift = math.hypot(final_pos[0] - post_turn_pos[0][0], 
                               final_pos[1] - post_turn_pos[0][1])
        
        print(f"\n📋 DRIFT ANALYSIS RESULTS:")
        print(f"  📏 Total drift distance: {total_drift:.4f}m")
        print(f"  🏃 Final speed: {final_speed:.6f} m/s")
        
        if total_drift > 0.01:  # 1cm threshold
            print(f"  ❌ SIGNIFICANT DRIFT DETECTED!")
            
            # Analyze drift pattern
            drift_distances = [math.hypot(pos[0] - post_turn_pos[0][0], pos[1] - post_turn_pos[0][1]) 
                             for pos in drift_positions]
            
            max_drift = max(drift_distances)
            avg_drift_rate = total_drift / 10.0  # m/s
            
            print(f"  📊 Max drift: {max_drift:.4f}m")
            print(f"  📊 Avg drift rate: {avg_drift_rate:.6f} m/s")
            
            # Check if drift is accelerating or constant
            early_drift = drift_distances[25] - drift_distances[0]  # First 2.5s
            late_drift = drift_distances[-1] - drift_distances[75]   # Last 2.5s
            
            if late_drift > early_drift * 1.5:
                print(f"  🔺 ACCELERATING DRIFT (physics instability)")
            elif abs(late_drift - early_drift) < 0.001:
                print(f"  ➡️  CONSTANT DRIFT (residual velocity)")
            else:
                print(f"  🔻 DECELERATING DRIFT (momentum decay)")
        else:
            print(f"  ✅ DRIFT WITHIN ACCEPTABLE LIMITS")
            
    except Exception as e:
        print(f"   ❌ Error during turn: {e}")
        import traceback
        traceback.print_exc()
    
    print(f"\n🔧 MOTOR STATE DIAGNOSIS:")
    
    # Check current motor control modes
    for joint_idx, joint_name in [(left_joint, "LEFT"), (right_joint, "RIGHT")]:
        joint_info = p.getJointInfo(robot_id, joint_idx)
        joint_state = p.getJointState(robot_id, joint_idx)
        
        print(f"  {joint_name} WHEEL:")
        print(f"    Joint type: {joint_info[2]}")
        print(f"    Position: {joint_state[0]:.4f} rad")
        print(f"    Velocity: {joint_state[1]:.6f} rad/s") 
        print(f"    Applied force: {joint_state[3]:.2f} N⋅m")
    
    print(f"\n🧪 ATTEMPTING EMERGENCY STOP PROCEDURES:")
    
    # Try different stopping methods
    methods = [
        ("VELOCITY_CONTROL zeros", lambda: [
            p.setJointMotorControl2(robot_id, left_joint, p.VELOCITY_CONTROL, targetVelocity=0.0, force=50000),
            p.setJointMotorControl2(robot_id, right_joint, p.VELOCITY_CONTROL, targetVelocity=0.0, force=50000)
        ]),
        ("POSITION_CONTROL lock", lambda: [
            p.setJointMotorControl2(robot_id, left_joint, p.POSITION_CONTROL, 
                                  targetPosition=p.getJointState(robot_id, left_joint)[0], force=100000),
            p.setJointMotorControl2(robot_id, right_joint, p.POSITION_CONTROL,
                                  targetPosition=p.getJointState(robot_id, right_joint)[0], force=100000)
        ]),
        ("TORQUE_CONTROL disable", lambda: [
            p.setJointMotorControl2(robot_id, left_joint, p.TORQUE_CONTROL, force=0),
            p.setJointMotorControl2(robot_id, right_joint, p.TORQUE_CONTROL, force=0)
        ])
    ]
    
    for method_name, method_func in methods:
        print(f"\n  Testing {method_name}...")
        
        pre_pos, _, _ = integration.pose()[0], integration.pose()[1], 0
        
        # Apply method
        method_func()
        
        # Let it settle for 2 seconds
        for _ in range(480):  # 2 seconds at 240Hz
            p.stepSimulation()
            time.sleep(1/240)
        
        post_pos, _, _ = integration.pose()[0], integration.pose()[1], 0
        method_drift = math.hypot(post_pos[0] - pre_pos[0], post_pos[1] - pre_pos[1])
        
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        final_speed = math.hypot(lin_vel[0], lin_vel[1])
        
        print(f"    Drift during method: {method_drift:.6f}m")
        print(f"    Final speed: {final_speed:.6f} m/s")
        
        if method_drift < 0.001 and final_speed < 0.001:
            print(f"    ✅ {method_name} EFFECTIVE!")
        else:
            print(f"    ❌ {method_name} insufficient")
    
    input(f"\nPress ENTER to exit debug session...")
    integration.close_environment()

if __name__ == "__main__":
    debug_turn_drift()