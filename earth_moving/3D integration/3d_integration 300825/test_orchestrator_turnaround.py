import math
import time
import pybullet as p
from pybullet_integration import PyBulletIntegration
from main import run_2d_env

def test_orchestrator_turnaround():
    """
    Test the complete orchestrator turn-around sequence:
    1. Execute a trajectory to the target zone
    2. Verify target zone delivery detection  
    3. Confirm turn-around execution
    4. Check for no spurious movement
    """
    print("🎯 Testing Orchestrator Turn-Around Integration")
    print("=" * 60)
    
    # Initialize same as orchestrator
    env_radius = 1.0
    target_zone_radius = 0.3
    num_pebbles = 20
    random_seed = 10
    initial_robot_pose = (0.0, -1.5, math.pi / 2)  # Start closer to objects
    
    integration = PyBulletIntegration(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        num_pebbles=num_pebbles,
        random_seed=random_seed,
        gui=True,
        initial_robot_pose=initial_robot_pose
    )
    
    integration.set_robot_dim(L=0.2, R=0.07)
    
    print("Setting up 3D environment...")
    objects_3d = integration.run(auto_continue=True)
    shovel_width = integration.shovel_width
    
    # Create 2D environment
    print("Setting up 2D environment...")
    env, visualizer = run_2d_env(
        env_radius=env_radius,
        target_zone_radius=target_zone_radius,
        shovel_width=shovel_width,
        real_objects=objects_3d,
        manual_mode=False,
        use_spillage_model=True,
        visualize_potential=True
    )
    
    # Find a cell that should lead to target zone
    print("Finding suitable target trajectory...")
    target_trajectory = None
    clicked_cell = None
    
    # Look for cells with objects that can reach target
    for cell in env.cells_with_objects:
        if cell.num_objects > 0:
            # Test if this cell can generate a target path
            current_paths = env.get_path_for_preview(cell, "target")
            if current_paths and len(current_paths) > 0:
                trajectory = current_paths[0]['path']
                path_info = current_paths[0]
                
                # Check if path ends near target zone (distance to center)
                final_cell = trajectory[-1] 
                final_world = integration.coord_converter.convert_2d_to_3d(final_cell.x, final_cell.y)
                distance_to_center = math.hypot(final_world[0], final_world[1])
                
                print(f"  Cell({cell.x}, {cell.y}): {cell.num_objects} objects")
                print(f"    Path length: {len(trajectory)} cells")
                print(f"    Final cell: ({final_cell.x}, {final_cell.y})")
                print(f"    Distance to center: {distance_to_center:.3f}m (target radius: {target_zone_radius:.3f}m)")
                
                if distance_to_center <= (target_zone_radius + 0.1):
                    print(f"    ✅ This should be a TARGET ZONE delivery!")
                    target_trajectory = trajectory
                    clicked_cell = cell
                    break
                else:
                    print(f"    ❌ This would NOT be target zone delivery")
    
    if not target_trajectory:
        print("❌ No suitable target trajectory found! Test cannot proceed.")
        integration.close_environment()
        return
        
    # Execute the trajectory (same as orchestrator)
    print(f"\n🚗 Executing target trajectory from Cell({clicked_cell.x}, {clicked_cell.y})...")
    
    # Set up environment state (same as orchestrator)
    trajectory_coords = [(cell.x, cell.y) for cell in target_trajectory]
    env.current_trajectory = trajectory_coords
    env.current_cell = clicked_cell
    env.current_path_type = "target"
    env.current_use_spillage = True
    
    # Get robot position before trajectory
    (x_start, y_start), theta_start = integration.pose()
    print(f"  📍 Starting position: ({x_start:.3f}, {y_start:.3f})")
    print(f"  🧭 Starting heading: {math.degrees(theta_start):.1f}°")
    
    # Build and execute path (same as orchestrator)
    approach, task_world, path_world = integration.build_world_path(
        env, 
        approach_duration=1.5,
        approach_scale=0.25,
        ds=0.03
    )
    print(f"  📊 Path: {len(approach)} approach, {len(task_world)} task, {len(path_world)} final points")
    
    # Execute Pure Pursuit
    print("  🚗 Starting Pure Pursuit execution...")
    integration.follow_trajectory_pure_pursuit(
        path_world,
        dt=1/240,
        lookahead=0.08,
        v_nom=0.25,
        a_lat_max=0.8,
        yaw_slew=4.0,
        use_tcp_pose=True
    )
    
    # Brief settling (same as orchestrator)
    print("  🛑 Brief settling after trajectory completion...")
    for _ in range(30):
        integration.set_wheel_speeds_unitsafe(0.0, 0.0, 1/240)
    
    # Get position after trajectory but before turn-around
    (x_after_traj, y_after_traj), theta_after_traj = integration.pose()
    print(f"  📍 Position after trajectory: ({x_after_traj:.3f}, {y_after_traj:.3f})")
    print(f"  🧭 Heading after trajectory: {math.degrees(theta_after_traj):.1f}°")
    
    distance_to_center = math.hypot(x_after_traj, y_after_traj)
    print(f"  📏 Distance to target center: {distance_to_center:.3f}m")
    
    # Now test the turn-around detection and execution
    print(f"\n🔄 Testing post-delivery turn-around...")
    print(f"  🎯 Turn-around should be triggered for target zone delivery")
    
    # Execute post-trajectory processing with turn-around 
    transfer_result = integration._post_trajectory_update_with_turnaround(env)
    
    # Get final position after turn-around
    (x_final, y_final), theta_final = integration.pose()
    print(f"\n📊 FINAL RESULTS:")
    print(f"  📍 Final position: ({x_final:.3f}, {y_final:.3f})")  
    print(f"  🧭 Final heading: {math.degrees(theta_final):.1f}°")
    
    # Calculate movement during turn-around
    turn_distance = math.hypot(x_final - x_after_traj, y_final - y_after_traj)
    heading_change = abs(math.degrees(theta_final - theta_after_traj))
    
    print(f"  📏 Movement during turn-around: {turn_distance:.4f}m")
    print(f"  🔄 Heading change: {heading_change:.1f}°")
    
    # Expected: rover should face away from center (180° from inward direction)
    expected_outward_angle = math.atan2(-y_final, -x_final) + math.pi
    expected_outward_angle = math.atan2(math.sin(expected_outward_angle), math.cos(expected_outward_angle))  # Normalize
    heading_error = abs(math.degrees(theta_final - expected_outward_angle))
    if heading_error > 180:
        heading_error = 360 - heading_error
        
    print(f"  🎯 Expected outward heading: {math.degrees(expected_outward_angle):.1f}°")
    print(f"  📐 Heading accuracy: {heading_error:.1f}° error")
    
    # Check for spurious movement after turn
    print(f"\n🔍 Testing for spurious movement (monitoring 5 seconds)...")
    monitoring_positions = []
    
    for i in range(50):  # 5 seconds at 10Hz
        time.sleep(0.1)
        (x_monitor, y_monitor), theta_monitor = integration.pose()
        monitoring_positions.append((x_monitor, y_monitor))
        
        if i % 10 == 0:  # Log every second
            drift_from_final = math.hypot(x_monitor - x_final, y_monitor - y_final)
            print(f"  T+{i/10:.0f}s: pos=({x_monitor:.4f}, {y_monitor:.4f}), drift={drift_from_final:.4f}m")
    
    # Final drift analysis
    total_drift = math.hypot(monitoring_positions[-1][0] - x_final, monitoring_positions[-1][1] - y_final)
    
    print(f"\n🏁 TEST RESULTS:")
    print(f"  ✅ Trajectory executed: {'SUCCESS' if turn_distance < 0.2 else 'EXCESSIVE MOVEMENT'}")
    print(f"  ✅ Turn-around executed: {'SUCCESS' if 150 < heading_change < 210 else 'FAILED'}")  
    print(f"  ✅ Heading accuracy: {'SUCCESS' if heading_error < 15 else 'POOR'}")
    print(f"  ✅ No spurious drift: {'SUCCESS' if total_drift < 0.01 else 'DRIFT DETECTED'}")
    print(f"  📊 Total drift over 5s: {total_drift:.4f}m")
    
    if (turn_distance < 0.2 and 150 < heading_change < 210 and 
        heading_error < 15 and total_drift < 0.01):
        print(f"\n🎉 ORCHESTRATOR TURN-AROUND TEST: ✅ PASSED")
    else:
        print(f"\n❌ ORCHESTRATOR TURN-AROUND TEST: ❌ FAILED")
        
    input(f"\nPress ENTER to exit...")
    integration.close_environment()

if __name__ == "__main__":
    test_orchestrator_turnaround()