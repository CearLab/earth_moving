import math
import time
import pybullet as p
from pybullet_integration import PyBulletIntegration

def test_turn_in_place_functionality():
    """
    Test the _turn_in_place function with various scenarios:
    1. Small turn (< 35 degrees) - should not trigger turn-in-place
    2. Medium turn (45 degrees) - should trigger turn-in-place  
    3. Large turn (90 degrees) - should trigger turn-in-place
    4. Full reverse (180 degrees) - should trigger turn-in-place
    """
    
    print("🔄 Testing Turn-in-Place Functionality")
    print("=" * 50)
    
    # Initialize PyBullet integration
    integration = PyBulletIntegration(
        env_radius=1.0,
        target_zone_radius=0.3,
        num_pebbles=10,  # Less pebbles for cleaner testing
        random_seed=42,
        gui=True,
        initial_robot_pose=(0.0, 0.0, 0.0)  # Start facing East (0°)
    )
    
    integration.set_robot_dim(L=0.2, R=0.07)
    
    print("Setting up 3D environment...")
    objects_3d = integration.run(auto_continue=True)
    
    # Test scenarios with different target angles
    test_angles = [
        (math.radians(30), "30° - Should NOT turn in place (< 35°)"),
        (math.radians(45), "45° - Should turn in place"),
        (math.radians(90), "90° - Should turn in place"), 
        (math.radians(135), "135° - Should turn in place"),
        (math.radians(180), "180° - Full reverse, should turn in place"),
        (math.radians(-90), "-90° - Should turn in place (CCW)")
    ]
    
    for target_angle, description in test_angles:
        print(f"\n🎯 Test: {description}")
        print(f"   Target angle: {math.degrees(target_angle):.1f}°")
        
        # Get current robot pose
        (x, y), current_theta = integration.pose()
        print(f"   Current angle: {math.degrees(current_theta):.1f}°")
        
        # Calculate heading error
        from pybullet_integration import wrap_angle
        hdg_err = wrap_angle(target_angle - current_theta)
        print(f"   Heading error: {math.degrees(hdg_err):.1f}°")
        
        # Draw target direction line for visualization
        target_x = x + 0.5 * math.cos(target_angle)
        target_y = y + 0.5 * math.sin(target_angle)
        p.addUserDebugLine([x, y, 0.05], [target_x, target_y, 0.05], 
                          [1, 0, 0], lineWidth=4.0, lifeTime=10.0)
        
        # Add text label
        p.addUserDebugText(f"{math.degrees(target_angle):.0f}°", 
                          [target_x, target_y, 0.1], 
                          textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=10.0)
        
        # Test the turn-in-place function
        print("   Executing turn-in-place...")
        start_time = time.time()
        
        try:
            integration._turn_in_place(target_angle, max_rate=2.0, tol=0.05)
            end_time = time.time()
            
            # Verify final angle
            (final_x, final_y), final_theta = integration.pose()
            final_error = abs(wrap_angle(target_angle - final_theta))
            
            print(f"   ✅ Turn completed in {end_time - start_time:.2f}s")
            print(f"   Final angle: {math.degrees(final_theta):.1f}°")
            print(f"   Final error: {math.degrees(final_error):.2f}°")
            
            # Check if rover stayed in place (didn't translate much)
            distance_moved = math.hypot(final_x - x, final_y - y)
            print(f"   Distance moved: {distance_moved:.4f}m (should be ~0)")
            
            if final_error > math.radians(10):  # 10 degree tolerance
                print(f"   ⚠️  Warning: Large final error!")
            if distance_moved > 0.1:  # 10cm tolerance  
                print(f"   ⚠️  Warning: Rover moved significantly!")
                
        except Exception as e:
            print(f"   ❌ Error during turn: {e}")
        
        # Wait for user input to proceed to next test
        print("   Press ENTER to continue to next test...")
        input()
        
        # Clear debug lines for next test
        p.removeAllUserDebugItems()
    
    print("\n🎉 All turn-in-place tests completed!")
    print("\nTest Summary:")
    print("- Green approach lines should show smooth curves after turns")
    print("- Rover should rotate in place without significant translation")  
    print("- Angles > 35° should trigger turn-in-place behavior")
    print("- Final heading should match target angle within tolerance")
    
    input("\nPress ENTER to exit...")
    integration.close_environment()

if __name__ == "__main__":
    test_turn_in_place_functionality()