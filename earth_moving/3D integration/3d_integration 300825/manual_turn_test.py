import math
import pybullet as p
from pybullet_integration import PyBulletIntegration

def manual_turn_test():
    """
    Interactive test for turn-in-place functionality.
    User can press keys to test different turn angles.
    """
    
    print("🎮 Manual Turn-in-Place Test")
    print("=" * 40)
    print("Controls:")
    print("  1 = Turn 30°")
    print("  2 = Turn 45°") 
    print("  3 = Turn 90°")
    print("  4 = Turn 135°")
    print("  5 = Turn 180°")
    print("  6 = Turn -90° (CCW)")
    print("  r = Reset to 0°")
    print("  q = Quit")
    print("=" * 40)
    
    # Initialize environment
    integration = PyBulletIntegration(
        env_radius=1.0,
        target_zone_radius=0.3,
        num_pebbles=5,  # Minimal pebbles for cleaner view
        random_seed=42,
        gui=True,
        initial_robot_pose=(0.0, 0.0, 0.0)  # Start facing East
    )
    
    integration.set_robot_dim(L=0.2, R=0.07)
    
    print("\nSetting up environment...")
    objects_3d = integration.run(auto_continue=True)
    
    # Draw coordinate axes for reference
    # X-axis (East) - Red
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], lineWidth=3)
    p.addUserDebugText("East (0°)", [1, 0, 0.1], textColorRGB=[1, 0, 0], textSize=1)
    
    # Y-axis (North) - Green  
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], lineWidth=3)
    p.addUserDebugText("North (90°)", [0, 1, 0.1], textColorRGB=[0, 1, 0], textSize=1)
    
    # -X-axis (West) - Blue
    p.addUserDebugLine([0, 0, 0], [-1, 0, 0], [0, 0, 1], lineWidth=3)
    p.addUserDebugText("West (180°)", [-1, 0, 0.1], textColorRGB=[0, 0, 1], textSize=1)
    
    # -Y-axis (South) - Yellow
    p.addUserDebugLine([0, 0, 0], [0, -1, 0], [1, 1, 0], lineWidth=3)
    p.addUserDebugText("South (-90°)", [0, -1, 0.1], textColorRGB=[1, 1, 0], textSize=1)
    
    def show_current_heading():
        (x, y), theta = integration.pose()
        print(f"Current heading: {math.degrees(theta):.1f}°")
        
        # Draw current heading vector
        end_x = x + 0.3 * math.cos(theta)
        end_y = y + 0.3 * math.sin(theta)
        p.addUserDebugLine([x, y, 0.05], [end_x, end_y, 0.05], 
                          [1, 0, 1], lineWidth=5, lifeTime=0.5)
    
    def test_turn(target_degrees):
        target_radians = math.radians(target_degrees)
        print(f"\n🔄 Turning to {target_degrees}°...")
        
        try:
            # Make sure rover is completely stopped before turning
            integration.control_rover_velocity(0.0, 0.0)
            for _ in range(60):  # Let it settle for 60 frames (0.25 seconds)
                p.stepSimulation()
            
            integration._turn_in_place(target_radians, max_rate=3.0, tol=0.03)
            
            # Make sure rover is completely stopped after turning
            integration.control_rover_velocity(0.0, 0.0)
            for _ in range(60):  # Let it settle for 60 frames (0.25 seconds)
                p.stepSimulation()
            
            show_current_heading()
            print("✅ Turn completed!")
            
        except Exception as e:
            print(f"❌ Turn failed: {e}")
    
    # Main interaction loop
    print("\nReady! Current rover is at origin facing East (0°)")
    show_current_heading()
    
    while True:
        try:
            print("\nEnter command (1-6, r, q): ", end="", flush=True)
            key = input().strip().lower()
            
            # Handle single character commands only
            if len(key) > 1:
                key = key[0]  # Take first character only
            
            if key == '1':
                test_turn(30)
            elif key == '2':
                test_turn(45)
            elif key == '3':
                test_turn(90)
            elif key == '4':
                test_turn(135)
            elif key == '5':
                test_turn(180)
            elif key == '6':
                test_turn(-90)
            elif key == 'r':
                test_turn(0)
            elif key == 'q':
                print("Exiting...")
                break
            else:
                print(f"Invalid command '{key}'. Use 1-6, r, or q")
                continue
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except EOFError:
            print("\nExiting...")
            break
    
    print("\n👋 Exiting...")
    integration.close_environment()

if __name__ == "__main__":
    manual_turn_test()