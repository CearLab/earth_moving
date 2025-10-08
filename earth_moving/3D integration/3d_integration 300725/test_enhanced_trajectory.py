import math
from pybullet_integration import PyBulletIntegration
from main import run_2d_env

def test_enhanced_trajectory_with_extension():
    """
    Interactive test for enhanced trajectory execution with backward extension.
    
    MUCH SIMPLER APPROACH:
    - Just like the main orchestrator, but using the enhanced trajectory execution
    - You click cells interactively to see the new behavior
    - Visual feedback shows extension vs. fillet decisions
    """
    
    print("🚀 Interactive Enhanced Trajectory Test")
    print("=" * 50)
    print("This works just like the main orchestrator, but uses the enhanced trajectory system.")
    print("\n🎯 What to Look For:")
    print("📍 SHARP TURNS (>35°):")
    print("   - Orange line shows extended pre-task trajectory (~0.44m back)")
    print("   - Red 'TURN' marker shows where turn-in-place happens")
    print("   - Turn happens OUTSIDE collection cells")
    print("\n📍 MODERATE TURNS (<35°):")
    print("   - Uses fillet smoothing (existing GPT improvement)")
    print("   - No extension, no turn-in-place marker")
    print("\n📍 SMOOTH MODE (Sharp turns, turn-in-place disabled):")
    print("   - Extends task trajectory backwards (60% of turn-in-place distance)")
    print("   - Creates gentler approach curves with reduced curvature")
    print("   - Wider, more gradual trajectories for smoother overall path")
    print("   - Skips fillet for continuous gentle curve from approach to task")
    print("\n📍 Visual Legend:")
    print("   🟢 Green = Approach trajectory")
    print("   🌊 Light Blue = SMOOTH approach (when turn-in-place disabled)")
    print("   🟠 Orange = Extended pre-task trajectory") 
    print("   🔴 Red TURN = Turn-in-place position")
    print("   🔵 Blue = Collection task trajectory")
    print("   ⚫ Black = Resampled path")
    print("\n⚙️ Controls:")
    print("   T = Toggle turn-in-place ON/OFF")
    print("   ESC = Exit")
    
    # Control state
    turn_in_place_enabled = True  # Default: enabled
    
    # Initialize exactly like main orchestrator
    integration = PyBulletIntegration(
        env_radius=1.0,
        target_zone_radius=0.3,
        num_pebbles=40,
        random_seed=10,
        gui=True,
        initial_robot_pose=(0.0, -2.0, math.pi / 2)  # Same as main orchestrator
    )
    
    integration.set_robot_dim(L=0.2, R=0.07)
    
    print("\nSetting up 3D environment...")
    objects_3d = integration.run(auto_continue=True)
    shovel_width = integration.shovel_width
    
    # Initialize 2D environment
    env, visualizer = run_2d_env(
        env_radius=1.0,
        target_zone_radius=0.3,
        shovel_width=shovel_width,
        real_objects=objects_3d,
        manual_mode=False
    )
    
    print(f"\n✅ Ready! Turn-in-place: {'🟢 ENABLED' if turn_in_place_enabled else '🔴 DISABLED'}")
    print("   Click cells to test, press 'T' to toggle turn-in-place on/off")
    
    # Main loop - exactly like orchestrator.py structure
    import pygame
    running = True
    while running:
        visualizer.screen.fill((255, 255, 255))
        visualizer.draw_elements()
        pygame.display.flip()
        visualizer.clock.tick(30)
        
        # Handle events inline (like orchestrator.py)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_t:  # Toggle turn-in-place
                    turn_in_place_enabled = not turn_in_place_enabled
                    status = '🟢 ENABLED' if turn_in_place_enabled else '🔴 DISABLED'
                    print(f"\n⚙️ Turn-in-place toggled: {status}")
                    print("   Click a cell to test the new setting...")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                clicked_cell = visualizer.handle_click_event(pos)
                if clicked_cell:
                    print(f"\n🎯 Testing cell: ({clicked_cell.x}, {clicked_cell.y}) with {clicked_cell.num_objects} objects")

                    trajectory = env.get_trajectory(clicked_cell, "target", True)
                    env.current_trajectory = trajectory
                    env.current_cell = clicked_cell
                    env.current_path_type = "target"
                    env.current_use_spillage = True

                    print("Visualizing trajectory...")
                    visualizer.set_trajectory(trajectory)
                    integration.visualize_trajectory(trajectory)

                    # USE ENHANCED EXECUTION with current turn-in-place setting
                    mode_str = "WITH turn-in-place" if turn_in_place_enabled else "WITHOUT turn-in-place"
                    print(f"🚀 Executing ENHANCED trajectory system {mode_str}...")
                    transfer_result = integration.execute_enhanced_pure_pursuit_trajectory(
                        env, 
                        mode="TCP",
                        enable_turn_in_place=turn_in_place_enabled,  # Use current setting
                        v_nom=0.35,  # Slightly slower for better observation
                        pivot_thresh_deg=35.0,
                        pivot_rate=2.0
                    )
                    
                    # Handle 2D environment recreation (same as orchestrator)
                    if transfer_result and transfer_result.get('needs_2d_recreation', False):
                        print("🔄 Recreating 2D environment with updated object positions...")
                        updated_positions_3d = transfer_result.get('new_object_positions_3d', [])
                        
                        env, visualizer = run_2d_env(
                            env_radius=1.0,
                            target_zone_radius=0.3,
                            shovel_width=integration.shovel_width,
                            real_objects=updated_positions_3d,
                            manual_mode=False
                        )
                        print("✅ 2D environment recreated!")

                    # Clean up for next test
                    integration.clear_trajectory()
                    visualizer.clear_trajectory()
                    env.current_trajectory = None
                    env.current_cell = None
                    env.current_path_type = None
                    env.current_use_spillage = None
                    
                    print("✅ Test completed - click another cell or press ESC to exit")

    pygame.quit()
    integration.close_environment()

if __name__ == "__main__":
    test_enhanced_trajectory_with_extension()