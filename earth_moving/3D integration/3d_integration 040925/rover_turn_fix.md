# Rover Keeps Moving After Turn — Debug & Fix Notes

## Short Answer
It “keeps moving” because the stop logic inside `_turn_in_place` isn’t actually braking the wheels, and you also step the physics twice per control cycle. Both together let the rover coast after the turn.

---

## Issues Identified

### 1. Gradual Velocity Reduction Loop
```python
for i in range(40):
    reduction_factor = (40 - i) / 40.0
    self.control_rover_velocity(0.0 * reduction_factor, 0.0 * reduction_factor)
    p.stepSimulation()
```

- `0.0 * reduction_factor` is always **0**, so there’s no ramp-down of the last command.
- This results in an immediate drop to free-wheel + extra stepping.

---

### 2. Motors Explicitly Disabled
- You call `setJointMotorControl2(..., force=0)` on both wheels.
- That removes active braking, leaving wheels free to roll.
- Residual momentum + low-ish floor friction → **slow drift**.

---

### 3. Double Simulation Step
- `control_rover_velocity(...)` already steps the sim (`self.simulate(time)`).
- Then you call `p.stepSimulation()` again inside the PID loop.
- Net effect: control runs at **2× rate** → hotter gains and extra motion.

---

### 4. Hand-Crafted Wheel Speeds
- Current approach: directly assign L/R wheel speeds from `control_signal` (rad/s).
- Correct approach: use helper `set_wheel_speeds_unitsafe(0.0, yaw_rate)` so the right kinematics are applied.

---

### 5. Floor Friction Is Modest (Minor)
- Wheels: `lateralFriction=10`
- Floor: `lateralFriction=1.0`
- With motors disabled, this floor value makes coasting more likely.

---

## Minimal Patch (Stops on the Spot)

**Goal:**  
- (a) Command yaw-rate via helper  
- (b) Don’t double-step  
- (c) Actively brake to zero  
- (d) Don’t disable motors — use smooth stopper  

### Patch for `pybullet_integration.py` (`_turn_in_place`)
```diff
@@
-            # For pure rotation: opposite wheel directions
-            if control_signal > 0:  # Turn counter-clockwise (CCW)
-                left_vel = -abs(control_signal)
-                right_vel =  abs(control_signal)
-            else:                   # Turn clockwise (CW)
-                left_vel =  abs(control_signal)
-                right_vel = -abs(control_signal)
-
-            # Apply wheel velocities
-            self.control_rover_velocity(left_vel, right_vel)
-
-            # Step simulation and update
-            p.stepSimulation()
+            # Drive by yaw-rate only; helper converts to correct wheel speeds & steps once
+            self.set_wheel_speeds_unitsafe(0.0, control_signal, dt=1/240)
+            # (No extra p.stepSimulation here)

@@
-        # Step 1: Gradual velocity reduction to avoid abrupt stopping
-        print(f"   🔧 Gradual velocity reduction.")
-        for i in range(40):
-            reduction_factor = (40 - i) / 40.0
-            # Gradually reduce to zero
-            self.control_rover_velocity(0.0 * reduction_factor, 0.0 * reduction_factor)
-            p.stepSimulation()
+        # Step 1: Actively ramp yaw-rate down to zero (real braking)
+        print(f"   🔧 Gradual velocity reduction...")
+        cmd_prev = 0.0
+        # store the last control_signal from the loop above
+        try:
+            cmd_prev = control_signal
+        except NameError:
+            cmd_prev = 0.0
+        for i in range(40):
+            factor = (40 - i) / 40.0
+            self.set_wheel_speeds_unitsafe(0.0, cmd_prev * factor, dt=1/240)

@@
-        # Step 2: Motor disable approach - completely disable motor control
-        print(f"   🔌 Disabling motor control.")
-        p.setJointMotorControl2(robot_id, left_joint,  p.VELOCITY_CONTROL, targetVelocity=0, force=0)
-        p.setJointMotorControl2(robot_id, right_joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
-        ...
-        # Step 4: Re-enable motors with low force and zero velocity
-        print(f"   🔄 Re-enabling motors with minimal control.")
-        p.setJointMotorControl2(robot_id, left_joint,  p.VELOCITY_CONTROL, targetVelocity=0.0, force=1000)
-        p.setJointMotorControl2(robot_id, right_joint, p.VELOCITY_CONTROL, targetVelocity=0.0, force=1000)
-
-        # Final settling
-        for _ in range(60):
-            p.stepSimulation()
+        # Final: active stop + settle (uses velocity=0 with motor force to hold)
+        self._stop_robot_smoothly(settle_time=1.0)
```

---

## Why This Works
- `set_wheel_speeds_unitsafe(0, yaw_rate)` → correct conversion, avoids double stepping.  
- Ramp-down now gradually reduces yaw-rate to **0** instead of dropping to free-wheel.  
- `_stop_robot_smoothly(...)` keeps motors engaged at zero velocity, holding the stop.  

---

## Optional Improvement
To reduce residual creep, adjust dynamics once at setup:

```python
p.changeDynamics(rover_id, left_joint,  lateralFriction=8.0, rollingFriction=0.1, spinningFriction=0.1)
p.changeDynamics(rover_id, right_joint, lateralFriction=8.0, rollingFriction=0.1, spinningFriction=0.1)
p.changeDynamics(self.object_ids[0], -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
```

- Leave motors enabled with `targetVelocity=0` so they hold the stop.

---

## Verification
- Run `test_turn_in_place.py` again.  
- Rover should rotate to the target angle and stay put.  
- Harness is already fine: it calls `_turn_in_place` and checks displacement.  

---

## Future Improvement
- Optionally, factor `_turn_in_place` to reuse `_stop_robot_smoothly` so both pure-pursuit and turn-in-place use identical stop behavior.
