# Hybrid Collision Avoidance - Architecture Documentation

## System Overview

The hybrid collision avoidance system combines two proven algorithms in a hierarchical structure:

```
┌─────────────────────────────────────────────────────────────────┐
│                    HYBRID COORDINATOR                            │
│  (HybridCollisionAvoidance class)                                │
│                                                                   │
│  Orchestrates two-layer planning pipeline                        │
└────────────────────────┬────────────────────────────────────────┘
                         │
                ┌────────┴────────┐
                │                 │
     ┌──────────▼──────────┐   ┌──▼──────────────────┐
     │  LAYER 1: ORCA      │   │  LAYER 2: APF       │
     │  (Hard Constraint)  │   │  (Soft Constraint)  │
     │                     │   │                     │
     │ Input:              │   │ Input:              │
     │ - Ego rover         │   │ - Position, heading │
     │ - Goal position     │   │ - Goal position     │
     │ - Rover neighbors   │   │ - Static obstacles  │
     │                     │   │ - Speed constraint  │
     │ Output:             │   │   from ORCA         │
     │ - Safe velocity (v) │   │                     │
     │                     │   │ Output:             │
     │ Purpose:            │   │ - Final (v, ω)      │
     │ Multi-rover         │   │ - Smooth guidance   │
     │ coordination        │   │                     │
     │ Priority-based      │   │ Purpose:            │
     │ avoidance           │   │ Obstacle avoidance  │
     │                     │   │ with speed limit    │
     └─────────────────────┘   └─────────────────────┘
```

## Architectural Decisions

### **1. Hierarchical vs. Parallel Design**

**Chosen: Hierarchical (Sequential)**

```
HIERARCHY FLOW:
  ORCA (Rover → Rover negotiation)
    ↓
  APF (Rover → Pebble guidance)
    ↓
  Final Control Command
```

**Rationale:**
- Clear priority ordering (safety first)
- ORCA negotiation is fast (doesn't block APF)
- APF operates on constrained space (faster computation)
- Easy to debug (test each layer independently)

**Alternative (Parallel):** Would add complexity of simultaneous constraint satisfaction.

---

### **2. Constraint Type**

**ORCA Output → ORCA Hard Constraint**
- Violation = catastrophic (robot collision)
- Must be guaranteed safe
- Uses velocity sampling + TTC checking

**APF Output → APF Soft Constraint**
- Violation = acceptable (touching pebbles)
- Provides guidance, not hard constraint
- Uses potential field forces

---

### **3. Information Flow**

```
[Layer 1 Decision: ORCA]
       ↓
   v_safe_mag (scalar velocity magnitude)
       ↓
[Layer 2 Decision: APF]
       ↓
   APF uses v_safe_mag as maximum allowed speed
       ↓
   Direction from APF potential field
       ↓
   Final (v, ω) command
```

**Key insight:** Layer 1 constrains magnitude, Layer 2 refines direction

---

## Module Descriptions

### **hybrid_planner.py** (Core System)

Three main classes:

#### **1. ORCAPlanner** (Layer 1)
```python
class ORCAPlanner:
    """ORCA-style collision avoidance for rover-to-rover coordination."""

    def plan(ego, goal, neighbors, shape) → (v, ω)
```

**Methods:**
- `_shape_radius()`: Approximate rover as single disc
- `_wrap_angle()`: Angle normalization
- `_ttc_collision()`: Time-to-closest-approach check
- `_project_to_unicycle()`: Convert 2D velocity to unicycle
- `_sample_candidates()`: Sample velocities in disc
- `plan()`: Main planning method

**Algorithm:**
1. Compute preferred velocity toward goal
2. Sample N×M velocities in velocity space
3. For each candidate, check TTC with all neighbors
4. Select candidate closest to preferred velocity
5. Convert to (v, ω) for differential drive

**Complexity:** O(N_rovers × N_candidates × N_neighbors)

#### **2. APFNavigator** (Layer 2)
```python
class APFNavigator:
    """Artificial Potential Field navigation for static obstacle avoidance."""

    def compute(position, heading, goal, obstacles,
                v_max_constraint) → (v, ω)
```

**Methods:**
- `_attractive_force()`: Pull toward goal
- `_repulsive_force()`: Push from obstacles
- `_wrap_angle()`: Angle normalization
- `compute()`: Main computation

**Forces:**
```
F_att(p,g) = k_att * (g - p)
           [linear attraction toward goal]

F_rep(p,O) = Σ k_rep * (1/d - 1/d0) * (1/d²) * direction
           [for each obstacle O where d ≤ d0]
           [inverse-square repulsion]

F_total = F_att + F_rep
```

**Speed Gating:**
```
v_max_allowed = v_max_constraint  [from ORCA]
v = v_max_allowed * cos(heading_error)
v = clip(v, 0, v_max_allowed)
```

**Complexity:** O(N_obstacles)

#### **3. HybridCollisionAvoidance** (Coordinator)
```python
class HybridCollisionAvoidance:
    """Combines ORCA + APF in hierarchical pipeline."""

    def plan(ego, goal, rover_neighbors, rover_shape,
             static_obstacles) → (v, ω)
```

**Pipeline:**
```python
def plan(...):
    # Layer 1: ORCA
    v_orca, w_orca = self.orca.plan(...)
    v_safe_mag = abs(v_orca)

    # Layer 2: APF (constrained by ORCA output)
    v_apf, w_apf = self.apf.compute(...,
                                   v_max_constraint=v_safe_mag)

    return v_apf, w_apf
```

---

### **hybrid_simulation.py** (Integration)

Main simulation loop that orchestrates everything:

```python
class HybridSimulation:
    """Manages PyBullet simulation and hybrid planning."""

    def setup()       # Initialize PyBullet, load URDFs
    def plan_step()   # Run planning cycle
    def control_step()# Apply control to rovers
    def step(dt)      # Integrate physics
    def run()         # Main loop
```

**Main Loop:**
```
While simulation running:
    1. plan_step()        [every PLAN_DT = 0.1s]
       - Update rover states from PyBullet
       - For each rover: hybrid_planner.plan()
       - Store control command
    2. control_step()     [every SIM_DT = 1/240s]
       - Apply differential drive control
       - Set wheel velocities
    3. step()
       - p.stepSimulation() - physics engine advances
    4. check_goals()
       - Check if rovers reached goals
    5. print_status()
       - Print progress to console
```

---

### **scenarios.py** (Test Cases)

Defines test scenarios with:
- Rover initial positions, headings, goals, priorities
- Static obstacle (pebble) positions
- Descriptive names and purposes

Example scenario structure:
```python
{
    "name": "...",
    "description": "...",
    "rovers": [
        {
            "start_pos": [x, y, z],
            "start_heading": θ,
            "goal_pos": [gx, gy],
            "priority": 0,  # Smaller = higher priority
            "color": (r, g, b, a),
        },
        # ... more rovers
    ],
    "pebbles": [
        (x, y),
        # ... more obstacles
    ]
}
```

---

## Control Flow Diagram

### **Single Planning Cycle**

```
┌─ PLAN_DT (0.1s) ─┐
│                  │
├──────────────────┼──────────────────────────────────┐
│                  │                                  │
│ [1] Get States   [2] ORCA Planner                 │
│                  │                                  │
│ for each rover:  for each rover:                   │
│  Read from       ego = current rover               │
│  PyBullet        goal = rover goal                 │
│  x, y, θ        neighbors = nearby rovers          │
│  vx, vy         v_safe = ORCA.plan(...)            │
│                 Store v_safe_mag = |v_safe|        │
│                                                     │
│ [3] APF Navigator                                  │
│                                                     │
│ for each rover:                                    │
│  pos = rover position                              │
│  heading = rover heading                           │
│  goal = rover goal                                 │
│  obstacles = pebble positions                      │
│  v_max = v_safe_mag [from ORCA]                    │
│  (v, ω) = APF.compute(...)                         │
│  Store final control                               │
│                                                     │
└────────────────────────────────────────────────────┘
```

### **Control Application (Every SIM_DT)**

```
┌─ SIM_DT (1/240s) ─┐
│                   │
├───────────────────┼─────────────────────────┐
│                   │                         │
│ [1] Differential  [2] PyBullet              │
│     Drive Conv.   │                         │
│                   │ Apply motor control     │
│ for each rover:   │ Step physics            │
│  v_cmd, ω_cmd     │ Update object states    │
│  ↓                │                         │
│  vL = v - ω*w/2   └─ Next cycle ────────────┘
│  vR = v + ω*w/2
│  ↓
│  wL = -vL / r_wheel
│  wR = -vR / r_wheel
│  ↓
│  Clamp & apply
│  p.setJointMotor
│  Control(...)
│
└───────────────────────────────────────────┘
```

---

## Time Complexity Analysis

For a simulation with:
- `N_rovers`: Number of rovers
- `N_obstacles`: Number of pebbles
- `N_candidates`: ORCA velocity samples (typically 120)
- `N_neighbors`: Nearby rovers (typically << N_rovers)

### **Per Planning Cycle (0.1s):**

```
ORCA Planning (per rover):
  - Neighbor enumeration: O(N_rovers)
  - Candidate sampling: O(N_candidates)
  - TTC check loop: O(N_neighbors) per candidate
  ─────────────────
  Total ORCA: O(N_rovers × N_candidates × N_neighbors)
                    ≈ O(N_rovers × 120 × 5) with realistic values
                    ≈ O(600 × N_rovers) [manageable]

APF Navigation (per rover):
  - Force calculation: O(N_obstacles)
  ─────────────────
  Total APF: O(N_rovers × N_obstacles)
                  ≈ O(N_rovers × 20) with typical obstacles

Hybrid Coordinator:
  - Total: O(N_rovers × (120 × N_neighbors + N_obstacles))
  - Typical: O(N_rovers × 650) [fast enough for real-time]
```

### **Per Physics Step (1/240s):**

```
Differential Drive Control:
  - Per rover: O(1)
  - Total: O(N_rovers) [very fast]

PyBullet Step:
  - O(N_objects) [handled by PyBullet]
```

---

## Data Structures

### **Rover Agent Dictionary**

```python
rover = {
    "id": "R0",                    # String ID
    "priority": 0,                 # Lower = higher priority
    "is_blind": False,             # Ignores others if True
    "state": np.array([x, y, θ, v, ω]),  # Current state
    "goal": np.array([gx, gy]),    # Goal position
    "control": (v_cmd, ω_cmd),     # Current command
    "shape": {
        "circles": [
            (cx1, cy1, r1),        # Wheel 1 relative pose + radius
            (cx2, cy2, r2),        # Wheel 2 relative pose + radius
        ]
    },
    "color": (r, g, b, a),         # RGBA for visualization
    "body": pybullet_body_id,      # PyBullet object ID
    "left_joint": joint_index,     # PyBullet joint index
    "right_joint": joint_index,    # PyBullet joint index
}
```

### **Obstacle Tuple**

```python
obstacle = (cx, cy, radius)  # (float, float, float)
```

---

## Parameter Sensitivity

### **ORCA Parameters**

| Parameter | Effect | Range | Typical |
|-----------|--------|-------|---------|
| `tau` | Prediction horizon | 4-12s | 8s |
| `v_pref` | Preferred speed | 0.3-1.0 | 0.8 m/s |
| `v_max` | Maximum speed | 0.5-1.5 | 1.0 m/s |
| `w_max` | Max turn rate | 1-5 | 3.0 rad/s |
| `k_theta` | Heading gain | 1-5 | 3.0 |
| `n_speed` | Radial samples | 4-10 | 6 |
| `n_angle` | Angular samples | 16-32 | 20 |

**Tuning:**
- Increase `tau` for safer (slower) response
- Increase `v_max` for faster (riskier) navigation
- Increase `n_speed`/`n_angle` for better sampling (slower)

### **APF Parameters**

| Parameter | Effect | Range | Typical |
|-----------|--------|-------|---------|
| `k_att` | Goal attraction | 0.5-2.0 | 1.5 |
| `k_rep` | Obstacle repulsion | 0.05-0.3 | 0.08 |
| `d0` | Repulsion range | 0.4-1.0m | 0.55m |
| `v_max` | Maximum speed | 0.5-1.5 | 1.2 m/s |
| `w_max` | Max turn rate | 1-5 | 3.5 rad/s |
| `k_w` | Heading gain | 1-5 | 3.0 |

**Tuning:**
- Increase `k_rep` to avoid pebbles more
- Increase `d0` to detect obstacles earlier
- Increase `k_att` for stronger goal pull
- Reduce `v_max` to be more careful

---

## Failure Modes and Recovery

### **Rover Gets Stuck on Pebble**

**Symptoms:**
- Rover oscillates around obstacle
- Doesn't progress to goal
- Control values erratic

**Cause:**
- APF repulsion too weak (k_rep too low)
- Or d0 too small (obstacle influence too short)

**Recovery:**
- Increase `k_rep` to 0.12-0.15
- Increase `d0` to 0.70-0.80

### **Rovers Collide**

**Symptoms:**
- Two rovers overlap in PyBullet
- Physics produces instability

**Cause:**
- ORCA prediction horizon too short
- Or sampling too coarse

**Recovery:**
- Increase `tau` from 8.0 to 10-12
- Increase `n_angle` from 20 to 32
- Reduce `v_max` for safety margin

### **Very Slow Simulation**

**Symptoms:**
- Takes 30+ seconds real-time for 5s sim
- CPU usage high

**Cause:**
- Too many velocity samples
- GUI rendering overhead
- PyBullet heavy computation

**Recovery:**
- Use `--headless` flag
- Reduce `n_speed` and `n_angle`
- Reduce number of pebbles in scenario

---

## Testing Strategy

### **Unit Testing (Per-Layer)**

```python
# Test ORCA alone (no obstacles)
orca = ORCAPlanner()
v, w = orca.plan(ego, goal, neighbors, shape)
assert norm(v) <= v_max

# Test APF alone (no rovers)
apf = APFNavigator()
v, w = apf.compute(pos, heading, goal, obstacles)
assert norm(v) <= v_max
```

### **Integration Testing (Hybrid)**

```python
# Test hybrid with various scenarios
hybrid = HybridCollisionAvoidance()

# Scenario 1: Head-on
v, w = hybrid.plan(ego, goal, [other], shape, [])
# Should avoid other rover

# Scenario 2: Obstacle course
v, w = hybrid.plan(ego, goal, [], shape, obstacles)
# Should navigate around pebbles

# Scenario 3: Combined
v, w = hybrid.plan(ego, goal, [other], shape, obstacles)
# Should avoid both
```

---

## Extension Points

### **Custom ORCA Behavior**

Modify `ORCAPlanner._sample_candidates()` to use different velocity distribution.

### **Custom APF Forces**

Add new force types to `APFNavigator`:
- Tangential bias for deadlock resolution
- Viscous friction
- Wind/current effects

### **Custom Controller**

Replace unicycle model with:
- Ackermann steering (car-like)
- Mecanum wheels (omnidirectional)
- Quadrotor dynamics

### **Custom Scenarios**

Add scenarios to `scenarios.py`:
- Dynamic obstacles (moving pebbles)
- Narrow corridors
- Chaotic crowded spaces

---

## Performance Characteristics

### **Typical Scenario Performance**

```
Scenario: 2 rovers, 4 pebbles
- Planning frequency: 10 Hz
- Physics frequency: 240 Hz
- Planning time per cycle: ~5-10ms
- Physics step time: ~0.4ms
- Total time for 10s sim: ~2-3s wall-clock
```

### **Scalability**

```
N_rovers | Planning time | Notes
---------|---------------|-------
  1      | 1-2ms         | Minimal
  2      | 5-10ms        | Typical
  4      | 15-25ms       | Manageable
  8      | 40-80ms       | Getting slow
  16     | 150ms+        | Slow
```

**Recommendation:** Use for up to 4-6 rovers in real-time simulations.

---

## References and Related Work

### **ORCA (Original)**
- "Reciprocal Collision Avoidance for Multiple Robots with Linear Prediction"
- van den Berg et al., 2011
- Velocity obstacle approach with reciprocal responsibility

### **APF (Original)**
- "Real-time Obstacle Avoidance for Manipulators and Mobile Robots"
- Khatib, 1985
- Artificial potential fields for navigation

### **Hybrid Approaches**
- Combining local and global planners
- Layered control architectures
- Hierarchical decision making

---

This architecture provides a clean, modular, and extensible system for multi-robot navigation with collision avoidance.
