# Hybrid Collision Avoidance System

A sophisticated hierarchical collision avoidance system that combines **ORCA** (Optimal Reciprocal Collision Avoidance) for rover-to-rover coordination with **APF** (Artificial Potential Field) for smooth static obstacle navigation.

## 🎯 Overview

This system implements a **two-layer hierarchical approach**:

```
Layer 1: ORCA Planner (Hard Constraint)
  └─ Avoids other ROVERS (critical safety)
     └─ Outputs: safe velocity magnitude
         └─ Layer 2: APF Navigator (Soft Constraint)
            ├─ Takes velocity magnitude as speed constraint
            ├─ Applies potential field around pebbles
            ├─ Enables natural pushing behavior
            └─ Outputs: final (v, ω) commands
```

## ✨ Key Features

### **Priority-Based Collision Avoidance**
- Higher-priority rovers get right-of-way
- Lower-priority rovers must yield
- Multi-agent negotiation prevents deadlock

### **Soft Obstacle Constraints**
- Static pebbles are avoided but not strictly
- Rovers can push pebbles if needed
- Natural, smooth navigation around obstacles

### **Clean Separation of Concerns**
- ORCA handles multi-agent coordination
- APF provides smooth obstacle guidance
- Each layer can be tested independently

### **Multiple Test Scenarios**
- Head-on collision avoidance
- Multi-robot crossing
- Corridor navigation
- Maze solving
- Goal convergence

## 📂 Directory Structure

```
hybrid_collision_avoidance/
├── hybrid_planner.py           # Core planner (ORCA + APF)
├── hybrid_simulation.py         # Main simulation script (RUN THIS)
├── scenarios.py                 # Test scenario definitions
├── 2_wheel_rover.urdf          # Rover robot model
├── pebbles.urdf                # Obstacle model
├── README.md                   # This file
├── QUICKSTART.md               # Quick start guide
└── ARCHITECTURE.md             # Detailed architecture documentation
```

## 🚀 Quick Start

### **Run Default Scenario**
```bash
python hybrid_simulation.py
```

### **List Available Scenarios**
```bash
python hybrid_simulation.py --list-scenarios
```

### **Run Specific Scenario**
```bash
python hybrid_simulation.py --scenario 4_crossing_obstacles
```

### **Headless Mode (No GUI)**
```bash
python hybrid_simulation.py --headless --max-time 30
```

### **Custom Time Limit**
```bash
python hybrid_simulation.py --max-time 60
```

## 📊 Available Scenarios

| Scenario | Name | Description |
|----------|------|-------------|
| `2_head_on_pebbles` | 2 Rovers Head-On | Two robots head-on with obstacles |
| `4_crossing_obstacles` | 4 Robots Crossing | Four robots crossing at center |
| `2_sparse` | 2 Rovers (Sparse) | Sparse obstacles, smooth navigation |
| `2_corridor` | 2 Rovers (Corridor) | Dense pebble corridor, pushing expected |
| `3_convergence` | 3 Rovers Convergence | Three robots to center |
| `1_maze` | Single Rover Maze | Complex obstacle maze |

## 🏗️ Architecture

### **Layer 1: ORCA Planner**

**Class**: `ORCAPlanner`

Handles rover-to-rover collision avoidance using velocity sampling and time-to-closest-approach (TTC) checking.

**Key Parameters:**
```python
tau = 8.0           # Prediction horizon (seconds)
v_pref = 0.8        # Preferred speed toward goal
v_max = 1.0         # Maximum translational speed
w_max = 3.0         # Maximum angular speed
k_theta = 3.0       # Heading feedback gain
n_speed = 6         # Velocity space samples (radial)
n_angle = 20        # Velocity space samples (angular)
```

**Output**: `(v, ω)` - Safe velocity considering only rover neighbors

### **Layer 2: APF Navigator**

**Class**: `APFNavigator`

Smooth obstacle avoidance using potential field with speed constraint from ORCA.

**Key Parameters:**
```python
k_att = 1.5         # Attractive gain (goal pull)
k_rep = 0.08        # Repulsive gain (obstacle push)
d0 = 0.55           # Repulsion cutoff (meters)
v_max = 1.2         # Maximum velocity
w_max = 3.5         # Maximum angular velocity
k_w = 3.0           # Angular feedback gain
```

**Forces:**
```
F_total = F_attractive + F_repulsive
        = pull_toward_goal + push_from_obstacles
```

**Output**: `(v, ω)` - Control commands with speed constraint from Layer 1

### **Coordinator: HybridCollisionAvoidance**

**Class**: `HybridCollisionAvoidance`

Orchestrates the two-layer pipeline:

1. **Layer 1 (ORCA)**: Plans safe velocity considering rover neighbors
2. **Layer 2 (APF)**: Refines direction and speed around static obstacles

```python
# Pipeline
v_safe_mag, _ = orca.plan(ego, goal, rover_neighbors, shape)
v_final, w_final = apf.compute(pos, heading, goal, obstacles,
                                v_max_constraint=v_safe_mag)
```

## 💭 Design Rationale

### **Why Hierarchical?**

1. **Safety First**: Rover collisions are catastrophic → handle first (ORCA)
2. **Soft Constraints**: Pebbles can be tolerated → handle second (APF)
3. **Clear Priority**: No ambiguity about what matters more

### **Why This Combination?**

| System | Strengths | Weaknesses |
|--------|-----------|-----------|
| **ORCA** | Multi-agent coordination, deadlock-free | Jerky paths, needs direction control |
| **APF** | Smooth paths, natural obstacle handling | No multi-agent awareness, can oscillate |
| **Hybrid** | ✅ Safe + Smooth + Multi-agent | Requires careful parameterization |

### **Natural Pushing Behavior**

When a pebble blocks the optimal path:

```
ORCA says: "Other rovers clear, so proceed"
APF says: "Pebble blocking, but it's soft"
Result: Rover slows, edges forward, and naturally pushes pebble
```

## 🎮 Usage Examples

### **Example 1: Basic Usage**
```bash
python hybrid_simulation.py
```
Runs default scenario with GUI and verbose output.

### **Example 2: Batch Testing**
```bash
for scenario in 2_head_on_pebbles 4_crossing_obstacles 2_corridor; do
    python hybrid_simulation.py --scenario $scenario --headless --quiet
done
```

### **Example 3: Extended Simulation**
```bash
python hybrid_simulation.py --scenario 3_convergence --max-time 60 --headless
```

## 📈 What to Expect

### **Typical Behavior**

**Head-On Scenario (2 rovers):**
- Robots detect each other (ORCA activates)
- Both sample alternative velocities
- They split to opposite sides
- Continue to goals with smooth APF guidance
- ~5-8 seconds total

**Corridor Scenario (2 rovers, dense pebbles):**
- Rovers navigate parallel to each other
- APF guides around pebbles
- Some pushing observed (expected)
- ~8-15 seconds total

**Maze Scenario (1 rover):**
- Pure APF navigation (no ORCA needed)
- Smooth weaving through obstacles
- Natural path following
- ~5-10 seconds total

## 🔧 Customization

### **Modify ORCA Parameters**

Edit `hybrid_simulation.py`:

```python
ORCA_PARAMS = {
    "tau": 8.0,              # Increase for more careful prediction
    "v_max": 1.0,            # Reduce for safety, increase for speed
    "w_max": 3.0,            # Rotation capability
}
```

### **Modify APF Parameters**

Edit `hybrid_simulation.py`:

```python
APF_PARAMS = {
    "k_rep": 0.08,           # Increase to push obstacles more
    "d0": 0.55,              # Increase repulsion range
    "k_att": 1.5,            # Increase goal pull strength
}
```

### **Create New Scenario**

Edit `scenarios.py` and add:

```python
def scenario_my_custom():
    return {
        "name": "My Custom Scenario",
        "description": "Test case description",
        "rovers": [
            {
                "start_pos": [x, y, heading],
                "start_heading": heading,
                "goal_pos": [gx, gy],
                "priority": 0,
                "color": (r, g, b, a),
            },
            # ... more rovers
        ],
        "pebbles": [
            (px1, py1),
            (px2, py2),
            # ... more pebbles
        ]
    }

# Register in SCENARIOS dict
SCENARIOS["my_scenario"] = scenario_my_custom
```

## 📊 Output

Each run produces:

```
==============================================================================
SIMULATION COMPLETE
==============================================================================
Simulated time: 8.45s
Real time: 2.33s
Speedup: 3.63x

Rovers Reached Goal: 2/2
  R0: 5.23s
  R1: 7.89s
==============================================================================
```

## 🐛 Troubleshooting

### **Rovers Not Moving**
- Check that PyBullet GUI window is focused
- Verify URDF files are in the same directory
- Check console output for error messages

### **Rovers Colliding**
- Increase ORCA `tau` parameter (longer prediction)
- Increase `v_max` safety margin
- Verify priorities are set correctly

### **Rovers Not Reaching Goals**
- Reduce APF `d0` (less pebble interference)
- Increase APF `k_att` (stronger goal pull)
- Check goal positions are reachable

### **Slow Simulation**
- Run with `--headless` flag
- Reduce `--max-time`
- Close other applications

## 📚 Further Reading

- See `ARCHITECTURE.md` for detailed design documentation
- See `QUICKSTART.md` for faster getting started
- See code comments in `hybrid_planner.py` for algorithm details

## 🎓 Learning Path

1. Run default scenario: `python hybrid_simulation.py`
2. List scenarios: `python hybrid_simulation.py --list-scenarios`
3. Try different scenarios: `python hybrid_simulation.py --scenario 2_corridor`
4. Modify parameters in `hybrid_simulation.py`
5. Create custom scenario in `scenarios.py`
6. Study algorithm in `hybrid_planner.py`

## 📝 Notes

- Rovers are differential-drive (unicycle model)
- Pebbles have 5cm radius by default
- Goal tolerance is 15cm
- Simulation runs at 240 Hz physics, 10 Hz planning
- ORCA and APF layers are independent and testable separately

## ✅ System Status

- ✅ ORCA layer fully implemented
- ✅ APF layer fully implemented
- ✅ Hybrid coordinator working
- ✅ PyBullet integration complete
- ✅ Multiple scenarios defined
- ✅ Differential drive control implemented
- ✅ Documentation complete

## 🚀 Future Enhancements

Potential improvements (not yet implemented):

- Real-time parameter tuning interface
- Visualization of potential field heatmap
- Trajectory recording and playback
- Statistics collection and analysis
- Formation control
- Cooperative pushing strategies
- Adaptive parameter tuning

---

**Ready to run!** Start with:

```bash
python hybrid_simulation.py
```
