# Hybrid Collision Avoidance - Quick Start (5 Minutes)

## 🚀 Get Running in 30 Seconds

### **Step 1: Navigate to Directory**
```bash
cd "C:\Users\nirm\Desktop\Nir\Master Degree\Thesis\Code\earth_moving\earth_moving\3D integration\November\8_11\hybrid_collision_avoidance"
```

### **Step 2: Run Default Scenario**
```bash
python hybrid_simulation.py
```

### **Step 3: Watch the Simulation**
- PyBullet window opens showing rovers and pebbles
- Rovers navigate while avoiding each other and obstacles
- Console prints status updates
- When done, results print to console

**That's it!** You're now running the hybrid collision avoidance system.

---

## 📋 What Just Happened?

The system ran the **"2 Rovers Head-On"** scenario:

```
Rover R0 (Blue)                    Rover R1 (Orange)
  ↓                                    ↑
  [start at -4, 0]        obstacles   [start at +4, 0]

  Pebbles at: (-0.5, ±0.3), (0.5, ±0.3)

  Goals: R0 → (4, 0), R1 → (-4, 0)
```

**What happened:**
1. **Layer 1 (ORCA)**: Both rovers detected each other
2. Both sampled velocities to avoid collision
3. They split to opposite sides (one up, one down)
4. **Layer 2 (APF)**: Smooth guidance around pebbles
5. Both navigated to goals successfully

---

## 🎮 Try Different Scenarios

### **List Available Scenarios**
```bash
python hybrid_simulation.py --list-scenarios
```

Output:
```
Available Scenarios:
======================================================================
  2_head_on_pebbles         - Two robots approaching each other with obstacles in between
  4_crossing_obstacles      - Four robots crossing at center, navigating around pebbles
  2_sparse                  - Two robots with sparse pebbles, smooth navigation expected
  2_corridor                - Two robots navigating through narrow pebble corridor
  3_convergence             - Three robots converging to center, avoiding pebbles and each other
  1_maze                    - Single robot navigating complex pebble arrangement
======================================================================
```

### **Run Specific Scenario**
```bash
python hybrid_simulation.py --scenario 2_corridor
```

Other examples:
```bash
python hybrid_simulation.py --scenario 4_crossing_obstacles
python hybrid_simulation.py --scenario 3_convergence
python hybrid_simulation.py --scenario 1_maze
```

---

## ⚙️ Common Options

### **Headless Mode (No GUI)**
```bash
python hybrid_simulation.py --headless
```
Useful for batch testing or when you don't need visualization.

### **Quiet Mode (No Console Output)**
```bash
python hybrid_simulation.py --quiet
```
Suppresses status messages, just show final results.

### **Set Maximum Time**
```bash
python hybrid_simulation.py --max-time 60
```
Runs simulation for maximum 60 seconds (default is 30).

### **Combine Options**
```bash
python hybrid_simulation.py --scenario 4_crossing_obstacles --headless --max-time 45
```

---

## 🔍 Understanding the Output

### **During Simulation**
```
[1.23s] R0 v,ω=(0.82,0.25) | R1 v,ω=(0.75,-0.18)
[2.45s] R0 v,ω=(0.89,0.05) | R1 v,ω=(0.88,-0.02)
[5.23s] R0 reached goal!
[7.89s] R1 reached goal!
```

- **Time**: Elapsed simulation time
- **v**: Forward velocity (m/s)
- **ω**: Angular velocity (rad/s)
- **Message**: When rover reaches goal

### **Final Results**
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

- **Simulated time**: How long in virtual time
- **Real time**: How long it took on your computer
- **Speedup**: How much faster than real-time
- **Goals reached**: How many rovers reached destinations
- **Completion times**: When each rover reached goal

---

## 💡 What Each Scenario Tests

### **2 Rovers Head-On (Default)**
✅ Basic multi-rover coordination
✅ ORCA avoidance priority
✅ APF obstacle guidance
**Speed**: Fast (5-8s)

### **4 Robots Crossing**
✅ Complex multi-agent scenarios
✅ Negotiated avoidance
✅ Pebble avoidance with multiple agents
**Speed**: Medium (8-15s)

### **2 Rovers Sparse**
✅ Minimal obstacle interference
✅ Smooth navigation
✅ Fast completion expected
**Speed**: Very Fast (3-5s)

### **2 Rovers Corridor** ⭐
✅ Dense obstacle field
✅ **Pushing behavior** (rovers push pebbles!)
✅ Tight navigation
**Speed**: Slow (10-20s)

### **3 Rovers Convergence**
✅ Three-agent coordination
✅ Triangular approach
✅ Symmetry breaking
**Speed**: Medium (8-12s)

### **Single Rover Maze**
✅ Pure APF testing (no ORCA)
✅ Complex obstacle weaving
✅ Single-agent navigation
**Speed**: Medium (5-10s)

---

## 🎯 Expected Results

### **Good Simulation Results**
- ✅ All rovers reach goals
- ✅ Completion time 5-15 seconds
- ✅ No collisions between rovers
- ✅ Some gentle pushing of pebbles (okay!)
- ✅ Smooth navigation paths

### **Problems to Watch For**
- ❌ Rovers colliding with each other (bad!)
- ❌ Rovers stuck on pebbles (maybe adjust parameters)
- ❌ Very slow completion (>20s) - might indicate parameter issues
- ⚠️ Rovers pushed many pebbles - APF k_rep might be too low

---

## 🔧 Quick Parameter Tuning

### **Rovers Colliding?**
Edit `hybrid_simulation.py`, increase ORCA safety:

```python
ORCA_PARAMS = {
    "tau": 12.0,        # Increase from 8.0 (longer prediction)
    "v_max": 0.8,       # Reduce from 1.0 (slower speed)
}
```

### **Too Much Pebble Pushing?**
Edit `hybrid_simulation.py`, increase APF repulsion:

```python
APF_PARAMS = {
    "k_rep": 0.15,      # Increase from 0.08 (stronger repulsion)
    "d0": 0.70,         # Increase from 0.55 (wider influence)
}
```

### **Slow Navigation?**
Edit `hybrid_simulation.py`, increase speeds:

```python
ORCA_PARAMS = {
    "v_max": 1.2,       # Increase from 1.0
}
APF_PARAMS = {
    "v_max": 1.5,       # Increase from 1.2
}
```

---

## 📊 Running Multiple Tests

### **Test All Scenarios (Batch)**
```bash
python hybrid_simulation.py --scenario 2_head_on_pebbles --headless --quiet
python hybrid_simulation.py --scenario 4_crossing_obstacles --headless --quiet
python hybrid_simulation.py --scenario 2_corridor --headless --quiet
python hybrid_simulation.py --scenario 1_maze --headless --quiet
```

### **Batch Script (Linux/Mac)**
```bash
#!/bin/bash
for scenario in 2_head_on_pebbles 4_crossing_obstacles 2_sparse 2_corridor 3_convergence 1_maze; do
    echo "Testing: $scenario"
    python hybrid_simulation.py --scenario $scenario --headless --quiet
done
```

### **Batch Script (Windows)**
```powershell
$scenarios = @("2_head_on_pebbles", "4_crossing_obstacles", "2_sparse", "2_corridor", "3_convergence", "1_maze")
foreach ($scenario in $scenarios) {
    Write-Host "Testing: $scenario"
    python hybrid_simulation.py --scenario $scenario --headless --quiet
}
```

---

## 🎓 Next Steps

### **Learn the System**
1. ✅ Run default scenario
2. ✅ Try different scenarios
3. 📖 Read `README.md` for full documentation
4. 📖 Read `ARCHITECTURE.md` for design details
5. 💻 Study code in `hybrid_planner.py`

### **Customize**
1. Create new scenario in `scenarios.py`
2. Adjust parameters in `hybrid_simulation.py`
3. Modify ORCA/APF parameters
4. Test custom scenarios

### **Integrate**
1. Import `HybridCollisionAvoidance` into your code
2. Use in your own PyBullet simulations
3. Extend with additional features
4. Adapt for your specific use case

---

## 🆘 Troubleshooting

### **"No module named 'pybullet'"**
```bash
pip install pybullet numpy
```

### **"Cannot load URDF"**
Make sure you're in the correct directory:
```bash
cd hybrid_collision_avoidance
```

### **PyBullet window won't open**
Try headless mode instead:
```bash
python hybrid_simulation.py --headless
```

### **Simulation very slow**
- Close other applications
- Use `--headless` for faster execution
- Reduce `--max-time` for testing

---

## 📚 Quick Reference

### **Most Common Commands**
```bash
# Run default scenario with GUI
python hybrid_simulation.py

# List all scenarios
python hybrid_simulation.py --list-scenarios

# Run specific scenario
python hybrid_simulation.py --scenario 2_corridor

# Batch testing (no GUI)
python hybrid_simulation.py --scenario 4_crossing_obstacles --headless

# Extended simulation
python hybrid_simulation.py --max-time 120
```

---

## ✨ What Makes This Special

- ✅ **Two-layer hierarchy**: Rovers first, then pebbles
- ✅ **ORCA coordination**: Multi-agent deadlock-free negotiation
- ✅ **APF guidance**: Smooth obstacle avoidance
- ✅ **Natural pushing**: Rovers can push pebbles when needed
- ✅ **Easy to customize**: Modify parameters or create scenarios
- ✅ **Well documented**: Guides and code comments

---

## 🚀 You're Ready!

Start with:
```bash
python hybrid_simulation.py
```

Then explore different scenarios:
```bash
python hybrid_simulation.py --list-scenarios
```

Enjoy the hybrid collision avoidance system! 🤖
