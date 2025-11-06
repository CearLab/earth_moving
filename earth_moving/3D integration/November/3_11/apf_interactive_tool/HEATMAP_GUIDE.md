# Potential Field Heatmap Guide

## Overview

Before the rover starts moving, you will see a **2D heatmap visualization** of the potential field. This is a powerful tool to understand the APF navigation before it happens!

## What is the Potential Field?

The potential field is a continuous 2D landscape where:
- **Low values** = good places for the rover to go (toward the goal, away from obstacles)
- **High values** = bad places for the rover to go (near obstacles, far from goal)

The rover follows the "valleys" of low potential down toward the goal, naturally avoiding obstacles!

## Understanding the Heatmap Colors

### Color Scale
- **Dark Blue/Navy** - Lowest potential (best paths)
- **Blue** - Very low potential (good alternative routes)
- **Cyan/Light Blue** - Low-moderate potential
- **Green** - Moderate potential
- **Yellow** - High potential (caution zones)
- **Orange/Red** - Very high potential (danger zones)

### What Each Color Means

| Color | Potential | Meaning |
|-------|-----------|---------|
| Dark Blue | Very Low | Optimal rover path - follow this! |
| Blue | Low | Alternative good paths |
| Green | Moderate | Acceptable but not ideal |
| Yellow | High | Repulsion zone - rover should avoid |
| Red | Very High | Obstacle or strong repulsion |

## Elements on the Heatmap

### 1. Heatmap Background
The colored field represents potential energy at each point.
- **Interpretation**: Where should the rover go?
- **Reading**: Follow dark blue valleys from start to goal

### 2. Red Circles (Obstacles)
Mark the exact positions of pebbles/obstacles.
- **Size**: Relative to actual obstacle size
- **Yellow Halo**: Shows repulsion influence zone
- **Key**: Wider halo = stronger repulsion

### 3. Lime Green Star (Goal)
The target position where the rover wants to go.
- **Location**: Shows exactly where goal is
- **Potential**: Lowest around this point (for attraction)
- **Key**: Potential increases with distance from goal

### 4. Cyan Square (Robot Start)
Starting position of the rover (usually at center).
- **Location**: Where rover begins
- **Potential**: Shows initial "difficulty" of the task
- **Key**: If on dark blue, easy start; if on yellow, many obstacles nearby

### 5. Black Dashed Circle (Boundary)
The environment boundary (2.0 meter radius).
- **Beyond this**: Rover cannot go outside
- **Purpose**: Defines simulation space
- **Importance**: Shows available navigation area

## How to Read the Heatmap

### Step 1: Find the Goal
Locate the lime green star (goal position).

### Step 2: Find the Start
Locate the cyan square (robot starting position).

### Step 3: Trace Dark Blue Path
Follow the darkest blue areas from start to goal.
- This is the optimal path according to APF
- The rover will likely follow this path

### Step 4: Check for Obstacles
Identify red circles and their yellow halos.
- Each red circle = obstacle
- Yellow halo = repulsion influence
- Narrow corridor = challenging but efficient path

### Step 5: Predict Navigation
Based on the path, ask yourself:
- Is there a clear path from start to goal?
- Does the path go around obstacles or through narrow passages?
- Does the path seem efficient (not zigzagging)?

## Interpreting the Aggressive Parameters

### k_rep = 0.05 (Very Low Repulsion)
**What you see in the heatmap:**
- Small yellow halos around obstacles
- Rover can get quite close to obstacles
- Narrow passages are possible
- More efficient paths through tight spaces

**Why this is good:**
- Tight navigation for efficiency
- Still maintains safety distance
- Reaches goals faster

### d0 = 0.45m (Short Influence Range)
**What you see in the heatmap:**
- Repulsion only affects nearby areas
- Distant obstacles have minimal influence
- Simpler overall potential field
- Fewer "peaks and valleys"

**Why this is good:**
- Local decision making
- Reduces complexity
- Faster path calculations

### k_att = 1.5 (Balanced Attraction)
**What you see in the heatmap:**
- Smooth gradient toward goal
- Gentle pull toward target
- Not too sharp of a "funnel" shape
- Balanced with repulsion forces

**Why this is good:**
- Stable navigation
- No oscillations
- Smooth trajectories

## Example Scenarios

### Scenario 1: Open Path
```
Heatmap: Clear dark blue corridor from start to goal
Interpretation: Easy navigation, straight path
Expected: Rover goes directly to goal in ~3 seconds
Potential: Mostly low with green star at goal
```

### Scenario 2: Narrow Passage
```
Heatmap: Thin dark blue "canyon" between obstacles
Interpretation: Must thread between obstacles
Expected: Rover carefully navigates through gap
Potential: Valleys on sides (yellow), safe corridor in middle (blue)
```

### Scenario 3: Multiple Obstacles
```
Heatmap: Several red circles with yellow halos
Interpretation: Must navigate around multiple barriers
Expected: Rover takes efficient slalom path
Potential: Complex field with many peaks and valleys
```

## What If the Heatmap Shows...

### ...Very Narrow Dark Blue Path?
**Meaning**: Tight navigation required
**Expected**: Rover will navigate carefully but efficiently
**Concern**: Might take longer (5-8 seconds instead of 3-5)

### ...No Clear Dark Blue Path?
**Meaning**: Goal unreachable or surrounded by obstacles
**Expected**: Rover may fail to reach goal
**Concern**: Increase obstacle spacing or change goal position

### ...Large Red/Yellow Area?
**Meaning**: Strong obstacle repulsion
**Expected**: Rover steers far around obstacle
**Concern**: If too strong, may increase navigation time

### ...Smooth Gradient?
**Meaning**: Few obstacles, clear path
**Expected**: Fast, efficient navigation
**Good Sign**: 3-4 second completion expected

## Using the Heatmap to Understand Navigation

### Question: Why did the rover take that path?
**Answer**: Look at the heatmap! The path follows low potential valleys.

### Question: Why did the rover hit that obstacle?
**Answer**: Check the heatmap. If the path shows yellow near obstacle, repulsion may be weak.

### Question: Why did it take so long?
**Answer**: Measure the dark blue path length. Longer path = longer navigation time.

### Question: Could it have gone faster?
**Answer**: Look for straighter dark blue corridors. If path has many turns, that's the physics at work.

## Tips for Using the Heatmap

1. **Close Inspection**: Right before pressing Enter, study the dark blue path carefully
2. **Obstacle Layout**: Count red circles to verify correct obstacle count
3. **Path Prediction**: Try to predict rover behavior before it starts
4. **Comparison**: Compare heatmaps for different obstacle seeds to see variation
5. **Parameter Understanding**: Watch how aggressive params create narrow but efficient paths

## Advanced: Reading the Mathematics

The heatmap shows:
```
V_total(x,y) = V_attractive(x,y) + V_repulsive(x,y)

V_attractive = 0.5 * k_att * distance_to_goal²
V_repulsive = 0.5 * k_rep * (1/distance_to_obstacle - 1/d0)²
```

**What this means:**
- Potential increases with distance from goal (parabolic)
- Potential increases near obstacles (repulsion)
- Total potential is sum of both
- Rover follows downhill gradient = optimal path!

## Common Questions

**Q: Why are some obstacles barely visible?**
A: Small repulsion zone (k_rep=0.05). This is intentional for aggressive navigation!

**Q: Why is the heatmap so complex?**
A: Multiple obstacles create multiple peaks/valleys in the field.

**Q: Can I trust the heatmap path?**
A: Yes! The rover will follow paths of low potential as shown in heatmap.

**Q: Why does the heatmap show such a narrow path?**
A: Aggressive parameters create efficient but tight corridors for fast navigation.

**Q: What if I disagree with the path?**
A: The APF has calculated the mathematically optimal path given the parameters!

## Summary

The potential field heatmap is your window into the APF decision-making process:

1. **Before Navigation**: Visualizes the forces guiding the rover
2. **Path Prediction**: Helps predict where rover will go
3. **Parameter Understanding**: Shows why aggressive params work
4. **Learning Tool**: Teaches APF navigation principles
5. **Validation**: Confirms the field looks reasonable before navigation

Use it to:
- Understand the navigation physics
- Predict and verify rover behavior
- Identify potential navigation challenges
- Appreciate the aggressive parameter design

The heatmap makes APF navigation transparent and understandable!

## See Also

- README.md - Complete documentation
- QUICKSTART.md - Quick start guide
- apf_interactive.py - Source code with heatmap function
