# Runnable Simulation Examples

## How to select a demo

Open `RUN_DEMO_SCENARIOS.py`, change the single `DEMO_NAME` line near the top, save, and click **Play** on that file in AntiGravity. The normal scheduled-orchestrator settings are not changed.

```python
DEMO_NAME = "baseline_circle"
```

## Recommended live sequence

### 1. `baseline_circle` — reference behavior

Centered circular target, uniform pebbles, and three identical small rovers. Use this to establish the original problem before showing new capabilities.

### 2. `heterogeneous_circle` — rover specialization

The target remains simple, but the fleet becomes small-large-small and the material is mixed. Compare the type and size of tasks selected by each rover.

### 3. `concave_translated` — combined difficult scenario

A translated non-convex target creates asymmetric approaches, target-exit difficulty, and more opportunities for rover conflicts.

### 4–5. Congestion comparison

Run `congestion_existing` and `congestion_reassignment`. They use the same target, fleet, material, and random seed; only optional congestion-aware task reassignment changes.

## Target-shape demonstrations

| Demo name | Target | What to watch |
|---|---|---|
| `off_center_circle` | Translated circle | Unequal travel and utilization |
| `rectangle_target` | Rectangle | Flat-edge entry, corners, and exit |
| `ellipse_target` | Rotated ellipse | Directional approaches and edge spillage |
| `semicircle_target` | Half-disk | Flat-side versus curved-side behavior |
| `thin_l_target` | Thin L-shape | Inner-corner feasibility and remaining material |
| `concave_translated` | Thick translated L | Congestion, keepout, and target exit |
| `amorphous_target` | Irregular polygon | General non-convex geometry stress test |

## Logs and images

Event logging, benchmark telemetry, and standardized 2D images remain enabled. After stopping or completing a demo, use the newest `LATEST_RUN.txt` and the notebook analysis section to inspect tasks, pushes, environment snapshots, congestion, and completion status.

## Practical live-demo advice

Do not wait for every run to finish. Run each preset long enough to make its important behavior visible, and keep a completed run or saved images ready for the log-analysis portion.
