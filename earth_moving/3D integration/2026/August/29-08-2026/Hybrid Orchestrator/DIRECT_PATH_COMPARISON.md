# Direct-Path Proof-of-Concept Comparison

## Purpose

Compare a simple straight, exposed-source baseline with progressively more flexible material-aware direct paths. All variants use one rover, target tasks only, the same physics, and no highway tasks.

## How to run

Open `RUN_DIRECT_PATH_COMPARISON.py`. Change only these two lines, save, and click **Play**:

```python
COMPARISON_NAME = "P0_straight_root"
TEST_SCENARIO = "baseline_circle_uniform"
```

For one automatically terminating run from a terminal:

```powershell
python RUN_DIRECT_PATH_COMPARISON.py P0_straight_root --scenario baseline_circle_uniform --auto-exit-on-completion --max-sim-time 7200 --max-no-delivery-time 1200
```

To run every preset in order, open `RUN_ALL_DIRECT_PATH_COMPARISONS.py` in
Antigravity and click Play. Its easy-edit settings control the preset list,
scenario, seed, pebble count, and three safeguards. No terminal is required.

For a validation-only Play run, temporarily set:

```python
VALIDATE_ONLY = True
```

It checks the real output directories, legacy Windows path budget, and write
access without opening PyBullet. After it prints `preflight: PASS`, set the
value back to `False` and click Play for the complete batch. The full run also
performs this preflight automatically before starting P0.

## Comparison ladder

| Preset | Path | Sources | Allocation value | Question |
|---|---|---|---|---|
| `P0_straight_root` | Straight to nearest boundary | Convex hull | Selected source only | Simple Shaked-inspired reference |
| `P1_narrow_root_5deg` | Normal planner, 5° cone | Convex hull | Selected source only | Does the narrow-cone approximation match the explicit straight mode? |
| `P2_wider_root_source_only` | Normal planner, 30° cone | Convex hull | Selected source only | Does path freedom alone help? |
| `P3_wider_root_material_aware` | Normal planner, 30° cone | Convex hull | Full swept corridor | Does deliberate multi-object value help? |
| `P4_expanded_direct` | Normal planner, 30° cone | All feasible | Full swept corridor | Does broader source eligibility add value? |
| `P5_expanded_direct_60deg` | Normal planner, 60° cone | All feasible | Full swept corridor | When does still wider visibility help or hurt? |

## Controlled conditions

- One small rover.
- Target tasks only.
- Highway tasks disabled.
- 60 uniform small pebbles.
- Count-based material map.
- Random seed 41.
- Same execution, replanning, logging, and physical spillage model.

## Initial scenarios

- `baseline_circle_uniform`
- `rectangle_target`
- `amorphous_target`

Run every preset on the same scenario before changing the scenario. This preserves identical initial material positions across methods.

## Logged comparison identity

Every run records the comparison label, path mode, candidate-value mode,
source-selection mode, root and convex-hull status of an accepted source,
target visibility, policy weights, scenario, and seed in `RUN_CONFIG` and task
events. The live convex hull is recomputed at every shared-map rebuild from all
outside occupied cells plus the exact target-zone boundary. A cell qualifies
when its grid footprint intersects that combined hull boundary.

## Batch completion and outputs

A successful automated child run ends only when both conditions hold:

- every live pebble is inside the target zone;
- the rover has completed its target exit and reached `PARKED`.

Each comparison writes to a compact physical directory under
`batch_logs/b<date>_<time>/p0` through `p5`. The short names are intentional:
the thesis directory is already long, and readable nested comparison names
caused Windows to reject the telemetry filenames. The manifest retains the
complete comparison names and maps each one to its physical log directory.

The batch root contains `batch_manifest.csv` and `batch_manifest.json`, updated
after every run. They record completion/stall/timeout/error status, wall and
simulated duration, final delivery, push count, efficiency, spillage, and paths
to the JSONL, CSV, JSON, snapshot, and image outputs. The most recent batch is
also recorded in `simulation_logs/LATEST_COMPARISON_BATCH.txt`.

Before accepting a child result, the supervisor now requires a readable run
summary plus the event JSONL, task CSV, push CSV, snapshot CSV, and artifact
directory. By default it stops immediately on an infrastructure/logging failure
instead of spending the night running comparisons that cannot be analyzed.

The adjustable safeguards are maximum simulated time, maximum simulated time
without a delivery increase, and a supervisor wall-time limit. A failed or
stalled child is recorded and the next preset runs unless `--stop-on-error` is
used.

## First metrics to inspect

- Delivered material and completion time.
- Number of pushes.
- Delivered and retained material per push.
- Pushing and repositioning distance.
- Lateral or target-exit spillage.
- Actual corridor collection versus material used for task scoring.
- Planner calculation time.

## Interpretation

`P0` is a proof-of-concept baseline inspired by straight convex-hull pushing;
it is not claimed as an exact reproduction of Shaked et al. The historical
`*_root*` preset names are retained so existing commands and logs remain
comparable, but `P0`-`P3` now use the explicit geometric `convex_hull` source
mode. The legacy graph-based `root` mode remains available through
`--target-source-mode root` for separate ablations.
