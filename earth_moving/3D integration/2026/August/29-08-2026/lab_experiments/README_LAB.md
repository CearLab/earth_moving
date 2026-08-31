# Single-Rover Lab Experiment Package

## Purpose

This package compares a simple straight, convex-hull-source proof of concept
with progressively wider and material-aware direct-target planning. Highway
tasks are disabled. This is not presented as an exact reproduction of Tom
Shaked's method; it is a controlled reference that isolates the benefit of
visibility, source eligibility, swept-path material value, target geometry,
target position, and rover size.

## Core experiment

The immutable core plan contains **1,120 simulations**:

- 14 planner variants: straight baseline, narrow 5°, and the full
  30°/45°/60° × convex-hull/all-sources × source-only/material-aware matrix.
- 8 targets: centered and off-center circle, rectangle, amorphous, and L shape.
- 2 rover profiles: small and large shovel rover.
- 5 paired random seeds.

Two non-duplicating robustness stages extend this to **2,400 simulations**:

- 960 deployment runs: four representative planners with clustered,
  near-target, and outer-ring material across all 8 targets, both rovers, and
  5 seeds. Uniform-area cases are reused from the core rather than repeated.
- 320 density runs: the same four planners at 30 and 90 pebbles for the four
  centered target shapes, both rovers, and 5 seeds. The 60-pebble cases are
  reused from the core.

Every planner receives the same initial pebble state within a target/seed pair.
The four target geometries are scaled to approximately the same area as the
0.8 m-radius circle, so the shape comparison is not mainly a target-size test.
The initial state is recorded and SHA-256 fingerprinted. No nominal capacity
floor, cap, utilization bonus, or over-capacity rejection is used in this
single-rover stage (`uncapped_corridor`). Physical shovel/rover differences
remain active. The old profile-cap behavior is still available as an explicit
configuration option for a later capacity-policy study.

## Runtime modes

- `gui`: current interactive PyBullet and Pygame behavior.
- `headless`: no windows, but paced in real time for parity diagnostics.
- `headless_fast`: no windows and no real-time sleep. It preserves 240 Hz
  PyBullet stepping and 20 Hz controller updates, uses simulated time for map
  refreshes, and runs single-rover allocation/map planning synchronously.

The scientific plan uses `headless_fast`. Planning computation time is still
measured as wall time; it is not mixed into simulated mission duration.

## Validation before the full run

On this PC, open `VALIDATE_LOCAL.py` and click Play. It performs:

1. Python compilation and all simulation unit tests.
2. Full-plan structure and paired-design validation.
3. Seven short headless runs spanning shapes, rover sizes, planner families,
   and all four deployment distributions.
4. A second identical launch to verify resume/skip behavior.
5. Strict log validation and normalized analysis generation.
6. Paced-headless versus accelerated-headless initial-state parity.

Then open `BENCHMARK_LOCAL_WORKERS.py` and click Play. It measures 1, 2, and 4
workers using short runs and records total peak resident memory. This is a
throughput check, not a scientific result.

Short validation runs are expected to end at their small simulated-time limit;
the pass criterion is correct deterministic initialization, termination, and
complete readable telemetry. Scientific completion still requires all pebbles
inside the target and the rover parked.

## Lab server preparation

Copy the complete `29-08-2026` folder to the Linux server, preserving the
`Hybrid Orchestrator`, `Path Tracking`, and `lab_experiments` sibling layout.

```bash
cd 29-08-2026/lab_experiments
conda env create -f environment.yml
conda activate earth_moving_lab
python CHECK_SERVER_ENVIRONMENT.py
python validate_results.py --structure-only
```

First run a calibration spread rather than the complete matrix:

```bash
python run_experiment_suite.py --mode calibration --workers 4
```

This calibration contains 32 full-size runs: the straight baseline and the 45°
all-source material-aware method across every target, both rover sizes, and the
first seed.

Inspect failure rate, wall time, RAM, CPU utilization, and output growth. Then
benchmark 2/4/6/8 workers on that host and choose the fastest stable count. With
64 GB RAM and 8 available threads, 4 workers remains the conservative starting
point.

Launch the complete resumable plan with:

```bash
python run_experiment_suite.py --mode full --workers 4
```

To run only the 1,120-row principal comparison first, use `--mode core`. The
later `--mode full` launch will skip those valid core rows and continue with the
robustness stages.

The supervisor skips already valid terminal runs after interruption. It retries
only infrastructure failures, writes the manifest atomically after every run,
sets BLAS libraries to one thread per worker, and terminates children that exceed
the wall-time safeguard.

## Outputs

Each compact run directory `runs/r000001` contains immutable raw telemetry:

- event JSONL and task CSV;
- physical per-push attribution CSV;
- environment snapshot CSV;
- compressed full heatmap/cell/pebble snapshots;
- milestone images only (initial, delivery milestones, final/parked/failure);
- run summary, console log, and self-describing `status.json`.

Environment snapshots include highway connectivity/coherence, heat totals and
normalized heatmap change, delivery state, target-distance distribution,
outside-material centroid/spread, exact pebble states, and active rover paths.

After any partial or complete batch, run:

```bash
python analyze_experiment.py
```

The `analysis` folder then contains:

- `run_summary.csv`: one easy-to-filter row per simulation;
- `planner_target_rover_summary.csv`: means, standard deviations, and 95% CI;
- `paired_comparisons.csv`: same-seed differences from the straight baseline;
- `pushes.csv`, `tasks.csv`, and `environment_snapshots.csv`: normalized tables
  with experiment factors repeated on every row;
- `REPORT.md`: compact completion/status overview.

Raw files are never modified by analysis, so new thesis metrics can be derived
later without rerunning physics.

## Editing the plan

All factors and safeguards live in `experiment_config.yaml`. After an edit, run
`generate_experiment_plan.py` and inspect `experiment_plan.csv`. The generator
saves the exact configuration snapshot and hash next to the plan. Do not change
the configuration in the middle of a scientific batch; use a new
`experiment_name` for a revised experiment. The supervisor refuses to mix a
changed configuration into an experiment that already contains run statuses.
