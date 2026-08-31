# Local Validation Report — 2026-08-29

## Outcome

The pre-server validation passed on the Windows development PC with Python
3.10.17. The full scientific plan was not launched locally.

## Checks completed

- 56 simulation tests passed.
- 9 lab-runner and experiment-definition tests passed.
- The generated staged plan contains exactly 2,400 unique runs: a 1,120-run
  principal matrix, 960 non-duplicating deployment-robustness runs, and 320
  non-duplicating density-robustness runs.
- Seven real PyBullet headless runs covered centered/off-center targets,
  small/large rovers, hull/all sources, source/material-aware scoring, and
  uniform/clustered/near-target/outer-ring deployments. All produced complete
  readable output sets.
- Re-running the same seven-run suite launched zero children, confirming resume
  and skip behavior.
- Paced headless and accelerated headless runs produced the same SHA-256 initial
  pebble-state fingerprint.
- A large-rover 45° material-aware smoke run confirmed
  `CapacityScoring=uncapped_corridor`, zero utilization weight, zero minimum
  utilization, and permissive over-capacity behavior.
- A one-pebble completion test ended only after delivery and parking:
  `mission_complete`, 1 delivered, 0 remaining, 1 push, 16.80 simulated seconds,
  3.73 total wall seconds including initialization/log images.
- Normalized run, task, push, and environment tables were generated successfully.

## Local worker smoke benchmark

These numbers use seven very short validation runs and are only for selecting a
starting worker count; they do not estimate the final scientific runtime.

| Workers | Wall time | Approx. runs/hour | Peak total RSS |
|---:|---:|---:|---:|
| 1 | 24.68 s | 1,021 | 0.360 GiB |
| 2 | 16.58 s | 1,520 | 0.491 GiB |
| 4 | 10.51 s | 2,397 | 0.926 GiB |

Four workers gave the best tested local throughput. The lab server should still
run its own 2/4/6/8-worker calibration because full 60-pebble missions have a
different planning, physics, telemetry, and memory profile.

## Server launch gate

Before the full plan, the server should pass `CHECK_SERVER_ENVIRONMENT.py`, plan
structure validation, and a calibration batch. The 1,120-run core matrix should
start only after checking mission completion rate, timeout/stall causes, peak
memory, storage growth, and stable throughput on that server. The 1,280
robustness runs can then resume into the same result set to complete all 2,400
runs.
