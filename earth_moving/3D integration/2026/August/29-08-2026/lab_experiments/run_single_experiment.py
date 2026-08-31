"""Execute one immutable plan row and write a self-describing run status."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

from experiment_utils import (
    DEFAULT_CONFIG, atomic_write_json, config_hash, load_config, read_csv,
    resolve_from_config, stable_json_hash, validate_run_outputs,
)


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _selected_row(plan_path: Path, run_id: str) -> dict:
    for row in read_csv(plan_path):
        if row["run_id"] == run_id:
            return row
    raise ValueError(f"run ID {run_id!r} is not present in {plan_path}")


def _git_commit(directory: Path):
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=str(directory),
            text=True, stderr=subprocess.DEVNULL, timeout=10,
        ).strip()
    except Exception:
        return None


def _initial_state_hash(jsonl_path: str | Path):
    try:
        with Path(jsonl_path).open("r", encoding="utf-8") as stream:
            for line in stream:
                record = json.loads(line)
                if record.get("event") == "RUN_CONFIG":
                    states = record.get("initial_pebble_states")
                    return stable_json_hash(states) if states is not None else None
    except (OSError, ValueError):
        return None
    return None


def _status_name(outcome, valid_outputs, remaining):
    if not valid_outputs:
        return "logging_failed"
    if outcome == "mission_complete" and int(remaining or 0) == 0:
        return "completed"
    text = str(outcome or "unknown").lower()
    if "stalled" in text or "no_delivery" in text:
        return "mission_stalled"
    if "max_sim" in text or "timeout" in text or "time_limit" in text:
        return "mission_timeout"
    return "simulation_stopped"


def run(args):
    config, config_path = load_config(args.config)
    fixed = config["fixed_conditions"]
    plan_path = Path(args.plan).resolve()
    row = _selected_row(plan_path, args.run_id)
    results_dir = Path(args.results_dir).resolve()
    run_dir = results_dir / "runs" / row["run_id"]
    run_dir.mkdir(parents=True, exist_ok=True)
    status_path = run_dir / "status.json"

    threads = str(config.get("runner", {}).get("blas_threads_per_worker", 1))
    for name in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = threads
    os.environ.setdefault("PYTHONHASHSEED", "0")

    runtime_mode = args.runtime_mode or fixed["runtime_mode"]
    pebble_count = int(args.pebbles if args.pebbles is not None else row["pebble_count"])
    max_sim_time = float(args.max_sim_time if args.max_sim_time is not None else fixed["max_sim_time_s"])
    max_no_delivery = float(
        args.max_no_delivery_time
        if args.max_no_delivery_time is not None else fixed["max_no_delivery_time_s"]
    )
    metadata = {
        "schema": 1,
        "run_id": row["run_id"],
        "state": "running",
        "started_utc": _utc_now(),
        "plan": row,
        "effective": {
            "runtime_mode": runtime_mode,
            "pebble_count": pebble_count,
            "max_sim_time_s": max_sim_time,
            "max_no_delivery_time_s": max_no_delivery,
            "capacity_scoring_mode": fixed["capacity_scoring_mode"],
        },
        "reproducibility": {
            "configuration_path": str(config_path),
            "configuration_sha256": config_hash(config),
            "plan_path": str(plan_path),
            "python": sys.version,
            "platform": platform.platform(),
            "hostname": platform.node(),
        },
    }
    atomic_write_json(status_path, metadata)
    started = time.perf_counter()

    simulation_dir = resolve_from_config(config_path, config["simulation_dir"])
    sys.path.insert(0, str(simulation_dir))
    try:
        import orchestrator_hybrid_multi_astar_scheduled as scheduled
        from RUN_DIRECT_PATH_COMPARISON import MATERIAL_AWARE_POLICY, SOURCE_ONLY_POLICY

        target_zone = scheduled.get_scenario(row["scenario"]).target_zone.to_dict()
        rover = scheduled.ROVER_PROFILES[row["rover_profile"]]
        metadata["experimental_factors"] = {
            "target_zone": target_zone,
            "rover": {
                "name": rover.name,
                "shovel_width_m": float(rover.shovel_width),
                "overlay_cell_size_m": float(rover.overlay_cell_size),
                "collision_radius_m": float(rover.collision_radius),
                "reservation_radius_m": float(rover.reservation_radius),
                "nominal_capacity_objects": int(rover.capacity_objects),
                "nominal_capacity_mass": int(rover.capacity_mass),
            },
        }
        atomic_write_json(status_path, metadata)

        value_mode = row["candidate_value_mode"]
        policy = MATERIAL_AWARE_POLICY if value_mode == "material_aware" else SOURCE_ONLY_POLICY
        policy = deepcopy(policy)
        # This single-rover stage studies geometry and swept material without a
        # policy-imposed nominal load floor/cap. Physical shovel differences
        # remain in PyBullet and the rover-specific overlay geometry.
        policy.update({
            "capacity_utilization_weight": 0.0,
            "minimum_capacity_utilization": 0.0,
            "preferred_capacity_min_fraction": 0.0,
            "preferred_capacity_max_fraction": 1.0,
            "overcapacity_behavior": "allow",
            "max_overcapacity_fraction": None,
            "capacity_fit_before_task_fallback": False,
        })
        scheduled.TARGET_ZONE_CENTER = None
        scheduled.TARGET_VISIBILITY_MODE = "fixed"
        scheduled.TARGET_VISIBILITY_ANGLE = float(row["visibility_angle_deg"])
        scheduled.MAX_PATH_LENGTH_FACTOR = float(row["max_path_length_factor"])
        scheduled.DIRECT_TARGET_PATH_MODE = row["path_mode"]
        scheduled.TARGET_CANDIDATE_VALUE_MODE = value_mode
        scheduled.TARGET_SOURCE_MODE = row["source_mode"]
        scheduled.COMPARISON_LABEL = row["planner_id"]
        scheduled.ROVER_POLICY_OVERRIDES = {
            row["rover_profile"]: policy,
        }

        orchestrator_args = [
            str(Path(scheduled.__file__).resolve()),
            "--scenario", row["scenario"],
            "--rovers", "1",
            "--pebbles", str(pebble_count),
            "--seed", str(int(row["seed"])),
            "--pebble-deployment", row["deployment"],
            "--material-mode", row["material_mode"],
            "--uniform-small-pebbles",
            "--rover-profiles", row["rover_profile"],
            "--target-path-mode", row["path_mode"],
            "--target-candidate-value-mode", value_mode,
            "--target-source-mode", row["source_mode"],
            "--capacity-scoring-mode", fixed["capacity_scoring_mode"],
            "--comparison-label", row["planner_id"],
            "--runtime-mode", runtime_mode,
            "--map-interval", str(float(fixed["map_update_interval_s"])),
            "--event-log-dir", str(run_dir),
            "--auto-exit-on-completion",
            "--max-sim-time", str(max_sim_time),
            "--max-no-delivery-time", str(max_no_delivery),
            "--no-congestion-reassignment",
            "--flow-field-vis", "never",
            "--no-draw-conflicts",
            "--hide-planning-boundaries",
            "--no-draw-execution-paths",
            "--no-benchmark-push-images",
        ]
        old_argv = sys.argv
        try:
            sys.argv = orchestrator_args
            scheduled.main()
        finally:
            sys.argv = old_argv

        valid, missing, latest, summary = validate_run_outputs(run_dir)
        progress = summary.get("final_material_progress", {}) or {}
        outcome = summary.get("outcome")
        metadata.update({
            "state": _status_name(outcome, valid, progress.get("remaining_count")),
            "finished_utc": _utc_now(),
            "wall_duration_s": round(time.perf_counter() - started, 6),
            "valid_outputs": valid,
            "missing_outputs": missing,
            "outcome": outcome,
            "end_sim_time_s": summary.get("end_sim_time"),
            "final_material_progress": progress,
            "push_count": summary.get("push_count"),
            "delivery_efficiency_count": summary.get("aggregate_delivery_efficiency_count"),
            "lateral_spillage_rate_count": summary.get("aggregate_lateral_spillage_rate_count"),
            "outputs": latest,
            "initial_state_sha256": _initial_state_hash(latest.get("JSONL", "")),
        })
        metadata["reproducibility"]["git_commit"] = _git_commit(simulation_dir)
        atomic_write_json(status_path, metadata)
        print(
            f"[LAB] {row['run_id']} state={metadata['state']} "
            f"outcome={outcome} valid_outputs={valid}"
        )
        return 0 if valid else 2
    except Exception as exc:
        metadata.update({
            "state": "process_failed",
            "finished_utc": _utc_now(),
            "wall_duration_s": round(time.perf_counter() - started, 6),
            "valid_outputs": False,
            "error": repr(exc),
            "traceback": traceback.format_exc(),
        })
        atomic_write_json(status_path, metadata)
        traceback.print_exc()
        return 3


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--plan", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--results-dir", required=True)
    parser.add_argument("--runtime-mode", choices=("gui", "headless", "headless_fast"), default=None)
    parser.add_argument("--pebbles", type=int, default=None)
    parser.add_argument("--max-sim-time", type=float, default=None)
    parser.add_argument("--max-no-delivery-time", type=float, default=None)
    args = parser.parse_args()
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
