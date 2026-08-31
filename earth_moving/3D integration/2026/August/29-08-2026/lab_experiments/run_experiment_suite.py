"""Resumable, bounded-parallel supervisor for local and Linux lab runs."""

from __future__ import annotations

import argparse
import concurrent.futures as cf
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from experiment_utils import (
    DEFAULT_CONFIG, atomic_write_csv, atomic_write_json, config_hash, load_config, read_csv,
    read_json, resolve_from_config,
)
from generate_experiment_plan import generate


# Change this line and click Play in Antigravity.  Use "validation" here; use
# "full" only on the lab server after the validation report passes.
RUN_MODE = "validation"  # plan_only / validation / calibration / full
WORKERS = 4

MANIFEST_FIELDS = (
    "run_id", "stage", "planner_id", "target_id", "deployment",
    "pebble_count", "rover_profile", "seed",
    "state", "outcome", "valid_outputs", "wall_duration_s", "end_sim_time_s",
    "delivered_count", "remaining_count", "push_count",
    "delivery_efficiency_count", "lateral_spillage_rate_count",
    "initial_state_sha256", "attempt", "run_dir", "error",
)


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _select_rows(config, rows, mode, limit=None):
    if mode == "full":
        selected = list(rows)
    elif mode == "core":
        selected = [row for row in rows if row["stage"] == "core"]
    elif mode == "calibration":
        # A spread across shapes, rovers, seeds and planner families; enough to
        # estimate runtime and failure rates before committing the full matrix.
        planner_ids = {
            "B00_straight_hull_source", "A45_all_material",
        }
        selected = [
            row for row in rows
            if row["stage"] == "core"
            and row["planner_id"] in planner_ids
            and int(row["seed"]) == int(config["seeds"][0])
        ]
    elif mode == "validation":
        wanted = {
            (
                item["stage"], item["planner"], item["target"],
                item["deployment"], item["rover"], str(item["seed"]),
            )
            for item in config["local_validation"]["cases"]
        }
        selected = [
            row for row in rows
            if (
                row["stage"], row["planner_id"], row["target_id"],
                row["deployment"], row["rover_profile"],
                str(row["seed"]),
            ) in wanted
        ]
        if len(selected) != len(wanted):
            raise RuntimeError(
                f"local validation requested {len(wanted)} cases but found {len(selected)}"
            )
    else:
        selected = []
    return selected[:limit] if limit else selected


def _status_row(row, run_dir, attempt=0, fallback=None):
    status = read_json(run_dir / "status.json", default={}) or {}
    progress = status.get("final_material_progress", {}) or {}
    return {
        **row,
        "state": status.get("state", fallback or "pending"),
        "outcome": status.get("outcome"),
        "valid_outputs": status.get("valid_outputs", False),
        "wall_duration_s": status.get("wall_duration_s"),
        "end_sim_time_s": status.get("end_sim_time_s"),
        "delivered_count": progress.get("delivered_count"),
        "remaining_count": progress.get("remaining_count"),
        "push_count": status.get("push_count"),
        "delivery_efficiency_count": status.get("delivery_efficiency_count"),
        "lateral_spillage_rate_count": status.get("lateral_spillage_rate_count"),
        "initial_state_sha256": status.get("initial_state_sha256"),
        "attempt": attempt,
        "run_dir": str(run_dir),
        "error": status.get("error"),
    }


def _terminal_valid(run_dir, expected_config_hash):
    status = read_json(run_dir / "status.json", default={}) or {}
    recorded_hash = (status.get("reproducibility") or {}).get("configuration_sha256")
    return recorded_hash == expected_config_hash and bool(status.get("valid_outputs")) and status.get("state") in {
        "completed", "mission_stalled", "mission_timeout", "simulation_stopped",
    }


def _child_command(args, config, config_path, plan_path, results_dir, row):
    command = [
        sys.executable, str(Path(__file__).resolve().parent / "run_single_experiment.py"),
        "--config", str(config_path), "--plan", str(plan_path),
        "--run-id", row["run_id"], "--results-dir", str(results_dir),
    ]
    if args.mode == "validation":
        local = config["local_validation"]
        command += [
            "--pebbles", str(local["pebble_count"]),
            "--max-sim-time", str(local["max_sim_time_s"]),
            "--max-no-delivery-time", str(local["max_no_delivery_time_s"]),
        ]
    return command


def _run_child(command, run_dir, wall_limit, stop_event):
    run_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = run_dir / "console.log"
    started = time.monotonic()
    with stdout_path.open("a", encoding="utf-8", buffering=1) as output:
        output.write(f"\n===== supervisor launch {_utc_now()} =====\n")
        output.write("COMMAND=" + json.dumps(command) + "\n")
        process = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT,
            cwd=str(Path(__file__).resolve().parent), env=os.environ.copy(),
        )
        while process.poll() is None:
            if stop_event.is_set() or time.monotonic() - started > wall_limit:
                process.terminate()
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=10)
                reason = "supervisor_cancelled" if stop_event.is_set() else "wall_timeout"
                status = read_json(run_dir / "status.json", default={}) or {}
                status.update({
                    "state": reason,
                    "valid_outputs": False,
                    "error": reason,
                    "supervisor_wall_duration_s": time.monotonic() - started,
                })
                atomic_write_json(run_dir / "status.json", status)
                return process.returncode if process.returncode is not None else -1
            time.sleep(1.0)
        return int(process.returncode)


def _write_manifest(results_dir, config, mode, rows):
    payload = {
        "schema": 1,
        "experiment_name": config["experiment_name"],
        "mode": mode,
        "updated_utc": _utc_now(),
        "selected_run_count": len(rows),
        "state_counts": {},
        "runs": rows,
    }
    for row in rows:
        state = row.get("state", "pending")
        payload["state_counts"][state] = payload["state_counts"].get(state, 0) + 1
    atomic_write_json(results_dir / "suite_manifest.json", payload)
    atomic_write_csv(results_dir / "suite_manifest.csv", rows, MANIFEST_FIELDS)


def run_suite(args):
    config, config_path = load_config(args.config)
    plan_path, _ = generate(config_path)
    rows = read_csv(plan_path)
    if args.mode == "plan_only":
        return 0
    selected = _select_rows(config, rows, args.mode, args.limit)
    results_root = resolve_from_config(config_path, config["results_root"])
    suffix = "_local_validation" if args.mode == "validation" else ""
    results_dir = (
        Path(args.results_dir).resolve()
        if args.results_dir else results_root / (config["experiment_name"] + suffix)
    )
    results_dir.mkdir(parents=True, exist_ok=True)
    workers = max(1, int(args.workers))
    wall_limit = float(
        config["local_validation"]["max_wall_time_s"]
        if args.mode == "validation" else config["fixed_conditions"]["max_wall_time_s"]
    )
    retries = max(0, int(config["runner"].get("infrastructure_retries", 0)))
    stop_event = threading.Event()
    old_handlers = {}

    def request_stop(signum, frame):
        print(f"[LAB] Received signal {signum}; stopping new launches.")
        stop_event.set()

    for signame in ("SIGINT", "SIGTERM"):
        signum = getattr(signal, signame, None)
        if signum is not None:
            old_handlers[signum] = signal.signal(signum, request_stop)

    attempts = {row["run_id"]: 0 for row in selected}
    manifest = []
    pending = []
    expected_config_hash = config_hash(config)
    for row in selected:
        run_dir = results_dir / "runs" / row["run_id"]
        if not args.rerun and _terminal_valid(run_dir, expected_config_hash):
            manifest.append(_status_row(row, run_dir))
        else:
            pending.append(row)
            manifest.append(_status_row(row, run_dir))
    _write_manifest(results_dir, config, args.mode, manifest)
    print(
        f"[LAB] mode={args.mode} selected={len(selected)} pending={len(pending)} "
        f"workers={workers} results={results_dir}"
    )

    try:
        queue = list(pending)
        while queue and not stop_event.is_set():
            current = queue
            queue = []
            with cf.ThreadPoolExecutor(max_workers=workers) as executor:
                futures = {}
                for row in current:
                    attempts[row["run_id"]] += 1
                    run_dir = results_dir / "runs" / row["run_id"]
                    command = _child_command(args, config, config_path, plan_path, results_dir, row)
                    futures[executor.submit(
                        _run_child, command, run_dir, wall_limit, stop_event
                    )] = row
                for future in cf.as_completed(futures):
                    row = futures[future]
                    run_dir = results_dir / "runs" / row["run_id"]
                    try:
                        return_code = future.result()
                    except Exception as exc:
                        return_code = -2
                        atomic_write_json(run_dir / "status.json", {
                            "run_id": row["run_id"], "state": "process_failed",
                            "valid_outputs": False, "error": repr(exc),
                        })
                    updated = _status_row(
                        row, run_dir, attempts[row["run_id"]],
                        fallback="process_failed" if return_code else None,
                    )
                    manifest = [updated if item["run_id"] == row["run_id"] else item for item in manifest]
                    _write_manifest(results_dir, config, args.mode, manifest)
                    print(
                        f"[LAB] {row['run_id']} attempt={attempts[row['run_id']]} "
                        f"state={updated['state']} rc={return_code}"
                    )
                    infrastructure_failure = updated["state"] in {
                        "process_failed", "logging_failed", "wall_timeout",
                        "supervisor_cancelled",
                    }
                    if infrastructure_failure and attempts[row["run_id"]] <= retries:
                        queue.append(row)
                    elif infrastructure_failure and config["runner"].get("stop_on_infrastructure_failure"):
                        stop_event.set()
        _write_manifest(results_dir, config, args.mode, manifest)
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    failures = [row for row in manifest if not row.get("valid_outputs")]
    print(f"[LAB] finished: valid={len(manifest) - len(failures)} invalid={len(failures)}")
    return 0 if not failures and len(manifest) == len(selected) else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument(
        "--mode", choices=("plan_only", "validation", "calibration", "core", "full"),
        default=RUN_MODE,
    )
    parser.add_argument("--workers", type=int, default=WORKERS)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--rerun", action="store_true")
    args = parser.parse_args()
    raise SystemExit(run_suite(args))


if __name__ == "__main__":
    main()
