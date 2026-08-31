"""Run every direct-target comparison sequentially in isolated processes.

Each child run owns its PyBullet/Pygame lifecycle and exits only after all
material is delivered and its rover is parked, or after an adjustable safeguard
is reached.  A manifest is rewritten after every child so partial batches remain
usable if the machine or a later simulation stops.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

from RUN_DIRECT_PATH_COMPARISON import (
    COMPARISON_PRESETS,
    COMPARISON_SCENARIOS,
    PEBBLE_COUNT,
    RANDOM_SEED,
    TEST_SCENARIO,
)


# ===================== EASY EDIT SETTINGS =====================
COMPARISONS_TO_RUN = tuple(COMPARISON_PRESETS)
BATCH_SCENARIO = TEST_SCENARIO
BATCH_PEBBLE_COUNT = PEBBLE_COUNT
BATCH_RANDOM_SEED = RANDOM_SEED
MAX_SIM_TIME_PER_RUN = 7200.0
MAX_NO_DELIVERY_SIM_TIME = 1200.0
MAX_WALL_TIME_PER_RUN = 10800.0
STOP_BATCH_ON_INVALID_RUN = True
# Set True, click Play once, and inspect the printed PASS result.  No physics
# simulation starts.  Set it back to False for the overnight batch.
VALIDATE_ONLY = False
# ==============================================================


HERE = Path(__file__).resolve().parent
DEFAULT_BATCH_ROOT = HERE / "batch_logs"
LATEST_BATCH_POINTER = HERE / "simulation_logs" / "LATEST_COMPARISON_BATCH.txt"
WINDOWS_SAFE_PATH_LIMIT = 245
RUN_DIRECTORY_CODES = {
    name: f"p{index}"
    for index, name in enumerate(COMPARISON_PRESETS)
}
MANIFEST_FIELDS = (
    "comparison", "scenario", "status", "outcome", "return_code",
    "wall_duration_s", "end_sim_time_s", "delivered_count",
    "remaining_count", "push_count", "delivery_efficiency_count",
    "lateral_spillage_rate_count", "run_summary_json", "jsonl",
    "tasks_csv", "pushes_csv", "snapshots_csv", "artifact_dir",
    "log_dir", "error",
)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run controlled direct-target comparisons one after another."
    )
    parser.add_argument(
        "--comparisons", nargs="+", choices=tuple(COMPARISON_PRESETS),
        default=list(COMPARISONS_TO_RUN),
    )
    parser.add_argument(
        "--scenario", choices=COMPARISON_SCENARIOS, default=BATCH_SCENARIO,
    )
    parser.add_argument("--pebbles", type=int, default=BATCH_PEBBLE_COUNT)
    parser.add_argument("--seed", type=int, default=BATCH_RANDOM_SEED)
    parser.add_argument("--max-sim-time", type=float, default=MAX_SIM_TIME_PER_RUN)
    parser.add_argument(
        "--max-no-delivery-time", type=float,
        default=MAX_NO_DELIVERY_SIM_TIME,
    )
    parser.add_argument("--max-wall-time", type=float, default=MAX_WALL_TIME_PER_RUN)
    parser.add_argument("--batch-name", default=None)
    parser.add_argument("--output-root", default=str(DEFAULT_BATCH_ROOT))
    parser.add_argument(
        "--stop-on-error", dest="stop_on_error", action="store_true",
        help="Stop after a child has missing/corrupt logs or a process failure.",
    )
    parser.add_argument(
        "--continue-on-error", dest="stop_on_error", action="store_false",
        help="Record infrastructure failures and continue to the next preset.",
    )
    parser.set_defaults(stop_on_error=STOP_BATCH_ON_INVALID_RUN)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _safe_batch_name(value):
    raw = value or datetime.now().strftime("batch_%Y%m%d_%H%M%S")
    cleaned = "".join(char if char.isalnum() or char in "-_" else "_" for char in raw)
    return cleaned.strip("_") or datetime.now().strftime("batch_%Y%m%d_%H%M%S")


def _read_latest(log_dir):
    latest_path = Path(log_dir) / "LATEST_RUN.txt"
    result = {}
    if not latest_path.exists():
        return result
    for line in latest_path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def _read_summary(latest):
    path = latest.get("RUN_SUMMARY_JSON")
    if not path or not Path(path).exists():
        return {}
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _manifest_payload(args, batch_name, rows):
    return {
        "schema": 1,
        "batch_name": batch_name,
        "physical_batch_id": getattr(args, "physical_batch_id", None),
        "batch_dir": str(getattr(args, "batch_dir", "")),
        "scenario": args.scenario,
        "pebbles": int(args.pebbles),
        "seed": int(args.seed),
        "comparisons": list(args.comparisons),
        "max_sim_time_per_run_s": float(args.max_sim_time),
        "max_no_delivery_time_s": float(args.max_no_delivery_time),
        "max_wall_time_per_run_s": float(args.max_wall_time),
        "runs": rows,
    }


def _write_manifest(batch_dir, args, batch_name, rows):
    batch_dir.mkdir(parents=True, exist_ok=True)
    json_path = batch_dir / "batch_manifest.json"
    csv_path = batch_dir / "batch_manifest.csv"
    json_path.write_text(
        json.dumps(_manifest_payload(args, batch_name, rows), indent=2) + "\n",
        encoding="utf-8",
    )
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in MANIFEST_FIELDS})
    LATEST_BATCH_POINTER.parent.mkdir(parents=True, exist_ok=True)
    LATEST_BATCH_POINTER.write_text(
        f"BATCH_DIR={batch_dir}\n"
        f"MANIFEST_JSON={json_path}\n"
        f"MANIFEST_CSV={csv_path}\n",
        encoding="utf-8",
    )
    return json_path, csv_path


def _preflight_run_directory(run_log_dir):
    """Verify legacy-Windows path budget and actual file write access."""
    run_log_dir = Path(run_log_dir).resolve()
    probe_relatives = (
        "simulation_20260826_235959_123456_abcdef.jsonl",
        "simulation_20260826_235959_123456_abcdef_tasks.csv",
        "b_123456abcdef_push.csv",
        "b_123456abcdef_snap.csv",
        "b_123456abcdef_run.json",
        "b_123456abcdef/data/00001.json.gz",
        "b_123456abcdef/img/00001.png",
    )
    longest = max((run_log_dir / relative for relative in probe_relatives), key=lambda p: len(str(p)))
    if len(str(longest)) > WINDOWS_SAFE_PATH_LIMIT:
        raise RuntimeError(
            "Batch log path is too long for reliable Windows telemetry: "
            f"{len(str(longest))} characters (safe limit "
            f"{WINDOWS_SAFE_PATH_LIMIT}). Choose a shorter output root. "
            f"Longest path: {longest}"
        )

    created_files = []
    created_dirs = []
    try:
        run_log_dir.mkdir(parents=True, exist_ok=True)
        for relative in probe_relatives:
            path = run_log_dir / ("_preflight_" + relative if "/" not in relative else relative.replace("b_123456abcdef", "_preflight_b"))
            if not path.parent.exists():
                path.parent.mkdir(parents=True, exist_ok=True)
                created_dirs.append(path.parent)
            path.write_bytes(b"preflight")
            created_files.append(path)
    finally:
        for path in reversed(created_files):
            try:
                path.unlink()
            except OSError:
                pass
        for path in sorted(set(created_dirs), key=lambda item: len(str(item)), reverse=True):
            try:
                path.rmdir()
            except OSError:
                pass
        probe_root = run_log_dir / "_preflight_b"
        for path in (probe_root / "data", probe_root / "img", probe_root):
            try:
                path.rmdir()
            except OSError:
                pass
    return len(str(longest)), longest


def _preflight_batch(batch_dir, comparisons):
    results = []
    for comparison in comparisons:
        run_dir = batch_dir / RUN_DIRECTORY_CODES[comparison]
        longest_length, longest_path = _preflight_run_directory(run_dir)
        results.append((comparison, run_dir, longest_length, longest_path))
    return results


def _command(args, comparison, run_log_dir):
    command = [
        sys.executable,
        str(HERE / "RUN_DIRECT_PATH_COMPARISON.py"),
        comparison,
        "--scenario", args.scenario,
        "--pebbles", str(args.pebbles),
        "--seed", str(args.seed),
        "--event-log-dir", str(run_log_dir),
        "--auto-exit-on-completion",
        "--max-sim-time", str(args.max_sim_time),
        "--max-no-delivery-time", str(args.max_no_delivery_time),
    ]
    return command


def _result_row(args, comparison, run_log_dir, return_code, wall_duration,
                error=""):
    latest = _read_latest(run_log_dir)
    summary = _read_summary(latest)
    progress = summary.get("final_material_progress", {}) or {}
    outcome = summary.get("outcome") or (
        "process_error" if return_code else "missing_run_summary"
    )
    remaining = progress.get("remaining_count")
    status = (
        "completed"
        if outcome == "mission_complete" and int(remaining or 0) == 0
        else outcome
    )
    required_outputs = (
        "JSONL", "TASKS_CSV", "PUSHES_CSV", "SNAPSHOTS_CSV",
        "RUN_SUMMARY_JSON", "ARTIFACT_DIR",
    )
    missing_outputs = [
        key for key in required_outputs
        if not latest.get(key) or not Path(latest[key]).exists()
    ]
    if not summary:
        status = "missing_run_summary"
        outcome = "missing_run_summary"
        if not error:
            error = "child exited without a readable RUN_SUMMARY_JSON"
    elif missing_outputs:
        status = "incomplete_logging"
        if not error:
            error = "missing benchmark outputs: " + ", ".join(missing_outputs)
    if return_code not in (0, None):
        status = "wall_timeout" if return_code == -1 else "process_error"
        if not error:
            error = f"child process exited with code {return_code}"
    return {
        "comparison": comparison,
        "scenario": args.scenario,
        "status": status,
        "outcome": outcome,
        "return_code": return_code,
        "wall_duration_s": round(float(wall_duration), 3),
        "end_sim_time_s": summary.get("end_sim_time"),
        "delivered_count": progress.get("delivered_count"),
        "remaining_count": remaining,
        "push_count": summary.get("push_count"),
        "delivery_efficiency_count": summary.get(
            "aggregate_delivery_efficiency_count"
        ),
        "lateral_spillage_rate_count": summary.get(
            "aggregate_lateral_spillage_rate_count"
        ),
        "run_summary_json": latest.get("RUN_SUMMARY_JSON"),
        "jsonl": latest.get("JSONL"),
        "tasks_csv": latest.get("TASKS_CSV"),
        "pushes_csv": latest.get("PUSHES_CSV"),
        "snapshots_csv": latest.get("SNAPSHOTS_CSV"),
        "artifact_dir": latest.get("ARTIFACT_DIR"),
        "log_dir": str(run_log_dir),
        "error": error,
    }


def main():
    args = _parse_args()
    if VALIDATE_ONLY:
        args.dry_run = True
    batch_name = _safe_batch_name(args.batch_name)
    physical_batch_id = datetime.now().strftime("b%y%m%d_%H%M%S")
    batch_dir = Path(args.output_root).resolve() / physical_batch_id
    args.physical_batch_id = physical_batch_id
    args.batch_dir = batch_dir
    preflight = _preflight_batch(batch_dir, args.comparisons)
    print("Sequential comparison batch")
    print(f"  name:        {batch_name}")
    print(f"  physical id: {physical_batch_id}")
    print(f"  scenario:    {args.scenario}")
    print(f"  comparisons: {' -> '.join(args.comparisons)}")
    print(f"  output:      {batch_dir}")
    print("  preflight:   PASS")
    for comparison, run_dir, length, _ in preflight:
        print(f"    {comparison}: {run_dir.name}, longest path={length}")
    for comparison in args.comparisons:
        print("  " + " ".join(_command(
            args, comparison, batch_dir / RUN_DIRECTORY_CODES[comparison]
        )))
    if args.dry_run:
        print("Validation/dry run only; no simulation was started.")
        return 0

    rows = []
    _write_manifest(batch_dir, args, batch_name, rows)
    for index, comparison in enumerate(args.comparisons, start=1):
        run_log_dir = batch_dir / RUN_DIRECTORY_CODES[comparison]
        run_log_dir.mkdir(parents=True, exist_ok=True)
        command = _command(args, comparison, run_log_dir)
        print("\n" + "=" * 78)
        print(f"BATCH RUN {index}/{len(args.comparisons)}: {comparison}")
        print("=" * 78)
        started = time.monotonic()
        error = ""
        return_code = None
        try:
            completed = subprocess.run(
                command,
                cwd=str(HERE),
                check=False,
                timeout=max(1.0, float(args.max_wall_time)),
            )
            return_code = int(completed.returncode)
            if return_code:
                error = f"child process exited with code {return_code}"
        except subprocess.TimeoutExpired:
            return_code = -1
            error = f"wall-time limit exceeded ({args.max_wall_time:.1f}s)"
        except Exception as exc:
            return_code = -2
            error = repr(exc)
        wall_duration = time.monotonic() - started
        row = _result_row(
            args, comparison, run_log_dir, return_code, wall_duration, error
        )
        if error and not row.get("run_summary_json"):
            if return_code == -1:
                row["status"] = "wall_timeout"
                row["outcome"] = "wall_timeout"
            elif return_code not in (0, None):
                row["status"] = "process_error"
                row["outcome"] = "process_error"
        rows.append(row)
        json_path, csv_path = _write_manifest(
            batch_dir, args, batch_name, rows
        )
        print(
            f"[BATCH] {comparison}: status={row['status']}, "
            f"remaining={row['remaining_count']}, wall={wall_duration:.1f}s"
        )
        print(f"[BATCH] Manifest updated: {csv_path}")
        infrastructure_failure = row["status"] in {
            "missing_run_summary", "incomplete_logging", "wall_timeout",
            "process_error",
        }
        if args.stop_on_error and infrastructure_failure:
            print("[BATCH] Stopping because this run did not produce valid logs.")
            break

    print("\nBatch finished.")
    print(f"JSON manifest: {json_path}")
    print(f"CSV manifest:  {csv_path}")
    return 0 if rows and all(row["status"] == "completed" for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
