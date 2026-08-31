"""Compare paced and accelerated headless initialization for one plan row."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

from experiment_utils import DEFAULT_CONFIG, load_config, read_json, resolve_from_config
from generate_experiment_plan import generate


def main():
    config, config_path = load_config(DEFAULT_CONFIG)
    plan_path, rows = generate(config_path)
    run_id = rows[0]["run_id"]
    root = resolve_from_config(config_path, config["results_root"]) / "runtime_parity"
    statuses = {}
    for mode in ("headless", "headless_fast"):
        result_dir = root / mode
        command = [
            sys.executable, str(Path(__file__).resolve().parent / "run_single_experiment.py"),
            "--config", str(config_path), "--plan", str(plan_path),
            "--run-id", run_id, "--results-dir", str(result_dir),
            "--runtime-mode", mode, "--pebbles", "2",
            "--max-sim-time", "1.0", "--max-no-delivery-time", "1.0",
        ]
        completed = subprocess.run(command, cwd=str(Path(__file__).resolve().parent), check=False)
        if completed.returncode:
            raise RuntimeError(f"{mode} parity child failed with {completed.returncode}")
        statuses[mode] = read_json(result_dir / "runs" / run_id / "status.json", default={})
    left, right = statuses["headless"], statuses["headless_fast"]
    if left.get("initial_state_sha256") != right.get("initial_state_sha256"):
        raise AssertionError("paced and accelerated modes did not start from identical pebble states")
    for status in statuses.values():
        if not status.get("valid_outputs"):
            raise AssertionError(f"parity run has invalid outputs: {json.dumps(status, indent=2)}")
    print(
        "RUNTIME PARITY PASS: identical initial-state fingerprint; "
        "both modes produced complete telemetry."
    )


if __name__ == "__main__":
    main()

