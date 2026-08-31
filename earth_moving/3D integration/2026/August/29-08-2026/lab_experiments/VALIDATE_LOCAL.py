"""Click Play here for the pre-server validation sequence."""

from __future__ import annotations

from pathlib import Path
import os
import subprocess
import sys

from experiment_utils import DEFAULT_CONFIG, load_config, resolve_from_config


# Quick covers unit tests, plan structure, four short headless cases, resume,
# normalized analysis, and runtime parity. Run BENCHMARK_LOCAL_WORKERS.py after
# this passes to choose the lab worker count.
VALIDATION_LEVEL = "quick"


def _run(command, cwd):
    print("\n[VALIDATE] " + " ".join(str(item) for item in command))
    completed = subprocess.run(command, cwd=str(cwd), check=False)
    if completed.returncode:
        raise SystemExit(completed.returncode)


def main():
    here = Path(__file__).resolve().parent
    config, config_path = load_config(DEFAULT_CONFIG)
    simulation_dir = resolve_from_config(config_path, config["simulation_dir"])
    validation_results = (
        resolve_from_config(config_path, config["results_root"])
        / (config["experiment_name"] + "_local_validation")
    )
    local_temp = validation_results / "_tmp"
    local_temp.mkdir(parents=True, exist_ok=True)
    os.environ["TMP"] = str(local_temp)
    os.environ["TEMP"] = str(local_temp)
    _run([
        sys.executable, "-m", "compileall", "-q",
        "-x", r"(results|\.test_artifacts|__pycache__)",
        str(simulation_dir), str(here),
    ], here)
    _run([sys.executable, "-m", "unittest", "discover", "-s", str(simulation_dir), "-p", "test_*.py"], here)
    _run([sys.executable, "-m", "unittest", "discover", "-s", str(here), "-p", "test_*.py"], here)
    _run([sys.executable, str(here / "CHECK_SERVER_ENVIRONMENT.py")], here)
    _run([sys.executable, str(here / "validate_results.py"), "--config", str(config_path), "--structure-only"], here)
    _run([sys.executable, str(here / "run_experiment_suite.py"), "--config", str(config_path), "--mode", "validation", "--workers", "1"], here)
    # A second launch must discover valid terminal statuses and skip them.
    _run([sys.executable, str(here / "run_experiment_suite.py"), "--config", str(config_path), "--mode", "validation", "--workers", "1"], here)
    _run([sys.executable, str(here / "validate_results.py"), "--config", str(config_path), "--results-dir", str(validation_results)], here)
    _run([sys.executable, str(here / "analyze_experiment.py"), "--config", str(config_path), "--results-dir", str(validation_results)], here)
    plan_path = (
        resolve_from_config(config_path, config["results_root"])
        / config["experiment_name"] / "experiment_plan.csv"
    )
    completion_results = resolve_from_config(config_path, config["results_root"]) / "completion_smoke"
    _run([
        sys.executable, str(here / "run_single_experiment.py"),
        "--config", str(config_path), "--plan", str(plan_path),
        "--run-id", "r000001", "--results-dir", str(completion_results),
        "--runtime-mode", "headless_fast", "--pebbles", "1",
        "--max-sim-time", "600", "--max-no-delivery-time", "300",
    ], here)
    _run([
        sys.executable, str(here / "validate_results.py"),
        "--config", str(config_path), "--results-dir", str(completion_results),
        "--require-complete",
    ], here)
    if VALIDATION_LEVEL == "quick":
        _run([sys.executable, str(here / "VALIDATE_RUNTIME_PARITY.py")], here)
    print("\nLOCAL VALIDATION PASS. Run BENCHMARK_LOCAL_WORKERS.py next, then a calibration batch on the lab server.")


if __name__ == "__main__":
    main()
