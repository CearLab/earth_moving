"""Measure safe local throughput at 1, 2 and 4 workers using tiny smoke runs."""

from __future__ import annotations

import csv
from pathlib import Path
import subprocess
import sys
import time

import psutil

from experiment_utils import DEFAULT_CONFIG, atomic_write_csv, load_config, resolve_from_config


WORKER_COUNTS = (1, 2, 4)


def main():
    config, config_path = load_config(DEFAULT_CONFIG)
    root = resolve_from_config(config_path, config["results_root"]) / "worker_benchmark"
    rows = []
    for workers in WORKER_COUNTS:
        result_dir = root / f"w{workers}"
        command = [
            sys.executable, str(Path(__file__).resolve().parent / "run_experiment_suite.py"),
            "--config", str(config_path), "--mode", "validation",
            "--workers", str(workers), "--results-dir", str(result_dir), "--rerun",
        ]
        started = time.perf_counter()
        process = subprocess.Popen(command, cwd=str(Path(__file__).resolve().parent))
        peak_rss = 0
        while process.poll() is None:
            try:
                parent = psutil.Process(process.pid)
                processes = [parent] + parent.children(recursive=True)
                rss = sum(item.memory_info().rss for item in processes if item.is_running())
                peak_rss = max(peak_rss, rss)
            except (psutil.Error, OSError):
                pass
            time.sleep(0.10)
        return_code = int(process.returncode)
        elapsed = time.perf_counter() - started
        rows.append({
            "workers": workers, "wall_duration_s": elapsed,
            "runs": len(config["local_validation"]["cases"]),
            "runs_per_hour": 3600.0 * len(config["local_validation"]["cases"]) / elapsed,
            "peak_total_rss_gib": peak_rss / 2**30,
            "return_code": return_code,
        })
        if return_code:
            raise RuntimeError(f"worker benchmark failed at {workers} workers")
    atomic_write_csv(root / "worker_benchmark.csv", rows, rows[0].keys())
    print(f"Worker benchmark: {root / 'worker_benchmark.csv'}")


if __name__ == "__main__":
    main()
