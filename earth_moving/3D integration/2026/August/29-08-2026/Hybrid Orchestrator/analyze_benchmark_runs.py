"""Aggregate benchmark outputs produced by the scheduled hybrid orchestrator.

Usage:
    python analyze_benchmark_runs.py simulation_logs

The aggregate CSV is deliberately built from raw sums; per-push percentages are
never averaged together.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def _number(row, key):
    try:
        return float(row.get(key, 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def aggregate_run(pushes_path: Path):
    rows = list(csv.DictReader(pushes_path.open(encoding="utf-8-sig", newline="")))
    contacted_count = sum(_number(row, "contacted_count") for row in rows)
    contacted_mass = sum(_number(row, "contacted_mass") for row in rows)
    entered_count = sum(_number(row, "entered_target_count") for row in rows)
    entered_mass = sum(_number(row, "entered_target_mass") for row in rows)
    lateral_count = sum(_number(row, "lateral_spill_count") for row in rows)
    left_count = sum(_number(row, "left_target_count") for row in rows)
    gain = sum(_number(row, "transport_gain_m_mass") for row in rows)
    run_id = rows[0]["run_id"] if rows else pushes_path.stem.removeprefix("simulation_").removesuffix("_pushes")
    return {
        "run_id": run_id,
        "push_count": len(rows),
        "contacted_count_sum": contacted_count,
        "contacted_mass_sum": contacted_mass,
        "entered_target_count_sum": entered_count,
        "entered_target_mass_sum": entered_mass,
        "lateral_spill_count_sum": lateral_count,
        "left_target_count_sum": left_count,
        "transport_gain_m_mass_sum": gain,
        "delivery_efficiency_count": entered_count / contacted_count if contacted_count else 0.0,
        "delivery_efficiency_mass": entered_mass / contacted_mass if contacted_mass else 0.0,
        "lateral_spillage_rate_count": lateral_count / contacted_count if contacted_count else 0.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", nargs="?", default="simulation_logs")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    log_dir = Path(args.log_dir).resolve()
    push_files = set(log_dir.glob("simulation_*_pushes.csv"))
    push_files.update(log_dir.glob("b_*_push.csv"))
    runs = [aggregate_run(path) for path in sorted(push_files)]
    output = Path(args.output).resolve() if args.output else log_dir / "benchmark_runs.csv"
    fields = list(runs[0]) if runs else ["run_id", "push_count"]
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(runs)
    print(json.dumps({"runs": len(runs), "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
