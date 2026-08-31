"""Create normalized CSV tables and a compact report from immutable raw runs."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import statistics
import tempfile

from experiment_utils import DEFAULT_CONFIG, atomic_write_csv, atomic_write_text, load_config, read_csv, read_json, resolve_from_config


META_FIELDS = (
    "experiment_run_id", "stage", "planner_id", "target_id", "scenario",
    "deployment", "pebble_count", "rover_profile", "seed", "initial_state_id",
)


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _ci95(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return None, None, None
    mean = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    half = 1.96 * sd / math.sqrt(len(values)) if len(values) > 1 else 0.0
    return mean, sd, half


def _difference(left, right):
    left, right = _number(left), _number(right)
    return None if left is None or right is None else left - right


def _combine_table(run_rows, output_path, output_key):
    sources = []
    native_fields = []
    for run in run_rows:
        path = (run.get("outputs") or {}).get(output_key)
        if not path or not Path(path).exists():
            continue
        with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            for field in reader.fieldnames or ():
                if field not in native_fields:
                    native_fields.append(field)
        sources.append((run, Path(path)))
    fields = list(META_FIELDS) + native_fields
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8-sig", newline="", delete=False,
        dir=str(output_path.parent), prefix=output_path.name + ".", suffix=".tmp",
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for run, source in sources:
            meta = {
                "experiment_run_id": run["run_id"],
                **{key: run.get(key) for key in META_FIELDS if key != "experiment_run_id"},
            }
            with source.open("r", encoding="utf-8-sig", newline="") as input_stream:
                for row in csv.DictReader(input_stream):
                    writer.writerow({**meta, **row})
        temporary = Path(stream.name)
    os.replace(temporary, output_path)


def analyze(config_path=DEFAULT_CONFIG, results_dir=None):
    config, config_path = load_config(config_path)
    if results_dir is None:
        results_dir = resolve_from_config(config_path, config["results_root"]) / config["experiment_name"]
    results_dir = Path(results_dir).resolve()
    analysis_dir = results_dir / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    plan_path = resolve_from_config(config_path, config["results_root"]) / config["experiment_name"] / "experiment_plan.csv"
    plan = {row["run_id"]: row for row in read_csv(plan_path)}

    run_rows = []
    for status_path in sorted((results_dir / "runs").glob("r*/status.json")):
        status = read_json(status_path, default={}) or {}
        run_id = status.get("run_id") or status_path.parent.name
        row = plan.get(run_id, status.get("plan", {})).copy()
        progress = status.get("final_material_progress", {}) or {}
        factors = status.get("experimental_factors", {}) or {}
        target_zone = factors.get("target_zone", {}) or {}
        target_center = target_zone.get("center") or (None, None)
        rover_factors = factors.get("rover", {}) or {}
        row.update({
            "state": status.get("state"), "outcome": status.get("outcome"),
            "valid_outputs": status.get("valid_outputs"),
            "wall_duration_s": status.get("wall_duration_s"),
            "end_sim_time_s": status.get("end_sim_time_s"),
            "delivered_count": progress.get("delivered_count"),
            "remaining_count": progress.get("remaining_count"),
            "push_count": status.get("push_count"),
            "delivery_efficiency_count": status.get("delivery_efficiency_count"),
            "lateral_spillage_rate_count": status.get("lateral_spillage_rate_count"),
            "initial_state_sha256": status.get("initial_state_sha256"),
            "runtime_mode": (status.get("effective") or {}).get("runtime_mode"),
            "capacity_scoring_mode": (status.get("effective") or {}).get("capacity_scoring_mode"),
            "effective_pebble_count": (status.get("effective") or {}).get("pebble_count"),
            "target_shape": target_zone.get("shape"),
            "target_center_x_m": target_center[0],
            "target_center_y_m": target_center[1],
            "target_area_m2": target_zone.get("area"),
            "shovel_width_m": rover_factors.get("shovel_width_m"),
            "overlay_cell_size_m": rover_factors.get("overlay_cell_size_m"),
            "nominal_capacity_objects": rover_factors.get("nominal_capacity_objects"),
            "outputs": status.get("outputs") or {},
        })
        run_rows.append(row)

    summary_fields = [
        "run_id", "stage", "planner_id", "path_mode", "source_mode", "candidate_value_mode",
        "visibility_angle_deg", "target_id", "scenario", "rover_profile", "seed",
        "deployment", "pebble_count",
        "initial_state_id", "initial_state_sha256", "runtime_mode", "capacity_scoring_mode",
        "effective_pebble_count", "target_shape", "target_center_x_m", "target_center_y_m",
        "target_area_m2", "shovel_width_m", "overlay_cell_size_m", "nominal_capacity_objects",
        "state", "outcome", "valid_outputs", "wall_duration_s", "end_sim_time_s",
        "delivered_count", "remaining_count", "push_count", "delivery_efficiency_count",
        "lateral_spillage_rate_count",
    ]
    atomic_write_csv(analysis_dir / "run_summary.csv", run_rows, summary_fields)

    completed = [row for row in run_rows if row.get("state") == "completed"]
    groups = {}
    for row in completed:
        key = (
            row.get("stage"), row.get("planner_id"), row.get("target_id"),
            row.get("deployment"), row.get("pebble_count"), row.get("rover_profile"),
        )
        groups.setdefault(key, []).append(row)
    planner_summary = []
    for (stage, planner, target, deployment, pebble_count, rover), rows in sorted(groups.items()):
        summary = {
            "stage": stage, "planner_id": planner, "target_id": target,
            "deployment": deployment, "pebble_count": pebble_count,
            "rover_profile": rover,
            "completed_runs": len(rows),
        }
        for field in ("end_sim_time_s", "wall_duration_s", "push_count", "delivery_efficiency_count", "lateral_spillage_rate_count"):
            mean, sd, half = _ci95([_number(row.get(field)) for row in rows])
            summary[field + "_mean"] = mean
            summary[field + "_sd"] = sd
            summary[field + "_ci95_half"] = half
        planner_summary.append(summary)
    planner_fields = list(planner_summary[0]) if planner_summary else ["stage", "planner_id", "target_id", "deployment", "pebble_count", "rover_profile", "completed_runs"]
    atomic_write_csv(analysis_dir / "planner_target_rover_summary.csv", planner_summary, planner_fields)

    baseline_id = config["planners"][0]["id"]
    paired = {}
    for row in completed:
        key = (
            row.get("stage"), row.get("target_id"), row.get("deployment"),
            row.get("pebble_count"), row.get("rover_profile"), row.get("seed"),
        )
        paired.setdefault(key, {})[row.get("planner_id")] = row
    comparisons = []
    for key, methods in sorted(paired.items()):
        baseline = methods.get(baseline_id)
        if baseline is None:
            continue
        for planner, row in sorted(methods.items()):
            if planner == baseline_id:
                continue
            comparisons.append({
                "stage": key[0], "target_id": key[1], "deployment": key[2],
                "pebble_count": key[3], "rover_profile": key[4], "seed": key[5],
                "baseline_planner": baseline_id, "planner_id": planner,
                "delta_sim_time_s": _difference(row.get("end_sim_time_s"), baseline.get("end_sim_time_s")),
                "delta_push_count": _difference(row.get("push_count"), baseline.get("push_count")),
                "delta_delivery_efficiency": _difference(row.get("delivery_efficiency_count"), baseline.get("delivery_efficiency_count")),
                "delta_lateral_spillage_rate": _difference(row.get("lateral_spillage_rate_count"), baseline.get("lateral_spillage_rate_count")),
            })
    comparison_fields = list(comparisons[0]) if comparisons else ["stage", "target_id", "deployment", "pebble_count", "rover_profile", "seed", "baseline_planner", "planner_id"]
    atomic_write_csv(analysis_dir / "paired_comparisons.csv", comparisons, comparison_fields)

    for key, name in (("PUSHES_CSV", "pushes.csv"), ("TASKS_CSV", "tasks.csv"), ("SNAPSHOTS_CSV", "environment_snapshots.csv")):
        _combine_table(run_rows, analysis_dir / name, key)

    state_counts = {}
    for row in run_rows:
        state_counts[row.get("state", "unknown")] = state_counts.get(row.get("state", "unknown"), 0) + 1
    lines = [
        "# Experiment analysis", "",
        f"- Results: `{results_dir}`",
        f"- Runs discovered: {len(run_rows)}",
        f"- Completed missions: {len(completed)}",
        f"- States: {state_counts}",
        f"- Paired comparisons against `{baseline_id}`: {len(comparisons)}", "",
        "The normalized tables keep planner, target, rover, seed, and paired initial-state identifiers on every row.",
        "Raw JSONL and compressed heatmap/pebble snapshots remain immutable in each run directory.",
    ]
    atomic_write_text(analysis_dir / "REPORT.md", "\n".join(lines) + "\n")
    print(f"Analysis written to {analysis_dir}")
    return analysis_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    analyze(args.config, args.results_dir)


if __name__ == "__main__":
    main()
