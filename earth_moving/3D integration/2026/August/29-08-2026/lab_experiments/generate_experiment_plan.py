"""Generate the immutable, paired full-factorial experiment plan."""

from __future__ import annotations

import argparse
from pathlib import Path

from experiment_utils import (
    DEFAULT_CONFIG, atomic_write_csv, atomic_write_json, atomic_write_text, config_hash,
    load_config, read_json, resolve_from_config,
)


PLAN_FIELDS = (
    "run_id", "stage", "planner_id", "path_mode", "source_mode",
    "candidate_value_mode", "visibility_angle_deg", "max_path_length_factor",
    "target_id", "scenario", "deployment", "rover_profile", "seed", "pebble_count",
    "material_mode", "initial_state_id",
)


def build_plan(config: dict) -> list[dict]:
    fixed = config["fixed_conditions"]
    planner_by_id = {item["id"]: item for item in config["planners"]}
    target_by_id = {item["id"]: item for item in config["targets"]}
    rows = []
    def append_stage(stage, planners, targets, deployments, pebble_counts):
        for target in targets:
            for deployment in deployments:
                for pebble_count in pebble_counts:
                    for rover in config["rover_profiles"]:
                        for seed in config["seeds"]:
                            initial_state_id = (
                                f"{target['id']}__{deployment}__seed{int(seed)}"
                                f"__n{int(pebble_count)}__uniform-small"
                            )
                            for planner in planners:
                                rows.append({
                        "run_id": None,
                        "stage": stage,
                        "planner_id": planner["id"],
                        "path_mode": planner["path_mode"],
                        "source_mode": planner["source_mode"],
                        "candidate_value_mode": planner["candidate_value_mode"],
                        "visibility_angle_deg": float(planner["visibility_angle_deg"]),
                        "max_path_length_factor": float(planner["max_path_length_factor"]),
                        "target_id": target["id"],
                        "scenario": target["scenario"],
                        "deployment": deployment,
                        "rover_profile": rover,
                        "seed": int(seed),
                        "pebble_count": int(pebble_count),
                        "material_mode": fixed["material_mode"],
                        "initial_state_id": initial_state_id,
                                })

    append_stage(
        "core", config["planners"], config["targets"],
        [fixed["core_deployment"]], [fixed["pebble_count"]],
    )
    extended = config["extended_stages"]
    selected_planners = [planner_by_id[name] for name in extended["selected_planners"]]
    deployment = extended["deployment_robustness"]
    append_stage(
        "deployment_robustness", selected_planners,
        [target_by_id[name] for name in deployment["target_ids"]],
        deployment["deployments"], [fixed["pebble_count"]],
    )
    density = extended["density_robustness"]
    append_stage(
        "density_robustness", selected_planners,
        [target_by_id[name] for name in density["target_ids"]],
        [density["deployment"]], density["pebble_counts"],
    )
    for index, row in enumerate(rows, start=1):
        row["run_id"] = f"r{index:06d}"
    return rows


def validate_plan(config: dict, rows: list[dict]) -> None:
    core_expected = (
        len(config["planners"]) * len(config["targets"])
        * len(config["rover_profiles"]) * len(config["seeds"])
    )
    extended = config["extended_stages"]
    selected_count = len(extended["selected_planners"])
    deployment = extended["deployment_robustness"]
    density = extended["density_robustness"]
    deployment_expected = (
        selected_count * len(deployment["target_ids"]) * len(deployment["deployments"])
        * len(config["rover_profiles"]) * len(config["seeds"])
    )
    density_expected = (
        selected_count * len(density["target_ids"]) * len(density["pebble_counts"])
        * len(config["rover_profiles"]) * len(config["seeds"])
    )
    expected = core_expected + deployment_expected + density_expected
    if len(rows) != expected:
        raise AssertionError(f"expected {expected} rows, generated {len(rows)}")
    run_ids = [row["run_id"] for row in rows]
    if len(run_ids) != len(set(run_ids)):
        raise AssertionError("run IDs are not unique")
    expected_by_stage = {
        "core": {item["id"] for item in config["planners"]},
        "deployment_robustness": set(extended["selected_planners"]),
        "density_robustness": set(extended["selected_planners"]),
    }
    groups = {}
    for row in rows:
        key = row["stage"] + "__" + row["initial_state_id"] + "__" + row["rover_profile"]
        groups.setdefault(key, set()).add(row["planner_id"])
    incomplete = [
        key for key, planners in groups.items()
        if planners != expected_by_stage[key.split("__", 1)[0]]
    ]
    if incomplete:
        raise AssertionError(f"paired planner coverage is incomplete for {incomplete[:3]}")


def generate(config_path=DEFAULT_CONFIG, output=None):
    config, config_path = load_config(config_path)
    results_root = resolve_from_config(config_path, config["results_root"])
    experiment_dir = results_root / config["experiment_name"]
    plan_path = Path(output).resolve() if output else experiment_dir / "experiment_plan.csv"
    definition_path = experiment_dir / "experiment_definition.json"
    current_hash = config_hash(config)
    existing = read_json(definition_path, default={}) or {}
    has_runs = any((experiment_dir / "runs").glob("r*/status.json"))
    if has_runs and existing.get("configuration_sha256") != current_hash:
        raise RuntimeError(
            "configuration changed after this experiment started. Choose a new "
            "experiment_name instead of mixing configurations in one result set."
        )
    rows = build_plan(config)
    validate_plan(config, rows)
    atomic_write_csv(plan_path, rows, PLAN_FIELDS)
    atomic_write_text(
        experiment_dir / "experiment_config_snapshot.yaml",
        config_path.read_text(encoding="utf-8"),
    )
    atomic_write_json(definition_path, {
        "schema": 1,
        "experiment_name": config["experiment_name"],
        "configuration_path": str(config_path),
        "configuration_sha256": current_hash,
        "run_count": len(rows),
        "factor_counts": {
            "planners": len(config["planners"]),
            "targets": len(config["targets"]),
            "rover_profiles": len(config["rover_profiles"]),
            "seeds": len(config["seeds"]),
            "stages": {
                stage: sum(1 for row in rows if row["stage"] == stage)
                for stage in sorted({row["stage"] for row in rows})
            },
        },
        "plan_path": str(plan_path),
    })
    print(f"Generated {len(rows)} paired runs: {plan_path}")
    return plan_path, rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    generate(args.config, args.output)


if __name__ == "__main__":
    main()
