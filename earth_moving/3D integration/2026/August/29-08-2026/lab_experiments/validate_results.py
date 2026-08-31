"""Fail-fast structural and output validation before a server-scale launch."""

from __future__ import annotations

import argparse
from pathlib import Path

from experiment_utils import DEFAULT_CONFIG, config_hash, load_config, read_csv, read_json, resolve_from_config, validate_run_outputs
from generate_experiment_plan import build_plan, validate_plan


def validate_structure(config_path=DEFAULT_CONFIG):
    config, config_path = load_config(config_path)
    rows = build_plan(config)
    validate_plan(config, rows)
    if len(config["planners"]) != 14:
        raise AssertionError("the core plan must contain 14 planner variants")
    if len(config["targets"]) != 8:
        raise AssertionError("the core plan must contain 8 target scenarios")
    if set(config["rover_profiles"]) != {"small", "large"}:
        raise AssertionError("the core plan must compare small and large rovers")
    if len(config["seeds"]) < 5:
        raise AssertionError("at least five paired seeds are required")
    core_expected = 14 * 8 * 2 * len(config["seeds"])
    core_rows = [row for row in rows if row["stage"] == "core"]
    if len(core_rows) != core_expected:
        raise AssertionError(f"expected {core_expected} core rows, got {len(core_rows)}")
    if len(rows) != 2400:
        raise AssertionError(f"expected 2400 staged rows, got {len(rows)}")
    simulation_dir = resolve_from_config(config_path, config["simulation_dir"])
    required = [
        simulation_dir / "orchestrator_hybrid_multi_astar_scheduled.py",
        simulation_dir / "2_wheel_rover.urdf",
        simulation_dir.parent / "Path Tracking" / "astar_path_following_flowfield.py",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise AssertionError(f"missing simulation dependencies: {missing}")
    print(
        f"STRUCTURE PASS: {len(rows)} staged runs "
        f"({len(core_rows)} core), 14 planners, 8 targets, 2 rovers, "
        f"{len(config['seeds'])} seeds"
    )
    return rows


def validate_results(config_path=DEFAULT_CONFIG, results_dir=None, require_complete=False):
    config, config_path = load_config(config_path)
    if results_dir is None:
        results_dir = resolve_from_config(config_path, config["results_root"]) / config["experiment_name"]
    results_dir = Path(results_dir).resolve()
    status_paths = sorted((results_dir / "runs").glob("r*/status.json"))
    if not status_paths:
        raise AssertionError(f"no run status files found under {results_dir}")
    errors = []
    hashes = {}
    expected_hash = config_hash(config)
    for status_path in status_paths:
        status = read_json(status_path, default={}) or {}
        recorded_hash = (status.get("reproducibility") or {}).get("configuration_sha256")
        if recorded_hash != expected_hash:
            errors.append(f"{status_path.parent.name}: configuration hash mismatch")
        valid, missing, _, summary = validate_run_outputs(status_path.parent)
        if bool(status.get("valid_outputs")) != valid:
            errors.append(f"{status_path.parent.name}: status/output validity mismatch")
        if missing:
            errors.append(f"{status_path.parent.name}: missing {missing}")
        if require_complete and status.get("state") != "completed":
            errors.append(f"{status_path.parent.name}: state={status.get('state')}")
        initial_id = (status.get("plan") or {}).get("initial_state_id")
        digest = status.get("initial_state_sha256")
        if valid and not digest:
            errors.append(f"{status_path.parent.name}: missing initial-state fingerprint")
        if initial_id and digest:
            hashes.setdefault(initial_id, set()).add(digest)
        if status.get("state") == "completed":
            progress = summary.get("final_material_progress", {}) or {}
            if summary.get("outcome") != "mission_complete" or int(progress.get("remaining_count", -1)) != 0:
                errors.append(f"{status_path.parent.name}: invalid completed-mission claim")
    inconsistent = {key: value for key, value in hashes.items() if len(value) > 1}
    if inconsistent:
        errors.append(f"paired initial states differ: {sorted(inconsistent)[:5]}")
    if errors:
        raise AssertionError("RESULT VALIDATION FAILED\n- " + "\n- ".join(errors))
    print(f"RESULTS PASS: {len(status_paths)} valid runs; paired initial-state fingerprints are consistent")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--structure-only", action="store_true")
    parser.add_argument("--require-complete", action="store_true")
    args = parser.parse_args()
    validate_structure(args.config)
    if not args.structure_only:
        validate_results(args.config, args.results_dir, args.require_complete)


if __name__ == "__main__":
    main()
