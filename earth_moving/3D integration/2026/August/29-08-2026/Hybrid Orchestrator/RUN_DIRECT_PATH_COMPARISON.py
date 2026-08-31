"""Run controlled single-rover direct-path comparisons by clicking Play.

Change COMPARISON_NAME and TEST_SCENARIO below, save, and click Play on this
file in AntiGravity.  Every preset is target-only and uses the same execution,
physics, logging, and benchmark telemetry as the full scheduled orchestrator.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys


# ===================== EASY EDIT SETTINGS =====================
COMPARISON_NAME = "P2_wider_root_source_only"
TEST_SCENARIO = "baseline_circle_uniform"
PEBBLE_COUNT = 60
RANDOM_SEED = 41
# ==============================================================


SOURCE_ONLY_POLICY = {
    "allowed_tasks": frozenset({"target"}),
    "task_policy_mode": "target_only",
    "target_root_sources_only": True,
    "delivered_weight": 1.0,
    "target_weight": 1.0,
    "highway_weight": 0.0,
    "heat_weight": 0.0,
    "capacity_utilization_weight": 0.0,
    "approach_distance_weight": 0.5,
    "spillage_weight": 0.0,
    "minimum_task_objects": 1.0,
    "minimum_capacity_utilization": 0.0,
    "preferred_capacity_min_fraction": 0.0,
    "preferred_capacity_max_fraction": 1.0,
    "overcapacity_behavior": "allow",
    "max_overcapacity_fraction": None,
    "capacity_fit_before_task_fallback": False,
}

MATERIAL_AWARE_POLICY = {
    "allowed_tasks": frozenset({"target"}),
    "task_policy_mode": "target_only",
    "target_root_sources_only": True,
    "delivered_weight": 10.0,
    "target_weight": 1.0,
    "highway_weight": 0.0,
    "heat_weight": 2.0,
    "capacity_utilization_weight": 2.0,
    "approach_distance_weight": 0.5,
    "spillage_weight": 2.0,
    "minimum_task_objects": 1.0,
    "minimum_capacity_utilization": 0.0,
    "preferred_capacity_min_fraction": 0.0,
    "preferred_capacity_max_fraction": 1.0,
    "overcapacity_behavior": "deprioritize",
    "max_overcapacity_fraction": None,
    "capacity_fit_before_task_fallback": False,
}


COMPARISON_PRESETS = {
    "P0_straight_root": {
        "title": "Straight convex-hull-source baseline",
        "description": (
            "Geometrically exposed convex-hull sources only; one explicit "
            "straight push to the nearest target boundary; selected-source value only."
        ),
        "path_mode": "straight_nearest",
        "candidate_value_mode": "source_only",
        "source_mode": "convex_hull",
        "visibility_angle": 1.0,
        "max_path_length_factor": 1.01,
        "policy": SOURCE_ONLY_POLICY,
    },
    "P1_narrow_root_5deg": {
        "title": "Narrow-visibility convex-hull-source planner",
        "description": (
            "Convex-hull sources only with the normal direct planner limited to a "
            "5-degree target cone; selected-source value only."
        ),
        "path_mode": "material_aware",
        "candidate_value_mode": "source_only",
        "source_mode": "convex_hull",
        "visibility_angle": 5.0,
        "max_path_length_factor": 1.05,
        "policy": SOURCE_ONLY_POLICY,
    },
    "P2_wider_root_source_only": {
        "title": "Wider paths without multi-object task value",
        "description": (
            "Convex-hull sources only with a 30-degree target cone, while allocation "
            "still values only the selected source."
        ),
        "path_mode": "material_aware",
        "candidate_value_mode": "source_only",
        "source_mode": "convex_hull",
        "visibility_angle": 30.0,
        "max_path_length_factor": 2.50,
        "policy": SOURCE_ONLY_POLICY,
    },
    "P3_wider_root_material_aware": {
        "title": "Wider convex-hull-source material-aware planner",
        "description": (
            "The same 30-degree convex-hull-source planner, now valuing useful material "
            "along the complete swept path."
        ),
        "path_mode": "material_aware",
        "candidate_value_mode": "material_aware",
        "source_mode": "convex_hull",
        "visibility_angle": 30.0,
        "max_path_length_factor": 2.50,
        "policy": MATERIAL_AWARE_POLICY,
    },
    "P4_expanded_direct": {
        "title": "Expanded material-aware direct planner",
        "description": (
            "Target-only direct planning with a 30-degree cone, material-aware "
            "value, and both root and non-root feasible sources."
        ),
        "path_mode": "material_aware",
        "candidate_value_mode": "material_aware",
        "source_mode": "all",
        "visibility_angle": 30.0,
        "max_path_length_factor": 2.50,
        "policy": MATERIAL_AWARE_POLICY,
    },
    "P5_expanded_direct_60deg": {
        "title": "Wide material-aware direct planner",
        "description": (
            "The expanded target-only planner with a 60-degree cone to test "
            "whether additional freedom helps or creates inefficient paths."
        ),
        "path_mode": "material_aware",
        "candidate_value_mode": "material_aware",
        "source_mode": "all",
        "visibility_angle": 60.0,
        "max_path_length_factor": 2.50,
        "policy": MATERIAL_AWARE_POLICY,
    },
}

COMPARISON_SCENARIOS = (
    "baseline_circle_uniform",
    "rectangle_target",
    "amorphous_target",
)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run one controlled single-rover direct-target comparison."
    )
    parser.add_argument(
        "comparison", nargs="?", default=COMPARISON_NAME,
        choices=sorted(COMPARISON_PRESETS),
    )
    parser.add_argument(
        "--scenario", default=TEST_SCENARIO, choices=COMPARISON_SCENARIOS,
    )
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--pebbles", type=int, default=PEBBLE_COUNT)
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    parser.add_argument("--event-log-dir", default=None)
    parser.add_argument("--auto-exit-on-completion", action="store_true")
    parser.add_argument("--max-sim-time", type=float, default=None)
    parser.add_argument("--max-no-delivery-time", type=float, default=None)
    return parser.parse_args()


def _print_preset(name, scenario, preset, pebbles=PEBBLE_COUNT, seed=RANDOM_SEED):
    print(f"\n{name}: {preset['title']}")
    print(f"  {preset['description']}")
    print(
        "  Settings: "
        f"scenario={scenario}, source_mode={preset['source_mode']}, "
        f"path_mode={preset['path_mode']}, "
        f"candidate_value={preset['candidate_value_mode']}, "
        f"visibility={preset['visibility_angle']:.0f} deg, "
        f"pebbles={pebbles}, seed={seed}"
    )


def _list_presets():
    print("Direct-target comparison presets")
    print("Recommended order: " + " -> ".join(COMPARISON_PRESETS))
    print("Scenarios: " + ", ".join(COMPARISON_SCENARIOS))
    for name, preset in COMPARISON_PRESETS.items():
        _print_preset(name, TEST_SCENARIO, preset)


def _run(name, scenario, preset, args):
    import orchestrator_hybrid_multi_astar_scheduled as scheduled

    scheduled.TARGET_ZONE_CENTER = None
    scheduled.TARGET_VISIBILITY_MODE = "fixed"
    scheduled.TARGET_VISIBILITY_ANGLE = float(preset["visibility_angle"])
    scheduled.MAX_PATH_LENGTH_FACTOR = float(preset["max_path_length_factor"])
    scheduled.DIRECT_TARGET_PATH_MODE = preset["path_mode"]
    scheduled.TARGET_CANDIDATE_VALUE_MODE = preset["candidate_value_mode"]
    scheduled.TARGET_SOURCE_MODE = preset["source_mode"]
    scheduled.COMPARISON_LABEL = name
    scheduled.ROVER_POLICY_OVERRIDES = {
        "small": deepcopy(preset["policy"]),
    }

    orchestrator_args = [
        str(Path(scheduled.__file__).resolve()),
        "--scenario", scenario,
        "--rovers", "1",
        "--pebbles", str(args.pebbles),
        "--seed", str(args.seed),
        "--material-mode", "count",
        "--uniform-small-pebbles",
        "--rover-profiles", "small",
        "--target-path-mode", preset["path_mode"],
        "--target-candidate-value-mode", preset["candidate_value_mode"],
        "--comparison-label", name,
        "--target-source-mode", preset["source_mode"],
        "--no-congestion-reassignment",
        "--flow-field-vis", "never",
        "--no-draw-conflicts",
        "--hide-planning-boundaries",
        "--no-draw-execution-paths",
    ]
    if args.event_log_dir:
        orchestrator_args.extend(("--event-log-dir", str(Path(args.event_log_dir).resolve())))
    if args.auto_exit_on_completion:
        orchestrator_args.append("--auto-exit-on-completion")
    if args.max_sim_time is not None:
        orchestrator_args.extend(("--max-sim-time", str(args.max_sim_time)))
    if args.max_no_delivery_time is not None:
        orchestrator_args.extend((
            "--max-no-delivery-time", str(args.max_no_delivery_time)
        ))

    print("\n" + "=" * 76)
    print(f"RUNNING {name}: {preset['title']}")
    print(preset["description"])
    print("Highway tasks are disabled. Logs, push metrics, and images are enabled.")
    print("=" * 76 + "\n")

    old_argv = sys.argv
    try:
        sys.argv = orchestrator_args
        scheduled.main()
    finally:
        sys.argv = old_argv


def main():
    args = _parse_args()
    if args.list:
        _list_presets()
        return
    preset = COMPARISON_PRESETS[args.comparison]
    _print_preset(
        args.comparison, args.scenario, preset, args.pebbles, args.seed
    )
    if args.dry_run:
        return
    _run(args.comparison, args.scenario, preset, args)


if __name__ == "__main__":
    main()
