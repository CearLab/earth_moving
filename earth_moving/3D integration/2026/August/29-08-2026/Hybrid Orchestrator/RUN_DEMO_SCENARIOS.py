"""Click-to-run demonstrations for the multi-rover thesis simulation.

Normal orchestrator defaults are not modified.  To choose a demonstration in
AntiGravity, change only DEMO_NAME below, save, and click Play on this file.

Command-line helpers are also available:
    python RUN_DEMO_SCENARIOS.py --list
    python RUN_DEMO_SCENARIOS.py heterogeneous_circle --dry-run
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys


# ===================== CHANGE ONLY THIS LINE =====================
DEMO_NAME = "baseline_circle"
# ================================================================


CURRENT_POLICY = {
    "small": {
        "task_policy_mode": "dynamic",
        "good_location_potential_ratio": 0.65,
        "target_preference_enter_fraction": 0.80,
        "target_preference_exit_fraction": 0.70,
        "allow_highway_fallback_when_target_preferred": True,
        "allow_target_fallback_when_highway_preferred": True,
        "endgame_target_only_remaining_fraction": 0.20,
        "preferred_capacity_min_fraction": 0.35,
        "preferred_capacity_max_fraction": 1.00,
        "minimum_capacity_utilization": 0.00,
        "overcapacity_behavior": "deprioritize",
        "max_overcapacity_fraction": None,
        "capacity_fit_before_task_fallback": True,
    },
    "large": {
        "task_policy_mode": "target_only",
        "good_location_potential_ratio": 0.65,
        "preferred_capacity_min_fraction": 0.35,
        "preferred_capacity_max_fraction": 1.00,
        "minimum_capacity_utilization": 0.35,
        "overcapacity_behavior": "deprioritize",
        "max_overcapacity_fraction": None,
        "capacity_fit_before_task_fallback": True,
    },
}


DEMO_PRESETS = {
    "baseline_circle": {
        "title": "Homogeneous circular baseline",
        "description": (
            "Centered circular target, uniform small pebbles, three small "
            "rovers, and the same policy for the complete fleet."
        ),
        "watch": "Use as the reference for every later change.",
        "scenario": "baseline_circle_uniform",
        "rover_profiles": "small,small,small",
        "target_center": None,
        "target_visibility_angle": 45.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "heterogeneous_circle": {
        "title": "Heterogeneous rover cooperation",
        "description": (
            "Centered circle with mixed material and a small-large-small fleet. "
            "The large rover is target-focused and requires a meaningful load."
        ),
        "watch": (
            "Compare task type, load size, and contribution of the large rover "
            "with the two small rovers."
        ),
        "scenario": "heterogeneous_cooperation",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "off_center_circle": {
        "title": "Off-center circular target",
        "description": (
            "The target is translated to (1.25, 0.65), creating unequal travel "
            "distances and asymmetric work zones."
        ),
        "watch": "Look for unequal rover utilization and convergence near one side.",
        "scenario": "off_center_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "rectangle_target": {
        "title": "Rectangular target",
        "description": "Comparable-area rectangle with flat sides and sharp corners.",
        "watch": "Compare entry directions, corner behavior, exit, and spillage.",
        "scenario": "rectangle_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "ellipse_target": {
        "title": "Rotated elliptical target",
        "description": "A 25-degree rotated ellipse with a directional long axis.",
        "watch": "Look for preferred approach directions and boundary spillage.",
        "scenario": "ellipse_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "semicircle_target": {
        "title": "Semicircular target",
        "description": "An oriented target with one flat and one curved boundary.",
        "watch": "Compare behavior on the flat side with behavior on the curved side.",
        "scenario": "semicircle_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "thin_l_target": {
        "title": "Thin L-shaped target",
        "description": "A narrow non-convex target with a difficult inner corner.",
        "watch": "Look for approach feasibility and material near the concavity.",
        "scenario": "l_shape_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "concave_translated": {
        "title": "Translated thick concave target",
        "description": (
            "A thick L-shaped target translated to (1.25, 0.65), combining "
            "non-convex geometry with asymmetric traffic."
        ),
        "watch": "Look for target-exit recovery, competing approaches, and long waits.",
        "scenario": "concave_target",
        "rover_profiles": "small,large,small",
        "target_center": (1.25, 0.65),
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": True,
    },
    "amorphous_target": {
        "title": "Amorphous non-convex target",
        "description": "A fixed reproducible irregular polygon used as a geometry stress test.",
        "watch": "Look for uneven entry quality and material retained near irregular edges.",
        "scenario": "amorphous_target",
        "rover_profiles": "small,large,small",
        "target_center": None,
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "congestion_existing": {
        "title": "Congestion comparison: existing recovery only",
        "description": (
            "The translated concave scenario with congestion-aware task "
            "reassignment disabled."
        ),
        "watch": "Record prolonged waiting, repeated replans, and eventual recovery.",
        "scenario": "concave_target",
        "rover_profiles": "small,large,small",
        "target_center": (1.25, 0.65),
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": False,
    },
    "congestion_reassignment": {
        "title": "Congestion comparison: task reassignment enabled",
        "description": (
            "The identical translated concave scenario and seed, now allowing a "
            "persistently delayed rover to abandon the dense work zone."
        ),
        "watch": (
            "Compare waiting time, reassignment events, completion progress, "
            "and whether useful tasks are unnecessarily abandoned."
        ),
        "scenario": "concave_target",
        "rover_profiles": "small,large,small",
        "target_center": (1.25, 0.65),
        "target_visibility_angle": 30.0,
        "highway_visibility_angle": 60.0,
        "congestion_reassignment": True,
    },
}


RECOMMENDED_SHOWCASE = (
    "baseline_circle",
    "heterogeneous_circle",
    "concave_translated",
    "congestion_existing",
    "congestion_reassignment",
)


def _parse_launcher_args():
    parser = argparse.ArgumentParser(
        description="Select and run one prepared multi-rover demonstration."
    )
    parser.add_argument(
        "demo",
        nargs="?",
        default=DEMO_NAME,
        choices=sorted(DEMO_PRESETS),
        help="Demo preset; when omitted, DEMO_NAME at the top of this file is used.",
    )
    parser.add_argument("--list", action="store_true", help="List presets without running.")
    parser.add_argument(
        "--dry-run", action="store_true", help="Print resolved settings without running."
    )
    return parser.parse_args()


def _print_preset(name, preset):
    reassignment = "ON" if preset["congestion_reassignment"] else "OFF"
    print(f"\n{name}: {preset['title']}")
    print(f"  {preset['description']}")
    print(f"  Watch: {preset['watch']}")
    print(
        "  Settings: "
        f"scenario={preset['scenario']}, "
        f"profiles={preset['rover_profiles']}, "
        f"target_center={preset['target_center']}, "
        f"target_visibility={preset['target_visibility_angle']:.0f} deg, "
        f"congestion_reassignment={reassignment}"
    )


def _list_presets():
    print("Prepared simulation demonstrations")
    print("Recommended live sequence: " + " -> ".join(RECOMMENDED_SHOWCASE))
    for name, preset in DEMO_PRESETS.items():
        _print_preset(name, preset)


def _run_demo(name, preset):
    # Imported only for a real run so --list and --dry-run stay lightweight.
    import orchestrator_hybrid_multi_astar_scheduled as scheduled

    scheduled.TARGET_ZONE_CENTER = preset["target_center"]
    scheduled.TARGET_VISIBILITY_MODE = "fixed"
    scheduled.TARGET_VISIBILITY_ANGLE = float(preset["target_visibility_angle"])
    scheduled.HIGHWAY_VISIBILITY_MODE = "fixed"
    scheduled.HIGHWAY_VISIBILITY_ANGLE = float(preset["highway_visibility_angle"])
    scheduled.ROVER_POLICY_OVERRIDES = deepcopy(CURRENT_POLICY)

    orchestrator_args = [
        str(Path(scheduled.__file__).resolve()),
        "--scenario", preset["scenario"],
        "--rovers", "3",
        "--rover-profiles", preset["rover_profiles"],
        "--flow-field-vis", "never",
        "--no-draw-conflicts",
        "--hide-planning-boundaries",
        "--no-draw-execution-paths",
        "--congestion-reassignment"
        if preset["congestion_reassignment"]
        else "--no-congestion-reassignment",
    ]

    print("\n" + "=" * 72)
    print(f"RUNNING DEMO: {name} — {preset['title']}")
    print(preset["description"])
    print(f"WHAT TO WATCH: {preset['watch']}")
    print("Structured logs and benchmark images remain enabled.")
    print("=" * 72 + "\n")

    old_argv = sys.argv
    try:
        sys.argv = orchestrator_args
        scheduled.main()
    finally:
        sys.argv = old_argv


def main():
    args = _parse_launcher_args()
    if args.list:
        _list_presets()
        return

    preset = DEMO_PRESETS[args.demo]
    _print_preset(args.demo, preset)
    if args.dry_run:
        return
    _run_demo(args.demo, preset)


if __name__ == "__main__":
    main()
