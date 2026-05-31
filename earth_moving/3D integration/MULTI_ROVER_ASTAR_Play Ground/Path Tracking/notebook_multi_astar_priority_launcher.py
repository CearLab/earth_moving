"""
Notebook launcher for multi_astar_priority_scheduling3.py.

This file exists so the scheduler is imported with its normal module name.
That is important on Windows because ProcessPoolExecutor workers need to import
functions such as multi_astar_priority_scheduling3.planner_warmup.
"""

import argparse
import multiprocessing
import sys

import multi_astar_priority_scheduling3 as scheduler


def parse_launcher_args():
    parser = argparse.ArgumentParser(
        description="Launch multi_astar_priority_scheduling3.py from the playground notebook."
    )
    parser.add_argument("--scenario", default=scheduler.SCENARIO_NAME)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-time", type=float, default=90.0)
    parser.add_argument("--no-draw", action="store_true")
    parser.add_argument(
        "--pebbles",
        choices=("scenario", "none", "random"),
        default=scheduler.PEBBLE_MODE,
    )
    parser.add_argument("--num-pebbles", type=int, default=scheduler.NUM_PEBBLES)
    parser.add_argument("--pebble-seed", type=int, default=scheduler.PEBBLE_SEED)
    parser.add_argument("--v-max", type=float, default=scheduler.V_MAX)
    parser.add_argument("--w-max", type=float, default=scheduler.W_MAX)
    parser.add_argument("--max-wheel-speed", type=float, default=scheduler.MAX_WHEEL_SPEED)
    parser.add_argument("--max-torque", type=float, default=scheduler.MAX_TORQUE)
    parser.add_argument("--replan-workers", type=int, default=scheduler.REPLAN_WORKERS)
    parser.add_argument(
        "--run-cell-time-calibration",
        dest="run_cell_time_calibration",
        action="store_true",
    )
    parser.add_argument(
        "--skip-cell-time-calibration",
        dest="run_cell_time_calibration",
        action="store_false",
    )
    parser.set_defaults(run_cell_time_calibration=scheduler.RUN_CELL_TIME_CALIBRATION)
    return parser.parse_args()


def main():
    args = parse_launcher_args()

    scheduler.V_MAX = float(args.v_max)
    scheduler.W_MAX = float(args.w_max)
    scheduler.MAX_WHEEL_SPEED = float(args.max_wheel_speed)
    scheduler.MAX_TORQUE = float(args.max_torque)
    scheduler.REPLAN_WORKERS = int(args.replan_workers)
    scheduler.RUN_CELL_TIME_CALIBRATION = bool(args.run_cell_time_calibration)

    scheduler_argv = [
        "multi_astar_priority_scheduling3.py",
        "--scenario",
        args.scenario,
        "--max-time",
        str(args.max_time),
        "--pebbles",
        args.pebbles,
        "--num-pebbles",
        str(args.num_pebbles),
        "--pebble-seed",
        str(args.pebble_seed),
    ]
    if args.headless:
        scheduler_argv.append("--headless")
    if args.no_draw:
        scheduler_argv.append("--no-draw")

    sys.argv = scheduler_argv

    print("[notebook launcher] using real module:", scheduler.__file__)
    print("[notebook launcher] scheduler argv:", " ".join(scheduler_argv))
    print(
        "[notebook launcher] controls: "
        f"V_MAX={scheduler.V_MAX}, W_MAX={scheduler.W_MAX}, "
        f"wheel={scheduler.MAX_WHEEL_SPEED}, torque={scheduler.MAX_TORQUE}, "
        f"workers={scheduler.REPLAN_WORKERS}, "
        f"cell_time_calibration={scheduler.RUN_CELL_TIME_CALIBRATION}"
    )
    scheduler.main()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
