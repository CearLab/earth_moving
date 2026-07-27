"""
orchestrator_hybrid_multi_safe.py

Multi-agent hybrid orchestrator with:
- one main PyBullet/control thread
- one background 2D-planning Future per rover
- a central winner/yielder collision-safety layer before wheel commands

This file intentionally reuses "Federico orchestrator_hybrid_multi.py" as the
baseline so the threaded planner and visualizer behavior stay familiar.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from multi_agent_collision_safety import (
    MultiAgentCollisionSafety,
    PolicyConfig,
    SafetyAgentContext,
)


def _load_federico_base():
    here = os.path.dirname(os.path.abspath(__file__))
    base_path = os.path.join(here, "Federico orchestrator_hybrid_multi.py")
    module_name = "_federico_orchestrator_hybrid_multi_base"
    if module_name in sys.modules:
        return sys.modules[module_name]

    if here not in sys.path:
        sys.path.insert(0, here)

    spec = importlib.util.spec_from_file_location(module_name, base_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load base orchestrator from {base_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


_base = _load_federico_base()

SIMULATION_CONFIG = _base.SIMULATION_CONFIG
SPILLAGE_CONFIG = _base.SPILLAGE_CONFIG
GATE_CONFIG = _base.GATE_CONFIG

ROVER_POSE_PRESETS = {
    2: [
        (2.0, 2.0, -math.pi / 2),
        (-2.0, -2.0, math.pi / 2),
    ],
    3: [
        (2.0, 2.0, -math.pi / 2),
        (-2.0, -2.0, math.pi / 2),
        (2.0, -2.0, math.pi),
    ],
    4: [
        (2.0, 2.0, -math.pi / 2),
        (-2.0, -2.0, math.pi / 2),
        (2.0, -2.0, math.pi),
        (-2.0, 2.0, 0.0),
    ],
}


def _path_reserved_cells(path_info: Dict[str, Any]) -> set:
    traj = path_info.get("path", [])
    required = {(c.x, c.y) for c in traj}
    required.update(path_info.get("impacted_cells", {}).keys())
    return required


def _pick_best_path_for_agent(env_2d, coord_converter, rover_pos_xy, reserved_global):
    """
    Match the single-agent selection policy, with reservation checks:
    1. target paths with positive delivered objects
    2. highway paths if no target-delivered path exists
    3. raw target fallback if spillage predicts zero delivered objects
    """
    best_cell = None
    best_score = -float("inf")
    best_path_info = None
    best_choice = "target"
    best_reserved = set()

    reserved_global = set(reserved_global or [])

    def consider(cell, choice, path_info, score_objects):
        nonlocal best_cell, best_score, best_path_info, best_choice, best_reserved
        traj = path_info.get("path", [])
        if not traj:
            return

        required_cells = _path_reserved_cells(path_info)
        if reserved_global and required_cells.intersection(reserved_global):
            return

        start_c = traj[0]
        start_wx, start_wy = coord_converter.convert_2d_to_3d(start_c.x, start_c.y)
        dist_to_start = math.hypot(rover_pos_xy[0] - start_wx, rover_pos_xy[1] - start_wy)
        score = score_objects - 0.5 * dist_to_start

        if score > best_score:
            best_score = score
            best_cell = cell
            best_path_info = path_info
            best_choice = choice
            best_reserved = required_cells

    for cell in getattr(env_2d, "cells_with_objects", []):
        paths = env_2d.get_path_for_preview(cell, "target")
        if not paths:
            continue
        path_info = paths[0]
        delivered_objects = path_info.get("objects", 0)
        if delivered_objects <= 0:
            continue
        consider(cell, "target", path_info, delivered_objects)

    if best_cell is None:
        for cell in getattr(env_2d, "cells_with_objects", []):
            paths = env_2d.get_path_for_preview(cell, "highway")
            if not paths:
                continue
            path_info = paths[0]
            highway_objects = path_info.get("objects", 0)
            if highway_objects <= 0:
                continue
            consider(cell, "highway", path_info, highway_objects)

    if best_cell is None:
        for cell in getattr(env_2d, "cells_with_objects", []):
            paths = env_2d.get_path_for_preview(cell, "target")
            if not paths:
                continue
            path_info = paths[0]
            raw_objects = path_info.get("raw_objects", getattr(cell, "num_objects", 0))
            if raw_objects <= 0:
                continue
            consider(cell, "target", path_info, raw_objects)

    return best_cell, best_choice, best_path_info, best_reserved


def plan_for_agent(snapshot: Any):
    try:
        pebbles_3d = [(float(x), float(y), 0.01) for x, y in snapshot.pebbles_xy]
        env_2d = _base.compute_2d_env(
            snapshot.env_radius,
            snapshot.target_zone_radius,
            snapshot.shovel_width,
            pebbles_3d,
            manual_mode=False,
            use_spillage_model=SIMULATION_CONFIG["use_spillage_model"],
            visualize_potential=SIMULATION_CONFIG["visualize_potential"],
        )

        coord_converter = _base.CoordinateConverter(
            snapshot.env_radius,
            snapshot.target_zone_radius,
            snapshot.shovel_width,
        )
        reserved = set(snapshot.global_reserved_cells) if snapshot.global_reserved_cells else set()
        best_cell, best_choice, best_path_info, best_reserved = _pick_best_path_for_agent(
            env_2d,
            coord_converter,
            snapshot.agent_pose_xy,
            reserved,
        )

        return _base.PlanResult(
            agent_id=snapshot.agent_id,
            plan_id=snapshot.plan_id,
            env_radius=snapshot.env_radius,
            env_2d=env_2d,
            best_cell=best_cell,
            best_choice=best_choice,
            best_path_info=best_path_info,
            best_reserved=best_reserved,
            error=None,
        )
    except Exception as exc:
        return _base.PlanResult(
            agent_id=snapshot.agent_id,
            plan_id=snapshot.plan_id,
            env_radius=snapshot.env_radius,
            env_2d=None,
            best_cell=None,
            best_choice="target",
            best_path_info=None,
            best_reserved=set(),
            error=str(exc),
        )


class SafeMultiAgentHybridOrchestrator(_base.MultiAgentHybridOrchestrator):
    def __init__(self, *args, safety_config: Optional[PolicyConfig] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.safety = MultiAgentCollisionSafety(safety_config)
        self._sim_time = 0.0
        self._last_safety_status_print = -10.0

    def _start_planning_for_agent(self, agent, pebbles_xy=None, env_r=None):
        fut = agent.get("future", None)
        if fut is not None and not fut.done():
            return

        if pebbles_xy is None:
            pebbles_xy = self._snapshot_pebbles_xy()
        else:
            pebbles_xy = np.array(pebbles_xy, dtype=float, copy=True)

        if env_r is None:
            env_r = self.calculate_dynamic_env_radius(self._get_live_pebble_centers())

        agent["plan_id"] += 1
        pid = agent["plan_id"]

        st = _base.get_state(agent["body_id"])
        rover_xy = (float(st[0]), float(st[1]))

        with self._reservation_lock:
            reserved = tuple(self.global_reserved_cells)

        snap = _base.WorldSnapshot(
            env_radius=env_r,
            target_zone_radius=self.target_zone_radius,
            shovel_width=self.shovel_width,
            agent_id=agent["id"],
            agent_pose_xy=rover_xy,
            pebbles_xy=pebbles_xy,
            global_reserved_cells=reserved,
            plan_id=pid,
        )

        agent["state"] = "PLANNING"
        agent["future"] = self.executor.submit(plan_for_agent, snap)
        print(f"[PLAN] Submitted safe plan for {agent['id']} (plan_id={pid})")

    def _navigator_mode_name(self, agent) -> str:
        nav = agent.get("navigator")
        if nav is None:
            return "IDLE"
        try:
            mode = nav.get_mode()
        except Exception:
            return "IDLE"
        return getattr(mode, "name", str(mode))

    def _agent_path_points(self, agent) -> Tuple[Tuple[float, float], ...]:
        nav = agent.get("navigator")
        current_path = getattr(nav, "current_path", None) if nav is not None else None
        if current_path:
            return tuple((float(x), float(y)) for x, y in current_path)
        selection = agent.get("selection")
        if selection and selection.get("world_pts"):
            return tuple((float(x), float(y)) for x, y in selection["world_pts"])
        return ()

    def _agent_goal(self, agent, path_points: Sequence[Tuple[float, float]], path_constrained: bool):
        if agent.get("state") == "ROLLBACK":
            state = agent.get("state_val")
            if state is not None:
                x, y, yaw = float(state[0]), float(state[1]), float(state[2])
                return x - 0.50 * math.cos(yaw), y - 0.50 * math.sin(yaw)

        if path_points:
            if path_constrained:
                return path_points[-1]
            return path_points[0]

        selection = agent.get("selection")
        if selection and selection.get("gate"):
            gx, gy = selection["gate"]
            return float(gx), float(gy)
        return None

    def _build_safety_contexts(self):
        contexts = []
        for agent in self.agents:
            idx = int(agent["index"])
            state = agent.get("state_val")
            if state is None:
                state = _base.get_state(agent["body_id"])
            mode_name = self._navigator_mode_name(agent)
            active = agent.get("state") in ("NAVIGATING", "ROLLBACK")
            path_constrained = active and mode_name == "PATH_TRACK"
            path_points = self._agent_path_points(agent) if active else ()
            goal = self._agent_goal(agent, path_points, path_constrained) if active else None
            contexts.append(
                SafetyAgentContext(
                    idx=idx,
                    agent_id=agent["id"],
                    state=np.array(state, dtype=float, copy=True),
                    active=active,
                    path_constrained=path_constrained,
                    path_points=path_points,
                    goal=goal,
                    priority=float(agent.get("priority", 0.5)),
                )
            )

        contexts = self.safety.with_priorities(contexts)
        by_idx = {ctx.idx: ctx for ctx in contexts}
        for agent in self.agents:
            agent["priority"] = by_idx[int(agent["index"])].priority
        return contexts

    def _orca_others_for_agent(self, agent, contexts):
        ego_idx = int(agent["index"])
        by_idx = {ctx.idx: ctx for ctx in contexts}
        others = []
        for other in self.agents:
            other_idx = int(other["index"])
            if other_idx == ego_idx:
                continue
            other_ctx = by_idx[other_idx]
            state = other.get("state_val")
            if state is None:
                state = _base.get_state(other["body_id"])
            others.append({
                "id": other["id"],
                "state": np.array(state, dtype=float, copy=True),
                "priority": float(other_ctx.priority),
                "is_blind": not other_ctx.active,
                "force_avoid": self.safety.should_force_avoid(ego_idx, other_idx),
                "shape": {"circles": [(0.0, 0.0, 0.35)]},
            })
        return others

    def _rollback_command(self, agent, dt):
        agent["rollback_timer"] -= dt
        if agent["rollback_timer"] > 0:
            return -0.5, 0.0
        agent["state"] = "SYNC"
        return 0.0, 0.0

    def _print_safety_status(self, contexts, controls):
        episode = self.safety.episode
        if not episode.active or self._sim_time - self._last_safety_status_print < 1.0:
            return

        self._last_safety_status_print = self._sim_time
        agent_by_idx = {int(agent["index"]): agent for agent in self.agents}
        parts = []
        for ctx in contexts:
            agent = agent_by_idx.get(ctx.idx)
            mode_name = self._navigator_mode_name(agent) if agent is not None else "IDLE"
            behavior = self.safety.behavior_for(ctx.idx, contexts)
            v_cmd, w_cmd = controls.get(ctx.idx, (0.0, 0.0))
            role = "winner" if ctx.idx == episode.winner_idx else (
                "yielder" if ctx.idx in episode.yielder_indices else "free"
            )
            parts.append(
                f"{ctx.agent_id}:{role}/{mode_name}/{behavior.value} "
                f"v=({v_cmd:.2f},{w_cmd:.2f})"
            )
        print(f"[SAFETY] phase={episode.phase.value} | " + " | ".join(parts))

    def _step_agents(self, dt):
        """
        Tick all agents, then run the central collision safety layer before
        any wheel commands are applied.
        """
        self._sim_time += dt
        self._snapshot_pebbles_xy()
        live_pebbles = self._get_live_pebble_centers()

        for agent in self.agents:
            agent["state_val"] = _base.get_state(agent["body_id"])

        contexts = self._build_safety_contexts()
        self.safety.update(contexts, self._sim_time)

        nominal_controls: Dict[int, Tuple[float, float]] = {}
        for agent in self.agents:
            idx = int(agent["index"])
            state_name = agent["state"]

            if state_name in ("IDLE", "PLANNING"):
                nominal_controls[idx] = (0.0, 0.0)
                continue

            if state_name == "NAVIGATING":
                nav_agent_dict = {
                    "id": agent["id"],
                    "state": agent["state_val"],
                    "priority": float(agent.get("priority", 0.5)),
                    "shape": {"circles": [(0.0, 0.0, 0.35)]},
                }
                agent["navigator"].set_agent(nav_agent_dict)
                agent["navigator"].update_pebbles(live_pebbles)
                status = agent["navigator"].step(dt, self._orca_others_for_agent(agent, contexts))
                nominal_controls[idx] = (status.v_cmd, status.w_cmd)

                if status.completed:
                    print(f"  Agent {agent['id']} path complete. Rolling back.")
                    agent["state"] = "ROLLBACK"
                    agent["rollback_timer"] = 0.5
                    nominal_controls[idx] = (0.0, 0.0)
                continue

            if state_name == "ROLLBACK":
                nominal_controls[idx] = self._rollback_command(agent, dt)
                continue

            if state_name == "SYNC":
                nominal_controls[idx] = (0.0, 0.0)
                self._step_sync(agent)
                continue

            nominal_controls[idx] = (0.0, 0.0)

        final_controls = self.safety.filter_controls(contexts, nominal_controls)
        self._print_safety_status(contexts, final_controls)
        for agent in self.agents:
            idx = int(agent["index"])
            v_cmd, w_cmd = final_controls.get(idx, (0.0, 0.0))
            _base.set_wheel_velocities(
                agent["body_id"],
                agent["left_joint"],
                agent["right_joint"],
                v_cmd,
                w_cmd,
            )


MultiAgentHybridOrchestrator = SafeMultiAgentHybridOrchestrator


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run the safe multi-agent hybrid orchestrator."
    )
    parser.add_argument(
        "--rovers",
        type=int,
        choices=sorted(ROVER_POSE_PRESETS),
        default=3,
        help="Number of rovers to spawn. Use 3 to add one rover, 4 to add two rovers.",
    )
    parser.add_argument(
        "--pebbles",
        type=int,
        default=50,
        help="Number of pebbles to spawn.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=41,
        help="Random seed for pebble placement.",
    )
    parser.add_argument(
        "--flow-field-vis",
        choices=("never", "ask", "always"),
        default="never",
        help="3D flow-field visualization mode.",
    )
    parser.add_argument(
        "--phase1-tracking-point",
        choices=("base", "shovel"),
        default="shovel",
        help="Tracking point used while navigating to the path start.",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    use_spillage_in_2d = True
    show_spillage_plot = False
    flow_field_vis_3d = args.flow_field_vis
    auto_mode = True
    phase1_tracking_point = args.phase1_tracking_point
    initial_robot_poses = ROVER_POSE_PRESETS[args.rovers]

    SIMULATION_CONFIG["use_spillage_model"] = use_spillage_in_2d
    SIMULATION_CONFIG["visualize_potential"] = show_spillage_plot

    print(
        "Config: "
        f"Rovers={args.rovers}, "
        f"Spillage2D={use_spillage_in_2d}, "
        f"ShowPlot={show_spillage_plot}, "
        f"3D_Vis={flow_field_vis_3d}, "
        f"Auto={auto_mode}, "
        f"Phase1Track={phase1_tracking_point}, "
        "Safety=central-winner-yielder"
    )

    orch = SafeMultiAgentHybridOrchestrator(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=args.pebbles,
        random_seed=args.seed,
        initial_robot_poses=initial_robot_poses,
        shovel_width=0.22,
        auto_mode=auto_mode,
        phase1_tracking_point=phase1_tracking_point,
    )
    orch.flow_field_vis_mode = flow_field_vis_3d
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()
