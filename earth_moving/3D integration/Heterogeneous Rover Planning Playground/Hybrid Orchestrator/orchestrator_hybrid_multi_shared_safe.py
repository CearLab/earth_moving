"""
orchestrator_hybrid_multi_shared_safe.py

New multi-agent runner with:
- one always-running PyBullet/control thread
- one single-worker shared 2D map builder
- lightweight per-agent path allocation from the latest shared 2D snapshot
- deadlock-aware winner/yielder safety near the target zone

This intentionally leaves the older Federico/safe files untouched.
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import math
import time
from typing import Optional, Tuple

import numpy as np

import orchestrator_hybrid_multi_safe as _safe
from multi_agent_deadlock_safety import (
    DeadlockAwareCollisionSafety,
    DeadlockPolicyConfig,
)
from shared_path_allocator import (
    PathAllocationRequest,
    PathAllocationResult,
    SharedMapBuildRequest,
    SharedMapSnapshot,
    allocate_path_from_shared_map,
    build_shared_map_snapshot,
)


_base = _safe._base
SIMULATION_CONFIG = _safe.SIMULATION_CONFIG
ROVER_POSE_PRESETS = _safe.ROVER_POSE_PRESETS


def build_shared_map_job(request: SharedMapBuildRequest) -> SharedMapSnapshot:
    return build_shared_map_snapshot(
        request,
        _base.compute_2d_env,
        _base.CoordinateConverter,
    )


def allocate_path_job(
    snapshot: SharedMapSnapshot,
    request: PathAllocationRequest,
) -> PathAllocationResult:
    return allocate_path_from_shared_map(snapshot, request)


class SharedPlanningDeadlockOrchestrator(_safe.SafeMultiAgentHybridOrchestrator):
    def __init__(
        self,
        *args,
        map_update_interval: float = 12.0,
        target_root_sources_only: Optional[bool] = None,
        deadlock_safety_config: Optional[DeadlockPolicyConfig] = None,
        **kwargs,
    ):
        super().__init__(*args, safety_config=None, **kwargs)
        safety_config = deadlock_safety_config or DeadlockPolicyConfig(
            target_zone_radius=self.target_zone_radius,
            target_zone=self.target_zone,
        )
        self.safety = DeadlockAwareCollisionSafety(safety_config)

        self.map_update_interval = float(map_update_interval)
        self.target_root_sources_only = (
            None if target_root_sources_only is None
            else bool(target_root_sources_only)
        )
        self.map_executor = None
        self.shared_map = None
        self.shared_map_future = None
        self.shared_map_epoch = 0
        self.last_shared_map_request_time = 0.0

        self.reserved_object_cells = set()
        self.consumed_object_cells = set()

    def _allocation_event(self, event_type, agent, **data):
        callback = getattr(self, "_log_event", None)
        if callable(callback):
            callback(event_type, agent, **data)

    def initialize(self):
        super().initialize()
        self.map_executor = cf.ThreadPoolExecutor(max_workers=1)
        initial_instances = self._get_live_pebble_instances()
        self.shared_map = SharedMapSnapshot(
            epoch=0,
            env_radius=self.env_radius,
            env_2d=self.env_2d,
            coord_converter=self.coord_converter,
            built_wall_time=time.time(),
            pebbles_count=len(self.pebble_centers),
            pebbles_material_mass=sum(instance.material_mass for instance in initial_instances),
            material_value_mode=self.material_value_mode,
        )
        print(
            f"[MAP] Initial shared 2D snapshot ready "
            f"(epoch=0, objects={self.shared_map.pebbles_count}, "
            f"mass={self.shared_map.pebbles_material_mass:.0f}, mode={self.shared_map.material_value_mode})"
        )

    def run(self):
        try:
            super().run()
        finally:
            if self.map_executor is not None:
                self.map_executor.shutdown(wait=False, cancel_futures=True)

    def _poll_shared_map_result(self):
        fut = self.shared_map_future
        if fut is None or not fut.done():
            return

        self.shared_map_future = None
        try:
            snapshot = fut.result()
        except Exception as exc:
            print(f"[MAP] Shared 2D map build failed: {exc}")
            return

        self.shared_map = snapshot
        self.env_2d = snapshot.env_2d
        self.coord_converter = snapshot.coord_converter
        with self._reservation_lock:
            self.consumed_object_cells.clear()

        try:
            self.visualizer.update_env(snapshot.env_2d)
            self._needs_redraw = True
        except Exception:
            pass

        print(
            f"[MAP] Shared 2D map ready "
            f"(epoch={snapshot.epoch}, objects={snapshot.pebbles_count}, "
            f"mass={snapshot.pebbles_material_mass:.0f}, mode={snapshot.material_value_mode})"
        )

    def _maybe_start_shared_map_build(self, force: bool = False):
        self._poll_shared_map_result()
        if self.map_executor is None:
            return
        if self.shared_map_future is not None and not self.shared_map_future.done():
            return

        now = time.time()
        last_built = self.shared_map.built_wall_time if self.shared_map is not None else 0.0
        if not force and self.shared_map is not None and now - last_built < self.map_update_interval:
            return

        pebbles_xy = self._snapshot_pebbles_xy()
        live_instances = self._get_live_pebble_instances()
        live_pebbles = self._get_live_pebble_centers()
        env_r = self.calculate_dynamic_env_radius(live_pebbles)
        self.shared_map_epoch += 1
        req = SharedMapBuildRequest(
            epoch=self.shared_map_epoch,
            env_radius=env_r,
            target_zone_radius=self.target_zone_radius,
            shovel_width=self.shovel_width,
            pebbles_xy=tuple((float(x), float(y)) for x, y in pebbles_xy),
            use_spillage_model=SIMULATION_CONFIG["use_spillage_model"],
            visualize_potential=SIMULATION_CONFIG["visualize_potential"],
            pebble_materials=tuple(live_instances),
            material_value_mode=self.material_value_mode,
            target_zone=self.target_zone,
        )
        self.last_shared_map_request_time = now
        self.shared_map_future = self.map_executor.submit(build_shared_map_job, req)
        print(f"[MAP] Submitted shared 2D map build (epoch={req.epoch})")

    def _reservation_snapshot(self):
        with self._reservation_lock:
            return (
                tuple(self.global_reserved_cells),
                tuple(self.reserved_object_cells),
                tuple(self.consumed_object_cells),
            )

    def _start_planning_for_agent(self, agent, pebbles_xy=None, env_r=None):
        fut = agent.get("future", None)
        if fut is not None and not fut.done():
            return

        self._poll_shared_map_result()
        if self.shared_map is None:
            self._maybe_start_shared_map_build(force=True)
            return

        agent["plan_id"] += 1
        pid = agent["plan_id"]
        st = _base.get_state(agent["body_id"])
        rover_xy = (float(st[0]), float(st[1]))
        reserved_path, reserved_objects, consumed_objects = self._reservation_snapshot()
        profile = agent.get("rover_profile")

        req = PathAllocationRequest(
            agent_id=agent["id"],
            plan_id=pid,
            map_epoch=self.shared_map.epoch,
            agent_pose_xy=rover_xy,
            reserved_path_cells=reserved_path,
            reserved_object_cells=reserved_objects,
            consumed_object_cells=consumed_objects,
            rover_type=profile,
            shovel_width=(float(profile.shovel_width) if profile is not None else self.shovel_width),
            overlay_cell_size=(float(profile.overlay_cell_size) if profile is not None else 0.0),
            reservation_radius=(float(profile.reservation_radius) if profile is not None else self.shovel_width / 2.0 + 0.03),
            target_root_sources_only=self.target_root_sources_only,
        )
        agent["state"] = "PLANNING"
        agent["future"] = self.executor.submit(allocate_path_job, self.shared_map, req)
        policy_label = ",".join(sorted(profile.policy.allowed_tasks)) if profile is not None else "legacy"
        preference_label = (
            ">".join(profile.policy.task_fallback_order)
            if profile is not None and profile.policy.task_fallback_order else "score"
        )
        root_only = (
            bool(profile.policy.target_root_sources_only)
            if self.target_root_sources_only is None and profile is not None
            else bool(self.target_root_sources_only)
        )
        capacity = (
            profile.capacity_mass if profile is not None and self.material_value_mode == "mass"
            else profile.capacity_objects if profile is not None else "legacy"
        )
        self._allocation_event(
            "ALLOCATION_SUBMITTED", agent,
            plan_id=pid, map_epoch=req.map_epoch,
            agent_pose=rover_xy, overlay_cell_size=req.overlay_cell_size,
            capacity=capacity, allowed_tasks=policy_label,
            preference=preference_label,
            target_root_sources_only=root_only,
            reserved_path_cells=len(reserved_path),
            reserved_object_cells=len(reserved_objects),
            consumed_object_cells=len(consumed_objects),
        )
        print(
            f"[ALLOC] Submitted path allocation for {agent['id']} "
            f"(plan_id={pid}, map_epoch={req.map_epoch}, overlay={req.overlay_cell_size:.3f}m, "
            f"capacity={capacity} {self.material_value_mode}, policy={policy_label}, "
            f"preference={preference_label}, target_root_only={root_only})"
        )

    def _poll_planning_results(self):
        self._poll_shared_map_result()

        for agent in self.agents:
            fut = agent.get("future", None)
            if fut is None or not fut.done():
                continue

            try:
                res = fut.result()
            except Exception as exc:
                self._allocation_event("ALLOCATION_FAILED", agent, reason="worker_crash", error=repr(exc))
                print(f"[ALLOC] {agent['id']} allocation crashed: {exc}")
                agent["future"] = None
                agent["state"] = "IDLE"
                continue

            agent["future"] = None

            if res.plan_id != agent["plan_id"]:
                continue

            if res.error is not None:
                self._allocation_event("ALLOCATION_FAILED", agent, reason="allocator_error", error=res.error)
                print(f"[ALLOC] {agent['id']} allocation error: {res.error}")
                agent["state"] = "IDLE"
                continue

            if self.shared_map is None or res.map_epoch != self.shared_map.epoch:
                self._allocation_event(
                    "ALLOCATION_RETRY", agent, reason="stale_map",
                    result_map_epoch=res.map_epoch,
                    current_map_epoch=self.shared_map.epoch if self.shared_map is not None else None,
                )
                print(f"[ALLOC] {agent['id']} stale map result; retrying on current map.")
                agent["state"] = "IDLE"
                continue

            if res.best_cell is None or res.best_path_info is None:
                self._allocation_event(
                    "ALLOCATION_NO_VALID_TASK", agent,
                    plan_id=res.plan_id, map_epoch=res.map_epoch,
                )
                agent["state"] = "IDLE"
                continue

            with self._reservation_lock:
                active_or_consumed_objects = set(self.reserved_object_cells)
                active_or_consumed_objects.update(self.consumed_object_cells)

                if res.reserved_path_cells.intersection(self.global_reserved_cells):
                    self._allocation_event(
                        "ALLOCATION_RETRY", agent, reason="path_cells_reserved",
                        conflicting_cells=res.reserved_path_cells.intersection(self.global_reserved_cells),
                    )
                    print(f"[ALLOC] {agent['id']} path cells already reserved; retrying.")
                    agent["state"] = "IDLE"
                    continue
                if res.reserved_object_cells.intersection(active_or_consumed_objects):
                    self._allocation_event(
                        "ALLOCATION_RETRY", agent, reason="object_cells_reserved",
                        conflicting_cells=res.reserved_object_cells.intersection(active_or_consumed_objects),
                    )
                    print(f"[ALLOC] {agent['id']} object cells already claimed; retrying.")
                    agent["state"] = "IDLE"
                    continue

                self.global_reserved_cells.update(res.reserved_path_cells)
                self.reserved_object_cells.update(res.reserved_object_cells)
                agent["reserved_cells"] = set(res.reserved_path_cells)
                agent["reserved_object_cells"] = set(res.reserved_object_cells)

            agent["env_2d"] = res.env_2d
            agent["coord_converter"] = res.coord_converter

            try:
                self.visualizer.update_env(res.env_2d)
                self.coord_converter = res.coord_converter
            except Exception:
                pass

            try:
                self._setup_agent_selection(
                    agent,
                    cell=res.best_cell,
                    choice=res.best_choice,
                    path_info=res.best_path_info,
                )
            except Exception as exc:
                with self._reservation_lock:
                    self.global_reserved_cells.difference_update(res.reserved_path_cells)
                    self.reserved_object_cells.difference_update(res.reserved_object_cells)
                agent["reserved_cells"] = set()
                agent["reserved_object_cells"] = set()
                agent["selection"] = None
                agent["state"] = "IDLE"
                self._allocation_event(
                    "ALLOCATION_PATH_REJECTED", agent,
                    plan_id=res.plan_id, map_epoch=res.map_epoch,
                    task_type=res.best_choice, error=repr(exc),
                    path_info=res.best_path_info,
                )
                print(f"[PATH-GUARD] {agent['id']} rejected task path: {exc}")
                continue
            agent["state"] = "NAVIGATING"
            self._allocation_event(
                "ALLOCATION_ACCEPTED", agent,
                plan_id=res.plan_id, map_epoch=res.map_epoch,
                task_type=res.best_choice,
                source_cell=(res.best_cell.x, res.best_cell.y),
                is_root_source=res.best_path_info.get("is_root_source"),
                target_root_sources_only=res.best_path_info.get("policy_target_root_sources_only"),
                reserved_path_cells=len(res.reserved_path_cells),
                reserved_object_cells=len(res.reserved_object_cells),
            )
            self._needs_redraw = True

            print(
                f"[ALLOC] {agent['id']} -> NAVIGATING "
                f"(task={res.best_choice}, cell=({res.best_cell.x},{res.best_cell.y}), "
                f"source_cells={len(res.best_path_info.get('source_canonical_cells', []))}, "
                f"root_source={res.best_path_info.get('is_root_source', 'n/a')}, "
                f"expected={res.best_path_info.get('expected_collected', 0.0):.1f}/"
                f"{res.best_path_info.get('capacity_quantity', 0.0):.0f} "
                f"{res.best_path_info.get('material_value_mode', 'count')}, "
                f"objects={res.best_path_info.get('expected_collected_objects', 0.0):.1f}, "
                f"mass={res.best_path_info.get('expected_collected_mass', 0.0):.1f}, "
                f"path_cells={len(res.reserved_path_cells)}, overlay_cells={len(res.best_path_info.get('overlay_cells', []))}, "
                f"object_cells={len(res.reserved_object_cells)})"
            )

    def _handle_auto_mode(self):
        self._maybe_start_shared_map_build(force=False)
        for agent in self.agents:
            if agent["state"] == "IDLE":
                self._start_planning_for_agent(agent)

    def _step_sync(self, agent):
        print(f"\n[SYNC] Agent {agent['id']} reached target. Going IDLE.")

        reserved_path_cells = set(agent.get("reserved_cells", set()))
        reserved_object_cells = set(agent.get("reserved_object_cells", set()))
        with self._reservation_lock:
            self.global_reserved_cells.difference_update(reserved_path_cells)
            self.reserved_object_cells.difference_update(reserved_object_cells)
            self.consumed_object_cells.update(reserved_object_cells)

        agent["reserved_cells"] = set()
        agent["reserved_object_cells"] = set()
        agent["state"] = "IDLE"
        agent["selection"] = None
        self._needs_redraw = True

        if reserved_object_cells:
            print(
                f"[ALLOC] {agent['id']} consumed {len(reserved_object_cells)} "
                "object cells until next shared map refresh."
            )


MultiAgentHybridOrchestrator = SharedPlanningDeadlockOrchestrator


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run shared-planning deadlock-safe multi-agent orchestrator."
    )
    parser.add_argument(
        "--rovers",
        type=int,
        choices=sorted(ROVER_POSE_PRESETS),
        default=4,
        help="Number of rovers to spawn.",
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
        "--map-interval",
        type=float,
        default=12.0,
        help="Seconds between shared 2D map rebuilds.",
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
    auto_mode = True
    initial_robot_poses = ROVER_POSE_PRESETS[args.rovers]

    SIMULATION_CONFIG["use_spillage_model"] = use_spillage_in_2d
    SIMULATION_CONFIG["visualize_potential"] = show_spillage_plot

    print(
        "Config: "
        f"Rovers={args.rovers}, "
        f"SharedMapInterval={args.map_interval:.1f}s, "
        f"Spillage2D={use_spillage_in_2d}, "
        f"ShowPlot={show_spillage_plot}, "
        f"3D_Vis={args.flow_field_vis}, "
        f"Auto={auto_mode}, "
        f"Phase1Track={args.phase1_tracking_point}, "
        "Safety=deadlock-aware"
    )

    orch = SharedPlanningDeadlockOrchestrator(
        env_radius=3.0,
        target_zone_radius=0.8,
        num_pebbles=args.pebbles,
        random_seed=args.seed,
        initial_robot_poses=initial_robot_poses,
        shovel_width=0.22,
        auto_mode=auto_mode,
        phase1_tracking_point=args.phase1_tracking_point,
        map_update_interval=args.map_interval,
    )
    orch.flow_field_vis_mode = args.flow_field_vis
    orch.initialize()
    orch.run()


if __name__ == "__main__":
    main()









