import math
import unittest

import numpy as np

from fleet_deadlock_recovery import (
    EscapeRoute,
    FleetDeadlockConfig,
    FleetDeadlockRecoveryManager,
)
from multi_agent_collision_safety import SafetyAgentContext
from orchestrator_hybrid_multi_astar_scheduled import (
    ASTAR_APPROACH_CONFIG,
    MultiAStarScheduledHybridOrchestrator,
    TargetKeepoutBlocked,
    _build_hybrid_astar_schedule_agent,
)
from rover_profiles import ROVER_PROFILES
from scenario_configs import get_scenario


class ExecutionRecoveryRegressionTests(unittest.TestCase):
    def _orchestrator_shell(self):
        orch = object.__new__(MultiAStarScheduledHybridOrchestrator)
        orch.target_zone = get_scenario("concave_target").with_target_center(
            (1.25, 0.65)
        ).target_zone
        orch.phase1_tracking_point = "shovel"
        orch.shovel_width = 0.22
        orch._sim_time = 10.0
        orch.parking_retry_interval = 1.5
        return orch

    def test_target_exit_requires_shovel_to_clear_not_only_base(self):
        orch = self._orchestrator_shell()
        profile = ROVER_PROFILES["small"]
        # Exact terminal pose from the failed run: the base is above the top
        # edge, but the south-facing shovel still overlaps the target polygon.
        state = np.array([1.16906895, 1.47703774, -math.pi / 2.0, 0.0, 0.0])
        agent = {"rover_profile": profile, "state_val": state}

        base_clearance, shovel_clearances = (
            orch._target_exit_footprint_clearances(agent, state)
        )
        required = orch._target_exit_required_clearance(agent)
        effective = orch._target_exit_clearance(agent, state)

        self.assertGreater(base_clearance, required)
        self.assertLess(
            min(shovel_clearances),
            ASTAR_APPROACH_CONFIG["target_zone_guard_margin"],
        )
        self.assertLess(effective, required)

        _, direction, goal = orch._plan_target_exit(agent, state)
        self.assertGreater(direction[1], 0.0)
        self.assertGreater(goal[1], state[1])

    def test_keepout_exception_carries_clearance(self):
        zone = get_scenario("concave_target").target_zone
        with self.assertRaises(TargetKeepoutBlocked) as raised:
            _build_hybrid_astar_schedule_agent(
                agent_id="R0",
                priority=0.0,
                color=(1.0, 0.0, 0.0, 1.0),
                start_xy=(1.0, 0.5),
                gate_xy=(-1.0, 0.5),
                push_world_pts=((-1.0, 0.5), (-0.5, 0.5)),
                env_radius=4.0,
                target_zone_radius=zone.bounding_radius,
                pebble_centers=(),
                allow_replan=False,
                rover_radius=0.15,
                target_zone=zone,
            )
        self.assertLess(
            raised.exception.signed_clearance,
            raised.exception.required_clearance,
        )

    def test_setup_rejection_latches_exit_and_throttles_retry(self):
        orch = self._orchestrator_shell()

        def fail(*_args, **_kwargs):
            raise TargetKeepoutBlocked("R0", -0.04, 0.10)

        orch._setup_agent_selection_impl = fail
        agent = {}
        with self.assertRaises(TargetKeepoutBlocked):
            orch._setup_agent_selection(agent, None, "highway", {})
        self.assertEqual(
            agent["pending_target_exit_recovery"]["reason"],
            "allocation_start_inside_target_keepout",
        )
        self.assertGreaterEqual(agent["next_task_allocation_time"], 11.5)
        self.assertEqual(agent["path_rejection_count"], 1)

    def test_reservation_conflict_promotes_idle_blocker_to_fleet_recovery(self):
        orch = self._orchestrator_shell()
        orch._priority_reservation_conflict_indices = {0, 2}
        orch.agents = [
            {
                "index": 0,
                "id": "R0",
                "state": "PLANNING",
                "astar_phase": None,
                "state_val": np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
                "scheduler_agent": None,
                "selection": None,
            },
            {
                "index": 2,
                "id": "R2",
                "state": "NAVIGATING",
                "astar_phase": "APPROACH",
                "state_val": np.array([0.96, 0.0, math.pi, 0.0, 0.0]),
                "scheduler_agent": None,
                "selection": None,
            },
        ]
        contexts = [
            SafetyAgentContext(
                idx=0, agent_id="R0", state=orch.agents[0]["state_val"],
                active=False, path_constrained=False, path_points=(), goal=None,
                priority=45.0, collision_radius=0.46,
            ),
            SafetyAgentContext(
                idx=2, agent_id="R2", state=orch.agents[1]["state_val"],
                active=True, path_constrained=False, path_points=(), goal=(1.2, 0.0),
                priority=20.0, collision_radius=0.46,
            ),
        ]
        snapshots = orch._fleet_recovery_snapshots(contexts)
        self.assertTrue(all(snapshot.active for snapshot in snapshots))

        manager = FleetDeadlockRecoveryManager(
            FleetDeadlockConfig(stuck_duration=0.10)
        )

        def route(mover, _component, _retain_push, _attempt):
            return EscapeRoute(
                points=(mover.position, (mover.position[0] - 1.0, mover.position[1])),
                reverse=False,
                mode="test_reservation_escape",
            )

        manager.update(snapshots, {0: (0.0, 0.0), 2: (0.0, 0.0)}, 0.0, route)
        result = manager.update(
            snapshots, {0: (0.0, 0.0), 2: (0.0, 0.0)}, 0.2, route
        )
        self.assertEqual(result.active_component, (0, 2))
        self.assertEqual(result.winner_idx, 2)
        self.assertEqual(result.mover_idx, 0)


if __name__ == "__main__":
    unittest.main()
