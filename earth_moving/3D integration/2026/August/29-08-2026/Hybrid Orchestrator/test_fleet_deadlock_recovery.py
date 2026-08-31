import unittest

from fleet_deadlock_recovery import (
    EscapeRoute,
    FleetDeadlockConfig,
    FleetDeadlockRecoveryManager,
    FleetRecoveryAgent,
)


def agent(
    idx,
    x,
    y=0.0,
    yaw=0.0,
    phase="APPROACH",
    priority=20.0,
    remaining=3.0,
    radius=0.46,
    speed=0.0,
):
    return FleetRecoveryAgent(
        idx=idx,
        agent_id=f"R{idx}",
        state=(x, y, yaw, speed, 0.0),
        active=True,
        phase=phase,
        priority=priority,
        collision_radius=radius,
        remaining_path_m=remaining,
        expected_quantity=10.0 + idx,
        path_progress_m=0.0,
    )


def route_away(mover, component, preferred_reverse, attempt):
    others = [value for value in component if value.idx != mover.idx]
    center = sum(value.position[0] for value in others) / len(others)
    direction = -1.0 if preferred_reverse else (
        -1.0 if mover.position[0] <= center else 1.0
    )
    goal = (mover.position[0] + direction * (1.2 + 0.1 * attempt), 0.0)
    return EscapeRoute(
        points=(mover.position, goal),
        reverse=preferred_reverse,
        mode="rollback" if preferred_reverse else "escape",
    )


class FleetDeadlockRecoveryTests(unittest.TestCase):
    def manager(self):
        return FleetDeadlockRecoveryManager(
            FleetDeadlockConfig(
                stuck_duration=3.0,
                release_hold_time=0.8,
                total_timeout=30.0,
            )
        )

    def trigger(self, manager, agents, planner=route_away, controls=None):
        controls = controls or {value.idx: (0.0, 0.0) for value in agents}
        manager.update(agents, controls, 0.0, planner)
        return manager.update(agents, controls, 3.1, planner)

    def test_covers_old_reservation_recovery_dead_zone(self):
        manager = self.manager()
        agents = [agent(0, 0.0, priority=20.0), agent(2, 1.06, priority=20.002)]
        result = self.trigger(manager, agents)
        self.assertEqual(result.active_component, (0, 2))
        self.assertEqual(result.winner_idx, 0)
        self.assertEqual(result.mover_idx, 2)
        self.assertEqual(result.abort_task_indices, {2})

    def test_measured_stall_triggers_even_with_nonzero_wheel_commands(self):
        manager = self.manager()
        agents = [agent(0, 0.0), agent(1, 1.0)]
        commands = {0: (0.3, 0.0), 1: (-0.2, 0.0)}
        result = self.trigger(manager, agents, controls=commands)
        self.assertEqual(result.active_component, (0, 1))

    def test_connected_three_rover_component_is_owned_as_one_group(self):
        manager = self.manager()
        agents = [
            agent(0, 0.0, phase="APPROACH", priority=20.0),
            agent(1, 1.0, phase="PUSH", priority=0.0, remaining=0.7),
            agent(2, 2.0, phase="APPROACH", priority=20.002),
        ]
        result = self.trigger(manager, agents)
        self.assertEqual(result.active_component, (0, 1, 2))
        self.assertEqual(result.winner_idx, 1)
        self.assertIn(result.mover_idx, (0, 2))
        self.assertNotEqual(result.mover_idx, 1)

    def test_all_pushers_choose_closest_to_completion_and_rollback_loser(self):
        manager = self.manager()
        calls = []

        def planner(mover, component, preferred_reverse, attempt):
            calls.append((mover.idx, preferred_reverse, attempt))
            return route_away(mover, component, preferred_reverse, attempt)

        agents = [
            agent(0, 0.0, phase="PUSH", priority=0.0, remaining=2.0),
            agent(1, 1.0, phase="PUSH", priority=0.001, remaining=0.4),
            agent(2, 2.0, phase="PUSH", priority=0.002, remaining=1.1),
        ]
        result = self.trigger(manager, agents, planner=planner)
        self.assertEqual(result.winner_idx, 1)
        self.assertNotEqual(result.mover_idx, 1)
        self.assertTrue(calls[0][1])
        self.assertEqual(result.abort_task_indices, set())
        self.assertLess(result.controls[result.mover_idx][0], 0.0)

    def test_failed_push_rollback_aborts_only_that_task_then_uses_general_escape(self):
        manager = self.manager()
        calls = []

        def planner(mover, component, preferred_reverse, attempt):
            calls.append((mover.idx, preferred_reverse, attempt))
            if preferred_reverse:
                return None
            return route_away(mover, component, False, attempt)

        agents = [
            agent(0, 0.0, phase="PUSH", priority=0.0, remaining=0.3),
            agent(1, 1.0, phase="PUSH", priority=0.001, remaining=2.0),
        ]
        result = self.trigger(manager, agents, planner=planner)
        self.assertEqual(result.winner_idx, 0)
        self.assertEqual(result.abort_task_indices, {1})
        self.assertIn((1, True, 0), calls)
        self.assertTrue(any(idx == 1 and not reverse for idx, reverse, _ in calls))

    def test_resolution_requires_clearance_hold_and_resumes_every_member(self):
        manager = self.manager()
        close = [agent(0, 0.0), agent(1, 1.0)]
        self.trigger(manager, close)
        far = [agent(0, -1.0), agent(1, 1.5)]
        first = manager.update(far, {0: (0.0, 0.0), 1: (0.0, 0.0)}, 4.0, route_away)
        self.assertTrue(first.active_component)
        done = manager.update(far, {0: (0.0, 0.0), 1: (0.0, 0.0)}, 4.9, route_away)
        self.assertEqual(done.active_component, ())
        self.assertEqual(done.resume_indices, {0, 1})
        self.assertTrue(any(event["event"] == "GROUP_DEADLOCK_RESOLVED" for event in done.events))

    def test_heading_alignment_counts_as_escape_progress(self):
        manager = FleetDeadlockRecoveryManager(
            FleetDeadlockConfig(
                stuck_duration=3.0,
                route_progress_timeout=2.5,
                route_timeout=10.0,
                total_timeout=30.0,
            )
        )

        def north_route(mover, component, preferred_reverse, attempt):
            return EscapeRoute(
                points=(mover.position, (mover.position[0], mover.position[1] + 1.2)),
                reverse=False,
                mode="turn_then_escape",
            )

        close = [agent(0, 0.0), agent(1, 1.0)]
        self.trigger(manager, close, planner=north_route)
        mover_idx = manager.episode.current_mover_idx
        turning = [
            agent(value.idx, value.position[0], yaw=(0.55 if value.idx == mover_idx else 0.0))
            for value in close
        ]
        result = manager.update(
            turning, {0: (0.0, 0.0), 1: (0.0, 0.0)}, 5.8, north_route,
        )
        self.assertFalse(any(
            event["event"] == "GROUP_ESCAPE_REPLAN" for event in result.events
        ))
        self.assertEqual(result.mover_idx, mover_idx)

    def test_expansion_preserves_productive_winner_over_group_escape_member(self):
        manager = self.manager()
        close = [agent(0, 0.0, priority=20.0), agent(1, 1.0, priority=20.001)]
        self.trigger(manager, close)
        self.assertEqual(manager.episode.winner_idx, 0)
        expanded = [
            agent(0, 0.0, priority=20.0),
            agent(1, 1.0, phase="GROUP_ESCAPE", priority=15.0),
            agent(2, 2.0, priority=20.002),
        ]
        result = manager.update(
            expanded, {0: (0.0, 0.0), 1: (0.0, 0.0), 2: (0.0, 0.0)},
            3.2, route_away,
        )
        self.assertEqual(result.winner_idx, 0)
        self.assertEqual(manager.episode.original_phases[1], "APPROACH")

    def test_individually_clear_member_leaves_unresolved_group(self):
        manager = self.manager()
        close = [agent(0, 0.0), agent(1, 1.0), agent(2, 2.0)]
        self.trigger(manager, close)
        partly_clear = [agent(0, 0.0), agent(1, 1.0), agent(2, 3.0)]
        manager.update(
            partly_clear, {0: (0.0, 0.0), 1: (0.0, 0.0), 2: (0.0, 0.0)},
            4.0, route_away,
        )
        result = manager.update(
            partly_clear, {0: (0.0, 0.0), 1: (0.0, 0.0), 2: (0.0, 0.0)},
            4.9, route_away,
        )
        self.assertEqual(result.active_component, (0, 1))
        self.assertEqual(result.resume_indices, {2})
        self.assertTrue(any(
            event["event"] == "GROUP_MEMBERS_RELEASED" for event in result.events
        ))

    def test_group_timeout_rearbitrates_instead_of_latching_safe_stop(self):
        manager = FleetDeadlockRecoveryManager(
            FleetDeadlockConfig(
                stuck_duration=3.0,
                route_progress_timeout=20.0,
                route_timeout=20.0,
                total_timeout=4.0,
            )
        )
        close = [agent(0, 0.0), agent(1, 1.0)]
        self.trigger(manager, close)
        result = manager.update(
            close, {0: (0.0, 0.0), 1: (0.0, 0.0)}, 7.2, route_away,
        )
        self.assertEqual(result.phase, "GROUP_ESCAPE")
        self.assertFalse(manager.episode.blocked)
        self.assertTrue(any(
            event["event"] == "GROUP_DEADLOCK_REARBITRATED"
            for event in result.events
        ))

    def test_no_verified_route_rotates_winner_without_permanent_stop(self):
        manager = self.manager()

        def no_route(mover, component, preferred_reverse, attempt):
            return None

        close = [agent(0, 0.0), agent(1, 1.0)]
        result = self.trigger(manager, close, planner=no_route)
        self.assertEqual(result.phase, "GROUP_ESCAPE")
        self.assertFalse(manager.episode.blocked)
        self.assertTrue(any(
            event["event"] == "GROUP_DEADLOCK_REARBITRATED"
            for event in result.events
        ))


if __name__ == "__main__":
    unittest.main()
