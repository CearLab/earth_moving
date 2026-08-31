import unittest
from types import SimpleNamespace

from congestion_aware_reassignment import (
    ApproachCongestionMonitor, ApproachObservation, CongestionConfig,
)
from shared_path_allocator import (
    PathAllocationRequest, candidate_intersects_excluded_task_zone,
)


def observation(
    idx, time_s, distance, conflicted=True, blockers=(1,), task_type="highway",
    load=0.4, failures=0,
):
    return ApproachObservation(
        idx=idx,
        agent_id=f"R{idx}",
        sim_time=time_s,
        task_id=f"R{idx}-T1",
        task_started_at=0.0,
        position=(distance, float(idx)),
        gate=(0.0, 0.0),
        gate_distance_m=distance,
        conflicted=conflicted,
        blocker_indices=blockers,
        failed_replans_total=failures,
        expected_load_ratio=load,
        task_type=task_type,
        priority=20.0 + idx,
    )


class CongestionMonitorTests(unittest.TestCase):
    def config(self, enabled=True):
        return CongestionConfig(
            enabled=enabled,
            window_s=10.0,
            sample_interval_s=1.0,
            minimum_task_age_s=5.0,
            minimum_gate_progress_m=0.15,
            conflict_fraction=0.5,
            failed_replans=3,
            same_blocker_samples=3,
            cooldown_s=20.0,
            exclusion_radius_m=1.2,
            rearm_s=5.0,
        )

    def test_disabled_policy_never_intervenes(self):
        monitor = ApproachCongestionMonitor(self.config(enabled=False))
        for time_s in range(12):
            decision = monitor.update(
                [observation(0, time_s, 2.0, blockers=(1,))], time_s,
            )
        self.assertIsNone(decision)

    def test_persistent_conflict_triggers_after_full_window(self):
        monitor = ApproachCongestionMonitor(self.config())
        decision = None
        for time_s in range(12):
            decision = monitor.update(
                [observation(0, time_s, 2.0 - 0.005 * time_s)], time_s,
            )
            if decision is not None:
                break
        self.assertIsNotNone(decision)
        self.assertEqual(decision.mover_idx, 0)
        self.assertEqual(decision.blocker_indices, (1,))
        self.assertLess(decision.gate_progress_m, 0.15)

    def test_meaningful_gate_progress_prevents_reassignment(self):
        monitor = ApproachCongestionMonitor(self.config())
        decision = None
        for time_s in range(12):
            decision = monitor.update(
                [observation(0, time_s, 2.0 - 0.03 * time_s)], time_s,
            )
        self.assertIsNone(decision)

    def test_lower_commitment_candidate_is_selected(self):
        monitor = ApproachCongestionMonitor(self.config())
        decision = None
        for time_s in range(12):
            observations = [
                observation(0, time_s, 2.0, blockers=(1,), task_type="target", load=0.9),
                observation(1, time_s, 2.0, blockers=(0,), task_type="highway", load=0.3),
            ]
            decision = monitor.update(observations, time_s)
            if decision is not None:
                break
        self.assertIsNotNone(decision)
        self.assertEqual(decision.mover_idx, 1)

    def test_allocation_request_defaults_to_no_exclusions(self):
        request = PathAllocationRequest(
            agent_id="R0", plan_id=1, map_epoch=0, agent_pose_xy=(0.0, 0.0),
        )
        self.assertEqual(request.excluded_task_zones, ())

    def test_task_source_or_endpoint_inside_zone_is_excluded(self):
        candidate = SimpleNamespace(
            source=SimpleNamespace(world_center=(3.0, 3.0)),
            path_info={"world_path": ((2.0, 2.0), (0.20, 0.10))},
        )
        self.assertTrue(candidate_intersects_excluded_task_zone(
            candidate, ((0.0, 0.0, 0.50),),
        ))
        self.assertFalse(candidate_intersects_excluded_task_zone(
            candidate, ((-2.0, -2.0, 0.50),),
        ))


if __name__ == "__main__":
    unittest.main()
