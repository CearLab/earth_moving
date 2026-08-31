from types import SimpleNamespace
from dataclasses import replace
import unittest
from unittest.mock import patch

from rover_profiles import (
    HIGHWAY_TASK,
    TARGET_TASK,
    ROVER_TYPES,
    RoverCapabilities,
    RoverGeometry,
    RoverType,
    TaskPolicy,
)
from shared_path_allocator import (
    PathAllocationRequest,
    allocate_path_from_shared_map,
    calculate_target_ready_fraction,
)


def _cell(x, physical_count, heat, planning_quantity):
    return SimpleNamespace(
        x=x,
        y=0,
        physical_object_count=float(physical_count),
        num_objects=float(planning_quantity),
        heat_map=float(heat),
        is_target_zone=False,
    )


def _candidate(
    source, task_type, delivered=1.0, reserved_path=(),
    collectable_objects=1.0, collectable_mass=1.0,
):
    return SimpleNamespace(
        source=source,
        task_type=task_type,
        representative_cell=source.canonical_cells[0],
        path_info={
            "world_path": [(0.0, 0.0), (0.1, 0.0)],
            "guide_mode": "test",
            "collectable_objects": float(collectable_objects),
            "collectable_material_mass": float(collectable_mass),
        },
        reserved_path_cells=frozenset(reserved_path),
        reserved_object_cells=frozenset({source.key}),
        expected_collected=1.0,
        expected_collected_objects=1.0,
        expected_collected_mass=1.0,
        expected_delivered=float(delivered),
        expected_spillage=0.0,
        is_root_source=True,
    )


def _snapshot_and_overlay(good_count=3.0, poor_count=7.0):
    good = _cell(1, good_count, 0.80, 30.0)
    poor = _cell(2, poor_count, 0.20, 10.0)
    peak = _cell(3, 0.0, 1.00, 0.0)
    converter = SimpleNamespace(is_in_target_zone=lambda x, y: False)
    env = SimpleNamespace(cells_with_objects=[good, poor], all_cells=[good, poor, peak])
    snapshot = SimpleNamespace(
        epoch=1,
        env_radius=3.0,
        env_2d=env,
        coord_converter=converter,
        material_value_mode="count",
        pebbles_count=float(good_count + poor_count),
    )
    good_source = SimpleNamespace(
        key=(1, 0),
        canonical_cells=(good,),
        heat=0.80,
        world_center=(0.0, 0.0),
    )
    poor_source = SimpleNamespace(
        key=(2, 0),
        canonical_cells=(poor,),
        heat=0.20,
        world_center=(0.5, 0.0),
    )
    candidates = (
        _candidate(good_source, TARGET_TASK, delivered=1.0),
        _candidate(good_source, HIGHWAY_TASK, delivered=20.0),
        _candidate(poor_source, TARGET_TASK, delivered=1.0),
        _candidate(poor_source, HIGHWAY_TASK, delivered=20.0),
    )
    overlay = SimpleNamespace(
        sources=(good_source, poor_source),
        candidates=candidates,
        generation_failures=(),
    )
    return snapshot, overlay


class PercentageTaskPolicyTests(unittest.TestCase):
    def test_target_ready_fraction_uses_physical_count_not_planning_mass(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        metrics = calculate_target_ready_fraction(
            snapshot,
            overlay,
            ROVER_TYPES["small"].policy,
        )
        self.assertAlmostEqual(metrics["outside_pebble_count"], 10.0)
        self.assertAlmostEqual(metrics["target_ready_pebble_count"], 3.0)
        self.assertAlmostEqual(metrics["target_ready_fraction"], 0.30)

    def test_dynamic_policy_enters_and_exits_with_hysteresis(self):
        policy = ROVER_TYPES["small"].policy
        self.assertEqual(
            policy.selection_tiers(0.30, HIGHWAY_TASK),
            (frozenset({TARGET_TASK}), frozenset({HIGHWAY_TASK})),
        )
        self.assertEqual(
            policy.selection_tiers(0.25, TARGET_TASK),
            (frozenset({TARGET_TASK}), frozenset({HIGHWAY_TASK})),
        )
        self.assertEqual(
            policy.selection_tiers(0.20, TARGET_TASK),
            (frozenset({HIGHWAY_TASK}), frozenset({TARGET_TASK})),
        )
        self.assertEqual(
            policy.selection_tiers(0.25, None),
            (frozenset({HIGHWAY_TASK}), frozenset({TARGET_TASK})),
        )

    def test_explicit_modes_are_strict(self):
        self.assertEqual(
            ROVER_TYPES["large"].policy.selection_tiers(0.0, HIGHWAY_TASK),
            (frozenset({TARGET_TASK}),),
        )
        highway_only = TaskPolicy(
            task_policy_mode="highway_only",
        )
        self.assertEqual(
            highway_only.selection_tiers(1.0, TARGET_TASK),
            (frozenset({HIGHWAY_TASK}),),
        )

    def test_dynamic_fraction_endpoints_are_not_hidden_strict_modes(self):
        policy = TaskPolicy(
            task_policy_mode="dynamic",
            target_preference_enter_fraction=1.0,
            target_preference_exit_fraction=1.0,
        )
        self.assertEqual(
            policy.selection_tiers(1.0, HIGHWAY_TASK, remaining_fraction=1.0),
            (frozenset({TARGET_TASK}), frozenset({HIGHWAY_TASK})),
        )

    def test_endgame_fraction_disables_highway_fallback(self):
        policy = ROVER_TYPES["small"].policy
        self.assertEqual(
            policy.selection_tiers(0.0, HIGHWAY_TASK, remaining_fraction=0.20),
            (frozenset({TARGET_TASK}),),
        )
        self.assertEqual(
            policy.selection_tiers(0.0, HIGHWAY_TASK, remaining_fraction=0.21),
            (frozenset({HIGHWAY_TASK}), frozenset({TARGET_TASK})),
        )

    def test_allocator_prefers_target_at_configured_fraction(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        request = PathAllocationRequest(
            agent_id="R0",
            plan_id=1,
            map_epoch=1,
            agent_pose_xy=(0.0, 0.0),
            rover_type=ROVER_TYPES["small"],
            previous_task_preference=HIGHWAY_TASK,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNone(result.error)
        self.assertEqual(result.best_choice, TARGET_TASK)
        self.assertAlmostEqual(result.best_path_info["target_ready_fraction"], 0.30)
        self.assertEqual(result.best_path_info["policy_task_preference"], TARGET_TASK)

    def test_dynamic_policy_falls_back_when_preferred_type_is_unavailable(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        replacement = []
        target_index = 0
        for candidate in overlay.candidates:
            if candidate.task_type == TARGET_TASK:
                replacement.append(
                    _candidate(
                        candidate.source,
                        TARGET_TASK,
                        reserved_path=((99, target_index),),
                    )
                )
                target_index += 1
            else:
                replacement.append(candidate)
        overlay.candidates = tuple(replacement)
        target_paths = {(99, index) for index in range(target_index)}
        request = PathAllocationRequest(
            agent_id="R0",
            plan_id=2,
            map_epoch=1,
            agent_pose_xy=(0.0, 0.0),
            reserved_path_cells=tuple(target_paths),
            rover_type=ROVER_TYPES["small"],
            previous_task_preference=HIGHWAY_TASK,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNone(result.error)
        self.assertEqual(result.best_choice, HIGHWAY_TASK)
        self.assertEqual(result.best_path_info["policy_task_preference"], TARGET_TASK)
        self.assertEqual(result.best_path_info["selected_task_tier"], 1)

    def test_target_only_profile_does_not_fall_back_to_highway(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=0.0, poor_count=10.0)
        replacement = []
        target_index = 0
        for candidate in overlay.candidates:
            if candidate.task_type == TARGET_TASK:
                replacement.append(
                    _candidate(
                        candidate.source,
                        TARGET_TASK,
                        reserved_path=((88, target_index),),
                    )
                )
                target_index += 1
            else:
                replacement.append(candidate)
        overlay.candidates = tuple(replacement)
        target_paths = {(88, index) for index in range(target_index)}
        request = PathAllocationRequest(
            agent_id="R1",
            plan_id=3,
            map_epoch=1,
            agent_pose_xy=(0.0, 0.0),
            reserved_path_cells=tuple(target_paths),
            rover_type=ROVER_TYPES["large"],
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNone(result.error)
        self.assertIsNone(result.best_cell)
        self.assertEqual(result.diagnostics["task_preference"], TARGET_TASK)
        self.assertGreater(
            result.diagnostics["rejections"].get(
                "task_disabled_by_percentage_policy", 0
            ),
            0,
        )

    def test_invalid_fraction_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            TaskPolicy(
                target_preference_enter_fraction=0.20,
                target_preference_exit_fraction=0.30,
            )

    def test_capacity_fit_can_rank_before_task_fallback(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        source = overlay.sources[0]
        overlay.candidates = (
            _candidate(
                source, TARGET_TASK, delivered=100.0,
                collectable_objects=12.0, collectable_mass=10.0,
            ),
            _candidate(
                source, HIGHWAY_TASK, delivered=1.0,
                collectable_objects=8.0, collectable_mass=10.0,
            ),
        )
        base = ROVER_TYPES["small"]
        capacity_first = replace(
            base,
            policy=replace(
                base.policy,
                capacity_fit_before_task_fallback=True,
                max_overcapacity_fraction=2.0,
            ),
        )
        request = PathAllocationRequest(
            agent_id="R0", plan_id=4, map_epoch=1,
            agent_pose_xy=(0.0, 0.0), rover_type=capacity_first,
            previous_task_preference=HIGHWAY_TASK,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertEqual(result.best_choice, HIGHWAY_TASK)
        self.assertEqual(result.best_path_info["capacity_class"], "fit")
        self.assertEqual(result.best_path_info["selected_capacity_tier"], 0)

    def test_task_preference_can_rank_before_capacity_when_configured(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        source = overlay.sources[0]
        overlay.candidates = (
            _candidate(
                source, TARGET_TASK, delivered=1.0,
                collectable_objects=12.0, collectable_mass=10.0,
            ),
            _candidate(
                source, HIGHWAY_TASK, delivered=100.0,
                collectable_objects=8.0, collectable_mass=10.0,
            ),
        )
        base = ROVER_TYPES["small"]
        task_first = replace(
            base,
            policy=replace(
                base.policy,
                capacity_fit_before_task_fallback=False,
                max_overcapacity_fraction=2.0,
            ),
        )
        request = PathAllocationRequest(
            agent_id="R0", plan_id=5, map_epoch=1,
            agent_pose_xy=(0.0, 0.0), rover_type=task_first,
            previous_task_preference=HIGHWAY_TASK,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertEqual(result.best_choice, TARGET_TASK)
        self.assertEqual(result.best_path_info["capacity_class"], "overcapacity")
        self.assertAlmostEqual(
            result.best_path_info["capacity_effective_load_ratio"], 1.5
        )

    def test_reject_overcapacity_removes_candidate(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=3.0, poor_count=7.0)
        source = overlay.sources[0]
        overlay.candidates = (
            _candidate(source, TARGET_TASK, collectable_objects=9.0),
        )
        base = ROVER_TYPES["small"]
        strict_capacity = replace(
            base,
            policy=replace(base.policy, overcapacity_behavior="reject"),
        )
        request = PathAllocationRequest(
            agent_id="R0", plan_id=6, map_epoch=1,
            agent_pose_xy=(0.0, 0.0), rover_type=strict_capacity,
            previous_task_preference=HIGHWAY_TASK,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNone(result.best_cell)
        self.assertEqual(result.diagnostics["rejections"]["overcapacity_rejected"], 1)

    def test_hard_minimum_utilization_rejects_underfilled_large_rover_path(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=1.0, poor_count=0.0)
        source = overlay.sources[0]
        overlay.candidates = (
            _candidate(
                source, TARGET_TASK, delivered=1.0,
                collectable_objects=1.0, collectable_mass=1.0,
            ),
        )
        base = ROVER_TYPES["large"]
        strict_underfill = replace(
            base,
            policy=replace(base.policy, minimum_capacity_utilization=0.35),
        )
        request = PathAllocationRequest(
            agent_id="R1", plan_id=7, map_epoch=1,
            agent_pose_xy=(0.0, 0.0), rover_type=strict_underfill,
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNone(result.best_cell)
        self.assertEqual(
            result.diagnostics["rejections"]["below_capacity_utilization"], 1,
        )

    def test_hard_minimum_uses_mass_or_object_utilization_independent_of_mode(self):
        snapshot, overlay = _snapshot_and_overlay(good_count=1.0, poor_count=0.0)
        snapshot.material_value_mode = "count"
        source = overlay.sources[0]
        # One large pebble is only 1/12 by count, but 7/18 by mass. It qualifies
        # at 35%, which prevents an arbitrary dependence on planner value mode.
        overlay.candidates = (
            _candidate(
                source, TARGET_TASK, delivered=1.0,
                collectable_objects=1.0, collectable_mass=7.0,
            ),
        )
        base = ROVER_TYPES["large"]
        policy = replace(base.policy, minimum_capacity_utilization=0.35)
        request = PathAllocationRequest(
            agent_id="R1", plan_id=8, map_epoch=1,
            agent_pose_xy=(0.0, 0.0), rover_type=replace(base, policy=policy),
        )
        with patch("shared_path_allocator.get_profile_overlay", return_value=overlay):
            result = allocate_path_from_shared_map(snapshot, request)
        self.assertIsNotNone(result.best_cell)
        self.assertAlmostEqual(
            result.best_path_info["capacity_effective_load_ratio"], 7.0 / 18.0,
        )
        self.assertEqual(
            result.best_path_info["capacity_utilization_definition"],
            "max(object_load_ratio,mass_load_ratio)",
        )


if __name__ == "__main__":
    unittest.main()
