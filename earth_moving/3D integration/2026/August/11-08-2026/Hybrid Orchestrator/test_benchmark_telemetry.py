from types import SimpleNamespace

import pytest

from benchmark_telemetry import calculate_environment_metrics, point_to_polyline_distance


def test_point_to_polyline_distance_uses_segments_not_only_waypoints():
    assert point_to_polyline_distance((1.0, 0.5), [(0.0, 0.0), (2.0, 0.0)]) == pytest.approx(0.5)


def test_environment_metrics_separates_planner_and_fixed_reference_thresholds():
    def cell(x, y, count, heat, distance, target=False, velocity=(1.0, 0.0)):
        return SimpleNamespace(
            x=x, y=y, physical_object_count=count, material_mass=count,
            num_objects=count, heat_map=heat, distance_to_target=distance,
            is_target_zone=target, velocity_highway=velocity,
            best_path_target=[1] if not target else [],
        )

    outside = [cell(0, 0, 2, 10, 3), cell(1, 0, 1, 6, 2), cell(4, 4, 1, 2, 6)]
    target = [cell(2, 2, 1, 0, 0, target=True)]
    env = SimpleNamespace(
        cells_with_objects=outside, target_zone_cells=target,
        all_cells=outside + target, highway_threshold=2.0, grid_size=5,
    )
    metrics = calculate_environment_metrics(env, "count", reference_highway_ratio=0.5)
    assert metrics["delivery_fraction"] == pytest.approx(0.2)
    assert metrics["planner_metrics"]["highway_material_fraction"] == pytest.approx(1.0)
    assert metrics["reference_metrics"]["highway_material_fraction"] == pytest.approx(0.75)
    assert metrics["reference_metrics"]["component_count"] == 1
