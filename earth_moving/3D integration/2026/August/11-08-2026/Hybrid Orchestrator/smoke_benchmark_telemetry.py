"""Dependency-light smoke checks; use pytest for the full project test suite."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as S

from benchmark_telemetry import BenchmarkTelemetry, calculate_environment_metrics, point_to_polyline_distance


assert abs(point_to_polyline_distance((1.0, 0.5), [(0.0, 0.0), (2.0, 0.0)]) - 0.5) < 1e-9


def cell(x, y, count, heat, distance, target=False):
    return S(
        x=x, y=y, physical_object_count=count, material_mass=count,
        num_objects=count, heat_map=heat, distance_to_target=distance,
        is_target_zone=target, velocity_highway=(1.0, 0.0),
        best_path_target=[] if target else [1],
    )


outside = [cell(0, 0, 2, 10, 3), cell(1, 0, 1, 6, 2), cell(4, 4, 1, 2, 6)]
target = [cell(2, 2, 1, 0, 0, True)]
env = S(
    cells_with_objects=outside, target_zone_cells=target,
    all_cells=outside + target, highway_threshold=2.0, grid_size=5,
)
metrics = calculate_environment_metrics(env, "count", 0.5)
assert abs(metrics["delivery_fraction"] - 0.2) < 1e-9
assert abs(metrics["reference_metrics"]["highway_material_fraction"] - 0.75) < 1e-9

with TemporaryDirectory() as temporary:
    telemetry = BenchmarkTelemetry(temporary, "smoke", capture_images=False)
    agent = {
        "id": "R1", "event_task_id": "T1", "rover_profile": None,
        "selection": {"path_type": "target", "shovel_width": 0.2, "path_info": {
            "expected_collected": 2.0, "expected_delivered": 1.5, "expected_spillage": 0.5,
        }},
    }
    start = {
        1: {"pebble_id": 1, "profile": "small", "material_mass": 1,
            "position": [0.1, 0.0, 0.02], "in_target": False,
            "signed_target_distance_m": 0.1},
        2: {"pebble_id": 2, "profile": "small", "material_mass": 1,
            "position": [0.2, 0.02, 0.02], "in_target": False,
            "signed_target_distance_m": 0.2},
    }
    telemetry.start_push(agent, 1.0, [(0.0, 0.0), (1.0, 0.0)], start)
    telemetry.record_contacts("R1", [1])
    telemetry.end_push("R1", 2.0, start)
    assert not telemetry.observe_due(2.5, start)
    observed = dict(start)
    observed[1] = dict(start[1], position=[-0.1, 0.0, 0.02], in_target=True,
                       signed_target_distance_m=-0.1)
    row = telemetry.observe_due(3.0, observed)[0]
    assert row["corridor_candidate_count"] == 2
    assert row["contacted_count"] == 1.0
    assert row["entered_target_count"] == 1.0
    telemetry.close(3.0, "complete", {})
    assert Path(telemetry.run_summary_path).exists()
print("benchmark telemetry smoke checks passed")
