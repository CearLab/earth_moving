from pathlib import Path
import unittest

from benchmark_telemetry import BenchmarkTelemetry


def pebble(pid, x, y, inside=False, mass=1):
    return {
        "pebble_id": pid, "bullet_id": 100 + pid, "profile": "small",
        "material_mass": mass, "visual_scale": 1.0,
        "position": [x, y, 0.02], "velocity": [0.0, 0.0, 0.0],
        "speed_m_s": 0.0, "in_target": inside,
        "signed_target_distance_m": -0.1 if inside else abs(x),
    }


class PushAttributionTests(unittest.TestCase):
    def test_push_metrics_require_contact_and_use_delayed_observation(self):
        directory = Path(__file__).resolve().parent / ".test_artifacts" / "push_attribution"
        directory.mkdir(parents=True, exist_ok=True)
        telemetry = BenchmarkTelemetry(directory, "test", capture_images=False)
        agent = {
                "id": "R1", "event_task_id": "T1", "rover_profile": None,
                "selection": {"path_type": "target", "shovel_width": 0.2, "path_info": {
                    "expected_collected": 2, "expected_delivered": 1.5, "expected_spillage": 0.5,
                }},
        }
        start = {1: pebble(1, 0.1, 0.0), 2: pebble(2, 0.2, 0.02)}
        telemetry.start_push(agent, 1.0, [(0, 0), (1, 0)], start)
        telemetry.record_contacts("R1", [1])
        telemetry.end_push("R1", 2.0, start)
        self.assertEqual([], telemetry.observe_due(2.5, start))
        observed = {1: pebble(1, -0.1, 0.0, True), 2: start[2]}
        rows = telemetry.observe_due(3.0, observed)
        self.assertEqual(2, rows[0]["corridor_candidate_count"])
        self.assertEqual(1, rows[0]["contacted_count"])
        self.assertEqual(1, rows[0]["entered_target_count"])
        self.assertAlmostEqual(1.0, rows[0]["delivery_efficiency_count"])
        telemetry.close(3.0, "complete", {})
