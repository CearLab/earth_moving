from pathlib import Path
from types import MethodType
import unittest

from orchestrator_hybrid_multi_astar_scheduled import (
    MultiAStarScheduledHybridOrchestrator,
)
from RUN_ALL_DIRECT_PATH_COMPARISONS import (
    WINDOWS_SAFE_PATH_LIMIT,
    _preflight_run_directory,
    _result_row,
    _write_manifest,
)
from RUN_DIRECT_PATH_COMPARISON import COMPARISON_PRESETS


def _lifecycle_stub(progress, *, sim_time=10.0):
    orchestrator = MultiAStarScheduledHybridOrchestrator.__new__(
        MultiAStarScheduledHybridOrchestrator
    )
    orchestrator.benchmark_telemetry = None
    orchestrator._sim_time = float(sim_time)
    orchestrator._benchmark_material_complete_logged = False
    orchestrator._benchmark_all_parked_logged = False
    orchestrator._last_delivered_count = None
    orchestrator._last_delivery_progress_time = 0.0
    orchestrator._run_outcome = None
    orchestrator.max_sim_time = None
    orchestrator.max_no_delivery_time = None
    orchestrator.auto_exit_on_completion = True
    orchestrator.running = True
    orchestrator.agents = [{"state": "PARKED"}]
    orchestrator._get_material_progress = MethodType(
        lambda self: dict(progress), orchestrator
    )
    orchestrator._capture_benchmark_snapshot = MethodType(
        lambda self, *args, **kwargs: None, orchestrator
    )
    orchestrator._log_event = MethodType(
        lambda self, *args, **kwargs: None, orchestrator
    )
    return orchestrator


class ComparisonBatchTests(unittest.TestCase):
    @staticmethod
    def _artifact_dir(name):
        path = Path(__file__).resolve().parent / ".test_artifacts" / name
        path.mkdir(parents=True, exist_ok=True)
        return path

    def test_first_four_presets_use_hull_and_expanded_presets_use_all(self):
        names = list(COMPARISON_PRESETS)
        self.assertTrue(all(
            COMPARISON_PRESETS[name]["source_mode"] == "convex_hull"
            for name in names[:4]
        ))
        self.assertTrue(all(
            COMPARISON_PRESETS[name]["source_mode"] == "all"
            for name in names[4:]
        ))

    def test_auto_exit_requires_material_complete_and_parked(self):
        orchestrator = _lifecycle_stub({
            "delivered_count": 60,
            "remaining_count": 0,
        })
        orchestrator._update_benchmark_lifecycle()
        self.assertFalse(orchestrator.running)
        self.assertEqual(orchestrator._run_outcome, "mission_complete")
        self.assertTrue(orchestrator._benchmark_all_parked_logged)

    def test_no_delivery_safeguard_has_distinct_outcome(self):
        orchestrator = _lifecycle_stub(
            {"delivered_count": 2, "remaining_count": 58}, sim_time=100.0
        )
        orchestrator._last_delivered_count = 2
        orchestrator._last_delivery_progress_time = 0.0
        orchestrator.max_no_delivery_time = 50.0
        orchestrator._update_benchmark_lifecycle()
        self.assertFalse(orchestrator.running)
        self.assertEqual(
            orchestrator._run_outcome, "stalled_no_delivery_progress"
        )

    def test_manifest_is_written_as_json_and_csv(self):
        args = type("Args", (), {
            "scenario": "baseline_circle_uniform",
            "pebbles": 60,
            "seed": 41,
            "comparisons": ["P0_straight_root"],
            "max_sim_time": 100.0,
            "max_no_delivery_time": 50.0,
            "max_wall_time": 200.0,
        })()
        directory = self._artifact_dir("manifest")
        json_path, csv_path = _write_manifest(
                directory, args, "test_batch", [{
                    "comparison": "P0_straight_root",
                    "scenario": args.scenario,
                    "status": "completed",
                }]
        )
        self.assertTrue(json_path.exists())
        self.assertTrue(csv_path.exists())
        self.assertIn("test_batch", json_path.read_text(encoding="utf-8"))

    def test_short_run_directory_passes_write_and_path_preflight(self):
        directory = self._artifact_dir("preflight")
        length, longest = _preflight_run_directory(
            directory / "b260826_120000" / "p5"
        )
        self.assertLessEqual(length, WINDOWS_SAFE_PATH_LIMIT)
        self.assertFalse(any(
            path.name.startswith("_preflight")
            for path in directory.rglob("*")
        ))
        self.assertIn("p5", str(longest))

    def test_missing_summary_is_a_hard_logging_failure(self):
        args = type("Args", (), {"scenario": "baseline_circle_uniform"})()
        directory = self._artifact_dir("missing_summary")
        latest = directory / "LATEST_RUN.txt"
        if latest.exists():
            latest.unlink()
        row = _result_row(
            args, "P0_straight_root", directory, 0, 1.0
        )
        self.assertEqual(row["status"], "missing_run_summary")
        self.assertIn("RUN_SUMMARY_JSON", row["error"])


if __name__ == "__main__":
    unittest.main()
