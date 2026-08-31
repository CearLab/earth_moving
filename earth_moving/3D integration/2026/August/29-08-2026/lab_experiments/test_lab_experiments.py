from __future__ import annotations

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from experiment_utils import load_config, resolve_from_config
from generate_experiment_plan import build_plan, validate_plan
from run_experiment_suite import _select_rows
from run_single_experiment import _status_name


class LabExperimentDefinitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, cls.config_path = load_config(HERE / "experiment_config.yaml")
        cls.rows = build_plan(cls.config)

    def test_staged_plan_has_2400_runs_and_1120_core_runs(self):
        validate_plan(self.config, self.rows)
        self.assertEqual(2400, len(self.rows))
        self.assertEqual(1120, sum(row["stage"] == "core" for row in self.rows))
        self.assertEqual(14, len(self.config["planners"]))
        self.assertEqual(8, len(self.config["targets"]))

    def test_every_initial_condition_has_all_planners(self):
        expected = {item["id"] for item in self.config["planners"]}
        groups = {}
        for row in self.rows:
            if row["stage"] != "core":
                continue
            key = (row["initial_state_id"], row["rover_profile"])
            groups.setdefault(key, set()).add(row["planner_id"])
        self.assertTrue(groups)
        self.assertTrue(all(value == expected for value in groups.values()))

    def test_local_validation_cases_resolve_uniquely(self):
        selected = _select_rows(self.config, self.rows, "validation")
        self.assertEqual(len(self.config["local_validation"]["cases"]), len(selected))

    def test_server_calibration_is_32_balanced_runs(self):
        selected = _select_rows(self.config, self.rows, "calibration")
        self.assertEqual(32, len(selected))

    def test_linux_portable_relative_simulation_path(self):
        simulation = resolve_from_config(self.config_path, self.config["simulation_dir"])
        self.assertTrue((simulation / "orchestrator_hybrid_multi_astar_scheduled.py").exists())
        self.assertNotIn("C:\\", self.config["simulation_dir"])

    def test_status_classification_requires_valid_logs(self):
        self.assertEqual("completed", _status_name("mission_complete", True, 0))
        self.assertEqual("logging_failed", _status_name("mission_complete", False, 0))
        self.assertEqual("mission_stalled", _status_name("stalled_no_delivery_progress", True, 2))
        self.assertEqual("mission_timeout", _status_name("max_sim_time", True, 2))

    def test_target_set_contains_centered_and_offcenter_pairs(self):
        targets = {item["id"] for item in self.config["targets"]}
        for shape in ("circle", "rectangle", "amorphous", "l"):
            self.assertIn(f"{shape}_center", targets)
            self.assertIn(f"{shape}_offcenter", targets)

    def test_lab_target_shapes_have_equal_area(self):
        simulation = resolve_from_config(self.config_path, self.config["simulation_dir"])
        sys.path.insert(0, str(simulation))
        from scenario_configs import SCENARIOS
        areas = [
            SCENARIOS[name].target_zone.area
            for name in (
                "lab_circle_center", "lab_rectangle_center",
                "lab_amorphous_center", "lab_l_center",
            )
        ]
        self.assertLess((max(areas) - min(areas)) / areas[0], 0.01)

    def test_single_rover_matrix_is_capacity_policy_neutral(self):
        self.assertEqual(
            "uncapped_corridor",
            self.config["fixed_conditions"]["capacity_scoring_mode"],
        )


if __name__ == "__main__":
    unittest.main()
