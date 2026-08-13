from types import SimpleNamespace
import unittest

import main as planning_main
from env import SimulationEnv


class PlanningHyperparameterTests(unittest.TestCase):
    def setUp(self):
        self.original = dict(planning_main.PLANNING_HYPERPARAMETERS)

    def tearDown(self):
        planning_main.PLANNING_HYPERPARAMETERS.clear()
        planning_main.PLANNING_HYPERPARAMETERS.update(self.original)

    def test_configuration_updates_are_effective_and_reported(self):
        effective = planning_main.configure_planning_hyperparameters(
            target_visibility_mode="dynamic",
            dynamic_visibility_min_angle=20.0,
            dynamic_visibility_max_angle=75.0,
            highway_threshold_ratio=0.65,
        )
        self.assertEqual(effective["target_visibility_mode"], "dynamic")
        self.assertEqual(effective["dynamic_visibility_min_angle"], 20.0)
        self.assertEqual(effective["dynamic_visibility_max_angle"], 75.0)
        self.assertEqual(effective["highway_threshold_ratio"], 0.65)

    def test_invalid_visibility_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            planning_main.configure_planning_hyperparameters(
                target_visibility_mode="unknown"
            )

    def test_fixed_and_dynamic_visibility_modes_select_different_angles(self):
        env = SimulationEnv.__new__(SimulationEnv)
        env.target_visibility_mode = "fixed"
        env.highway_visibility_mode = "fixed"
        env.target_angle_tolerance = 42.0
        env.highway_angle_tolerance = 58.0
        env.angle_min_deg = 20.0
        env.angle_max_deg = 80.0
        env.grid_diagonal = 100.0
        current = SimpleNamespace(x=0, y=0, closest_x=10.5, closest_y=0.5)
        target = SimpleNamespace(x=10, y=0)

        self.assertEqual(env.target_visibility_angle(current), 42.0)
        self.assertEqual(env.highway_visibility_angle(current, target), 58.0)

        env.target_visibility_mode = "dynamic"
        env.highway_visibility_mode = "dynamic"
        self.assertIsNone(env.target_visibility_angle(current))
        self.assertIsNone(env.highway_visibility_angle(current, target))
        dynamic_angle = env.get_angle_tolerance_deg(current)
        self.assertGreaterEqual(dynamic_angle, env.angle_min_deg)
        self.assertLessEqual(dynamic_angle, env.angle_max_deg)


if __name__ == "__main__":
    unittest.main()
