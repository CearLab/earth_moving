"""Regression tests for occupied-cell target-path heat contribution."""

from types import SimpleNamespace
import unittest

from env import SimulationEnv


def _cell(x, y, *, objects=0, path_value=0.0, velocity=(0.0, 0.0)):
    cell = SimpleNamespace(
        x=x,
        y=y,
        num_objects=objects,
        is_target_zone=False,
        closest_x=0.5,
        closest_y=0.5,
        velocity_target=velocity,
        total_objects_path_target=path_value,
        best_path_target=[(x, y)] if path_value else [],
        next_child_target=None,
        heat_map=0.0,
    )
    return cell


class HeatmapSelfContributionTests(unittest.TestCase):
    def test_occupied_cell_receives_heat_from_its_own_target_path(self):
        source = _cell(
            8,
            8,
            objects=1,
            path_value=6.0,
            velocity=(-2 ** -0.5, -2 ** -0.5),
        )
        empty = _cell(4, 4)
        fake_env = SimpleNamespace(
            cells_with_objects=[source],
            all_cells=[source, empty],
            highway_threshold_ratio=0.45,
            use_spillage_model=False,
            own_path_heat_weight=1.0,
        )

        SimulationEnv.update_heat_map(fake_env)

        self.assertEqual(6.0, source.own_path_heat)
        self.assertEqual(0.0, source.incoming_path_heat)
        self.assertEqual(6.0, source.heat_map)
        self.assertAlmostEqual(2.7, fake_env.highway_threshold)

    def test_final_heat_uses_max_not_sum_of_components(self):
        source = _cell(
            8,
            8,
            objects=1,
            path_value=6.0,
            velocity=(-2 ** -0.5, -2 ** -0.5),
        )
        contributing_source = _cell(
            9,
            9,
            objects=1,
            path_value=4.0,
            velocity=(-2 ** -0.5, -2 ** -0.5),
        )
        fake_env = SimpleNamespace(
            cells_with_objects=[source, contributing_source],
            all_cells=[source, contributing_source],
            highway_threshold_ratio=0.5,
            use_spillage_model=False,
            own_path_heat_weight=1.0,
        )

        SimulationEnv.update_heat_map(fake_env)

        self.assertEqual(6.0, source.own_path_heat)
        self.assertGreaterEqual(source.incoming_path_heat, 0.0)
        self.assertEqual(
            max(source.own_path_heat, source.incoming_path_heat),
            source.heat_map,
        )

    def test_own_path_heat_can_be_logged_without_affecting_final_heat(self):
        source = _cell(
            8, 8, objects=1, path_value=6.0,
            velocity=(-2 ** -0.5, -2 ** -0.5),
        )
        fake_env = SimpleNamespace(
            cells_with_objects=[source],
            all_cells=[source],
            highway_threshold_ratio=0.5,
            use_spillage_model=False,
            own_path_heat_weight=0.0,
        )

        SimulationEnv.update_heat_map(fake_env)

        self.assertEqual(6.0, source.own_path_heat)
        self.assertEqual(0.0, source.incoming_path_heat)
        self.assertEqual(0.0, source.heat_map)
        self.assertEqual(0.0, fake_env.highway_threshold)

    def test_alignment_power_suppresses_off_axis_heat(self):
        source = _cell(
            8, 8, objects=1, path_value=6.0, velocity=(-1.0, 0.0)
        )
        probe = _cell(6, 7)
        fake_env = SimpleNamespace(
            cells_with_objects=[source],
            all_cells=[source, probe],
            highway_threshold_ratio=0.5,
            use_spillage_model=False,
            own_path_heat_weight=0.0,
            heat_alignment_power=1.0,
            heat_radial_gate_angle_deg=75.0,
        )

        SimulationEnv.update_heat_map(fake_env)
        linear_heat = probe.heat_map
        fake_env.heat_alignment_power = 4.0
        SimulationEnv.update_heat_map(fake_env)
        powered_heat = probe.heat_map

        self.assertGreater(linear_heat, 0.0)
        self.assertGreater(powered_heat, 0.0)
        self.assertLess(powered_heat, linear_heat)
        expected_dot = 2.0 / (5.0 ** 0.5)
        self.assertAlmostEqual(
            powered_heat,
            6.0 * expected_dot ** 4.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
