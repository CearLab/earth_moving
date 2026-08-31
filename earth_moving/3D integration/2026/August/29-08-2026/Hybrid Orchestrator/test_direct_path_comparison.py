import math
import unittest

from rover_planning_overlay import (
    build_straight_target_overlay_path,
    convex_hull_source_cell_keys,
    normalize_target_candidate_value_mode,
    normalize_target_path_mode,
    normalize_target_source_mode,
)
from target_zones import TargetZoneSpec


class _Converter:
    def __init__(self):
        self.env_radius = 3.0
        self.cell_size = 0.10
        self.grid_size = 61
        self.target_zone_spec = TargetZoneSpec.circle(0.8)

    def convert_3d_to_2d(self, x, y):
        return (
            int(round((float(x) + self.env_radius) / self.cell_size)),
            int(round((self.env_radius - float(y)) / self.cell_size)),
        )

    def convert_2d_to_3d(self, x, y):
        return (
            -self.env_radius + int(x) * self.cell_size,
            self.env_radius - int(y) * self.cell_size,
        )

    def is_in_target_zone(self, x, y):
        return self.target_zone_spec.contains_world(*self.convert_2d_to_3d(x, y))


class DirectPathComparisonTests(unittest.TestCase):
    def test_comparison_modes_are_strictly_validated(self):
        self.assertEqual(normalize_target_path_mode("straight_nearest"), "straight_nearest")
        self.assertEqual(normalize_target_candidate_value_mode("source_only"), "source_only")
        self.assertEqual(normalize_target_source_mode("convex_hull"), "convex_hull")
        with self.assertRaises(ValueError):
            normalize_target_path_mode("almost_straight")
        with self.assertRaises(ValueError):
            normalize_target_candidate_value_mode("unknown")
        with self.assertRaises(ValueError):
            normalize_target_source_mode("outline-ish")

    def test_convex_hull_uses_geometric_exposure_not_graph_roots(self):
        converter = _Converter()

        class Cell:
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.num_objects = 1.0

        exposed = [Cell(50, 30), Cell(10, 30), Cell(30, 10), Cell(30, 50)]
        interior = Cell(40, 30)
        env = type("Env", (), {"cells_with_objects": exposed + [interior]})()
        keys = convex_hull_source_cell_keys(env, converter)
        self.assertTrue({(cell.x, cell.y) for cell in exposed}.issubset(keys))
        self.assertNotIn((interior.x, interior.y), keys)

    def test_straight_mode_goes_to_nearest_circle_boundary_without_zigzag(self):
        converter = _Converter()
        result = build_straight_target_overlay_path(
            env_2d=None,
            converter=converter,
            start_world=(2.0, 0.0),
            cell_size=0.16,
            reservation_radius=0.12,
        )
        self.assertIsNotNone(result)
        self.assertEqual(len(result.world_path), 2)
        start, goal = result.world_path
        self.assertAlmostEqual(start[1], 0.0, places=9)
        self.assertAlmostEqual(goal[1], 0.0, places=9)
        self.assertLessEqual(goal[0], 0.8 + 1e-9)
        self.assertTrue(converter.target_zone_spec.contains_world(*goal))
        self.assertGreater(len(result.swept_canonical_cells), 0)
        self.assertAlmostEqual(
            math.dist(start, goal), start[0] - goal[0], places=9,
        )

    def test_straight_mode_ignores_sources_already_inside_target(self):
        converter = _Converter()
        self.assertIsNone(build_straight_target_overlay_path(
            env_2d=None,
            converter=converter,
            start_world=(0.2, 0.0),
            cell_size=0.16,
            reservation_radius=0.12,
        ))


if __name__ == "__main__":
    unittest.main()
