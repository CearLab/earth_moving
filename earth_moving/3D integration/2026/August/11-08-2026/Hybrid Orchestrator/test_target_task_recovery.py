import contextlib
import io
import os
import sys
import unittest

# The test may be executed from a staging directory while the remaining
# simulator modules live in the Hybrid Orchestrator directory.
sys.path.append(os.getcwd())

from coordinate_converter import CoordinateConverter
from main import compute_2d_env
from rover_planning_overlay import (
    _best_guide,
    build_profile_overlay_snapshot,
)
from rover_profiles import ROVER_TYPES
from scenario_configs import get_scenario


FINAL_ORPHAN_POSITIONS = (
    (0.2258, -0.0365),
    (-0.5001, -1.5321),
    (-0.0832, -0.6788),
    (-0.0892, -0.6563),
    (0.4079, -0.0955),
    (0.4451, -0.1881),
    (-0.1792, -0.7932),
    (-0.0481, -1.2762),
    (1.2188, -0.1420),
)


class TargetTaskRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = get_scenario("concave_target").with_target_center((1.25, 0.65))
        cls.env_radius = 3.7492074898996375
        objects = [(x, y, 0.01) for x, y in FINAL_ORPHAN_POSITIONS]
        with contextlib.redirect_stdout(io.StringIO()):
            cls.env = compute_2d_env(
                cls.env_radius,
                cls.scenario.target_zone.bounding_radius,
                0.22,
                objects,
                manual_mode=False,
                use_spillage_model=True,
                visualize_potential=False,
                target_zone=cls.scenario.target_zone,
            )
            cls.converter = CoordinateConverter(
                cls.env_radius,
                cls.scenario.target_zone.bounding_radius,
                0.22,
                target_zone=cls.scenario.target_zone,
            )

    def test_every_remaining_cell_reaches_a_real_target_terminal(self):
        self.assertGreater(len(self.env.cells_with_objects), 0)
        for cell in self.env.cells_with_objects:
            path = list(getattr(cell, "best_path_target", ()) or ())
            self.assertGreaterEqual(len(path), 2, (cell.x, cell.y))
            self.assertTrue(path[-1].is_target_zone, (cell.x, cell.y))
            self.assertEqual(len(path), len({id(value) for value in path}))

    def test_visibility_never_contains_the_observer_itself(self):
        for cell in self.env.cells_with_objects:
            children = [item["cell"] for item in cell.visible_cells_target]
            self.assertNotIn(cell, children)

    def test_small_and_large_overlays_offer_target_tasks_for_all_sources(self):
        for rover_name in ("small", "large"):
            overlay = build_profile_overlay_snapshot(
                1,
                self.env,
                self.converter,
                ROVER_TYPES[rover_name],
                material_value_mode="count",
            )
            target_sources = {
                candidate.source.key
                for candidate in overlay.candidates
                if candidate.task_type == "target"
            }
            self.assertEqual(target_sources, {source.key for source in overlay.sources})
            for candidate in overlay.candidates:
                if candidate.task_type == "target":
                    self.assertGreaterEqual(len(candidate.path_info["world_path"]), 2)

    def test_missing_canonical_guide_gets_direct_target_recovery_guide(self):
        overlay = build_profile_overlay_snapshot(
            2,
            self.env,
            self.converter,
            ROVER_TYPES["small"],
            material_value_mode="count",
        )
        source = overlay.sources[0]
        saved = [
            (cell, cell.best_path_target)
            for cell in source.canonical_cells
        ]
        try:
            for cell, _ in saved:
                cell.best_path_target = []
            guide = _best_guide(self.env, source, "target")
        finally:
            for cell, path in saved:
                cell.best_path_target = path
        self.assertIsNotNone(guide)
        self.assertEqual(guide[-1]["guide_mode"], "direct_target_orphan_recovery")


if __name__ == "__main__":
    unittest.main()
