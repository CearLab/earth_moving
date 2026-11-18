"""Conservation and fresh-rebuild regressions for abstract no-spillage actions."""
import unittest,sys,json,copy,contextlib,io
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'thesis/scripts'))
from seminar_scenario import build,material,compare

class TransferTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((Path(__file__).resolve().parents[1]/'thesis/seminar/scenarios/corridor.json').read_text())

    def test_empty_and_occupied_connector_endpoints(self):
        for side_y in [7,8,9,10]:
            with self.subTest(side_y=side_y),contextlib.redirect_stdout(io.StringIO()):
                cfg=copy.deepcopy(self.config);cfg['material'][-1][1]=side_y
                e=build(cfg);c=e.get_cell(15,side_y)
                self.assertGreater(len(c.best_path_highway),1)
                expected=sum(n.num_objects for n in c.best_path_highway)
                endpoint=c.best_path_highway[-1]
                e.execute_path(c,'highway',use_spillage=False)
                self.assertEqual(endpoint.num_objects,expected)
                self.assertEqual(sum(n.num_objects for n in e.all_cells),16)
                e.update_environment()
                result=compare(e,build(cfg,material(e)))
                self.assertTrue(result['passed'],result)

    def test_live_material_not_cached_candidate_yield(self):
        with contextlib.redirect_stdout(io.StringIO()):
            e=build(self.config);start=e.get_cell(15,8);end=e.get_cell(8,7)
            e.execute_path(start,'highway',use_spillage=False,
                precomputed_path={'path':[start,end],'objects':999,'distance':7})
            self.assertEqual(end.num_objects,6)
            self.assertEqual(end.current_objects,6)
            self.assertEqual(sum(c.num_objects for c in e.all_cells),16)

    def test_delivery_preserves_material_already_in_target(self):
        with contextlib.redirect_stdout(io.StringIO()):
            e=build(self.config);start=e.get_cell(8,3);end=start.best_path_target[-1]
            end.num_objects=end.current_objects=3
            e.execute_path(start,'target',use_spillage=False)
            self.assertEqual(end.num_objects,17)
            self.assertEqual(sum(c.num_objects for c in e.all_cells),19)
            e.update_environment()
            result=compare(e,build(self.config,material(e)))
            self.assertTrue(result['passed'],result)

if __name__=='__main__':unittest.main()
