"""Differential checks: use selective state directly, never editor fallback."""
import unittest,sys,copy,io,contextlib,random,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'thesis/scripts'))
from interactive_scenario import original_scene
from seminar_scenario import build,material,compare

class SelectiveTests(unittest.TestCase):
    def test_action_sequences(self):
        records=[]
        for mode in range(12):
            with self.subTest(mode=mode),contextlib.redirect_stdout(io.StringIO()):
                config=original_scene()
                config['spillage']={'enabled':mode%2==1}
                if mode%6>=2:
                    config['planner'].update(target_visibility_mode='dynamic',highway_visibility_mode='dynamic',own_path_heat_weight=.5)
                if mode%6>=4:
                    config['target']={'center':[14,10],'radius':2.5}
                    config['planner'].update(target_zone_gate_factor=.7,heat_alignment_power=2)
                if mode>=6:
                    rng_layout=random.Random(31+mode)
                    for item in config['material']:
                        item[2]=rng_layout.randint(1,4)
                env=build(config);rng=random.Random(17+mode)
                initial=sum(c.num_objects for c in env.all_cells)
                for step in range(8):
                    kind='highway' if step%2 else 'target'
                    sources=[c for c in env.cells_with_objects if not c.is_target_zone and len(getattr(c,'best_path_'+kind) or [])>1]
                    if not sources:continue
                    source=rng.choice(sorted(sources,key=lambda c:(c.x,c.y)))
                    retained={(c.x,c.y):c.best_path_target for c in env.cells_with_objects}
                    searched=set()
                    original=env.calculate_potential_field
                    def quiet(*args,**kwargs):
                        searched.update((c.x,c.y) for c in kwargs['affected_cells'])
                        kwargs['visualize']=False
                        return original(*args,**kwargs)
                    env.calculate_potential_field=quiet
                    env.execute_path(source,kind,use_spillage=config['spillage']['enabled'])
                    env.update_environment();del env.calculate_potential_field
                    self.assertEqual(sum(c.num_objects for c in env.all_cells),initial,(mode,step))
                    fresh=build(config,material(env));result=compare(env,fresh)
                    self.assertTrue(result['passed'],(mode,step,(source.x,source.y),result))
                    for c in env.cells_with_objects:
                        key=(c.x,c.y)
                        if not c.is_target_zone and key not in searched and key in retained:
                            self.assertIs(c.best_path_target,retained[key], 'Unchanged target path was recomputed')
                    records.append(dict(mode=mode,step=step,kind=kind,selective=len(searched),full=len(fresh.cells_with_objects),match=True))
                    for c in env.cells_with_objects:
                        if c.best_path_highway:
                            self.assertIs(c.best_path_highway[-1],c.chosen_highway_target)
        out=Path(__file__).resolve().parents[1]/'research/selective_update_20261114'
        out.mkdir(parents=True,exist_ok=True)
        (out/'validation.json').write_text(json.dumps({'actions':len(records),'records':records},indent=2))

if __name__=='__main__':unittest.main()
