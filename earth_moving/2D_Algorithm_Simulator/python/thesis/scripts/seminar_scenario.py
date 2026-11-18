"""Small, headless scenario API. Importing this module runs no simulation."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
if not getattr(sys,'frozen',False):sys.path.insert(0,str(ROOT/'Hybrid Orchestrator'))
from env import SimulationEnv
from shapely.geometry import Point
import numpy as np

def build(config, material=None):
    target=config['target']
    e=SimulationEnv(grid_size=config['grid_size'],num_random_objects=0,
        target_zone=Point(*target['center']).buffer(target['radius']),**config.get('planner',{}))
    items=config['material'] if material is None else material
    seen=set()
    for x,y,n in items:
        if (x,y) in seen or n<0 or int(n)!=n or e.get_cell(x,y) is None:
            raise ValueError('Material cells must be unique, in bounds, and have nonnegative integer counts')
        seen.add((x,y));c=e.get_cell(x,y);c.num_objects=n;c.current_objects=n
    e.cells_with_objects=[c for c in e.all_cells if c.num_objects>0 and not c.is_target_zone]
    e.target_zone_cells=[c for c in e.all_cells if c.num_objects>0 and c.is_target_zone]
    e.cells_without_objects=[c for c in e.all_cells if c.num_objects==0]
    for c in e.cells_with_objects:
        c.visible_cells_target,c.distance_to_children_target=e.calculate_target_zone_visibility(c,angle_tolerance=e.target_visibility_angle(c))
        c.h_vis_target=sum(v['cell'].num_objects for v in c.visible_cells_target)
    spill=config.get('spillage',{})
    e.spillage_factor=spill.get('factor',.05)
    e.min_spillage_threshold=spill.get('threshold',.03)
    enabled=spill.get('enabled',False)
    e.calculate_potential_field(enabled,visualize=False)
    e.calculate_velocity_field();e.update_heat_map();e.calculate_path_to_highway(enabled)
    return e

def material(e):return [[c.x,c.y,c.num_objects] for c in e.all_cells if c.num_objects]
def xy(path):return [[c.x,c.y] for c in path or []]
def compare(e,fresh):
    sources=[c for c in e.all_cells if c.num_objects>0 and not c.is_target_zone]
    mismatches=[]
    for c in sources:
        d=fresh.get_cell(c.x,c.y)
        for field in ['best_path_target','best_path_highway']:
            if xy(getattr(c,field))!=xy(getattr(d,field)):mismatches.append([c.x,c.y,field])
        for field in ['total_objects_raw','total_objects_target','total_distance_target','total_objects_highway','total_distance_highway','velocity_target','velocity_highway','total_objects_path_target','own_path_heat','incoming_path_heat']:
            if not np.allclose(getattr(c,field),getattr(d,field),atol=1e-9,rtol=0):mismatches.append([c.x,c.y,field])
        for field in ['impacted_cells_target','impacted_cells_highway']:
            if getattr(c,field,{})!=getattr(d,field,{}):mismatches.append([c.x,c.y,field])
        def anchor(cell):
            a=getattr(cell,'chosen_highway_target',None)
            return (a.x,a.y) if a else None
        if anchor(c)!=anchor(d):mismatches.append([c.x,c.y,'chosen_highway_target'])
    heat=max(abs(c.heat_map-fresh.get_cell(c.x,c.y).heat_map) for c in e.all_cells)
    return dict(material_equal=material(e)==material(fresh),max_absolute_heat_difference=heat,
                source_mismatches=mismatches,passed=material(e)==material(fresh) and heat<1e-9 and not mismatches)
