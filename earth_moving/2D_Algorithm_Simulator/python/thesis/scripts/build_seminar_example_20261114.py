"""Generate a teaching fixture by executing the current planner, not a heat approximation."""
from pathlib import Path
import sys, json, hashlib, contextlib, io, os, argparse, math, html
_dll = os.add_dll_directory(str(Path(sys.prefix)/'Library/bin')) if os.name=='nt' and (Path(sys.prefix)/'Library/bin').exists() else None
ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'Hybrid Orchestrator'
sys.path.insert(0, str(SOURCE))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Wedge, Rectangle
from shapely.geometry import Point
from env import SimulationEnv
from spillage_model import simulate_spillage
from seminar_scenario import build,compare

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scenario',type=Path,default=ROOT/'thesis/seminar/scenarios/corridor.json')
parser.add_argument('--output',type=Path,default=ROOT/'thesis/seminar/running_example')
args=parser.parse_args()
CONFIG=json.loads(args.scenario.read_text(encoding='utf8'))
OUT = args.output
OUT.mkdir(parents=True, exist_ok=True)
FIXTURE=CONFIG['material'];FOCUS=CONFIG['focus_source'];ACTION=CONFIG['action_source']
SIZE=CONFIG['grid_size'];TARGET=Point(*CONFIG['target']['center']).buffer(CONFIG['target']['radius'])

def make(items):
    return build(CONFIG,items)

def xy(path): return [[c.x,c.y] for c in path or []]
def snap(e):
    return dict(material=[[c.x,c.y,c.num_objects] for c in e.all_cells if c.num_objects],
      heat=[[float(e.get_cell(x,y).heat_map) for x in range(e.grid_size)] for y in range(e.grid_size)],
      paths=[dict(source=[c.x,c.y],target=xy(c.best_path_target),highway=xy(c.best_path_highway),
        raw=c.total_objects_raw,propagated=c.total_objects_path_target,
        heading=list(c.velocity_target),heat=c.heat_map,
        successors=xy([i['cell'] for i in c.visible_cells_target])) for c in e.cells_with_objects])

log=io.StringIO()
with contextlib.redirect_stdout(log):
    env=make(FIXTURE); before=snap(env)
    for label,position in [('focus_source',FOCUS),('action_source',ACTION)]:
        candidate=env.get_cell(*position)
        if candidate is None or candidate.num_objects<=0 or candidate.is_target_zone:
            raise ValueError(f'{label} must name an occupied cell outside the target: {position}')
    feeder=env.get_cell(*ACTION)
    connector=xy(getattr(feeder,'best_path_'+CONFIG['action_type']))
    assert len(connector)>1, 'Selected source has no executable path; choose another source or configuration'
    total_before=sum(c.num_objects for c in env.all_cells)
    calls=[]; original=env.calculate_potential_field
    def traced(*args,**kwargs):
        calls.append(sorted([[c.x,c.y] for c in kwargs.get('affected_cells',env.cells_with_objects)]))
        kwargs['visualize']=False
        return original(*args,**kwargs)
    env.calculate_potential_field=traced
    env.execute_path(feeder,CONFIG['action_type'],use_spillage=CONFIG.get('spillage',{}).get('enabled',False))
    env.update_environment(); after=snap(env)
    (OUT/'planner_log.txt').write_text(log.getvalue(),encoding='utf8')
    assert sum(c.num_objects for c in env.all_cells)==total_before
    fresh=make(after['material']); rebuilt=snap(fresh)
    verification=compare(env,fresh)
    assert verification['passed'], verification
    old_counts={(x,y):n for x,y,n in before['material']}
    new_counts={(x,y):n for x,y,n in after['material']}
    changed={p for p in old_counts.keys()|new_counts.keys() if old_counts.get(p,0)!=new_counts.get(p,0)}
    searched={tuple(p) for p in calls[-1]}
    update_groups=dict(
        emptied=sorted(p for p in changed if new_counts.get(p,0)==0),
        changed_occupied=sorted(p for p in changed if new_counts.get(p,0)>0),
        dependent_sources=sorted(searched-changed),
        reused_sources=sorted((c.x,c.y) for c in env.all_cells if c.num_objects>0 and not c.is_target_zone and (c.x,c.y) not in searched))
    delta=float(np.max(np.abs(np.array(after['heat'])-np.array(rebuilt['heat']))))
    # Apply the actual estimator to a selected target route of this same scene.
    route=next(p['target'] for p in before['paths'] if p['source']==ACTION)
    counts={(x,y):n for x,y,n in FIXTURE if [x,y] in route}
    smooth,deposits,retained,raw=simulate_spillage(route,counts,8,env.spillage_factor,env.min_spillage_threshold)

report=dict(scenario=CONFIG['name'],configuration=CONFIG,
 grid_size=SIZE,target=CONFIG['target'],initial_material=FIXTURE,
 parameters=CONFIG['planner'],planning_spillage=CONFIG.get('spillage',{}).get('enabled',False),before=before,after=after,connector=connector,
 checks=dict(objects_before=total_before,objects_after=sum(c.num_objects for c in env.all_cells),
 selective_target_search_sources=calls,full_rebuild_target_sources=len(fresh.cells_with_objects),
 max_absolute_heat_difference_vs_fresh=delta,
 target_paths_equal=sorted((p['source'],p['target']) for p in after['paths'])==sorted((p['source'],p['target']) for p in rebuilt['paths'])),
 spillage=dict(route=route,raw=raw,retained=retained,deposits=[[x,y,n] for (x,y),n in deposits.items()],
 model='current curvature model; capacity argument is not enforced in this branch'),
 full_rebuild_verification=verification,update_groups=update_groups,
 provenance={f:hashlib.sha256((SOURCE/f).read_bytes()).hexdigest() for f in ['env.py','cell.py','search.py','spillage_model.py']})
(OUT/'example_data.json').write_text(json.dumps(report,indent=2),encoding='utf8')
(OUT/'planner_log.txt').write_text(log.getvalue(),encoding='utf8')
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.titlesize':17,'svg.fonttype':'none'})
VMAX=max(np.max(before['heat']),np.max(after['heat']))
def frame(title,state=before,heat=False):
    fig,ax=plt.subplots(figsize=(8,8),layout='constrained')
    ax.set(xlim=(0,SIZE),ylim=(0,SIZE),aspect='equal',title=title,xlabel='x (grid cells)',ylabel='y (grid cells)')
    ax.set_xticks(range(0,SIZE+1,max(1,SIZE//11)));ax.set_yticks(ax.get_xticks());ax.grid(alpha=.12)
    if heat:
        im=ax.imshow(state['heat'],origin='lower',extent=(0,SIZE,0,SIZE),cmap='magma',vmin=0,vmax=VMAX,interpolation='nearest')
        fig.colorbar(im,ax=ax,shrink=.65,label='Directional path value (heat)')
    ax.add_patch(Polygon(np.array(TARGET.exterior.coords),facecolor='#39c6ab',edgecolor='#066757',lw=2,zorder=5))
    ax.text(*CONFIG['target']['center'],'Target',ha='center',va='center',fontsize=11,zorder=6)
    for x,y,n in state['material']:
        ax.scatter(x+.5,y+.5,s=150+n*25,c='white',edgecolors='#163a54',linewidths=2,zorder=7)
        ax.text(x+.5,y+.5,str(n),ha='center',va='center',color='#163a54',weight='bold',zorder=8)
    return fig,ax
def path(ax,points,color='#dd6b20',lw=3,style='-'):
    a=np.array(points,dtype=float)+.5
    ax.plot(a[:,0],a[:,1],style,color=color,lw=lw,zorder=6)
    for p,q in zip(a[:-1],a[1:]): ax.annotate('',xy=q,xytext=p,arrowprops=dict(arrowstyle='->',color=color,lw=lw),zorder=6)
def save(fig,name):
    for ext in ['png','svg']: fig.savefig(OUT/f'{name}.{ext}',dpi=180,facecolor='white')
    plt.close(fig)

fig,ax=frame('One scene, two useful kinds of push');save(fig,'01_material')
fig,ax=frame('Which material can this source collect next?')
source=env.get_cell(*FOCUS);center=(FOCUS[0]+.5,FOCUS[1]+.5)
angle=math.degrees(math.atan2(source.closest_y-center[1],source.closest_x-center[0]))
half=env.target_visibility_angle(source)
if half is None:half=env.get_angle_tolerance_deg(source)
ax.add_patch(Wedge(center,source.distance_to_target*env.target_zone_gate_factor,angle-half,angle+half,facecolor='#5e9cd3',alpha=.16))
for q in next(p['successors'] for p in before['paths'] if p['source']==FOCUS):path(ax,[FOCUS,q],color='#527f9e',lw=1.5)
save(fig,'02_successors')
fig,ax=frame('Target routes share a collection corridor')
for p in before['paths']:path(ax,p['target'],color='#607f98',lw=1.4)
path(ax,next(p['target'] for p in before['paths'] if p['source']==FOCUS))
save(fig,'03_routes')
fig,ax=frame('Path values propagate; headings give direction')
for p in before['paths']:
    x,y=p['source'];u,v=p['heading'];ax.arrow(x+.5,y+.5,u*1.8,v*1.8,width=.06,color='#dd6b20',zorder=9)
    ax.text(x+1.2,y+.6,f"ψ = {p['propagated']:g}",fontsize=11)
save(fig,'04_values')
fig,ax=frame('Heat reveals a useful collection corridor',heat=True);save(fig,'05_heat')
fig,ax=frame('A side pile can join the corridor',heat=True);path(ax,connector,color='#66d9ff');save(fig,'06_connector')
fig,ax=frame('After staging: the environment has changed',state=after,heat=True);save(fig,'07_after')
fig,ax=frame('Recompute affected target routes',state=after)
styles=[('emptied','#bd3087','E','Emptied: no target search'),
        ('changed_occupied','#007db8','D','Material changed: update'),
        ('dependent_sources','#bb6a00','R','Dependent source: update'),
        ('reused_sources','#358255','U','Unaffected source: reuse')]
for group,color,letter,label in styles:
    for x,y in update_groups[group]:
        ax.add_patch(Rectangle((x-.08,y-.08),1.16,1.16,fill=False,edgecolor=color,lw=2,zorder=10))
        ax.text(x+1.3,y+.5,letter,color=color,va='center',weight='bold',fontsize=12,zorder=11)
    ax.plot([],[],color=color,lw=2,label=f'{letter} — {label}')
ax.legend(loc='upper left',bbox_to_anchor=(0,-.11),frameon=False,fontsize=10,ncol=2,
          title='Heat and connector selection still refresh globally.',title_fontsize=10)
ax.text(.4,SIZE-.6,f"Target searches: {len(calls[-1])} selective / {len(fresh.cells_with_objects)} fresh",fontsize=11)
save(fig,'08_update')
fig,ax=frame(f'Spillage estimate: {retained:g} of {raw:g} reaches the endpoint')
ax.plot(np.array(smooth)[:,0],np.array(smooth)[:,1],color='#dd6b20',lw=3,zorder=6)
for (x,y),n in deposits.items():
    ax.scatter(x+.5,y+.5,marker='x',s=120,color='#b8344b',zorder=10)
    ax.text(x+1,y+.5,f'deposit {n:g}',fontsize=10)
save(fig,'09_spillage')
titles=['Material','Successors','Routes','Values and headings','Heat','Selected action','After action','Update scope','Spillage estimate']
names=['01_material','02_successors','03_routes','04_values','05_heat','06_connector','07_after','08_update','09_spillage']
sections=''.join(f'<section><h2>{i+1}. {title}</h2><img src="{name}.png" alt="{title}"><p><a href="{name}.svg">SVG</a> · <a href="{name}.png">PNG</a></p></section>' for i,(title,name) in enumerate(zip(titles,names)))
(OUT/'index.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>'+html.escape(CONFIG['name'])+'</title><style>body{font:18px system-ui;max-width:1000px;margin:30px auto;padding:0 18px;color:#19374a}img{max-width:100%;max-height:80vh}section{margin:35px 0;border-top:1px solid #ddd}a{color:#006c80}</style><h1>'+html.escape(CONFIG['name'])+'</h1><p>Current planner outputs. Material conserved; selective update matches fresh recalculation for this scenario.</p><p><a href="example_data.json">Inputs, outputs and verification</a></p>'+sections+'</html>',encoding='utf8')
print(json.dumps(report['checks'],indent=2));print('connector',connector,'spillage',report['spillage'])
