"""Pin and verify the live seminar scene; export offline screenshots and cue sheet."""
import os,json,html
from pathlib import Path
os.environ['SDL_VIDEODRIVER']='dummy'
from interactive_scenario import Editor,original_scene,calculate
from seminar_scenario import material,xy
import pygame
ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'thesis/seminar';OUT=BASE/'live_demo'
OUT.mkdir(parents=True,exist_ok=True)
c=original_scene();c['name']='Seminar live demo — 55 objects, seed 31'
c['action_source']=c['focus_source']=[0,15];c['presentation_preset']='visibility'
(BASE/'scenarios/seminar_live_55.json').write_text(json.dumps(c,indent=2))
editor=Editor(c);editor.future.result();editor.collect();source=editor.env.get_cell(0,15)
route=xy(source.best_path_target);yield_count=source.total_objects_target
after=calculate(c,editor.env,(0,15),'target');assert after[4]['passed']
assert sum(n for _,_,n in material(after[1]))==55
rows=[]
def capture(file,title):
    editor.draw();pygame.image.save(editor.screen,str(OUT/(file+'.png')));rows.append((file,title))
editor.scope=False;capture('01_scene','55 objects; target in the center')
editor.scope=True;capture('02_scope','Selected source (0,15): visibility, with connection lines hidden')
editor.scope=False;editor.paths='selected';capture('03_route',f'Target route: {yield_count:g} new objects')
editor.paths='off';editor.vectors=True;capture('04_vectors','First-step directions from occupied sources')
editor.vectors=False;editor.heat=True;editor.paths='selected';capture('05_heat','Directional heat and selected target route')
editor.config,editor.env,editor.groups,editor.status,editor.check,editor.log=after
editor.page='scene';editor.scope=False;editor.paths='off';editor.heat=False;editor.show_updates=True;editor.show_reused=False
capture('06_update','Affected sources only; selective/full verification passes')
editor.pool.shutdown();pygame.quit()
data=dict(source=[0,15],route=route,new_delivery=yield_count,verification=after[4],status=after[3],groups={k:sorted(v) for k,v in after[2].items()})
(OUT/'verified_action.json').write_text(json.dumps(data,indent=2))
steps=[
('0:00–0:40','Start the prepared scene','Use Start seminar demo.cmd before the talk. The selected pile is (0,15): the far-left column, row 15 from the top. State that this is the old 55-object layout, running the current algorithm.'),
('0:40–1:30','Explain visibility','Display panel: Scope selected, Scope lines off. Point to cone/range and circled successors. Briefly enable Scope lines if needed, then disable them. Do not change parameters during the main demo.'),
('1:30–2:20','Reveal the target route','Scope off; Paths selected. Explain the route gathering '+str(yield_count)+' new objects. Its saved cell sequence is '+str(route)+'. Click an empty cell once to clear the preview, then reselect (0,15).'),
('2:20–3:10','Reveal direction and heat','Paths off; Vectors on briefly. Then Vectors off, Heat on, Paths selected. Explain that heat scores a transport opportunity, not material already in a cell. The palette auto-rescales per state.'),
('3:10–4:40','Execute and show the selective update','Return to Scene controls; Target mode (T); press Enter exactly once. Open Display: Heat off, Paths off, Scope off, Updates ON. Read the verification message: '+after[3]+' Point to E/D/R; unchanged sources have no colored borders. Green circles are still piles.'),
('4:40–5:20','State what is reused','Target searches rerun for the dependency closure; other target paths are retained. Propagated values, heat and staging routes still rebuild globally. Full recalculation also runs here for verification, so this is not a timing benchmark.'),
('5:20–6:00','Return to the wider thesis','Leave the verified result on screen, then switch back to slides: a useful 2D opportunity still needs rover approach, alignment, tracking, collision avoidance and action selection. No further live actions in the main talk.'),
]
style='<style>body{font:18px/1.6 system-ui;max-width:1050px;margin:30px auto;padding:0 20px;color:#193447}table{border-collapse:collapse}td,th{border:1px solid #ccd6dc;padding:10px;text-align:left;vertical-align:top}img{max-width:100%}section{margin:40px 0}a{color:#007083}</style>'
body='<h1>Six-minute live demo: rehearsal and backup</h1><p>Scheduled at minutes 11–17 of the 45-minute talk. Later work starts at minute 22.</p><p><b>Recommended scene:</b> the original 55-object layout, with one pinned target action. It shows a nontrivial field and a small affected subset. Keep the 16-object corridor as an explanatory backup, not a second live demo.</p><table><tr><th>Demo clock</th><th>Step</th><th>Action and explanation</th></tr>'
body+=''.join('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in row)+'</tr>' for row in steps)+'</table>'
body+='<h2>Rehearsal and fallback</h2><p>Reopen the dedicated launcher to restore the exact initial scene. Rehearse on the projector: text size, mouse selection at the left edge, and switching back to slides. Do not use Load latest for the presentation. Save experimentation separately.</p><p><b>Hard stop:</b> leave the simulator by talk minute 17. If you fall behind, skip the vector layer. If startup/verification fails, do not debug live: show the screenshots below and state they are saved outputs from the verified run. No measured speedup claim.</p><p>Spillage and hyperparameter changes are useful for questions after the talk. They are excluded from the main six minutes to protect the later work. The slides still explain the estimator and its replaceability.</p>'
body+=''.join(f'<section><h2>{html.escape(title)}</h2><img src="{file}.png" alt="{html.escape(title)}"></section>' for file,title in rows)
(OUT/'DEMO_RUNBOOK.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Live demo runbook</title>'+style+body+'</html>',encoding='utf8')
(OUT/'DEMO_CUES.txt').write_text('SIX-MINUTE LIVE DEMO (talk minutes 11–17)\n\n'+'\n\n'.join(' | '.join(row) for row in steps)+'\n\nIf behind: skip vectors. If failure: use DEMO_RUNBOOK.html screenshots. Stop by minute 17. Later work starts at minute 22.',encoding='utf8')
print(json.dumps(data,indent=2))
