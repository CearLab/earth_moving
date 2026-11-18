"""Mouse-editable Pygame scenes backed by the current 2D planner."""
from pathlib import Path
import os,sys,json,copy,random,math,datetime,traceback,contextlib,io,argparse,webbrowser
from concurrent.futures import ThreadPoolExecutor
from simulator_paths import ROOT,SEMINAR,FROZEN,writable_folder
OUT=None
os.environ['PYGAME_HIDE_SUPPORT_PROMPT']='1'
import pygame
from shapely.geometry import Point
from seminar_scenario import build,material,compare,xy

def output_folder():
    global OUT
    if OUT is None:OUT=writable_folder('Saved Scenarios')
    return OUT

def original_scene():
    # Matches demo_main.py's scene parameters and core_env.py's sampling order.
    c=json.loads((SEMINAR/'scenarios/corridor.json').read_text())
    c.update(name='Original demo layout: 55 objects, seed 31',grid_size=25,
             target={'center':[12,12],'radius':3},action_type='target')
    c['planner'].update(target_angle_tolerance=45,max_path_length_factor=2)
    rng=random.Random(31);counts={};target=Point(12,12).buffer(3)
    while sum(counts.values())<55:
        p=(rng.randint(0,24),rng.randint(0,24))
        if not target.contains(Point(p[0]+.5,p[1]+.5)):counts[p]=counts.get(p,0)+1
    c['material']=[[x,y,n] for (x,y),n in counts.items()]
    c['focus_source']=c['action_source']=list(next(iter(counts)))
    return c

def moved(config,source,destination):
    c=copy.deepcopy(config);counts={(x,y):n for x,y,n in c['material']}
    if source!=destination and source in counts:
        counts[destination]=counts.get(destination,0)+counts.pop(source)
    c['material']=[[x,y,n] for (x,y),n in sorted(counts.items())]
    return c

def clone_environment(previous):
    # Preseed graph nodes so copying adjacency links cannot recurse across the grid.
    e=copy.copy(previous);memo={id(previous):e}
    for cell in previous.all_cells:memo[id(cell)]=copy.copy(cell)
    for cell in previous.all_cells:
        memo[id(cell)].__dict__=copy.deepcopy(cell.__dict__,memo)
    e.__dict__=copy.deepcopy(previous.__dict__,memo)
    return e

def calculate(config,previous=None,source=None,kind='target'):
    """Worker owns its environment; never mutates the displayed one."""
    log=io.StringIO()
    with contextlib.redirect_stdout(log):
        groups={};check=None;searched=[]
        if previous is None:
            e=build(config);message='Rebuilt after editing. Choose a pile to preview a path.'
        else:
            e=clone_environment(previous);s=e.get_cell(*source)
            path=getattr(s,'best_path_'+kind,[]) if s else []
            if len(path)<2:raise ValueError('No path of this type. Choose another pile or Target / Staging.')
            old={(x,y):n for x,y,n in material(e)};total=sum(old.values())
            original=e.calculate_potential_field
            def traced(*args,**kwargs):
                searched.extend((c.x,c.y) for c in kwargs.get('affected_cells',e.cells_with_objects))
                kwargs['visualize']=False
                return original(*args,**kwargs)
            e.calculate_potential_field=traced
            spill=config.get('spillage',{}).get('enabled',False)
            deposits=copy.deepcopy(getattr(s,'impacted_cells_'+kind,{})) if spill else {}
            endpoint=(path[-1].x,path[-1].y)
            e.execute_path(s,kind,use_spillage=spill);e.update_environment()
            del e.calculate_potential_field
            new={(x,y):n for x,y,n in material(e)}
            if sum(new.values())!=total:raise ValueError('Conservation check failed. Action was not accepted.')
            config=copy.deepcopy(config);config['material']=material(e)
            fresh=build(config);check=compare(e,fresh)
            changed={p for p in old.keys()|new.keys() if old.get(p,0)!=new.get(p,0)}
            groups={'E':{p for p in changed if not new.get(p,0)},
                    'D':{p for p in changed if new.get(p,0)},
                    'R':set(searched)-changed,
                    'U':{(c.x,c.y) for c in fresh.cells_with_objects}-set(searched)}
            groups['S']={p for p,n in deposits.items() if n>0 and p!=endpoint}
            message=f'{len(set(searched))} target searches / {len(fresh.cells_with_objects)} fresh. '
            if check['passed']:message+='Matches fresh calculation.'
            else:
                e=fresh
                message+='Selective mismatch: displaying FULL rebuild. See saved diagnostic.'
                output_folder().mkdir(parents=True,exist_ok=True)
                stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                (output_folder()/f'mismatch_{stamp}.json').write_text(json.dumps({'configuration':config,'check':check},indent=2))
    return config,e,groups,message,check,log.getvalue()

class Editor:
    MAP=750;PANEL=400
    PARAMETERS={
        'target_angle_tolerance':('Target angle',45,5,5,90),
        'highway_angle_tolerance':('Staging angle',60,5,5,120),
        'dynamic_visibility_min_angle':('Dynamic min angle',30,5,5,90),
        'dynamic_visibility_max_angle':('Dynamic max angle',60,5,5,120),
        'target_zone_gate_factor':('Visibility range factor',1,.1,.1,3),
        'max_path_length_factor':('Path length factor',2,.25,1,5),
        'own_path_heat_weight':('Own-path heat weight',0,.1,0,1),
        'heat_alignment_power':('Alignment exponent',1,.25,.25,4),
        'heat_radial_gate_angle_deg':('Heat radial gate',75,5,5,90),
        'highway_min_heat_ratio':('Anchor min heat ratio',.3,.05,.05,1),
        'highway_threshold_ratio':('High-heat threshold',.5,.05,.05,1),
        'highway_heat_weight':('Heat weight (rest: distance)',.7,.05,0,1),
        'factor':('Spillage coefficient',.05,.01,0,.5),
        'threshold':('Minimum spill',.03,.01,0,.5),
    }
    def __init__(self,config):
        pygame.init();pygame.display.set_caption('2D Algorithm Simulator — current planner')
        self.screen=pygame.display.set_mode((self.MAP+self.PANEL,self.MAP))
        self.font=pygame.font.SysFont('segoeui',17);self.small=pygame.font.SysFont('segoeui',14)
        self.clock=pygame.time.Clock();self.pool=ThreadPoolExecutor(max_workers=1)
        self.config=copy.deepcopy(config);self.initial=copy.deepcopy(config)
        self.env=None;self.groups={};self.history=[];self.future=None;self.pending=None
        self.mode='piles';self.kind='target';self.selected=None;self.drag=None
        self.heat=True;self.scope=False;self.scope_all=False;self.show_reused=False;self.page='scene'
        self.paths='selected';self.vectors=False;self.show_updates=True;self.scope_lines=True
        self.running=True;self.status='Calculating scene...';self.last_saved=None
        self.check=None;self.log='';self.buttons=[]
        if config.get('presentation_preset')=='visibility':
            self.command('visibility_only');self.scope_lines=False;self.page='display'
            self.selected=tuple(config['action_source'])
        self.submit(config,remember=False)

    def submit(self,c,action=False,remember=True):
        if self.future:return
        self.pending=(copy.deepcopy(self.config),remember)
        self.status='Calculating... editing is paused.'
        self.future=self.pool.submit(calculate,copy.deepcopy(c),self.env if action else None,self.selected,self.kind)

    def collect(self):
        if self.future and self.future.done():
            try:
                c,e,g,msg,check,log=self.future.result()
                if self.pending[1]:self.history.append(self.pending[0])
                self.config,self.env,self.groups,self.status,self.check,self.log=c,e,g,msg,check,log
            except Exception as ex:self.status=str(ex)
            self.future=None;self.pending=None

    def cell_at(self,pos):
        x,y=pos;n=self.config['grid_size']
        return (min(n-1,int(x*n/self.MAP)),min(n-1,int(y*n/self.MAP))) if 0<=x<self.MAP and 0<=y<self.MAP else None

    def count_at(self,p):return next((n for x,y,n in self.config['material'] if (x,y)==p),0)

    def edit_count(self,p,delta):
        c=copy.deepcopy(self.config);counts={(x,y):n for x,y,n in c['material']}
        counts[p]=max(0,counts.get(p,0)+delta)
        c['material']=[[x,y,n] for (x,y),n in counts.items() if n]
        self.selected=p;self.submit(c)

    def save(self):
        output_folder().mkdir(parents=True,exist_ok=True);stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        c=copy.deepcopy(self.config)
        candidates=[v for v in material(self.env) if not self.env.get_cell(v[0],v[1]).is_target_zone]
        chosen=self.selected if self.selected and any(v[:2]==list(self.selected) for v in candidates) else (candidates[0][:2] if candidates else None)
        if chosen:c['action_source']=list(chosen);c['focus_source']=list(chosen)
        c['action_type']=self.kind
        self.last_saved=output_folder()/f'scene_{stamp}.json';self.last_saved.write_text(json.dumps(c,indent=2))
        pygame.image.save(self.screen,str(self.last_saved.with_suffix('.png')))
        self.last_saved.with_suffix('.log.txt').write_text(self.log,encoding='utf8')
        self.status='Saved scene + screenshot in '+('Saved Scenarios.' if FROZEN else 'interactive_saved.')

    def command(self,key):
        if self.future:return
        if key=='page':self.page='settings' if self.page=='scene' else 'scene';return
        if key=='display_page':self.page='display' if self.page=='scene' else 'scene';return
        if key=='scope_lines':self.scope_lines=not self.scope_lines;return
        if key=='scope':
            if not self.scope:self.scope=True;self.scope_all=False
            elif not self.scope_all:self.scope_all=True
            else:self.scope=False;self.scope_all=False
            return
        if key=='paths':self.paths={'off':'selected','selected':'all','all':'off'}[self.paths];return
        if key=='vectors':self.vectors=not self.vectors;return
        if key=='updates':
            self.show_updates=not self.show_updates
            if self.show_updates:self.show_reused=False
            return
        if key=='visibility_only':
            self.heat=False;self.paths='off';self.vectors=False;self.show_updates=False
            self.scope=True;self.scope_all=False;self.kind='target';return
        if key=='reused':self.show_reused=not self.show_reused;return
        if key=='spill':
            c=copy.deepcopy(self.config);s=c.setdefault('spillage',{});s['enabled']=not s.get('enabled',False)
            self.submit(c);return
        if key=='dynamic':
            c=copy.deepcopy(self.config);p=c['planner'];mode='dynamic' if p.get('target_visibility_mode','fixed')=='fixed' else 'fixed'
            p['target_visibility_mode']=p['highway_visibility_mode']=mode;self.submit(c);return
        if key.startswith('param:'):
            _,name,direction=key.split(':');label,default,step,lo,hi=self.PARAMETERS[name]
            c=copy.deepcopy(self.config);section=c.setdefault('spillage' if name in ['factor','threshold'] else 'planner',{})
            section[name]=round(max(lo,min(hi,section.get(name,default)+int(direction)*step)),5)
            if name=='highway_heat_weight':section['highway_distance_weight']=round(1-section[name],5)
            if name=='dynamic_visibility_min_angle':section['dynamic_visibility_max_angle']=max(section.get('dynamic_visibility_max_angle',60),section[name])
            if name=='dynamic_visibility_max_angle':section['dynamic_visibility_min_angle']=min(section.get('dynamic_visibility_min_angle',30),section[name])
            self.submit(c);return
        if key in ['piles','target','add','erase']:self.mode=key
        elif key in ['target_path','highway_path']:self.kind='target' if key=='target_path' else 'highway'
        elif key=='heat':self.heat=not self.heat
        elif key=='run':
            if self.selected:self.submit(self.config,action=True)
            else:self.status='Select an occupied pile first.'
        elif key=='undo' and self.history:
            self.submit(self.history.pop(),remember=False)
        elif key=='reset':self.submit(self.initial)
        elif key=='save' and self.env:self.save()
        elif key=='load':
            saved=sorted(output_folder().glob('scene_*.json'))
            if saved:self.submit(json.loads(saved[-1].read_text()))
            else:self.status='No saved scene yet.'

    def event(self,event):
        if event.type==pygame.QUIT:self.running=False;return
        if event.type==pygame.KEYDOWN:
            if event.key==pygame.K_ESCAPE:self.running=False
            if event.key in [pygame.K_F1,pygame.K_F2]:
                guide='SIMULATOR_QUICK_GUIDE.html' if event.key==pygame.K_F1 else 'SIMULATOR_ALGORITHM_GUIDE.html'
                webbrowser.open((SEMINAR/guide).as_uri())
            mapping={pygame.K_RETURN:'run',pygame.K_t:'target_path',pygame.K_h:'highway_path',pygame.K_v:'heat',pygame.K_u:'undo',pygame.K_s:'save',pygame.K_l:'load'}
            if event.key in mapping:self.command(mapping[event.key])
        if self.future:return
        if event.type==pygame.MOUSEBUTTONDOWN:
            pos=event.pos
            if event.button==1:
                for rect,key,_ in self.buttons:
                    if rect.collidepoint(pos):self.command(key);return
            p=self.cell_at(pos)
            if p is None:return
            if event.button==3:self.edit_count(p,-self.count_at(p));return
            if event.button!=1:return
            if self.mode=='target':self.drag=('target',p)
            elif self.mode=='add':self.edit_count(p,1)
            elif self.mode=='erase':self.edit_count(p,-self.count_at(p))
            elif self.count_at(p):self.selected=p;self.drag=('pile',p)
            else:
                self.selected=None;self.drag=None
                self.status='Preview cleared. Select another occupied cell.'
        elif event.type==pygame.MOUSEBUTTONUP and event.button==1 and self.drag:
            kind,origin=self.drag;self.drag=None;p=self.cell_at(event.pos)
            if p is None:return
            if kind=='pile' and p!=origin:
                self.selected=p;self.submit(moved(self.config,origin,p))
            elif kind=='target':
                c=copy.deepcopy(self.config);n=c['grid_size'];r=c['target']['radius']
                center=[max(r,min(n-r,v*n/self.MAP)) for v in event.pos]
                c['target']['center']=center;self.submit(c)
        elif event.type==pygame.MOUSEWHEEL:
            p=self.cell_at(pygame.mouse.get_pos())
            if p is None:return
            if self.mode=='target':
                c=copy.deepcopy(self.config);n=c['grid_size']
                c['target']['radius']=max(.75,min(n/2-1,c['target']['radius']+event.y*.25))
                r=c['target']['radius'];c['target']['center']=[max(r,min(n-r,v)) for v in c['target']['center']]
                self.submit(c)
            else:self.edit_count(p,event.y)

    def text(self,words,x,y,color=(220,229,239),small=False):
        self.screen.blit((self.small if small else self.font).render(str(words),True,color),(x,y))

    def wrap(self,words,y,color=(220,229,239)):
        line=''
        for word in words.split():
            candidate=(line+' '+word).strip()
            if self.font.size(candidate)[0]>self.PANEL-32:
                self.text(line,self.MAP+16,y,color);y+=23;line=word
            else:line=candidate
        self.text(line,self.MAP+16,y,color);return y+27

    def draw(self):
        self.screen.fill((24,33,48));n=self.config['grid_size'];size=self.MAP/n
        pygame.draw.rect(self.screen,'white',(0,0,self.MAP,self.MAP))
        if self.env:
            outside=[c for c in self.env.all_cells if not c.is_target_zone]
            low=min((c.heat_map for c in outside),default=0);high=max((c.heat_map for c in outside),default=0)
            for c in self.env.all_cells:
                rect=pygame.Rect(round(c.x*size),round(c.y*size),math.ceil(size),math.ceil(size))
                if self.heat and not c.is_target_zone:
                    v=(c.heat_map-low)/(high-low) if high>low else 0
                    pygame.draw.rect(self.screen,(int(255*v),0,int(255*(1-v))),rect)
                pygame.draw.rect(self.screen,(135,145,160),rect,1)
            target=self.config['target'];center=tuple(round(v*size) for v in target['center'])
            pygame.draw.circle(self.screen,(255,55,55),center,round(target['radius']*size),3)
            if self.paths=='all':
                for source in self.env.cells_with_objects:
                    points=[((v.x+.5)*size,(v.y+.5)*size) for v in getattr(source,'best_path_'+self.kind,[]) or []]
                    if len(points)>1:pygame.draw.lines(self.screen,(165,195,225) if self.heat else (75,110,145),False,points,1)
            if self.selected:
                c=self.env.get_cell(*self.selected)
                path=getattr(c,'best_path_'+self.kind,[]) if c else []
                points=[((v.x+.5)*size,(v.y+.5)*size) for v in path or []]
                for a,b in (zip(points,points[1:]) if self.paths!='off' else []):
                    pygame.draw.line(self.screen,(255,190,20),a,b,3)
                    angle=math.atan2(b[1]-a[1],b[0]-a[0])
                    pygame.draw.polygon(self.screen,(255,190,20),[b,(b[0]-10*math.cos(angle-.45),b[1]-10*math.sin(angle-.45)),(b[0]-10*math.cos(angle+.45),b[1]-10*math.sin(angle+.45))])
                pygame.draw.rect(self.screen,'white' if self.heat else (25,65,115),(*[int(v*size) for v in self.selected],int(size),int(size)),3)
            for x,y,count in material(self.env):
                center=(round((x+.5)*size),round((y+.5)*size))
                pygame.draw.circle(self.screen,(0,240,70),center,max(5,min(int(size*.36),int(size*.17*count))))
                label=self.small.render(str(count),True,(0,0,0));self.screen.blit(label,label.get_rect(center=center))
            sources=self.env.cells_with_objects if self.scope_all else ([self.env.get_cell(*self.selected)] if self.selected else [])
            for source in (sources if self.scope else []):
                if source:
                    origin=((source.x+.5)*size,(source.y+.5)*size)
                    if self.kind=='target' and not source.is_target_zone:
                        angle=math.atan2(source.closest_y-source.y-.5,source.closest_x-source.x-.5)
                        degrees=self.env.target_visibility_angle(source)
                        if degrees is None:degrees=self.env.get_angle_tolerance_deg(source)
                        half=math.radians(degrees)
                        radius=source.distance_to_target*self.env.target_zone_gate_factor*size
                        points=[origin]+[(origin[0]+radius*math.cos(angle-half+2*half*i/40),origin[1]+radius*math.sin(angle-half+2*half*i/40)) for i in range(41)]
                        overlay=pygame.Surface((self.MAP,self.MAP),pygame.SRCALPHA)
                        tint=(255,255,255) if self.heat else (65,115,190)
                        pygame.draw.polygon(overlay,(*tint,8 if self.scope_all else 42),points);self.screen.blit(overlay,(0,0))
                    scope_color=(255,255,255) if self.heat else (35,85,150)
                    for info in getattr(source,'visible_cells_'+self.kind,[]):
                        c=info['cell'];center=((c.x+.5)*size,(c.y+.5)*size)
                        if self.scope_lines:pygame.draw.line(self.screen,scope_color,origin,center,1)
                        pygame.draw.circle(self.screen,scope_color,center,int(size*.46),2)
            if self.vectors:
                for source in self.env.cells_with_objects:
                    if source.is_target_zone:continue
                    vx,vy=getattr(source,'velocity_'+self.kind,(0,0))
                    if math.hypot(vx,vy)<1e-6:continue
                    a=((source.x+.5)*size,(source.y+.5)*size)
                    b=(a[0]+vx*size*.85,a[1]+vy*size*.85)
                    color=(0,245,240) if self.heat else (0,100,100)
                    pygame.draw.line(self.screen,color,a,b,2)
                    angle=math.atan2(vy,vx)
                    pygame.draw.polygon(self.screen,color,[b,(b[0]-7*math.cos(angle-.45),b[1]-7*math.sin(angle-.45)),(b[0]-7*math.cos(angle+.45),b[1]-7*math.sin(angle+.45))])
            colors={'E':(255,70,220),'D':(0,235,255),'R':(255,235,0),'U':(145,255,165),'S':(200,115,255)}
            for tag,items in self.groups.items():
                if not self.show_updates:continue
                if tag=='U' and not self.show_reused:continue
                for x,y in items:
                    rect=pygame.Rect(round(x*size+1),round(y*size+1),int(size-2),int(size-2))
                    pygame.draw.rect(self.screen,colors[tag],rect,2)
                    self.text(tag,rect.x+1,rect.y,colors[tag],small=True)
        if self.drag:
            pos=pygame.mouse.get_pos()
            pygame.draw.circle(self.screen,(255,255,255),pos,12,2)
        x=self.MAP+16;self.text('2D ALGORITHM SIMULATOR',x,12)
        self.text('F1: User guide | F2: Algorithm and math',x,40,small=True)
        self.buttons=[]
        options=[('piles','Move piles'),('target','Move target'),('add','Add pile'),('erase','Erase pile'),('target_path','Target path [T]'),('highway_path','Staging path [H]'),('run','Run action [Enter]'),('undo','Undo [U]'),('save','Save scene [S]'),('load','Load latest [L]'),('display_page','Display >'),('page','Parameters >'),('reset','Reset scene'),('spill','Spillage: '+('ON' if self.config.get('spillage',{}).get('enabled') else 'off'))]
        if self.page=='display':
            options=[('display_page','< Scene controls'),('visibility_only','Visibility only'),('heat','Heat: '+('ON' if self.heat else 'off')),('paths','Paths: '+self.paths),('vectors','Vectors: '+('ON' if self.vectors else 'off')),('scope','Scope: '+('all' if self.scope_all else 'selected' if self.scope else 'off')),('scope_lines','Scope lines: '+('ON' if self.scope_lines else 'off')),('updates','Updates: '+('ON' if self.show_updates else 'off')),('reused','Unchanged: '+('ON' if self.show_reused else 'off'))]
        if self.page=='settings':options=[('page','< Scene controls'),('spill','Spillage: '+('ON' if self.config.get('spillage',{}).get('enabled') else 'off')),('dynamic','Scope: '+self.config['planner'].get('target_visibility_mode','fixed')),('scope','Show scope')]
        for i,(key,label) in enumerate(options):
            rect=pygame.Rect(x+(i%2)*183,75+(i//2)*42,175,34)
            active=key==self.mode or key==self.kind+'_path'
            pygame.draw.rect(self.screen,(44,111,133) if active else (51,65,85),rect,border_radius=5)
            self.text(label,rect.x+8,rect.y+5);self.buttons.append((rect,key,label))
        if self.page=='settings':
            y=165
            for name,(label,default,step,lo,hi) in self.PARAMETERS.items():
                section=self.config.get('spillage' if name in ['factor','threshold'] else 'planner',{})
                self.text(f'{label}: {section.get(name,default):g}',x,y+4,small=True)
                for i,direction in enumerate([-1,1]):
                    rect=pygame.Rect(x+290+i*38,y,32,26)
                    pygame.draw.rect(self.screen,(51,65,85),rect,border_radius=4)
                    self.text('-' if direction<0 else '+',rect.x+11,y+1)
                    self.buttons.append((rect,f'param:{name}:{direction}',label))
                y+=30
            self.wrap('Changes rebuild all paths. Heat/proximity weights sum to 1. Curvature spillage is an estimate; capacity is not enforced by this model.',y)
            self.wrap(self.status,690,(130,240,190));pygame.display.flip();return
        if self.page=='display':
            y=self.wrap('Display controls hide or reveal layers; they do not recalculate the planner.',310)
            y=self.wrap('Scope lines connects the selected source to its visible successors. Turn it OFF to keep just the cone and successor circles.',y)
            y=self.wrap('Visibility only is the starting view for the live demo. Then reveal paths, vectors and heat in order.',y)
            y=self.wrap('In Move piles mode, click an empty cell to clear the selected preview. Global layers such as all paths remain controlled here.',y)
            self.wrap(self.status,y+15,(130,240,190));pygame.display.flip();return
        y=385
        y=self.wrap('Drag to edit; wheel changes counts (or target radius). Right-click erases. Edits rebuild ALL target paths.',y)
        if self.env:
            y=self.wrap(f'Objects: {sum(v[2] for v in material(self.env))} | Grid: {n} x {n} | Selected: {self.selected}',y)
            y=self.wrap(f'Preview: {self.kind}. Heat scale: {low:.2f} to {high:.2f} (auto per state).',y)
        legend='E emptied (pink) | D changed (cyan) | R dependent (yellow) | S spill (purple). Reused cells hidden by default.'
        if self.check and not self.check['passed']:
            legend='Selective attempt: E emptied | D changed | R searched | S spill. Display uses FULL rebuild.'
        y=self.wrap(legend,y,(255,220,120))
        y=self.wrap('Actions: selective attempt + FULL verification. Heat and connectors refresh globally.',y)
        self.wrap(self.status,y,(255,164,120) if 'mismatch' in self.status.lower() else (130,240,190))
        pygame.display.flip()

    def run(self):
        try:
            while self.running:
                self.collect()
                for event in pygame.event.get():self.event(event)
                self.draw();self.clock.tick(30)
        finally:self.pool.shutdown(wait=True);pygame.quit()

def main():
    global OUT
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scenario',type=Path);p.add_argument('--self-test',action='store_true');p.add_argument('--test-output',type=Path);a=p.parse_args()
    if a.test_output:OUT=a.test_output.resolve()
    scenario=a.scenario or (SEMINAR/'scenarios/seminar_live_55.json' if FROZEN and not a.self_test else None)
    config=json.loads(scenario.read_text(encoding='utf8')) if scenario else original_scene()
    if a.self_test:
        OUT=output_folder();os.environ['SDL_VIDEODRIVER']='dummy';OUT.mkdir(parents=True,exist_ok=True)
        assert (SEMINAR/'SIMULATOR_QUICK_GUIDE.html').exists()
        assert (SEMINAR/'SIMULATOR_ALGORITHM_GUIDE.html').exists()
        pinned=json.loads((SEMINAR/'scenarios/seminar_live_55.json').read_text())
        pinned_state=calculate(pinned)
        pinned_action=calculate(pinned,pinned_state[1],tuple(pinned['action_source']),'target')
        assert pinned_action[4]['passed']
        result=calculate(config);assert sum(v[2] for v in material(result[1]))==55
        source=next((c.x,c.y) for c in result[1].cells_with_objects if len(c.best_path_target)>1)
        before=material(result[1]);action=calculate(config,result[1],source,'target')
        assert sum(v[2] for v in material(action[1]))==55
        assert material(result[1])==before, 'Worker mutated the displayed environment'
        assert compare(action[1],build(action[0]))['passed'], 'Displayed fallback must match fresh state'
        changed=moved(config,tuple(config['material'][0][:2]),(1,1));assert sum(v[2] for v in changed['material'])==55
        changed['target']['center']=[14,12];calculate(changed)
        editor=Editor(config);editor.future.result();editor.collect();editor.selected=source;editor.draw()
        pygame.image.save(editor.screen,str(OUT/'editor_preview.png'))
        old_config=copy.deepcopy(editor.config)
        empty=next((x,y) for x in range(25) for y in range(25) if not editor.count_at((x,y)))
        editor.event(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=(int((empty[0]+.5)*30),int((empty[1]+.5)*30))))
        assert editor.selected is None and editor.drag is None and editor.future is None
        assert editor.config==old_config
        editor.selected=source;editor.command('display_page');assert editor.page=='display'
        editor.command('scope_lines');assert not editor.scope_lines
        editor.draw();pygame.image.save(editor.screen,str(OUT/'editor_display_panel.png'))
        editor.command('display_page');assert editor.page=='scene'
        displayed=editor.env
        editor.command('visibility_only');editor.draw()
        assert not editor.heat and editor.paths=='off' and not editor.vectors
        assert editor.env is displayed and editor.future is None
        pygame.image.save(editor.screen,str(OUT/'editor_visibility_only.png'))
        editor.command('scope');assert editor.scope_all;editor.draw()
        editor.command('scope');assert not editor.scope
        editor.command('vectors');editor.command('paths');editor.draw()
        assert editor.vectors and editor.paths=='selected'
        pygame.image.save(editor.screen,str(OUT/'editor_vectors.png'))
        editor.command('paths');assert editor.paths=='all';editor.draw()
        editor.command('paths');assert editor.paths=='off'
        editor.command('heat');editor.command('updates');editor.command('vectors');editor.command('paths')
        editor.show_reused=True;editor.show_updates=False;editor.command('updates')
        assert editor.show_updates and not editor.show_reused
        # Exercise the actual mouse handlers, rather than only the model helpers.
        pos=(int((source[0]+.5)*30),int((source[1]+.5)*30))
        editor.event(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=pos))
        editor.event(pygame.event.Event(pygame.MOUSEBUTTONUP,button=1,pos=(45,45)))
        editor.future.result();editor.collect();assert editor.count_at((1,1))>=1
        editor.command('target')
        editor.event(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=(360,360)))
        editor.event(pygame.event.Event(pygame.MOUSEBUTTONUP,button=1,pos=(420,360)))
        editor.future.result();editor.collect();assert editor.config['target']['center']==[14,12]
        editor.command('undo');editor.future.result();editor.collect()
        assert editor.config['target']['center']==[12,12]
        editor.command('param:own_path_heat_weight:1');editor.future.result();editor.collect()
        assert editor.config['planner']['own_path_heat_weight']==.1
        editor.command('spill');editor.future.result();editor.collect()
        assert editor.env.use_spillage_model
        spill_source=next((c.x,c.y) for c in editor.env.cells_with_objects if len(c.best_path_target)>1)
        spill_action=calculate(editor.config,editor.env,spill_source,'target')
        assert sum(v[2] for v in material(spill_action[1]))==55
        editor.command('dynamic');editor.future.result();editor.collect()
        assert editor.env.target_visibility_mode=='dynamic'
        editor.selected=spill_source;editor.scope=True;editor.page='settings';editor.draw()
        pygame.image.save(editor.screen,str(OUT/'editor_parameters.png'))
        editor.page='scene'
        editor.config,editor.env,editor.groups,editor.status,editor.check,editor.log=action
        editor.draw();pygame.image.save(editor.screen,str(OUT/'editor_after_action.png'))
        editor.save();saved_path=editor.last_saved
        assert saved_path.exists() and saved_path.with_suffix('.png').exists()
        editor.command('load');editor.future.result();editor.collect()
        assert material(editor.env)==material(action[1])
        editor.pool.shutdown();pygame.quit()
        (OUT/'self_test.json').write_text(json.dumps({'initial_objects':55,'action_check':action[4],'message':action[3],
            'frozen':FROZEN,'executable':sys.executable,'asset_root':str(SEMINAR),'module_paths':{name:getattr(sys.modules.get(name),'__file__',None) for name in ['numpy','scipy','shapely','pygame','env']},
            'save_reload_passed':True,'seminar_action_passed':pinned_action[4]['passed']},indent=2))
        print('PASS: default scene, material move, target edit, action conservation, render. '+action[3])
    else:Editor(config).run()

if __name__=='__main__':main()
