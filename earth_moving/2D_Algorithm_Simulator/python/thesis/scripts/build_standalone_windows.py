"""Reproducible one-folder Windows build (run in the earth_moving environment)."""
from pathlib import Path
import sys,os,shutil,json,hashlib,importlib.metadata
ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build/standalone_windows_20261115'
sys.path.insert(0,str(BUILD/'packaging_tools'))
os.environ['PYINSTALLER_CONFIG_DIR']=str(BUILD/'pyinstaller_cache')
os.environ['MPLBACKEND']='Agg'
from PyInstaller.__main__ import run
args=[str(ROOT/'thesis/scripts/standalone_entry.py'),'--noconfirm','--onedir','--windowed',
      '--name','2D Algorithm Simulator','--distpath',str(BUILD/'dist'),'--workpath',str(BUILD/'work'),
      '--specpath',str(BUILD),'--paths',str(ROOT/'thesis/scripts'),'--paths',str(ROOT/'Hybrid Orchestrator')]
for module in ['torch','tensorflow','pandas','IPython','pytest','tkinter','PyQt5','PyQt6','PySide2','PySide6','notebook','jupyter','sympy']:
    args+=['--exclude-module',module]
run(args)
APP=BUILD/'dist/2D Algorithm Simulator';DATA=APP/'data';DATA.mkdir(exist_ok=True)
source=ROOT/'thesis/seminar'
for filename in ['SIMULATOR_QUICK_GUIDE.html','SIMULATOR_ALGORITHM_GUIDE.html']:
    text=(source/filename).read_text(encoding='utf8')
    text=text.replace('Open interactive editor.cmd','2D Algorithm Simulator.exe')
    text=text.replace('Start seminar demo.cmd','2D Algorithm Simulator.exe')
    text=text.replace('interactive_saved','Saved Scenarios (beside the executable)')
    (DATA/filename).write_text(text,encoding='utf8')
for folder in ['scenarios','live_demo']:
    shutil.copytree(source/folder,DATA/folder,dirs_exist_ok=True)
for p in (DATA/'live_demo').glob('*.html'):
    text=p.read_text(encoding='utf8').replace('Start seminar demo.cmd','2D Algorithm Simulator.exe')
    p.write_text(text,encoding='utf8')
(APP/'READ ME FIRST.txt').write_text('''2D ALGORITHM SIMULATOR — WINDOWS x64
Standalone advisor edition — 15 November 2026

1. Extract the ENTIRE ZIP to a folder on your computer.
2. Double-click "2D Algorithm Simulator.exe".
No Python installation, terminal commands or internet connection is required.
Keep the executable, _internal and data folders together. Do not run inside
the ZIP preview and do not send the executable by itself.

The app opens the prepared 55-object seminar scene. Select a pile in Move piles
mode, press T for a target route or H for staging, and Enter to execute.
Display > controls heat, paths, vectors, visibility and update highlights.
Parameters > changes the model settings. Clicking an empty cell clears preview.
F1 opens the user guide. F2 opens the algorithm/mathematics guide.

S saves the edited scene, screenshot and log into "Saved Scenarios" beside
the executable. L loads the most recent saved scene. If the extracted folder
is read-only, saved data goes to LocalAppData/2D Algorithm Simulator instead.
Close/reopen the executable to restore the prepared seminar scene.
Other prepared JSON scenes are in data/scenarios.

Offline live-demo cues and backup images: data/live_demo/DEMO_RUNBOOK.html.
The verified seminar action starts at (0,15), delivers four new objects, and
matches a full rebuild with 8 target searches versus 47. The editor ALWAYS
performs a full verification calculation; it is not a timing benchmark.

This is a Windows x64 build, not a macOS/Linux application. It is unsigned.
Managed computers may require IT approval for downloaded executables.
If it fails to start, preserve the message and the Logs folder for diagnosis.
Do not disable antivirus or organization security controls.

The package contains the simulator runtime, scenarios and documentation only;
it does not contain the thesis workspace, private experiment archives or chats.
Third-party license texts are included in THIRD_PARTY_LICENSES.
''',encoding='utf8')
licenses=APP/'THIRD_PARTY_LICENSES';licenses.mkdir(exist_ok=True)
packages=['numpy','scipy','pygame','shapely','matplotlib','pillow','contourpy','cycler','fonttools','kiwisolver','packaging','pyparsing','python-dateutil','six','pyinstaller']
versions={}
for name in packages:
    try:
        dist=importlib.metadata.distribution(name);versions[name]=dist.version
        for f in dist.files or []:
            if any(term in Path(str(f)).name.lower() for term in ['license','copying','notice']) and dist.locate_file(f).is_file():
                dest=licenses/name/Path(str(f));dest.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(dist.locate_file(f),dest)
    except importlib.metadata.PackageNotFoundError:pass
for candidate in [Path(sys.base_prefix)/'LICENSE.txt',Path(sys.base_prefix)/'LICENSE_PYTHON.txt',Path(sys.base_prefix)/'LICENSE']:
    if candidate.is_file():shutil.copy2(candidate,licenses/('Python_'+candidate.name))
# Conda supplies native libraries (BLAS, SDL, ICU, etc.) outside wheel metadata.
cache=Path(sys.base_prefix).parents[1]/'pkgs'
native_index={}
for record_path in (Path(sys.base_prefix)/'conda-meta').glob('*.json'):
    record=json.loads(record_path.read_text())
    package=cache/(record['name']+'-'+record['version']+'-'+record['build'])
    source_licenses=package/'info/licenses'
    if source_licenses.is_dir():
        for index,src in enumerate(sorted(p for p in source_licenses.rglob('*') if p.is_file())):
            name=record['name'][:24]+'_'+str(index)+'.txt'
            shutil.copy2(src,licenses/name)
            native_index[name]={'package':record['name'],'version':record['version'],'original':str(src.relative_to(source_licenses))}
(licenses/'NATIVE_LICENSE_INDEX.json').write_text(json.dumps(native_index,indent=2))
pygame_license=Path(sys.base_prefix)/'Lib/site-packages/pygame/docs/generated/LGPL.txt'
if pygame_license.is_file():shutil.copy2(pygame_license,licenses/'pygame_LGPL.txt')
manifest={'build_date':'2026-11-15','python':sys.version,'packages':versions,
          'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'thesis/scripts/interactive_scenario.py',ROOT/'thesis/scripts/seminar_scenario.py',ROOT/'Hybrid Orchestrator/env.py',ROOT/'Hybrid Orchestrator/search.py',ROOT/'Hybrid Orchestrator/spillage_model.py']}}
(APP/'BUILD_INFO.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
# Keep license paths short enough for Windows Explorer extraction.
import tempfile
with tempfile.TemporaryDirectory(dir=BUILD) as temporary:
    stage=Path(temporary);index={}
    for i,p in enumerate(sorted(p for p in licenses.rglob('*') if p.is_file())):
        name=f'{i:04d}'+('.json' if p.suffix=='.json' else '.txt')
        shutil.copy2(p,stage/name);index[name]=str(p.relative_to(licenses))
    assert licenses.resolve().is_relative_to(APP.resolve())
    shutil.rmtree(licenses);licenses.mkdir()
    for p in stage.iterdir():shutil.copy2(p,licenses/p.name)
    (licenses/'LICENSE_INDEX.json').write_text(json.dumps(index,indent=2))
print('BUILT',APP)
