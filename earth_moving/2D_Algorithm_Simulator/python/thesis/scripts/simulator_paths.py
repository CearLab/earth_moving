"""Resource and writable locations for source and standalone editions."""
from pathlib import Path
import sys,os
FROZEN=bool(getattr(sys,'frozen',False))
ROOT=Path(sys.executable).resolve().parent if FROZEN else Path(__file__).resolve().parents[2]
SEMINAR=ROOT/'data' if FROZEN else ROOT/'thesis/seminar'

def writable_folder(name):
    preferred=ROOT/name if FROZEN else SEMINAR/('interactive_saved' if name=='Saved Scenarios' else name)
    try:
        preferred.mkdir(parents=True,exist_ok=True)
        probe=preferred/'.write_check';probe.write_text('');probe.unlink()
        return preferred
    except OSError:
        fallback=Path(os.environ.get('LOCALAPPDATA',str(Path.home())))/'2D Algorithm Simulator'/name
        fallback.mkdir(parents=True,exist_ok=True)
        return fallback
