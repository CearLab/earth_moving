"""Windowed executable entry: preserve errors in a readable log."""
import os,sys,datetime,traceback
os.environ.setdefault('MPLBACKEND','Agg')
from simulator_paths import writable_folder
log_path=writable_folder('Logs')/('simulator_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.log')
with log_path.open('w',encoding='utf8',buffering=1) as log:
    sys.stdout=sys.stderr=log
    try:
        from interactive_scenario import main
        main()
    except BaseException as error:
        if isinstance(error,SystemExit) and not error.code:raise
        traceback.print_exc()
        if '--self-test' not in sys.argv:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None,f'The simulator could not continue.\n\nDetails were saved in:\n{log_path}', '2D Algorithm Simulator',0x10)
        sys.exit(1)
