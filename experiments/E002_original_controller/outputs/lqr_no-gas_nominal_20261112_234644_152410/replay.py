import json,sys
from pathlib import Path
import numpy as np
root=Path.cwd()
sys.path.insert(0,str(root))
source_run=sys.argv[1]
events=json.loads((Path(source_run)/'results.json').read_text())['teleop']['key_events']
keys=[event for event in events if 'pressed' in event]
src=(root/'controller.py').read_text()
src=src.replace('if args.drive or trial is not None or args.reverse_trial else None','if True else None')
src=src.replace('                if args.reverse_trial:',"""                while replay_keys and data.time >= replay_keys[0]['time'] - 1e-9:
                    teleop.set_pressed(replay_keys.pop(0)['pressed'])
                if args.reverse_trial:""")
src=src.replace('            if phase == \'balance\' and',"""            if phase == 'failed' and previous_phase != 'failed':
                np.savez(out/'fallen_state.npz', qpos=data.qpos, qvel=data.qvel)
            if phase == 'balance' and""")
sys.argv=['controller.py','--headless','--step-up','--duration',str(max(10,int(events[-1]['time'])+6))]
exec(compile(src,str(root/'controller.py'),'exec'),{'__name__':'__main__','__file__':str(root/'controller.py'),'replay_keys':keys})
