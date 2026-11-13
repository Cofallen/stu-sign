"""Inject keyboard inputs or a short perturbation into the E002 loop for regression."""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import controller

parser = argparse.ArgumentParser()
parser.add_argument('case', choices=('high-jump', 'roll-fall', 'pitched-fall'))
parser.add_argument('--prepare', action='store_true')
args = parser.parse_args()
src = Path(controller.__file__).read_text()
src = src.replace("if args.drive or trial is not None or args.reverse_trial else None",
                  "if args.drive or trial is not None or args.reverse_trial or check.case == 'high-jump' else None")
src = src.replace("                if args.reverse_trial:", """
                if check.case == 'high-jump':
                    teleop.set_pressed('e' if 1. <= data.time < 2. else '')
                    if step == round(3. / dt):
                        teleop.on_key('j')
                if args.reverse_trial:""")
src = src.replace("            elif viewer is None:\n", """
            elif check.case in ('roll-fall', 'pitched-fall'):
                data.xfrc_applied[:] = 0.
                if 1. <= data.time < 1.22:
                    axis = 3 if check.case == 'roll-fall' else 4
                    data.xfrc_applied[model.body('base_Link_del').id, axis] = 95.
            elif viewer is None:
""")
original = controller.create_output
def create_output(*parameters):
    out = original(*parameters)
    (out/'drive_verification.py').write_bytes(Path(__file__).read_bytes())
    (out/'drive_verification.json').write_text(json.dumps(dict(args=vars(args), script_path=__file__), indent=2)+'\n')
    (out/'executed_controller.py').write_text(src)
    return out

sys.argv = ['controller.py', '--headless']
if args.prepare:
    sys.argv += ['--recovery-prepare']
namespace = {'__name__': '__probe__', '__file__': controller.__file__, 'check': args}
exec(compile(src, controller.__file__, 'exec'), namespace)
namespace['create_output'] = create_output
raise SystemExit(namespace['main']())
