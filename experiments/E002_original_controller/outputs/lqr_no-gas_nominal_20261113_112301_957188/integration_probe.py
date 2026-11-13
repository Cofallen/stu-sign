"""Replay a candidate recovery calibration through the normal E002 controller."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from LQR_six.parameters import RECOVERY
import controller

p=argparse.ArgumentParser()
p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--i',type=int,required=True)
p.add_argument('--wheel',type=float,default=3.)
p.add_argument('--gyro',type=float,default=2.7)
p.add_argument('--tilt',type=float,default=.28)
p.add_argument('--ramp',type=float)
p.add_argument('--prepare',action='store_true')
p.add_argument('--pitch',type=float,default=1.45)
args=p.parse_args()
params=next(x for l in args.candidate.read_text().splitlines() if (x:=json.loads(l))['i']==args.i)
RECOVERY.update(hip_waypoints=params['offsets'],hip_segment_time=params['dt_seg'],
                lqr_blend_time=params['ramp'] if args.ramp is None else args.ramp,
                rising_length=params['goal_length'],
                handoff_gyro=args.gyro,handoff_tilt=args.tilt,handoff_wheel_nm=args.wheel,
                handoff_earliest=.60)
original=controller.create_output
def create_out(*a):
    out=original(*a)
    (out/'integration_probe.py').write_bytes(Path(__file__).read_bytes())
    (out/'probe_input.json').write_text(json.dumps(dict(parameters=params, args=vars(args),
                                          candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest()),
                                                   default=str,indent=2))
    return out
controller.create_output=create_out
sys.argv=['controller.py','--headless',f'--recovery-trial={args.pitch}']+(['--recovery-prepare'] if args.prepare else [])
raise SystemExit(controller.main())
