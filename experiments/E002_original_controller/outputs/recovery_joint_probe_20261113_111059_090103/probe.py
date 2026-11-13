"""Identify a slower joint trajectory after a short VMC retract. Uses the same probe output layout."""
from pathlib import Path
src=Path('/tmp/e002_1113_full.py').read_text()
src=src.replace("retract=task_prepare(.216,[L.vmc['theta_body'],R.vmc['theta_body']],.45,16.)","retract=task_prepare(.205,[L.vmc['theta_body'],R.vmc['theta_body']],.32,16.)")
src=src.replace("offsets[:3]+=rng.normal(0,[.2,.12],(3,2))","offsets[:3]+=rng.normal(0,[.24,.12],(3,2))")
src=src.replace("dt_seg=.35502 if i==0 else rng.uniform(.30,.41)","dt_seg=.35502 if i==0 else rng.uniform(.35,.46)")
src=src.replace("goal_length=.235 if i==0 else rng.uniform(.21,.274)","goal_length=.235 if i==0 else rng.uniform(.209,.248)")
src=src.replace("wheel_limit=args.wheel*h","wheel_limit=args.wheel*blend((passed-.08)/.20)")
exec(compile(src,'lever_probe.py','exec'))
