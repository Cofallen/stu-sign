import sys
from pathlib import Path
import controller
from LQR_six.parameters import RECOVERY_BELLY
time,blend,length=map(float,sys.argv[1:])
RECOVERY_BELLY.update(hip_segment_time=time,lqr_blend_time=blend,wheel_blend_time=blend,rising_length=length)
old=controller.create_output
def create(*args):
    p=old(*args)
    (p/'probe.py').write_bytes(Path(__file__).read_bytes())
    (p/'probe_args.txt').write_text(f'{time} {blend} {length}\n')
    return p
controller.create_output=create
sys.argv=['controller.py','--headless','--recovery-trial=2','--duration=12']
raise SystemExit(controller.main())
