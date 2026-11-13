"""Search bounded hip paths; full VMC/LQR evaluation begins only near the capture region."""
from pathlib import Path
src=Path('/tmp/e002_1113_search_exact.py').read_text()
src=src.replace("                imu=read(est)\n                t=data.time","""                if est is not None or k%2==0:
                    imu=read(est)
                t=data.time""")
src=src.replace("gyro<3.5","gyro<4.0")
src=src.replace("offsets[:3]+=rng.normal(0,[.40,.21],(3,2))","offsets[:3]+=rng.normal(0,[.48,.23],(3,2))")
src=src.replace("base_knots=np.array([[-2.343134,.220593],[.374694,.035202],[.269909,.262576],[0,0]])",
"""base_knots=np.array([[-2.25,.17],[.38,.06],[.20,.30],[0,0]])""")
src=src.replace("at_least=.60","at_least=.48").replace("max_pitch=.25","max_pitch=.38")
exec(compile(src,'fast_recovery.py','exec'))
