"""Check actual independent leg angles and the controller's rear-prone gate."""
import datetime
import hashlib
import json
import os
import sys
from pathlib import Path
from zoneinfo import ZoneInfo

import mujoco
import numpy as np

from LQR_six.get_sensor import RobotSensor
from LQR_six.leg_data import Leg
from LQR_six.recovery import Recovery
from LQR_six.vmc import VMC
from LQR_six.write_sensor import RobotController

out = Path('outputs') / datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime(
    'recovery_gate_check_%Y%m%d_%H%M%S_%f')
out.mkdir()
(out/'verify.py').write_bytes(Path(__file__).read_bytes())
paths = ('assets/robot_no_gas_spring.xml', 'standing_pose.json', 'LQR_six/recovery.py',
         'LQR_six/vmc.py', 'LQR_six/leg_rotate.py', 'LQR_six/parameters.py')
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable, '-B', *sys.argv],
    cwd=os.getcwd(), seed=None, mujoco=mujoco.__version__, numpy=np.__version__,
    hashes={p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}), indent=2))
model = mujoco.MjModel.from_xml_path(paths[0])
data = mujoco.MjData(model)
qpos = np.array(json.loads(Path(paths[1]).read_text())['no-gas']['qpos'])
data.qpos[:] = qpos
mujoco.mj_forward(model, data)
legs = (Leg(.001), Leg(.001))
vmc = VMC(model, data)
vmc.update_left(legs[0], {})
vmc.update_right(legs[1], {})
angles = [leg.vmc['theta_body'] for leg in legs]
assert abs(angles[0] - angles[1]) < .001

# Hold one genuine rear-prone snapshot fixed to test the gate independently of dynamics.
data.qpos[:] = qpos
delta = np.zeros(model.nv)
delta[4] = 1.45
mujoco.mj_integratePos(model, data.qpos, delta, 1.)
for _ in range(1230):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
vmc.update_left(legs[0], {})
vmc.update_right(legs[1], {})
sensor = RobotSensor(model, data)
imu = sensor.get_state()
# Use the settled geometry; supplying zero gyro models a still robot for this logic check.
imu['gyro'][:] = 0.
original = [leg.vmc.copy() for leg in legs]
act = RobotController(model, data)
checks = {}
for name, value in [('rear_prone', None), ('one_front', ('theta_body', .9)),
                    ('one_short', ('L0', .24))]:
    for leg, snapshot in zip(legs, original):
        leg.vmc.update(snapshot)
    if value:
        legs[1].vmc[value[0]] = value[1]
    rec = Recovery(model, act, legs, qpos, {model.geom('floor').id})
    rec.begin_fall(0.)
    for tick in range(100):
        data.time = 1. + tick * .001
        rec.update(data, imu)
    rec.drive(data)
    expected = 'retract' if name == 'rear_prone' else 'settle'
    assert rec.phase == expected, (name, rec.phase, rec.events)
    if value:
        assert not np.any(data.ctrl)
    checks[name] = dict(phase=rec.phase, ctrl=data.ctrl.tolist())
result = dict(standing_theta_body_rad=angles, checks=checks, passed=True)
(out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
(out/'run.log').write_text(json.dumps(result)+'\n')
(out/'exit_status.json').write_text('{"exit_code":0}\n')
print(out)
