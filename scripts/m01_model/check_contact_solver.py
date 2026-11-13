"""Numerical contact sensitivity; same source, inertia, friction and initial pose."""
import datetime
import json
import time

import mujoco
import numpy as np

from audit_model import baseline
from prepare_model import ROOT, OUTPUT


def run(dt, time_constant):
    model = mujoco.MjModel.from_xml_path(str(OUTPUT / 'robot.xml'))
    model.opt.timestep = dt
    model.geom_solref[:, 0] = time_constant
    model.geom_solref[:, 1] = 1
    data = baseline(model)
    data.qpos[2] = 0
    mujoco.mj_forward(model, data)
    wheel_contacts = [c for c in data.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
    data.qpos[2] = -min(c.dist for c in wheel_contacts) + .001
    mujoco.mj_forward(model, data)
    minimum = 0.; tail_minimum = 0.
    start = time.perf_counter()
    for _ in range(round(5 / dt)):
        mujoco.mj_step(model, data)
        depth = min((c.dist for c in data.contact), default=0.)
        minimum = min(minimum, depth)
        if data.time > 4:
            tail_minimum = min(tail_minimum, depth)
    return dict(physics_dt=dt, solref_time_constant_s=time_constant, damping_ratio=1,
                min_contact_distance_m=float(minimum), last_second_min_distance_m=float(tail_minimum),
                final_base_xyz=data.qpos[:3].tolist(), final_speed_norm=float(np.linalg.norm(data.qvel)),
                finite=bool(np.isfinite(data.qpos).all()), warning_counts=[int(w.number) for w in data.warning],
                wall_seconds=time.perf_counter()-start, transient_3mm_gate=bool(minimum > -.003))


if __name__ == '__main__':
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs/m01_model' / f'contact_solver_{stamp}'
    output.mkdir(parents=True)
    results = [run(dt, ref) for dt, ref in [(.002, .02), (.002, .005), (.001, .005)]]
    (output / 'results.json').write_text(json.dumps(results, indent=2))
    print(output)
    print(json.dumps(results, indent=2))
