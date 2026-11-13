"""Regression: source kinematics/inertia and rotation at all four closure pins."""
import argparse
import datetime
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from build_parallel_variants import OUTPUT
from prepare_model import ROOT,COLOR_SOURCE,digest


def main(motion_run):
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out=ROOT/'runs/m01_model'/f'parallel_integrity_{stamp}';out.mkdir(parents=True)
    original=mujoco.MjModel.from_xml_path(str(COLOR_SOURCE))
    new=mujoco.MjModel.from_xml_path(str(OUTPUT/'robot_no_gas_spring.xml'))
    a=mujoco.MjData(original);b=mujoco.MjData(new);rng=np.random.default_rng(0);pose_error=0.
    for _ in range(20):
        a.qpos[:]=original.qpos0;b.qpos[:]=new.qpos0
        a.qpos[:7]=b.qpos[:7]=[0,0,.6,1,0,0,0]
        for i in range(1,original.njnt):
            assert original.joint(i).name==new.joint(i).name
            q=rng.uniform(-.2,.2);a.qpos[original.jnt_qposadr[i]]=q;b.qpos[new.jnt_qposadr[i]]=q
        mujoco.mj_forward(original,a);mujoco.mj_forward(new,b)
        pose_error=max(pose_error,float(abs(a.xpos-b.xpos).max()),float(abs(a.xmat-b.xmat).max()))
    b=mujoco.MjData(new);mujoco.mj_forward(new,b)
    J=b.efc_J.reshape(b.nefc,new.nv)[b.efc_type==mujoco.mjtConstraint.mjCNSTR_EQUALITY]
    rank=int(np.linalg.matrix_rank(J,tol=1e-8))
    data=mujoco.MjData(new);trace=np.load(motion_run/'robot_no_gas_spring_trace.npy')
    angles=[[] for _ in range(4)]
    for row in trace[::10]:
        data.qpos[7:]=row[1:15];mujoco.mj_forward(new,data)
        for k in range(new.neq):
            body1=new.site_bodyid[new.eq_obj1id[k]];body2=new.site_bodyid[new.eq_obj2id[k]]
            rel=data.xmat[body1].reshape(3,3).T@data.xmat[body2].reshape(3,3)
            angles[k].append(np.arctan2(rel[1,0],rel[0,0]))
    rotations={new.equality(k).name:float(np.ptp(np.unwrap(v))) for k,v in enumerate(angles)}
    m=mujoco.MjModel.from_xml_path(str(OUTPUT/'robot_gas_spring_damped.xml'));d=mujoco.MjData(m)
    m.tendon_stiffness[:]=0;m.dof_damping[:]=0
    d.qvel[m.joint('lf1_Joint').dofadr[0]]=.7;d.qvel[m.joint('rf1_Joint').dofadr[0]]=-.7
    damping=[]
    for c in [0.,10.,50.]:
        m.tendon_damping[:]=c;mujoco.mj_forward(m,d)
        damping.append(dict(c_n_s_m=c,power_w=float(d.qfrc_passive@d.qvel),expected_w=float(-c*(d.ten_velocity@d.ten_velocity))))
    report=dict(original_hinge_count=original.njnt-1,new_hinge_count=new.njnt-1,connect_count=new.neq,
        max_body_pose_error_20_source_coordinate_samples=pose_error,
        max_mass_error=float(abs(new.body_mass-original.body_mass).max()),
        max_inertia_error=float(abs(new.body_inertia-original.body_inertia).max()),
        equality_jacobian_rank=rank,internal_independent_dofs=14-rank,
        closure_pin_relative_rotation_range_rad=rotations,damping_test=damping,
        motion_run=str(motion_run.relative_to(ROOT)),xml_sha256={f:digest(OUTPUT/f) for f in ['robot_no_gas_spring.xml','robot_gas_spring_damped.xml']},
        source_sha256=digest(COLOR_SOURCE),script_sha256=digest(Path(__file__).resolve()),
        command=[sys.executable,'-B',str(Path(__file__).resolve()),'--motion-run',str(motion_run)])
    report['passed']=bool(pose_error<1e-10 and report['max_mass_error']==0 and report['max_inertia_error']==0 and rank==8
        and min(rotations.values())>.01 and damping[-1]['power_w']<damping[1]['power_w']<0
        and all(abs(x['power_w']-x['expected_w'])<1e-10 for x in damping))
    (out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(out);print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--motion-run',type=Path,required=True);args=p.parse_args();main(args.motion_run.resolve())
