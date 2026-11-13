"""Explore rear-leg joint-PD and staged LQR on full model; deterministic offline probes."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import sys
import traceback
from zoneinfo import ZoneInfo

import mujoco as m
import numpy as np

from LQR_six.get_sensor import RobotSensor
from LQR_six.get_state import StateEstimator
from LQR_six.leg_data import Leg
from LQR_six.lqr_controller import LQRController
from LQR_six.render_stand import load_play_scene
from LQR_six.vmc import VMC
from LQR_six.write_sensor import RobotController

parser=argparse.ArgumentParser()
parser.add_argument('--pitch',type=float,default=1.45)
parser.add_argument('--roll',type=float,default=0.)
parser.add_argument('--count',type=int,default=12)
parser.add_argument('--seed',type=int,default=6)
parser.add_argument('--wheel',type=float,default=5.)
parser.add_argument('--mode',choices=['rise','prepare'],default='rise')
parser.add_argument('--start-file',type=Path)
parser.add_argument('--shift',type=float,default=.26)
parser.add_argument('--reverse',action='store_true')
args=parser.parse_args()
out=Path('outputs')/datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('recovery_joint_probe_%Y%m%d_%H%M%S_%f')
out.mkdir()
(out/'probe.py').write_bytes(Path(__file__).read_bytes())
files=['assets/robot_no_gas_spring.xml','LQR_six/vmc.py','standing_pose.json','gain_schedule.json']
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],cwd=str(Path.cwd()),
                                       mujoco=m.__version__,numpy=np.__version__,args=vars(args),
                                       hashes={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files}),default=str,indent=2))
print(out,flush=True)
model=load_play_scene(Path(files[0]))
pose=json.loads(Path('standing_pose.json').read_text())['no-gas']
data=m.MjData(model)
data.qpos[:]=pose['qpos']
m.mj_forward(model,data)
dt=model.opt.timestep
L,R=Leg(dt),Leg(dt)
vmc=VMC(model,data)
sensor=RobotSensor(model,data)
act=RobotController(model,data)
lqr=LQRController(model=model)
base=model.body('base_Link_del').id
wheel_ids={model.geom(f'{s}_wheel_Link_collision_0').id for s in 'lr'}
floor=model.geom('floor').id
body_geom=model.geom('base_Link_del_collision_0').id
hips=['left_front','left_back','right_front','right_back']
ids=[act.actuator_ids[n] for n in hips]
wheel_act=[act.actuator_ids[n] for n in ['left_wheel','right_wheel']]
joints=model.actuator_trnid[ids,0]
q=model.jnt_qposadr[joints]
v=model.jnt_dofadr[joints]
ref=np.array(pose['qpos'])[q]
rng=np.random.default_rng(args.seed)
common=np.array([1,1,-1,-1]); split=np.array([1,-1,-1,1])
base_knots=np.array([[-2.343134,.220593],[.374694,.035202],[.269909,.262576],[0,0]])

def read(est=None):
    m.mj_forward(model,data)
    vmc.update_left(L,{})
    vmc.update_right(R,{})
    imu=sensor.get_state()
    if est is not None:
        est.update_left(L,imu,dt)
        est.update_right(R,imu,dt)
        motors={'left_wheel':imu['joints']['joint_wheel_L'],'right_wheel':imu['joints']['joint_wheel_R']}
        est.update(L,R,motors,imu)
    return imu

def feet():
    return {int(g) for c in data.contact if floor in c.geom for g in c.geom}

def geometry():
    r=data.xmat[base].reshape(3,3)
    return np.arctan2(-r[2,0],r[2,2]),r[2,1],data.qpos[2]

def blend(x):
    x=np.clip(x,0,1)
    return x*x*(3-2*x)

def blend_knots(knots,dt,t):
    i=min(int(t/dt),len(knots)-2)
    f=blend(t/dt-i)
    return knots[i]+f*(knots[i+1]-knots[i])

read(StateEstimator(dt))
lqr.set_standing_reference(L,R,np.array(pose['ctrl']))
if args.start_file:
    start_state=np.load(args.start_file)
    data.qpos[:]=start_state['qpos']
    data.qvel[:]=start_state['qvel']
else:
    vel=np.zeros(model.nv);vel[3:5]=[args.roll,args.pitch]
    m.mj_integratePos(model,data.qpos,vel,1.)
    for _ in range(700):
        m.mj_step(model,data)
read()
start_qpos=data.qpos.copy();start_qvel=data.qvel.copy()
print('ready',geometry(),[L.vmc['L0'],R.vmc['L0']],[L.vmc['theta_body'],R.vmc['theta_body']],flush=True)
best=10.
try:
    with (out/'trials.jsonl').open('w',buffering=1) as log:
        for i in range(args.count):
            m.mj_resetData(model,data)
            data.qpos[:]=start_qpos;data.qvel[:]=start_qvel
            read()
            ref_near=ref+2*np.pi*np.round((data.qpos[q]-ref)/(2*np.pi))
            if args.mode=='rise':
                offsets=base_knots.copy()
                if i:
                    offsets[:3]+=rng.normal(0,[.46,.22],(3,2))
                dt_seg=.35502 if i==0 else rng.uniform(.23,.52)
                at_least=.58
                height=.18 if i==0 else rng.uniform(.17,.204)
                max_pitch=.35
                min_l=.17
                ramp=.22 if i==0 else rng.uniform(.12,.35)
                goal_length=.235 if i==0 else rng.uniform(.21,.274)
                knot_q=ref_near+offsets[:,0,None]*common+offsets[:,1,None]*split
            else:
                dt_seg=rng.uniform(.26,.54)
                offsets=rng.uniform(-2.3,2.3,(3,4))
                knot_q=ref_near+offsets
            knots=np.vstack([data.qpos[q],knot_q])
            est=None; handoff=None;stable=0;entry=None
            trace=[];score=1e9;metrics={}
            for k in range(2350):
                if est is not None:
                    imu=read(est)
                elif k%4==0:
                    m.mj_forward(model,data)
                    imu=sensor.get_state()
                    rotation=data.xmat[base].reshape(3,3)
                    for side,leg in zip('lr',(L,R)):
                        rel=data.xpos[model.body(side+'_wheel_Link').id]-data.xanchor[model.joint(side+'f0_Joint').id]
                        body=rotation.T@rel
                        leg.vmc.update(L0=float(np.linalg.norm(rel)),theta_body=float(np.arctan2(-body[0],-body[2])))
                t=data.time
                pitch,roll,z=geometry()
                gyro=np.linalg.norm(imu['gyro'])
                lengths=np.array([L.vmc['L0'],R.vmc['L0']])
                angles=np.array([L.vmc['theta_body'],R.vmc['theta_body']])
                contacts=feet()
                if handoff is None:
                    target=blend_knots(knots,dt_seg,t)
                    limit=24*min(t/.2,1)
                    data.ctrl[ids]=np.clip(55*(target-data.qpos[q])-5*data.qvel[v],-limit,limit)
                    data.ctrl[wheel_act]=0.
                    if args.mode=='rise':
                        cost=abs(pitch)+abs(roll)*2+5*max(.2-z,0)+.1*gyro
                        if t>at_least and abs(pitch)<max_pitch and abs(roll)<.10 and z>height and gyro<5. and lengths.min()>min_l and wheel_ids<=contacts:
                            handoff=t
                            est=StateEstimator(dt)
                            lqr.next_time=dict(left=0.,right=0.)
                            init_torque=data.ctrl[ids].copy()
                            L_entry=lengths.copy()
                            entry=[t,pitch,roll,z,gyro,*angles.tolist(),*lengths.tolist(),data.qvel[:3].tolist()]
                    else:
                        cost=(abs(pitch-1.68)+2*abs(roll)+max(0,.28-lengths.min())*8
                              +abs(angles+1.22).max()*.6+.14*gyro)
                        if abs(pitch-1.68)<.33 and abs(roll)<.12 and lengths.min()>=.26 and np.all(angles<-.65) and body_geom in contacts and gyro<.5 and t>.5:
                            handoff=t
                            entry=[t,pitch,roll,z,gyro,*angles.tolist(),*lengths.tolist()]
                            np.savez_compressed(out/f'prepared_{i:03d}.npz',qpos=data.qpos,qvel=data.qvel)
                            break
                    if cost<score:
                        score=cost;metrics=dict(t=t,pitch=pitch,roll=roll,z=z,gyro=float(gyro),angles=angles.tolist(),lengths=lengths.tolist())
                else:
                    passed=t-handoff
                    lqr.time=t
                    for j,leg in enumerate([L,R]):
                        ref_L=L_entry[j]+blend(passed/.75)*(goal_length-L_entry[j])
                        ref_L+=(.278-goal_length)*blend((passed-.75)/.70)
                        leg.target.update(s=0.,dot_s=0.,l0=ref_L,jump_force=0.,yaw_rate=0.,braking=False)
                    tl,wl=lqr.control_left(L,imu);tr,wr=lqr.control_right(R,imu)
                    h=blend(passed/ramp)
                    data.ctrl[ids]=np.clip(init_torque*(1-h)+h*np.r_[-tl,tr],-24,24)
                    wheel_limit=args.wheel*blend(passed/.14)
                    data.ctrl[wheel_act]=np.clip(np.array([wl,-wr])*h,-wheel_limit,wheel_limit)
                    tilt=max(abs(imu['euler'][:2]))
                    stand=tilt<.05 and np.linalg.norm(data.qvel[:3])<.08 and z>.20 and wheel_ids<=contacts
                    stable=stable+1 if stand else 0
                    if tilt>.8 or z<.11:
                        break
                if k%10==0:
                    trace.append([t,pitch,roll,z,gyro,*lengths,*angles,*data.ctrl])
                if stable>=350 and t-handoff>1.3:
                    break
                m.mj_step(model,data)
                if any(w.number for w in data.warning):
                    break
            result=dict(i=i,offsets=offsets.tolist(),dt_seg=dt_seg,score=float(score),best=metrics,
                        entry=entry,stable_ms=stable,end_time=data.time,warnings=[int(w.number) for w in data.warning])
            if args.mode=='rise':
                result.update(height=height,goal_length=goal_length,ramp=ramp)
            log.write(json.dumps(result)+'\n')
            if score<best or handoff is not None:
                best=min(best,score)
                print(result,flush=True)
                np.savez_compressed(out/f'trace_{i:03d}.npz',samples=np.array(trace))
            if stable>=350 or (args.mode=='prepare' and entry):
                break
except BaseException:
    (out/'error.log').write_text(traceback.format_exc())
    (out/'exit_status.json').write_text('{"exit_code": 2}\n')
    raise
(out/'exit_status.json').write_text('{"exit_code": 0}\n')
