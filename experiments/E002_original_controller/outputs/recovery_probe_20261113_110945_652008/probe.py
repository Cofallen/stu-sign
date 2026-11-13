"""MuJoCo recovery probe. Params are trial seed/sweep type, output a unique directory."""
import argparse
import datetime
import json
import os
from pathlib import Path
import sys
import time
import traceback
import warnings
from zoneinfo import ZoneInfo

import mujoco as m
import numpy as np
from LQR_six.vmc import VMC
from LQR_six.leg_data import Leg
from LQR_six.render_stand import load_play_scene
from LQR_six.get_sensor import RobotSensor
from LQR_six.get_state import StateEstimator
from LQR_six.lqr_controller import LQRController
from LQR_six.write_sensor import RobotController
from LQR_six.simulation import closure_error

p=argparse.ArgumentParser()
p.add_argument('mode', choices=['prep','retract'])
p.add_argument('--pitch', type=float, default=1.45)
p.add_argument('--roll', type=float, default=0.)
p.add_argument('--count',type=int,default=16)
p.add_argument('--seed',type=int,default=42)
args=p.parse_args()
out=Path('outputs')/datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('recovery_probe_%Y%m%d_%H%M%S_%f')
out.mkdir()
(out/'probe.py').write_bytes(Path(__file__).read_bytes())
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],cwd=os.getcwd(),
                                         mujoco=m.__version__,numpy=np.__version__,args=vars(args)),indent=2))
print(out,flush=True)
model=load_play_scene(Path('assets/robot_no_gas_spring.xml'))
pose=json.loads(Path('standing_pose.json').read_text())['no-gas']
data=m.MjData(model)
data.qpos[:]=pose['qpos']
m.mj_forward(model,data)
dt=model.opt.timestep
legL,legR=Leg(dt),Leg(dt)
vmc=VMC(model,data)
sensor=RobotSensor(model,data)
act=RobotController(model,data)
lqr=LQRController(model=model)
base=model.body('base_Link_del').id
wheel_ids={model.geom(f'{s}_wheel_Link_collision_0').id for s in 'lr'}
floor=model.geom('floor').id
hips=['left_front','left_back','right_front','right_back']
motor_aliases=dict(left_wheel='joint_wheel_L',right_wheel='joint_wheel_R')
rng=np.random.default_rng(args.seed)

def read(est=None):
    m.mj_forward(model,data)
    vmc.update_left(legL,{})
    vmc.update_right(legR,{})
    imu=sensor.get_state()
    if est:
        est.update_left(legL,imu,dt);est.update_right(legR,imu,dt)
        est.update(legL,legR,{k:imu['joints'][v] for k,v in motor_aliases.items()},imu)
    return imu

def geom():
    r=data.xmat[base].reshape(3,3)
    return np.arctan2(-r[2,0],r[2,2]), r[2,1],data.qpos[2]

def legs():
    return [leg.vmc for leg in (legL,legR)]

def wrapped(theta):
    return (theta+np.pi)%(2*np.pi)-np.pi

def hip_command(lengths,angles,kL,kA,limit,rate=.1):
    for i,leg in enumerate(legs()):
        F=np.clip(kL*(lengths[i]-leg['L0'])-45*leg['L0_dot'],-160,160)
        T=np.clip(kA*wrapped(angles[i]-leg['theta_body'])-2.7*leg['theta_body_dot'],-22,22)
        for name,tau in zip(hips[2*i:2*i+2], np.clip(leg['J_body']@[F,T],-limit,limit)):
            act.set_actuator(name,tau)

def steer(est,Lref,blend,torque_limit,wheel_limit,initial):
    imu=read(est)
    lqr.time=data.time
    for leg in (legL,legR):
        leg.target.update(s=0.,dot_s=0.,l0=Lref,jump_force=0.,yaw_rate=0.,braking=False)
    tl,wl=lqr.control_left(legL,imu)
    tr,wr=lqr.control_right(legR,imu)
    for name,torque in zip(hips,np.clip((1-blend)*initial+blend*np.r_[-tl,tr],-torque_limit,torque_limit)):
        act.set_actuator(name,torque)
    act.set_actuator('left_wheel',np.clip(wl*blend,-wheel_limit,wheel_limit))
    act.set_actuator('right_wheel',np.clip(-wr*blend,-wheel_limit,wheel_limit))

read(StateEstimator(dt))
lqr.set_standing_reference(legL,legR,np.array(pose['ctrl']))
start=np.zeros(model.nv)
start[3:5]=[args.roll,args.pitch]
m.mj_integratePos(model,data.qpos,start,1.)
for k in range(1000):
    m.mj_step(model,data)
    if k==699:
        read()
        ready_qpos=data.qpos.copy()
        ready_qvel=data.qvel.copy()
print('ready',geom(),[(l['L0'],l['theta_body']) for l in legs()],flush=True)
best=1e9
try:
    with (out/'trials.jsonl').open('w',buffering=1) as file:
        for i in range(args.count):
            m.mj_resetData(model,data)
            data.qpos[:],data.qvel[:]=ready_qpos,ready_qvel
            read()
            startL=np.array([l['L0'] for l in legs()])
            startA=np.array([l['theta_body'] for l in legs()])
            if args.mode=='prep':
                goalL=float(rng.uniform(.28,.34))
                goalA=float(rng.uniform(-1.55,-.85))
                kL,kA=float(rng.uniform(350,1050)),float(rng.uniform(12,55))
                duration=float(rng.uniform(.4,1.3))
                handoff=.16
            else:
                goalL=float(rng.uniform(.15,.23))
                goalA=float(rng.uniform(-1.2,-.1))
                kL,kA=float(rng.uniform(350,1000)),float(rng.uniform(8,40))
                duration=float(rng.uniform(.35,1.1))
                handoff=float(rng.uniform(.10,.35))
            metrics=dict(min_error=1e9, handoff_at=None, stable=False, min_base=1., max_tilt=0.)
            est=None
            samples=[]
            for k in range(2700):
                imu=read()
                time_s=data.time
                frac=np.clip(time_s/duration,0,1)
                frac=frac*frac*(3-2*frac)
                if est is None:
                    pitch,roll,z=geom()
                    angles=np.array([l['theta_body'] for l in legs()])
                    lens=np.array([l['L0'] for l in legs()])
                    gyro=np.linalg.norm(imu['gyro'])
                    wheel_support={int(g) for c in data.contact if floor in c.geom for g in c.geom}
                    error=(abs(pitch)+abs(roll)*2+max(.12-z,0)*6
                           +max(.2-lens.min(),0)*2+.10*gyro)
                    if args.mode=='prep':
                        error=(abs(angles-goalA).max()+6*abs(lens-goalL).max()+abs(roll)
                               +max(0,1.05-pitch)+.15*gyro)
                    if error<metrics['min_error']:
                        metrics.update(min_error=float(error),at=float(time_s),geometry=[float(pitch),float(roll),float(z)],
                                       angles=angles.tolist(),lengths=lens.tolist(),gyro=float(gyro))
                    if args.mode=='retract' and pitch<.30 and abs(pitch)<.3 and abs(roll)<.16 and z>.12 and wheel_ids<=wheel_support and gyro<2.5:
                        est=StateEstimator(dt)
                        metrics['handoff_at']=float(time_s)
                        initial=np.array([data.ctrl[act.actuator_ids[h]] for h in hips])
                        lqr.next_time=dict(left=0.,right=0.)
                        rise_start=float(np.mean(lens))
                    else:
                        lens_target=startL+frac*(goalL-startL)
                        if args.mode=='prep':
                            progress=np.clip((time_s-.45)/duration,0,1)
                            angle_target=startA+progress*wrapped(goalA-startA)
                        else:
                            angle_target=startA+frac*(goalA-startA)
                        hip_command(lens_target,angle_target,kL,kA,22*min(time_s/.2,1.))
                if est is not None:
                    passed=time_s-metrics['handoff_at']
                    Lref=rise_start+np.clip((passed-.35)/.9,0,1)*(.278-rise_start)
                    steer(est,Lref,min(passed/handoff,1.),24.,min(max(0,passed-.06)*8,5.),initial)
                    if passed>.8 and max(abs(imu['euler'][:2]))<.05 and np.linalg.norm(data.qvel[:3])<.05 and data.qpos[2]>.225:
                        metrics['stable']=True
                        break
                if k%10==0:
                    samples.append([time_s,*geom(),*[l['L0'] for l in legs()],*[l['theta_body'] for l in legs()],*data.ctrl])
                m.mj_step(model,data)
                if any(w.number for w in data.warning):
                    metrics['warning']=True
                    break
            params=dict(L=goalL,A=goalA,kL=kL,kA=kA,duration=duration,handoff=handoff)
            report=dict(i=i,params=params,**metrics)
            file.write(json.dumps(report)+'\n')
            if metrics['min_error']<best or metrics['stable'] or metrics['handoff_at'] is not None:
                best=min(best,metrics['min_error'])
                print(report,flush=True)
                np.savez_compressed(out/f'trace_{i:03d}.npz',trace=np.array(samples))
            if metrics['stable']:
                break
except BaseException:
    (out/'error.log').write_text(traceback.format_exc())
    (out/'exit_status.json').write_text('{"exit_code": 2}\n')
    raise
(out/'exit_status.json').write_text('{"exit_code": 0}\n')
