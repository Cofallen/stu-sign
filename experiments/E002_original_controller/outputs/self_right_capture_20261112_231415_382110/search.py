import sys,json,time,datetime,shutil
from pathlib import Path
import numpy as np
import mujoco as m
root=Path.cwd()
sys.path.insert(0,str(root))
from LQR_six.get_sensor import RobotSensor
from LQR_six.get_state import StateEstimator
from LQR_six.vmc import VMC
from LQR_six.leg_data import Leg
from LQR_six.lqr_controller import LQRController
from LQR_six.write_sensor import RobotController
model=m.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'))
pose=np.array(json.loads((root/'standing_pose.json').read_text())['no-gas']['qpos'])
ids=[model.actuator(a).id for a in ['act_lf0','act_lf20','act_rf0','act_rf20']]
wids=[model.actuator(a).id for a in ['act_lw','act_rw']]
j=model.actuator_trnid[ids,0]; q=model.jnt_qposadr[j]; v=model.jnt_dofadr[j]
model.actuator_ctrlrange[ids]=[-40,40]
ang=float(sys.argv[1])
rng=np.random.default_rng(491)
d=m.MjData(model); d.qpos[:]=pose
vel=np.zeros(model.nv); vel[4]=ang
m.mj_integratePos(model,d.qpos,vel,1.)
for i in range(700): m.mj_step(model,d)
start=d.qpos.copy(); speed=d.qvel.copy()
sensor=RobotSensor(model,d)
vmc=VMC(model,d)
motor_names=dict(left_front='joint_l1_L',left_back='joint_l6_L',right_front='joint_l1_R',
                 right_back='joint_l6_R',left_wheel='joint_wheel_L',right_wheel='joint_wheel_R')
def read(est,L,R):
 state=sensor.get_state()
 imu={k:state[k] for k in ('acc','gyro','euler')}
 motors={k:state['joints'][v] for k,v in motor_names.items()}
 vmc.update_left(L,motors);vmc.update_right(R,motors)
 est.update_left(L,imu,.001);est.update_right(R,imu,.001)
 est.update(L,R,motors,imu)
 return imu
L,R=Leg(.001),Leg(.001)
lqr=LQRController(model=model)
act=RobotController(model,d)
d.qpos[:]=pose
d.qvel[:]=0
m.mj_forward(model,d)
read(StateEstimator(.001),L,R)
lqr.set_standing_reference(L,R,np.array(json.loads((root/'standing_pose.json').read_text())['no-gas']['ctrl']))
def balance():
 m.mj_forward(model,d)
 imu=read(est,L,R)
 lqr.time=d.time
 for leg in (L,R):
  leg.target['s']=leg.state['s']
  leg.target['dot_s']=0.
  leg.target['braking']=True
 a,w=lqr.control_left(L,imu); b,y=lqr.control_right(R,imu)
 d.ctrl[ids]=[-a[0],-a[1],b[0],b[1]]
 d.ctrl[wids]=[w,-y]
 return float(np.max(np.abs(imu['euler'][:2])))
out=root/'outputs'/('self_right_capture_'+datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f'))
out.mkdir()
shutil.copyfile(__file__,out/'search.py')
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],cwd=str(root),mujoco=m.__version__,seed=491,initial_pitch=ang)))
print(out,flush=True)
sign=np.sign(ang)*np.array([1,1,-1,-1])
best=99
with (out/'run.log').open('w',buffering=1) as log:
 for i in range(450):
  m.mj_resetData(model,d);d.qpos[:]=start;d.qvel[:]=speed
  # Common swing, differential front/back opening, and wheel assist.
  params=np.column_stack([rng.uniform(-2.7,2.7,3),rng.uniform(-.65,.65,3),rng.uniform(-5,5,3)])
  phase_steps=int(rng.uniform(.15,.48)*1000)
  score=99
  captured=None
  stable=0
  for k in range(phase_steps*3+1800):
   if captured is not None:
    tilt=balance()
    m.mj_step(model,d)
    stable=stable+1 if tilt<.05 and d.qpos[2]>.2 and np.linalg.norm(d.qvel[:3])<.12 else 0
    if stable>=400 or k-captured>1300 or tilt>.9:
     break
    continue
   a,b,w=params[min(k//phase_steps,2)]
   target=pose[q]+sign*a+np.array([1,-1,-1,1])*b
   d.ctrl[ids]=np.clip(95*(target-d.qpos[q])-4*d.qvel[v],-40,40)
   d.ctrl[wids]=[w,-w]
   m.mj_step(model,d)
   if k%4: continue
   R=d.xmat[1].reshape(3,3)
   pitch=np.arctan2(-R[2,0],R[2,2])
   value=abs(pitch)+.10*abs(d.qvel[4]) + 1.5*max(0,.20-d.qpos[2]) + .5*abs(R[2,1])
   if abs(pitch)<.43 and abs(R[2,1])<.20 and abs(d.qvel[4])<3.5 and d.qpos[2]>.105:
    captured=k
    est=StateEstimator(.001)
    lqr.next_time={'left':0.,'right':0.}
   if value<score:
    score=value; info=[k,pitch,d.qpos[2],d.qvel[4]]
  log.write(json.dumps(dict(i=i,score=score,info=info,phase_steps=phase_steps,params=params.tolist()))+'\n')
  if score<best:
   best=score
   print(i,score,info,phase_steps,params.tolist(),flush=True)
  if stable>=400:
   match=dict(captured=captured,phase_steps=phase_steps,params=params.tolist(),stable=stable)
   (out/'match.json').write_text(json.dumps(match))
   print('MATCH',match,flush=True)
   break
(out/'exit_status.json').write_text('{"exit_code":0}\n')
