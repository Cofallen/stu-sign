import sys,json,time,datetime,shutil
from pathlib import Path
import numpy as np
import mujoco as m
root=Path.cwd()
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
out=root/'outputs'/('self_right_search_'+datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f'))
out.mkdir()
shutil.copyfile(__file__,out/'search.py')
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],cwd=str(root),mujoco=m.__version__,seed=491,initial_pitch=ang)))
print(out,flush=True)
sign=np.sign(ang)*np.array([1,1,-1,-1])
best=99
with (out/'run.log').open('w',buffering=1) as log:
 for i in range(120):
  m.mj_resetData(model,d);d.qpos[:]=start;d.qvel[:]=speed
  # Common swing, differential front/back opening, and wheel assist.
  params=np.column_stack([rng.uniform(-2.7,2.7,3),rng.uniform(-.65,.65,3),rng.uniform(-5,5,3)])
  phase_steps=int(rng.uniform(.15,.48)*1000)
  score=99
  for k in range(phase_steps*3+450):
   a,b,w=params[min(k//phase_steps,2)]
   target=pose[q]+sign*a+np.array([1,-1,-1,1])*b
   d.ctrl[ids]=np.clip(95*(target-d.qpos[q])-4*d.qvel[v],-40,40)
   d.ctrl[wids]=[w,-w]
   m.mj_step(model,d)
   if k%4: continue
   R=d.xmat[1].reshape(3,3)
   pitch=np.arctan2(-R[2,0],R[2,2])
   value=abs(pitch)+.10*abs(d.qvel[4]) + 1.5*max(0,.20-d.qpos[2]) + .5*abs(R[2,1])
   if value<score:
    score=value; info=[k,pitch,d.qpos[2],d.qvel[4]]
  log.write(json.dumps(dict(i=i,score=score,info=info,phase_steps=phase_steps,params=params.tolist()))+'\n')
  if score<best:
   best=score
   print(i,score,info,phase_steps,params.tolist(),flush=True)
  if best<.20: break
(out/'exit_status.json').write_text('{"exit_code":0}\n')
