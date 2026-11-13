import sys,json,itertools
from pathlib import Path
import numpy as np
import mujoco as m
root=Path.cwd()
model=m.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'))
pose=np.array(json.loads((root/'standing_pose.json').read_text())['no-gas']['qpos'])
ids=[model.actuator(a).id for a in ['act_lf0','act_lf20','act_rf0','act_rf20']]
j=model.actuator_trnid[ids,0]; q=model.jnt_qposadr[j]; v=model.jnt_dofadr[j]
model.actuator_ctrlrange[ids]=[-40,40]
def angle(d):
 R=d.xmat[1].reshape(3,3)
 return np.arctan2(-R[2,0],R[2,2])
for ang, sign, off, ramp in itertools.product([-1.45,1.45],[-1,1],[.8,1.35,1.9,2.5],[1.5,4.]):
 d=m.MjData(model); d.qpos[:]=pose
 vel=np.zeros(model.nv); vel[4]=ang
 m.mj_integratePos(model,d.qpos,vel,1.)
 phases=[]; closest=10
 for i in range(3500):
  m.mj_forward(model,d)
  if i==700: fallen=angle(d); start=d.qpos[q].copy()
  if i>=700:
   f=min(1,(i-700)*.001*ramp)
   target=start*(1-f)+f*(pose[q]+np.sign(fallen)*sign*off*np.array([1,1,-1,-1]))
   d.ctrl[ids]=np.clip(95*(target-d.qpos[q])-4*d.qvel[v],-40,40)
   p=angle(d)
   if abs(p)<closest:
    closest=abs(p); near=[round(x,3) for x in [p,d.qpos[2],d.qvel[4],i*.001]]
  if i in (699,1100,1600,2400,3499): phases.append(round(angle(d),2))
  m.mj_step(model,d)
 print(ang,sign,off,ramp,phases,near,flush=True)
