import mujoco as m,numpy as n,json
from LQR_six.recovery import Recovery
from LQR_six.write_sensor import RobotController
model=m.MjModel.from_xml_path('assets/robot_no_gas_spring.xml')
d=m.MjData(model); pose=n.array(json.load(open('standing_pose.json'))['no-gas']['qpos'])
act=RobotController(model,d);rec=Recovery(model,act,pose,{model.geom('floor').id})
d.qpos[:]=pose;v=n.zeros(model.nv);v[4]=1.45;m.mj_integratePos(model,d.qpos,v,1)
for i in range(700):m.mj_step(model,d)
state=d.qpos.copy(),d.qvel.copy()
rng=n.random.default_rng(6);best=4.
for i in range(100):
 m.mj_resetData(model,d);d.qpos[:],d.qvel[:]=state
 rec.phase='swing';rec.start_hips=d.qpos[rec.position].copy();rec.since=0.
 rec.swing_offsets=tuple(zip(rng.uniform(-2.3,.5,3),rng.uniform(-.6,.6,3)))+((0.,0.),)
 rec.segment_seconds=float(rng.uniform(.16,.36))
 for k in range(1350):
  rec.drive(d);m.mj_step(model,d)
  if k%8:continue
  up=d.xmat[rec.base].reshape(3,3)[2]; p=n.arctan2(-up[0],up[2])
  score=abs(p)+abs(up[1])+max(0,.17-d.qpos[2])*4+.12*abs(d.qvel[4])
  if score<best:
   best=score;print(score,i,k,p,d.qpos[2],rec.segment_seconds,rec.swing_offsets,flush=True)
