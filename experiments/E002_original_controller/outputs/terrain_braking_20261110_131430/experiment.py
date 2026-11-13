"""本轮独立短测/工作点采样。来源为E002主循环；不写正式资产。"""
from pathlib import Path
import sys,json,time,numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from LQR_six.get_sensor import RobotSensor
from LQR_six.write_sensor import RobotController
from LQR_six.leg_data import Leg
from LQR_six.vmc import VMC
from LQR_six.get_state import StateEstimator
from LQR_six.lqr_controller import LQRController
from LQR_six.teleop import Teleop

def run(kind,length=.278,seconds=10,scheduled=True,tuck=True):
 m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/robot_no_gas_spring.xml'));d=mujoco.MjData(m);pose=json.loads((ROOT/'standing_pose.json').read_text())['no-gas'];d.qpos[:]=pose['qpos'];d.ctrl[:]=pose['ctrl'];mujoco.mj_forward(m,d)
 sensor=RobotSensor(m,d);writer=RobotController(m,d);vmc=VMC(m,d);ll,rr=Leg(.001),Leg(.001);est=StateEstimator(.001)
 def read():
  st=sensor.get_state();imu={k:st[k] for k in ('acc','gyro','euler')};j=st['joints'];motor=dict(left_front=j['joint_l1_L'],left_back=j['joint_l6_L'],right_front=j['joint_l1_R'],right_back=j['joint_l6_R'],left_wheel=j['joint_wheel_L'],right_wheel=j['joint_wheel_R']);vmc.update_left(ll,motor);vmc.update_right(rr,motor);est.update_left(ll,imu,.001);est.update_right(rr,imu,.001);est.update(ll,rr,motor,imu);return imu
 read();control=LQRController();control.set_standing_reference(ll,rr,np.array(pose['ctrl']));tele=Teleop(ll,rr);rows=[];qrows=[];urows=[];fail=None;first_fall=None;start=time.perf_counter()
 if hasattr(control,'schedule_enabled'):control.schedule_enabled=scheduled
 if hasattr(tele,'jump'):tele.jump.tuck_enabled=tuck
 wheels=[m.body(s+'_wheel_Link').id for s in ('l','r')]
 for step in range(round(seconds/.001)):
  mujoco.mj_forward(m,d);imu=read();control.time=d.time
  if kind=='pose':tele.l0+=float(np.clip(length-tele.l0,-.06*.001,.06*.001))
  if kind=='sweep':tele.set_pressed({'q'} if 1<d.time<2.2 else {'e'} if 3<d.time<5.0 else set())
  if kind=='move':tele.set_pressed({'w'} if 1<d.time<3 else {'s'} if 5<d.time<7 else set())
  if kind in ('turn_left','turn_right'):
   tele.set_pressed({('a' if kind=='turn_left' else 'd')} if 1<d.time<12 else set())
  if kind in ('spin_left','spin_right'):
   direction='a' if kind=='spin_left' else 'd'
   tele.set_pressed({'shift',direction} if 1<d.time<2 else {'shift'} if 2<=d.time<12 else set())
  if kind=='repeat_jump':
   tele.set_pressed({'w'} if 1<d.time<9 else set())
   if step in (2000,6000):tele.on_key('j')
  if kind=='move_jump':
   tele.set_pressed({'w'} if 1<d.time<7 else set())
   if step==3000:tele.on_key('j')
  if kind=='fall' and step==1000:
   offset=np.zeros(m.nv);offset[3]=1.;mujoco.mj_integratePos(m,d.qpos,offset,1.);mujoco.mj_forward(m,d)
  if kind=='jump' and step==2000:tele.on_key('j')
  tele.observe(m,d,.001);tele.update(imu,.001)
  for side,leg in [('left',ll),('right',rr)]:
   tau,tw=getattr(control,'control_'+side)(leg,imu);writer.set_actuator(side+'_front',(-1 if side=='left' else 1)*tau[0]);writer.set_actuator(side+'_back',(-1 if side=='left' else 1)*tau[1]);writer.set_actuator(side+'_wheel',(1 if side=='left' else -1)*tw)
  if step%10==0:
   com=(m.body_mass[:,None]*d.xipos).sum(axis=0)/sum(m.body_mass)
   rows.append([d.time,*d.qpos[:3],*imu['euler'],ll.vmc['L0'],rr.vmc['L0'],tele.l0,ll.target['l0'],com[2],min(d.xpos[w,2] for w in wheels)-.06,d.ncon,ll.state['dot_s'],imu['gyro'][2],*control.K.flatten()]);qrows.append(d.qpos.copy());urows.append(d.ctrl.copy())
  if d.qpos[2]<.12 or max(abs(imu['euler'][:2]))>.6:
   fail='fall';first_fall=float(d.time) if first_fall is None else first_fall
  if any(w.number for w in d.warning):fail='numerical_warning';break
  mujoco.mj_step(m,d)
 a=np.array(rows);result=dict(kind=kind,length=length,duration=float(d.time),wall_s=time.perf_counter()-start,failed=fail,first_fall=first_fall,max_tilt=float(abs(a[:,4:6]).max()),final_tilt=float(abs(a[-min(len(a),200):,4:6]).max()),actual_length=float(a[-1,7]),jump=tele.status(),max_com_z=float(a[:,11].max()),max_base_z=float(a[:,3].max()),max_wheel_clearance=float(a[:,12].max()),max_actual_yaw_rate=float(abs(a[:,15]).max()),final_actual_yaw_rate=float(abs(a[-100:,15]).max()),K_changed=bool(np.max(abs(a[:,16:]-a[0,16:]))>1e-6))
 return result,a,np.array(qrows),np.array(urows),m

if __name__=='__main__':
 out=Path(__file__).parent/sys.argv[1];out.mkdir(exist_ok=False);results={};points=[]
 import shutil
 shutil.copy(__file__,out/'driver.py')
 for source in (ROOT/'LQR_six').glob('*.py'):
  target=out/'code/LQR_six'/source.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy(source,target)
 names=['turn_left','spin_right','move'] if sys.argv[1].startswith('brake') else ['repeat_jump'] if sys.argv[1].startswith('repeat') else ['move_jump','jump'] if sys.argv[1].startswith('jump') else ['turn_left','turn_right','spin_left','spin_right','move_jump','jump','fall']
 cases=[(name,.278) for name in names]
 for kind,length in cases:
  name=f'{kind}_{length:.3f}';r,a,q,u,m=run(kind,length,seconds=20 if kind.startswith(('turn','spin')) else 12);results[name]=r;np.savez_compressed(out/(name+'.npz'),samples=a,qpos=q,ctrl=u);(out/(name+'.json')).write_text(json.dumps(r,indent=2));print(name,r['failed'],r['final_tilt'],r['actual_length'],flush=True)
  if kind=='pose':points.append(dict(length=r['actual_length'],qpos=q[-1].tolist(),ctrl=np.mean(u[-50:],axis=0).tolist()))
 (out/'points.json').write_text(json.dumps(points,indent=2));(out/'results.json').write_text(json.dumps(results,indent=2));(out/'exit_status.json').write_text(json.dumps(dict(exit_code=int(any(r['failed'] for r in results.values())))))
