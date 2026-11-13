from pathlib import Path
import json,sys,hashlib,runpy,shutil,ast,numpy as np,mujoco,scipy
r=Path(__file__).parent;root=r.parents[1];sys.path.insert(0,str(root))
from LQR_six.render_stand import load_play_scene
from LQR_six.lqr import GainSchedule
m=load_play_scene(root/'assets/robot_no_gas_spring.xml',root/'scenes/training_terrain.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d)
angle=float(np.degrees(np.arctan2(-d.geom_xmat[m.geom('ramp17').id].reshape(3,3)[2,0],d.geom_xmat[m.geom('ramp17').id].reshape(3,3)[0,0])))
terrain=dict(ramp_degrees=angle,step_top_m={n:float(m.geom(n).pos[2]+m.geom(n).size[2]) for n in ('step10_1','step10_2','step20_1','step20_2')})
old=json.loads((root/'SOURCE_MANIFEST.json').read_text());h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();checks=dict(original_assets_unchanged=all(h(root/n)==v for n,v in old['asset_sha256'].items()),ramp17=abs(angle-17)<1e-9,step_heights=all(abs(terrain['step_top_m'][n]-v)<1e-9 for n,v in zip(terrain['step_top_m'],[.1,.2,.2,.4])))
results=json.loads((r/'response_v3/results.json').read_text());checks['motion_and_landing']=all(x['failed'] is None and x['final_tilt']<.05 for x in results.values());checks['landing_length_restored']=all(abs(results[k]['actual_length']-.278)<.005 for k in ('jump','move_jump'));checks['translation_stop_under_09']=all(v<.9 for v in results['move']['release_to_body_speed_below_005_s'])
a=np.load(r/'response_v3/spin_right.npz')['samples'];idx=np.where((a[:,0]>=12)&(abs(a[:,15])<.05))[0];yaw_stop=float(a[idx[0],0]-12);checks['spin_stop_under_12']=yaw_stop<1.2
log=(r/'response_v3/terrain.log').read_text();run=Path(log.splitlines()[0]);terrain_result=json.loads((run/'results.json').read_text());checks['terrain_initial_stand']=terrain_result['passed']
g=GainSchedule();eigs=[]
for L in np.linspace(.18,.34,17):
 A,B,K,_=g.evaluate(L);K=K.copy();K[:,3]*=2;eigs.append(float(np.linalg.eigvals(A-B@K).real.max()))
checks['linear_brake_checks']=max(eigs)<0
for f in [root/'controller.py',*(root/'LQR_six').glob('*.py')]:ast.parse(f.read_text())
checks['syntax']=True
cfg=runpy.run_path(str(root/'LQR_six/parameters.py'));config={k:cfg[k] for k in ('DRIVE','JUMP','PID','CONTROL','MONITOR')}
summary=dict(command=[sys.executable,*sys.argv],seed=None,versions=dict(mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__),checks=checks,passed=all(checks.values()),terrain=terrain,terrain_stand_run=str(run),traversal_verified=False,translation_stop_s=results['move']['release_to_body_speed_below_005_s'],spin_stop_s=yaw_stop,results=results,brake_linear_max_real=max(eigs),config=config,source_sha256={str(f.relative_to(root)):h(f) for f in [root/'controller.py',root/'scenes/training_terrain.xml',*(root/'LQR_six').glob('*.py')]},limitations=['No stair/ramp traversal acceptance','Intermediate final_tests timed landing feedback failed; response_v3 adopted','Intermediate full code snapshots not all captured; final snapshot is explicit','No hard realtime or instantaneous stop guarantee'])
(r/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n');(r/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if summary['passed'] else 1))+'\n')
for f in [root/'controller.py',root/'scenes/training_terrain.xml',root/'gain_schedule.json',*(root/'LQR_six').glob('*.py')]:
 target=r/'final_code'/f.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy(f,target)
print(json.dumps(dict(checks=checks,terrain=terrain,stop=summary['translation_stop_s'],yaw_stop=yaw_stop),indent=2));assert summary['passed']
