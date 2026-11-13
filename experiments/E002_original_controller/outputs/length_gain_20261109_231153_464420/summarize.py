from pathlib import Path
import sys,json,hashlib,ast,shutil,numpy as np,mujoco,scipy
r=Path(__file__).parent;root=r.parents[1];sys.path.insert(0,str(root));from LQR_six.lqr import GainSchedule
manifest=json.loads((root/'SOURCE_MANIFEST.json').read_text());h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();checks={}
checks['assets_unchanged']=all(h(root/n)==s for n,s in manifest['asset_sha256'].items())
for f in [root/'controller.py',*(root/'LQR_six').glob('*.py')]:ast.parse(f.read_text())
checks['syntax']=True;g=GainSchedule()
checks['cached_K_exact_equal']=all(np.array_equal(g.get_K(L)[0],g.evaluate(L)[2]) for L in np.linspace(.1,.4,101))
m=mujoco.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'));d=mujoco.MjData(m);pairs=[(m.site(f'{s}_front_site{i}').id,m.site(f'{s}_rear_site{i}').id) for s in ('Left','Right') for i in (1,2)];cases=json.loads((r/'scheduled_v2/results.json').read_text());case_audit={}
for name,result in cases.items():
 a=np.load(r/'scheduled_v2'/(name+'.npz'));closure=0.
 for q in a['qpos']:
  d.qpos[:]=q;mujoco.mj_forward(m,d);closure=max(closure,max(np.linalg.norm(d.site_xpos[i]-d.site_xpos[j]) for i,j in pairs))
 torque_ok=bool((a['ctrl']>=m.actuator_ctrlrange[:,0]-1e-10).all() and (a['ctrl']<=m.actuator_ctrlrange[:,1]+1e-10).all());ok=result['failed'] is None and result['final_tilt']<.05 and closure<1e-4 and torque_ok;checks[name]=ok;case_audit[name]=dict(passed=ok,duration_s=result['duration'],final_2s_tilt_rad=result['final_tilt'],actual_leg_length_m=result['actual_length'],max_closure_m=float(closure),torque_in_limits=torque_ok)
ab=json.loads((r/'jump_ablation/results.json').read_text());no=ab['no_tuck']['jump']['jumps'][0];yes=ab['tuck']['jump']['jumps'][0];checks['tuck_clearance_improved']=yes['max_wheel_clearance_m']>no['max_wheel_clearance_m'];checks['jump_both_landed']=all(x['failed'] is None and x['jump']['jump_phase']=='idle' for x in ab.values())
short=json.loads((r/'short_checks/results.json').read_text());checks['short_regressions']=all(x['exit_code']==0 for x in short.values())
gui_name='realtime_gui_drive_20261109_233156_507447';gui=json.loads((root/'outputs'/gui_name/'results.json').read_text());checks['gui_near_realtime']=.95<=gui['result']['timing']['real_time_factor']<=1.05 and gui['result']['passed'];checks['gui_tuck_and_landing']=bool(gui['result']['teleop']['jumps']) and gui['result']['teleop']['jump_phase']=='idle'
fit=json.loads((r/'holdout_audit.json').read_text());checks['holdout_eigenvalues_stable']=all(x['max_closed_loop_real']<0 for x in fit['holdout']);fit_max=max(x['relative_K_error'] for x in fit['holdout']);data=g.data;raw=np.array(data['samples']['K']);pred=np.array([g.get_K(L)[0] for L in data['lengths_m']]);fit_train=float(np.linalg.norm(pred-raw)/np.linalg.norm(raw))
result=dict(command=[sys.executable,*sys.argv],versions=dict(python=sys.version,mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__),seed=None,checks=checks,functional_passed=all(checks.values()),fit_accuracy=dict(training_relative_norm=fit_train,max_holdout_relative_norm=fit_max,initial_2_percent_accuracy_check_passed=fit['passed'],note='Accuracy threshold miss retained; all heldout eigenvalue/functional tests reported separately, no extrapolation'),masses=data['masses'],cases=case_audit,jump=dict(no_tuck=no,tuck=yes,clearance_gain_m=yes['max_wheel_clearance_m']-no['max_wheel_clearance_m']),gui=dict(driver=gui_name,run=gui['run'],rtf=gui['result']['timing']['real_time_factor'],wall_s=gui['result']['wall_s'],note='Additional key events beyond driver script observed; not an exclusive deterministic input-latency test'),short_checks=short,source_sha256={str(f.relative_to(root)):h(f) for f in [root/'controller.py',root/'gain_schedule.json',*(root/'LQR_six').glob('*.py')]})
(r/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(r/'metadata.json').write_text(json.dumps(dict(interpreter=sys.executable,seed=None,config='fit_final.json',scope='No-gas fitted K + motor-only jump; no training; stage logs retain failures'),indent=2));(r/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if result['functional_passed'] else 1,fit_accuracy_initial_check_passed=fit['passed']))+'\n')
snapshot=r/'final_code';snapshot.mkdir(exist_ok=True)
for f in [root/'controller.py',root/'gain_schedule.json',*(root/'LQR_six').glob('*.py')]:
 target=snapshot/f.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy(f,target)
print(json.dumps(dict(functional_passed=result['functional_passed'],checks=checks,fit_accuracy=result['fit_accuracy'],jump_clearance_gain=result['jump']['clearance_gain_m']),indent=2))
