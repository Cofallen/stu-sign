from pathlib import Path
import sys,json,hashlib,ast,numpy as np,mujoco,scipy,shutil
r=Path(__file__).parent;root=r.parents[1];h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=json.loads((root/'SOURCE_MANIFEST.json').read_text());cases=json.loads((r/'final_validation/results.json').read_text());rows={};checks={}
for name,result in cases.items():
 a=np.load(r/'final_validation'/(name+'.npz'))['samples'];normal=name!='fall_0.278';duration=20 if name.startswith(('turn','spin')) else 12;ok=result['duration']>=duration-.001 and (result['failed'] is None and result['final_tilt']<.05 if normal else result['first_fall'] is not None)
 rows[name]=dict(passed=bool(ok),duration_s=result['duration'],first_fall_s=result['first_fall'],max_tilt_rad=result['max_tilt'],final_2s_tilt_rad=result['final_tilt'],max_actual_yaw_rate=result['max_actual_yaw_rate'],final_actual_yaw_rate=result['final_actual_yaw_rate'],yaw_rotation_rad=float(np.unwrap(a[:,6])[-1]-a[0,6]),base_x_travel_m=float(a[-1,1]-a[0,1]));checks[name]=bool(ok)
 if name.startswith(('turn','spin')):checks[name+'_stopped']=result['final_actual_yaw_rate']<.02
jumps={}
for name in ('jump','move_jump'):
 data=cases[name+'_0.278'];jump=data['jump']['jumps'][0];phases=jump['phases'];start=next(x['time'] for x in phases if x['phase']=='flight_tuck');end=next(x['time'] for x in phases if x['phase']=='flight_extend');jumps[name]=dict(tuck_hold_s=end-start,record=jump);checks[name+'_longer_tuck']=end-start>=.23
checks['assets_unchanged']=all(h(root/f)==v for f,v in old['asset_sha256'].items());checks['syntax']=True
for file in [root/'controller.py',*(root/'LQR_six').glob('*.py')]:ast.parse(file.read_text())
for name,file in [('gain_readability_equivalent','readability_audit.json'),('vmc_cache_equivalent','vmc_cache_equivalence.json'),('yaw_invariant','yaw_invariance.json')]:checks[name]=json.loads((r/file).read_text())['passed']
checks['id_cache_trace_exact']=json.loads((r/'id_cache_regression.json').read_text())['trace_exact_equal']
cont=json.loads((root/'outputs/lqr_no-gas_nominal_20261110_121513_606754/results.json').read_text());checks['main_fall_continues']=bool(cont['fall_events_s'] and cont['duration_s']>9.99 and not cont['terminated'] and not any(cont['warnings']) and cont['passed'] is False)
gui_driver='realtime_gui_drive_20261110_121541_861149';gui=json.loads((root/'outputs'/gui_driver/'results.json').read_text());events=gui['result']['teleop']['key_events'];checks['real_shift_detected']=any('shift' in v.get('pressed','') for v in events);checks['real_moving_J']=any(v.get('key')=='j' for v in events) and bool(gui['result']['teleop']['jumps'])
interactive=json.loads((root/'outputs/lqr_no-gas_nominal_20261110_121956_532504/results.json').read_text());checks['latest_gui_realtime']=.95<=interactive['timing']['real_time_factor']<=1.05;checks['actual_force_fall_continues']=bool(interactive['fall_events_s'] and not interactive['terminated'] and interactive['duration_s']>23.99 and not any(interactive['warnings']))
copy=json.loads((r/'copy_check.json').read_text());checks['independent_copy']=copy['passed']
result=dict(command=[sys.executable,*sys.argv],seed=None,versions=dict(mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__),checks=checks,passed=all(checks.values()),cases=rows,jumps=jumps,main_fall_run='lqr_no-gas_nominal_20261110_121513_606754',real_key_driver=gui_driver,first_gui_rtf=gui['result']['timing']['real_time_factor'],latest_gui=dict(run='lqr_no-gas_nominal_20261110_121956_532504',rtf=interactive['timing']['real_time_factor'],wall_s=interactive['wall_s'],external_force_n=interactive['max_external_force_n'],note='Driver X11 tree enumeration failed BadWindow; viewer continued with additional manual input/force. Not counted as a controlled key sequence.'),fit_unchanged=True,fit_holdout_limit='Previous .33m 2.35% relative K fit error retained; no refit this task',source_sha256={str(f.relative_to(root)):h(f) for f in [root/'controller.py',root/'gain_schedule.json',*(root/'LQR_six').glob('*.py')]})
(r/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(r/'metadata.json').write_text(json.dumps(dict(interpreter=sys.executable,seed=None,physical_dt=.001,feedback_dt=.01,scope='CPU short tests only; no formal asset/environment changes or training'),indent=2));(r/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if result['passed'] else 1))+'\n')
for f in [root/'controller.py',root/'gain_schedule.json',*(root/'LQR_six').glob('*.py')]:
 target=r/'final_code'/f.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy(f,target)
print(json.dumps(checks,indent=2));print('passed',result['passed'])
