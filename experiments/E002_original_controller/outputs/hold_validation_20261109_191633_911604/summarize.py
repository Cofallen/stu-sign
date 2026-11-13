from pathlib import Path
import json,numpy as np,datetime,hashlib
from zoneinfo import ZoneInfo
root=Path.cwd();p=root/'experiments/E002_original_controller';runs={
 'forward':'lqr_no-gas_nominal_20261109_190823_102893',
 'backward':'lqr_no-gas_nominal_20261109_190901_722408',
 'legs':'lqr_no-gas_nominal_20261109_190911_797771',
 'jump':'lqr_no-gas_nominal_20261109_190922_046888',
 'real_keys':'lqr_no-gas_nominal_20261109_191115_398914'}
summary={}
for case,name in runs.items():
 r=json.loads((p/'outputs'/name/'results.json').read_text());z=np.load(p/'outputs'/name/'trace.npz');a=z['samples'];cols=z['columns'].tolist();t=a[:,0];x=a[:,cols.index('base_x')];v=np.gradient(x,t)
 entry=dict(run=name,physical_passed=r['passed'],warnings=r['warnings'],max_tilt_rad=r['max_tilt_rad'],max_closure_m=r['max_closure_m'],max_external_force_n=r['max_external_force_n'],key_events=r['teleop']['key_events'])
 if case in ('forward','backward'):
  plateau=(t>=3.5)&(t<=4.8);stop=(t>=8)&(t<=11.8)
  entry['actual_speed_plateau_m_s']=float(np.median(v[plateau]));entry['late_stop_peak_speed_m_s']=float(abs(v[stop]).max())
  for i in np.where((t>=5)&(t<10))[0]:
   m=(t>=t[i])&(t<=t[i]+.5)
   if abs(v[m]).max()<.05:entry['release_to_below_005m_s']=float(t[i]-5);break
  assert abs(abs(entry['actual_speed_plateau_m_s'])-1.2)<.1 and entry['late_stop_peak_speed_m_s']<.05
  assert np.sign(entry['actual_speed_plateau_m_s'])==(-1 if case=='forward' else 1)
 elif case=='legs':
  L=a[:,cols.index('left_length')]
  entry['actual_short_mean_m']=float(L[(t>=3.3)&(t<=3.9)].mean());entry['actual_long_mean_m']=float(L[(t>=9)&(t<=11.8)].mean())
  assert abs(entry['actual_short_mean_m']-.18)<.002 and abs(entry['actual_long_mean_m']-.34)<.002
 if r['teleop']['jumps']:
  j=r['teleop']['jumps'][0];entry['jump_rise_m']=j['max_height']-j['start_height'];entry['airborne_s']=j['airborne_s'];assert entry['jump_rise_m']>.20 and entry['airborne_s']>.35
 assert r['passed'] and not any(r['warnings']) and r['passive_actuator_torque_nm']==0
 summary[case]=entry
out=p/'outputs'/('hold_validation_'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
result=dict(passed=True,cases=summary,criteria='Direction signs, actual 1.2±0.1m/s plateau, settled actual speed<0.05, actual length within2mm at 0.18/0.34 targets, jump rise>0.20m and airborne>0.35s, no warnings/passive actuation',scope='Four 12s deterministic held-key physical trials plus actual XTest down/up GUI trial',limitations=['Actual braking is not instantaneous; ~1.5 simulation seconds to <0.05m/s in this trial','X11 backend only','No hardware or arbitrary high-speed/leg-height combinations'])
(out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(out/'metadata.json').write_text(json.dumps(dict(command='mjlab-python summarize_held_controls.py',cwd=str(root),input_results_sha256={n:hashlib.sha256((p/'outputs'/n/'results.json').read_bytes()).hexdigest() for n in runs.values()},versions={'numpy':np.__version__}),indent=2)+'\n');(out/'summarize.py').write_text(Path(__file__).read_text());(out/'exit_status.json').write_text('{"exit_code":0}\n');print(out);print(json.dumps(result,indent=2,ensure_ascii=False))
