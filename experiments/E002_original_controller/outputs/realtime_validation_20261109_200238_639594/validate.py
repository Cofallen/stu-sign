from pathlib import Path
import ast,datetime,hashlib,json,shutil,sys
import numpy as np
p=Path('/home/cofallen/Code/毕业设计/experiments/E002_original_controller');out=p/'outputs'/('realtime_validation_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir();shutil.copy(__file__,out/'validate.py')
manifest=json.loads((p/'SOURCE_MANIFEST.json').read_text());h=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
checks={}
checks['assets_unchanged']=all(h(p/name)==sha for name,sha in manifest['asset_sha256'].items())
mods=['teleop.py','vmc.py','lqr_controller.py','get_sensor.py','get_state.py','write_sensor.py'];before=p/'outputs/lqr_no-gas_nominal_20261109_194755_279785/code/LQR_six'
checks['control_files_unchanged']=all(h(p/'LQR_six'/f)==h(before/f) for f in mods)
for file in [p/'controller.py',*list((p/'LQR_six').glob('*.py'))]:ast.parse(file.read_text())
checks['syntax']=True
old=np.load(p/'outputs/lqr_no-gas_nominal_20261109_191634_995848/trace.npz')['samples'];new=np.load(p/'outputs/lqr_no-gas_nominal_20261109_200109_536896/trace.npz')['samples'];checks['headless_trace_exact_equal']=np.array_equal(old,new)
runs={}
for label,name in [('actual_keys','realtime_gui_drive_20261109_195937_939707'),('hud_camera_panel','realtime_gui_drive_20261109_200043_835391'),('native_ui','realtime_gui_drive_20261109_200133_254348')]:
 r=json.loads((p/'outputs'/name/'results.json').read_text());v=r['result'];runs[label]=dict(driver=name,run=r['run'],rtf=v['timing']['real_time_factor'],wall_s=v['wall_s'],duration_s=v['duration_s']);checks[label]=r['process_exit_code']==0 and v['passed'] and .95<=v['timing']['real_time_factor']<=1.05
 if label=='hud_camera_panel':
  a,b=v['viewer_options']['initial'],v['viewer_options']['final'];checks['mouse_camera_rotated']=a['azimuth']!=b['azimuth'];checks['driving_does_not_toggle_flags']=a['vis_flags']==b['vis_flags'] and a['render_flags']==b['render_flags'];checks['jump_landed']=bool(v['teleop']['jumps'] and v['teleop']['jump_phase']=='idle');events=v['teleop']['key_events'];checks['panel_entry_stops']=events[1]['pressed']=='' and events[1]['time']-events[0]['time']<2.2
 if label=='native_ui':
  a,b=v['viewer_options']['initial'],v['viewer_options']['final'];checks['native_display_changes_persist']=a['vis_flags']!=b['vis_flags'] and a['render_flags']!=b['render_flags']
result=dict(command=[sys.executable,__file__],seed=None,checks=checks,runs=runs,source_sha256={str(f.relative_to(p)):h(f) for f in [p/'controller.py',*list((p/'LQR_six').glob('*.py'))]},passed=all(checks.values()))
(out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(out/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if result['passed'] else 1))+'\n');shutil.copy('/tmp/e002_realtime_final_headless.log',out/'headless_console.log');print(out);print(json.dumps(checks,indent=2));sys.exit(0 if result['passed'] else 1)
