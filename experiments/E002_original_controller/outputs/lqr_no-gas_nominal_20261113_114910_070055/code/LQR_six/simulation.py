"""只负责运行记录与验收，不参与控制。"""
import datetime
import hashlib
import json
from pathlib import Path
import sys
import mujoco
import numpy as np
import scipy
from .parameters import CONTROL, MONITOR, RECOVERY, RECOVERY_DIRECT, RECOVERY_PREPARED, RECOVERY_BELLY


def create_output(root, args, model):
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out = root/'outputs'/f'{args.control}_{args.variant}_{args.case}_{stamp}'
    out.mkdir(parents=True, exist_ok=False)
    files = [root/'controller.py', root/'standing_pose.json', root/'gain_schedule.json', *sorted((root/'LQR_six').glob('*.py')),
             *sorted((root/'assets').glob('*.xml')), *sorted((root/'scenes').glob('*.xml'))]
    metadata = dict(command=[sys.executable, '-B', *sys.argv], cwd=str(Path.cwd()), python=sys.executable,
                    runtime_paths=dict(project=str(root), output=str(out),
                                       modules={name:str(Path(module.__file__).resolve()) for name,module in sys.modules.items()
                                                if name.startswith('LQR_six') and getattr(module,'__file__',None)}),
                    args=vars(args), versions=dict(mujoco=mujoco.__version__, numpy=np.__version__, scipy=scipy.__version__),
                    actuator_ctrlrange_nm={model.actuator(i).name:model.actuator_ctrlrange[i].tolist()
                                           for i in range(model.nu)},
                    mass_kg=float(model.body_mass.sum()), physics_dt_s=float(model.opt.timestep), control_dt_s=CONTROL['feedback_dt'],
                    feedback='Author six-state LQR/VMC; IMU + wheel odometry; VMC uses full-model linkage Jacobian/qvel',
                    file_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                    thresholds=dict(duration_s=args.duration, max_xy_drift_m=.1, final_tilt_rad=.05,
                                    max_closure_m=1e-4, min_height_m=MONITOR['fall_height'], fall_tilt_rad=MONITOR['fall_tilt'],
                                    fall_is_terminal=False, numerical_error_is_terminal=True),
                    recovery=dict(common=RECOVERY, direct=RECOVERY_DIRECT, prepared=RECOVERY_PREPARED, belly=RECOVERY_BELLY), seed=None)
    (out/'metadata.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False)+'\n')
    for file in files:
        if file.suffix in ('.py', '.json') or file.parent.name=='scenes':
            target=out/'code'/file.relative_to(root);target.parent.mkdir(parents=True, exist_ok=True);target.write_bytes(file.read_bytes())
    (out/'run.log').write_text(json.dumps(metadata, ensure_ascii=False)+'\n')
    return out


def closure_error(model, data, site_pairs=None):
    distances = []
    if site_pairs is None:
        site_pairs = [(model.site(f'{side}_front_site{i}').id, model.site(f'{side}_rear_site{i}').id)
                      for side in ('Left','Right') for i in (1,2)]
    for a,b in site_pairs:
        distances.append(np.linalg.norm(data.site_xpos[a]-data.site_xpos[b]))
    return float(max(distances))


def save_result(out, result, rows, model, data):
    result['warnings'] = [int(w.number) for w in data.warning]
    if result.get('mode') == 'recovery' and result['recovery']['expected_upright']:
        result['acceptance'] = 'upright detection control: no fall or recovery event; stable final 5 s'
        result['passed'] = bool(result['timeout'] and not result['terminated'] and not any(result['warnings'])
                                and not result['recovery']['events'] and result['final_tilt_rad'] < .05)
    elif result.get('mode') == 'recovery':
        recovery = result['recovery']
        result['acceptance'] = ('self-right: completed hip swing and LQR handoff; stable final 5 s, '
                                'full duration, no warning, no passive actuation')
        result['passed'] = bool(result['timeout'] and not result['terminated'] and not any(result['warnings'])
                                and any(e['phase'] == 'swing' for e in recovery['events'])
                                and any(e.get('recovered') for e in recovery['events'])
                                and recovery['phase'] == 'idle' and result['final_tilt_rad'] < .05
                                and recovery['final_speed_m_s'] < .05
                                and result['passive_actuator_torque_nm'] == 0.)
    elif result.get('mode') == 'drive':
        # 键盘驾驶不是位置保持验收，不拿平面漂移 <0.1 m 判失败；只看按时完成、未倒、无警告。
        result['acceptance'] = ('drive demo: full duration, no termination/warnings, final 5 s tilt <0.05 rad, '
                                'passive axes unactuated; xy drift not judged')
        finished = result['timeout'] or (result.get('interactive') and result['termination_reason'] == 'viewer_closed')
        allowed_exit = not result['terminated'] or (result.get('interactive') and result['termination_reason'] == 'viewer_closed')
        result['passed'] = bool(finished and allowed_exit and not any(result['warnings'])
                                and ('step_trial' in result or result['final_tilt_rad'] < .05)
                                and result['passive_actuator_torque_nm'] == 0)
    else:
        result['acceptance'] = ('standing: full duration, final 5 s tilt <0.05 rad, xy drift <0.1 m, '
                                'closure <1e-4 m, no warning, no passive actuation, wheel-only contact')
        result['passed'] = bool(result['timeout'] and not result['terminated'] and not any(result['warnings'])
                                and result['final_tilt_rad'] < .05 and result['max_xy_drift_m'] < .1
                                and result['max_closure_m'] < 1e-4 and result['passive_actuator_torque_nm'] == 0
                                and result['nonwheel_contacts'] == [])
    if 'step_trial' in result:
        result['acceptance'] = ('step trial: full duration, no termination/warnings/falls, '
                                'passive axes unactuated, all step_trial.checks pass')
        result['passed'] = result['passed'] and all(result['step_trial']['checks'].values())
    if result.get('fall_events_s') and result.get('mode') != 'recovery':
        result['passed'] = False  # 继续运行不代表站立/行走验收通过。
    np.savez_compressed(out/'trace.npz', samples=np.array(rows),
                        columns=np.array(['time','base_x','base_y','base_z','roll','pitch','yaw',
                                          'theta','dtheta','s','dot_s','phi','dphi','left_length','right_length',
                                          *[model.actuator(i).name for i in range(model.nu)],
                                          'left_theta_body_rad','right_theta_body_rad',
                                          'left_theta_gravity_rad','right_theta_gravity_rad']))
    text = json.dumps(result, indent=2, ensure_ascii=False)+'\n'
    (out/'results.json').write_text(text)
    with (out/'run.log').open('a') as f: f.write(text)
    (out/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if result['passed'] else 1))+'\n')
    print(out);print(text)
    return 0 if result['passed'] else 1
