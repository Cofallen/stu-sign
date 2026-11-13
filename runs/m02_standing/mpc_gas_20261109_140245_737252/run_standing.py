#!/usr/bin/env python3
"""Run complete-parallel-robot standing; every invocation creates new evidence."""
import argparse
import contextlib
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


class Tee:
    def __init__(self, *streams): self.streams = streams
    def write(self, s):
        for stream in self.streams: stream.write(s); stream.flush()
    def flush(self):
        for stream in self.streams: stream.flush()


def evaluate(args, out):
    if not args.viewer:
        os.environ.setdefault('MUJOCO_GL', 'egl')
        os.environ.setdefault('EGL_PLATFORM', 'surfaceless')
        os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
        os.environ.setdefault('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/50_mesa.json')
    import mujoco
    import numpy as np
    import scipy
    from wheel_leg.control.standing import (StandingController, closure_errors,
                                           design_feedback, euler, solve_equilibrium)

    filename = 'robot_no_gas_spring.xml' if args.variant == 'no-gas' else 'robot_gas_spring_damped.xml'
    xml = ROOT/'assets/robots/infantry_parallel'/filename
    model = mujoco.MjModel.from_xml_path(str(xml))
    assert model.nq == 21 and model.nv == 20 and model.nu == 6 and model.neq == 4
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert np.allclose(model.opt.gravity, [0, 0, -9.81])
    mapping = []
    active_dofs = []
    for a in range(model.nu):
        j = int(model.actuator_trnid[a, 0])
        active_dofs.append(int(model.jnt_dofadr[j]))
        mapping.append(dict(actuator=model.actuator(a).name, joint=model.joint(j).name,
                            qpos_address=int(model.jnt_qposadr[j]), dof_address=int(model.jnt_dofadr[j]),
                            gear=float(model.actuator_gear[a, 0]), torque_range_nm=model.actuator_ctrlrange[a].tolist()))
    passive = [i for i in range(6, model.nv) if i not in active_dofs]
    write_json(out/'actuator_mapping.json', mapping)
    metadata = dict(command=[sys.executable, '-B', *sys.argv], cwd=str(Path.cwd()),
                    python=sys.executable, versions=dict(mujoco=mujoco.__version__, numpy=np.__version__, scipy=scipy.__version__),
                    args=vars(args), model=str(xml.relative_to(ROOT)), model_sha256=digest(xml),
                    source_main_commit='b0f14e8ee9331d572738885691389742028cb796',
                    source_lqr_mpc_commit='f8039cde95b8c2a9150c6016621ea928c9215c36',
                    code_sha256={str(p.relative_to(ROOT)): digest(p) for p in
                                 [Path(__file__).resolve(), ROOT/'src/wheel_leg/control/standing.py']},
                    mesh_sha256={p.name: digest(p) for p in xml.parent.glob('*.stl')},
                    seed=None, randomness='Deterministic cases, no randomized initialization or training',
                    physics_dt_s=float(model.opt.timestep), control_decimation=10,
                    gravity=model.opt.gravity.tolist(), nq=model.nq, nv=model.nv, nu=model.nu, neq=model.neq,
                    mass_kg=float(model.body_mass.sum()), state_access='privileged MuJoCo qpos/qvel; no deployable estimator',
                    thresholds=dict(duration_s=args.duration, final_window_s=min(5., args.duration),
                                    final_roll_pitch_peak_rad=.05, max_xy_drift_m=.1,
                                    max_closure_m=1e-4, fall_height_m=.12, fall_roll_pitch_rad=.6,
                                    solver_failures=0, numerical_warnings=0, passive_actuator_torque_nm=0))
    write_json(out/'metadata.json', metadata)
    print('Solving reset-only assembled/contact equilibrium...', flush=True)
    qref, uref, equilibrium = solve_equilibrium(model)
    design = design_feedback(model, qref, uref, decimation=10)
    design.save(out/'design.npz')
    write_json(out/'design.json', dict(equilibrium=equilibrium, feedback=design.info,
                                     qref=qref.tolist(), uref=uref.tolist()))
    print(json.dumps(dict(equilibrium=equilibrium, feedback=design.info)), flush=True)
    results = []
    for case in args.cases.split(','):
        if case not in ('nominal', 'pitch-plus', 'pitch-minus', 'pitch-large', 'push', 'roll'):
            raise ValueError(f'Unknown case: {case}')
        d = mujoco.MjData(model)
        d.qpos[:] = qref
        perturb = np.zeros(model.nv)
        if case == 'pitch-plus': perturb[4] = .03
        if case == 'pitch-minus': perturb[4] = -.03
        if case == 'pitch-large': perturb[4] = .1
        if case == 'roll': perturb[3] = .03
        mujoco.mj_integratePos(model, d.qpos, perturb, 1.)  # reset only
        mujoco.mj_forward(model, d)
        controller = StandingController(model, design, mode=args.controller, horizon=args.horizon)
        start_xy = d.qpos[:2].copy()
        rows, qrows, vrows, urows = [], [], [], []
        max_closure = max_drift = max_passive = 0.
        max_motor = np.zeros(model.nu)
        min_height = float(d.qpos[2])
        max_tilt = 0.
        final_tilts = []
        contacts_min = 1000
        nonwheel_contacts = set()
        termination = None
        finite = True
        renderer = encoder = None
        if args.video:
            from PIL import Image, ImageDraw
            renderer = mujoco.Renderer(model, height=480, width=640)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = 1.4, 135, -15
            options = mujoco.MjvOption()
            options.geomgroup[3] = 0
            options.sitegroup[:] = 0
            encoder = subprocess.Popen(['/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'error',
                                        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '640x480',
                                        '-r', '10', '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt',
                                        'yuv420p', str(out/f'{case}.mp4')], stdin=subprocess.PIPE)
        viewer = None
        if args.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, d)
            viewer.opt.geomgroup[3] = 0
            viewer.opt.sitegroup[:] = 0
        begin = time.perf_counter()
        steps = round(args.duration/model.opt.timestep)
        command_times = []
        try:
            for step in range(steps+1):
                mujoco.mj_forward(model, d)
                angles = euler(d)
                tilt = float(np.max(np.abs(angles[:2])))
                closure = float(closure_errors(model, d).max())
                drift = float(np.linalg.norm(d.qpos[:2]-start_xy))
                max_closure, max_drift = max(max_closure, closure), max(max_drift, drift)
                max_tilt, min_height = max(max_tilt, tilt), min(min_height, float(d.qpos[2]))
                max_passive = max(max_passive, float(np.max(np.abs(d.qfrc_actuator[passive]))))
                if d.time >= args.duration-5.: final_tilts.append(tilt)
                contacts_min = min(contacts_min, d.ncon)
                for contact in d.contact:
                    names = [model.geom(int(g)).name for g in contact.geom]
                    nonwheel_contacts.update(n for n in names if n not in
                                             ('floor', 'l_wheel_Link_collision_0', 'r_wheel_Link_collision_0'))
                finite = bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
                if not finite: termination = 'nonfinite'
                elif d.qpos[2] < .12: termination = 'fall_height'
                elif tilt > .6: termination = 'fall_tilt'
                elif any(w.number for w in d.warning): termination = 'numerical_warning'
                if step % 10 == 0 or step == steps or termination:
                    rows.append([d.time, *d.qpos[:3], *angles, closure, drift, *d.qvel[:6]])
                    qrows.append(d.qpos.copy()); vrows.append(d.qvel.copy()); urows.append(d.ctrl.copy())
                if renderer is not None and step % 100 == 0:
                    camera.lookat[:] = [d.qpos[0], d.qpos[1], .25]
                    renderer.update_scene(d, camera=camera, scene_option=options)
                    picture = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(picture)
                    draw.rectangle((0, 0, 640, 52), fill='black')
                    draw.text((8, 5), f'{args.controller.upper()} | {args.variant} | {case} | t={d.time:.2f}s', fill='white')
                    draw.text((8, 23), 'Free base / gravity ON / 6 motors / 4 closed pins', fill='white')
                    draw.text((8, 37), f'Pitch {angles[1]:+.4f} rad | closure {closure:.2e} m', fill='white')
                    encoder.stdin.write(np.asarray(picture).tobytes())
                    if step == 0: picture.save(out/f'{case}_initial.png')
                    if step == steps: picture.save(out/f'{case}_final.png')
                if viewer is not None and step % 10 == 0:
                    if not viewer.is_running():
                        termination = 'viewer_closed'; break
                    viewer.sync()
                    delay = begin+d.time-time.perf_counter()
                    if delay > 0: time.sleep(delay)
                if termination or step == steps: break
                if step % 10 == 0:
                    t0 = time.perf_counter()
                    d.ctrl[:] = controller.command(design.error(model, d))
                    command_times.append(time.perf_counter()-t0)
                max_motor = np.maximum(max_motor, np.abs(d.ctrl))
                d.xfrc_applied[:] = 0
                if case == 'push' and 5. <= d.time < 5.2:
                    d.xfrc_applied[model.body('base_Link_del').id, 0] = 10.
                mujoco.mj_step(model, d)
        finally:
            if viewer is not None: viewer.close()
            if encoder is not None:
                encoder.stdin.close()
                if encoder.wait() != 0: raise RuntimeError('ffmpeg failed')
                renderer.close()
        timeout = termination is None and step == steps
        last_peak = max(final_tilts) if final_tilts else None
        result = dict(case=case, duration_s=float(d.time), terminated=termination is not None,
                      termination_reason=termination, timeout=timeout, finite=finite,
                      final_roll_pitch_peak_rad=last_peak, maximum_tilt_rad=max_tilt,
                      max_xy_drift_m=max_drift, final_xy_drift_m=float(np.linalg.norm(d.qpos[:2]-start_xy)),
                      min_base_height_m=min_height, max_closure_m=max_closure,
                      max_passive_actuator_torque_nm=max_passive, max_motor_torque_nm=max_motor.tolist(),
                      warnings=[int(w.number) for w in d.warning], min_contacts=contacts_min,
                      nonwheel_contacts=sorted(nonwheel_contacts), wall_seconds=time.perf_counter()-begin,
                      controller_solves=controller.solves, constrained_mpc_solves=controller.constrained_solves,
                      solver_failures=controller.solver_failures, max_solver_iterations=controller.max_iterations,
                      saturation_ticks=controller.saturation_ticks,
                      command_wall_ms_p50_p99_max=(np.percentile(command_times, [50, 99, 100])*1000).tolist(),
                      disturbance=('world +X 10 N at base COM, t=[5,5.2)s' if case == 'push' else None),
                      initialization_perturbation_rad=perturb[3:6].tolist())
        result['passed'] = bool(timeout and finite and last_peak is not None and last_peak < .05
                                and max_drift < .1 and max_closure < 1e-4 and max_passive == 0
                                and not any(result['warnings']) and not nonwheel_contacts
                                and controller.solver_failures == 0
                                and np.all(max_motor <= np.maximum(abs(controller.low), abs(controller.high))+1e-9))
        np.savez_compressed(out/f'{case}_trace.npz', columns=np.array(['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                                                                     'closure', 'drift', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']),
                            metrics=np.array(rows), qpos=np.array(qrows), qvel=np.array(vrows), ctrl=np.array(urows))
        results.append(result)
        write_json(out/'results.json', dict(passed=all(r['passed'] for r in results), results=results))
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if all(r['passed'] for r in results) else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller', choices=['lqr', 'mpc'], default='lqr')
    p.add_argument('--variant', choices=['no-gas', 'gas'], default='no-gas')
    p.add_argument('--duration', type=float, default=30.)
    p.add_argument('--cases', default='nominal,pitch-plus,pitch-minus,push,roll')
    p.add_argument('--horizon', type=int, default=15)
    p.add_argument('--video', action='store_true')
    p.add_argument('--viewer', action='store_true')
    args = p.parse_args()
    if args.duration < 1 or args.horizon < 2: p.error('duration >= 1, horizon >= 2 required')
    if 'push' in args.cases.split(',') and args.duration < 10: p.error('push requires >= 10 s')
    stamp = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out = ROOT/'runs/m02_standing'/f'{args.controller}_{args.variant}_{stamp}'
    out.mkdir(parents=True, exist_ok=False)
    print(out, flush=True)
    # Snapshot the executed implementation, not just its mutable current path.
    for source in [Path(__file__).resolve(), ROOT/'src/wheel_leg/control/standing.py']:
        (out/source.name).write_bytes(source.read_bytes())
    write_json(out/'invocation.json', dict(command=[sys.executable, '-B', *sys.argv], cwd=str(Path.cwd()), args=vars(args)))
    code = 2
    with (out/'run.log').open('w') as log, contextlib.redirect_stdout(Tee(sys.stdout, log)), contextlib.redirect_stderr(Tee(sys.stderr, log)):
        try: code = evaluate(args, out)
        except Exception:
            traceback.print_exc()
        finally: write_json(out/'exit_status.json', dict(exit_code=code))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
