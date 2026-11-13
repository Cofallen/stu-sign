"""沿用作者 controller.py 的调用顺序；其余 Python 文件统一在 LQR_six。"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import traceback
import itertools
from collections import deque

# --video 走离屏渲染；MuJoCo 在 import 时选择 GL 后端，必须在导入前设置。
# 有 X 显示时用 GLFW（能用到显卡、快）；纯无头环境退回 EGL。已有 MUJOCO_GL 设置优先。
if any(a == '--video' or a.startswith('--video=') for a in sys.argv[1:]) and 'MUJOCO_GL' not in os.environ:
    os.environ['MUJOCO_GL'] = 'glfw' if os.environ.get('DISPLAY') else 'egl'

import mujoco
import numpy as np
from LQR_six.get_sensor import RobotSensor
from LQR_six.write_sensor import RobotController
from LQR_six.leg_data import Leg
from LQR_six.vmc import VMC
from LQR_six.get_state import StateEstimator
from LQR_six.lqr_controller import LQRController
from LQR_six.render_stand import CAMERA_PRESETS, StandVideo, setup_viewer, load_play_scene
from LQR_six.teleop import Teleop
from LQR_six.step_trial import StepTrial
from LQR_six.parameters import MONITOR
from LQR_six.keyboardInput import KeyboardInput
from LQR_six.simulation import create_output, closure_error, save_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terrain', choices=['flat','training','step10'], default='flat', help='独立台阶/17度斜坡场景')
    parser.add_argument('--control', choices=['lqr', 'lqr-mpc'], default='lqr')
    parser.add_argument('--variant', choices=['no-gas', 'gas'], default='no-gas')
    parser.add_argument('--case', choices=['nominal', 'pitch-plus', 'pitch-minus', 'push', 'roll'], default='nominal')
    parser.add_argument('--duration', type=float, default=None, help='有限评测秒数；默认交互模式持续运行')
    parser.add_argument('--headless', action='store_true', help='无窗口短评测，默认 10 秒')
    parser.add_argument('--viewer', action='store_true', help='打开 MuJoCo 交互窗口')
    parser.add_argument('--no-drive', action='store_true', help='仅站立/调试原生面板，关闭驾驶键')
    parser.add_argument('--no-ui', action='store_true', help='可选：隐藏原生左右面板')
    parser.add_argument('--camera-follow', action='store_true', help='可选：跟随机身；默认自由相机')
    parser.add_argument('--hud', action='store_true', help='可选：以最多10Hz刷新简要文字，默认关闭')
    parser.add_argument('--camera', choices=sorted(CAMERA_PRESETS), default='iso',
                        help='窗口/录像的相机预设：iso 斜视、side 侧视、front 正视、legs 腿部特写')
    parser.add_argument('--video', nargs='?', const='', default=None, metavar='PATH',
                        help='离屏录 mp4；省略 PATH 时写到本次 outputs/；无桌面时也能看')
    parser.add_argument('--video-size', default='960x540', metavar='WxH', help='录像分辨率，宽高须为偶数')
    parser.add_argument('--video-fps', type=int, default=30, help='录像帧率（按仿真时间）')
    parser.add_argument('--no-hud', action='store_true', help='录像不叠加姿态/腿长文字')
    parser.add_argument('--follow', action='store_true', help='录像相机水平跟随基座；默认固定机位便于看漂移')
    parser.add_argument('--hold', action='store_true', help='仿真结束后保持窗口打开，直到手动关闭')
    parser.add_argument('--drive', action='store_true',
                        help='键盘驾驶：W/S 前进后退、A/D 原地转向、Q/E 腿长、X 停、R 复位（需 --viewer）')
    for action in parser._actions:
        if action.dest not in ('camera', 'terrain', 'help', 'headless', 'viewer', 'duration', 'no_ui', 'camera_follow', 'hud', 'no_drive'):
            action.help = argparse.SUPPRESS
    parser.add_argument('--step-trial-x', type=float, metavar='X',
                        help='在 step10 无头短测中，于机身到达 X (m) 时触发一次 J')
    parser.add_argument('--step-up', action='store_true',
                        help='自动跳上独立 10 cm 台阶；可加 --viewer 或 --video')
    parser.add_argument('--reverse-trial', choices=['release', 'switch', 'crouch'],
                        help='无头后退/换向/短腿回归')
    args = parser.parse_args()
    if args.step_up:
        args.terrain = 'step10'
        if args.step_trial_x is None:
            args.step_trial_x = -.85
        if not args.viewer:
            args.headless = True
        if args.duration is None:
            args.duration = 10.
        if args.camera == 'iso':
            args.camera = 'terrain'
    if args.reverse_trial and (not args.headless or args.terrain != 'flat'):
        parser.error('--reverse-trial requires --headless with flat terrain')
    if args.step_trial_x is not None and (
        args.terrain != 'step10' or not (args.headless or (args.step_up and args.viewer)) or args.case != 'nominal'
        or args.variant != 'no-gas' or args.control != 'lqr'
        or not np.isfinite(args.step_trial_x) or not -1.6 < args.step_trial_x < 0
    ):
        parser.error('--step-trial-x 需要 --headless --terrain step10、默认控制/模型/初始条件和 -1.6 < X < 0')
    if not args.headless and args.duration is None and args.video is None:
        args.viewer = True
    if args.viewer:
        args.drive = not args.no_drive
    if args.hold and args.viewer:
        args.duration = None  # 继续真正积分，不能只保留一个冻结窗口。
    if not args.viewer and args.duration is None:
        args.duration = 10.
    if args.duration is not None and args.duration < 10:
        parser.error('短评测至少 10 s')
    if args.viewer and args.headless:
        parser.error('--headless 与 --viewer 二选一')
    if args.viewer and args.video is not None:
        parser.error('--viewer 与 --video 会争用 GL 后端，请二选一')
    if args.drive and not args.viewer:
        parser.error('--drive 需要同时打开 --viewer；无桌面请用 --video 只看不控')
    if args.video_fps <= 0: parser.error('--video-fps 必须为正整数')
    try:
        width, height = (int(v) for v in args.video_size.lower().split('x'))
    except ValueError:
        parser.error('--video-size 需要 WxH，例如 960x540')
    if args.video is not None and (width % 2 or height % 2):
        parser.error('--video-size 宽高必须是偶数（libx264 yuv420p）')
    root = Path(__file__).resolve().parent
    filename = 'robot_no_gas_spring.xml' if args.variant == 'no-gas' else 'robot_gas_spring_damped.xml'
    scene = {'training': 'training_terrain.xml', 'step10': 'single_step_10cm.xml'}.get(args.terrain)
    terrain = root/'scenes'/scene if scene else None
    model = load_play_scene(root/'assets'/filename, terrain)
    data = mujoco.MjData(model)
    pose = json.loads((root/'standing_pose.json').read_text())[args.variant]
    data.qpos[:] = pose['qpos']  # 只在 reset 写姿态，积分中只写原六个电机。
    data.ctrl[:] = pose['ctrl']
    mujoco.mj_forward(model, data)
    sensor = RobotSensor(model, data)
    controller = RobotController(model, data)
    dt = model.opt.timestep
    vmc = VMC(model, data)
    leg_L, leg_R = Leg(dt), Leg(dt)
    state_estimator = StateEstimator(dt)
    lqr = LQRController(mode=args.control, model=model, variant=args.variant)

    def read_state():
        state = sensor.get_state()
        imu = {'acc':state['acc'], 'gyro':state['gyro'], 'euler':state['euler']}
        motor = {
            'left_front': state['joints']['joint_l1_L'],
            'left_back': state['joints']['joint_l6_L'],
            'right_front': state['joints']['joint_l1_R'],
            'right_back': state['joints']['joint_l6_R'],
            'left_wheel': state['joints']['joint_wheel_L'],
            'right_wheel': state['joints']['joint_wheel_R'],
        }
        vmc.update_left(leg_L, motor)
        vmc.update_right(leg_R, motor)
        state_estimator.update_left(leg_L, imu, dt)
        state_estimator.update_right(leg_R, imu, dt)
        state_estimator.update(leg_L, leg_R, motor, imu)
        return imu

    imu = read_state()
    lqr.set_standing_reference(leg_L, leg_R, np.array(pose['ctrl']))
    trial = StepTrial(model, args.step_trial_x) if args.step_trial_x is not None else None
    teleop = Teleop(leg_L, leg_R) if args.drive or trial is not None or args.reverse_trial else None
    perturb = np.zeros(model.nv)
    if args.case == 'pitch-plus': perturb[4] = .03
    if args.case == 'pitch-minus': perturb[4] = -.03
    if args.case == 'roll': perturb[3] = .03
    mujoco.mj_integratePos(model, data.qpos, perturb, 1.)
    state_estimator = StateEstimator(dt)  # 初始化编码器差分，避免把 reset 跳变当速度。
    out = create_output(root, args, model)
    viewer = video = keyboard = None
    initial_view = None
    timing = dict(sync_calls=0, sync_wall_s=0., sync_max_s=0., hud_calls=0, hud_wall_s=0.,
                  sleep_wall_s=0., late_steps=0, max_lag_s=0., initial_sync_wall_s=0.)
    hud_cache = [None, 0.]
    if args.viewer:
        from mujoco import viewer as mjviewer
        try:
            keyboard = KeyboardInput() if teleop is not None and trial is None else None
            viewer = mjviewer.launch_passive(model, data,
                                             key_callback=keyboard.on_key if keyboard else None,
                                             show_left_ui=not args.no_ui, show_right_ui=not args.no_ui)
        except Exception as exc:
            raise SystemExit(f'无法打开 MuJoCo 窗口：{exc}\n没有可用桌面时请改用 --video 录像。')
        setup_viewer(viewer, model, args.camera, follow=args.camera_follow)
        initial_sync = time.perf_counter()
        viewer.sync()  # 启动时全量同步一次；运行中只同步状态。
        timing['initial_sync_wall_s'] = time.perf_counter()-initial_sync
        initial_view = dict(azimuth=float(viewer.cam.azimuth), elevation=float(viewer.cam.elevation),
                            vis_flags=viewer.opt.flags.tolist(), render_flags=viewer.user_scn.flags.tolist())
        print('窗口已打开：左键旋转、右键平移、滚轮缩放、H 帮助；关闭窗口即结束。', flush=True)
        if teleop is not None:
            print(Teleop.help_text()+'；鼠标移入三维区启用驾驶，移入侧面板或失焦停止。', flush=True)

    def update_viewer_text(force=False):
        if viewer is None or teleop is None or not args.hud:
            return
        text = (f"t={data.time:.1f}s x={data.qpos[0]:+.2f}m v={teleop.speed:+.2f}m/s "
                f"l0={teleop.l0:.3f}m {teleop.jump_phase}")
        now = time.perf_counter()
        if not force and (text == hud_cache[0] or now-hud_cache[1] < .1):
            return
        hud_cache[:] = [text, now]
        started = time.perf_counter()
        viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                          text, 'W/S A/D Q/E hold; J jump; release stops')])
        timing['hud_wall_s'] += time.perf_counter()-started
        timing['hud_calls'] += 1

    update_viewer_text(force=True)
    timing['initial_hud_wall_s'] = timing['hud_wall_s']
    timing['initial_hud_calls'] = timing['hud_calls']
    timing['hud_wall_s'] = 0.
    timing['hud_calls'] = 0
    if args.video is not None:
        video_path = Path(args.video) if args.video else out/f'standing_{args.case}.mp4'
        video = StandVideo(model, video_path, width=width, height=height, fps=args.video_fps,
                           camera=args.camera, hud=not args.no_hud, follow=args.follow)
        print(f'录制中：{video.path}（{width}x{height} @ {args.video_fps} fps，相机 {args.camera}）', flush=True)
    rows=[];motion=[];final_tilts=deque(maxlen=5001);contacts=set()
    max_external_force=0.
    max_closure=max_drift=max_passive=max_tilt=0.;min_height=float(data.qpos[2]);termination=None
    fall_events=[]; was_fallen=False
    closure_sites = [(model.site(f'{side}_front_site{i}').id, model.site(f'{side}_rear_site{i}').id)
                     for side in ('Left','Right') for i in (1,2)]
    geom_names = [model.geom(i).name for i in range(model.ngeom)]
    ground_names = {'floor'} | {n for n in geom_names if n.startswith(('step10_', 'step20_', 'ramp17'))}
    active = [int(model.jnt_dofadr[model.actuator_trnid[a,0]]) for a in range(model.nu)]
    passive = [i for i in range(6,model.nv) if i not in active]
    max_torque = np.zeros(model.nu)
    begin=time.perf_counter();steps=round(args.duration/dt) if args.duration is not None else None

    def video_info(status):
        return dict(control=args.control, variant=args.variant, case=args.case,
                    t=float(data.time), duration=args.duration,
                    roll=float(imu['euler'][0]), pitch=float(imu['euler'][1]),
                    tilt_peak=max_tilt, length_l=float(leg_L.vmc['L0']),
                    length_r=float(leg_R.vmc['L0']), base_z=float(data.qpos[2]), status=status)

    try:
        for step in itertools.count():
            mujoco.mj_forward(model, data)
            imu = read_state()
            max_external_force=max(max_external_force,float(np.linalg.norm(data.xfrc_applied[:,:3],axis=1).max()))
            lqr.time = data.time
            tilt = float(abs(imu['euler'][:2]).max())
            if teleop is not None:
                if trial is not None:
                    trial.sample(data, tilt, teleop)
                if args.reverse_trial:
                    t = data.time
                    keys = set()
                    if args.reverse_trial == 'crouch' and .25 <= t < 1:
                        keys.add('q')
                    if 1 <= t < 4:
                        keys.add('s')
                    elif args.reverse_trial == 'switch' and 4 <= t < 6:
                        keys.add('w')
                    teleop.set_pressed(keys)
                if keyboard is not None and step%10 == 0:
                    keyboard.set_viewport(viewer.viewport)
                    pressed, actions = keyboard.poll()
                    teleop.set_pressed(pressed)
                    for key in actions:
                        teleop.on_key(key)
                teleop.observe(model, data, dt)
                teleop.update(imu, dt)
            max_tilt=max(max_tilt,tilt)
            # 可视化只读当前状态；帧按仿真时间取，不改变控制周期。
            if video is not None:
                video.maybe_capture(data,video_info('standing' if tilt<.05 else 'recovering'))
            # 核心控制与作者一致：六状态反馈 → 支撑/姿态修正 → VMC → 六电机。
            tau_L, tau_w_L = lqr.control_left(leg_L, imu)
            tau_R, tau_w_R = lqr.control_right(leg_R, imu)
            controller.set_actuator('left_front', -tau_L[0])
            controller.set_actuator('left_back', -tau_L[1])
            controller.set_actuator('right_front', tau_R[0])
            controller.set_actuator('right_back', tau_R[1])
            controller.set_actuator('left_wheel', tau_w_L)    # 左轮正转向前。
            controller.set_actuator('right_wheel', -tau_w_R)  # 右轮转轴相反。

            # 以下只做日志/验收，不回馈控制器。
            max_closure=max(max_closure,closure_error(model,data,closure_sites))
            max_drift=max(max_drift,float(np.linalg.norm(data.qpos[:2])))
            max_passive=max(max_passive,float(abs(data.qfrc_actuator[passive]).max()))
            min_height=min(min_height,float(data.qpos[2]));max_torque=np.maximum(max_torque,abs(data.ctrl))
            if args.duration is None or data.time >= args.duration-5: final_tilts.append(tilt)
            for contact in data.contact:
                for g in contact.geom:
                    name=geom_names[int(g)]
                    if name not in ground_names and name not in ('l_wheel_Link_collision_0','r_wheel_Link_collision_0'):contacts.add(name)
            fallen = tilt>MONITOR['fall_tilt']  # 下蹲/落地压缩不能仅因高度低而关闭支撑
            if fallen and not was_fallen:
                fall_events.append(float(data.time))
                print(f'[fall t={data.time:.2f}] 继续仿真；倒地平衡输出归零，扶正自动恢复', flush=True)
            if fallen and MONITOR['disable_balance_when_fallen']:
                data.ctrl[:] = 0.
            was_fallen = fallen
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):termination='numerical_error'
            if step%10==0:
                motion.append([data.time, data.qpos[12], data.qpos[9], data.ncon, *data.qpos[:3]])
                rows.append([data.time,*data.qpos[:3],*imu['euler'],*[leg_L.state[k] for k in ('theta','dtheta','s','dot_s','phi','dphi')],leg_L.vmc['L0'],leg_R.vmc['L0'],*data.ctrl])
            if termination or (steps is not None and step>=steps):
                if video is not None:
                    video.capture_final(data,video_info('terminated' if termination else
                                                        ('standing' if tilt<.05 else 'recovering')))
                break
            if args.case=='push':
                data.xfrc_applied[:]=0
                if 5<=data.time<5.2:data.xfrc_applied[1,0]=10.
            elif viewer is None:
                # 无窗口的验收运行保持外力为零；窗口模式保留 MuJoCo 交互拖动产生的外力。
                data.xfrc_applied[:]=0
            mujoco.mj_step(model,data)
            if viewer is not None:
                if not viewer.is_running():termination='viewer_closed';break
                if step%33==0:  # 约30Hz显示，与1kHz物理/控制分离。
                    update_viewer_text()
                    sync_started = time.perf_counter()
                    viewer.sync(state_only=True)
                    sync_elapsed = time.perf_counter()-sync_started
                    timing['sync_wall_s'] += sync_elapsed
                    timing['sync_max_s'] = max(timing['sync_max_s'], sync_elapsed)
                    timing['sync_calls'] += 1
                delay=begin+data.time-time.perf_counter()
                if delay>0:
                    sleep_started=time.perf_counter()
                    time.sleep(delay)
                    timing['sleep_wall_s'] += time.perf_counter()-sleep_started
                else:
                    timing['max_lag_s'] = max(timing['max_lag_s'], -delay)
                    timing['late_steps'] += int(-delay > dt)
        wall_s = time.perf_counter()-begin
        timing['real_time_factor'] = float(data.time/wall_s) if wall_s else 0.
        timing['loop_non_ui_non_sleep_s'] = wall_s-timing['sync_wall_s']-timing['hud_wall_s']-timing['sleep_wall_s']
        result=dict(timing=timing, viewer_options=dict(camera_type=int(viewer.cam.type), left_ui=not args.no_ui, right_ui=not args.no_ui, hud=args.hud, initial=initial_view,
                    final=dict(azimuth=float(viewer.cam.azimuth), elevation=float(viewer.cam.elevation),
                               vis_flags=viewer.opt.flags.tolist(), render_flags=viewer.user_scn.flags.tolist())) if viewer is not None else None, case=args.case,control=args.control,variant=args.variant,duration_s=data.time,
                    terminated=termination is not None,termination_reason=termination,timeout=termination is None and steps is not None and step>=steps,
                    final_tilt_rad=max(final_tilts) if final_tilts else 100.,max_tilt_rad=max_tilt,max_xy_drift_m=max_drift,
                    max_external_force_n=max_external_force,max_closure_m=max_closure,passive_actuator_torque_nm=max_passive,min_height_m=min_height,
                    max_torque_nm=max_torque.tolist(),nonwheel_contacts=sorted(contacts),
                    mpc_solves=lqr.mpc.solve_count if lqr.mpc is not None else 0,
                    video=str(video.path) if video is not None else None,
                    camera=args.camera if (viewer is not None or video is not None) else None,
                    final_base_xy_m=[float(data.qpos[0]),float(data.qpos[1])],
                    final_yaw_rad=float(imu['euler'][2]),
                    mode='drive' if teleop is not None else 'stand',
                    interactive=args.viewer and args.duration is None,
                    teleop=teleop.status() if teleop is not None else None,
                    final_odometry_error_m=float(leg_L.state['s']-data.qpos[0]),wall_s=wall_s)
        np.savez_compressed(out/'motion.npz', samples=np.array(motion),
                            columns=np.array(['time','left_wheel_angle','right_wheel_angle','contact_count','base_x','base_y','base_z']))
        result['fall_events_s'] = fall_events
        result['gain_schedule'] = dict(lengths_m=lqr.gain_length, K_left=lqr.K_left.tolist(),
                                       K_right=lqr.K_right.tolist(), masses=lqr.masses,
                                       range_m=[lqr.gains.minimum,lqr.gains.maximum])
        if trial is not None:
            result['step_trial'] = trial.finish(out, result)
        saved=save_result(out,result,rows,model,data)
        if viewer is not None:
            # 本机 GLFW/passive viewer 在解释器退出阶段会崩溃；结果已写盘，关闭后直接结束进程。
            viewer.close()
            if keyboard is not None:keyboard.close()
            sys.stdout.flush();sys.stderr.flush();os._exit(saved)
        return saved
    except Exception:
        (out/'error.log').write_text(traceback.format_exc());(out/'exit_status.json').write_text('{"exit_code":2}\n')
        if viewer is not None:
            try:viewer.close()
            except Exception:pass
            sys.stdout.flush();sys.stderr.flush();os._exit(2)
        raise
    finally:
        if keyboard is not None:keyboard.close()
        if video is not None:video.close()
        if viewer is not None:
            try:viewer.close()
            except Exception:pass


if __name__ == '__main__':
    raise SystemExit(main())
