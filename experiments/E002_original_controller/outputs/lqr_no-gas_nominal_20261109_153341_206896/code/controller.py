"""沿用作者 controller.py 的调用顺序；其余 Python 文件统一在 LQR_six。"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import traceback

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
from LQR_six.render_stand import CAMERA_PRESETS, StandVideo, setup_viewer
from LQR_six.simulation import create_output, closure_error, save_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control', choices=['lqr', 'lqr-mpc'], default='lqr')
    parser.add_argument('--variant', choices=['no-gas', 'gas'], default='no-gas')
    parser.add_argument('--case', choices=['nominal', 'pitch-plus', 'pitch-minus', 'push', 'roll'], default='nominal')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--viewer', action='store_true', help='打开 MuJoCo 交互窗口')
    parser.add_argument('--camera', choices=sorted(CAMERA_PRESETS), default='iso',
                        help='窗口/录像的相机预设：iso 斜视、side 侧视、front 正视、legs 腿部特写')
    parser.add_argument('--video', nargs='?', const='', default=None, metavar='PATH',
                        help='离屏录 mp4；省略 PATH 时写到本次 outputs/；无桌面时也能看')
    parser.add_argument('--video-size', default='960x540', metavar='WxH', help='录像分辨率，宽高须为偶数')
    parser.add_argument('--video-fps', type=int, default=30, help='录像帧率（按仿真时间）')
    parser.add_argument('--no-hud', action='store_true', help='录像不叠加姿态/腿长文字')
    parser.add_argument('--follow', action='store_true', help='录像相机水平跟随基座；默认固定机位便于看漂移')
    parser.add_argument('--hold', action='store_true', help='仿真结束后保持窗口打开，直到手动关闭')
    args = parser.parse_args()
    if args.duration < 10: parser.error('使用至少 10 s 的短站立验收')
    if args.viewer and args.video is not None:
        parser.error('--viewer 与 --video 会争用 GL 后端，请二选一')
    if args.video_fps <= 0: parser.error('--video-fps 必须为正整数')
    try:
        width, height = (int(v) for v in args.video_size.lower().split('x'))
    except ValueError:
        parser.error('--video-size 需要 WxH，例如 960x540')
    if args.video is not None and (width % 2 or height % 2):
        parser.error('--video-size 宽高必须是偶数（libx264 yuv420p）')
    root = Path(__file__).resolve().parent
    filename = 'robot_no_gas_spring.xml' if args.variant == 'no-gas' else 'robot_gas_spring_damped.xml'
    model = mujoco.MjModel.from_xml_path(str(root/'assets'/filename))
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
    lqr = LQRController(mode=args.control, dt=.01)

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

    read_state()
    lqr.set_standing_reference(leg_L, leg_R, np.array(pose['ctrl']))
    perturb = np.zeros(model.nv)
    if args.case == 'pitch-plus': perturb[4] = .03
    if args.case == 'pitch-minus': perturb[4] = -.03
    if args.case == 'roll': perturb[3] = .03
    mujoco.mj_integratePos(model, data.qpos, perturb, 1.)
    state_estimator = StateEstimator(dt)  # 初始化编码器差分，避免把 reset 跳变当速度。
    out = create_output(root, args, model)
    viewer = video = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        try:
            viewer = mjviewer.launch_passive(model, data)
        except Exception as exc:
            raise SystemExit(f'无法打开 MuJoCo 窗口：{exc}\n没有可用桌面时请改用 --video 录像。')
        setup_viewer(viewer, model, args.camera)
        print('窗口已打开：左键旋转、右键平移、滚轮缩放、H 帮助；关闭窗口即结束。', flush=True)
    if args.video is not None:
        video_path = Path(args.video) if args.video else out/f'standing_{args.case}.mp4'
        video = StandVideo(model, video_path, width=width, height=height, fps=args.video_fps,
                           camera=args.camera, hud=not args.no_hud, follow=args.follow)
        print(f'录制中：{video.path}（{width}x{height} @ {args.video_fps} fps，相机 {args.camera}）', flush=True)
    rows=[];final_tilts=[];contacts=set()
    max_closure=max_drift=max_passive=max_tilt=0.;min_height=float(data.qpos[2]);termination=None
    active = [int(model.jnt_dofadr[model.actuator_trnid[a,0]]) for a in range(model.nu)]
    passive = [i for i in range(6,model.nv) if i not in active]
    max_torque = np.zeros(model.nu)
    begin=time.perf_counter();steps=round(args.duration/dt)

    def video_info(status):
        return dict(control=args.control, variant=args.variant, case=args.case,
                    t=float(data.time), duration=args.duration,
                    roll=float(imu['euler'][0]), pitch=float(imu['euler'][1]),
                    tilt_peak=max_tilt, length_l=float(leg_L.vmc['L0']),
                    length_r=float(leg_R.vmc['L0']), base_z=float(data.qpos[2]), status=status)

    try:
        for step in range(steps+1):
            mujoco.mj_forward(model, data)
            imu = read_state()
            lqr.time = data.time
            tilt=float(abs(imu['euler'][:2]).max());max_tilt=max(max_tilt,tilt)
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
            max_closure=max(max_closure,closure_error(model,data))
            max_drift=max(max_drift,float(np.linalg.norm(data.qpos[:2])))
            max_passive=max(max_passive,float(abs(data.qfrc_actuator[passive]).max()))
            min_height=min(min_height,float(data.qpos[2]));max_torque=np.maximum(max_torque,abs(data.ctrl))
            if data.time >= args.duration-5: final_tilts.append(tilt)
            for contact in data.contact:
                for g in contact.geom:
                    name=model.geom(int(g)).name
                    if name not in ('floor','l_wheel_Link_collision_0','r_wheel_Link_collision_0'):contacts.add(name)
            if data.qpos[2]<.12 or tilt>.6:termination='fall'
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):termination='numerical_error'
            if step%10==0:
                rows.append([data.time,*data.qpos[:3],*imu['euler'],*[leg_L.state[k] for k in ('theta','dtheta','s','dot_s','phi','dphi')],leg_L.vmc['L0'],leg_R.vmc['L0'],*data.ctrl])
            if termination or step==steps:
                if video is not None:
                    video.capture_final(data,video_info('terminated' if termination else
                                                        ('standing' if tilt<.05 else 'recovering')))
                break
            data.xfrc_applied[:]=0
            if args.case=='push' and 5<=data.time<5.2:data.xfrc_applied[1,0]=10.
            mujoco.mj_step(model,data)
            if viewer is not None:
                if not viewer.is_running():termination='viewer_closed';break
                if step%10==0:viewer.sync()
                delay=begin+data.time-time.perf_counter()
                if delay>0:time.sleep(delay)
        result=dict(case=args.case,control=args.control,variant=args.variant,duration_s=data.time,
                    terminated=termination is not None,termination_reason=termination,timeout=termination is None and step==steps,
                    final_tilt_rad=max(final_tilts) if final_tilts else 100.,max_tilt_rad=max_tilt,max_xy_drift_m=max_drift,
                    max_closure_m=max_closure,passive_actuator_torque_nm=max_passive,min_height_m=min_height,
                    max_torque_nm=max_torque.tolist(),nonwheel_contacts=sorted(contacts),
                    mpc_solves=lqr.mpc.solve_count if lqr.mpc is not None else 0,
                    video=str(video.path) if video is not None else None,
                    camera=args.camera if (viewer is not None or video is not None) else None,
                    final_odometry_error_m=float(leg_L.state['s']-data.qpos[0]),wall_s=time.perf_counter()-begin)
        saved=save_result(out,result,rows,model,data)
        if viewer is not None:
            if args.hold:
                print('窗口保持打开；关闭窗口或 Ctrl-C 结束。', flush=True)
                while viewer.is_running():
                    viewer.sync();time.sleep(.05)
            # 本机 GLFW/passive viewer 在解释器退出阶段会崩溃；结果已写盘，关闭后直接结束进程。
            viewer.close();sys.stdout.flush();sys.stderr.flush();os._exit(saved)
        return saved
    except Exception:
        (out/'error.log').write_text(traceback.format_exc());(out/'exit_status.json').write_text('{"exit_code":2}\n')
        if viewer is not None:
            try:viewer.close()
            except Exception:pass
            sys.stdout.flush();sys.stderr.flush();os._exit(2)
        raise
    finally:
        if video is not None:video.close()
        if viewer is not None:
            try:viewer.close()
            except Exception:pass


if __name__ == '__main__':
    raise SystemExit(main())
