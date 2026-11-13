"""沿用作者 controller.py 的调用顺序；其余 Python 文件统一在 LQR_six。"""
import argparse
import json
from pathlib import Path
import time
import traceback

import mujoco
import numpy as np
from LQR_six.get_sensor import RobotSensor
from LQR_six.write_sensor import RobotController
from LQR_six.leg_data import Leg
from LQR_six.vmc import VMC
from LQR_six.get_state import StateEstimator
from LQR_six.lqr_controller import LQRController
from LQR_six.simulation import create_output, closure_error, save_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control', choices=['lqr', 'lqr-mpc'], default='lqr')
    parser.add_argument('--variant', choices=['no-gas', 'gas'], default='no-gas')
    parser.add_argument('--case', choices=['nominal', 'pitch-plus', 'pitch-minus', 'push', 'roll'], default='nominal')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    if args.duration < 10: parser.error('使用至少 10 s 的短站立验收')
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
        motor = dict(zip(('left_front','left_back','right_front','right_back','left_wheel','right_wheel'),
                         (state['joints'][n] for n in ('joint_l1_L','joint_l6_L','joint_l1_R','joint_l6_R','joint_wheel_L','joint_wheel_R'))))
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
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
        viewer.opt.geomgroup[3] = 0
    rows=[];final_tilts=[];contacts=set()
    max_closure=max_drift=max_passive=0.;min_height=float(data.qpos[2]);termination=None
    active = [int(model.jnt_dofadr[model.actuator_trnid[a,0]]) for a in range(model.nu)]
    passive = [i for i in range(6,model.nv) if i not in active]
    max_torque = np.zeros(model.nu)
    begin=time.perf_counter();steps=round(args.duration/dt)
    try:
        for step in range(steps+1):
            mujoco.mj_forward(model, data)
            imu = read_state()
            lqr.time = data.time
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
            tilt=float(abs(imu['euler'][:2]).max())
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
            if termination or step==steps:break
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
                    final_tilt_rad=max(final_tilts) if final_tilts else 100.,max_xy_drift_m=max_drift,
                    max_closure_m=max_closure,passive_actuator_torque_nm=max_passive,min_height_m=min_height,
                    max_torque_nm=max_torque.tolist(),nonwheel_contacts=sorted(contacts),
                    final_odometry_error_m=float(leg_L.state['s']-data.qpos[0]),wall_s=time.perf_counter()-begin)
        return save_result(out,result,rows,model,data)
    except Exception:
        (out/'error.log').write_text(traceback.format_exc());(out/'exit_status.json').write_text('{"exit_code":2}\n')
        raise
    finally:
        if viewer is not None:viewer.close()


if __name__ == '__main__':
    raise SystemExit(main())
