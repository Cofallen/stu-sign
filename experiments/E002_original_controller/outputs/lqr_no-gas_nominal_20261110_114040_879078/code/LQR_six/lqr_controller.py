"""由作者 LQRController 适配：六状态、支撑 PID、左右 VMC 输出保持原结构。"""
import numpy as np
import mujoco
from .lqr import GainSchedule, mass_parameters, ROOT
from .mymath import PID_control
from .mympc import MPCController

GRAVITY = 9.81
MAX_TORQUE_LEG_T = 40.0
MAX_TORQUE_LEG_W = 5.0


class LQRController:
    def __init__(self, mode='lqr', dt=0.01, model=None, variant='no-gas'):
        # K(L)离线拟合，运行时按各腿实测长度求值；不在1kHz循环做CARE。
        self.gains = GainSchedule(variant)
        self.A, self.B, self.K, _ = self.gains.evaluate(.278)
        self.schedule_enabled = True
        self.K_left, self.K_right = self.K.copy(), self.K.copy()
        self.gain_length = {'left': .278, 'right': .278}
        if model is None:
            filename = 'robot_no_gas_spring.xml' if variant == 'no-gas' else 'robot_gas_spring_damped.xml'
            model = mujoco.MjModel.from_xml_path(str(ROOT/'assets'/filename))
        self.masses = mass_parameters(model)
        self.support_mass = [self.masses['body_per_side_kg']+self.masses['leg_kg'][side]
                             for side in ('l','r')]
        self.mode = mode
        self.dt = dt
        self.time = 0.0
        self.next_time = {side:0. for side in ('left','right')}
        self.u = {side:np.zeros(2) for side in ('left','right')}
        # 兼容MPC仍用nominal A/B；本轮K(L)不声称时变MPC已验收。
        self.mpc = MPCController(self.A, self.B, N=30, dt=dt) if mode == 'lqr-mpc' else None
        if self.mpc is not None:
            self.mpc.u_min = np.array([-5., -20.])
            self.mpc.u_max = np.array([5., 20.])
        self.pid_l0_p = PID_control(2000, 0, 0, 0.278)
        self.pid_l0_s = PID_control(2000, 0, 0, 0.278)
        self.pid_roll = PID_control(1000, 0, 0, 0)
        self.pid_delta = PID_control(50, 0, 0, 0)
        self.pid_yaw = PID_control(10, 0, 1000, 0)
        self.force_trim = [0., 0.]
        self.pivot_trim = [0., 0.]

    def set_standing_reference(self, leg_L, leg_R, reference_torque):
        # 重力支撑修正来自此模型的静态站立点，不向被动轴施加力矩。
        for i, leg in enumerate((leg_L, leg_R)):
            sign = -1 if i == 0 else 1
            physical_J = sign*leg.vmc['J']
            F, Tp = np.linalg.solve(physical_J, reference_torque[[0, 4] if i == 0 else [2, 5]])
            self.force_trim[i] = F - self.support_mass[i]*GRAVITY
            self.pivot_trim[i] = Tp
            leg.target['l0'] = leg.vmc['L0']
            leg.target['theta'] = leg.state['theta']
            leg.target['phi'] = leg.state['phi']

    def compute_lqr(self, leg, side='left'):
        if self.time + 1e-10 < self.next_time[side]:
            return self.u[side]
        self.next_time[side] = self.time + self.dt
        length = leg.vmc['L0'] if self.schedule_enabled else .278
        K, used_length = self.gains.get_K(length)
        setattr(self, 'K_'+side, K)
        self.K = K  # 兼容旧调试接口；左右实际矩阵见K_left/K_right。
        self.gain_length[side] = used_length
        s = leg.state
        x = np.array([s['theta'], s['dtheta'], s['s'], s['dot_s'], s['phi'], s['dphi']])
        x_ref = np.array([leg.target[k] for k in ('theta', 'dtheta', 's', 'dot_s', 'phi', 'dphi')])
        err = x - x_ref
        # 原程序把负号放在电机输出；这里显式 u=-K*err，轮轴方向在启动文件写清。
        self.u[side] = -K @ err
        if self.mpc is not None:
            # 保留作者 lqr+mpc 分支的 0.2 叠加形式。未实现扰动估计时不伪造 d_hat。
            self.u[side] += 0.2*self.mpc.solve(err, np.zeros(2), np.zeros(6))
        return self.u[side]

    def control_left(self, leg, imu):
        T_w, T_p = self.compute_lqr(leg, 'left')
        F0_p = self.pid_l0_p.position_pid(leg.target['l0'], leg.vmc['L0'])
        dF_roll = self.pid_roll.position_pid(leg.target['roll'], imu['euler'][0]) - 80*imu['gyro'][0]
        dF_delta = self.pid_delta.position_pid(leg.target['d2theta'], leg.state['delta']) - 3*leg.state['ddelta']
        dF_yaw = self.pid_yaw.position_pid(leg.target['yaw'], imu['euler'][2]) - imu['gyro'][2]
        F_0 = self.support_mass[0]*GRAVITY + self.force_trim[0] + F0_p - 80*leg.vmc['L0_dot'] + dF_roll + leg.target.get('jump_force', 0.)
        T_p = T_p + self.pivot_trim[0] + dF_delta
        T_w = T_w - dF_yaw
        tau = leg.vmc['J'] @ np.array([F_0, T_p])
        tau = np.clip(tau, -MAX_TORQUE_LEG_T, MAX_TORQUE_LEG_T)
        tau_w = np.clip(T_w, -MAX_TORQUE_LEG_W, MAX_TORQUE_LEG_W)
        return tau, tau_w

    def control_right(self, leg, imu):
        T_w, T_p = self.compute_lqr(leg, 'right')
        F0_p = self.pid_l0_s.position_pid(leg.target['l0'], leg.vmc['L0'])
        dF_roll = self.pid_roll.position_pid(leg.target['roll'], imu['euler'][0]) - 80*imu['gyro'][0]
        dF_delta = self.pid_delta.position_pid(leg.target['d2theta'], leg.state['delta']) - 3*leg.state['ddelta']
        dF_yaw = self.pid_yaw.position_pid(leg.target['yaw'], imu['euler'][2]) - imu['gyro'][2]
        F_0 = self.support_mass[1]*GRAVITY + self.force_trim[1] + F0_p - 80*leg.vmc['L0_dot'] - dF_roll + leg.target.get('jump_force', 0.)
        T_p = T_p + self.pivot_trim[1] - dF_delta
        T_w = T_w + dF_yaw
        tau = leg.vmc['J'] @ np.array([F_0, T_p])
        tau = np.clip(tau, -MAX_TORQUE_LEG_T, MAX_TORQUE_LEG_T)
        tau_w = np.clip(T_w, -MAX_TORQUE_LEG_W, MAX_TORQUE_LEG_W)
        return tau, tau_w
