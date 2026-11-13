"""由作者 LQRController 适配：六状态、支撑 PID、左右 VMC 输出保持原结构。"""
import numpy as np
from .mymath import PID_control
from .mympc import MPCController

MASS_BODY = 16.8775740  # 当前 XML 的机身质量；整机质量为 21.054625 kg
GRAVITY = 9.81
MAX_TORQUE_LEG_T = 30.0
MAX_TORQUE_LEG_W = 5.0


class LQRController:
    def __init__(self, mode='lqr', dt=0.01):
        # 当前完整模型的六状态矩阵，不使用上一版 37 维反馈。
        # 状态顺序沿用作者：[theta, dtheta, s, dot_s, phi, dphi]。
        self.A = np.array([
            [0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
            [579.88547021, 0.00000000, 0.00000000, 0.00000000, 2.03882573, 0.00000000],
            [0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000],
            [-75.30447545, 0.00000000, 0.00000000, 0.00000000, -0.03637708, 0.00000000],
            [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000],
            [11.55713523, 0.00000000, 0.00000000, 0.00000000, 9.97483193, 0.00000000],
        ])

        self.B = np.array([
            [0.00000000, 0.00000000],
            [-158.75735182, 45.01293770],
            [0.00000000, 0.00000000],
            [22.60763474, -5.74689327],
            [0.00000000, 0.00000000],
            [-1.19345126, 5.18286734],
        ])

        self.K = np.array([
            [-62.93176700, -4.26160532, -21.92214721, -20.55268503, 45.29264391, 5.10136858],
            [15.34956794, 0.77649768, 4.40675184, 3.77937958, 169.69752929, 7.91124698],
        ])

        self.mode = mode
        self.dt = dt
        self.time = 0.0
        self.next_time = 0.0
        self.u = np.zeros(2)
        self.mpc = MPCController(self.A, self.B, N=30, dt=dt) if mode == 'lqr-mpc' else None
        if self.mpc is not None:
            self.mpc.u_min = np.array([-5., -20.])
            self.mpc.u_max = np.array([5., 20.])
        self.pid_l0_p = PID_control(2000, 0, 0, 0.278)
        self.pid_l0_s = PID_control(2000, 0, 0, 0.278)
        self.pid_roll = PID_control(1000, 0, 0, 0)
        self.pid_delta = PID_control(50, 0, 0, 0)
        self.pid_yaw = PID_control(5, 0, 0, 0)
        self.force_trim = [0., 0.]
        self.pivot_trim = [0., 0.]

    def set_standing_reference(self, leg_L, leg_R, reference_torque):
        # 重力支撑修正来自此模型的静态站立点，不向被动轴施加力矩。
        for i, leg in enumerate((leg_L, leg_R)):
            sign = -1 if i == 0 else 1
            physical_J = sign*leg.vmc['J']
            F, Tp = np.linalg.solve(physical_J, reference_torque[[0, 4] if i == 0 else [2, 5]])
            self.force_trim[i] = F - MASS_BODY*GRAVITY/2
            self.pivot_trim[i] = Tp
            leg.target['l0'] = leg.vmc['L0']
            leg.target['theta'] = leg.state['theta']
            leg.target['phi'] = leg.state['phi']

    def compute_lqr(self, leg):
        if self.time + 1e-10 < self.next_time:
            return self.u
        self.next_time = self.time + self.dt
        s = leg.state
        x = np.array([s['theta'], s['dtheta'], s['s'], s['dot_s'], s['phi'], s['dphi']])
        x_ref = np.array([leg.target[k] for k in ('theta', 'dtheta', 's', 'dot_s', 'phi', 'dphi')])
        err = x - x_ref
        # 原程序把负号放在电机输出；这里显式 u=-K*err，轮轴方向在启动文件写清。
        self.u = -self.K @ err
        if self.mpc is not None:
            # 保留作者 lqr+mpc 分支的 0.2 叠加形式。未实现扰动估计时不伪造 d_hat。
            self.u += 0.2*self.mpc.solve(err, np.zeros(2), np.zeros(6))
        return self.u

    def control_left(self, leg, imu):
        T_w, T_p = self.compute_lqr(leg)
        F0_p = self.pid_l0_p.position_pid(leg.target['l0'], leg.vmc['L0'])
        dF_roll = self.pid_roll.position_pid(leg.target['roll'], imu['euler'][0]) - 80*imu['gyro'][0]
        dF_delta = self.pid_delta.position_pid(leg.target['d2theta'], leg.state['delta']) - 3*leg.state['ddelta']
        dF_yaw = self.pid_yaw.position_pid(leg.target['yaw'], imu['euler'][2]) - imu['gyro'][2]
        F_0 = MASS_BODY/2*GRAVITY + self.force_trim[0] + F0_p - 80*leg.vmc['L0_dot'] + dF_roll + leg.target.get('jump_force', 0.)
        T_p = T_p + self.pivot_trim[0] + dF_delta
        T_w = T_w - dF_yaw
        tau = leg.vmc['J'] @ np.array([F_0, T_p])
        tau = np.clip(tau, -MAX_TORQUE_LEG_T, MAX_TORQUE_LEG_T)
        tau_w = np.clip(T_w, -MAX_TORQUE_LEG_W, MAX_TORQUE_LEG_W)
        return tau, tau_w

    def control_right(self, leg, imu):
        T_w, T_p = self.compute_lqr(leg)
        F0_p = self.pid_l0_s.position_pid(leg.target['l0'], leg.vmc['L0'])
        dF_roll = self.pid_roll.position_pid(leg.target['roll'], imu['euler'][0]) - 80*imu['gyro'][0]
        dF_delta = self.pid_delta.position_pid(leg.target['d2theta'], leg.state['delta']) - 3*leg.state['ddelta']
        dF_yaw = self.pid_yaw.position_pid(leg.target['yaw'], imu['euler'][2]) - imu['gyro'][2]
        F_0 = MASS_BODY/2*GRAVITY + self.force_trim[1] + F0_p - 80*leg.vmc['L0_dot'] - dF_roll + leg.target.get('jump_force', 0.)
        T_p = T_p + self.pivot_trim[1] - dF_delta
        T_w = T_w + dF_yaw
        tau = leg.vmc['J'] @ np.array([F_0, T_p])
        tau = np.clip(tau, -MAX_TORQUE_LEG_T, MAX_TORQUE_LEG_T)
        tau_w = np.clip(T_w, -MAX_TORQUE_LEG_W, MAX_TORQUE_LEG_W)
        return tau, tau_w
