"""作者六状态接口；按当前模型轮径、轮轴方向和轮架转动补偿适配。"""
import numpy as np
from .mymath import Discreteness

RADIUS_WHEEL = 0.06
HIP_HEIGHT = 0.08725


class StateEstimator:
    def __init__(self, dt):
        self.dt = dt
        self.disc_s = Discreteness(dt)
        self.disc_theta_wl = Discreteness(dt)
        self.disc_theta_wr = Discreteness(dt)
        self.started = False

    def update_left(self, leg, imu, dt):
        leg.state['theta'] = np.pi/2 - leg.vmc['phi0'] + imu['euler'][1]
        leg.state['dtheta'] = leg.vmc['theta_dot']
        leg.state['phi'] = -imu['euler'][1]
        leg.state['dphi'] = -imu['gyro'][1]

    def update_right(self, leg, imu, dt):
        leg.state['theta'] = -np.pi/2 + leg.vmc['phi0'] + imu['euler'][1]
        leg.state['dtheta'] = leg.vmc['theta_dot']
        leg.state['phi'] = -imu['euler'][1]
        leg.state['dphi'] = -imu['gyro'][1]

    def update(self, leg_L, leg_R, motor, imu):
        pitch = imu['euler'][1]
        # 当前左轮正转/右轮反转为同一前进方向；加轮架相对机身的旋转。
        theta_wl = -(motor['left_wheel'] + leg_L.vmc['carrier_pitch'] + pitch)
        theta_wr = motor['right_wheel'] - leg_R.vmc['carrier_pitch'] - pitch
        if not self.started:
            self.disc_theta_wl.last_diff = theta_wl
            self.disc_theta_wr.last_diff = theta_wr
            self.started = True
        dtheta_wl = self.disc_theta_wl.Diff(theta_wl)
        dtheta_wr = self.disc_theta_wr.Diff(theta_wr)
        wheel_speed = -RADIUS_WHEEL*(dtheta_wl+dtheta_wr)/2
        # 轮心速度 + 腿几何补偿 - 髋轴偏置转动，得到机身 X 方向里程计。
        compensation = 0.0
        for leg in (leg_L, leg_R):
            theta = leg.state['theta']
            compensation += (leg.vmc['L0']*leg.state['dtheta']*np.cos(theta)
                             + leg.vmc['L0_dot']*np.sin(theta))/2
        dot_s = wheel_speed + compensation - HIP_HEIGHT*np.cos(pitch)*imu['gyro'][1]
        s = self.disc_s.Sum(dot_s)
        delta = leg_L.state['theta'] - leg_R.state['theta']
        ddelta = leg_L.state['dtheta'] - leg_R.state['dtheta']
        theta = (leg_L.state['theta']+leg_R.state['theta'])/2
        dtheta = (leg_L.state['dtheta']+leg_R.state['dtheta'])/2
        for leg in (leg_L, leg_R):
            leg.state.update(s=s, dot_s=dot_s, theta=theta, dtheta=dtheta,
                             delta=delta, ddelta=ddelta)
