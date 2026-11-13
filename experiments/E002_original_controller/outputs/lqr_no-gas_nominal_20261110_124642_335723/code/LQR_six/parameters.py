"""全部常用调参入口。单位见注释；不修改这里的模型真实质量/惯量。

PID沿用作者离散形式：P=kp*e，D=kd*(e-e_last)，不是kd*de/dt。
当前使用的ki都为0（原PID实现未把积分加入返回值）。
"""
DRIVE = dict(
    speed_max=3.0, forward_sign=-1.,           # m/s；W对应模型-X
    accel=1.5, brake_accel=3.,                # m/s²
    yaw_rate_max=3., spin_rate=2.,            # rad/s，普通A/D / 按住Shift
    yaw_accel=.6, yaw_brake=.6,               # rad/s²，逐渐起转/停转
    spin_direction=1.,                        # +1左转；A/D可更新方向
    yaw_stopped=.08, yaw_command_deadband=.01, speed_stopped=.03, stop_hold_delay=.5,       # rad/s、s；停稳才锁位置
    turn_brake_accel=.8,                      # m/s²，转完后的纵向制动
    l0_min=.16, l0_max=.36, l0_rate=.12,       # m、m/s；K仍截断至.18–.34
)
JUMP = dict(
    crouch_length=.20, extend_length=.34, force=300.,  # m、N/侧
    crouch_time=.40, thrust_time=.20, total_time=2.60, # s
    tuck_length=.18, tuck_hold_time=.2, length_rate=2.,              # m、s；实际离地后维持
    landing_clearance=.06, landing_vz=-.3,            # 低空下落时提前展腿
    liftoff_confirm=.006, liftoff_vz=.05,             # s、m/s
    tuck_enabled=True,
)
# (kp, ki, kd, 初始目标)；目标在每步position_pid调用中更新。
PID = dict(
    length_left=(80000., 0., 10000., .278),
    length_right=(80000., 0., 10000., .278),
    roll=(1000., 0., 0., 0.),
    delta=(50., 0., 0., 0.),
    yaw_rate=(1.5, 0., 0., 0.),                # 对角速度误差反馈，不再追角度
    legacy_leg_force=(4000., 1., 10000., 0.), # Leg历史接口；主控制不调用
    legacy_leg_roll=(1000., 0., 3000., 0.),    # Leg历史接口；主控制不调用
)
CONTROL = dict(
    feedback_dt=.01, nominal_length=.278,
    roll_damping=80., delta_damping=3., length_damping=80.,
    yaw_torque_limit=1.2,                     # Nm/轮差动，给平衡留力矩余量
    leg_torque_limit=30., wheel_torque_limit=5., # 最终仍受XML限幅
    mpc_mix=.2, mpc_horizon=30, mpc_wheel_limit=5., mpc_pivot_limit=20.,
    gravity=9.81,
)
TARGET = dict(
    theta=0., dtheta=0., s=0., dot_s=0., phi=0., dphi=0.,
    l0=.278, roll=0., yaw=0., yaw_rate=0., d2theta=0., jump_force=0.,
)
REFERENCE = dict(
    measured_pose=True,                       # reset实测theta/phi/L0作为平衡点
    theta_offset=0., phi_offset=0.,           # rad；基于实测姿态微调
)
# 评测记录跌倒，但不因跌倒退出；只在非有限数值/数值警告时停止。
MONITOR = dict(fall_height=.12, fall_tilt=.6, disable_balance_when_fallen=True)
# LQR代价和离线拟合参数。修改Q/R后必须重新生成系数，运行不会自动重拟合。
LQR_FIT = dict(q=(3000., 1., 500., 1., 30000., 1.), r=(1., 1.),
               order=3, center=.26, scale=.08, minimum=.18, maximum=.34)

# 已有gain_schedule.json标定时使用的支撑内环，供离线模型求导。
# 与运行PID分开列出：更改运行PID后需复验，不能声称旧K已重新标定。
LINEARIZATION = dict(length_kp=2000., length_damping=80.,
                     roll_kp=1000., roll_damping=80., delta_kp=50., delta_damping=3.,
                     epsilon=1e-5, projection_limit=.01, pinv_rcond=1e-5)
