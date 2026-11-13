"""键盘驾驶层：只改 Leg.target，不直接写电机；站立/平衡仍由原控制链负责。

按键（窗口需有焦点；MuJoCo 只转发按下事件，所以命令是锁存的）：
  W / S   前进速度 +0.1 / -0.1 m/s（±0.4，按 0.5 m/s² 平滑加减速）
  A / D   左/右转 0.2 rad/s（速率控制，松开后自动阻尼，不锁死绝对航向）
  Q / E   腿长 -0.02 / +0.02 m（0.22–0.30）
  X       停车并停止转向（保留当前航向）
  R       复位速度/转向/腿长

转向不用绝对航向保持：给作者 target['yaw'] 一个 yaw_rate/kp 的固定超前量，
得到 dF≈(yaw_rate-gyro) 的速率环。原地转向时速度目标跟随实测，避免轮子侧滑
引起的里程计误差与 6 状态反馈互拉；这组参数已在 E002 离线扫描中验证：
前进 0.3/0.4 m/s、原地 0.2 rad/s、0.3 m/s + 0.2 rad/s 同时进行 10–12 s 均未倒。
"""
import numpy as np


class Teleop:
    def __init__(self, leg_L, leg_R, yaw_kp=5.0, speed_step=.1, speed_max=.4,
                 yaw_rate_max=.2, accel=.5, l0_min=.22, l0_max=.30):
        self.legs = (leg_L, leg_R)
        self.l0_home = float(leg_L.target['l0'])
        self.l0 = self.l0_home
        self.speed = 0.
        self.yaw_rate = 0.
        self.dot_target = 0.
        self.was_turning = False
        self.yaw_kp = float(yaw_kp)
        self.speed_step, self.speed_max = speed_step, speed_max
        self.yaw_rate_max, self.accel = yaw_rate_max, accel
        self.l0_min, self.l0_max = l0_min, l0_max
        self.last_yaw = 0.
        self.message = 'ready'
        self.clock = 0.
        self.jump_start = None
        self.jump_pending = False
        self.jump_phase = 'idle'
        self.jump_history = []
        self.jump_record = None
        self.base_height = 0.

    def reset(self):
        self.speed = 0.
        self.yaw_rate = 0.
        self.l0 = self.l0_home
        self.message = 'reset'

    def on_key(self, key):
        """viewer 的 key_callback：key 是 GLFW 键码，只有按下事件。"""
        ch = chr(key).lower() if 32 <= key <= 126 else ''
        if ch == 'w':
            self.speed = min(self.speed_max, round(self.speed + self.speed_step, 3))
        elif ch == 's':
            self.speed = max(-self.speed_max, round(self.speed - self.speed_step, 3))
        elif ch == 'a':
            self.yaw_rate = +self.yaw_rate_max
        elif ch == 'd':
            self.yaw_rate = -self.yaw_rate_max
        elif ch == 'q':
            self.l0 = max(self.l0_min, round(self.l0 - .02, 3))
        elif ch == 'e':
            self.l0 = min(self.l0_max, round(self.l0 + .02, 3))
        elif ch == 'j':
            if self.jump_start is not None or self.jump_pending:
                return
            if abs(self.speed) > .02 or abs(self.yaw_rate) > 1e-9:
                self.message = 'Press X to stop before jumping'
                print('[teleop] '+self.message, flush=True)
                return
            self.jump_pending = True
            self.message = 'jump requested'
        elif ch == 'x':
            self.speed = 0.
            self.yaw_rate = 0.
            self.message = 'stop'
        elif ch == 'r':
            self.reset()
        else:
            return
        if ch not in 'xr':
            self.message = (f"v {self.speed:+.2f} m/s  w {self.yaw_rate:+.2f} rad/s  "
                            f"l0 {self.l0:.2f} m")
        print(f'[teleop] {self.message}', flush=True)

    def update(self, imu, dt):
        """每个物理步调用；只写两条腿的 target，返回当前状态字典。"""
        self.clock += dt
        self.last_yaw = float(imu['euler'][2])
        if self.jump_pending:
            self.jump_pending = False
            self.jump_start = self.clock
            self.jump_record = dict(start=self.clock, start_height=self.base_height, max_height=self.base_height,
                                    airborne_s=0., max_upward_velocity=0.)
        jump_force = 0.
        target_length = self.l0
        if self.jump_start is not None:
            elapsed = self.clock-self.jump_start
            if elapsed < .30:
                self.jump_phase = 'crouch'
                target_length = .24
            elif elapsed < .45:
                self.jump_phase = 'thrust'
                target_length = .30
                jump_force = 140.
            elif elapsed < 1.30:
                self.jump_phase = 'landing'
                target_length = self.l0_home
            else:
                self.jump_phase = 'idle'
                self.jump_history.append(self.jump_record)
                self.jump_start = None
                self.message = 'jump complete'
        driving = abs(self.speed) > .02
        turning = abs(self.yaw_rate) > 1e-9
        measured = float(self.legs[0].state['dot_s'])
        if turning and not driving:
            dot_target = measured  # 原地转向：不平白施加纵向速度修正
        else:
            if self.was_turning:
                self.dot_target = measured  # 转向结束后从实测速度无跳变地开始制动
            self.dot_target += float(np.clip(self.speed - self.dot_target,
                                             -self.accel * dt, self.accel * dt))
            dot_target = self.dot_target
        self.was_turning = turning
        for leg in self.legs:
            leg.target['s'] = leg.state['s']
            leg.target['dot_s'] = measured if self.jump_start is not None else dot_target
            leg.target['l0'] = target_length
            leg.target['jump_force'] = jump_force
            # dF = kp*(target-yaw) - gyro；令 target-yaw = yaw_rate/kp 得到速率环。
            leg.target['yaw'] = imu['euler'][2] + self.yaw_rate / self.yaw_kp
        return self.status()

    def status(self):
        return dict(speed=self.speed, dot_target=self.dot_target, yaw_rate=self.yaw_rate,
                    l0=self.l0, yaw=self.last_yaw, message=self.message, jump_phase=self.jump_phase,
                    jumps=self.jump_history, current_jump=self.jump_record if self.jump_start is not None else None)

    def observe(self, model, data, dt):
        # 仅记录起跳/腾空证据，不修改位置/速度或附加基座向上力。
        self.base_height = float(data.qpos[2])
        if self.jump_start is not None:
            self.jump_record['max_height'] = max(self.jump_record['max_height'], self.base_height)
            self.jump_record['max_upward_velocity'] = max(self.jump_record['max_upward_velocity'], float(data.qvel[2]))
            if data.ncon == 0:
                self.jump_record['airborne_s'] += dt

    @staticmethod
    def help_text():
        return 'W/S speed  A/D turn  Q/E height  J jump  X stop  R reset  (click window first)'
