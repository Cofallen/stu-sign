"""按住控制与动作目标；不直接写电机、基座外力或 qpos/qvel。

参数集中在 parameters.py；连续轴与J可叠加，Shift按住小陀螺。
"""
import numpy as np
from .jump import JumpControl


class Teleop:
    def __init__(self, leg_L, leg_R, **overrides):
        from .parameters import DRIVE, JUMP
        unknown = set(overrides)-set(DRIVE)
        if unknown:
            raise TypeError(f'未知驾驶参数{sorted(unknown)}；跳跃/PID请改parameters.py。')
        self.params = {**DRIVE, **overrides}
        self.legs = (leg_L, leg_R)
        self.l0_home = self.l0 = float(leg_L.target['l0'])
        self.jump = JumpControl(**JUMP)
        self.speed = self.yaw_rate = self.dot_target = self.clock = self.last_yaw = 0.
        self.spin_direction = self.params['spin_direction']
        self.pressed = set()
        self.blocked_until_release = False
        self.hold_s = None
        self.turn_stopping = False
        self.quiet_time = 0.
        self.message = 'ready'
        self.key_events = []

    def set_pressed(self, keys):
        """当前真实按住键；空集合表示松开/失焦，不是锁存命令。"""
        keys = set(keys) & (set('wsadqe') | {'shift'})
        if keys != self.pressed:
            self.key_events.append(dict(time=self.clock, pressed=''.join(sorted(keys))))
            print(f"[keys t={self.clock:.2f}] {''.join(sorted(keys)) or 'released'}", flush=True)
        self.pressed = keys
        if not keys:
            self.blocked_until_release = False

    def on_key(self, key):
        """单次动作 J/X/R；连续轴必须调用 set_pressed，不能用此接口锁存。"""
        ch = key.lower() if isinstance(key, str) else (chr(key).lower() if 32 <= key <= 126 else '')
        if ch == 'j':
            if self.jump.active:
                return
            self.jump.request(self.l0)
            self.message = 'jump requested'
        elif ch in ('x', 'r'):
            self.speed = self.yaw_rate = 0.
            self.blocked_until_release = True
            if ch == 'r':
                self.l0 = self.l0_home
            self.message = 'stop' if ch == 'x' else 'reset targets'
        else:
            return
        self.key_events.append(dict(time=self.clock, key=ch))
        print(f'[teleop t={self.clock:.2f}] {self.message}', flush=True)

    def update(self, imu, dt):
        """每个物理步：按住轴→斜坡/制动→Leg.target；松开腿长保持当前目标。"""
        self.clock += dt
        self.last_yaw = float(imu['euler'][2])
        held = set() if self.blocked_until_release else self.pressed
        p = self.params
        self.speed = p['forward_sign']*p['speed_max']*(('w' in held)-('s' in held))
        direction = ('a' in held)-('d' in held)
        if direction:
            self.spin_direction = float(direction)
        yaw_goal = self.spin_direction*p['spin_rate'] if 'shift' in held else direction*p['yaw_rate_max']
        rate = p['yaw_brake'] if yaw_goal == 0 or yaw_goal*self.yaw_rate < 0 else p['yaw_accel']
        self.yaw_rate += float(np.clip(yaw_goal-self.yaw_rate, -rate*dt, rate*dt))
        if not self.jump.active:
            self.l0 = float(np.clip(self.l0+p['l0_rate']*dt*(('e' in held)-('q' in held)),
                                    p['l0_min'], p['l0_max']))
        jumping = self.jump.active
        target_length, force = self.jump.update(self.clock, self.l0_home) if jumping else (self.l0, 0.)
        if jumping and not self.jump.active:
            self.message = 'jump complete'
        measured = float(self.legs[0].state['dot_s'])
        # 等实际偏航也停下来才恢复位置保持，避免旋转释放时猛刹。
        turning = abs(self.yaw_rate) > p['yaw_command_deadband'] or abs(imu['gyro'][2]) > p['yaw_stopped']
        if turning:
            self.turn_stopping = True
        elif abs(measured) < p['speed_stopped']:
            self.turn_stopping = False
        if turning and self.speed == 0:
            self.dot_target = measured
        else:
            rate = p['turn_brake_accel'] if self.turn_stopping else p['brake_accel']
            if self.speed != 0 and self.speed*self.dot_target >= 0:
                rate = p['accel']
            self.dot_target += float(np.clip(self.speed-self.dot_target, -rate*dt, rate*dt))
        idle = not turning and self.speed == 0 and not self.jump.active
        self.quiet_time = self.quiet_time+dt if idle and abs(measured)<p['speed_stopped'] else 0.
        if self.quiet_time >= p['stop_hold_delay']:
            if self.hold_s is None:
                self.hold_s = float(self.legs[0].state['s'])
        else:
            self.hold_s = None
        for leg in self.legs:
            leg.target['s'] = self.hold_s if self.hold_s is not None else leg.state['s']
            # 空中轮速不等于机身速度；仅暂停轮里程计速度误差反馈，W/S命令保留。
            airborne_control = self.jump.active
            leg.target['dot_s'] = measured if airborne_control else self.dot_target
            leg.target['l0'] = target_length
            leg.target['jump_force'] = force
            leg.target['yaw'] = imu['euler'][2]  # 显示/兼容；实际按yaw_rate闭环
            leg.target['yaw_rate'] = self.yaw_rate
        return self.status()

    def observe(self, model, data, dt):
        """采集跳跃接触/高度；状态机和记录集中于 jump.py。"""
        self.jump.observe(model, data, dt)

    def status(self):
        return dict(speed=self.speed, dot_target=self.dot_target, yaw_rate=self.yaw_rate,
                    l0=self.l0, yaw=self.last_yaw, message=self.message, pressed=''.join(sorted(self.pressed)),
                    jump_phase=self.jump.phase, jumps=self.jump.history, key_events=self.key_events,
                    current_jump=self.jump.record if self.jump.active else None)

    @property
    def jump_phase(self):
        return self.jump.phase

    @staticmethod
    def help_text():
        return 'HOLD W/S move, A/D turn, Shift spin, Q/E length; J can combine; X stop, R reset'
