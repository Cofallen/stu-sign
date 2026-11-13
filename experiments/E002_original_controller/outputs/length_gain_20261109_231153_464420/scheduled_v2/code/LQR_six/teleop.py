"""按住控制与动作目标；不直接写电机、基座外力或 qpos/qvel。

参数集中在 Teleop.__init__，函数接口/单位/坐标见根 README。
"""
import numpy as np
from .jump import JumpControl


class Teleop:
    def __init__(self, leg_L, leg_R, yaw_kp=5., speed_max=1.2, forward_sign=-1.,
                 yaw_rate_max=.2, accel=1.5, brake_accel=3.,
                 l0_min=.18, l0_max=.34, l0_rate=.12,
                 jump_crouch=.20, jump_extend=.34, jump_force=200.,
                 jump_crouch_time=.40, jump_thrust_time=.20, jump_total_time=1.60,
                 jump_tuck=.18, jump_tuck_enabled=True):
        self.legs = (leg_L, leg_R)
        self.l0_home = float(leg_L.target['l0'])
        self.l0 = self.l0_home
        self.speed_max, self.forward_sign = speed_max, forward_sign
        self.yaw_rate_max, self.yaw_kp = yaw_rate_max, yaw_kp
        self.accel, self.brake_accel = accel, brake_accel
        self.l0_min, self.l0_max, self.l0_rate = l0_min, l0_max, l0_rate
        self.jump = JumpControl(crouch_length=jump_crouch, extend_length=jump_extend,
                                force=jump_force, crouch_time=jump_crouch_time,
                                thrust_time=jump_thrust_time, total_time=jump_total_time,
                                tuck_length=jump_tuck, tuck_enabled=jump_tuck_enabled)
        self.speed = self.yaw_rate = self.dot_target = self.clock = self.last_yaw = 0.
        self.pressed = set()
        self.blocked_until_release = False
        self.hold_s = None
        self.was_turning = False
        self.message = 'ready'
        self.key_events = []

    def set_pressed(self, keys):
        """当前真实按住键；空集合表示松开/失焦，不是锁存命令。"""
        keys = set(keys) & set('wsadqe')
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
            measured = self.legs[0].state['dot_s']
            if self.jump.active:
                return
            if self.pressed & set('wsad') or abs(measured) > .05:
                self.message = 'Release movement keys and wait for stop before J'
                print('[teleop] '+self.message, flush=True)
                return
            self.jump.request()
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
        self.speed = self.forward_sign*self.speed_max*(('w' in held)-('s' in held))
        self.yaw_rate = self.yaw_rate_max*(('a' in held)-('d' in held))
        if not self.jump.active:
            self.l0 = float(np.clip(self.l0 + self.l0_rate*dt*(('e' in held)-('q' in held)),
                                    self.l0_min, self.l0_max))
        target_length, force = self.jump.update(self.clock, self.l0_home) if self.jump.active else (self.l0, 0.)
        measured = float(self.legs[0].state['dot_s'])
        turning = abs(self.yaw_rate) > 1e-9
        if turning and abs(self.speed) < .02:
            dot_target = measured
            self.hold_s = None
        else:
            if self.was_turning:
                self.dot_target = measured
            rate = self.brake_accel if self.speed == 0 or self.speed*self.dot_target < 0 else self.accel
            self.dot_target += float(np.clip(self.speed-self.dot_target, -rate*dt, rate*dt))
            dot_target = self.dot_target
        self.was_turning = turning
        idle = not turning and self.speed == 0 and not self.jump.active
        if idle and abs(dot_target) < 1e-8 and abs(measured) < .03:
            if self.hold_s is None:
                self.hold_s = float(self.legs[0].state['s'])
        else:
            self.hold_s = None
        for leg in self.legs:
            leg.target['s'] = self.hold_s if self.hold_s is not None else leg.state['s']
            leg.target['dot_s'] = measured if self.jump.active else dot_target
            leg.target['l0'] = target_length
            leg.target['jump_force'] = force
            leg.target['yaw'] = imu['euler'][2] + self.yaw_rate/self.yaw_kp
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
        return 'HOLD W/S move, A/D turn, Q/E length; RELEASE brakes/holds; J jump, X stop, R reset'
