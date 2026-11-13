"""跳跃只做一件事：按阶段给出 (腿长目标, 附加支撑力)。

下蹲 → 蹬地 → 实际离地收腿并维持 → 展腿 → 接触恢复。
参数统一在parameters.py/JUMP；不写qpos、qvel或基座外力。
"""
from .parameters import JUMP


class JumpControl:
    def __init__(self, **overrides):
        self.p = {**JUMP, **overrides}
        self.tuck_enabled = self.p['tuck_enabled']
        self.start = self.liftoff = None
        self.pending = False
        self.phase = 'idle'
        self.history, self.record = [], None
        self.base_z = self.base_vz = self.com_z = self.com_vz = self.clearance = 0.
        self.air_time = 0.
        self.touching = True
        self.previous_com_z = self.wheel_ids = None
        self.command_length = self.last_update = None

    @property
    def active(self):
        return self.pending or self.start is not None

    def request(self):
        if not self.active:
            self.pending = True

    def _phase(self, name, now):
        if self.phase != name:
            self.phase = name
            self.record['phases'].append(dict(time=now, phase=name))

    def update(self, now, home_length):
        """阶段给目标，腿长按m/s限速，避免下蹲瞬间把支撑目标拉成大负值。"""
        dt = .001 if self.last_update is None else max(0., now-self.last_update)
        self.last_update = now
        if self.command_length is None:
            self.command_length = home_length
        target, force = self._targets(now, home_length)
        change = self.p['length_rate']*dt
        self.command_length += min(change, max(-change, target-self.command_length))
        return self.command_length, force

    def _targets(self, now, home_length):
        p = self.p
        if self.pending:
            self.pending = False
            self.start, self.liftoff = now, None
            self.record = dict(start=now, start_height=self.base_z, max_height=self.base_z,
                               start_com_z=self.com_z, max_com_z=self.com_z,
                               max_wheel_clearance_m=self.clearance, airborne_s=0.,
                               max_upward_velocity=0., phases=[])
            self._phase('crouch', now)
        if self.start is None:
            return home_length, 0.
        elapsed = now-self.start
        if elapsed < p['crouch_time']:
            return p['crouch_length'], 0.

        # 必须实际离地才收腿；继续在地面时仍处于蹬地阶段。
        if self.liftoff is None and self.air_time >= p['liftoff_confirm'] and self.base_vz > p['liftoff_vz']:
            self.liftoff = now
            self.record['liftoff'] = now
            self._phase('flight_tuck' if self.tuck_enabled else 'flight_extend', now)
        if self.liftoff is None and elapsed < p['crouch_time']+p['thrust_time']:
            self._phase('thrust', now)
            return p['extend_length'], p['force']

        if self.phase == 'flight_tuck':
            held_long_enough = now-self.liftoff >= p['tuck_hold_time']
            near_landing = self.com_vz < p['landing_vz'] and self.clearance < p['landing_clearance']
            if not held_long_enough and not near_landing and not self.touching:
                return p['tuck_length'], 0.
            self._phase('flight_extend', now)
        if self.touching:
            self._phase('landing', now)
            if elapsed >= p['total_time']:
                self.record['no_liftoff'] = self.liftoff is None
                self._phase('idle', now)
                self.history.append(self.record)
                self.start = None
        return home_length, 0.

    def observe(self, model, data, dt):
        """只读观测和记录：整机质心高度与轮底间隙不能混淆。"""
        self.base_z, self.base_vz = float(data.qpos[2]), float(data.qvel[2])
        if not self.active:
            self.previous_com_z = None
            return
        if self.wheel_ids is None:
            self.wheel_ids = [model.body(s+'_wheel_Link').id for s in ('l','r')]
        self.com_z = float((model.body_mass*data.xipos[:,2]).sum()/model.body_mass.sum())
        self.com_vz = 0. if self.previous_com_z is None else (self.com_z-self.previous_com_z)/dt
        self.previous_com_z = self.com_z
        self.clearance = min(float(data.xpos[i,2])-.06 for i in self.wheel_ids)
        self.touching = data.ncon > 0
        self.air_time = 0. if self.touching else self.air_time+dt
        if self.start is not None:
            r = self.record
            r['max_height'] = max(r['max_height'], self.base_z)
            r['max_com_z'] = max(r['max_com_z'], self.com_z)
            r['max_wheel_clearance_m'] = max(r['max_wheel_clearance_m'], self.clearance)
            r['max_upward_velocity'] = max(r['max_upward_velocity'], self.base_vz)
            r['airborne_s'] += 0. if self.touching else dt
