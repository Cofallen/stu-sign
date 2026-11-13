"""纯电机跳跃状态机：下蹲→蹬地→实际腾空收腿→展腿→接触恢复。

只输出腿长/附加支撑目标，不写电机、基座外力、qpos 或 qvel。
收腿提高轮底间隙；没有额外外力时不能凭内部动作提高整机质心弹道。
"""


class JumpControl:
    def __init__(self, crouch_length=.20, extend_length=.34, force=200.,
                 crouch_time=.40, thrust_time=.20, total_time=1.60,
                 tuck_length=.18, tuck_min_time=.10, tuck_max_time=.22,
                 extend_com_vz=.45, tuck_enabled=True):
        self.crouch_length, self.extend_length, self.force = crouch_length, extend_length, force
        self.crouch_time, self.thrust_time, self.total_time = crouch_time, thrust_time, total_time
        self.tuck_length, self.tuck_min_time, self.tuck_max_time = tuck_length, tuck_min_time, tuck_max_time
        self.extend_com_vz, self.tuck_enabled = extend_com_vz, tuck_enabled
        self.start = None
        self.pending = False
        self.phase = 'idle'
        self.history = []
        self.record = None
        self.base_z = self.com_z = self.com_vz = self.base_vz = 0.
        self.clearance = 0.
        self.air_time = 0.
        self.touching = True
        self.liftoff = None
        self.previous_com_z = None
        self.wheel_ids = None

    @property
    def active(self):
        return self.start is not None or self.pending

    def request(self):
        if not self.active:
            self.pending = True

    def _phase(self, phase, now):
        if phase != self.phase:
            self.phase = phase
            self.record['phases'].append(dict(time=now,phase=phase))

    def update(self, now, home_length):
        if self.pending:
            self.pending = False
            self.start, self.liftoff, self.air_time = now, None, 0.
            self.record = dict(start=now,start_height=self.base_z,max_height=self.base_z,
                               start_com_z=self.com_z,max_com_z=self.com_z,
                               max_wheel_clearance_m=self.clearance,airborne_s=0.,max_upward_velocity=0.,phases=[])
            self._phase('crouch',now)
        if self.start is None:
            return home_length,0.
        elapsed = now-self.start
        if elapsed < self.crouch_time:
            return self.crouch_length,0.
        if self.liftoff is None and not self.touching and self.air_time >= .006 and self.base_vz > .05:
            self.liftoff = now
            self.record['liftoff'] = now
            self._phase('flight_tuck' if self.tuck_enabled else 'flight_extend',now)
        if self.liftoff is not None:
            age = now-self.liftoff
            if self.phase == 'flight_tuck' and (age >= self.tuck_max_time or
                    (age >= self.tuck_min_time and self.com_vz <= self.extend_com_vz)):
                self._phase('flight_extend',now)
            if self.touching and age > .03:
                self._phase('landing',now)
            if self.phase == 'flight_tuck':
                return self.tuck_length,0.
            # 腾空后不继续给额外支撑力；预落地展回站立腿长。
            if elapsed >= self.total_time and self.touching:
                self._phase('idle',now)
                self.history.append(self.record)
                self.start = None
            return home_length,0.
        if elapsed < self.crouch_time+self.thrust_time:
            self._phase('thrust',now)
            return self.extend_length,self.force
        self._phase('landing',now)  # 未检测到起跳的安全恢复，不能伪报飞行。
        if elapsed >= self.total_time:
            self.record['no_liftoff'] = True
            self._phase('idle',now)
            self.history.append(self.record)
            self.start = None
        return home_length,0.

    def observe(self, model, data, dt):
        self.base_z, self.base_vz = float(data.qpos[2]), float(data.qvel[2])
        if not self.active:
            self.previous_com_z = None
            return
        if self.wheel_ids is None:
            self.wheel_ids = [model.body(s+'_wheel_Link').id for s in ('l','r')]
        self.com_z = float((model.body_mass*data.xipos[:,2]).sum()/model.body_mass.sum())
        self.com_vz = 0. if self.previous_com_z is None else (self.com_z-self.previous_com_z)/dt
        self.previous_com_z = self.com_z
        self.clearance = min(float(data.xpos[b,2])-.06 for b in self.wheel_ids)
        self.touching = data.ncon > 0
        self.air_time = 0. if self.touching else self.air_time+dt
        if self.start is not None:
            r = self.record
            r['max_height'] = max(r['max_height'],self.base_z)
            r['max_com_z'] = max(r['max_com_z'],self.com_z)
            r['max_wheel_clearance_m'] = max(r['max_wheel_clearance_m'],self.clearance)
            r['max_upward_velocity'] = max(r['max_upward_velocity'],self.base_vz)
            if not self.touching:
                r['airborne_s'] += dt
