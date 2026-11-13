"""Flat-ground sagittal recovery using joint PD, then standing LQR."""
import numpy as np


class Recovery:
    # Each row is common hip offset, front/rear differential, and wheel torque.
    # These trajectories were selected on fallen resets of the no-gas model.
    profiles = {
        1: (.200, ((-1.681251, .223603, 4.463142), (-.390745, -.116951, -1.737727),
                   (.099481, .033574, -1.991395))),
    }
    hips = ('left_front', 'left_back', 'right_front', 'right_back')
    common = np.array([1., 1., -1., -1.])
    differential = np.array([1., -1., -1., 1.])

    def __init__(self, model, actuators, standing_qpos):
        self.actuators = actuators
        ids = [actuators.actuator_ids[name] for name in self.hips]
        joints = model.actuator_trnid[ids, 0]
        self.position = model.jnt_qposadr[joints]
        self.velocity = model.jnt_dofadr[joints]
        self.standing = np.asarray(standing_qpos)[self.position]
        self.base = model.body('base_Link_del').id
        self.floor = model.geom('floor').id
        self.wheels = {model.geom(f'{side}_wheel_Link_collision_0').id for side in ('l', 'r')}
        self.phase = 'idle'
        self.since = 0.
        self.stable_since = None
        self.events = []

    def transition(self, phase, now, **details):
        self.phase, self.since = phase, now
        self.events.append(dict(phase=phase, time_s=now, **details))

    def update(self, data, tilt):
        now = data.time
        up = data.xmat[self.base].reshape(3, 3)[2]
        pitch = float(np.arctan2(-up[0], up[2]))
        elapsed = now - self.since
        if self.phase == 'idle' and (tilt > .6 or up[2] < .8):
            self.transition('settle', now, pitch_rad=pitch)
        elif self.phase == 'settle' and elapsed >= .7 - 1e-9:
            self.direction = 1 if pitch > 0 else -1
            if tilt < .35 and up[2] > .9:
                self.transition('balance', now)
            elif (self.direction in self.profiles and .7 < abs(pitch) < 2.65
                  and abs(up[1]) < .2 and np.linalg.norm(data.qvel[3:6]) < .6
                  and any(self.floor in c.geom for c in data.contact)):
                self.transition('swing', now, pitch_rad=pitch)
            else:
                self.transition('failed', now, reason='unsupported_or_moving_pose')
        elif self.phase == 'swing':
            if abs(pitch) < .43 and abs(up[1]) < .2 and abs(data.qvel[4]) < 3.5 and data.qpos[2] > .105:
                self.transition('balance', now, pitch_rad=pitch)
            elif elapsed > 2.:
                self.transition('failed', now, reason='no_balance_entry')
        elif self.phase == 'balance':
            wheel_contacts = {int(g) for c in data.contact if self.floor in c.geom for g in c.geom}
            stable = (tilt < .05 and data.qpos[2] > .2 and self.wheels <= wheel_contacts
                      and np.linalg.norm(data.qvel[:3]) < .12)
            self.stable_since = (self.stable_since if self.stable_since is not None else now) if stable else None
            if self.stable_since is not None and now - self.stable_since >= .5:
                self.transition('idle', now, recovered=True)
                self.stable_since = None
            elif tilt > .9 or elapsed > 2.:
                self.transition('failed', now, reason='balance_timeout_or_refall')
        elif self.phase == 'failed' and tilt < .35 and up[2] > .9:
            self.transition('balance', now, manually_upright=True)
        return self.phase

    def drive(self, data):
        self.actuators.stop_all()
        if self.phase != 'swing':
            return
        period, profile = self.profiles[self.direction]
        index = min(int((data.time - self.since + 1e-9) / period), len(profile) - 1)
        swing, split, wheel = profile[index]
        target = self.standing + self.direction * swing * self.common + split * self.differential
        torque = 95. * (target - data.qpos[self.position]) - 4. * data.qvel[self.velocity]
        for name, value in zip(self.hips, torque):
            self.actuators.set_actuator(name, value)
        self.actuators.set_actuator('left_wheel', wheel)
        self.actuators.set_actuator('right_wheel', -wheel)
