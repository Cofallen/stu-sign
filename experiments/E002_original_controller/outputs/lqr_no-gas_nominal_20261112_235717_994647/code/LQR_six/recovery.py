"""Sagittal recovery using joint PD, then standing LQR."""
import numpy as np

from .parameters import MONITOR


class Recovery:
    # Sagittal hip offsets for the rear-leg posture, in radians.
    swing_offsets = ((-2.343134, .220593), (.374694, .035202),
                     (.269909, .262576), (0., 0.))
    segment_seconds = .35502
    hip_limit_nm = 24.
    hips = ('left_front', 'left_back', 'right_front', 'right_back')
    common = np.array([1., 1., -1., -1.])
    differential = np.array([1., -1., -1., 1.])

    def __init__(self, model, actuators, standing_qpos, ground_ids):
        self.actuators = actuators
        ids = [actuators.actuator_ids[name] for name in self.hips]
        joints = model.actuator_trnid[ids, 0]
        self.position = model.jnt_qposadr[joints]
        self.velocity = model.jnt_dofadr[joints]
        self.standing = np.asarray(standing_qpos)[self.position]
        self.base = model.body('base_Link_del').id
        self.ground_ids = ground_ids
        self.wheel_bodies = [model.body(f'{side}_wheel_Link').id for side in ('l', 'r')]
        self.wheels = {model.geom(f'{side}_wheel_Link_collision_0').id for side in ('l', 'r')}
        self.phase = 'idle'
        self.since = 0.
        self.stable_since = None
        self.events = []
        self.attempts = 0

    def transition(self, phase, now, **details):
        self.phase, self.since = phase, now
        self.events.append(dict(phase=phase, time_s=now, attempt=self.attempts, **details))

    def update(self, data, tilt):
        now = data.time
        up = data.xmat[self.base].reshape(3, 3)[2]
        pitch = float(np.arctan2(-up[0], up[2]))
        elapsed = now - self.since
        if self.phase == 'idle' and tilt > MONITOR['fall_tilt']:
            # Wheel/terrain contact alone can occur during a recoverable landing.
            for contact in data.contact:
                support = self.ground_ids.intersection(contact.geom)
                if support and set(contact.geom) - support - self.wheels:
                    self.transition('settle', now, pitch_rad=pitch, trigger='body_contact')
                    break
            else:
                if up[2] < .35:
                    self.transition('settle', now, pitch_rad=pitch, trigger='large_tilt')
        elif self.phase == 'settle' and elapsed >= .7 - 1e-9:
            grounded = any(self.ground_ids.intersection(c.geom) for c in data.contact)
            base_forward = data.xmat[self.base].reshape(3, 3)[:, 0]
            # W drives along body -X, so rearward wheel centers project onto +X.
            wheel_x = (data.xpos[self.wheel_bodies] - data.xpos[self.base]) @ base_forward
            if tilt < .2 and up[2] > .97 and data.qpos[2] > .2:
                self.transition('balance', now)
            elif (grounded and np.all(wheel_x > .08) and .7 < pitch < 2.65
                  and abs(up[1]) < .2 and np.linalg.norm(data.qvel[3:6]) < .6):
                self.start_hips = data.qpos[self.position].copy()
                self.attempts += 1
                self.transition('swing', now, pitch_rad=pitch, wheel_body_x_m=wheel_x.tolist())
        elif self.phase == 'swing':
            if tilt < .14 and np.linalg.norm(data.qvel[3:6]) < .8 and data.qpos[2] > .2:
                self.transition('balance', now, pitch_rad=pitch)
            elif elapsed > 2.:
                self.transition('failed', now, reason='no_balance_entry')
        elif self.phase == 'balance':
            wheel_contacts = {int(g) for c in data.contact if self.ground_ids.intersection(c.geom) for g in c.geom}
            stable = (tilt < .05 and data.qpos[2] > .2 and self.wheels <= wheel_contacts
                      and np.linalg.norm(data.qvel[:3]) < .12)
            self.stable_since = (self.stable_since if self.stable_since is not None else now) if stable else None
            if self.stable_since is not None and now - self.stable_since >= .5:
                self.transition('idle', now, recovered=True)
                self.stable_since = None
            elif tilt > .9 or elapsed > 2.:
                self.stable_since = None
                self.transition('failed', now, reason='balance_timeout_or_refall')
        elif (self.phase == 'failed' and tilt < .2 and up[2] > .97
              and data.qpos[2] > .2 and np.linalg.norm(data.qvel[:6]) < .4):
            self.transition('balance', now, manually_upright=True)
        elif self.phase == 'failed' and elapsed >= .5:
            self.transition('settle', now, retry=True)
        return self.phase

    def drive(self, data):
        self.actuators.stop_all()
        if self.phase != 'swing':
            return
        progress = (data.time - self.since) / self.segment_seconds
        index = min(int(progress), len(self.swing_offsets) - 1)
        fraction = min(progress - index, 1.)
        blend = fraction * fraction * (3. - 2. * fraction)
        swing, split = self.swing_offsets[index]
        target = self.standing + swing * self.common + split * self.differential
        if index == 0:
            start = self.start_hips
        else:
            swing, split = self.swing_offsets[index - 1]
            start = self.standing + swing * self.common + split * self.differential
        desired = start + blend * (target - start)
        torque = 55. * (desired - data.qpos[self.position]) - 5. * data.qvel[self.velocity]
        limit = self.hip_limit_nm * min((data.time - self.since) / .2, 1.)
        for name, value in zip(self.hips, np.clip(torque, -limit, limit)):
            self.actuators.set_actuator(name, value)
