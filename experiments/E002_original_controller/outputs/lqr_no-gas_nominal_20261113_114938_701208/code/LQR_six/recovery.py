"""Fall detection and staged recovery for the complete E002 linkage."""
import numpy as np

from .leg_rotate import LegRotate, angle_error, smooth_progress
from .parameters import RECOVERY, RECOVERY_DIRECT, RECOVERY_PREPARED, RECOVERY_BELLY


class Recovery:
    hips = ('left_front', 'left_back', 'right_front', 'right_back')
    wheels = ('left_wheel', 'right_wheel')
    lqr_phases = ('rising', 'transition')

    def __init__(self, model, actuators, legs, standing_qpos, ground_ids, prepare=False):
        self.actuators = actuators
        self.legs = legs
        self.rotate = LegRotate(legs)
        self.hip_ids = [actuators.actuator_ids[name] for name in self.hips]
        joints = model.actuator_trnid[self.hip_ids, 0]
        self.position = model.jnt_qposadr[joints]
        self.velocity = model.jnt_dofadr[joints]
        self.standing = np.asarray(standing_qpos)[self.position]
        self.home_lengths = np.array([leg.target['l0'] for leg in legs])
        self.base = model.body('base_Link_del').id
        self.body_collision = model.geom('base_Link_del_collision_0').id
        self.ground_ids = ground_ids
        self.wheel_geoms = {model.geom(f'{side}_wheel_Link_collision_0').id for side in ('l', 'r')}
        self.prepare = prepare
        self.phase = 'idle'
        self.since = 0.
        self.fall_since = None
        self.upright_since = None
        self.stable_since = None
        self.events = []
        self.attempts = 0
        self.prepared = False
        self.profile = RECOVERY_DIRECT

    def transition(self, phase, now, **details):
        self.phase, self.since = phase, now
        self.stable_since = None
        self.upright_since = None
        self.events.append(dict(phase=phase, time_s=now, attempt=self.attempts, **details))

    def begin_fall(self, now, **details):
        self.fall_since = None
        self.prepared = False
        self.transition('settle', now, **details)

    def held(self, condition, now, duration):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= duration - 1e-9

    def update(self, data, imu):
        p = RECOVERY
        now = data.time
        elapsed = now - self.since
        up = data.xmat[self.base].reshape(3, 3)[2]
        gyro = float(np.linalg.norm(imu['gyro']))
        tilt = float(np.max(np.abs(imu['euler'][:2])))
        body_pitch = float(np.arctan2(-up[0], up[2]))
        self.body_pitch = body_pitch
        gravity_theta = np.array([leg.vmc['theta_gravity'] for leg in self.legs])
        theta = np.array([leg.vmc['theta_body'] for leg in self.legs])
        lengths = np.array([leg.vmc['L0'] for leg in self.legs])
        contacts = {int(g) for c in data.contact if self.ground_ids.intersection(c.geom) for g in c.geom}
        nonwheel_contact = bool(contacts - self.ground_ids - self.wheel_geoms)
        body_contact = self.body_collision in contacts
        both_wheels = self.wheel_geoms <= contacts
        leg_asymmetry = abs(angle_error(theta[0], theta[1]))
        rear = bool(np.all((theta >= p['rear_angle_min']) & (theta <= p['rear_angle_max'])))
        prone = (body_contact and abs(up[1]) < .25
                 and (up[2] > .72 or p['prone_pitch_min'] <= body_pitch <= p['prone_pitch_max']))
        ready = prone and rear and lengths.min() >= p['ready_length'] and leg_asymmetry < .28 and gyro < .75
        details = dict(body_pitch_rad=body_pitch, theta_body_rad=theta.tolist(),
                       theta_gravity_rad=gravity_theta.tolist(), leg_lengths_m=lengths.tolist())
        if self.phase == 'idle':
            leg_overturn = np.max(np.abs(gravity_theta)) > p['fall_theta']
            imu_overturn = (up[2] < .35 or (tilt > p['fall_imu_tilt'] and nonwheel_contact)
                            or (body_contact and data.qpos[2] < .12))
            falling = imu_overturn or (leg_overturn and (nonwheel_contact or tilt > p['fall_imu_tilt']))
            self.fall_since = (now if self.fall_since is None else self.fall_since) if falling else None
            if self.fall_since is not None and now - self.fall_since >= p['fall_hold'] - 1e-9:
                self.begin_fall(now, **details)
        elif self.phase == 'settle':
            ready_long_enough = self.held(ready and elapsed >= p['settle_time'] - 1e-9,
                                          now, p['ready_hold'])
            if ready_long_enough:
                self.begin_retract(now, lengths, details)
            elif (not ready and self.prepare and not self.prepared and elapsed >= p['settle_time'] + p['ready_hold']
                  and nonwheel_contact and gyro < .55 and not self.upright(both_wheels, nonwheel_contact, up[2], tilt, gyro, data)):
                self.prepared = True
                self.rotate.begin(p['extend_length'], p['extend_time'], limit_nm=p['prepare_hip_nm'])
                self.transition('extend', now, **details)
            elif not ready:
                upright = self.upright(both_wheels, nonwheel_contact, up[2], tilt, gyro, data)
                self.upright_since = (now if self.upright_since is None else self.upright_since) if upright else None
                if self.upright_since is not None and now - self.upright_since >= p['stable_hold']:
                    self.transition('idle', now, manually_upright=True, **details)
            else:
                self.upright_since = None
        elif self.phase == 'extend':
            if not contacts and abs(data.qvel[2]) > 1.:
                self.transition('failed', now, reason='lost_support', **details)
            elif elapsed >= p['extend_time'] + p['prepare_settle'] - 1e-9:
                self.rotate.begin(p['extend_length'], p['rotate_time'], angle=p['rear_target'],
                                  limit_nm=p['prepare_hip_nm'])
                self.transition('rotate', now, **details)
        elif self.phase == 'rotate':
            if not contacts and abs(data.qvel[2]) > 1.:
                self.transition('failed', now, reason='lost_support', **details)
            elif elapsed >= p['rotate_time'] + p['prepare_settle'] - 1e-9 and ready:
                self.begin_retract(now, lengths, details)
            elif elapsed >= p['prepare_timeout']:
                self.transition('failed', now, reason='rear_prone_pose_not_reached', **details)
        elif self.phase == 'retract':
            if not contacts and abs(data.qvel[2]) > 1.:
                self.transition('failed', now, reason='lost_support', **details)
            elif (elapsed >= p['retract_time'] + p['prepare_settle'] - 1e-9
                  and lengths.max() <= self.retract_goal + .012 and prone and rear):
                q = data.qpos[self.position]
                nearest = self.standing + 2. * np.pi * np.round((q - self.standing) / (2. * np.pi))
                self.belly_lift = up[2] > .72
                if self.belly_lift:
                    self.profile = {**self.profile, **RECOVERY_BELLY}
                    self.rotate.begin(.29, .75, angle=0., limit_nm=24.)
                offsets = (np.array(self.profile['lift_waypoints']) if self.belly_lift
                           else np.array((*self.profile['hip_waypoints'], (0., 0.))))
                common = np.array([1., 1., -1., -1.])
                differential = np.array([1., -1., -1., 1.])
                self.hip_path = np.vstack((q, nearest + offsets[:, :1]*common + offsets[:, 1:]*differential))
                self.transition('swing', now, **details)
            elif elapsed >= p['retract_timeout']:
                self.transition('failed', now, reason='retract_incomplete_or_lost_prone_pose', **details)
        elif self.phase == 'swing':
            if (elapsed > (.30 if self.belly_lift else p['handoff_earliest']) and tilt < self.profile['handoff_tilt']
                    and abs(up[1]) < .10 and up[2] > (.85 if self.belly_lift else .9) and gyro < self.profile['handoff_gyro']
                    and data.qpos[2] > .177 and lengths.min() > .17 and both_wheels):
                self.entry_hips = data.ctrl[self.hip_ids].copy()
                self.entry_lengths = lengths
                self.transition('rising', now, **details)
            elif elapsed >= p['swing_timeout']:
                self.transition('failed', now, reason='lqr_entry_not_reached', **details)
        elif self.phase in self.lqr_phases:
            if tilt > .8 or data.qpos[2] < .11 or (elapsed > self.profile['lqr_blend_time'] and nonwheel_contact):
                self.transition('failed', now, reason='fell_during_lqr_handoff', **details)
            elif self.phase == 'rising' and elapsed >= p['rising_time']:
                self.transition('transition', now, **details)
            elif self.phase == 'transition':
                stable = self.upright(both_wheels, nonwheel_contact, up[2], tilt, gyro, data)
                upright = self.held(stable, now, p['stable_hold'])
                if elapsed >= p['stand_extend_time'] and upright:
                    self.transition('idle', now, recovered=True, **details)
                elif elapsed >= p['transition_timeout']:
                    self.transition('failed', now, reason='did_not_stabilize', **details)
        elif self.phase == 'failed' and elapsed >= .5:
            self.transition('settle', now, retry=True, **details)
        return self.phase

    def begin_retract(self, now, lengths, details):
        p = RECOVERY
        self.attempts += 1
        self.profile = RECOVERY_PREPARED if self.phase == 'rotate' else RECOVERY_DIRECT
        self.retract_goal = max(p['retract_length'], float(lengths.min()) - p['retract_amount'])
        self.rotate.begin(self.retract_goal, p['retract_time'], limit_nm=p['retract_hip_nm'])
        profile = 'prepared' if self.profile is RECOVERY_PREPARED else 'direct'
        self.transition('retract', now, target_length_m=self.retract_goal, profile=profile, **details)

    def upright(self, both_wheels, nonwheel_contact, up_z, tilt, gyro, data):
        return (both_wheels and not nonwheel_contact and up_z > .97 and tilt < .05
                and gyro < .3 and np.linalg.norm(data.qvel[:3]) < .08)

    def targets(self, now):
        """Leg length and torque blend used by LQR once the wheels support the robot."""
        p = RECOVERY
        elapsed = now - self.since
        if self.phase == 'rising':
            lengths = self.entry_lengths + smooth_progress(elapsed, p['rising_time']) * (
                self.profile['rising_length'] - self.entry_lengths)
            return lengths, smooth_progress(elapsed, self.profile['lqr_blend_time'])
        lengths = self.profile['rising_length'] + smooth_progress(elapsed, p['stand_extend_time']) * (
            self.home_lengths - self.profile['rising_length'])
        return lengths, 1.

    def drive(self, data, lqr_torques=None):
        self.actuators.stop_all()
        p = RECOVERY
        elapsed = data.time - self.since
        if self.phase in ('extend', 'rotate', 'retract'):
            hips = self.rotate.torques(elapsed)
        elif self.phase == 'swing' and self.belly_lift:
            hips = self.rotate.torques(elapsed, angle_offset=-self.body_pitch)
        elif self.phase == 'swing':
            position = elapsed / self.profile['hip_segment_time']
            index = min(int(position), len(self.hip_path) - 2)
            blend = smooth_progress(position - index, 1.)
            goal = self.hip_path[index] + blend * (self.hip_path[index+1] - self.hip_path[index])
            torque = 55. * (goal - data.qpos[self.position]) - 5. * data.qvel[self.velocity]
            ramp = min(elapsed / .2, 1.)
            limit = p['swing_hip_nm'] * ramp
            hips = np.clip(torque, -limit, limit)
        elif self.phase in self.lqr_phases:
            hip_torque, wheel_torque = lqr_torques
            _, blend = self.targets(data.time)
            hips = np.clip((1. - blend)*self.entry_hips + blend*hip_torque,
                           -p['swing_hip_nm'], p['swing_hip_nm'])
            wheel_blend = (smooth_progress(elapsed, self.profile['wheel_blend_time'])
                           if self.phase == 'rising' else 1.)
            wheel_limit = wheel_blend * p['handoff_wheel_nm']
            for name, value in zip(self.wheels, np.clip(wheel_blend*wheel_torque, -wheel_limit, wheel_limit)):
                self.actuators.set_actuator(name, value)
        else:
            return
        for name, value in zip(self.hips, hips):
            self.actuators.set_actuator(name, value)
