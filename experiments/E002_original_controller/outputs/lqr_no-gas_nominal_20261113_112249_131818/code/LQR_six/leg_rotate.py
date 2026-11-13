"""Slow length/angle PD for the two legs while the chassis is on the ground."""
import numpy as np


def angle_error(target, measured):
    """Shortest signed angle from measured to target, in radians."""
    error = target - measured
    return np.arctan2(np.sin(error), np.cos(error))


def smooth_progress(elapsed, duration):
    fraction = float(np.clip(elapsed / duration, 0., 1.))
    return fraction * fraction * (3. - 2. * fraction)


class LegRotate:
    def __init__(self, legs):
        self.legs = legs

    def begin(self, length, duration, angle=None, limit_nm=15.):
        self.duration = duration
        self.limit_nm = limit_nm
        self.start_lengths = np.array([leg.vmc['L0'] for leg in self.legs])
        self.start_angles = np.array([leg.vmc['theta_body'] for leg in self.legs])
        self.length_delta = length - self.start_lengths
        self.angle_delta = (np.zeros(2) if angle is None
                            else angle_error(angle, self.start_angles))

    def torques(self, elapsed):
        progress = smooth_progress(elapsed, self.duration)
        lengths = self.start_lengths + progress * self.length_delta
        angles = self.start_angles + progress * self.angle_delta
        limit = self.limit_nm * smooth_progress(elapsed, .18)
        outputs = []
        for leg, length, angle in zip(self.legs, lengths, angles):
            v = leg.vmc
            force = np.clip(800. * (length - v['L0']) - 45. * v['L0_dot'], -160., 160.)
            pivot = np.clip(44. * angle_error(angle, v['theta_body']) - 2.7 * v['theta_body_dot'],
                            -22., 22.)
            outputs.extend(np.clip(v['J_body'] @ np.array([force, pivot]), -limit, limit))
        return np.array(outputs)
