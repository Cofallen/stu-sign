import numpy as np
from .mymath import Discreteness, PID_control
from .parameters import TARGET, PID, CONTROL


class Leg:
    def __init__(self, dt):
        # -------- discreteness --------
        self.disc_L = Discreteness(dt)
        self.disc_dL = Discreteness(dt)
        self.disc_theta = Discreteness(dt)
        self.disc_dtheta = Discreteness(dt)

        # -------- state --------
        self.state = {
            "theta": 0.0,
            "dtheta": 0.0,
            "ddtheta": 0.0,
            "s": 0.0,
            "dot_s": 0.0,
            "phi": 0.0,
            "dphi": 0.0,
            "delta": 0.0,
        }

        # -------- target --------
        self.target = TARGET.copy()

        # -------- VMC --------
        self.vmc = {
            "phi0": 0.0,
            "phi1": 0.0,
            "phi2": 0.0,
            "phi3": 0.0,
            "phi4": 0.0,
            "L0": 0.0,
            "L0_dot": 0.0,
            "L0_ddot": 0.0,
            "J": np.zeros((2, 2)),
        }

        # -------- LQR --------
        self.LQR = {
            "K": np.zeros(12),
            "T_w": 0.0,
            "T_p": 0.0,
            "F_0": 0.0,
            "torque_leg": np.zeros(2),
            "torque_wheel": 0.0,
        }

        # -------- PID --------
        self.pid_F = PID_control(*PID['legacy_leg_force'])
        self.pid_roll = PID_control(*PID['legacy_leg_roll'])

        # -------- limit --------
        self.limit = {
            "T_max": CONTROL['leg_torque_limit'],
            "W_max": CONTROL['wheel_torque_limit'],
        }