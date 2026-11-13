"""Independent full-MJCF standing baseline, informed by Cofallen's LQR/VMC/MPC.

No source robot constants or code are copied. Feedback uses privileged simulation
state; this is not an onboard estimator or a reproduction of the six-state model.
"""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.linalg import cho_factor, cho_solve, solve_discrete_are
from scipy.optimize import least_squares, minimize

LOOPS = [(f'{side}_front_site{i}', f'{side}_rear_site{i}')
         for side in ('Left', 'Right') for i in (1, 2)]


def closure_errors(model, data):
    return np.array([np.linalg.norm(data.site_xpos[model.site(a).id]
                                    - data.site_xpos[model.site(b).id])
                     for a, b in LOOPS])


def euler(data):
    """World XYZ roll, pitch, yaw from wxyz root quaternion (radians)."""
    w, x, y, z = data.qpos[3:7]
    return np.array([np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                     np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def solve_equilibrium(model, height=.25):
    """Find an assembled stance, then force-balanced compliant-contact rest state.

    Geometry optimization is reset-only. Simulation never rewrites qpos/qvel.
    3-point absolute finite differences matter near zero passive reference angles.
    """
    d = mujoco.MjData(model)
    reference = model.qpos0.copy()
    joints = [j for j in range(1, model.njnt) if 'wheel' not in model.joint(j).name]
    qa = [int(model.jnt_qposadr[j]) for j in joints]
    wheels = [model.body(n).id for n in ('l_wheel_Link', 'r_wheel_Link')]
    sites = [(model.site(a).id, model.site(b).id) for a, b in LOOPS]
    radius = float(model.geom('l_wheel_Link_collision_0').size[0])

    def geometry(x):
        d.qpos[:] = reference
        d.qpos[2], d.qpos[qa] = x[0], x[1:]
        mujoco.mj_forward(model, d)
        return np.r_[np.concatenate([d.site_xpos[a]-d.site_xpos[b] for a, b in sites])*20,
                     (d.xpos[wheels, 0]-d.subtree_com[1, 0])*5,
                     (d.xpos[wheels, 2]-(radius-.0002))*5,
                     (x[0]-height)*5, (x[1:]-reference[qa])*.001]

    geo = least_squares(geometry, np.r_[height, reference[qa]],
                        xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=300)
    geometry(geo.x)
    target = d.qpos.copy()
    n = len(qa)

    def force_balance(x):
        d.qpos[:] = target
        d.qpos[2], d.qpos[qa] = x[0], x[1:n+1]
        pitch = x[n+1]
        d.qpos[3:7] = [np.cos(pitch/2), 0, np.sin(pitch/2), 0]
        d.qvel[:] = 0
        d.ctrl[:] = x[n+2:]
        mujoco.mj_forward(model, d)
        return np.r_[d.qacc*.01, (d.qpos[qa]-target[qa])*.01, pitch*.1]

    rest = least_squares(force_balance, np.r_[target[2], target[qa], 0., np.zeros(model.nu)],
                         jac='3-point', x_scale='jac', xtol=1e-12,
                         ftol=1e-12, gtol=1e-11, max_nfev=1000)
    force_balance(rest.x)
    info = dict(geometry_cost=float(geo.cost), balance_cost=float(rest.cost),
                geometry_success=bool(geo.success), balance_success=bool(rest.success),
                max_acceleration=float(np.max(np.abs(d.qacc))), contacts=int(d.ncon),
                closure_m=closure_errors(model, d).tolist(), height_m=float(d.qpos[2]),
                torque_nm=d.ctrl.tolist())
    within_limits = np.all(d.ctrl >= model.actuator_ctrlrange[:, 0]) and np.all(d.ctrl <= model.actuator_ctrlrange[:, 1])
    info['torques_within_limits'] = bool(within_limits)
    if not rest.success or info['max_acceleration'] > .001 or d.ncon < 2 or not within_limits:
        raise RuntimeError(f'Equilibrium rejected: {info}')
    return d.qpos.copy(), d.ctrl.copy(), info


@dataclass
class LinearDesign:
    qref: np.ndarray
    uref: np.ndarray
    indices: np.ndarray
    A: np.ndarray
    B: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    P: np.ndarray
    K: np.ndarray
    info: dict

    def error(self, model, data):
        dq = np.empty(model.nv)
        mujoco.mj_differentiatePos(model, dq, 1., self.qref, data.qpos)
        return np.r_[dq, data.qvel][self.indices]

    def save(self, path):
        np.savez(path, **{k: v for k, v in vars(self).items() if k != 'info'})


def design_feedback(model, qref, uref, decimation=10):
    d = mujoco.MjData(model)
    d.qpos[:], d.ctrl[:] = qref, uref
    mujoco.mj_forward(model, d)
    nv = model.nv
    full_A, full_B = np.zeros((2*nv, 2*nv)), np.zeros((2*nv, model.nu))
    mujoco.mjd_transitionFD(model, d, 1e-6, True, full_A, full_B, None, None)
    # These cyclic positions have no restoring objective. Their velocities remain.
    omitted = [1] + [int(model.joint(n).dofadr[0]) for n in
                     ('l_wheel_Joint', 'r_wheel_Joint')]
    idx = np.array([i for i in range(2*nv) if i not in omitted])
    A1, B1 = full_A[np.ix_(idx, idx)], full_B[idx]
    A = np.linalg.matrix_power(A1, decimation)
    B = sum(np.linalg.matrix_power(A1, i)@B1 for i in range(decimation))
    qdiag = np.r_[[1000, 0, 3000, 2000, 4000, 100],
                  [100 if 'wheel' not in model.joint(j).name else 0 for j in range(1, model.njnt)],
                  [10, 0, 30, 10, 20, 10],
                  [1 if 'wheel' not in model.joint(j).name else .01 for j in range(1, model.njnt)]]
    Q, R = np.diag(qdiag[idx]), np.eye(model.nu)*.1
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R+B.T@P@B, B.T@P@A)
    residual = float(np.max(np.abs(P-(Q+A.T@P@(A-B@K)))))
    radius = float(np.max(np.abs(np.linalg.eigvals(A-B@K))))
    info = dict(state_size=len(idx), omitted_position_indices=omitted,
                control_dt_s=decimation*model.opt.timestep,
                linear_closed_loop_spectral_radius=radius, dare_residual=residual,
                note='Local contact linearization; eigenvalues are not a nonlinear guarantee.')
    if radius >= 1 or residual > 1e-4:
        raise RuntimeError(f'Linear feedback rejected: {info}')
    return LinearDesign(qref, uref, idx, A, B, Q, R, P, K, info)


class StandingController:
    """LQR or box-constrained receding-horizon MPC with an LQR terminal cost."""
    def __init__(self, model, design, mode='lqr', horizon=15):
        self.design, self.mode, self.horizon = design, mode, horizon
        self.low, self.high = model.actuator_ctrlrange.T.copy()
        self.solves = self.constrained_solves = self.solver_failures = 0
        self.saturation_ticks = 0
        self.max_iterations = 0
        if mode != 'mpc':
            return
        A, B, Q, R = design.A, design.B, design.Q, design.R
        nx, nu = B.shape
        S, T = np.zeros((horizon*nx, nx)), np.zeros((horizon*nx, horizon*nu))
        for i in range(horizon):
            S[i*nx:(i+1)*nx] = np.linalg.matrix_power(A, i+1)
            for j in range(i+1):
                T[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(A, i-j)@B
        # Cost: x1..x(N-1) Q, xN P; all input deviations R. x0 is constant.
        QT = T.copy()
        QS = S.copy()
        for i in range(horizon):
            weight = design.P if i == horizon-1 else Q
            QT[i*nx:(i+1)*nx] = weight@T[i*nx:(i+1)*nx]
            QS[i*nx:(i+1)*nx] = weight@S[i*nx:(i+1)*nx]
        self.H = T.T@QT + np.kron(np.eye(horizon), R)
        self.H = (self.H+self.H.T)/2
        self.F = T.T@QS
        self.factor = cho_factor(self.H)
        self.bounds_low = np.tile(self.low-design.uref, horizon)
        self.bounds_high = np.tile(self.high-design.uref, horizon)
        self.previous = np.zeros(horizon*nu)

    def command(self, state):
        self.solves += 1
        design = self.design
        if self.mode == 'lqr':
            raw = design.uref-design.K@state
        else:
            f = self.F@state
            solution = -cho_solve(self.factor, f)
            if np.any(solution < self.bounds_low) or np.any(solution > self.bounds_high):
                self.constrained_solves += 1
                result = minimize(lambda u: (.5*u@self.H@u+f@u, self.H@u+f),
                                  np.clip(self.previous, self.bounds_low, self.bounds_high),
                                  jac=True, bounds=list(zip(self.bounds_low, self.bounds_high)),
                                  method='L-BFGS-B', options=dict(maxiter=300, ftol=1e-12, gtol=1e-7))
                self.max_iterations = max(self.max_iterations, int(result.nit))
                if not result.success:
                    self.solver_failures += 1
                    # Explicit bounded fallback, surfaced in acceptance metrics.
                    raw = design.uref-design.K@state
                    return np.clip(raw, self.low, self.high)
                solution = result.x
            self.last_solution = solution.copy()
            nu = len(design.uref)
            self.previous = np.r_[solution[nu:], solution[-nu:]]
            raw = design.uref+solution[:nu]
        self.saturation_ticks += int(np.any(raw <= self.low+1e-6) or np.any(raw >= self.high-1e-6))
        return np.clip(raw, self.low, self.high)
