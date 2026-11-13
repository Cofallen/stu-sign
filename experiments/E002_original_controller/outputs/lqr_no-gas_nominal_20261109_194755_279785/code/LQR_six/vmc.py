"""沿用作者 VMC/Leg 接口；仅把旧五杆几何换成完整并联模型的几何。

当前后支路不是原 chuan.xml 的五杆，不能只替换四个杆长。
这里从四处 connect 的 Jacobian 消去八个被动关节，仍只输出两腿电机。
"""
import mujoco
import numpy as np


class VMC:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.active = [int(model.joint(n).dofadr[0]) for n in
                       ('lf0_Joint', 'l20_Joint', 'rf0_Joint', 'r20_Joint')]
        wheels = [int(model.joint(n).dofadr[0]) for n in ('l_wheel_Joint', 'r_wheel_Joint')]
        self.passive = [i for i in range(6, model.nv) if i not in self.active+wheels]

    def update_left(self, leg, motor):
        self._update(leg, 'l', 0, -1)

    def update_right(self, leg, motor):
        self._update(leg, 'r', 2, +1)

    def _update(self, leg, side, column, output_sign):
        model, data = self.model, self.data
        # 1. 被动角速度 = -J_passive^+ J_active * 主动角速度。
        equality = np.asarray(data.efc_J).reshape(-1, model.nv)
        equality = equality[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
        coupling = np.zeros((model.nv, 4))
        coupling[self.active] = np.eye(4)
        coupling[self.passive] = -np.linalg.pinv(equality[:, self.passive], rcond=1e-5) @ equality[:, self.active]

        # 2. 髋轴到轮心的相对位置/速度；模型几何始终保留原完整闭链。
        hip = data.xanchor[model.joint(side+'f0_Joint').id]
        wheel = model.body(side+'_wheel_Link').id
        wheel_jac = np.zeros((3, model.nv)); rotation_jac = np.zeros_like(wheel_jac)
        hip_jac = np.zeros_like(wheel_jac)
        mujoco.mj_jacBody(model, data, wheel_jac, rotation_jac, wheel)
        mujoco.mj_jac(model, data, hip_jac, rotation_jac, hip, model.body('base_Link_del').id)
        relative = data.xpos[wheel] - hip
        relative_jac = wheel_jac - hip_jac
        length = np.linalg.norm(relative)
        length_jac = relative @ relative_jac / length
        theta_jac = (relative[2]*relative_jac[0] - relative[0]*relative_jac[2]) / (relative[0]**2+relative[2]**2)

        # 3. 虚功：电机力矩 = [dL/dq; dtheta/dq].T @ [支撑力, 摆腿力矩]。
        jacobian = np.vstack([length_jac, theta_jac]) @ coupling[:, column:column+2]
        base_rotation = data.xmat[model.body('base_Link_del').id].reshape(3, 3)
        relative_body = base_rotation.T @ relative
        theta_body = np.arctan2(-relative_body[0], -relative_body[2])
        knee_rotation = data.xmat[model.body(side+'f1_Link').id].reshape(3, 3)
        carrier = base_rotation.T @ knee_rotation

        leg.vmc['phi0'] = np.pi/2-theta_body if side == 'l' else np.pi/2+theta_body
        leg.vmc['L0'] = float(length)
        leg.vmc['L0_dot'] = float(length_jac @ data.qvel)
        leg.vmc['theta_dot'] = float(theta_jac @ data.qvel)
        leg.vmc['carrier_pitch'] = float(np.arctan2(-carrier[2, 0], carrier[0, 0]))
        # 保留作者启动循环的“左腿取负、右腿取正”输出习惯。
        leg.vmc['J'] = output_sign * jacobian.T
