"""Local equilibrium feedback with PPO torque corrections."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from mjlab.envs.mdp.actions.actions import JointEffortAction, JointEffortActionCfg
from mjlab.utils.lab_api.math import euler_xyz_from_quat, quat_apply_inverse


@dataclass(kw_only=True)
class BalanceCfg(JointEffortActionCfg):
    def build(self, env):
        return Balance(self, env)


class Balance(JointEffortAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        source = np.load(Path(__file__).with_name("balance_gain.npz"))
        model = env.sim.mj_model
        motor_ids = [next(i for i in range(model.nu)
                          if model.joint(model.actuator_trnid[i, 0]).name == f"robot/{name}")
                     for name in self.target_names]
        self.gain = torch.tensor(source["K"][motor_ids], device=self.device, dtype=torch.float32)
        self.reference = torch.tensor(source["qref"], device=self.device, dtype=torch.float32)
        self.trim = torch.tensor(source["uref"][motor_ids], device=self.device, dtype=torch.float32)
        self.indices = source["indices"].tolist()
        self.joint_adr = [int(model.joint(f"robot/{name}").dofadr[0]) for name in
                          ("l_wheel_Joint", "r_wheel_Joint")]
        self.error_integral = torch.zeros((self.num_envs, 2), device=self.device)

    def reset(self, env_ids=None):
        self.error_integral[env_ids if env_ids is not None else slice(None)] = 0.0
        return super().reset(env_ids)

    def apply_actions(self):
        env = self._env
        if env._sim_step_counter % env.cfg.decimation != 1:
            return
        data = env.sim.data
        cmd = env.command_manager.get_command("targets")
        qpos, qvel = data.qpos, data.qvel
        roll, pitch, _ = euler_xyz_from_quat(qpos[:, 3:7])
        roll = torch.atan2(roll.sin(), roll.cos())
        pitch = torch.atan2(pitch.sin(), pitch.cos())
        local_velocity = quat_apply_inverse(qpos[:, 3:7], qvel[:, :3])
        error_velocity = qvel.clone()
        error_velocity[:, :3] = local_velocity
        error_velocity[:, 0] += cmd[:, 0]
        error_velocity[:, 5] -= cmd[:, 2]
        for i, side in enumerate((1, -1)):
            error_velocity[:, self.joint_adr[i]] += side * cmd[:, 0] / 0.06
            error_velocity[:, self.joint_adr[i]] += 0.208 * cmd[:, 2] / 0.06
        self.error_integral += error_velocity[:, [0, 5]] * env.step_dt
        self.error_integral.clamp_(-0.25, 0.25)
        error_position = torch.stack((
            self.error_integral[:, 0], torch.zeros_like(roll), qpos[:, 2] - self.reference[2],
            roll, pitch, self.error_integral[:, 1],
        ), dim=1)
        state = torch.cat((error_position, qpos[:, 7:] - self.reference[7:], error_velocity), dim=1)
        effort = self.trim - state[:, self.indices] @ self.gain.T + self._processed_actions
        effort = effort.clamp(self._clip[:, :, 0], self._clip[:, :, 1])
        self._entity.set_joint_effort_target(effort, joint_ids=self.target_ids)
