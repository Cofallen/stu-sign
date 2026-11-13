"""Complete E002 linkage as a manager-based mjlab task."""
from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import torch
from mjlab.actuator import XmlActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg, mdp
from mjlab.envs.mdp.actions import JointEffortActionCfg
from mjlab.managers import ObservationGroupCfg, ObservationTermCfg, RewardTermCfg, TerminationTermCfg
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg

ROOT = Path(__file__).resolve().parent
XML = ROOT / "assets/robot_no_gas_spring.xml"
POSE = json.loads((ROOT / "standing_pose.json").read_text())["no-gas"]
DECIMATION = 10
FORWARD = -1.0


def native_model():
    return mujoco.MjModel.from_xml_path(str(XML))


def robot_spec():
    spec = mujoco.MjSpec.from_file(str(XML))
    # Scene compilation applies the solver options separately.
    defaults = mujoco.MjSpec().option
    spec.option.timestep = defaults.timestep
    spec.option.tolerance = defaults.tolerance
    spec.option.integrator = defaults.integrator
    return spec


def motor_map(model):
    motors = sorted(range(model.nu), key=lambda i: model.actuator_trnid[i, 0])
    return {
        model.joint(model.actuator_trnid[i, 0]).name: tuple(map(float, model.actuator_ctrlrange[i]))
        for i in motors
    }


@dataclass(kw_only=True)
class CommandCfg(CommandTermCfg):
    stage: str
    manual: bool

    def build(self, env):
        return Command(self, env)


class Command(CommandTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.targets = torch.zeros((self.num_envs, 3), device=self.device)

    @property
    def command(self):
        return self.targets

    def _resample_command(self, env_ids):
        speed = self.targets[env_ids, 0]
        length = self.targets[env_ids, 1]
        yaw = torch.zeros_like(speed)
        speed.zero_()
        length.fill_(0.28)
        if not self.cfg.manual:
            mode = torch.rand_like(speed)
            if self.cfg.stage in ("velocity", "length-static", "length-moving"):
                moving = mode >= 0.25
                if self.cfg.stage == "length-static":
                    moving &= mode < 0.65
                elif self.cfg.stage == "length-moving":
                    moving &= mode < 0.85
                speed.copy_(torch.empty_like(speed).uniform_(-0.4, 0.4) * moving)
            if self.cfg.stage == "velocity":
                speed[(mode >= 0.25) & (mode < 0.5)] = 0.0
                yaw = torch.empty_like(speed).uniform_(-0.4, 0.4) * (mode >= 0.25) * (mode < 0.5)
            if self.cfg.stage in ("length-static", "length-moving"):
                varying = mode >= 0.65
                length[varying] = torch.empty_like(length[varying]).uniform_(0.22, 0.30)
        self.targets[env_ids] = torch.stack((speed, length, yaw), dim=-1)

    def _update_metrics(self):
        pass

    def _update_command(self, env_ids=None):
        pass


def targets(env):
    return env.command_manager.get_command("targets")


def actor_observation(env, joints, hips):
    entity = env.scene["robot"]
    return torch.cat((
        targets(env),
        entity.data.projected_gravity_b,
        env.scene["robot/base_ang_vel"].data,
        entity.data.root_link_lin_vel_b,
        (entity.data.joint_pos - entity.data.default_joint_pos)[:, hips],
        entity.data.joint_vel[:, joints] * 0.1,
        env.action_manager.action.clamp(-1.0, 1.0),
    ), dim=-1)


def lengths(env, hip_ids, wheel_ids):
    data = env.sim.data
    return (data.xpos[:, wheel_ids] - data.xanchor[:, hip_ids]).norm(dim=-1)


def reward(env, hip_ids, wheel_ids):
    robot = env.scene["robot"].data
    cmd = targets(env)
    velocity_error = FORWARD * robot.root_link_lin_vel_b[:, 0] - cmd[:, 0]
    length_error = lengths(env, hip_ids, wheel_ids) - cmd[:, 1:2]
    gravity = robot.projected_gravity_b
    return (
        1.5 * torch.exp(-gravity[:, :2].square().sum(dim=1) / 0.04)
        + torch.exp(-velocity_error.square() / 0.06)
        + torch.exp(-length_error.square().mean(dim=1) / 0.0025)
        + torch.exp(-(robot.root_link_ang_vel_b[:, 2] - cmd[:, 2]).square() / 0.04)
        - 2.0 * robot.root_link_lin_vel_b[:, 1].square()
        - 0.1 * robot.root_link_ang_vel_b[:, :2].square().sum(dim=1)
        - 0.02 * env.action_manager.action.clamp(-1.0, 1.0).square().mean(dim=1)
        - 150.0 * fallen(env)
    )


def fallen(env):
    robot = env.scene["robot"].data
    return (
        (robot.root_link_pos_w[:, 2] < 0.12)
        | (robot.projected_gravity_b[:, 2] > -0.8253356)
        | (~torch.isfinite(env.sim.data.qpos).all(dim=1))
        | (~torch.isfinite(env.sim.data.qvel).all(dim=1))
    )


def make_config(num_envs, seed, stage="stand", *, evaluation=False, assisted=False):
    model = native_model()
    motors = motor_map(model)
    joint_names = [model.joint(i).name for i in range(1, model.njnt)]
    joint_pos = {name: POSE["qpos"][int(model.joint(name).qposadr[0])] for name in joint_names}
    indices = [joint_names.index(name) for name in motors]
    hips = [index for name, index in zip(motors, indices, strict=True) if "_wheel_" not in name]
    qpos = POSE["qpos"]
    robot = EntityCfg(
        spec_fn=robot_spec,
        init_state=EntityCfg.InitialStateCfg(pos=tuple(qpos[:3]), rot=tuple(qpos[3:7]), joint_pos=joint_pos),
        articulation=EntityArticulationInfoCfg(
            actuators=(XmlActuatorCfg(target_names_expr=tuple(motors), command_field="effort"),)
        ),
    )
    obs = ObservationGroupCfg(terms={
        "sensors": ObservationTermCfg(func=actor_observation, params={"joints": indices, "hips": hips})
    })
    action_cls = JointEffortActionCfg
    if assisted:
        from balance import BalanceCfg
        action_cls = BalanceCfg
    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(num_envs=num_envs, entities={"robot": robot}),
        sim=SimulationCfg(
            nconmax=96, njmax=256,
            mujoco=MujocoCfg(
                timestep=model.opt.timestep, integrator="implicitfast", solver="newton",
                iterations=int(model.opt.iterations), tolerance=model.opt.tolerance,
            ),
        ),
        decimation=DECIMATION,
        seed=seed,
        episode_length_s=30.0 if evaluation else 6.0,
        auto_reset=not evaluation,
        commands={"targets": CommandCfg(
            resampling_time_range=(60.0, 60.0) if evaluation else (2.0, 4.0),
            stage=stage, manual=evaluation,
        )},
        actions={"torque": action_cls(
            entity_name="robot", actuator_names=tuple(motors),
            scale={name: (0.2 if assisted else high) for name, (_, high) in motors.items()}, clip=motors,
        )},
        observations={"actor": obs, "critic": obs},
        rewards={"balance_tracking": RewardTermCfg(
            func=reward, weight=1.0,
            params={
                "hip_ids": [model.joint(f"{side}f0_Joint").id for side in ("l", "r")],
                "wheel_ids": [model.body(f"{side}_wheel_Link").id for side in ("l", "r")],
            },
        )},
        terminations={
            "fall": TerminationTermCfg(func=fallen),
            "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
        },
    )
