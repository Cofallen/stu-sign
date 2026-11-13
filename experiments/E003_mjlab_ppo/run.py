"""Standalone E003 checks, bounded PPO training, and checkpoint evaluation."""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parent
ACCEPTANCE = json.loads((ROOT / "acceptance.json").read_text())


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("native", "check", "train", "evaluate", "play"))
    parser.add_argument("--preset", choices=("stand", "velocity", "length-static", "length-moving"), default="stand")
    parser.add_argument("--baseline", action="store_true", help="Evaluate balance feedback with zero PPO correction")
    parser.add_argument("--assisted", action="store_true", help="Use learned torque corrections on equilibrium feedback")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--num-envs", type=int, default=256)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--scenario", choices=("stand", "velocity", "rotation", "length-static", "length-moving"), default="stand")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--physics-steps", type=int, default=ACCEPTANCE["short_train_max_physics_steps"])
    parser.add_argument("--wall-seconds", type=int, default=ACCEPTANCE["short_train_max_wall_seconds"])
    parser.add_argument("--video", action="store_true", help="Record the first evaluation episode")
    args = parser.parse_args()
    if args.num_envs < 1:
        parser.error("--num-envs must be positive")
    if not 0 < args.physics_steps <= 120_000_000:
        parser.error("--physics-steps must be between 1 and 120000000")
    if not 0 < args.wall_seconds <= ACCEPTANCE["short_train_max_wall_seconds"]:
        parser.error("--wall-seconds must be between 1 and 1800")
    if args.baseline and (not args.assisted or args.mode not in ("evaluate", "play")):
        parser.error("--baseline requires assisted evaluation or play")
    if args.mode in ("evaluate", "play") and not args.checkpoint and not args.baseline:
        parser.error("provide --checkpoint from an E003 run")
    if args.checkpoint and args.checkpoint.parent.joinpath("run.json").is_file():
        saved = json.loads(args.checkpoint.parent.joinpath("run.json").read_text())["settings"]
        if saved.get("assisted", False) != args.assisted:
            parser.error("checkpoint torque mode differs from --assisted")
    if args.mode == "train" and args.preset != "stand" and not args.checkpoint:
        parser.error("provide the previous stage checkpoint with --checkpoint")
    return args


def save_json(path, result):
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


def launch(args):
    tag = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    output = ROOT / "outputs" / f"{args.mode}_{tag}"
    output.mkdir(parents=True)
    print(f"Run directory: {output}", flush=True)
    env = dict(os.environ, E003_OUTPUT=str(output), PYTHONNOUSERSITE="1", PYTHONUNBUFFERED="1",
               MPLCONFIGDIR=str(output / "mpl"))
    command = [sys.executable, "-B", str(Path(__file__).resolve()), *sys.argv[1:]]
    save_json(output / "run.json", {
        "argv": command, "cwd": str(Path.cwd()), "started_at": tag,
        "settings": vars(args) | {"checkpoint": str(args.checkpoint.resolve()) if args.checkpoint else None},
        "versions": {name: version(name) for name in
                     ("mujoco", "mujoco-warp", "warp-lang", "torch", "mjlab", "rsl-rl-lib")},
        "file_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in [*ROOT.glob("*.py"), *ROOT.glob("*.json"), *ROOT.glob("*.npz"), *(ROOT / "assets").iterdir()]},
    })
    if args.mode in ("check", "train", "evaluate", "play"):
        env["E003_WARP_CACHE"] = str(ROOT / "outputs" / ".warp_cache")
    started = time.monotonic()
    with (output / "run.log").open("w") as log:
        if args.mode == "train":
            command = ["timeout", "--signal=TERM", str(args.wall_seconds), *command]
        status = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env).returncode
    save_json(output / "status.json", {"exit_code": status, "wall_seconds": time.monotonic() - started})
    print(f"Exit {status}. Log: {output / 'run.log'}")
    return status


def model_contract(model, prefix=""):
    import mujoco

    assert model.nu == 6 and model.neq == 4
    assert sum(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE) == 14
    assert sum(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE) == 1
    assert all(model.eq_type == mujoco.mjtEq.mjEQ_CONNECT)
    assert model.geom(prefix + "floor").contype == 2
    return {
        model.joint(model.actuator_trnid[i, 0]).name.removeprefix(prefix):
            [model.actuator(i).name.removeprefix(prefix), *model.actuator_ctrlrange[i]]
        for i in range(model.nu)
    }


def site_pairs(model):
    return model.eq_obj1id, model.eq_obj2id


def native_check(output):
    import mujoco
    import numpy as np
    from task import POSE, native_model

    model = native_model()
    mapping = model_contract(model)
    data = mujoco.MjData(model)
    data.qpos[:] = POSE["qpos"]
    data.ctrl[:] = POSE["ctrl"]
    left, right = site_pairs(model)
    maximum = 0.0
    for _ in range(100):
        mujoco.mj_step(model, data)
        maximum = max(maximum, float(np.linalg.norm(data.site_xpos[left] - data.site_xpos[right], axis=-1).max()))
    assert maximum < 0.0001 and not any(data.warning.number)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    save_json(output / "results.json", {
        "xml": str(Path(__file__).parent / "assets/robot_no_gas_spring.xml"),
        "motor_mapping": mapping, "steps": 100, "max_closure_error_m": maximum,
    })


def warp_check(args, output):
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from task import POSE, make_config, motor_map, native_model

    source = native_model()
    cfg = make_config(1, args.seed, evaluation=True)
    env = ManagerBasedRlEnv(cfg, device=args.device)
    try:
        model = env.sim.mj_model
        assert model_contract(model, "robot/") == model_contract(source)
        obs, _ = env.reset()
        assert obs["actor"].shape == (1, 28)
        assert torch.isfinite(obs["actor"]).all()
        expected = torch.tensor(POSE["qpos"], device=args.device)
        assert torch.allclose(env.sim.data.qpos[0], expected, atol=1e-6)
        motors = motor_map(source)
        ctrl_ids = [model.actuator("robot/" + model_contract(source)[name][0]).id for name in motors]
        limits = torch.tensor([high for _, high in motors.values()], device=args.device, dtype=torch.float32)
        actions = torch.tensor([[2.0, -2.0, 0.25, -0.25, 0.5, -0.5]], device=args.device)
        env.step(actions)
        assert torch.allclose(env.sim.data.ctrl[0, ctrl_ids], actions[0].clamp(-1, 1) * limits)
        obs, _ = env.reset()
        left, right = site_pairs(model)
        left, right = left.tolist(), right.tolist()
        support = torch.tensor(POSE["ctrl"], device=args.device)[None, ctrl_ids] / limits
        residual = 0.0
        for _ in range(10):
            obs, reward, terminated, timeout, _ = env.step(support)
            assert torch.isfinite(obs["actor"]).all() and torch.isfinite(reward).all()
            assert not terminated.item() and not timeout.item()
            sites = env.sim.data.site_xpos
            residual = max(residual, (sites[:, left] - sites[:, right]).norm(dim=-1).max().item())
        assert residual < 0.0001, residual
        env.episode_length_buf.fill_(env.max_episode_length - 1)
        _, _, terminated, timeout, _ = env.step(support)
        assert timeout.item() and not terminated.item()
        env.reset()
        env.sim.data.qpos[0, 2] = 0.08
        env.sim.forward()
        _, _, terminated, timeout, _ = env.step(torch.zeros_like(support))
        assert terminated.item() and not timeout.item()
        save_json(output / "results.json", {
            "device": args.device, "action_order": list(motors),
            "observation_size": 28, "supported_steps": 10, "max_closure_error_m": residual,
            "signed_torque_clipping": True, "terminated_and_timeout_separate": True,
        })
    finally:
        env.close()


def make_runner(env, args, output=None):
    from mjlab.rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from mjlab.rl.runner import MjlabOnPolicyRunner

    settings = RslRlOnPolicyRunnerCfg(logger="tensorboard", upload_model=False, clip_actions=1.0,
                                      seed=args.seed, save_interval=4)
    settings.actor.distribution_cfg["init_std"] = 0.04
    settings.actor.obs_normalization = True
    settings.critic.obs_normalization = True
    if output is not None:
        save_json(output / "ppo.json", asdict(settings))
    wrapper = RslRlVecEnvWrapper(env, clip_actions=settings.clip_actions)
    runner = MjlabOnPolicyRunner(wrapper, asdict(settings), str(output) if output else None, args.device)
    return runner, settings.num_steps_per_env


def train(args, output):
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from task import DECIMATION, POSE, make_config, motor_map, native_model

    steps_per_iteration = 24 * args.num_envs * DECIMATION
    iterations = args.physics_steps // steps_per_iteration
    if iterations < 1:
        raise ValueError("Requested environment count exceeds the short-run budget for one PPO update")
    env = ManagerBasedRlEnv(make_config(args.num_envs, args.seed, stage=args.preset, assisted=args.assisted), device=args.device)
    try:
        runner, rollout = make_runner(env, args, output)
        assert rollout == 24
        if args.checkpoint:
            runner.load(str(args.checkpoint.resolve()), map_location=args.device)
            runner.current_learning_iteration += 1
        else:
            model = native_model()
            torque_by_joint = {model.joint(model.actuator_trnid[i, 0]).name: torque
                               for i, torque in enumerate(POSE["ctrl"])}
            nominal = [0.0 if args.assisted else torque_by_joint[name] / high
                       for name, (_, high) in motor_map(model).items()]
            # Initialize the actor mean near static balance; deployment still maps actions directly to torque.
            with torch.no_grad():
                output_layer = runner.alg.get_policy().mlp[-1]
                output_layer.weight.zero_()
                output_layer.bias.copy_(output_layer.bias.new_tensor(nominal))
        first_iteration = runner.current_learning_iteration
        initial_steps = env.common_step_counter
        save_json(output / "budget.json", {"iterations": iterations, "steps_per_iteration": steps_per_iteration,
                                           "physics_steps_cap": iterations * steps_per_iteration})
        runner.learn(num_learning_iterations=iterations)
        assert all(torch.isfinite(p).all() for p in runner.alg.get_policy().parameters())
        save_json(output / "results.json", {"iterations": iterations, "physics_steps": iterations * steps_per_iteration,
                                            "policy_steps": (env.common_step_counter - initial_steps) * args.num_envs,
                                            "checkpoint": f"model_{first_iteration + iterations - 1}.pt"})
    finally:
        save_json(output / "progress.json", {"vector_steps": env.common_step_counter})
        env.close()


def scheduled_command(scenario, second):
    segment = min(int(second // 5), 5)
    speed = (0, -0.4, 0.4, 0, 0.3, -0.3)[segment] if scenario == "velocity" else 0.0
    length = 0.28
    if scenario.startswith("length"):
        length = (0.28, 0.22, 0.26, 0.30, 0.24, 0.28)[segment]
        if scenario == "length-moving":
            speed = 0.25
    yaw = (0, -0.3, 0.3, 0, 0.2, -0.2)[segment] if scenario == "rotation" else 0.0
    return speed, length, yaw


def score_episode(values, dt, seed, terminated, timeout, scenario, output):
    import numpy as np

    timeline = np.arange(len(values)) * dt
    stable = (timeline % 5) >= ACCEPTANCE["steady_state_ignore_seconds_after_change"]
    full = bool(timeout and not terminated and np.isfinite(values).all())
    drift = float(values[:, 0].max())
    tilt = float(values[-round(5 / dt):, 1:3].max())
    mae = values[stable, 3:7].mean(axis=0).tolist() if stable.any() else [None] * 4
    if scenario == "stand":
        passed = full and drift <= ACCEPTANCE["stand_drift_m"] and tilt <= ACCEPTANCE["stand_final_tilt_rad"]
    elif scenario in ("velocity", "rotation"):
        column, threshold = (0, "velocity_mae_m_s") if scenario == "velocity" else (3, "yaw_mae_rad_s")
        passed = full and mae[column] <= ACCEPTANCE[threshold]
    else:
        passed = full and max(mae[1:3]) <= ACCEPTANCE["leg_length_mae_m"]
    np.save(output / f"trace_{seed}.npy", values)
    return {"seed": seed, "seconds": len(values) * dt, "passed": bool(passed),
            "terminated": bool(terminated), "timeout": bool(timeout),
            "drift_m": drift if np.isfinite(drift) else None,
            "final_tilt_rad": tilt if np.isfinite(tilt) else None,
            "steady_mae": [v if v is not None and np.isfinite(v) else None for v in mae]}


def evaluate(args, output):
    import numpy as np
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from task import FORWARD, make_config

    seeds = ACCEPTANCE["held_out_seeds"] if args.mode == "evaluate" else [args.seed]
    env = ManagerBasedRlEnv(make_config(len(seeds), args.seed, evaluation=True, assisted=args.assisted),
                            device=args.device)
    try:
        runner, _ = make_runner(env, args)
        if not args.baseline:
            runner.load(str(args.checkpoint.resolve()), map_location=args.device)
        policy = runner.get_inference_policy(device=args.device)
        model = env.sim.mj_model
        hips = [model.joint(f"robot/{side}f0_Joint").id for side in ("l", "r")]
        wheels = [model.body(f"robot/{side}_wheel_Link").id for side in ("l", "r")]
        robot = env.scene["robot"].data
        command = env.command_manager.get_command("targets")
        renderer = None
        if args.video:
            import imageio.v2 as imageio
            import mujoco
            renderer = mujoco.Renderer(model, 360, 640)
            cpu_data = mujoco.MjData(model)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = 1.8, 120, -17
            writer = imageio.get_writer(output / "rollout.mp4", fps=25, macro_block_size=1)
        rows = []
        try:
            with torch.inference_mode():
                env.reset()
                pitch = torch.tensor([np.random.default_rng(seed).uniform(*ACCEPTANCE["initial_base_pitch_rad"])
                                      for seed in seeds], device=args.device, dtype=torch.float32)
                data = env.sim.data
                data.qpos[:, 3:7] = torch.stack(
                    ((pitch / 2).cos(), torch.zeros_like(pitch),
                     (pitch / 2).sin(), torch.zeros_like(pitch)), dim=1)
                env.sim.forward()
                initial_xy = data.qpos[:, :2].clone()
                finished = torch.zeros(len(seeds), dtype=torch.bool, device=args.device)
                end_step = torch.zeros(len(seeds), dtype=torch.long, device=args.device)
                failed = finished.clone()
                samples = []
                for step in range(env.max_episode_length):
                    cmd = scheduled_command(args.scenario, step * env.step_dt)
                    command[:] = torch.tensor(cmd, device=args.device)
                    action = (torch.zeros((len(seeds), 6), device=args.device) if args.baseline else
                              policy(runner.env.get_observations()).clamp(-1, 1))
                    _, _, terminated, timeout, _ = env.step(action)
                    w, x, y, z = robot.root_link_quat_w.unbind(dim=1)
                    roll = torch.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
                    pitch = torch.asin((2 * (w*y - z*x)).clamp(-1, 1))
                    length_error = ((data.xpos[:, wheels] - data.xanchor[:, hips]).norm(dim=2) - cmd[1]).abs()
                    forward = FORWARD * robot.root_link_lin_vel_b[:, 0]
                    yaw = robot.root_link_ang_vel_b[:, 2]
                    samples.append(torch.cat((
                        torch.stack(((data.qpos[:, :2] - initial_xy).norm(dim=1),
                                     roll.abs(), pitch.abs(), (forward - cmd[0]).abs()), dim=1),
                        length_error, (yaw - cmd[2]).abs()[:, None], forward[:, None], yaw[:, None],
                        data.ctrl.clone(), data.qpos[:, 2:3].clone(),
                    ), dim=1))
                    if renderer and not finished[0].item() and step % 4 == 0:
                        cpu_data.qpos[:] = data.qpos[0].cpu().numpy()
                        mujoco.mj_forward(model, cpu_data)
                        camera.lookat[:] = cpu_data.qpos[:3]
                        renderer.update_scene(cpu_data, camera=camera)
                        writer.append_data(renderer.render())
                    done = terminated | timeout
                    first_done = done & ~finished
                    end_step[first_done] = step + 1
                    failed[first_done] = terminated[first_done]
                    finished |= done
                    if finished.all().item():
                        break
                    done_ids = done.nonzero().flatten()
                    if len(done_ids):
                        env.reset(env_ids=done_ids)
                values = torch.stack(samples).cpu().numpy()
                for i, seed in enumerate(seeds):
                    count = end_step[i].item()
                    rows.append(score_episode(values[:count or len(values), i], env.step_dt, seed,
                                              failed[i].item(), bool(count and not failed[i].item()),
                                              args.scenario, output))
        finally:
            if renderer:
                writer.close()
                renderer.close()
        passes = sum(row["passed"] for row in rows)
        save_json(output / "results.json", {"scenario": args.scenario, "passes": passes,
                                             "accepted": args.mode == "evaluate" and passes >= ACCEPTANCE["required_passes"],
                                             "trace_columns": ["drift_m", "abs_roll_rad", "abs_pitch_rad",
                                                               "speed_abs_error_m_s", "left_length_abs_error_m",
                                                               "right_length_abs_error_m", "yaw_abs_error_rad_s",
                                                               "forward_m_s", "yaw_rad_s", *[model.actuator(i).name
                                                               for i in range(model.nu)], "base_height_m"],
                                             "episodes": rows})
    finally:
        env.close()


def main():
    args = parse_args()
    if "E003_OUTPUT" not in os.environ:
        return launch(args)
    output = Path(os.environ["E003_OUTPUT"])
    if args.mode == "native":
        native_check(output)
        return 0
    import warp
    warp.config.kernel_cache_dir = os.environ["E003_WARP_CACHE"]
    handlers = {"check": warp_check, "train": train, "evaluate": evaluate, "play": evaluate}
    handlers[args.mode](args, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
