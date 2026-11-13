"""Compare the training and native deployment inputs and actions at matched states."""
from datetime import datetime
import json
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import mujoco
import numpy as np
import torch
from tensordict import TensorDict
from mjlab.envs import ManagerBasedRlEnv

from deploy import Actor, DEFAULT, Robot, ROOT
from run import make_runner
from task import make_config


def main():
    output = ROOT / "outputs" / datetime.now(ZoneInfo("Asia/Shanghai")).strftime("parity_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    env = ManagerBasedRlEnv(make_config(1, 42, evaluation=True), device="cuda:0")
    try:
        runner, _ = make_runner(env, SimpleNamespace(seed=42, device="cuda:0"))
        runner.load(str(DEFAULT), map_location="cuda:0")
        policy = runner.get_inference_policy(device="cuda:0")
        native, actor = Robot(), Actor(DEFAULT)
        results = []
        with torch.inference_mode():
            for angle in (0., .02, -.06):
                env.reset()
                native.reset()
                native.data.qpos[3:7] = [np.cos(angle / 2), 0, np.sin(angle / 2), 0]
                native.data.qvel[:6] = [.15, -.04, .02, .03, -.06, .18]
                mujoco.mj_forward(native.model, native.data)
                for name in ("qpos", "qvel"):
                    getattr(env.sim.data, name)[0] = torch.as_tensor(getattr(native.data, name),
                                                                    device=env.device, dtype=torch.float32)
                command = (.2, .28, -.15)
                env.command_manager.get_command("targets")[0] = torch.tensor(command, device=env.device)
                env.sim.forward()
                raw = env.observation_manager.compute_group("actor")
                expected = policy(TensorDict({"actor": raw}, batch_size=[1])).clamp(-1, 1).cpu()
                actual = torch.from_numpy(native.observation(command))[None]
                results.append({"pitch": angle, "obs": float((raw.cpu() - actual).abs().max()),
                                "action": float((expected - actor(actual)).abs().max())})
        print(results)
        passed = all(row["obs"] < 1e-5 and row["action"] < 1e-4 for row in results)
        (output / "results.json").write_text(json.dumps({"passed": passed, "cases": results}, indent=2))
        return 0 if passed else 2
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
