"""SuperMimic-inspired online world-model and policy training in mjlab."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

import torch
from torch import nn
from mjlab.envs import ManagerBasedRlEnv

from deploy import Actor
from task import make_config


def network(inputs, outputs):
    return nn.Sequential(nn.Linear(inputs, 256), nn.ELU(), nn.Linear(256, 256), nn.ELU(),
                         nn.Linear(256, outputs))


def imagined_reward(state, goal, action):
    gravity, gyro, velocity = state[:, :3], state[:, 3:6], state[:, 6:9]
    return (1.5 * (-gravity[:, :2].square().sum(1) / 0.04).exp()
            + (-(-velocity[:, 0] - goal[:, 0]).square() / 0.06).exp()
            + (-(gyro[:, 2] - goal[:, 2]).square() / 0.04).exp()
            - 2 * velocity[:, 1].square()
            - 0.1 * gyro[:, :2].square().sum(1)
            - 0.02 * action.square().mean(1))


def run(args, output):
    torch.manual_seed(args.seed)
    device = args.device
    actor = Actor(args.initial_actor).to(device)
    actor.train()
    anchor = Actor(args.initial_actor).to(device)
    world = network(34, 19).to(device)
    optimizer = torch.optim.AdamW(world.parameters(), lr=1e-3)
    policy_optimizer = torch.optim.AdamW(actor.parameters(), lr=1e-5)
    env = ManagerBasedRlEnv(make_config(args.num_envs, args.seed, stage="velocity"), device=device)
    history = []
    replay = []
    try:
        obs, _ = env.reset()
        for epoch in range(args.iterations):
            samples = []
            with torch.no_grad():
                for _ in range(48):
                    current = obs["actor"].clone()
                    action = (actor(current) + 0.025 * torch.randn_like(current[:, -6:])).clamp(-1, 1)
                    obs, _, fall, timeout, _ = env.step(action)
                    samples.append((current, action, obs["actor"].clone(), ~(fall | timeout)))
            source, action, target, alive = [torch.stack(v) for v in zip(*samples, strict=True)]
            split = max(1, args.num_envs // 8)
            replay.append(tuple(v[:, split:].flatten(0, 1) for v in (source, action, target, alive)))
            replay = replay[-12:]
            x, u, y, ok = [torch.cat(v) for v in zip(*replay, strict=True)]
            x, u, y = x[ok], u[ok], y[ok]
            if not len(x):
                raise RuntimeError("No nonterminal transitions")
            for _ in range(120):
                ids = torch.randint(len(x), (512,), device=device)
                delta = (y[ids, 3:22] - x[ids, 3:22]) / actor.std[:, 3:22].clamp_min(0.05)
                prediction = world(torch.cat(((x[ids] - actor.mean) / (actor.std + 0.01), u[ids]), 1))
                loss = (prediction - delta).square().mean()
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            def transition(o, a):
                inputs = torch.cat(((o - actor.mean) / (actor.std + 0.01), a), dim=1)
                delta = world(inputs) * actor.std[:, 3:22].clamp_min(0.05)
                return torch.cat((o[:, :3], o[:, 3:22] + delta, a), dim=1)
            with torch.no_grad():
                validation = source[:, :split].flatten(0, 1)
                valid = alive[:, :split].flatten()
                predicted = transition(validation, action[:, :split].flatten(0, 1))
                error = (predicted[valid, 3:22] - target[:, :split].flatten(0, 1)[valid, 3:22]).abs().mean().item()
            world.requires_grad_(False)
            start = x[torch.randint(len(x), (512,), device=device)]
            imagined = start
            objective = 0.
            for step in range(5):
                a = actor(imagined)
                with torch.no_grad():
                    reference = anchor(imagined)
                imagined = transition(imagined, a)
                objective = objective + 0.97 ** step * (
                    imagined_reward(imagined[:, 3:22], imagined[:, :3], a)
                    - 10 * (a - reference).square().mean(1)).mean()
            policy_optimizer.zero_grad()
            (-objective).backward()
            grad = nn.utils.clip_grad_norm_(actor.parameters(), 1., error_if_nonfinite=True)
            policy_optimizer.step()
            world.requires_grad_(True)
            row = {"iteration": epoch + 1, "validation_one_step_mae": error,
                   "imagined_return": float(objective.detach()), "actor_gradient_norm": float(grad)}
            print(json.dumps(row, allow_nan=False), flush=True)
            history.append(row)
        state = torch.load(args.initial_actor, map_location="cpu", weights_only=True)["actor_state_dict"]
        state.update({f"mlp.{key}": val.detach().cpu() for key, val in actor.mlp.state_dict().items()})
        torch.save({"actor_state_dict": state}, output / "actor.pt")
        torch.save(world.state_dict(), output / "world_model.pt")
        (output / "metrics.json").write_text(json.dumps(history, indent=2))
    finally:
        env.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--initial-actor", type=Path, required=True)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--num-envs", type=int, default=128)
    p.add_argument("--iterations", type=int, default=8)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()
    if args.num_envs < 2 or args.iterations < 1:
        p.error("Use at least two environments and one iteration")
    output = Path(__file__).resolve().parent / "outputs" / datetime.now(ZoneInfo("Asia/Shanghai")).strftime("train_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    print(output, flush=True)
    metadata = vars(args) | {"initial_actor": str(args.initial_actor.resolve()), "argv": sys.argv,
                             "initial_sha256": hashlib.sha256(args.initial_actor.read_bytes()).hexdigest(),
                             "python": sys.executable, "torch": torch.__version__}
    code = 1
    try:
        run(args, output)
        code = 0
    finally:
        (output / "run.json").write_text(json.dumps({"settings": metadata, "exit_code": code}, indent=2))


if __name__ == "__main__":
    main()
