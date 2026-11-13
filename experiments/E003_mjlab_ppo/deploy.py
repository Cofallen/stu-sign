"""Run the E003 direct-torque PPO actor with native MuJoCo (CPU)."""
import argparse
from contextlib import contextmanager, nullcontext
from copy import copy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
from threading import Event, Lock, Thread
import time
from zoneinfo import ZoneInfo

import mujoco
import numpy as np
import torch
from torch import nn

VIEWER_INTERVAL = 1.0 / 30.0

ROOT = Path(__file__).resolve().parent
DEFAULT = ROOT / "outputs/train_20261113_163126_398557/model_1478.pt"
ORDER = ("rf0_Joint", "r_wheel_Joint", "lf0_Joint", "l_wheel_Joint", "r20_Joint", "l20_Joint")
POSE = json.loads((ROOT / "standing_pose.json").read_text())["no-gas"]["qpos"]


class Actor(nn.Module):
    def __init__(self, checkpoint):
        super().__init__()
        manifest = checkpoint.parent / "run.json"
        if manifest.exists() and json.loads(manifest.read_text())["settings"].get("assisted", False):
            raise ValueError("Expected a direct-torque checkpoint, received a residual checkpoint")
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)["actor_state_dict"]
        self.register_buffer("mean", state["obs_normalizer._mean"])
        self.register_buffer("std", state["obs_normalizer._std"])
        self.mlp = nn.Sequential(nn.Linear(28, 128), nn.ELU(), nn.Linear(128, 128), nn.ELU(),
                                 nn.Linear(128, 128), nn.ELU(), nn.Linear(128, 6))
        self.mlp.load_state_dict({key[4:]: val for key, val in state.items() if key.startswith("mlp.")})
        self.eval()

    def forward(self, obs):
        return self.mlp((obs - self.mean) / (self.std + 0.01)).clamp(-1, 1)


class Robot:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/robot_no_gas_spring.xml"))
        self.data = mujoco.MjData(self.model)
        self.body = self.model.body("base_Link_del").id
        ids = [self.model.joint(name).id for name in ORDER]
        self.actuators = [int(np.flatnonzero(self.model.actuator_trnid[:, 0] == i)[0]) for i in ids]
        self.hip_qpos = [self.model.jnt_qposadr[i] for i in (ids[0], ids[2], ids[4], ids[5])]
        self.motor_qvel = self.model.jnt_dofadr[ids]
        self.limits = self.model.actuator_ctrlrange[self.actuators, 1]
        self.last_action = np.zeros(6)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = POSE
        self.last_action.fill(0)
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        rot = self.data.xmat[self.body].reshape(3, 3)
        gravity = -rot[2]
        local_velocity = rot.T @ self.data.qvel[:3]
        return np.concatenate((command, gravity, self.data.sensor("base_ang_vel").data, local_velocity,
                               self.data.qpos[self.hip_qpos] - np.asarray(POSE)[self.hip_qpos],
                               0.1 * self.data.qvel[self.motor_qvel], self.last_action)).astype(np.float32)

    def step(self, actor, command):
        obs = self.observation(command)
        self.last_action[:] = actor(torch.from_numpy(obs)[None])[0].numpy()
        self.data.ctrl[self.actuators] = self.last_action * self.limits
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        # Refresh kinematics and gyro for the next policy sample, as mjlab does.
        mujoco.mj_forward(self.model, self.data)
        rot = self.data.xmat[self.body].reshape(3, 3)
        return (-float(rot[:, 0] @ self.data.qvel[:3]), float(self.data.sensor("base_ang_vel").data[2]))


class KeyboardCommands:
    """Latched commands received through the native viewer key callback."""

    KEYS = {ord("W"): (0.4, 0.), ord("S"): (-0.4, 0.),
            ord("A"): (0., 0.3), ord("D"): (0., -0.3), ord("X"): (0., 0.)}

    def __init__(self):
        self.lock = Lock()
        self.motion = (0., 0.)
        self.reset_pending = False

    def on_key(self, key):
        with self.lock:
            if key in self.KEYS:
                self.motion = self.KEYS[key]
            elif key == ord("R"):
                self.motion = (0., 0.)
                self.reset_pending = True

    def poll(self):
        with self.lock:
            speed, yaw = self.motion
            reset = self.reset_pending
            self.reset_pending = False
        return (speed, 0.28, yaw), reset


@contextmanager
def live_view(robot, keyboard):
    """Render the latest snapshot without blocking the simulation clock."""
    from mujoco.viewer import launch_passive

    model = copy(robot.model)
    data = mujoco.MjData(model)
    data.qpos[:] = robot.data.qpos
    snapshot = (data.qpos.copy(), 0.)
    stop = Event()
    with launch_passive(model, data, key_callback=keyboard.on_key if keyboard else None) as viewer:
        with viewer.lock():
            viewer.opt.geomgroup[3] = 0
            viewer.cam.distance = 1.7
            viewer.cam.azimuth = 120
            viewer.cam.elevation = -18

        def refresh():
            while not stop.is_set() and viewer.is_running():
                qpos, sim_time = snapshot
                data.qpos[:] = qpos
                data.time = sim_time
                viewer.cam.lookat[:] = qpos[:3]
                viewer.sync(state_only=True)
                stop.wait(VIEWER_INTERVAL)

        def publish():
            nonlocal snapshot
            snapshot = (robot.data.qpos.copy(), robot.data.time)

        worker = Thread(target=refresh, daemon=True)
        worker.start()
        try:
            yield viewer, publish
        finally:
            stop.set()
            worker.join(timeout=0.5)


def main():
    from run import scheduled_command

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT)
    parser.add_argument("--scenario", choices=("keyboard", "stand", "velocity", "rotation", "demo"))
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--video", action="store_true", help="Save a native MuJoCo MP4")
    args = parser.parse_args()
    args.scenario = args.scenario or ("demo" if args.headless else "keyboard")
    interactive = args.scenario == "keyboard"
    if interactive and (args.headless or args.video):
        parser.error("keyboard requires a live viewer; use --scenario demo for recording")
    if args.seconds is None and not interactive:
        args.seconds = 30
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds < 0.01):
        parser.error("--seconds must be finite and at least 0.01")
    output = ROOT / "outputs" / ("native_deploy_" + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True)
    print(f"Native MuJoCo evidence: {output}", flush=True)
    metadata = {"argv": [sys.executable, *sys.argv], "mujoco": mujoco.__version__, "torch": torch.__version__,
                "checkpoint": str(args.checkpoint.resolve()), "scenario": args.scenario}
    rows = []
    keyboard = KeyboardCommands() if interactive else None
    status = 1
    loop_started = None
    renderer = writer = None
    try:
        metadata["checkpoint_sha256"] = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
        metadata["xml_sha256"] = hashlib.sha256((ROOT / "assets/robot_no_gas_spring.xml").read_bytes()).hexdigest()
        # A single small actor batch does not benefit from a CPU thread pool.
        torch.set_num_threads(1)
        actor = Actor(args.checkpoint)
        robot = Robot()
        viewer_context = nullcontext()
        visual = mujoco.MjvOption()
        visual.geomgroup[3] = 0
        if args.video:
            import imageio.v2 as imageio
            renderer = mujoco.Renderer(robot.model, 360, 640)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = 1.7, 120, -18
            writer = imageio.get_writer(output / "rollout.mp4", fps=25, macro_block_size=1)
        if not args.headless:
            viewer_context = live_view(robot, keyboard)
        with viewer_context as display, torch.inference_mode():
            viewer, publish = display if display else (None, None)
            if keyboard:
                print("W/S forward/back, A/D rotate, X stop, R reset. Commands hold until changed.", flush=True)
            from itertools import count
            steps = count() if args.seconds is None else range(round(args.seconds / 0.01))
            fallen = False
            loop_started = started = time.monotonic()
            for step in steps:
                if viewer and not viewer.is_running():
                    break
                second = step * 0.01
                if keyboard:
                    command, reset = keyboard.poll()
                    if reset:
                        robot.reset()
                        started = time.monotonic()
                        fallen = False
                    if fallen:
                        time.sleep(0.01)
                        continue
                elif args.scenario == "demo":
                    speed, yaw = [(0., 0.), (0.4, 0.), (-0.4, 0.), (0., 0.3), (0., -0.3), (0., 0.)][min(step // 500, 5)]
                    command = (speed, 0.28, yaw)
                else:
                    command = scheduled_command(args.scenario, second)
                speed, yaw = robot.step(actor, command)
                d = robot.data
                tilt = np.arccos(np.clip(d.xmat[robot.body, 8], -1., 1.))
                rows.append((d.time, *command, speed, yaw, *d.qpos[:3], tilt, *d.ctrl))
                if not np.isfinite(rows[-1]).all() or tilt > 0.6 or d.qpos[2] < 0.12 or any(d.warning.number):
                    print(f"Terminated at {d.time:.3f} s" + ("; press R to reset" if keyboard else ""), flush=True)
                    if keyboard:
                        fallen = True
                        publish()
                        continue
                    break
                if writer and step % 4 == 0:
                    camera.lookat[:] = d.qpos[:3]
                    renderer.update_scene(d, camera=camera, scene_option=visual)
                    writer.append_data(renderer.render())
                if viewer:
                    publish()
                    remaining = started + d.time - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
            metadata["loop_wall_seconds"] = time.monotonic() - loop_started
            status = 0 if interactive else (0 if len(rows) == round(args.seconds / 0.01) and np.isfinite(rows[-1]).all() and rows[-1][8] >= 0.12 and rows[-1][9] <= 0.6 and not any(robot.data.warning.number) else 2)
    finally:
        if writer:
            writer.close()
        if renderer:
            renderer.close()
        np.savetxt(output / "trace.csv", np.asarray(rows).reshape(-1, 16), delimiter=",",
                   header="t_s,cmd_forward,cmd_length,cmd_yaw,forward,yaw,x,y,z,tilt," + ",".join(
                       ("act_lf0", "act_lw", "act_rf0", "act_rw", "act_lf20", "act_rf20")), comments="")
        metadata.update(exit_code=status, seconds=len(rows) * 0.01)
        (output / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
