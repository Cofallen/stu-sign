# 这台电脑的 mjlab / IsaacLab / Isaac Sim 使用手册

> 用途：交接文档。读者（人或 AI）读完这一份，应当能在这台机器上找到东西、跑起仿真、训练、并知道哪儿会踩坑。
> 所有路径均已在本机核对（2026-11-08）。

---

## 0. 先看这一段（TL;DR）

**机器**：Lenovo Legion R7000 笔记本 / Ryzen 7 7840H / **RTX 4060 Laptop 8 GB** / 15 GB RAM /
Ubuntu 22.04.5（内核 6.8.0-138）/ NVIDIA 驱动 **580.178.04**。
双显卡：AMD Radeon 780M 核显 + NVIDIA 独显，`prime` 为 **on-demand**（这决定了很多坑，见第 5 节）。

> ⚠️ 在 **AI agent 的沙箱 shell** 里 `/dev` 是裁剪过的（只有 `null`/`zero` 等），没有
> `/dev/nvidia*`，所以 `nvidia-smi` 会报 "couldn't communicate with the NVIDIA driver"、
> `torch.cuda.is_available()` 返回 `False`。**这不是显卡坏了**：`/proc/driver/nvidia/gpus/0000:01:00.0`
> 里能看到 RTX 4060 Laptop。凡是需要 GPU 的命令，都要在**用户自己的终端**里跑。

**三套东西的关系**（最容易搞混的地方）：

| 组件 | 是什么 | 后端 | 本机版本 |
|---|---|---|---|
| **Isaac Sim** | NVIDIA 的 Omniverse Kit 仿真器（一个 app） | PhysX / RTX | 4.5.0（pip 包装进 conda env） |
| **IsaacLab** | 建在 Isaac Sim 之上的机器人学习框架（manager-based / direct 两套 API） | PhysX | 2.1.1（源码仓库，editable） |
| **mjlab** | **复刻 IsaacLab 的 API 设计，但后端换成 MuJoCo-Warp**，不依赖 Isaac Sim | MuJoCo-Warp | 1.6.0（源码仓库，editable） |

一句话：**mjlab 长得很像 IsaacLab，但两者运行环境完全独立、互不 import**。
`conda activate mjlab` 只能跑 mjlab，`conda activate isaac` 只能跑 IsaacLab/Isaac Sim。

**两个 conda 环境，别搞混**：

| 环境 | Python | 用途 | 位置 |
|---|---|---|---|
| `mjlab` | 3.12.13 | 跑 mjlab | `~/miniconda3/envs/mjlab` |
| `isaac` | 3.10.20 | 跑 Isaac Sim + IsaacLab | `~/miniconda3/envs/isaac` |

**建议阅读顺序**（AI 也一样）：
1. 第 1 节路径表 —— 建立"东西在哪"的地图；
2. 第 2.1 / 3.1 节 —— 搞清三者的边界，别把 mjlab 当 IsaacLab 用；
3. 第 5 节坑表 —— 在这台机器上，**90% 的失败是这 12 条里的某一条**；
4. 第 6 节速查 —— 直接能跑的命令；
5. 需要改代码时再看 2.4 / 3.4 的代码骨架。

---

## 1. 安装位置总表

| 内容 | 路径 |
|---|---|
| mjlab 源码（editable 安装） | `/home/cofallen/mjlab` |
| mjlab Python 环境 | `/home/cofallen/miniconda3/envs/mjlab` |
| mjlab 仓库自带的 uv venv（3.13，**日常没用它**） | `/home/cofallen/mjlab/.venv` |
| IsaacLab 源码（v2.1.1，editable） | `/home/cofallen/IsaacLab` |
| Isaac Sim 4.5 包 | `/home/cofallen/miniconda3/envs/isaac/lib/python3.10/site-packages/isaacsim` |
| Isaac Sim + IsaacLab 的 Python 环境 | `/home/cofallen/miniconda3/envs/isaac` |
| Isaac 资产本地镜像 | `/home/cofallen/isaac_assets`（`Isaac/`、`NVIDIA/`） |
| 资产同步脚本 | `/home/cofallen/isaac_env/mirror_isaac_assets.py` |
| 全局命令（已在 PATH） | `/home/cofallen/.local/bin/{isaacsim,isaaclab,isaac-python,isaac-assets-sync,mjlab}` |
| mjlab 脚手架工具 | `/home/cofallen/mjlab_tools`（symlink 到 `~/.local/bin/mjlab`） |
| Unitree 的 IsaacLab 扩展 | `/home/cofallen/Code/unitree_rl_lab` |
| 用户自己的 IsaacLab 工程 | `/home/cofallen/Code/IsaacTest/cartpole`（`Template-Cartpole-Direct-v0`） |
| 用户自己的 mjlab 工程 | `/home/cofallen/Code/mjlab_ws/{cart_test,slide_test,blank_test}` |
| 历史排障文档（很有用） | `/home/cofallen/Code/isaac/REPORT.md`、`ISAACLAB_简单用法.md`、`ISAAC_TUTORIALS.md` |
| Isaac Sim 资产根配置 | `~/Documents/Kit/shared/user.toml`（指向 `file:///home/cofallen/isaac_assets`） |
| 光标缓存 / 着色器缓存 | `~/.cache/ov`（约 460 MB，删了会重编 1~3 分钟） |
| GUI 版 Kit 日志 | `~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/4.5/kit_*.log` |
| headless 版 Kit 日志 | `<envs/isaac>/lib/python3.10/site-packages/omni/logs/Kit/Isaac-Sim/4.5/` |

---

## 2. mjlab

### 2.1 它是什么

mjlab = **Isaac Lab 的 manager-based API + MuJoCo Warp 后端**。
依赖很轻（`mujoco`、`mujoco-warp`、`warp-lang`、`torch`、`rsl-rl-lib`、`viser`），
**不需要 Isaac Sim**，也不需要 Omniverse，启动是秒级的。
仓库里有 `src/mjlab/utils/lab_api/`，就是从 IsaacLab fork 过来的 API 兼容层。

版本：**1.6.0**（git `0fb8a68 Prepare for 1.6.0`，detached HEAD，工作区干净）。
环境里的关键包：`mujoco 3.11.0`、`mujoco-warp 3.11.0`、`warp-lang 1.19.0`、
`torch 2.15.0+cu132`、`rsl-rl-lib 5.4.2`、`torchrunx 0.4.0`。

安装方式：`pip install -e` 进 conda 环境 `mjlab`（`pip list` 里 `mjlab 1.6.0 /home/cofallen/mjlab`）。
仓库里另有 `uv` 的 `.venv`（Python 3.13），但 `src/**/__pycache__` 是 `cpython-312`，
说明**实际跑的是 conda `mjlab` 环境（3.12）**，不是 `.venv`。

### 2.2 日常命令

```bash
conda activate mjlab

# 看有哪些任务（会打印 17 个 task id）
list-envs            # 等价 mjlab --tasks

# 训练（RSL-RL PPO）。--env.scene.num-envs 是覆盖环境数的标准写法
train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 4096 --agent.logger tensorboard

# 播放 / 评测：三个来源任选其一
play Mjlab-Cartpole-Swingup --checkpoint-file logs/rsl_rl/xxx/model_500.pt --viewer native
play Mjlab-Velocity-Flat-Unitree-G1 --wandb-run-path your-org/mjlab/run-id
play Mjlab-Cartpole-Swingup --agent zero --viewer viser    # 不打 checkpoint 的冒烟

# 其它入口
demo                 # 自带 demo，验证装好没
export-scene         # 导出场景（调试用）
viz-nan              # NaN 可视化调试
```

在 **mjlab 仓库目录里**也可以用 uv 写法（会走 `.venv`）：`uv run train ...`、`uv run demo`。

常用 flag（`src/mjlab/scripts/train.py` / `play.py`）：

| flag | 含义 |
|---|---|
| `--env.scene.num-envs N` | 并行环境数（4096 是 README 的推荐值；本机 8 GB 显存建议 512~2048 起） |
| `--agent.logger tensorboard\|wandb` | 日志后端 |
| `--gpu-ids "[0,1]"` | 多卡（torchrunx） |
| `--video` / `--video-interval` / `--video-length` | 录像 |
| `--enable-nan-guard` | NaN 守卫 |
| `--log-root logs/rsl_rl` | **默认值，相对当前工作目录** |
| `--registry-name org/motions/name` | 动作模仿任务的动作库 |
| play：`--viewer auto\|native\|viser` | `auto` = 有 DISPLAY 用 MuJoCo 原生窗，否则起 Viser（localhost:8080） |
| play：`--agent trained\|zero\|random` | 调试用零动作/随机动作 |

日志落在 `$PWD/logs/rsl_rl/<experiment_name>/<时间戳>/`（`experiment_name` 在 `rl_cfg.py` 里，如 `g1_velocity`）。
所以**在哪个目录启动，日志就在哪个目录下**——现成的例子：`~/Code/mjlab_ws/logs/rsl_rl/cart_test/`。

看曲线：`tensorboard --logdir logs/rsl_rl --port 6006`。

### 2.3 环境里现有的 17 个任务

```
Mjlab-Cartpole-Balance            Mjlab-Cartpole-Swingup
Mjlab-Velocity-Flat-Unitree-G1    Mjlab-Velocity-Rough-Unitree-G1
Mjlab-Velocity-Flat-Unitree-Go1   Mjlab-Velocity-Rough-Unitree-Go1
Mjlab-Tracking-Flat-Unitree-G1    Mjlab-Tracking-Flat-Unitree-G1-No-State-Estimation
Mjlab-Lift-Cube-Yam               Mjlab-Lift-Cube-Yam-Depth / -Rgb
Mjlab-Multi-Cube-Seg-Yam
# 下面这些来自用户自己 pip install -e 的工程（不是官方任务）
Mjlab-CartTest-Balance / -Swingup   # ← 包 cart_test
Mjlab-SlideTest-Slider              # ← 包 slide_test
Mjlab-Pend33-Task                   # ← 包 pend33
Mjlab-Model-Task                    # ← 包 model
```

任务发现机制：`mjlab.tasks` **entry point**。任何 pip 安装的包只要声明
`[project.entry-points."mjlab.tasks"] xxx = "xxx"`，`mjlab.tasks` 就会 import 它，
里面的 `register_mjlab_task(...)` 就生效了。

### 2.4 代码结构：一个任务 = 八个块

官方任务在 `/home/cofallen/mjlab/src/mjlab/tasks/`：

```
cartpole/  manipulation/  tracking/  velocity/
        └ config/<robot>/env_cfgs.py   ← 环境定义
                        /rl_cfg.py     ← PPO / 网络超参
                        /__init__.py   ← register_mjlab_task
```

注册一个任务（`tasks/velocity/config/g1/__init__.py`，原文）：

```python
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
from .env_cfgs import unitree_g1_flat_env_cfg, unitree_g1_rough_env_cfg
from .rl_cfg import unitree_g1_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Unitree-G1",
  env_cfg=unitree_g1_flat_env_cfg(),
  play_env_cfg=unitree_g1_flat_env_cfg(play=True),   # 播放时关掉观测噪声/加长 episode
  rl_cfg=unitree_g1_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
```

RL 超参（`rl_cfg.py`，原文节选）：`hidden_dims=(512,256,128)`、`activation="elu"`、
PPO `clip=0.2 / entropy=0.01 / lr=1e-3 (adaptive) / 5 epochs / 4 minibatches`、
`num_steps_per_env=24`、`max_iterations=30000`、`save_interval=50`。

**一个完整的任务实现长什么样**（用户模板 `~/Code/mjlab_ws/cart_test/.../env_cfgs.py`，296 行，
八个块的注释是模板自带的）：

```
0. Names        XML 里 joint/motor/body 的名字
1. Entity       MJCF + actuator + 初始状态
2. Observations ObservationGroupCfg（actor / critic 两组）
3. Actions      JointEffortActionCfg / JointPositionActionCfg ...
4. Rewards      RewardTermCfg（每个函数返回 [num_envs]）
5. Terminations TerminationTermCfg
6. Events       reset 随机化
7. Scene + Sim  terrain / num_envs / MujocoCfg / viewer / decimation
```

关键代码（同文件）：

```python
def _entity_cfg(swing_up: bool = False) -> EntityCfg:
  return EntityCfg(
    spec_fn=_get_spec,                                   # mujoco.MjSpec.from_file(robot.xml)
    articulation=EntityArticulationInfoCfg(
      actuators=(XmlActuatorCfg(target_names_expr=("slider",)),),   # 按 joint 名匹配！
    ),
    init_state=EntityCfg.InitialStateCfg(
      pos=(0.0, 0.0, 0.0),
      joint_pos={"slider": 0.0, "hinge_1": math.pi if swing_up else 0.0},
      joint_vel={".*": 0.0},
    ),
  )

def _env_cfg(swing_up=False, play=False) -> ManagerBasedRlEnvCfg:
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(terrain=TerrainEntityCfg(terrain_type="plane"),
                   entities={_ENTITY_NAME: _entity_cfg(swing_up)},
                   num_envs=1, env_spacing=4.0),
    observations=_observations(), actions=_actions(),
    rewards=_rewards(), terminations=_terminations(), events=_events(swing_up),
    viewer=ViewerConfig(origin_type=ViewerConfig.OriginType.ASSET_BODY,
                        entity_name=_ENTITY_NAME, body_name="cart",
                        distance=4.0, elevation=-15.0),
    sim=SimulationCfg(mujoco=MujocoCfg(timestep=0.01, disableflags=("contact",))),
    decimation=5,            # 0.01*5 = 0.05 s → 20 Hz
    episode_length_s=50.0,
  )
  if play:
    cfg.episode_length_s = 1e10
    cfg.observations["actor"].enable_corruption = False
  return cfg
```

观测/奖励函数都是普通函数，签名 `(env, asset_cfg=...) -> torch.Tensor`，
用 `env.scene[cfg.name].data.joint_pos[:, cfg.joint_ids]` 这类批量化张量：

```python
def pole_angle_cos_sin(env, asset_cfg=_POLE) -> torch.Tensor:
  angle = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids]
  return torch.cat([torch.cos(angle), torch.sin(angle)], dim=-1)   # [num_envs, 2]
```

⚠️ 命名规则（模板 README 反复强调）：代码里一律用 **`<joint name>`**，
不要用 `<motor name>`（mjlab 按 actuator 的 `joint=` 匹配），`body_names=` 只给 sensor/camera 用。

### 2.5 用 `mjlab-tools` 新建自己的任务

`~/mjlab_tools/mjlab`（v0.7.0）是一个**纯 bash** 的脚手架，模仿 `isaaclab.sh --new`：

```bash
mjlab --help
mjlab --new                                   # 交互式
mjlab --new --template cartpole --name my_robot --install
mjlab --new --template velocity --name my_go2 --path ~/Code/my_go2 --git
mjlab --templates        # cartpole / slider / velocity / blank
mjlab --tasks            # 当前环境有哪些 task id
mjlab --info             # python / torch / CUDA / mjlab / warp 版本
```

生成的项目结构（`cart_test` 就是产物）：

```
my_robot/
├── pyproject.toml      # dependencies=["mjlab"] + [project.entry-points."mjlab.tasks"]
├── README.md / DEPLOY.md
└── src/my_robot/
    ├── __init__.py
    └── tasks/my_task/{__init__.py, env_cfgs.py, rl_cfg.py, robot.xml}
```

然后 `pip install -e .`（在 `conda activate mjlab` 下），`list-envs` 里就能看到
`Mjlab-<Name>-Balance` 之类的 id。

---

## 3. IsaacLab（v2.1.1）

### 3.1 安装形态

- 源码仓库：`/home/cofallen/IsaacLab`，tag **v2.1.1**，`isaaclab.sh` 是入口脚本。
- 用 `pip install -e` 装进了 **conda `isaac`**：`isaaclab 0.41.3`、`isaaclab_assets 0.2.2`、
  `isaaclab_rl 0.1.8`、`isaaclab_tasks 0.10.36`、`isaaclab_mimic`。
- **没有 `_isaac_sim/` 目录**：Isaac Sim 是 pip 包装的，所以 `isaaclab.sh` 走的是
  "conda 里的 python + `isaacsim` entry point" 这条分支。
- 版本对应关系（README）：IsaacLab v2.1.x ⇔ Isaac Sim **4.5**。别升 5.0，除非切
  `feature/isaacsim_5_0` 分支。

### 3.2 三种启动方式（等价，选一个）

```bash
# ① 全局命令，任意目录、不用 activate（内部已设 CONDA_PREFIX/PATH/PRIME/EULA/TERM）
isaaclab -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Cartpole-v0 --headless

# ② 传统方式：先激活环境，再直接用 python
conda activate isaac
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Cartpole-v0 --headless

# ③ 只想要 isaac 环境的解释器
isaac-python -c "import isaacsim, os; print(os.environ['ISAAC_PATH'])"

# 只开仿真器 GUI
isaacsim
```

`isaaclab` / `isaacsim` 是用户写的 wrapper（`~/.local/bin/` 里），在本机把这几个坑一次性处理了：

```bash
export OMNI_KIT_ACCEPT_EULA=Y
export PYTHONNOUSERSITE=1
export __NV_PRIME_RENDER_OFFLOAD=1        # 强制走独显
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export CONDA_PREFIX=/home/cofallen/miniconda3/envs/isaac
export PATH="$CONDA_PREFIX/bin:$PATH"
[ "$TERM" = dumb ] && export TERM=xterm-256color    # 否则 isaaclab.sh 的 `tabs 4` + set -e 直接退出
exec /home/cofallen/IsaacLab/isaaclab.sh "$@"
```

`isaaclab.sh` 的参数（`./isaaclab.sh -h`）：

| flag | 作用 |
|---|---|
| `-p, --python` | 用 Isaac Sim / 当前 venv 的 python 跑脚本（**最常用**） |
| `-i, --install [LIB]` | 安装 `source/` 里的扩展 + RL 框架 |
| `-s, --sim` | 启动 `isaac-sim.sh` |
| `-t, --test` | 跑 pytest |
| `-n, --new` | 从模板新建外部工程 / 内部任务 |
| `-c, --conda [NAME]` | 建 conda 环境 |
| `-f/-d/-v/-o/-h` | format / docs / vscode / docker / help |

### 3.3 训练 / 播放 / 教程

```bash
cd ~/IsaacLab

# 训练（一定要 --headless：省显存，也不会卡在窗口加载）
isaaclab -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-v0 --headless --num_envs 64 --max_iterations 150 \
    --logger tensorboard

# 播放
isaaclab -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-v0 --checkpoint logs/rsl_rl/cartpole/<ts>/model_149.pt

# 看有哪些注册过的 task
isaaclab -p scripts/environments/list_envs.py

# 仿真入门教程（对应 docs 的 tutorials）
isaaclab -p scripts/tutorials/00_sim/create_empty.py
isaaclab -p scripts/tutorials/01_assets/run_articulation.py
```

已存在的训练产物：`~/IsaacLab/logs/rsl_rl/cartpole/`（rsl_rl）和
`~/IsaacLab/logs/skrl/cart_double_pendulum_direct/`（skrl）；`~/IsaacLab/outputs/<date>/` 是 Hydra 的。

RL 后端共 5 套：`scripts/reinforcement_learning/{rsl_rl, rl_games, skrl, sb3, ray}`。
另有 `scripts/imitation_learning/`（模仿学习）、`scripts/demos/`、`scripts/benchmarks/`。

### 3.4 用模板新建一个工程

```bash
isaaclab --new            # 生成 ~/Code/xxx 风格的 external project（交互式）
```

用户的现成例子：`~/Code/IsaacTest/cartpole`，
结构 = `source/cartpole/cartpole/tasks/{direct,manager_based}/` + `scripts/rsl_rl/{train,play}.py`，
注册的 gym id 是 `Template-Cartpole-Direct-v0`（还被手动加了一个短名 `v0`）。

**Task id 必须在代码里注册过**，不能自己编——这是最常见的报错来源：

```python
# source/.../tasks/direct/cartpole/__init__.py
import gymnasium as gym
from . import agents

gym.register(
    id="Template-Cartpole-Direct-v0",
    entry_point=f"{__name__}.cartpole_env:CartpoleEnv",      # direct 风格：自己写 env
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cartpole_env_cfg:CartpoleEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
    },
)
```

官方 manager-based 任务则是 `entry_point="isaaclab.envs:ManagerBasedRLEnv"` +
`env_cfg_entry_point`（见 `source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/cartpole/__init__.py`，id `Isaac-Cartpole-v0`）。

现成机器人配置（manager_based）：`ant, anymal_b/c/d, a1, cassie, digit, g1, go1, go2, h1, spot, humanoid, cartpole`。

### 3.5 Unitree 扩展（用户实际在用的）

`~/Code/unitree_rl_lab`，脚本 `unitree_rl_lab.sh` 把常用动作包成短参：

```bash
conda activate isaac
cd ~/Code/unitree_rl_lab
./unitree_rl_lab.sh -i                              # 安装（pip -e + conda hook + argcomplete）
./unitree_rl_lab.sh -l                              # 列任务（比 isaaclab 快）
./unitree_rl_lab.sh -t --task Unitree-G1-29dof-Velocity   # 训练（自动带 --headless）
./unitree_rl_lab.sh -p --task Unitree-G1-29dof-Velocity   # 播放
```

里面有 `deploy/mujoco/go2_sim2sim.py`（sim2sim）、`source/unitree_rl_lab/tasks/locomotion/`。
G1 的 USD 模型在 `~/Code/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0`（git-lfs）。

---

## 4. Isaac Sim

### 4.1 启动

```bash
isaacsim                  # 直接开 GUI；4 秒出窗，20~30 秒才真正可用
# 8 GB 显存 / 15 GB 内存吃紧时，用低配模式（关 RTX 特性 + 1280x720）：
bash ~/Code/Mujoco/tools/run_isaac_gui.sh
```

- 启动后 **`~6.9 GB` 内存 + `1.6 GB` 显存**；首次或清空 `~/.cache/ov` 后要重编着色器 1~3 分钟，
  期间窗口"无响应"是正常现象，**别杀**。
- 同一时刻只允许一个实例（抢 kvdb 锁 + 内存翻倍），`isaacsim` wrapper 里有 `pgrep` 检查。
- 版本：`isaacsim 4.5.0.0`（`isaacsim-*.dist-info`），`ISAAC_PATH` 由包自己设成
  `.../site-packages/isaacsim`。

### 4.2 资产（这台机器被墙，所以做了本地镜像）

默认 Isaac 从 CloudFront 拉 USD，本机不通 → 改成 **本地文件镜像**：

```bash
isaac-assets-sync --dry-run Isaac/Props/          # 先看体积
isaac-assets-sync Isaac/Props/                    # 下到 ~/isaac_assets
isaac-assets-sync Isaac/IsaacLab/Robots/Anymal/   # 按需下某个机器人
```

路径相对 `Assets/Isaac/4.5/`。已改的配置（改坏了按 REPORT.md 第 7 节回滚）：

- `~/Documents/Kit/shared/user.toml` → 资产根 `file:///home/cofallen/isaac_assets`
- `<envs/isaac>/…/omni/data/Kit/Isaac-Sim/4.5/user.config.json`（3 个键，有 `.bak`）
- `~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/user.config.json`（2 个键，有 `.bak`）
- `~/IsaacLab/apps/isaaclab.python*.kit`（5 个文件，15 行，git 可回滚）

### 4.3 conda 激活钩子（重要）

`<envs/isaac>/etc/conda/activate.d/` 下有两个文件：

| 文件 | 内容 |
|---|---|
| `zz-isaac-isolation.sh` | `PYTHONNOUSERSITE=1`、`OMNI_KIT_ACCEPT_EULA=Y`、`unset ISAAC_PATH`、PRIME 三件套 |
| `setenv.sh` | `ISAACLAB_PATH=`（**空的**，由 unitree_rl_lab 生成时变量没展开，属于历史残留）、`source ~/Code/unitree_rl_lab/unitree_rl_lab.sh` |

所以 **`conda activate isaac` 之后不需要手动 export 任何变量**。

---

## 5. 这台机器特有的坑（按踩坑频率排序）

| # | 症状 | 原因 | 处理 |
|---|---|---|---|
| 1 | `nvidia-smi` 报错 / `torch.cuda.is_available()=False` | **在 agent 沙箱里** `/dev/nvidia*` 被隐藏 | 换用户自己的终端；沙箱里不要跑 GPU 任务 |
| 2 | GUI 卡在 `Waiting for RtPso` / 窗口空白 / 极慢 | 双显卡 on-demand，默认落到 **AMD 780M 核显** | `__NV_PRIME_RENDER_OFFLOAD=1` + `__GLX_VENDOR_LIBRARY_NAME=nvidia`（wrapper 已带） |
| 3 | `isaaclab.sh` 一启动就退出 | `TERM=dumb` 时脚本里 `tabs 4` + `set -e` | `export TERM=xterm-256color`（wrapper 已处理） |
| 4 | 窗口"无响应" / 启动卡几分钟 | Omni 反复连被墙的 `cloudfront.net`，DNS 挂 20 s | wrapper 把代理指向死端口让它快速失败；或 `/etc/hosts` 指到 127.0.0.1 |
| 5 | `ModuleNotFoundError: isaacsim.simulation_app` + `IndexError: list index out of range` | `ISAAC_PATH` 残留了不存在的旧路径 | `unset ISAAC_PATH` 或开新终端（`zz-isaac-isolation.sh` 已 unset） |
| 6 | `import numpy` 版本不对 / 各种诡异覆盖 | `~/.local/lib/python3.10/site-packages` 里有 fairseq/gradio/vosk，优先级高于 conda | `PYTHONNOUSERSITE=1`（已设） |
| 7 | `EOFError` / 卡在 EULA | 非交互 shell 里 Kit 问 EULA | `OMNI_KIT_ACCEPT_EULA=Y`（已设） |
| 8 | 训练/播放一跑就黑屏或 OOM | 没加 `--headless`，Full GUI 只剩 ~1 GB 余量 | 训练一律 `--headless`；GUI 用 `run_isaac_gui.sh` |
| 9 | `Environment xxx doesn't exist` | `--task` 用了没注册的 id | 先 `list_envs.py` 看真名，或改 `gym.register(id=...)` |
| 10 | 训练日志"卡住"无输出 | 异常被 `env.close()` 吞了 | `python -u` 重跑，看 traceback |
| 11 | mjlab 里 `torch.cuda.is_available()` 为 False | 同上第 1 条（沙箱），或没 `--gpu-ids` | 用户在真终端跑；`mjlab --info` 可自检 |
| 12 | 内存不够（15 GB） | 双实例 / Full GUI | `free -h`、`pgrep -f isaacsim`；`~/Code/Mujoco/tools/setup_swap.sh` 可加 swap |

其它环境事实：

- `~/.local/bin` 的 `python3` 相关包很多（fairseq/gradio/vosk…），**不要**把它们混进 conda 环境。
- conda 里还有 `GPTSoVits / RVC / UVR5 / base` 等无关环境，别在里面装 mjlab/torch。
- `DISPLAY=:0` 存在（有桌面会话），所以 mjlab 的 `--viewer auto` 会选原生窗。

---

## 6. 速查：从零到跑起来

**mjlab（30 秒）**

```bash
conda activate mjlab
list-envs
train Mjlab-Cartpole-Swingup --env.scene.num-envs 1024 --agent.logger tensorboard
tensorboard --logdir logs/rsl_rl --port 6006
play Mjlab-Cartpole-Swingup --checkpoint-file logs/rsl_rl/cartpole/<ts>/model_<n>.pt --viewer native
```

**IsaacLab（1 分钟，第一次会编着色器）**

```bash
conda activate isaac
cd ~/IsaacLab
isaaclab -p scripts/tutorials/00_sim/create_empty.py          # 冒烟
isaaclab -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-v0 --headless --num_envs 64 --max_iterations 150
tensorboard --logdir logs/rsl_rl --port 6006
```

**自检脚本**

```bash
bash ~/Code/isaac/verify_isaac.sh
isaaclab -p ~/Code/Mujoco/tools/isaaclab_smoke.py   # 期望 "ISAACLAB OK: Isaac-Cartpole-v0 ..."
mjlab --info                                        # python/torch/mjlab/warp 版本 + CUDA 是否可用
```

---

## 7. 这份文档没写、但值得知道的事

- 三套文档的权威来源：mjlab `docs/source/*.rst`（本仓库）、IsaacLab `docs/` + 官网、
  Isaac Sim 官网 `docs.isaacsim.omniverse.nvidia.com`。
- 用户之前的排障细节（7 个问题的根因、时间线、回滚方法）在 `~/Code/isaac/REPORT.md`，
  以及 `~/Code/isaac/ISAAC_TUTORIALS.md`（教程笔记）、`ISAACLAB_简单用法.md`（速查）。
- 论文/引用：mjlab 对应 arXiv 2601.22074（Zakka et al.）。
- 若要新增机器人：mjlab 走 `mjlab --new --template velocity` + 自己的 MJCF；
  IsaacLab 走 `isaaclab --new` + `source/<pkg>/<pkg>/tasks/`，并在 `__init__.py` 里 `gym.register`。
