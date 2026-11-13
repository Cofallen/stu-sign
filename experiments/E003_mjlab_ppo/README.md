# E003 完整并联轮腿的 mjlab PPO 基线

[从零训练与 sim2sim 教程](TUTORIAL.md) · 原生窗口：`python -B deploy.py`。

E003 使用 E002 无气弹簧完整闭链模型，沿用原 XML 的四髋 ±30 N·m 与双轮 ±5 N·m。资产、引用网格和初始关节姿态逐文件记录在 `ASSET_SOURCES.json`，原始资产许可状态见 `THIRD_PARTY.md`。站立预设用 E002 静力配平力矩初始化策略输出均值，探索标准差设为 0.035。训练和回放期间由策略直接输出六路力矩。

## 运行

使用现有 `mjlab` Conda 环境。版本见本机环境速查（原工作区 `docs/00_context/04_本机环境速查.md`）。使用相对或绝对路径指向本目录的 `run.py` 均可。

```bash
conda activate mjlab
python -B run.py native
python -B run.py check --device cuda:0
python -B run.py train --device cuda:0 --num-envs 256 --seed 42
python -B run.py evaluate --device cuda:0 --checkpoint /path/to/model_12.pt --scenario stand
MUJOCO_GL=egl python -B run.py play --device cuda:0 --checkpoint /path/to/model_12.pt --scenario stand --video
```

`native` 检查原生 MuJoCo，`check` 检查单环境 mjlab/Warp。`train --preset` 选择课程，默认 `stand`。默认每次训练上限为 100 万物理步或 30 分钟，先到即停。显式传入 `--physics-steps` 可增加训练步数。这里的物理步包括控制间隔内的 10 个积分子步，并累计并行环境。`evaluate` 使用 20 个固定留出种子，`play` 用单个回合保存轨迹，可同时输出 MP4。评测入口另支持 `velocity`、`length-static` 和 `length-moving`。

每次运行保存到本目录新建的 `outputs/<运行ID>/`，其中有命令、版本、输入哈希、日志、退出状态及模式对应的结果。Warp 编译缓存位于本实验的 `outputs/.warp_cache`。复制运行需要本目录的 Python、JSON 和 `assets/`，外部依赖为现有 mjlab 解释器及安装包，播放带视频还需要可用的 EGL 与 ffmpeg。

## 策略接口

- 控制频率 100 Hz，物理频率 1000 Hz。动作顺序是右前髋、右轮、左前髋、左轮、右后髋、左后髋，归一化范围 ±1。XML 原生 actuator 顺序不同，由 mjlab 按关节名称绑定。
- 28 维观测依次为纵向速度、腿长及偏航角速度指令、机身重力方向、IMU 角速度、仿真机身线速度、四个主动髋角偏差、六路缩放后的编码器角速度、上一步动作。重力方向是仿真姿态估计代理。速度正方向对应模型机身 −X。
- 左右腿长分别为前髋轴到对应轮心的三维距离，与 E002 定义相同。奖励用仿真速度、姿态及腿长，策略输入来自上述观测接口。验收阈值见 `acceptance.json`。

## 首轮结果

单环境及异地复制检查通过。首批 PPO checkpoint 完成训练链路检查，20 个留出站立回合全部约 0.20 秒跌倒。请参见首轮阶段报告（原工作区 `docs/02_milestones/M03_e003_ppo.md`）中的日志、checkpoint 和失败回放。速度跟踪与变腿长训练待后续授权。

## 训练预设

| 预设 | 指令采样 |
|---|---|
| `stand` | 零速度，腿长 0.28 m |
| `velocity` | 零速、±0.4 m/s 纵向平移或 ±0.4 rad/s 原地旋转，腿长 0.28 m |
| `length-static` | 站立、标称腿长行驶和静止变腿长 |
| `length-moving` | 站立、速度跟踪、静止和行进中变腿长 |

顺序为 `stand → velocity → length-static → length-moving`。后续每段用 `python -B run.py train --preset <名称> --checkpoint <上段模型>`。使用最后一个 checkpoint 分别运行 `evaluate --scenario stand`、`velocity`、`length-static` 和 `length-moving`，各场景保留独立输出。

第一功能只需 `stand → velocity`。用同一最终 checkpoint 依次评测 `--scenario stand`、`velocity`、`rotation`。速度 MAE 的单位为 m/s，偏航角速度 MAE 的单位为 rad/s。当前观测中的机身线速度来自仿真。旧 27 维 checkpoint 对应上一版观测，复验需使用其原始代码版本。

可选 `--assisted` 启用基于既有闭链模型 LQR 的反馈，PPO 学习六路各 ±0.2 N·m 的修正。使用同模式训练、评测和播放。`play --assisted --baseline --scenario stand` 可复验零修正对照。此模式使用仿真全状态。

已验收的混合控制回放（从 E003 目录执行）

```bash
conda activate mjlab
CKPT=outputs/train_20261113_162936_149060/model_259.pt
for scenario in stand velocity rotation; do
  python -B run.py evaluate --assisted --checkpoint "$CKPT" --scenario "$scenario"
done
python -B run.py play --assisted --checkpoint "$CKPT" --scenario rotation --video
```

纯 PPO 版本通过站立、前后平移和偏航速度三项验收。从 E003 目录运行下面的命令，评测自动使用 20 个固定留出初始俯仰。

```bash
conda activate mjlab
CKPT=outputs/train_20261113_163126_398557/model_1478.pt
for scenario in stand velocity rotation; do
  python -B run.py evaluate --checkpoint "$CKPT" --scenario "$scenario"
done
```

纯 PPO 旋转评测的平面偏移最高约 0.65 m。需要原地旋转时使用上面的混合控制命令。

原生 MuJoCo 窗口默认是键盘模式：点击窗口后，`W/S` 前进/后退（±0.4 m/s），`A/D` 左转/右转（±0.3 rad/s），`X` 停车，`R` 重置并停车。指令保持到下一次按键；跌倒后按 `R` 继续。关闭窗口结束并保存轨迹。固定指令演示用 `python -B deploy.py --scenario demo`；录制用 `MUJOCO_GL=egl python -B deploy.py --headless --scenario demo --video`。

默认 XML 的轮子外观使用原始 STL，地面使用方格纹理；圆柱仍用于碰撞，在部署窗口默认隐藏。

窗口默认按真实时间 1× 运行，保持 100 Hz 策略更新。可用 `python -B deploy.py --scenario demo --seconds 8` 复测，结果写入新目录的 `run.json`（`loop_wall_seconds`）。无头回放不等待墙钟。
