# 从零复刻 E003 的 mjlab PPO 与 MuJoCo 部署

## 1 准备解释器和模型

```bash
conda activate mjlab
python -B run.py native
python -B run.py check --device cuda:0
```

在 E003 目录执行。现有环境使用 Python 3.12、mjlab 1.6、MuJoCo 3.11、Warp 1.19、RSL-RL 5.4。第一条检查走原生 CPU，第二条走 Warp GPU。XML 内有完整自由基座、14 个铰链、4 个销接约束和 6 个电机。网格、XML 和站姿来源见 `ASSET_SOURCES.json`。四个髋的限幅为 ±30 N·m，双轮为 ±5 N·m。XML actuator 声明顺序和策略顺序不同，先按名称建立映射。

## 2 写任务配置

读 `task.py` 的 `make_config()`。用 `MjSpec.from_file()` 加载模型，把 `standing_pose.json` 中的自由基座位姿和全部关节角填入 `EntityCfg.InitialStateCfg`。用 `XmlActuatorCfg` 接管已有 motor，`JointEffortActionCfg` 对六路归一化动作缩放并按 XML 限幅。物理步长 1 ms、decimation 10，策略输出每 10 ms 更新一次。同一动作保持 10 个物理子步。

`Command._resample_command()` 为每个环境填三个量，依次为纵向速度、腿长、偏航角速度。站立用 `(0, 0.28, 0)`。速度课程每 2–4 秒重采样，约 25% 零速、25% 纯旋转、50% 纯平移。纵向正速度沿模型机身 −X。命令单位分别为 m/s、m 和 rad/s。

## 3 写观测函数

`actor_observation(env, joints, hips)` 返回 `(环境数, 28)` 的 Tensor。按下表用 `torch.cat(..., dim=-1)` 拼接。critic 使用相同的输入组。

| 零基索引 | 量 | 获取方式 |
|---|---|---|
| 0–2 | 三路命令 | `command_manager.get_command("targets")` |
| 3–5 | 机身坐标下的单位重力方向 | `projected_gravity_b`，静止直立约为 (0,0,-1) |
| 6–8 | 三轴陀螺仪 rad/s | XML `base_ang_vel` |
| 9–11 | 机身原点线速度 m/s | `root_link_lin_vel_b`，仿真真值 |
| 12–15 | 四髋角相对初始站姿的偏差 rad | 关节角减 `default_joint_pos` |
| 16–21 | 六路关节角速度乘 0.1 | `joint_vel` |
| 22–27 | 上一次已裁剪动作 | 首帧全零，下一帧用刚应用的动作 |

六路顺序是右前髋、右轮、左前髋、左轮、右后髋、左后髋。四髋顺序从该列表跳过双轮得到。机身位置、被动轴和世界朝向没有作为 actor 输入。实机需要另行提供姿态和机身速度估计。

## 4 写奖励和终止

`lengths()` 取前髋铰链锚点到对应轮心的欧氏距离。令前进速度误差为 `ev=-vx_b-cmd_v`，偏航误差为 `ew=wz_b-cmd_yaw`，左右长度误差为 `eL`。代码逐环境计算：

```python
r = (
    1.5 * exp(-(gx**2 + gy**2) / 0.04)
    + exp(-ev**2 / 0.06)
    + exp(-mean(eL**2, dim=1) / 0.0025)
    + exp(-ew**2 / 0.04)
    - 2.0 * vy_b**2
    - 0.1 * (wx_b**2 + wy_b**2)
    - 0.02 * mean(action**2, dim=1)
    - 150.0 * fallen
)
```

姿态和腿长项维持支撑，速度和偏航项跟踪命令，侧滑与俯仰滚转角速度受罚。mjlab 默认将奖励乘控制步长 0.01 s，故跌倒那一步的显式惩罚为 -1.5。机身低于 0.12 m、倾斜约超过 0.6 rad 或状态非有限属于 `terminated`；训练满 6 s 属于 `timeout`，PPO 可在超时时 bootstrap。评测满 30 s。

## 5 启动 PPO 和续训

`run.py` 的 `make_runner()` 建 28→128→128→128→6 的 ELU actor 和独立 critic，分别维护观测均值、标准差。训练采样高斯动作，评测取均值。新站立策略把 actor 末层权重清零，偏置设成配平力矩除各轴限幅，探索标准差为 0.04。这个初始化不在运行时调用 LQR。

```bash
python -B run.py train --preset stand --num-envs 512 --physics-steps 24000000
python -B run.py train --preset velocity --num-envs 512 --physics-steps 48000000 --checkpoint <上一步checkpoint>
python -B run.py train --preset velocity --num-envs 1024 --physics-steps 120000000 --checkpoint <上一步checkpoint>
python -B run.py train --preset velocity --num-envs 1024 --physics-steps 100000000 --checkpoint <上一步checkpoint>
```

将尖括号项替换为对应新运行目录下 `results.json` 的 checkpoint 路径。每次收集 24 策略步再做 PPO 更新；物理步预算按 `环境数 × 24 × 10 × 更新次数` 计算。默认 gamma 0.99、GAE λ 0.95、clip 0.2、5 个 epoch、4 个 minibatch。观察 TensorBoard 中平均存活时长、回报和跌倒频次。历史短训虽保存了权重，但会在半秒左右跌倒，不能用训练退出状态代替验收。

## 6 分别验证 mjlab 和原生 MuJoCo

```bash
CKPT=outputs/train_20261113_163126_398557/model_1478.pt
for s in stand velocity rotation; do
  python -B run.py evaluate --scenario "$s" --checkpoint "$CKPT"
done
python -B deploy.py --scenario demo --checkpoint "$CKPT"
python -B deploy.py --headless --scenario demo --checkpoint "$CKPT"
```

`deploy.py` 默认打开可键盘操作的原生 MuJoCo 窗口：W/S 前后、A/D 左右转、X 停车、R 重置。上述 `--scenario demo` 则按 5 s 一段依次站立、前进、后退、左转、右转、停车。它读取 checkpoint 的 actor 和归一化缓冲区，用 `(obs-mean)/(std+0.01)` 预处理，取 ELU 网络输出均值，裁剪 ±1 并写六路 `data.ctrl`。不启动 Warp、不调用训练环境或经典平衡器。每个控制周期用 `mj_step` 积分 10 次，再用 `mj_forward` 更新观测。CPU 无头运行记录同一指令轨迹。仿真器间接触数值可能不同，动作一致性与闭环表现要分别验证。

同状态对齐可运行 `python -B check_sim2sim.py`，此检查需要 GPU。手动写 qpos、qvel 或命令后，先调用 `forward()`，再直接调用 `observation_manager.compute_group("actor")`。普通包装器读取可能返回上一帧缓存，这在第一次核对中造成了 0.2 的观测差；复验观测最大差 1.49e-8、动作最大差 5.96e-8。该问题发生在对齐检查的手动写入路径。
