> 本文件为 M00 环境阶段记录。当前进度以 [项目总览](../01_project/overview.md) 为准；正文中的后续状态保留当时语境。

# 轮腿机器人与可微世界模型：任务总览和进度

更新日期：2026-11-08（Asia/Shanghai）。本轮范围：读取已有文档、取得候选仓库、验证现有环境导入、记录进度。不启动训练。

## 1. 文档与当前目标

已读取工作区四份材料：`01_Codex使用指南_机器人毕设.md`、`02_毕业设计选题大纲_文献核对版.md`、`03_本机环境总览_mjlab_IsaacLab_IsaacSim.md`、`毕设选题大纲_可微世界模型持续学习.md`。

01 提供项目位置和科研记录约定；02 收敛持续学习问题并修正文献表述；原大纲保留研究假设和候选方案，不能把预算估计、预期创新或历史运行描述当成本轮结果。03 是本机环境使用依据，优先于旧大纲中的“驱动需修复”描述。

当前实施平台以用户本次明确指定的复旦轮腿模型为准。旧稿中的 G1 主平台安排保留为历史选题材料，不自动替代轮腿任务。近期实施顺序为模型审计 → mjlab 简单任务 → PPO 小基线 → 原生 MuJoCo 策略闭环 → 同一任务的可微世界模型实验。Isaac Lab 适配逐步验证。

## 2. 本轮实际完成

| 项目 | 结果 | 验证边界 |
|---|---|---|
| 候选仓库下载 | 已完成，位于 `upstream/fudan_rl_wheel_leg` | 浅克隆，318 个受 Git 管理文件，目录约 168 MB，工作区干净；`git fsck --full` 返回 0 |
| MuJoCo 导入 | 成功，3.11.0，Python 3.12.13 | 使用已有 mjlab conda 环境 |
| mjlab 导入 | 成功，1.6.0 | 未创建 Warp 仿真或训练环境 |
| CPU 物理步进 | 成功，时间推进至 0.002 s | 使用临时构造的球体 XML，仅检查库调用，不是轮腿模型验收 |
| Isaac Sim 导入 | 成功，4.5.0.0，Python 3.10.20 | 使用已有 `isaac-python` wrapper |
| SimulationApp 类导入 | 成功 | 未实例化；未验证 Kit、PhysX、GPU 或轮腿场景运行 |
| 环境保护 | 未安装、升级、卸载依赖 | 未修改已有环境、启动 wrapper、驱动、资产镜像或其他项目 |

证据目录：`runs/m00_environment/imports_20261108_234030_306309/`，含 `mjlab.log`、`isaac.log`、`results.json` 和两个环境的包版本快照。两项导入命令与两项清单命令退出码均为 0。

版本清单是当前安装状态快照，尚不是完整可重建的依赖锁（例如 editable 项目的本地修改和二进制来源仍需后续记录）。不将它用于自动覆盖安装。

## 3. 环境使用约定

| 用途 | 解释器/入口 | 本轮确认 |
|---|---|---|
| MuJoCo、mjlab | `/home/cofallen/miniconda3/envs/mjlab/bin/python` | Python 3.12.13、MuJoCo 3.11.0、mjlab 1.6.0 导入通过 |
| Isaac Sim | `/home/cofallen/.local/bin/isaac-python` | Python 3.10.20、Isaac Sim 4.5.0.0、SimulationApp 类导入通过 |
| Isaac Lab | `/home/cofallen/IsaacLab`；已有 `isaaclab` wrapper | 本轮未验证运行 |
| SuperMimic 候选实现 | `/home/cofallen/Code/supermimic-mjlab` | 目录存在，已读取 README 和 pyproject，未改动、未运行 |

按 03 手册，机器为 RTX 4060 Laptop 8 GB、双显卡 on-demand。当前 agent 沙箱内 `nvidia-smi` 失败不能证明宿主驱动故障，不因此修复或重装驱动。GPU 仿真需在有设备访问权限的执行环境另行验证。Isaac GUI 一次只启动一个实例，保留现有 PRIME、隔离和本地资产配置。

重复本轮检查（不安装依赖、不创建 SimulationApp）：

```bash
cd /home/cofallen/Code/毕业设计
python -B scripts/m00_environment/check_imports.py
```

脚本使用各自的绝对解释器路径，设置 `PYTHONNOUSERSITE=1`、禁用字节码写入，Matplotlib 缓存写入本次运行目录。每次生成独立日志，不覆盖旧结果。已有 Torch 导入发出 `torch.jit.script` 弃用警告，命令仍成功；本轮不为此更改依赖。

## 4. 来源与版本

| 内容 | 来源 | 当前 commit |
|---|---|---|
| 轮腿候选仓库 | https://github.com/yly-true/fudan_rl_wheel_leg | `8204e853dfd2ed06d85a322e1a998c3d20a3be2c` |
| 本地 mjlab | `/home/cofallen/mjlab` | `0fb8a681136be94ffc636a3dd423cabb97d91f10` |
| 本地 Isaac Lab | `/home/cofallen/IsaacLab` | `90b79bb2d44feb8d833f260f2bf37da3487180ba` |
| SuperMimic 候选实现 | https://github.com/YHFone/supermimic-mjlab | `9aab0a588c6941bf9862a2d444b711fd73e2b832` |

以上本地仓库 commit 不代表其全部工作区修改已审计。轮腿仓库本轮下载命令为 `git clone --depth 1 https://github.com/yly-true/fudan_rl_wheel_leg.git upstream/fudan_rl_wheel_leg`；已有下载完成后不重复克隆。

机器人来源文章：https://bbs.robomaster.com/article/1940342?source=11 。文章页面标注 CC BY-NC-SA 4.0；该标注不能自动当作仓库中所有模型和代码的授权。当前 checkout 未发现名称含 LICENSE/license 的文件，资产再分发许可待确认，保留原始仓库和全部原有声明，不自行附加许可证。

上游 README 使用 Isaac Gym Preview 4 与旧版 MuJoCo，并非 Isaac Lab。只取其机器人模型及必要资产，不安装上游训练环境，不复用其任务实现。本轮未发现 Git LFS 指针文本，但 mesh 完整性和模型引用仍需在模型审计时逐项验证。

候选模型含 `plane/resources/robots/*/urdf/`、`jump/resources/robots/*/urdf/` 和 `mujoco/assert_now/infantry_binglian_yuntai/infantry_V2/meshes/mjmodel.xml`；存在多个串联/并联及机器人版本，当前未选定主模型，不混用不同版本参数。

SuperMimic 论文：https://arxiv.org/abs/2610.28480 ，项目：https://supermimic.github.io/ 。已核对摘要与项目说明，尚未完成全文算法审计。本地 YHFone 仓库自称 reference implementation，先按第三方候选实现管理，不能认定为作者代码。README 描述采集、模型拟合、冻结模型中的 BPTT 与 critic bootstrap；描述不等于本轮验证。

## 5. 后续里程碑与验收门槛

| 里程碑 | 最小交付及验收 | 状态 |
|---|---|---|
| M0 取得仓库与导入 | 记录 commit、独立环境导入日志、环境保护约定 | 本轮完成 |
| M1 模型审计与物理验收 | 选定同源模型版本；质量/惯量/轴/限位/执行器/闭链/碰撞审计、关节映射；加载、落体、静态接触、小动作方向和穿透检查 | 待开始 |
| M2 mjlab 最小任务 | 统一任务接口；平衡、前后速度、偏航命令；高度仅在结构可调时加入；验证 reset、奖励、terminated/timeout、日志 | 待 M1 |
| M3 PPO 小基线 | 先短训练；预定义阈值、时长与留出命令/扰动/种子；报告成功率、误差、跌倒率、饱和比例 | 待 M2 |
| M4 原生 MuJoCo 策略运行 | actor 与完整推理元数据；同输入动作一致性；前进/停止/倒退/转向脚本、评测与视频 | 待 M3 |
| M5 尽早接入世界模型 | 同一平地任务收集转移，留出集单步/多步预测，短时 BPTT；冻结模型参数但保留状态/动作梯度；交替训练并回仿真评测 | 待简单闭环，不等复杂技能 |
| M6 统一参考与方法比较 | ReferenceMotion mask；先静止/低速/转向参考；A 命令任务方法迁移、B 参考跟踪分别报告；记录总交互、总耗时、显存和失败类型 | 待 M5 |
| M7 Isaac Lab 适配 | 共享语义、模型参数和控制周期对照、物理验收、独立跨引擎评测 | 当前未验证 |
| M8 复杂技能和集成 | 按自起 → 低跳 → 单台阶 → 楼梯逐个验收，再检查策略切换 | 当前不实施 |

接口定义须明确 SI 单位、坐标/四元数/角速度约定、关节顺序与符号、动作尺度与限制、physics_dt/control_dt/decimation、actor/critic/世界模型状态、历史与归一化。连续轮相位不直接展开输入，不默认横向速度可独立控制。动作方案待 M1 模型证据决定。

世界模型须覆盖执行器/延迟等动力学状态，合理更新姿态，并明确不可微奖励与终止、bootstrap 的处理。普通动力学预测器不算完整 SuperMimic。正式比较尽量三个种子，计入采集、模型拟合和策略更新成本；不承诺论文加速倍数或彻底免奖励设计。

## 6. 当前未验证与下一步

本轮没有轮腿模型加载结果、训练 checkpoint、策略成绩或视频；没有 Isaac Sim 应用启动成功或双后端运行结论。GPU 访问边界、模型版本选择、mesh 引用、闭链兼容性、许可证范围均需后续核查。

下一阶段最小任务：只读比较串联/并联模型与上游控制映射，形成审计报告和关节表，选定同源模型后做原生 MuJoCo CPU 加载及物理小测试。先完成这一关，再实现训练任务；不先搭建未运行的大量训练代码。
