# 轮腿机器人自适应世界模型毕业设计

**新对话先读：[项目结构规范.md](项目结构规范.md)。** 可直接对新窗口说：“请遵守根目录《项目结构规范.md》，按小项目独立、可复制验证的要求完成本次任务。”本规范优先规定新增小实验的组织方式，现有目录保留原位。

**认识本机。** 机器人训练用 `~/miniconda3/envs/mjlab/bin/python`（Python 3.12.13、mjlab 1.6.0、MuJoCo/Warp 3.11.0）。Isaac Lab 2.1.1 和 Isaac Sim 4.5.0.0 使用 `isaac` 环境及 `~/.local/bin/{isaaclab,isaacsim,isaac-python}`。宿主 RTX 4060 Laptop 已运行 E003 的 Warp 单环境检查。其余 Conda 环境、LaTeX 和语音工具的版本与入口直接列在 [项目 AGENTS.md](AGENTS.md#新对话如何认识这台电脑)。

这是持续迭代到 2027 年 5 月的项目工作区。当前研究方向是 **动力学适应 + 可微世界模型中的策略学习（A+B）**；正式题目和算法贡献仍待论证。当前事实与验收范围统一见[总览](docs/01_project/overview.md)，独立小项目从 [experiments/](experiments/README.md) 开始。

## 每次打开项目，从这里开始

| 你要做什么 | 打开哪里 |
|---|---|
| 找已有小项目、开始新实验 | **[独立小项目](experiments/README.md)** |
| 看真实进度、研究主线、下一步 | **[当前总览](docs/01_project/overview.md)**：全项目唯一的进度总表 |
| 找当前任务、查看哪个窗口在做什么 | **[任务看板](docs/04_tasks/README.md)**：任务入口，状态以对应任务卡为准 |
| 理解目录、扩展功能、接力协作 | **[工作区使用手册](docs/01_project/WORKSPACE_GUIDE.md)** |
| 阅读论文与参考代码 | [资料导读](references/READING_GUIDE.md)：12 篇论文、8 份代码 |
| 本机环境、mjlab PPO | [一页环境速查](docs/00_context/04_本机环境速查.md)、[E003 独立实验](experiments/E003_mjlab_ppo/README.md) |
| 可微世界模型 | [E004 SuperMimic 式训练与部署](experiments/E004_supermimic/README.md) |
| 键盘行走、腿长与跳跃 | [按原代码结构的站立项目](experiments/E002_original_controller/README.md)、[原控制入口](scripts/README.md)、[LQR/MPC 站立报告](docs/02_milestones/M02_parallel_standing.md) |
| 学习已做操作和原理 | [学习笔记](docs/03_learning/README.md) |

第一次使用：看总览 → 找任务卡 → 按任务卡指定入口工作。每次离开：把实际结果、证据和下一步写回任务卡；阶段结论由总览维护窗口汇总。

## 目录用日常语言解释

| 目录 | 含义 | 你通常怎么用 |
|---|---|---|
| [experiments/](experiments/README.md) | 后续开发的默认位置，一项实验一个小项目 | 在项目内找入口、依赖、资产和运行结果 |
| [docs/](docs/README.md) | 项目说明、报告和学习记录 | 读计划、查证据、写结论 |
| [assets/](assets/README.md) | 本项目实际使用的机器人模型 | 找正式 XML、网格和来源；不混用机器人版本 |
| [scripts/](scripts/README.md) | 已有的项目级检查、资产生成与站立基线入口 | 按索引执行模型检查；旧入口单独标为历史 |
| [runs/](runs/README.md) | 已有项目级运行证据，一次执行一个目录 | 查日志、结果、录像、配置；历史记录保留原样 |
| [references/](references/README.md) | **阅读资料和引用** | 找论文 PDF、来源、许可证和阅读顺序 |
| [upstream/](upstream/README.md) | **下载的原始第三方源码** | 阅读/对照，保持原始 checkout，不在里面开发毕设 |

`scripts` 是入口，`runs` 是入口执行后留下的证据，`upstream` 是外部参考代码。后续新实验默认在 `experiments/<编号>_<用途>/` 内维护，启动命令写在自己的 README，日志和结果留在自己的 `outputs/<运行ID>/`。已有项目级入口和证据继续通过 scripts 和 runs 查找。

## 当前正式机器人

[完整并联模型说明](assets/robots/infantry_parallel/README.md)：保留 14 个转动关节、4 个闭合销接与全部独立连杆。

- [无气弹簧版本](assets/robots/infantry_parallel/robot_no_gas_spring.xml)
- [等效压缩弹簧阻尼版本](assets/robots/infantry_parallel/robot_gas_spring_damped.xml)
- [模型验收报告](docs/02_milestones/M01_parallel_linkage.md)

旧串联简化资产仅供历史对照。机构检查录像与站立控制录像分开归档，见 [M01](docs/02_milestones/M01_parallel_linkage.md) / [M02](docs/02_milestones/M02_parallel_standing.md)；均不是学习策略训练成果。

小项目命令见各自 README，项目级命令见 [scripts 使用说明](scripts/README.md)。阶段报告保留当时的执行命令。环境使用[本机手册](docs/00_context/03_本机环境总览_mjlab_IsaacLab_IsaacSim.md)中的已有独立解释器。

项目协作约定见 [AGENTS.md](AGENTS.md)，实验和引用规范见 [CONVENTIONS.md](docs/01_project/CONVENTIONS.md)，第三方边界见 [THIRD_PARTY.md](THIRD_PARTY.md)。
