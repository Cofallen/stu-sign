# 独立小项目

后续的小型开发、方法验证和学习案例默认从这里开始。每个小项目有自己的说明、入口、实际需要的配置和资源，以及运行输出。

| 小项目 | 用途 | 入口 |
|---|---|---|
| [E002 原结构控制](E002_original_controller/README.md) | 完整闭链 MuJoCo 控制、键盘驾驶与场景演示 | [controller.py](E002_original_controller/controller.py) |
| [E003 mjlab PPO](E003_mjlab_ppo/README.md) | 完整闭链接入、PPO 训练和留出评测 | [run.py](E003_mjlab_ppo/run.py) |

E001 曾用于独立站立对照，当前目录未包含该副本。新增编号前查阅现有目录和[任务卡](../docs/04_tasks/README.md)。

## 新增一个小项目

1. 在这里创建 `<编号>_<用途>/`，在 README 写清目的、入口、依赖和复验方法。
2. 把代码、需要随项目带走的资源和配置集中在该目录。外部模型或大数据通过显式输入提供，并记录版本和来源。
3. 每次运行使用新的 `outputs/<运行ID>/`。阶段报告可链接到原始结果，目录随小项目一起复制。

已有项目级入口见 [scripts](../scripts/README.md)，相应运行证据见 [runs](../runs/README.md)。独立交付的路径和复制验证要求见[项目结构规范](../项目结构规范.md)。
