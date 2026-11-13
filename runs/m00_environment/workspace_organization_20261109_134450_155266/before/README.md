# 轮腿机器人控制与可微世界模型毕业设计

当前进度：M00 环境导入已验证；M01 完整并联机构的 14 个转动关节、4 个闭合销接已通过 CPU 运动检查。正式模型位于 `assets/robots/infantry_parallel/`，旧六关节简化版仅作历史对照。mjlab 训练和 Isaac Lab 仿真尚未验证。

当前两套模型：[完整闭链无气弹簧](assets/robots/infantry_parallel/robot_no_gas_spring.xml)、[完整闭链可压缩弹簧阻尼](assets/robots/infantry_parallel/robot_gas_spring_damped.xml)。

2026-11-09 选题资料：[三个世界模型候选与条件路线](docs/01_project/TOPIC_CANDIDATES_20261109.md)、[12 篇论文与 8 份代码阅读入口](references/READING_GUIDE.md)。结合用户已有 LQR 实物，优先建议自适应世界模型与预测控制；尚未定题，本轮没有算法实现或训练。

## 阅读入口

1. [任务总览与当前进度](docs/01_project/overview.md)：当前做什么、完成证据、下一步。
2. [目录和交付规范](docs/01_project/CONVENTIONS.md)：文件放哪里、里程碑如何验收。
3. [本机环境手册](docs/00_context/03_本机环境总览_mjlab_IsaacLab_IsaacSim.md)：已有解释器、wrapper 和沙箱边界。
4. [完整并联模型报告](docs/02_milestones/M01_parallel_linkage.md)：全部连杆、关节、闭环修正、气弹簧依据和实际测试。
5. [开源来源与使用边界](THIRD_PARTY.md)：仓库 commit、许可证、论文与第三方复现的区别。
6. [运行记录索引](runs/INDEX.md)：验收采用哪次结果，历史失败与迭代记录在哪里。
7. [历史简化版审计](docs/02_milestones/M01_model_audit.md)：保留前期取舍和错误记录，不作为完整机构交付。
8. [关键操作记录](docs/01_project/KEY_OPERATIONS.md)与[学习笔记](docs/03_learning/README.md)：解释怎么做、为什么这样做、怎样验证；包含 STL、完整闭链和气弹簧专题。

## 目录

```text
docs/
  00_context/       原始选题资料与本机设置，保留历史语境
  01_project/       当前总览、目录规范、迁移清单
  02_milestones/    M00、M01 等阶段报告
  03_learning/      关键操作的原理、代码与复验学习笔记
scripts/
  m00_environment/ 环境只读检查
  m01_model/       资产派生、模型审计、接触对照、诊断录像
assets/robots/     可生成的本项目机器人派生资产与来源 manifest
upstream/          上游原始 checkout，不直接改动
references/        结构化来源、论文引用、许可证原文快照
runs/
  m00_environment/ 环境运行证据
  m01_model/       模型审计与录像证据
```

仅在实际开始下一个阶段时创建对应实现目录；不提前搭建复杂技能框架。后续共享任务代码进入 `src/wheel_leg/`，与一次性运行入口和第三方代码分开。

## 当前可运行命令

在项目根目录运行；使用已有环境，不执行安装命令。

```bash
python -B scripts/m00_environment/check_imports.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/build_parallel_variants.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_variants.py --video
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_integrity.py --motion-run runs/m01_model/parallel_check_20261109_004237_385074
```

生成器重建完整并联两版本，检查命令创建独立时间戳目录。白色机身/深灰轮胎配色沿用用户选择，碰撞几何默认隐藏。当前录像是固定基座机构检查，只驱动真实电机轴，包含全部被动杆件与闭环销接的运动；不是平衡策略。旧串联检查脚本保留在 m01_model 中供历史复核。train/play/evaluate/export 入口在实现并实际验证后再登记。

项目原创部分尚未选择发布许可证；第三方资产不能自动套用项目未来的许可证。
