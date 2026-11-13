# scripts：项目级工具入口

环境检查、模型生成与验收、站立基线等已有工具在这里启动，运行证据见 [runs](../runs/README.md)。下列命令从项目根目录运行，环境见[本机手册](../docs/00_context/03_本机环境总览_mjlab_IsaacLab_IsaacSim.md)。

[E002 控制演示、E003 PPO 和后续小项目](../experiments/README.md)各有自己的入口和输出目录。

## 当前完整模型入口

| 文件 | 用途 | 执行影响 |
|---|---|---|
| [run_standing.py](m02_standing/run_standing.py) | 完整闭链自由基座站立，LQR（默认）/MPC，可加 `--variant gas`、`--video`、`--viewer` | 新建 run；CPU 仿真特权状态基线，无训练 |
| [summarize_standing.py](m02_standing/summarize_standing.py) | 显式指定既有 runs，汇总表格/曲线，不重跑仿真 | 新建报告 run；示例输入见 M02 报告四组正式证据 |
| [check_design.py](m02_standing/check_design.py) | MPC/LQR 等价性、QP 约束最优性和模型哈希检查 | 读取 `--design-run`；新建证据，不积分 |
| [check_imports.py](m00_environment/check_imports.py) | 已有解释器导入检查 | 新建检查记录；不安装依赖 |
| [build_parallel_variants.py](m01_model/build_parallel_variants.py) | 生成正式闭链两版本 | **重建派生资产**，不是只读查看 |
| [check_parallel_variants.py](m01_model/check_parallel_variants.py) | 完整机构运动检查，可录像 | 创建新 run；不是平衡训练 |
| [check_parallel_integrity.py](m01_model/check_parallel_integrity.py) | 源几何/惯量/销接转动复核 | 创建新 run；需指定已有动态记录 |

已验证命令：

```bash
python -B scripts/m00_environment/check_imports.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/build_parallel_variants.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_variants.py --video
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_integrity.py --motion-run runs/m01_model/parallel_check_20261109_004237_385074
```

按任务需要选择命令，不需每次打开项目都重建/重跑。

## 历史入口

`prepare_model.py`、`audit_model.py`、`render_model_audit.py`、`check_contact_solver.py`、`check_visual_model.py`、`build_gas_spring_variants.py`、`check_gas_spring_variants.py` 保留旧串联简化/外观/气弹簧审计用途；其结果不能作为正式闭链验收。详见[历史报告](../docs/02_milestones/M01_model_audit.md)。

[fetch_research_reading.py](m00_environment/fetch_research_reading.py) 是资料归档工具，资料已下载；拒绝覆盖已有目标，无需重复执行。

站立控制（从项目根目录）：

```bash
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m02_standing/run_standing.py --duration 10 --cases nominal
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m02_standing/run_standing.py --controller lqr --cases push --viewer --duration 30
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m02_standing/run_standing.py --controller mpc --duration 30
```

`--video` 离屏保存实际仿真；`--viewer` 在桌面显示（原生 GUI 本轮未验收）。MPC 限幅激活时求解延迟可能超过 10 ms，不能据离线积分通过声称实时部署达标。报告见 [M02](../docs/02_milestones/M02_parallel_standing.md)。
