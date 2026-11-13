# 工作规范

## 必须先读的结构契约

- 新对话开展工作前，先完整阅读根目录 `项目结构规范.md`。用户要求小项目尽量独立、可复制到其他位置验证；新增小实验默认在 `experiments/<编号>_<用途>/` 集中组织，不自动把小程序拆散到公共框架。
- 本规范关于新增独立小项目的约定优先于旧目录手册的分层建议。现有代码/资产/运行记录保持原位，不自动迁移；独立输出允许位于小项目自己的 `outputs/<新运行ID>/`，正式采用用索引追溯。
- 开始说明修改目录与范围；独立交付不能隐式导入父工程、硬编码原工作区路径或通过软链接写回原工程。用户要求复制验证时，按规范检查依赖闭包和异地运行，准确记录未验证项。

## 当前任务和推进方式

- 先读 `README.md`、`docs/01_project/overview.md` 和当前里程碑报告，再修改代码。
- 用户最新任务优先于历史选题稿；旧稿中的题目、预算和实验成绩不是当前验收证据。
- 按里程碑推进，先运行最小案例再扩展；普通可逆决定自行完成，不额外设置常规确认流程。
- 不主动启动长训练或同时开发复杂技能。先完成模型验收、短训练与日志检查，再按当前授权推进。
- 只有用户或适用规范明确要求时才使用子代理。

## 新对话如何认识这台电脑

核对日期：2026-11-13。Conda 根目录为 `~/miniconda3`，命名环境位于其 `envs/`。路径和版本来自本机安装元数据与命令检查。

| Conda | Python | 用途和入口 |
|---|---|---|
| `mjlab` | 3.12.13 | 本毕设仿真与训练。激活后使用 `train`、`play`。mjlab 1.6.0（源码 `~/mjlab`），MuJoCo / MuJoCo Warp 3.11.0，Warp 1.19.0，PyTorch 2.15.0，RSL-RL 5.4.2。 |
| `isaac` | 3.10.20 | Isaac Lab 源码 `~/IsaacLab`，源码版本 2.1.1（Python 包 0.41.3）。Isaac Sim 4.5.0.0，PyTorch 2.5.1+cu121。入口 `~/.local/bin/{isaaclab,isaacsim,isaac-python}`，资产 `~/isaac_assets`。 |
| `rl-course` | 3.10.22 | 强化学习课程，入口 `conda run -n rl-course python`。Gymnasium 1.4.0，PyTorch 2.14.1。 |
| `base` | 3.13.13 | Conda 管理及通用工具，`conda env list` 查看已登记环境。 |
| `GPTSoVits` | 3.10.20 | 语音合成，PyTorch 2.2.2+cu118。入口 `~/Code/GPT-SoVITS/webui.py`。 |
| `RVC` | 无独立解释器 | 变声项目入口 `~/Code/Retrieval-based-Voice-Conversion-WebUI/infer-web.py`。同名 Conda 目录已登记，运行解释器待确认。 |
| `UVR5` | 无独立解释器 | 人声分离项目入口 `~/Code/ultimatevocalremovergui/UVR.py`。同名 Conda 目录已登记，运行解释器待确认。 |

**LaTeX** 使用 TeX Live 2026，位于 `/usr/local/texlive/2026/bin/x86_64-linux/`，已找到 `xelatex`、`pdflatex`、`lualatex`、`latexmk`。中文论文可从 `latexmk -xelatex main.tex` 开始。

**验证状态**（2026-11-13）：宿主 `nvidia-smi` 识别 RTX 4060 Laptop（8188 MiB），E003 已在 GPU 上通过 mjlab/Warp 单环境检查并完成有界 PPO 短训。Isaac Sim 与 SimulationApp 有导入记录，Kit 启动、Isaac GPU 任务及语音工具运行未在本次盘点中验证。

- 开始工作先确认当前目录和项目规则。按任务选现有解释器，如 `~/miniconda3/envs/mjlab/bin/python` 或 `~/.local/bin/isaac-python`。环境版本和工具路径有变化时，用 `conda env list`、`command -v` 和目标解释器的包元数据做最小只读核对。
- Conda 已登记、解释器可用和任务实际通过是三个层次的证据。GPU 检查区分沙箱设备可见性、宿主 `nvidia-smi` 和实际任务结果。沙箱缺少 NVIDIA 设备节点时，先检查执行边界。

- Isaac 启动、资产镜像及 wrapper 的细节见 [环境手册](docs/00_context/03_本机环境总览_mjlab_IsaacLab_IsaacSim.md)。

## 现有环境保护

- 使用已有 `mjlab` 和 `isaac` 独立解释器；不在 base 或无关 conda 环境中装机器人依赖。
- 不为解决导入、GPU 不可见或单个资产错误而升级、卸载、重装已有环境或修驱动；先定位执行边界与具体错误。
- 不修改已有 wrapper、全局配置、其他项目、缓存或 checkpoint；本项目输出留在工作区。
- 沙箱内 GPU 不可见不能当作宿主驱动故障；未运行的 GPU/Isaac Lab 测试明确标记为未验证。

## 结构与来源

- 遵循 `docs/01_project/CONVENTIONS.md`，脚本、报告、资产与证据按阶段归档。
- `upstream/` 保留原始 checkout；改动在本项目代码或派生资产中实现，并保存生成方法、源路径、commit 和哈希。
- 机器人只能来自用户指定项目。不同模型版本、串联简化与并联机构不得混用参数或默认视作等价。
- 当前正式模型为 `assets/robots/infantry_parallel/` 的完整闭链版本。保留独立连杆、全部转动关节与闭环销接；不得再用六关节串联简化版替代完整机构。旧简化资产仅作历史对照。
- 原始视觉网格与碰撞几何分开：碰撞凸包不能替代正式外观。保留源视觉形状；配色采用明确参考并记录来源，不能擅自统一涂色。格式限制优先用可验证的保形转换处理。
- 新增外部参考时更新 `THIRD_PARTY.md`、`references/sources.json`；论文引用更新 `references/references.bib`。
- 保留许可证、版权与 attribution；许可证未确认时写 unknown，不自行补发开源授权。
- 区分作者实现、第三方复现、本项目独立实现和受论文启发的设计。

## 实验与交付

- 每次运行使用新目录，保存命令、解释器、配置、种子（适用时）、版本、日志、退出状态和验收结果。
- 不覆写历史结果或把历史元数据改成新路径；迁移通过索引和清单说明。
- 训练前定义成功阈值、评测时长和留出条件。区分 terminated 与 timeout，区分特权状态与可部署观测。
- 依据实际测试报告结论；失败保留原错误，不编造成功率、训练结果、视频或论文复现程度。
- 完成必要检查后更新当前总览与阶段报告，并给出下一项最小任务。
- 关键操作必须同步留下便于用户学习的记录：更新 `docs/01_project/KEY_OPERATIONS.md`，在 `docs/03_learning/` 解释问题现象、依据、原理、改了哪里、关键代码/参数、复验命令与结果、适用范围和未确认项。明确区分失败尝试、已撤回方案和当前采用方案，不只写“已修复”。
- 学习笔记应纠正容易混淆的概念（例如固定 STL、视觉几何、关节自由度、可变长度 tendon），不能把当前等效模型写成尚未实现的真实机构。仅文档更新不必重复启动仿真或训练。

## 长期工作区与多窗口接力

- 开始工作先读 `docs/01_project/WORKSPACE_GUIDE.md` 和 `docs/04_tasks/README.md`；任务状态以对应任务卡为准。overview 是唯一项目进度总表，README 只导航。
- 每个窗口在任务卡登记负责人、修改范围、时间与设备占用，重新读取确认无冲突；有人负责时先交接。卡片是协作约定，不是文件锁，不能自动接管或假定并发安全。
- 共享文件（overview、AGENTS、CONVENTIONS、来源清单）同一时间只由约定的汇总窗口修改；其他窗口在各自任务卡提供待合并结论。
- 结束/中断前更新任务卡中的实际修改、证据、未确认项和下一步。GPU/实物/模型生成目录的冲突操作顺序执行，运行输出各用独立目录。
- 目录职责见使用手册；未来 src/configs/data/deliverables 按实际实现创建，不提前造空框架。用户当前工作方向是 A+B 融合，正式题名/创新点待论证，不沿用旧候选文档的排序覆盖最新方向。
