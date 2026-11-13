# 外部来源、开源参考与许可证

更新：2026-11-09。机器可读登记见 [sources.json](references/sources.json)，论文引用见 [references.bib](references/references.bib)。下表区分实际使用、只读参考和未来计划，不将第三方工作写成本项目原创。

| 来源 | 实际用途 | 版本与归属 | 许可记录 |
|---|---|---|---|
| [复旦轮腿候选仓库](https://github.com/yly-true/fudan_rl_wheel_leg) | 指定机器人原生并联 MJCF/独立连杆 mesh；URDF 仅保留历史简化实验 | commit `8204e853dfd2ed06d85a322e1a998c3d20a3be2c` | 未发现仓库级 LICENSE；所选模型/mesh 授权范围未确认 |
| [复旦开源文章](https://bbs.robomaster.com/article/1940342?source=11) | 核对机器人来源与项目背景 | 页面署名杨陆宇/星云 EGA 战队，2026-08-18 | 页面标注 CC BY-NC-SA 4.0，不能自动延伸至所有仓库资产 |
| [mjlab](https://github.com/mujocolab/mjlab) | 已有环境导入；后续训练后端 | 1.6.0，commit `0fb8a681136be94ffc636a3dd423cabb97d91f10` | 本地 LICENSE 为 Apache-2.0，原文已快照 |
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab) | 已有环境位置核查；适配尚未运行 | commit `90b79bb2d44feb8d833f260f2bf37da3487180ba` | 主框架 BSD-3-Clause；另保留 LICENSE-mimic，不合并授权范围 |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/) | 已导入模块与 SimulationApp 类 | 4.5.0.0 | 按已安装分发条款使用；本轮不将其笼统称为开源资产 |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | 原生 CPU 物理检查与模型渲染 | 3.11.0 | 本阶段未独立整理其分发许可，不影响记录实际运行版本 |
| [SuperMimic 原论文](https://arxiv.org/abs/2610.28480) / [作者项目页](https://supermimic.github.io/) | 方法参考；已核对摘要和项目概述 | 原论文与作者页面；尚未完成全文方法审计 | 学术引用，不由论文存在推断代码授权或作者代码已公开 |
| [YHFone/supermimic-mjlab](https://github.com/YHFone/supermimic-mjlab) | 阅读已有本地 README/配置，供后续模块核查 | 第三方 reference implementation，commit `9aab0a588c6941bf9862a2d444b711fd73e2b832` | 本地 Apache-2.0 原文已快照；本轮未复制/执行其算法模块 |

## 本项目独立实现与派生范围

**当前正式资产为 `assets/robots/infantry_parallel/`。** 从上游完整原生 MJCF 派生，保留所有 14 个树形铰链、4 个闭环销接与独立后连杆；所有物理质量/惯量取该原生模型，不再使用六关节串联代理。连接点只沿源销轴对齐 10 mm，完整修改见新 manifest。已接受的完整机身外观作为单独视觉层沿用，不替代物理模型。旧串联资产及其以下说明均保留为历史记录，已撤回完整机构交付状态。

当前原创脚本仅负责环境检查、源模型审计、URDF 到 MJCF 的独立转换、碰撞凸包生成、数值接触对照与诊断录像。未复制上游训练环境、奖励函数、PPO 或控制实现；阅读代码取得的映射事实在 M01 报告中注明来源。

派生资产位于 `assets/robots/infantry_v4_standard_simplified/`。其 manifest 记录源 commit、源 URDF/mesh 和派生文件 SHA256；质量惯量来自标准 V4，不混用 increase/并联版本。完整视觉网格采用源 STL 三角记录保形分块；按用户选择，仅配色取自同仓库并联 MJCF 的白色机身/深灰轮胎，颜色来源单独登记，不复用其物理参数。派生物仍继承来源追溯义务，不能因转换格式或生成凸包而视为不受源资产授权约束。

另按用户要求新增无气弹簧/可压缩阻尼两变体：在派生网格中移除源 CAD 气弹簧缸体/杆，保留版的安装点明确引用同仓库原生 MJCF 的气弹簧 sites，再适配到标准 V4 link 坐标。该引用范围不再只是配色，因此单独登记在 sources.json 与 `gas_spring_variants.json`。刚度、阻尼和自由长度为本项目未标定的演示设置，没有当成上游硬件辨识值；整体惯量不猜测拆分。

上游代码中存在 NVIDIA/ETH 的 BSD-3-Clause 文件头，这只能证明相应代码文件的声明，不证明整仓库机器人模型和 mesh 的许可证。当前保留完整原始 checkout，没有删除原声明，也未擅自为其加 MIT/Apache LICENSE。

## 引用与发布更新规则

后续实际复用任何模块，应在 sources.json 增加文件范围、修改内容、commit 和许可证证据；复制代码时保留版权/SPDX。仅受某方法启发时注明差异，不写成完整复现。

论文中的“世界模型方法迁移到命令跟踪”与“原论文参考动作跟踪复现”分别报告。本地 README 成绩、作者论文数字和本项目实际指标分别引用。尚未运行的实验不生成结果引用。

原始许可快照位于 [references/licenses/](references/licenses/)，每份的源位置和哈希在 sources.json。项目原创代码的发布许可证尚未选择；对外分发机器人派生资产前需先明确该资产授权范围。目录整理和本地审计不改变授权状态。

## 2026-11-09 算法选题检索（仅阅读）

新增原论文：显式约束动力学学习（2010.13581）、MACURA（2405.19014）、SAPO（2412.12089）、DMO（2509.00215）、MBPO（1906.08253）。原始 URL、作者和已核对版本见 sources.json；引用见 references.bib。仅核对摘要与作者项目概述，未完成全文审计，未复制或运行代码，未将论文授权延伸为代码授权（代码许可 unknown）。候选方案见学习笔记 M05_00；不表示用户已经确定选题。

## 2026-11-09 世界模型选题资料包

用户授权下载的 12 篇原论文和 8 份代码，仅供阅读，未安装或执行。PDF 放在 `references/papers/research_20261109/`；源码固定于 `upstream/research_20261109/`，不改动上游。逐项版本、作者、原始 URL、SHA256、Git commit/tree、许可和下载日志见 [完整清单](references/reading_manifest_20261109.json)，阅读入口见 [导读](references/READING_GUIDE.md)。随仓库附带的其他机器人资产仅作为原始 checkout 保留，未用于本项目机器人，不改变指定完整闭链模型来源。未另行拉取外部 checkpoint、大型数据集、Git LFS 或 submodule；仓库自带文件原样保留。

| 代码 | 归属边界 | 许可（未推广到全部附带资产） | 固定 commit |
|---|---|---|---|
| [facebookresearch/mbrl-lib](https://github.com/facebookresearch/mbrl-lib) | 研究库维护者实现；含 PETS/MBPO，不等同原论文代码 | MIT (root LICENSE) | `3f93cccfc8d635f74e335a2f07aab6e9a48fc021` |
| [antonilo/rl_locomotion](https://github.com/antonilo/rl_locomotion) | RMA 作者项目页链接的后续 CMS/RMA 派生实现 | GPL-3.0 (root LICENSE.md); Raisim and bundled assets separate | `f71db8195bfb49b6d964bf53be3c5adadb0c11b1` |
| [danijar/dreamerv3](https://github.com/danijar/dreamerv3) | 作者维护的重实现；不是论文历史环境锁 | MIT (root LICENSE) | `e01491fad6434b2245a3b8ca201dd7faedcc458c` |
| [nicklashansen/tdmpc2](https://github.com/nicklashansen/tdmpc2) | 作者官方实现 | MIT (root LICENSE) | `332f19e4aba4bffe2dc736f45ea7f7bdc585903a` |
| [danijar/daydreamer](https://github.com/danijar/daydreamer) | 作者实现 | unknown at repository root; component licenses retained, no blanket grant inferred | `709016ce11c32dad1f24c4cc77afdf980d2a92a8` |
| [YHFone/supermimic-mjlab](https://github.com/YHFone/supermimic-mjlab) | 第三方 reference implementation；非作者官方代码 | Apache-2.0 (root LICENSE) | `9aab0a588c6941bf9862a2d444b711fd73e2b832` |
| [machines-in-motion/DMO_code](https://github.com/machines-in-motion/DMO_code) | 作者项目页链接；页面标注 work in progress | NVIDIA Source Code License for DiffRL (custom; root LICENSE.md); component terms separate | `f60a9ddb8f0a840b2e00602cd51e627fba394b78` |
| [bytedance/WMP](https://github.com/bytedance/WMP) | 论文作者项目对应公开代码；保留依赖的其他项目许可 | Apache-2.0 (root LICENSE.txt); bundled components retain separate licenses | `c232c115ada4517453ebded5019078ba055456de` |

论文下载不等于获得再发布授权。DayDreamer 无根目录统一许可，按组件保留原许可，整体写 unknown；DMO 的根许可为 NVIDIA 自定义 DiffRL 许可，不能标成 MIT/Apache。RMA 项目链接的 rl_locomotion 当前是 CMS 派生代码；SuperMimic-mjlab 是第三方实现；DreamerV3 作者仓库自称重实现，均不声称与论文历史实验完全一致。DreamWaQ 与近期 latent-adaptation 论文未在本轮核对到可确认的对应作者训练代码，保留论文和来源，不用同名仓库冒充。

岗位参考：[智元世界模型校招条目](https://agirobot.jobs.feishu.cn/campusrecruitment/position/7670760310362671411/detail)，仅用于能力差距分析；检索到的官方域名摘要非录用或岗位长期开放保证。

## 2026-11-09 用户原站立控制参考

用户指定 [Cofallen/Leg_simulator_mujoco](https://github.com/Cofallen/Leg_simulator_mujoco)，本地原始 checkout 为 `upstream/Leg_simulator_mujoco`，main `b0f14e8ee9331d572738885691389742028cb796`；LQR+MPC 分支 `f8039cde95b8c2a9150c6016621ea928c9215c36`，MPC 分支 `faaa4e2f5e08bfd6c3ff0d7e09933f17b44e85eb`。只读参考 LQR_six 的状态反馈、支撑/VMC 和滚动时域思想，保留原始文件与来源。未找到根 LICENSE，许可为 **unknown**。未复制旧控制源码到本项目控制器，也未使用该仓库的机器人资产、旧质量/杆长/轮径/A/B/K。

本项目站立基线独立实现完整 MJCF 的接触平衡求解、离散 LQR 与输入约束 MPC，直接使用仿真特权状态；与原六状态估计器/VMC 的实现不同，不能写成逐行迁移、原仓库复现或可部署控制器。源码阅读快照和早期失败原型见 `runs/m02_standing/source_and_probes_20261109_135708_139243/`。上文“原创脚本仅负责审计”是 M01 历史范围；本轮新增站立实现，仍未复制/运行上游学习训练代码。

## 2026-11-09 按原作者代码重新适配（E002）

用户进一步明确要求尽量保留其原实现。本轮 `experiments/E002_original_controller/` **实际复制并适配** main 的 controller/get_sensor/write_sensor/leg_data/mymath/get_state/vmc/lqr_controller/lqr/vofa/keyboardInput，以及 lqr+mpc 的 mympc。上文“只参考未复制”仅描述前版 E001，不适用于 E002。原提交、逐文件 SHA256、具体修改 patch 与 unknown 许可声明保存在 E002 的 SOURCE_MANIFEST.json、SOURCE_CHANGES.patch、THIRD_PARTY.md。原声明/版权保留；不另加开源许可。新增日志辅助、完整机构几何适配与六状态矩阵标定注明本项目适配，不使用 37 维反馈假扮作者代码。

2026-11-09 交互鼠标操作参考：读取 [MuJoCo 官方 simulate 操作说明](https://mujoco.readthedocs.io/en/3.3.1/programming/samples.html#simulate)，用于核对双击选择/Ctrl 拖动，没有复制正文或源码，也不将文档许可延伸到机器人资产。轮子恢复的是已有源 STL，地面为本项目程序化棋盘材质，没有新增外部模型或贴图。

2026-11-09 按住/释放更新：E002/keyboardInput.py 已由原pynput参考工具改为本项目新增的系统libX11查询实现，实际用于聚焦窗口输入；旧“键盘工具未启用”的说明仅属历史。通过Python标准库ctypes调用已安装库，未复制/分发X11库或装包，其分发条款不延伸到机器人资产。Teleop改真实held-state目标与增强跳跃，原作者控制来源仍按现有固定commit追溯。

2026-11-13，E002 自起状态机参考用户提供的嵌入式 `StateSelect.c` 和 `RecoveryControl.c` 片段，用于阶段顺序和输出限幅设计。仿真控制为本项目针对完整闭链重新实现，用户片段未复制到工程，发布许可记为 unknown。记录见 [E002 来源说明](experiments/E002_original_controller/THIRD_PARTY.md)。
