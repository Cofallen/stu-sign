# 世界模型选题资料：先读什么、代码看哪里

更新：2026-11-09。用户当前要求以选题为主，不编写算法代码。已下载 **12 篇论文、8 份原始代码 checkout**，用于离线查阅；没有安装依赖、执行第三方训练、下载外部大模型权重或改变正式机器人。仓库自带资产原样保留，不作为本项目机器人来源。

先看[候选选题与路线](../docs/01_project/TOPIC_CANDIDATES_20261109.md)。推荐优先考虑“自适应世界模型与预测控制”，但尚未确定最终选题。

## 1. 按选题阅读，不把 12 篇都当作前置作业

| 顺序/用途 | 本地论文 | 要回答的问题 | 对应代码 |
|---|---|---|---|
| 入门第一篇 | [PETS](papers/research_20261109/pets.pdf) | 动作如何进入动力学预测？模型误差怎样影响规划？ | MBRL-Lib 的实现参考，不冒称 PETS 原始作者代码 |
| 候选 A 必读 | [RMA](papers/research_20261109/rma.pdf) | 为什么历史观测能帮助适应负载/摩擦？部署时有哪些信息拿不到？ | RMA 项目页链接的 CMS 派生代码 |
| 候选 A 查重 | [Online Adaptation to Latent Physical Properties](papers/research_20261109/latent_adaptation.pdf) | 预测误差更新隐变量已经怎么做？本项目还剩什么具体研究问题？ | 本轮未确认对应作者训练代码；不以同名项目替代 |
| 候选 A 控制参考 | [TD-MPC2](papers/research_20261109/tdmpc2.pdf) | 面向控制的潜在模型和在线规划如何结合？ | 作者官方实现 |
| 本体感知基础 | [DreamWaQ](papers/research_20261109/dreamwaq.pdf) | 历史编码、状态估计与策略如何衔接？ | 本轮未确认作者训练代码 |
| 候选 B 核心 | [SuperMimic](papers/research_20261109/supermimic.pdf) | 模型学习和策略学习如何交替？梯度经过哪里？ | 本地归档为第三方 supermimic-mjlab |
| 世界模型体系 | [DreamerV3](papers/research_20261109/dreamerv3.pdf) | 隐状态、动作条件预测、想象轨迹和 actor-critic 的关系？ | 作者维护的重实现，非论文历史环境锁 |
| 候选 B 对照 | [DMO](papers/research_20261109/dmo.pdf) | 为什么仿真器前向与学习模型反向可以分开？ | 作者代码，项目仍标注 work in progress |
| 模型误差基础 | [MBPO](papers/research_20261109/mbpo.pdf) | 模型展开为什么不能盲目加长？ | MBRL-Lib 实现参考 |
| 候选 B 查重 | [MACURA](papers/research_20261109/macura.pdf) | 不确定性控制展开长度有哪些已有方法？ | 本轮未另行下载对应代码 |
| 候选 C 核心 | [WMP](papers/research_20261109/wmp.pdf) | 预测世界模型如何为感知与控制提供表征？ | 论文作者项目对应代码 |
| 实物工程选读 | [DayDreamer](papers/research_20261109/daydreamer.pdf) | 实机数据收集、模型训练与控制怎样形成闭环？ | 作者实现，根目录统一许可未确认 |

阅读范围说明：本轮核对原论文元信息、摘要，以及 RMA/DreamWaQ、SuperMimic、WMP、近期隐变量适应论文的相关方法段和部分代码入口；其余做摘要/README 层面筛选。未逐篇完整复现、未完成系统性新颖性审查。PDF 有 arXiv 版本差异，精确版本与 SHA256 见清单；会议版/期刊版名称或作者顺序不擅自覆盖所下载版本。

## 2. 八份代码的离线入口

以下源码全部保留在 `upstream/research_20261109/`，不要直接修改。

| 入口 | 优先阅读 | 用途与边界 |
|---|---|---|
| [MBRL-Lib](../upstream/research_20261109/mbrl-lib/README.md) | [概率 MLP](../upstream/research_20261109/mbrl-lib/mbrl/models/gaussian_mlp.py)、[模型训练器](../upstream/research_20261109/mbrl-lib/mbrl/models/model_trainer.py) | 动力学模型与训练器；MIT 根许可 |
| [RMA/CMS](../upstream/research_20261109/rl_locomotion/README.md) | [网络模块](../upstream/research_20261109/rl_locomotion/raisimGymTorch/algo/ppo/module.py) | 作者项目页链接的后续 CMS 派生实现；依赖 Raisim；GPL-3.0 根许可 |
| [TD-MPC2](../upstream/research_20261109/tdmpc2/README.md) | [世界模型](../upstream/research_20261109/tdmpc2/tdmpc2/common/world_model.py)、[规划与更新](../upstream/research_20261109/tdmpc2/tdmpc2/tdmpc2.py) | 作者官方 PyTorch 实现；MIT 根许可 |
| [SuperMimic-mjlab](../upstream/research_20261109/supermimic-mjlab/README.md) | [网络](../upstream/research_20261109/supermimic-mjlab/src/supermimic/algo/networks.py)、[训练器](../upstream/research_20261109/supermimic-mjlab/src/supermimic/algo/trainer.py) | 第三方实现；Apache-2.0 根许可；未验证闭链适配 |
| [DreamerV3](../upstream/research_20261109/dreamerv3/README.md) | [Agent](../upstream/research_20261109/dreamerv3/dreamerv3/agent.py) | 作者 JAX 重实现；MIT 根许可；本轮不向现有环境安装 JAX 栈 |
| [DMO](../upstream/research_20261109/DMO_code/README.md) | [DMO-SHAC](../upstream/research_20261109/DMO_code/algorithms/dmo_shac.py) | 作者实现仍有未开放/待支持范围；根许可是 NVIDIA Source Code License for DiffRL，不写成 MIT |
| [WMP](../upstream/research_20261109/WMP/README.md) | [模型](../upstream/research_20261109/WMP/dreamer/models.py)、[策略接口](../upstream/research_20261109/WMP/rsl_rl/modules/actor_critic_wmp.py) | 根许可 Apache-2.0，组件许可另存；README 原训练配置约需 23 GB 显存，不能照搬到当前 8 GB GPU |
| [DayDreamer](../upstream/research_20261109/daydreamer/README.md) | 先读 README 的数据/机器人接口组织 | 作者实现；无统一根许可，组件许可保留，整体标 unknown |

它们包含不同年代的 Isaac Gym、Raisim、DFlex、JAX 等依赖。下载的目的首先是阅读算法结构；不能照抄安装命令去升级、卸载已有 mjlab/Isaac 环境。用户当前未要求算法实现。

## 3. 看完一篇，留下什么

每篇写半页即可：输入/输出是什么；优化哪个损失；训练和部署各能看哪些信息；论文实验验证了什么；哪一项缺陷与你的轮腿有关。画出“数据 → 模型 → 决策 → 新数据”的箭头，区分是在模型中训练策略，还是部署时用模型规划。

选题确定后才做最小复现。A 优先读 PETS、RMA、近期隐变量适应、TD-MPC2；B 优先读 SuperMimic、MBPO、DMO；C 优先读 WMP、DreamWaQ。DreamerV3 用来建立完整概念，DayDreamer 用于实机闭环阅读，不要求同时跑通所有框架。

## 4. 来源与下载验收

- [完整资料清单](reading_manifest_20261109.json)：12 篇 PDF 的版本、SHA256、页数；8 个 Git checkout 的 commit/tree、归属和许可。
- [结构化来源](sources.json)、[BibTeX](references.bib)、[第三方边界](../THIRD_PARTY.md)。
- [第一批下载记录](../runs/m00_environment/literature_fetch_20261109_124550_330884/results.json)：10 篇论文、7 个代码仓库。
- [补充下载记录](../runs/m00_environment/literature_fetch_20261109_124643_581871/results.json)：WMP、近期隐变量适应论文、WMP 代码。

验证：PDF 文件头、pdfinfo 可解析、可提取正文、SHA256；Git commit/tree 固定，tracked checkout 干净。未验证算法能运行、未报告任何训练性能。没有另行获取外部 checkpoint、Git LFS 大文件和 submodule；仓库已跟踪的原始文件照原样保留。
