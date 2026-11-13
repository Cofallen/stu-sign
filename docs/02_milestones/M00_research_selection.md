# M00 补充：选题调研与离线资料归档

日期：2026-11-09。状态：**资料归档通过，选题待确定**。这是研究准备记录，不代表模型训练或实物新增验收通过。

## 目标与输入

用户希望以世界模型技术研发为求职方向，结合当前学习进度和已有轮腿实物选择毕业设计题目，并下载论文/代码便于阅读。最后补充要求以几个候选选题为主，不写代码。

只读参考：当前 README/overview/M01 完整闭链报告、`/home/cofallen/Code/IsaacTest/cartpole/rl-study/` 中学习路线与实验台账、Cartpole sim2sim 脚本、`references/resume.pdf`。简历采用文字提取和单页渲染核对；未修改简历、旧学习工程、控制固件或运行环境。

用户新增信息（自述，未现场复验）：2027 年 5 月；每周 20–30 h；STM32H723 与达妙 MC02；实物已能 LQR 站立/行走；可增加 NVIDIA 推理设备，型号原文“xvwria 2”待核对。相机、采样率、通信延迟和实时性能未确认。

## 当前交付

- [三个候选与条件路线](../01_project/TOPIC_CANDIDATES_20261109.md)：A 自适应世界模型/预测控制优先，B 可微世界模型/策略学习，C 多模态预测感知。未替用户确定最终题目。
- [阅读导读](../../references/READING_GUIDE.md)：按候选排序的论文与代码入口。
- [归档清单](../../references/reading_manifest_20261109.json)：12 篇 PDF、8 份源码，逐项记录作者、版本、来源、哈希/commit、许可与阅读范围。
- 来源同步进入 `THIRD_PARTY.md`、`references/sources.json`、`references/references.bib`。根许可证快照位于 `references/licenses/reading_20261109_*`，组件许可仍在原 checkout。

论文路径：`references/papers/research_20261109/`。源码路径：`upstream/research_20261109/`。没有采用其中的其他机器人替换正式闭链模型。没有安装依赖或执行第三方代码；未另行拉取外部权重、数据集、Git LFS 与 submodule，仓库已跟踪文件原样保留。

## 实际获取与检查

资料获取工具为 `scripts/m00_environment/fetch_research_reading.py`，仅负责公开资料下载、元数据记录和静态验证；它在用户进一步要求“不写代码”之前创建，不是模型/控制/训练算法实现。其默认行为拒绝覆盖已有目标。

实际命令：

```bash
python scripts/m00_environment/fetch_research_reading.py
python scripts/m00_environment/fetch_research_reading.py references/reading_catalog_supplement.json
```

本轮已完成，无需重复运行。解释器、cwd、命令、目录和退出状态见各 run 的 invocation/results/log；种子不适用。

| 记录 | 内容 | 结果 |
|---|---|---|
| [第一批](../../runs/m00_environment/literature_fetch_20261109_124550_330884/results.json) | 10 篇论文、7 个仓库 | 所有条目退出 0 |
| [补充批](../../runs/m00_environment/literature_fetch_20261109_124643_581871/results.json) | 2 篇论文、1 个仓库 | 所有条目退出 0 |

首次普通沙箱联网探测出现 `curl: (6) Could not resolve host: arxiv.org`；在用户已授权下载的范围内，经网络权限执行后下载成功。没有将网络边界误判为环境损坏，也没有修改代理或系统配置。该最初探测输出来自本轮工具返回，未伪造为 run 内原始日志。

验收标准与结果：12 个文件具备 PDF 签名，pdfinfo 可解析且可提取文本；SHA256 与清单一致。8 份代码可读取固定 HEAD 和 tree，tracked working tree 均干净。根许可逐项读取；DayDreamer 的统一根许可仍 unknown，DMO 是自定义 DiffRL 许可。来源页面与代码归属有差异的条目已明确标注。

## 范围与下一步

没有运行 GPU/Isaac/轮腿训练，没有新的机器人成功率或性能结论。只读方法段与部分代码入口，不构成完整论文复现或穷尽查新。原先闭链约束建议降为备选专题；当前首选依据是用户已有 LQR 实物与控制基础，而不是模型拓扑本身。

下一项最小任务是选定 A/B/C 之一并明确一句研究问题。定题后再确定数据接口和预注册验收，不自动开展算法实现或长训练。
