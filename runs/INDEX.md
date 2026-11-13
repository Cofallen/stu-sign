# 运行证据索引

更新：2026-11-09。时间均为 Asia/Shanghai。目录迁移后保留历史记录内部的旧命令；当前可执行命令以根 README 为准。

## 当前验收采用的记录

**当前正式模型改为完整并联机构；以下旧串联结果不作为完整闭链的验收依据。** 当前记录：[动态检查](m01_model/parallel_check_20261109_004237_385074/results.json)、[14 个关节转动范围](m01_model/parallel_check_20261109_004237_385074/joint_mapping.csv)、[源几何/惯量与 4 个销接相对转动复核](m01_model/parallel_integrity_20261109_004606_083292/results.json)、[完整机构视频](m01_model/parallel_check_20261109_004237_385074/parallel_retraction.mp4)、[收缩截图](m01_model/parallel_check_20261109_004237_385074/retracted.png)。无录像的首次完整动态检查在 `parallel_check_20261109_004200_084359`，保留。

## 当前作者代码适配（E002）

用户最新要求保留原代码结构；当前入口为 [controller.py + LQR_six](../experiments/E002_original_controller/README.md)。本轮新证据保存在小项目自身 outputs，不移动旧 runs：[八项后续评测](../experiments/E002_original_controller/outputs/suite_20261109_145715_480103/results.json)、[独立复制](../experiments/E002_original_controller/outputs/copy_check_20261109_150018_774770/results.json)、[报告](../experiments/E002_original_controller/REPORT.md)。原传感器姿态误读失败与原型摘要保留在该目录；不将下面旧版 18 个案例作为 E002 实测结果。

## M02 完整闭链站立控制

采用 [汇总指标](m02_standing/report_20261109_140332_146596/summary.json)、[评测曲线](m02_standing/report_20261109_140332_146596/standing_evaluation.png)。18 个明确案例各 30 s，通过范围和限制见 [M02 报告](../docs/02_milestones/M02_parallel_standing.md)，不是随机统计成功率。

| 记录 | 用途/结论 |
|---|---|
| [LQR 无弹簧](m02_standing/lqr_no-gas_20261109_135714_434732/results.json) | nominal、±0.03 rad pitch、0.03 rad roll、10 N×0.2 s 推力，各 30 s |
| [MPC 无弹簧](m02_standing/mpc_no-gas_20261109_135947_125217/results.json) | 同五案例加 0.1 rad pitch；约束实际激活 2 次；最慢 41.29 ms，实时性未通过 |
| [LQR 等效弹簧](m02_standing/lqr_gas_20261109_140138_434688/results.json) | 五案例各 30 s，独立重求平衡和增益；弹簧未标定 |
| [MPC 等效弹簧](m02_standing/mpc_gas_20261109_140245_737252/results.json) | nominal/push 两案例各 30 s，不推广为所有倾角验证 |
| [离屏推力录像](m02_standing/lqr_no-gas_20261109_140329_963534/push.mp4) | 单独 10 s 实际 LQR 推力积分，通过；640×480，101 帧，首尾画面/ffprobe 已核查 |
| [交付静态检查](m02_standing/delivery_check_20261109_140716_441221/results.json) | 语法、163 个文档链接、来源 JSON、模型哈希、上游 clean 与录像格式 |
| [数学与资产检查](m02_standing/design_check_20261109_135930_484209/results.json) | MPC/LQR 小误差等价、全序列边界/投影梯度、正式 XML 哈希未变 |
| [源阅读/早期原型](m02_standing/source_and_probes_20261109_135708_139243/metadata.json) | 固定 main/lqr+mpc/mpc commit；相对差分自由落体原型被拒绝，源码和日志保留 |
| [正式入口第一次失败](m02_standing/lqr_no-gas_20261109_135634_476468/run.log) | 静态优化无接触，不满足门槛，退出 2；不计正式通过案例 |
| [LQR 最小冒烟](m02_standing/lqr_no-gas_20261109_135650_661227/results.json) | nominal 10 s |
| [MPC 最小冒烟](m02_standing/mpc_no-gas_20261109_135828_283303/results.json) | nominal/pitch-plus 各 10 s |
| [等效弹簧最小冒烟](m02_standing/lqr_gas_20261109_135902_736665/results.json) | nominal 10 s |

## 历史串联简化模型记录

气弹簧变体当前验收：[收缩与阻尼结果](m01_model/gas_spring_check_20261109_002615_952147/results.json)、[正常重力冒烟](m01_model/gas_spring_check_20261109_002615_952147/free_root_smoke.json)、[左右对照视频](m01_model/gas_spring_check_20261109_002615_952147/retract_extend_comparison.mp4)、[收缩截图](m01_model/gas_spring_check_20261109_002615_952147/retracted.png)。此处明确固定基座、零重力诊断，不代表平衡成绩；以下基础模型记录保留其原有范围。

| 阶段/目的 | 证据 | 结论和边界 |
|---|---|---|
| M00 整理后重验导入 | [results.json](m00_environment/imports_20261109_000046_333350/results.json) | 两个解释器的导入与包清单命令均退出 0；未创建 SimulationApp |
| M01 整理后重验模型 | [acceptance.json](m01_model/model_audit_20261109_001430_298256/acceptance.json) | 8 项标准 V4 简化模型 CPU 冒烟门槛通过；不包括并联、自由直立、mjlab GPU、Isaac Lab |
| M01 关节映射 | [joint_mapping.csv](m01_model/model_audit_20261109_001430_298256/joint_mapping.csv) | 6 维动作、名字、地址、轴、限位和源声明上限 |
| M01 详细物理数据 | [physics_checks.json](m01_model/model_audit_20261109_001430_298256/physics_checks.json) | 包括未通过的自由被动静止条件，不能只看通过项 |
| M01 源模型比较 | [urdf_inventory.json](m01_model/model_audit_20261109_001430_298256/urdf_inventory.json)、[upstream_loads.json](m01_model/model_audit_20261109_001430_298256/upstream_loads.json) | 多版本、源 mesh 缺失、非法惯量、原生并联约束残差 |
| M01 软件/源码版本 | [run_metadata.json](m01_model/model_audit_20261109_001430_298256/run_metadata.json) | 解释器、依赖、命令、commit、脚本和模型哈希 |
| M01 接触求解敏感性 | [results.json](m01_model/contact_solver_20261108_235311_344833/results.json) | 保存默认 20 ms 未达门槛与 5 ms/减半步长对照 |
| M01 外观修正后录像 | [passive_contact.mp4](m01_model/model_video_20261109_001408_539483/passive_contact.mp4)、[metadata.json](m01_model/model_video_20261109_001408_539483/metadata.json) | 完整源视觉网格，用户指定白身灰轮；CPU 实际渲染，无策略 |
| M01 图像/状态 | [initial.png](m01_model/model_video_20261109_001408_539483/initial.png)、[final.png](m01_model/model_video_20261109_001408_539483/final.png)、[states.npz](m01_model/model_video_20261109_001408_539483/states.npz) | 完整源网格、指定配色与实际模拟状态 |

外观纠正校验：[visual_check results.json](m01_model/visual_check_20261109_001408_522185/results.json)，7 个 link、1,880,077 个三角面全部逐字节保留；配色符合用户指定参考，250 步动力学对照差为 0。

## 历史与迭代记录

| 记录 | 用途/状态 |
|---|---|
| `m00_environment/imports_20261108_234030_306309` | 整理前导入通过，原始日志保留 |
| `m01_model/model_audit_20261108_235056_985207` | 早期 inventory 在旧版非法数值处异常，未写出结果；目录保留，不能当作已完成运行 |
| `m01_model/model_audit_20261108_235110_336317` | 初版审计；20 ms 接触设置，瞬态穿透约 8.23 mm |
| `m01_model/model_audit_20261108_235218_419039` | 增加静态/瞬态区分和高度几何扫描，仍为旧接触设置 |
| `m01_model/model_audit_20261108_235417_273979` | 5 ms 接触设置；尚无夹具静态接触测试 |
| `m01_model/model_audit_20261108_235540_281023` | 增加夹具静态测试，诊断迭代 |
| `m01_model/model_audit_20261108_235622_886178` | 整理前 8 项 CPU 冒烟通过；被新路径下复验取代为当前入口 |
| `m01_model/model_video_20261108_235448_375655` | 旧碰撞凸包显示，机身/颜色错误，仅保留历史诊断用途 |
| `m01_model/model_audit_20261109_000046_850126` | 目录整理后的旧碰撞显示版物理检查；当前模型使用上方新记录 |
| `m01_model/model_video_20261109_000046_832740` | 整理后的旧凸包/蓝色录像，不作为外观验收 |
| `m01_model/model_video_20261109_001148_390474` | 恢复原网格后的全白单帧预览，未采用最终用户配色 |
| `m01_model/model_audit_20261109_001212_885278` | 全白原网格版本的物理复验，当前配色前的迭代记录 |

历史目录不自动删除，也不把旧 metadata 的命令改写成新路径。迁移路径与移动时哈希见 [folder_migration.json](../docs/01_project/folder_migration.json)。
# 2026-11-09 选题资料获取补充

以下为资料下载与静态验证记录，不是仿真/训练验收，不改变下文历史运行的原始含义。

- [第一批：10 篇论文、7 个代码仓库](m00_environment/literature_fetch_20261109_124550_330884/results.json)
- [补充：2 篇论文、1 个代码仓库](m00_environment/literature_fetch_20261109_124643_581871/results.json)
- [选题调研报告](../docs/02_milestones/M00_research_selection.md)


## 工作区整理记录

[workspace_organization_20261109_134450_155266](m00_environment/workspace_organization_20261109_134450_155266/change_record.json)：文档编辑前快照/哈希与静态验证；无实验数据搬迁，无训练或模型变更。
