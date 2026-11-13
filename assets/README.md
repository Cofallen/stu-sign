# assets：本项目实际使用的模型资源

当前正式机器人是 [infantry_parallel](robots/infantry_parallel/README.md)，两份完整闭链 XML、网格与 manifest 位于同目录。保留全部独立连杆、转动关节与闭合销接。

[infantry_v4_standard_simplified](robots/infantry_v4_standard_simplified/README.md) 已撤回正式模型地位，仅保留历史对照，不能因名称含 standard 就误当当前版本。

原始模型来源在 upstream；生成入口在 scripts；检查结果在 runs；为什么采用该模型看[M01 报告](../docs/02_milestones/M01_parallel_linkage.md)。视觉与碰撞分开，不混用不同版本质量/惯量，生成方法与来源哈希随资产保存。
