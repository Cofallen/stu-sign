# M05 E004 平地可微世界模型短训

2026-11-13。已有 mjlab Conda，RTX 4060，完整闭链无气弹簧模型，128 环境，种子 42。使用 E003 纯 PPO 热启动。8 轮合计采集 49,152 个控制转移，动力学拟合每轮 120 次，策略每轮沿 5 步想象轨迹更新一次。第一轮到第八轮留出环境单步混合单位 MAE 从 0.0128 到 0.00171。各轮 actor 梯度有限且非零。[训练指标](../../experiments/E004_supermimic/outputs/train_20261113_164401_385717/metrics.json)

原生 MuJoCo CPU 30 秒混合指令回放通过，分段稳态前后速度 MAE 0.0219 m/s，偏航速度 MAE 0.0264 rad/s。桌面回放也正常结束。[无头指标](../../experiments/E004_supermimic/outputs/native_deploy_20261113_164413_796795/summary.json) · [桌面运行](../../experiments/E004_supermimic/outputs/native_deploy_20261113_164805_200219/run.json)

独立适配覆盖平地命令任务，使用模型内奖励反传，省略原论文的终止预测、价值 bootstrap 与参考动作数据。单次回放还不足以判定训练后比 E003 更好。下一项是用固定的多步留出序列核对世界模型预测误差，并按同一评测协议对比 E003 和 E004。 [运行说明](../../experiments/E004_supermimic/README.md) · [教程](../../experiments/E004_supermimic/TUTORIAL.md)

2026-11-13：原生 sim2sim 默认支持键盘，默认 XML 恢复轮子 STL 外观并使用方格地面。[操作和回放证据](../03_learning/M05_02_sim2sim轮子方格与键盘.md)。

2026-11-13：E003/E004 桌面 sim2sim 将画面刷新与物理循环分开，两者均实测 8 秒仿真对应约 8 秒墙钟，轨迹与无头一致。[复验](../03_learning/M05_02_sim2sim轮子方格与键盘.md)。
