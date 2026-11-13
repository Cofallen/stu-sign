# 来源与归属

- 完整无气弹簧 XML 与网格来自 E002 中的正式 infantry_parallel 派生资产，原来源 `yly-true/fudan_rl_wheel_leg` commit `8204e853dfd2ed06d85a322e1a998c3d20a3be2c`。机器人资产许可仍为 unknown。原资产 manifest 保留，E003 实际载荷哈希见 `ASSET_SOURCES.json`。
- 复用了 E002 的驱动/传感器命名、前进方向、腿长几何定义和静力配平的初始姿态。E003 的 manager-based 任务与运行入口为本项目实现。PPO 使用本机 mjlab 和 RSL-RL 提供的实现。

- 可选平衡辅助的 `balance_gain.npz` 来自本项目 M02 无气弹簧 LQR 运行 `lqr_no-gas_20261109_135714_434732/design.npz`。SHA-256 为 `e69938f1d39274f569889b74148a0f20bf23d2850e78f1f2ca22ae92e09f7de7`。使用已有线性化增益与平衡点，E003 增加车体坐标速度指令和 PPO 残差。
