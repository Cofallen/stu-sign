# upstream：外部代码的原始参考区

这是别人项目的原始 checkout，供阅读、追溯模型来源和对照算法。保持原始文件与许可，不在这里日常开发毕设。

| 位置 | 用途 |
|---|---|
| [fudan_rl_wheel_leg](fudan_rl_wheel_leg/README.md) | 用户指定机器人来源；正式资产从其原生并联模型派生 |
| [Leg_simulator_mujoco](Leg_simulator_mujoco/README.md) | 用户原 LQR/VMC/MPC 只读参考；main checkout，另读固定分支对象；不使用其机器人参数 |
| `research_20261109/` | 下载的 8 份世界模型/适应参考实现，入口见[资料导读](../references/READING_GUIDE.md) |

正式机器人实际使用文件在 [assets](../assets/README.md)。参考仓库中附带的其他机器人仅原样保存，不自动成为毕设模型。

需要借鉴/改编时，将本项目实现放入 src/scripts，登记引用、许可和改动范围。作者实现、第三方复现、独立实现分开说明。不要照着旧仓库 README 安装命令修改现有环境；归档不代表已安装、执行或复现。

源码版本/许可：[sources.json](../references/sources.json)、[资料清单](../references/reading_manifest_20261109.json)、[THIRD_PARTY](../THIRD_PARTY.md)。
