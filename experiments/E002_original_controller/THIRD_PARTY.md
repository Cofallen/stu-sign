# 来源与归属

本目录是对用户原实现的适配，不能再称为“只参考思想、未复制控制源码”。

- `Cofallen/Leg_simulator_mujoco` main commit `b0f14e8ee9331d572738885691389742028cb796`：controller、get_sensor、write_sensor、leg_data、mymath、get_state、vmc、lqr_controller、lqr、vofa、keyboardInput 为原文件基础；mympc 取自 lqr+mpc commit `f8039cde95b8c2a9150c6016621ea928c9215c36`。保留作者文件名/接口与现有声明，逐文件源 SHA256 和改动见 SOURCE_MANIFEST.json / SOURCE_CHANGES.patch。未发现仓库级 LICENSE，许可记录为 unknown，不自行补 MIT/Apache；用户明确授权本地适配其代码。
- 机器人为主工程正式 infantry_parallel 派生资产，原来源 `yly-true/fudan_rl_wheel_leg` commit `8204e853dfd2ed06d85a322e1a998c3d20a3be2c`；全部 XML、网格和原 manifest 原样复制。模型许可仍 unknown。资产内 README 的来源描述与该 manifest 一起保留；没有复制用户原 chuan 机器人来代替指定完整闭链。
- 本次新增 simulation.py 日志/验收、模型命名/坐标适配、完整机构 VMC Jacobian 与六状态矩阵标定属于本项目适配。2026-11-09 追加的 render_stand.py（相机/离屏录像）与 teleop.py（键盘目标层）也是本项目原创增补，不含上游代码；它们分别使用本机 Pillow 与系统 ffmpeg，未把第三方二进制复制进本目录。没有使用 E001 的 37 维反馈；静态 reset 姿态来自其已记录模型平衡点，只用于初始化，不参与每步反馈。
- 保留原 VOFA/键盘/符号推导文件便于对照，默认不启用网络、键盘或旧模型求解器。原 lqr.py 中旧参数明确标作历史参考；当前实际 A/B/K 在 lqr_controller.py。

没有新论文复现或对外发布。现有环境的库按其分发条款使用；原创增补代码的发布许可证未选择。

本轮腿长拟合与腾空收腿：当前lqr.py为本项目完整闭链数值标定/三次拟合，参考本项目前期author_probe_v3.py标定方法；不冒称作者原符号推导。lqr_controller继续改编作者六状态/PID结构；jump.py和gain_schedule.json为本项目独立新增。未引入新外部源码/许可证；原仓库许可证unknown边界不变。旧符号参考与失败试算在outputs/length_gain_20261109_231153_464420保留。

2026-11-10：parameters.py、lqr_model.py拆分为本项目本地调参与高级标定模块；VMC随朝向Jacobian/叠加/倒地策略为本项目修正。原mymath.py本轮增加可选实测D输入，不再逐字节等同上游；默认两参PID调用形式保留，原作者归属仍保留。无新外部代码/许可证，license unknown边界不变。

2026-11-10 独立training_terrain.xml为本项目直接编写的原生MuJoCo box台阶/17°坡，不下载外部网格，无新增外部许可来源；机器人原XML与许可边界不变。落地反馈/制动为本项目修正，基础拟合K不变，额外制动模式不冒称新最优LQR。

2026-11-13 自起状态机参考用户随任务提供的嵌入式 `StateSelect.c` 和 `RecoveryControl.c` 片段。采用分阶段判定、髋轴控制和 LQR 交接的思路，使用 E002 几何、接触与 IMU 重新实现。片段未随实验复制，版本和发布许可未提供，许可记为 unknown。
