# 按作者原结构适配的完整轮腿站立控制

本版本直接以你原仓库的 `LQR_six` 文件为起点，保留 `RobotSensor → VMC → StateEstimator → LQRController → RobotController` 调用流程、Leg 数据结构、六状态顺序、支撑 PID 和左右 VMC 输出。前面的 E001 是另一套控制实现的历史版本，本目录是这次按原代码习惯整理的新交付。2026-11-09 追加了 MuJoCo 可视化、离屏 mp4 录像和键盘驾驶，控制链本身未改。

## 固定文件格式

```text
E002_original_controller/
  controller.py              # 最外层唯一启动文件；主循环可直接顺着读
  LQR_six/                   # 其余 Python 文件全部在此
    get_sensor.py            # 原传感器接口，改模型/关节/IMU 名称
    write_sensor.py          # 原电机接口，改 actuator 名称/力矩限幅
    leg_data.py              # 原 Leg 数据结构
    mymath.py                # 原差分、积分、PID 工具
    vmc.py                   # 同样的 VMC 接口；适配当前完整闭链几何
    get_state.py             # 原六状态接口，修正轮径/方向/轮架补偿
    lqr_controller.py        # 六状态 A/B/K、支撑 PID、左右控制函数
    mympc.py                 # 原 lqr+mpc 分支的 MPC，保留 0.2 叠加方式
    render_stand.py          # 新增：相机预设、离屏 mp4 录像、viewer 相机
    teleop.py                # 新增：键盘速度/转向/腿长目标（只改 Leg.target）
    simulation.py            # 单独放日志/验收，不参与反馈控制
    lqr.py                   # 原旧模型符号推导参考，启动时不导入
    vofa.py                  # 原工具保留，默认不发送 UDP
    keyboardInput.py         # 原工具保留，默认不监听键盘
  assets/                    # 两份正式完整闭链 XML、24 STL、manifest
  standing_pose.json         # 模型对应的静态 reset 姿态与支撑力矩
  requirements.txt
  REPORT.md                  # 本轮修改、真实验证和限制
  SOURCE_MANIFEST.json       # 源 commit、逐文件哈希/归属
  SOURCE_CHANGES.patch       # 相对你原文件的具体差异
  outputs/                   # 本目录各次新运行；不写回旧 runs
```

没有向父工程添加 sys.path，没有要求安装本项目包，没有软链接或自动同步。资产随目录携带，默认不会下载东西。旧工程和原始 checkout 保留原位。

## 运行

在本目录中使用现有解释器：

```bash
# 默认 10 s 站立，输出到 outputs/<新目录>
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py

# 打开桌面窗口观看（本轮已在真实桌面确认）
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --viewer --duration 30
# 仿真结束后保持窗口，直到手动关闭
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --viewer --duration 30 --hold

# 无桌面/想留证据：离屏录 mp4（默认 960x540@30fps，写到本次 outputs/）
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --video
# 腿部侧视、指定文件、关闭 HUD
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py \
  --camera side --video /tmp/stand_side.mp4 --video-size 1280x720
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --video --no-hud

# 键盘驾驶（必须先点一下窗口让它获得焦点；到时结束，关闭窗口可提前结束）
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py \
  --viewer --drive --duration 120

# 前后倾、侧倾、推力；角度单位 rad，时长单位 s
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --case pitch-plus --duration 30
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --case push --duration 30
# 作者 LQR+0.2*MPC 形式，及等效弹簧版本
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --control lqr-mpc --case push
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py --variant gas
```

也能从其他工作目录用入口绝对路径启动。依赖已有 MuJoCo/NumPy/SciPy；不装包、不升级环境、不运行训练。`--video` 另需系统 `ffmpeg`（编码 mp4）和 Pillow（HUD，缺失时自动只录画面）；`--viewer` 需要可用桌面。保留的 `lqr.py` 若单独学习需 SymPy，`keyboardInput.py` 若另外启用需 pynput；默认站立不导入它们，也不导入 VOFA。

`pitch-plus/minus` 为初始 ±0.03 rad；`roll` 为 +0.03 rad；`push` 在第 5–5.2 秒施加世界 +X 方向 10 N。输出力矩按名字映射：左腿输出取负、右腿取正沿用作者写法；当前左轮正向/右轮反向对应同一前进方向。腿 ±30 N·m、轮 ±5 N·m 是仿真 XML 上限，不是硬件额定值。

## 相机、录像和键盘驾驶

- 相机预设：`--camera iso`（默认，斜前视）、`side`（侧面看腿）、`front`（正面）、`legs`（腿部特写）。窗口和录像共用。录像相机默认固定机位，便于看漂移；加 `--follow` 可让机位水平跟随基座。
- 录像按仿真时间取帧，不改变物理；mp4 旁边会写同名 `.json`（分辨率、帧数、相机、时长）和首/末帧 `_t000.png` / `_final.png`。HUD 在画面底部，显示时间、roll/pitch、腿长 `L0`、基座高度和 `STANDING` 状态。
- 键盘命令是**锁存**的（MuJoCo 的 key_callback 只转发按下事件）：

  | 键 | 作用 |
  |---|---|
  | `W` / `S` | 前进速度 +0.1 / −0.1 m/s（±0.4，按 0.5 m/s² 平滑加减速） |
  | `A` / `D` | 左 / 右转 0.2 rad/s（速率控制；松开后只保留阻尼，不锁死绝对航向） |
  | `Q` / `E` | 腿长 −0.02 / +0.02 m（0.22–0.30） |
  | `X` | 停车并停止转向 |
  | `R` | 复位速度、转向和腿长 |

  窗口左/右上角有按键提示，左下角显示当前速度/转向/腿长；控制台也打印每一步。
- 窗口模式下 `data.xfrc_applied` 不再每步清零，所以 MuJoCo 自己的 Ctrl+拖动（扰动）现在能推动机器人；推 `--case push` 仍按原脚本施加世界 +X 10 N。
- 已验证的运动范围（10–12 s，未倒）：前进 0.3/0.4 m/s；原地 0.2 rad/s 转约 10 s；0.3 m/s + 0.2 rad/s 同时进行；腿长 0.22 / 0.30 m。**原地 0.3 rad/s 及以上会失稳**，转完立即用绝对航向保持再制动也会失稳，所以当前用速率控制、松键后自然阻尼。
- 被动 viewer 的 GUI 渲染会让仿真比实时慢（本机约 2×，10 s 仿真约 20 s 墙钟；窗口越大越慢）。键盘命令按仿真时间生效，看起来会偏“慢动作”；需要精确时长/速度证据请用 `--video` 或无窗口运行。

## 看哪些改动

先看 `controller.py` 的六电机输出，再看 `get_sensor.py`/`write_sensor.py` 的名字映射。质量/限幅/六状态矩阵在 `lqr_controller.py`，轮径 0.06 m 和髋偏置在 `get_state.py`。`mymath.py` 保持原样，`leg_data.py` 只改相对 import。`mympc.py` 取自你原 lqr+mpc 分支，保留原无约束预测后裁剪首步的实现，不把它称为全预测序列约束 QP。新增的 `render_stand.py`/`teleop.py` 只读状态或写 `Leg.target`，不碰 `data.ctrl`。

**不能只改四个杆长的地方是 VMC**：当前完整模型有独立后支路和四处闭环，与原 chuan 五杆几何不同。`vmc.py` 只负责消去被动坐标、计算 `[F0,Tp] → 两电机力矩`，不把机器人换成串联简化，也不使用前版 37 维反馈。矩阵 K 已按当前模型的六状态重新标定；改变模型参数后不能假定该 K 仍有效。

姿态也有一个实际差异：此 XML 的 `orientation` framequat/body 是惯性主轴，直接使用会把直立判成倾倒；传感器接口改读机身坐标 xquat，gyro/acc 保持 IMU site 定义。代码中有注释，原错误与结果记录在 REPORT.md。

这仍是仿真控制：VMC 读取完整机构的 Jacobian 和速度；机身姿态也由仿真提供，不能说已经解决实机状态估计。没有倒地起身、行走或训练策略的验收。结果、复制检查及未验证项见 [REPORT.md](REPORT.md)。
