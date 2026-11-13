# 键盘控制轮腿机器人

关闭旧仿真窗口后，从本目录运行：

```bash
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py
```

点击窗口获得焦点。**W/S、A/D、Q/E 都改成按住生效；松开后不再锁存。** 关闭窗口结束；失去窗口焦点同样清空按住键。现有进程不会热更新，改代码后要重启。

| 按键 | 按住时 | 松开时 |
|---|---|---|
| W / S | 机器人前进 / 后退，目标速度 ±1.2 m/s | 目标归零、制动，停稳后位置保持 |
| A / D | 左 / 右转，目标 0.2 rad/s | 停止转向命令，保留阻尼 |
| Q / E | 连续缩腿 / 伸腿，0.12 m/s，范围 0.18–0.34 m | 保持松开时的腿长目标 |
| J | 停稳后按一次，执行下蹲→蹬地→落地 | 不重复触发 |
| X | 停车；持续键需先松开再按才重新启动 | — |
| R | 恢复默认速度/转向/腿长目标，不瞬移模型 | — |

W 的方向已按本模型外观的前方修正：初始姿态中对应 **模型 −X**，S 对应 +X。只改驾驶目标方向，不翻转已经配套的 IMU、轮编码器或电机力矩轴。

速度上限由 0.4 提至 **1.2 m/s（3 倍）**，目标加速度也由 0.5 提至 1.5 m/s²。松开立即进入制动，不会继续保持原速度；真实物理速度需要减速，不能瞬间清零。短测中从约 1.18 m/s 降至 0.05 m/s 以下约需 1.5 s。所有速度/时长均按**仿真时间**，窗口渲染若慢于实时，墙钟体验也会变慢，左下 RUNNING 时间可核对。避免同时开多个旧仿真窗口。

## 参数：统一在 Teleop 构造函数

位置：`LQR_six/teleop.py → Teleop.__init__`。启动文件实例化时也可用命名参数覆盖，例如：

```python
teleop = Teleop(leg_L, leg_R, yaw_kp=lqr.pid_yaw.kp,
                speed_max=1.2, forward_sign=-1.,
                l0_min=.18, l0_max=.34, l0_rate=.12,
                jump_crouch=.20, jump_extend=.34, jump_force=200.)
```

| 参数 | 当前值 | 单位 / 含义 |
|---|---:|---|
| `speed_max` | 1.2 | m/s，W/S 目标速度幅值 |
| `forward_sign` | -1 | W 到模型纵向速度的符号；模型版本换方向时改这里 |
| `accel` | 1.5 | m/s²，目标速度上升斜率 |
| `brake_accel` | 3.0 | m/s²，松开/换向时目标速度斜率，不等于实际机身瞬时减速度 |
| `yaw_rate_max` | 0.2 | rad/s，A/D 转向目标 |
| `yaw_kp` | 5.0 | 使用 LQRController 的 yaw P 增益，供速率目标换算 |
| `l0_min`, `l0_max` | 0.18, 0.34 | m，腿长目标边界（原 0.22–0.30） |
| `l0_rate` | 0.12 | m/s，Q/E 按住期间的目标变化率 |
| `jump_crouch` | 0.20 | m，起跳前下蹲目标（原 0.24） |
| `jump_extend` | 0.34 | m，蹬地目标（原 0.30） |
| `jump_force` | 200 | N，每侧额外 VMC 支撑目标（原 140） |
| `jump_crouch_time` | 0.40 | s，下蹲阶段时长 |
| `jump_thrust_time` | 0.20 | s，蹬地阶段时长（原 0.15） |
| `jump_total_time` | 1.60 | s，动作结束/恢复时间，相对起跳动作开始 |

`l0_home` 从静态 reset 标定读取，约 0.27792 m。跳跃仍经六个真实电机和 VMC 输出；腿 ±30 N·m、轮 ±5 N·m 没有提高。没有向基座施加向上外力、改 qpos/qvel 或解开闭链。参数增大后应复验，尤其不要把“增大 jump_force”理解为力矩可无上限增大。

## 函数接口

| 文件 / 类 | 接口 | 输入与输出 |
|---|---|---|
| `keyboardInput.py / KeyboardInput` | `on_key(keycode)` | 自有 viewer 的按下回调；绑定该窗口，只排队 J/X/R |
| 同上 | `poll() → (pressed, actions)` | 返回实际按住的 W/S/A/D/Q/E 集合与一次性动作列表；失焦返回空集合 |
| 同上 | `close()` | 关闭本地 X11 连接 |
| `teleop.py / Teleop` | `set_pressed(keys)` | 输入当前键集合；空集合表示松开，不是锁存命令 |
| 同上 | `on_key('j'/'x'/'r')` | 单次动作；J 要求方向键已松开且速度接近零 |
| 同上 | `update(imu, dt)` | 每物理步生成 `Leg.target` 的 `s/dot_s/l0/yaw/jump_force`；dt 单位 s |
| 同上 | `observe(model, data, dt)` | 只记录实际高度、向上速度和无接触时长，不写物理状态 |
| 同上 | `status()` | 当前按住键、速度/腿长、动作阶段和事件/跳跃记录 |
| `lqr_controller.py / LQRController` | `control_left/right(leg, imu)` | 六状态反馈 + 支撑/姿态修正 + VMC，返回腿电机力矩与轮力矩 |
| `write_sensor.py / RobotController` | `set_actuator(name, torque)` | 命名映射和 XML 力矩限幅，写入 `data.ctrl` |

主循环中两类输入分开处理，按键查询每 10 个物理步一次（约 100 Hz），目标与 VMC/支撑控制每 1 ms 更新：

```python
pressed, actions = keyboard.poll()
teleop.set_pressed(pressed)       # 即使为空集合也必须传入，才能处理释放
for action in actions:
    teleop.on_key(action)         # J/X/R 只触发一次
teleop.observe(model, data, dt)   # 记录，不改物理
teleop.update(imu, dt)            # 连续腿长/速度目标和动作阶段
```

MuJoCo passive viewer 的 `key_callback` 没有释放事件，所以不能再拿它累加 W/S 或腿长。本实现用系统已有 **libX11 的当前键位查询**，只在绑定的 MuJoCo 窗口有焦点时生效，不用自动重复按键猜测释放，不监听/记录其它应用文本。无需安装 pynput。此输入后端目前仅验证本机 Linux/X11，未验证原生 Wayland/Windows。

## 代码架构与修改建议

```mermaid
flowchart TD
  K[KeyboardInput 当前键位/单次动作] --> T[Teleop 按住目标/制动/跳跃阶段]
  T --> G[Leg.target]
  M[MuJoCo model/data] --> S[RobotSensor + VMC 几何 + StateEstimator]
  S --> L[Leg.state / Leg.vmc]
  G --> C[LQRController 左右控制]
  L --> C
  C --> W[RobotController 限幅/映射]
  W --> P[mj_step 物理积分]
  P --> M
```

- 最外层 `controller.py` 只负责组装、循环和窗口，其余 Python 继续放 `LQR_six/`；不为调参重排工程。
- **交互参数和目标规则改 Teleop；键位/焦点/释放改 KeyboardInput。** 不把键盘处理塞进 LQR 或直接写电机。
- **控制增益和支撑修正改 LQRController；完整机构 Jacobian 改 VMC。** 模型质量、惯量、轮径、几何变动需重新标定/验证 K，不通过翻转输入方向掩盖模型错误。
- 轮子 mesh、棋盘和显示设置放 `render_stand.py`；日志/验收放 `simulation.py`。显示不参与控制，测试脚本不代替真实按键验证。

## 本轮实测与输出

- 无头实际控制循环，W/S 分别到约 −1.18/+1.18 m/s，松开后制动停车；各 12 s 未倒。
- 腿长目标从 0.18 连续扫到 0.34 m，松开后保持；实际腿长及边界误差见 [REPORT.md](REPORT.md)。这是静止时分项测试，不是所有高度与最高速度组合都已验证。
- 增强跳跃：机身从约 0.24924 m 到 0.47061 m，上升约 **22.1 cm**（前版约 12.7 cm），双轮腾空约 **0.42 s** 后稳定落地。
- 本机真实 GLFW/XTest **按下和释放**已验证，日志含 `w → released → s → released`、Q/E 按下/释放和 J；不是只模拟 `on_key`。完整范围、失败和证据见 [REPORT.md](REPORT.md)。

每次输出在自身 `outputs/<新运行ID>/`，包含命令/解释器/版本、源码快照、真实键状态事件、目标/状态/力矩 trace、实际轮角/接触/机身位置 `motion.npz`、结果与退出码。数值检查可用 `--headless --duration 10`；其它旧录像/相机参数保留兼容，不是日常操作必需。

源轮 STL/正式 XML/闭链/地面碰撞与摩擦保持原样，棋盘只属运行显示。依赖已有 Python/MuJoCo/NumPy/SciPy 和 GUI 的系统 libX11，版本见 requirements.txt；不自动安装或修改环境。原作者来源/差异见 SOURCE_MANIFEST.json、SOURCE_CHANGES.patch、THIRD_PARTY.md。

当前仍是平地仿真：空中自由轮速积分不等于机身位移；没有跨机、实物、硬实时、长期反复跳跃、任意组合或学习训练验收。
