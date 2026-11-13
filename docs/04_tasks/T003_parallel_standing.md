# T003 完整并联模型站立控制

- 状态：E002 多角度 CPU 短测完成。+2.0 rad 达到状态 2，LQR 交接失败。负向、侧翻、倒扣和气弹簧自起待继续。
- 负责人：2026-11-13 E002 多角度自起窗口已释放，CPU 短测结束，无设备占用。
- 授权：用户要求参考 Cofallen/Leg_simulator_mujoco 的 LQR/MPC，使用本项目 assets 中完整 parallel 模型实现站立、检验并写报告。
- 范围：新增 src/wheel_leg 控制模块、scripts/m02_standing 入口、独立 runs、M02 报告和学习笔记；本窗口负责合并 overview、关键操作、来源清单和导航。upstream 只读；不改变正式模型、环境或其他任务。
- 设备：CPU MuJoCo 单进程，离屏渲染按需；不使用 GPU 训练或实物。
- 依赖：已阅读 README、使用手册、任务看板、overview、环境手册、CONVENTIONS 和 M01 完整闭链报告。T001/T002 尚未认领；用户最新任务优先，接口实现范围在本卡登记。

## 验收计划（运行前）

先检查源仓库与模型接口，建立自由基座、重力、地面接触下的最小站立控制。默认无气弹簧版先验收，再复验等效弹簧版；不锁基座、不删除闭链、不向被动轴驱动。10 s 最小案例→30 s 正式评测；终止条件为机身高度低于 0.12 m、roll/pitch 超过 0.6 rad 或非有限状态/数值警告。成功要求完整时长、最后 5 s roll/pitch 峰值 <0.05 rad、平面位置漂移 <0.1 m、最大闭环残差 <1e-4 m、输入不超 XML 限幅。留出初始 ±0.03 rad pitch、外力扰动；结果分别报告，不将调参案例当留出结果。

## 工作记录与交接

正在获取只读源仓库并检查 LQR/MPC 状态与参数，尚无站立成功结论。后续以实际 run 更新。

2026-11-09 14:02（Asia/Shanghai）进展：原仓库 main 与 lqr+mpc 分支已按固定 commit 阅读，只读 checkout 未修改。完整 MJCF 原文件/网格未修改。已新增特权状态 LQR 与输入约束 MPC；无弹簧 LQR 5 个、MPC 6 个指定案例各 30 s 均通过（其中 MPC 的 0.1 rad 前倾实际激活约束两次）。MPC 约束求解最大约 41 ms，未满足 10 ms 硬实时要求；默认仍为 LQR。等效弹簧 LQR 正在顺序复验。一次带边界静态平衡求解失败已保留，未放宽验收阈值。正在完成离屏录像与报告；无 GPU/实物占用。

## 最终交接

- 实际修改：新增 `src/wheel_leg/control/standing.py` 与包入口；`scripts/m02_standing/run_standing.py`、`check_design.py`、`summarize_standing.py`；M02 报告、学习笔记、来源与导航/overview/KEY_OPERATIONS。没有改正式模型、网格、环境、wrapper、其他项目或原始上游代码。
- [M02 报告](../02_milestones/M02_parallel_standing.md)保存目标、实现差异、映射、参数、失败、18 个 30 s 案例表与全部复验命令；[学习笔记](../03_learning/M02_01_完整闭链站立与LQR_MPC.md)解释平衡点、37 维特权状态、控制原理、数值差分和未知项。
- 正式证据：[聚合指标](../../runs/m02_standing/report_20261109_140332_146596/summary.json)。LQR 无弹簧 5 案例、MPC 无弹簧 6 案例、LQR 等效弹簧 5 案例、MPC 等效弹簧 2 案例，各 30 s，均达到原阈值。原型失败和正式静态优化失败独立保留，不计通过案例。
- [实际 10 s 推力录像](../../runs/m02_standing/lqr_no-gas_20261109_140329_963534/push.mp4)通过，首尾画面与 ffprobe 已检查；无原生 GUI、GPU/Isaac 或实物验收。
- [交付静态检查](../../runs/m02_standing/delivery_check_20261109_140716_441221/results.json)：6 个 Python 文件语法、163 个本地链接、来源 JSON、正式 XML 哈希与上游 clean 状态均通过。随后仅更新交接状态和链接。
- 未确认：基座/被动轴仿真真值尚无可部署估计；等效气弹簧未标定；MPC 约束激活最大 41.29 ms，超过 10 ms 周期，不视为实时通过；带弹簧 MPC 未覆盖全部倾角案例。无统计泛化率、行走/起身或训练结论。
- 下一项最小任务：认领 T002 后，在同一站立轨迹逐项对齐 IMU/四主动腿轴/轮编码器，明确坐标、速度符号与采样时序，做可部署状态估计对照。若继续 MPC，先固定求解预算并加入延迟测试。没有启动新训练。

## 本轮独立目录整理（用户追加要求）

先完整阅读《项目结构规范.md》；本轮仅在 `experiments/E001_parallel_standing/` 形成独立副本，保留原 `src/scripts/assets/runs` 与其历史元数据。调整本地导入、资源/输出相对位置，携带两份完整闭链 XML、全部引用网格、来源与依赖说明；控制算法/阈值不变。复用本卡，不创建新任务卡。验证将从 `/tmp` 新复制目录之外启动，记录本地导入/模型/输出路径、执行日志、源文件前后哈希与行为对照。本窗口仅追加相关文档导航、overview、KEY_OPERATIONS 与学习笔记，不重排其他目录或修改结构规范。当前未声明复制验证成功。

### 本轮最终交接

- 独立交付：[E001 README](../../experiments/E001_parallel_standing/README.md)、[REPORT](../../experiments/E001_parallel_standing/REPORT.md)。四个本地 Python 文件、两 XML/24 STL/manifest、requirements 和来源清单相邻放置；新结果进本地 outputs，无父工程导入、软链接或自动同步。旧 src/scripts/assets/runs 未改。
- 首次默认 nominal 10 s 通过；将完整运行载荷复制到 `/tmp/parallel_standing_copy_20261109_142701_636332/E001_parallel_standing`，从 `/tmp` 启动四个顺序仿真进程，五案例各 10 s 通过；metrics/qpos/qvel/ctrl 对原轨迹前缀最大差为 0，数学性质检查退出 0。
- [复制证据](../../experiments/E001_parallel_standing/outputs/portability_20261109_142701_636332/results.json)记录命令、解析后的本地模块/XML/output 路径、173 个原文件前后哈希；Python open 审计主动拒绝原工作区访问。strace 被沙箱拒绝，协调器曾误选日志路径，失败分别保留；不声称原生 C 文件访问被完整审计。
- 未确认：跨机器/其他运行环境、独立副本原生 GUI/录像、实物/硬实时、可部署估计未验收。无新训练或长评测。默认仅改成短 nominal，控制算法逐字节不变。
- 后续入口优先使用 E001；如需修改算法明确副本/集成边界，不能默默同步。下一项最小任务仍是可部署观测/采样契约对齐。

最终补充：复制目录内的 summarize_standing.py 已从 `/tmp` 普通启动，成功汇总五案例并输出自身 outputs/report_20261109_143027_896083；没有新仿真。[最终结构静态检查](../../experiments/E001_parallel_standing/outputs/layout_check_20261109_143029_118692/results.json)通过：四个运行脚本语法、本地导入/无硬编码原工作区、打包文件哈希、XML 全网格依赖闭包、173 个原文件保护、112 个文档链接，无软链接。

## 用户纠正：原作者代码优先

E001 只是上一版 37 维控制器的集中副本，不符合用户要求“主要保留原作者代码”的新明确约束。本轮不再以其作为新交付。在 `experiments/E002_original_controller/` 保留作者 RobotSensor/Leg/VMC/StateEstimator/LQRController/RobotController 链路，根 controller.py 启动，其余 Python 统一放 LQR_six，旧目录/证据保留。使用真实模型命名/方向/质量/轮径、六状态矩阵；完整后支路与旧五杆不同，必要的 Jacobian 适配只放 vmc.py。先 10 s CPU 验证，原阈值不变；记录失败与来源。未授权训练或实物。

### 原作者适配最终交接

- 当前交付：[E002](../../experiments/E002_original_controller/README.md)，最外层 controller.py，其余运行/参考 Python 全部在 LQR_six；此前 E001 保留历史。实际使用作者六状态 LQR、支撑 PID/VMC、roll/delta/yaw 与可选 LQR+0.2×MPC，不是给 37 维控制器改文件名。
- 原 mymath/vofa 逐字节保留，Leg 只改 import，其余适配的源 SHA256 与 diff 在 SOURCE_MANIFEST.json/SOURCE_CHANGES.patch；复制源码的归属已更新 THIRD_PARTY 与 sources.json。模型/源 checkout 未改。
- [报告](../../experiments/E002_original_controller/REPORT.md)记录实际八项后续评测（前倾/推力 LQR 各 30 s，另外六项 10 s）、前置两个前倾冒烟、[复制检查](../../experiments/E002_original_controller/outputs/copy_check_20261109_150018_774770/results.json)。全部在独立 outputs 留新记录；姿态 framequat 误读的 t=0 失败与早期原型失败保留。
- 当前旧 src/wheel_leg/control/standing.py 已不在磁盘；复制协调器明确把它列为此前即缺失，未恢复/删除/接管；其它 172 个当前存在的保护文件哈希与历史清单一致。不要据旧任务文字假定该 src 入口仍可用，优先 E002。
- 未验证原生 GUI/录像、其它机器环境、可部署状态估计、硬实时、VOFA/键盘、实物、GPU/Isaac、起身/行走、学习训练或弹簧标定。无新训练。下一项仍是实际 IMU/编码器与六状态/VMC 观测契约核对。

## 追加：MuJoCo 可视化与键盘驾驶（2026-11-09）

- 用户反馈窗口里只看到关节小数点变化、鼠标推不动，要求能用键盘控制并移动。授权范围限 E002 内的可视化和驾驶层；不替换作者六状态 LQR/VMC、不改模型/姿态/站立阈值。
- 实际修改：新增 `LQR_six/render_stand.py`（相机预设、离屏 mp4、viewer 相机）、`LQR_six/teleop.py`（速度/转向/腿长只写 `Leg.target`）；`controller.py` 增加 `--video/--camera/--video-size/--video-fps/--no-hud/--follow/--hold/--drive`、key_callback、覆盖层和窗口外力策略；`simulation.py` 增加驾驶模式验收口径；README/REPORT/requirements 同步。`mymath.py`、`leg_data.py`、`vmc.py`、`lqr_controller.py`、模型和站立阈值未改。
- 已验证：无窗口 10 s `155726` 通过且 max_tilt 与功能前逐值相同；iso/side 两段 960x540、301 帧 mp4（`153438`、`153357`）通过，trace 与功能前 `145716` 逐值一致；桌面 viewer 实际打开并截屏（`viewer_check_20261109_155757_090862`）；12 s 键盘端到端 `155459` 用 XTest 向真实窗口发键，最终 x=0.417 m、yaw=0.082 rad、max_tilt=6.7e-06，驾驶模式 `passed=true`。
- 限制：原地转 0.2 rad/s、速度 0.4 m/s、腿长 0.22–0.30 m 已验；0.3 rad/s 原地转和“绝对航向保持后制动”会失稳，已撤回，当前用速率转向 + 松键阻尼。被动 viewer 的 GUI 渲染使仿真约 2× 慢于实时（20 Hz 刷新；10 s 仿真约 20 s 墙钟），键盘命令是锁存式（key_callback 只有按下事件）。窗口模式保留 GUI 扰动力，鼠标 Ctrl+拖动现在可推动，但未量化。
- 未验证：鼠标扰动量的可重复性、其它机器/窗口尺寸的实时性、长时间驾驶、高速转向、实机、GPU/Isaac。没有训练或新模型。下一项最小任务仍是可部署 IMU/编码器/六状态观测契约；驾驶层只是演示，不改变站立验收和正式采用边界。

## 本轮交互修正

已读最新代码、前窗口交接与规范，前窗口已释放。本轮限 E002 的根启动、teleop、渲染场景、必要跳跃支撑输入与日志；保留现有相机/录像实现和历史证据，默认入口精简为持续键盘驾驶。源轮子 STL 恢复作视觉，圆柱只留碰撞；地面纹理只在运行场景派生，不改正式 XML/机构。先验行走/腿长，再验纯电机跳起和落地；未过则写清实际结果，不伪造起跳。

### 交互修正交接

- 默认 `controller.py` 即持续 viewer+drive，直到关闭；--hold 不再冻结。日常仅 W/S、A/D、Q/E、J、X/R，左右调试 UI 默认隐藏。旧录像/对照参数兼容但从 --help 隐藏，不重排目录。
- 原轮 STL 改为实际 visual，圆柱只保留原碰撞；地面 texuniform 棋盘仅 runtime 派生。正式 XML/mesh/机构/惯量/碰撞/摩擦未改；[检查结果](../../experiments/E002_original_controller/outputs/interaction_check_20261109_185134_877827/results.json)记录精确物理数组比较。无窗口站立与改动前 trace 逐值一致。
- J 定时下蹲/蹬地/落地，仅通过原六电机，经 VMC 叠加每侧支撑目标且保留限幅。真实键盘默认窗口运行 14.728 s，移动1.324 m，轮实际转动约 ±17.13 rad（行走区间），Q 两次实长 .23818 m，双轮腾空 .24 s、上升 .12745 m，落地后正常由窗口关闭。证据为 `real_keys_20261109_184410_602759` 及对应 `lqr_no-gas_nominal_20261109_184411_704809`。
- 初次 X11 沙箱无权限脚本未判空退出139；之后改桌面执行边界。鼠标焦点请求报 BadMatch，未做完整自动拖动；其打开的默认窗口继续收到交互指令，未抢占/强关。最后 W/J/X 内建显示冲突的 flags 保持修正仅做 API 单元验证，报告明确此范围。
- 关键修改只限 E002 的 controller/teleop/render_stand/simulation 与 lqr_controller 两处 jump_force 支撑叠加。原源码/历史输出不改。源码来源/hash/diff 和交互报告已更新；跳跃编码器空转里程计不等于机身位移的限制保留。
- 下一项最小任务：用户在重启后的同一入口确认日常驾驶手感；如需鼠标量化，再在用户不占用窗口时做受控输入。未验 gas 跳跃、复杂地形、连续长时驾驶、跨机、实物、硬实时或学习训练。

## 本轮：从锁存改为真实按住/松开

用户明确要求 W/S 松开停止、前后方向反转、速度提高3倍；腿长扩大且按住连续改变、松开保持；提高跳跃并在 README 说明参数/接口/架构。本轮限 E002 原位置，不改模型/力矩限幅/原 LQR/VMC。MuJoCo key_callback 无 release，因此改由现有 X11 库查询当前键位并绑定自有 viewer 焦点，不安装 pynput 或改环境；J/X/R 保留离散按下动作。先无头物理短测，再真实桌面按下/释放。所有新运行独立 outputs。

### 按住控制最终交接

- E002 原位置修改 KeyboardInput/Teleop 和根入口接线，参数/方法/架构写 README；不改模型、LQR/VMC或力矩上限，不安装键盘库。KeyboardInput 查询当前系统键位而非猜释放，绑定自有viewer焦点，松开/失焦清空连续轴；J/X/R排队一次性动作。
- W实际模型-X、S+X，目标速度±1.2（3倍）/目标加速1.5/制动3；Q/E .12m/s连续调 .18–.34m，松开保持；J下蹲.20/蹬地.34/额外支撑200N/阶段.40+.20s，总1.60s，限幅不变。
- [分项汇总](../../experiments/E002_original_controller/outputs/hold_validation_20261109_191633_911604/results.json)：四项12s物理时序通过；真实速度约±1.176、释放后约1.47s进入 |v|<.05，后期速度峰值约.0062；实际腿长.180436/.339732；跳升.22137m/腾空.421s且落地。
- 真实桌面 `real_hold_20261109_191114_299662` 发送实际 down/up，实际run `191115` 记录 w→空/s→空/q→空/e→空及J并通过。GUI渲染慢，时序墙钟与模拟时间分开说明，没有拿它夸大m/s。`191634` 无窗口站立10s回归通过。
- README 仍一条命令启动，新增参数表、函数接口表、主循环调用例子、Mermaid架构与修改建议。源码hash/diff、来源和学习笔记同步。原锁存操作仅保留历史，不作为当前说明。
- 限制：真实物理制动非瞬停；仅Linux/X11、默认无弹簧平地分项验证，不声明全部速度/腿长/转向组合、长期重复跳、其它平台、实物/硬实时/训练通过。下一项最小工作是按用户实际手感挑一个组合短测，不再盲目放大参数。

## 本轮：实时速度与原生界面

已读用户粘贴的 DeepSeek 建议、结构规范、当前总览与报告；前窗口已释放。采用最小 P0：state_only 同步、HUD默认关/低频、原生面板/free camera、不回写显示flags；物理模型、dt、重复mj_forward和VMC暂不改，先测墙钟预算。计时固定已有mjlab解释器，实测窗口静止/驾驶及基线轨迹；不把粘贴建议里的数字当本轮证据。不重排目录。

### 实时/UI最终交接

- 实际改动仅E002 controller/render_stand/keyboardInput及README/REPORT/来源hash/diff；默认原生面板与FREE视角、state_only约30Hz、HUD可选10Hz；X11后台线程和三维区局部抓取驾驶键，侧面板/失焦停止，--no-drive交还原生快捷键。原模型/dt/LQR/VMC/Teleop数值不变。
- 实际桌面按下/释放/W/J14s：RTF.99247；最终含鼠标旋转、面板移入制动和HUD：14/14.00197s，.99986。无驾驶原生快捷键显示设置能保持。最后headless10s轨迹与改前逐值一致；资产/控制hash/语法均过。证据 [realtime_validation](../../experiments/E002_original_controller/outputs/realtime_validation_20261109_200238_639594/results.json)。
- P0驾驶.51458、同步查询+抓取.44993失败保留；cProfile因os._exit无pstats，不作函数归因。已撤回周期覆写flags，未采用P1物理/几何缓存改动。说明与原理已更新overview/KEY_OPERATIONS/阶段报告/学习笔记。
- 边界：近1倍播放非硬实时，最终短时最大落后.164s；passive界面不等于完整simulate暂停/改模型流程。只验本机X11/无弹簧平地短测，极短动作点按可能漏读，跨机/长期/gas混合驾驶/实物/训练未验。下一步最小30s用户混合动作和落后检查，非新增复杂功能。

## 本轮腿长K与腾空收腿

用户要求lqr.py/lqr_controller.py按腿长拟合不同K，正确处理机身单边与腿质量；跳跃独立文件、腾空后收腿。范围E002及对应说明/来源；先核对原推导和完整模型，再离线拟合、留出长度稳定性、10s站立与变长/移动/跳跃对照。模型/质量不随意更改，真实COM高度和轮底离地高度分别报告，不保证内部收腿能无外力提高整机质心弹道。

### 腿长K/独立收腿交接

- E002实际改lqr.py、lqr_controller.py、teleop.py、主入口接线/结果、simulation快照包含gain_schedule；新增jump.py与gain_schedule.json。保持原根入口+LQR_six结构，原资产/VMC/状态估计/环境/upstream不动。来源hash/diff与README/REPORT/总览/关键操作/学习笔记同步。
- 完整模型多点平衡/数值线性化/CARE/三次拟合，各腿实际L分别查K，读XML质量，不把半机身或闭链后支路漏算。初始总支撑保留静态标定。默认控制只算K并缓存系数，MPC nominal模型仅兼容短测。
- [汇总](../../experiments/E002_original_controller/outputs/length_gain_20261109_231153_464420/summary.json)：8组腿长/连续变长/行走/跳跃、默认/前倾/gas默认/MPC默认有限检查通过，资产hash/语法/左右反馈通过。中点最大2.35%误差、初设2%检查失败单列；不宣称全部检查通过。初版gas借no-gas点的试算已弃用，最终gas单独求点。
- jump.py独立实际腾空收腿、提前展腿、接触恢复。仅切换收腿对照：轮底间隙.162626→.242151m，COM峰值.536600/.536689m，基座峰值略低；不是整机COM升高保证。正常落地。默认真实GUI14/14.00084秒、RTF.99994；含额外输入不是独占自动时序，首次HUD .873失败留存。
- 未确认：.32–.34m拟合精度、全部非对称腿长/转向/组合、gas驾驶/跳跃、长期、跨机/实物/硬实时/学习训练。下一项最小任务.32–.34m密集采样及新留出集，再决定分段拟合，不继续放大力或把看过的点称新留出。

- 收尾：新增gain_schedule依赖后，复制项目必要闭包到/tmp并从/tmp启动10s通过；模块/输出只在副本，原outputs未改。copy_check.json保留，Markdown链接与来源hash核对通过。

## 本轮用户追加交互与易读性

只改E002本地代码/文档/记录；先保存用户当前已改参数（yaw1、腿.16–.36、跳跃400N及PID变化），不直接回滚为上轮值。参数集中、延长实际腾空收腿、允许前进+J、按住Shift持续旋转/松开缓停、移除翻倒终止但继续记录跌倒。普通运行K值保持，复杂动力学与清晰K计算分开；不改正式模型/环境/历史结果。目标短测长时间A/D或Shift再释放不倒、移动跳跃落地、倒地仍计时；数值异常仍拒绝继续。

### 叠加/易读性本轮交接

- E002新增parameters.py与lqr_model.py，参数全部公开/原K等价拆分；jump阶段直写+限速，维持.24s；Teleop独立轴/J叠加、Shift记忆方向并按住转。修改VMC朝向平面修正90°退化、共享消元，PID公共量每步一次、长度D实测，偏航角速闭环/缓停。原模型/环境/upstream不变。
- controller删除fall终止，继续记录；倒地平衡力矩0扶正恢复，NaN/警告仍退出。评测跌倒不判成功。新名称缓存不变10s轨迹；资产/系数哈希和lqr求值等价审计通过。
- [summary](../../experiments/E002_original_controller/outputs/stacked_controls_20261110_115613_339792/summary.json)：A/D两向20s、Shift两向20s，按11s转约±7.67/±16.37rad再释放站稳；W+J/静止J12s收腿约.241s并落地；主入口跌倒继续10s、无警告/验收失败分开；独立/tmp复制10s通过。
- 真实W+J/Shift链路通过但首轮RTF.879失败；缓存后另一次viewer手动/259N外力倒地仍24s且RTF.99899，自动化枚举BadWindow失败，不冒称受控无扰动通过。所有早期强参数/坐标/跳跃数值失败及before快照保留。README重写为当前简单操作，报告/总览/关键操作/学习笔记/来源hash同步。
- 未确认：任意组合、极限.16/.36腿长、gas混合动作、长期/跨机/实物/训练/硬实时；旧K .33m误差2.35%未达2%精度仍保留。下一项最小30s人工混合日志，或另行密采.32–.34m与新留出；不默认继续加力/重拟合/训练。

- 收尾补验：连续两次移动J（2s/6s）完成，修正空闲间隔污染腿长限速dt；12s未倒，最新快照final_code_v2，旧快照未覆写。原final_validation聚合exit_status=1因预期跌倒案例保留，汇总逐项区分继续运行与站立通过。

- 文档检查：本轮新链接/来源hash/语法通过；另发现历史E001目录当前不在磁盘，旧章节6个E001链接失效，单列final_static_checks.json，未重建历史数据或搬目录。

## 当前参数/制动/落地/独立地形
当前窗口认领E002，CPU短测；先保存用户已改3m/s、3rad/s、腿长P20000/D100000参数，再调试。地形单独XML，不改正式模型。

### 本轮参数/架构/落地/制动/地形交接

- 新增PARAMETERS/ARCHITECTURE逐项说明与Mermaid；场景scenes/training_terrain.xml独立合并，默认flat原样，terrain自由总览，原XML/网格/hash不动。高度10/20cm每级两组、17°坡；无外部参考下载。
- 修正低高度导致落地关闭电机，稳定接触/竖速/腿长渐恢复反馈；恢复安全PID/200N，速度默认1.2（不是3m/s验收），速度误差制动增益2、偏航P3/制动斜坡2，所有参数公开。
- response_v3：W/S释放到.05m/s约.76/.75s，Shift到.05rad/s约1s；J/移动J12s未倒，末腿长.277923/.277924。原仅计时淡入试验失败、初版弧度误用、离屏缓冲失败及中断不完整测试保留。最终汇总/源码/配置见terrain_braking_20261110_131430，旧gain精度2.35%限制未改。
- 地形仅验几何/角度/加载/原点10s站立，未验自主爬台阶/17°坡跟踪；未新验GUI实时、gas地形/3m/s急停/极限腿长/任意组合/长期/跨机/实物/训练。下一最小任务10cm单级明确阈值和起跳落点、记录接触与视频。

## 2026-11-12 单级 10 cm 台阶交接

- 认领时已核对前窗口释放，修改范围为 E002 单级场景、现有入口的试验接线、试验判据和记录，以及当前窗口汇总的项目文档。完整闭链机器人 XML 前后哈希一致，旧场景和历史运行留存。
- [10 s 验收](../../experiments/E002_original_controller/outputs/lqr_no-gas_nominal_20261112_214404_536859/results.json)中 X=−0.85 m 触发，起跳、触顶、无侧碰/非轮接触、末两秒稳定和闭链均通过。[−0.92 m 复跑](../../experiments/E002_original_controller/outputs/lqr_no-gas_nominal_20261112_214448_322296/results.json)只腾空 15 ms 即碰前立面，虽然最终停在顶面，整体失败。[默认平地 10 s 回归](../../experiments/E002_original_controller/outputs/lqr_no-gas_nominal_20261112_214448_257172/results.json)通过。
- 报告保留漏碰、旧接触判据误判及较晚触发碰阶的独立记录。下一项最小任务是用相同判据测新触发点。自主地形感知、跨地形/气弹簧版、异地复制、实物及训练未验。
- [录像复跑](../../experiments/E002_original_controller/outputs/lqr_no-gas_nominal_20261112_214418_864854/standing_nominal.mp4)通过，接触样本与无录像运行逐字节一致。CPU 与录像进程已结束，本窗口释放。

## 2026-11-12 后退、自起与台阶入口交接

- 本窗口负责 E002 髋电机仿真限幅、后退目标/制动、自起、台阶入口及汇总。正式与实验机器人 XML 前后校验一致，未训练。改动和数据见 [E002 报告](../../experiments/E002_original_controller/REPORT.md#2026-11-12-髋力矩后退自起和上台阶入口)。
- 五组后退相关短测通过，三组基线也通过，偶发翻倒根因未定位。平地无气弹簧 LQR 自起在 +1.45 / +1.46 rad 倒姿通过，其他初始角度和负向倒姿有失败。桌面 `--step-up --viewer`、录像和无头台阶复跑通过。
- 下一步记录用户实际操作的翻车前序和停止倒姿，再做针对性自起回归。长时、跨地形、气弹簧版与实物未验。

## 多地形与自起桌面交接

2026-11-12，E002 窗口完成 CPU 与桌面短测并释放。新增独立多地形 XML 并接入 `--step-up`，无头及桌面首级 10 cm 验收通过。新增自起桌面演示，+1.45 rad 初始倒姿通过。第二级、20 cm 车道、斜坡通行及任意翻车后的自起待验。运行证据及下一步见 [E002 报告](../../experiments/E002_original_controller/REPORT.md#2026-11-12-多地形入口和自起桌面演示)。

2026-11-12，本窗口完成场景和自动动作参数分离，全部入口接入自起，CPU 短测结束并释放。无气弹簧模型的两组 +1.45 rad 短测通过，气弹簧模型自起失败。下一步针对失败倒姿标定轨迹。[结果](../../experiments/E002_original_controller/REPORT.md#2026-11-12-场景自动上阶与默认自起)

2026-11-12，本轮定位普通驾驶的输出归零来自侧卧后的自起失败。新增接触触发约束、窗口状态和失败状态快照。指定倒姿仍通过，两条真实键位的无头回放仍失败。CPU 与桌面短测结束并释放。详见 [E002 报告](../../experiments/E002_original_controller/REPORT.md#2026-11-12-普通驾驶倒地排查)。

2026-11-12，已修正自起失败后永久停在零力矩的状态。16 s 短测执行三次有力矩的摆腿，最终站稳仍失败。当前窗口短测结束并释放。[证据](../../experiments/E002_original_controller/outputs/lqr_no-gas_nominal_20261112_235147_115232/retry_checks.json)

2026-11-12，后摆限定自起 CPU 短测结束。正向倒姿触发髋关节，负向倒姿零输出，双轮均为零力矩。站立交接未通过，台阶回归通过。[报告](../../experiments/E002_original_controller/REPORT.md#2026-11-12-仅后摆倒姿的髋关节自起)

2026-11-13，当前窗口完成 E002 后摆长腿收腿与 LQR 接管。按用户嵌入式状态划分分离了控制状态和腿长/摆角 PD。默认对其他倒姿保持零力矩，主动伸转准备需 `--recovery-prepare`。两种后摆倒姿和两种主动准备演示通过，负向转趴与气弹簧自起未通过。高腿长跳后翻倒已复现，原跳跃输入与控制保留。下一步单测负向翻滚的机身翻转。[报告和运行证据](../../experiments/E002_original_controller/REPORT.md#2026-11-13-后摆长腿收腿与-lqr-接管)

## 2026-11-13 多角度自起交接

本轮在 E002 的 `controller.py` 和 `LQR_six/` 实现明确的后摆长腿入口、贴地识别、伸腿转腿和收腿后抬身交接。正式 XML 与历史输出保留。九次 CPU 短测中 +1.45/+1.48 rad 起身，+2.0 rad 首次在 3.135 s 到达状态 2 后接管失败，近直立对照未误触发。GUI、气弹簧和实物未复验。下一最小任务从 +2.0 的接管状态回放，先验证双轮接触能否持续，再扩展翻滚转趴。[阶段和全部输出](../../experiments/E002_original_controller/REPORT.md#2026-11-13-多角度倒姿与状态-2)

- 2026-11-13，E002 直入 LQR 试验，CPU 扫描，范围为状态 2 接管及复验。

2026-11-13，E002 状态 2 直入 LQR 的九点短测中，5 点进入 LQR，0 点通过完整自起。2.0 rad 进入后未在接管窗口内脱离机身触地。下一最小任务是在真实状态 2 回放中先建立双轮承重，再测 LQR。本轮 CPU 已释放。 [直入 LQR 对照](../../experiments/E002_original_controller/REPORT.md#状态-2-直入-lqr-对照)
