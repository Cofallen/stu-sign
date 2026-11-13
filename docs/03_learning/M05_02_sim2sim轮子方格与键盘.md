# E003 / E004：轮子外观、方格地面与键盘部署

旧 XML 的轮子虽然登记了 STL，但实际显示几何引用的是圆柱。两个实验的默认 XML 现在把左右轮子的 group 2 显示几何改为对应 STL，保留零质量并关闭接触。group 3 圆柱继续计算轮地接触；部署窗口与离屏渲染隐藏 group 3 以免盖住轮毂。地面的 `plane` 接触参数不变，方格来自 XML 的 `checker` 纹理和材质，因此直接打开 XML 也有方格。

在任一实验目录，用 `~/miniconda3/envs/mjlab/bin/python -B deploy.py` 打开原生 MuJoCo。点击窗口后用 W/S 前进与后退，A/D 左转与右转，X 停车，R 恢复初始站姿并清除上一动作。按键回调只更新指令和重置信号，主循环取出后把纵向速度、固定腿长 0.28 m、偏航角速度送给原策略。每个指令保持到下次按键。跌倒时窗口保持打开，按 R 可再试。

固定演示用 `MUJOCO_GL=egl ~/miniconda3/envs/mjlab/bin/python -B deploy.py --headless --scenario demo --video`。两次 30 秒回放都正常退出：[E003](../../experiments/E003_mjlab_ppo/outputs/native_deploy_20261113_170620_172589/run.json) · [E004](../../experiments/E004_supermimic/outputs/native_deploy_20261113_170631_031173/run.json)。同一初始姿态下，用相同六路正弦力矩驱动 E002 与两个新 XML，1000 个原生 MuJoCo 物理步的 qpos、qvel 逐步完全一致。本次未重新训练或做实机测试。

宿主 X11 向真实窗口依次发送 W/S/A/D/X/R，两份轨迹均记录了四个非零指令、停车和仿真时钟重置：[E003](../../experiments/E003_mjlab_ppo/outputs/native_deploy_20261113_170900_644257/trace.csv) · [E004](../../experiments/E004_supermimic/outputs/native_deploy_20261113_170956_272059/trace.csv)。自动化限时退出在宿主出现过 GLXBadContext；已写出的轨迹完整，正常关闭路径仍需留意。

窗口实时节拍：原部署每 10 ms 在控制线程同步一次窗口，显示争用可拖慢物理循环。现保持 100 Hz 策略和 1 ms 物理步长，用单线程执行小批量 CPU 推理；显示线程用独立 MuJoCo 数据读取最新位置快照，每次刷新后等约 33 ms。控制线程使用单调墙钟按 1× 节拍运行。无头模式仍不等待墙钟。复跑命令为 `python -B deploy.py --scenario demo --seconds 8`，可在 `run.json` 查仿真秒数和 `loop_wall_seconds`。宿主窗口 [E003](../../experiments/E003_mjlab_ppo/outputs/native_deploy_20261113_172020_652267/run.json) 和 [E004](../../experiments/E004_supermimic/outputs/native_deploy_20261113_172032_266051/run.json) 各用约 8.0001 秒完成 8 秒仿真并正常退出。两份轨迹与各自无头回放及历史演示的前 8 秒逐字节相同；没有重新训练。
