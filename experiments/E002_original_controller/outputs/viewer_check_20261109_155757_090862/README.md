# viewer 界面人工检查记录

命令（2026-11-09 15:32，Asia/Shanghai）：

```bash
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B controller.py \
  --viewer --camera iso --duration 60 --hold
```

窗口在桌面正常打开并显示完整机器人站立；约 7 s 时用 `ffmpeg -f x11grab` 截屏，
随后手动结束该进程（未等 60 s 结束，故本目录没有 results.json）。
截图已裁到 MuJoCo 3D 视口，去掉了桌面其他窗口。

同一入口另有两次完整的 10 s viewer 运行：
`outputs/lqr_no-gas_nominal_20261109_152925_759900/`（通过，退出码 0）。

键盘驾驶的端到端按键注入记录见
`outputs/lqr_no-gas_nominal_20261109_155459_786203/`（12 s，通过；
XTest 向窗口发送 W/W/W、X、A、X，最终 x=0.417 m、yaw=0.082 rad）。

注意：该截图是加入 `--drive` 键盘驾驶覆盖层之前的版本；当前窗口还会在
左下角显示速度/转向/腿长，顶部显示按键提示。
