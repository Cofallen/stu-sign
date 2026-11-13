# E004 轮腿可微世界模型训练与部署

在 E004 目录运行。使用现有 mjlab Conda 和完整闭链模型。训练以 E003 PPO actor 热启动，采用 SuperMimic 的交替动力学拟合与模型内策略梯度。 [从零复刻教程](TUTORIAL.md)

```bash
conda activate mjlab
python -B train.py --initial-actor ../E003_mjlab_ppo/outputs/train_20261113_163126_398557/model_1478.pt --num-envs 128 --iterations 8
python -B deploy.py
python -B deploy.py --headless
```

部署默认读取本目录已归档的 E004 actor。新训练用 `--checkpoint outputs/<新运行>/actor.pt` 指定。热启动权重可从任意路径传入，部署只需本地 XML、网格、NumPy、PyTorch 和原生 MuJoCo。

8 轮训练采集 49,152 条控制转移。留出环境单步观测 MAE 从 0.0128 降至 0.00171，梯度有限且非零。原生 MuJoCo CPU 的 30 秒混合指令回放正常结束。证据在本目录的 `outputs/train_20261113_164401_385717/` 与 `outputs/native_deploy_20261113_164413_796795/`。相比 E003 的性能增益、多回合通过率、变腿长、跳跃和随机振动地形待验证。

原生 MuJoCo 窗口默认是键盘模式：点击窗口后，`W/S` 前进/后退（±0.4 m/s），`A/D` 左转/右转（±0.3 rad/s），`X` 停车，`R` 重置并停车。指令保持到下一次按键；跌倒后按 `R` 继续。关闭窗口结束并保存轨迹。固定指令演示用 `python -B deploy.py --scenario demo`；录制用 `MUJOCO_GL=egl python -B deploy.py --headless --scenario demo --video`。

默认 XML 的轮子外观使用原始 STL，地面使用方格纹理；圆柱仍用于碰撞，在部署窗口默认隐藏。

窗口默认按真实时间 1× 运行，保持 100 Hz 策略更新。可用 `python -B deploy.py --scenario demo --seconds 8` 复测，结果写入新目录的 `run.json`（`loop_wall_seconds`）。无头回放不等待墙钟。
