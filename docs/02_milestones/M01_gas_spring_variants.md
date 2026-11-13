> **历史串联简化模型记录，已撤回正式完整机构地位。** 当前应使用 [完整并联闭链报告](M01_parallel_linkage.md) 及 `assets/robots/infantry_parallel/` 的两份 XML。下文成绩仅适用于当时的简化模型。

# M01 补充：无气弹簧与可压缩弹簧阻尼两套 XML

日期：2026-11-09。按用户要求生成两个可独立加载的 XML，源仓库和基础 `robot.xml` 保留不变。

## 文件与差异

| 模型 | 文件（均在 `assets/robots/infantry_v4_standard_simplified/`） | 行为 |
|---|---|---|
| 无气弹簧 | `robot_no_gas_spring.xml` | 移除两侧刚性气弹簧 CAD 缸体/杆，无 tendon、弹簧力或气弹簧阻尼 |
| 保留弹簧阻尼 | `robot_gas_spring_damped.xml` | 同样移除旧的固定 CAD 杆；两安装点之间使用可变长度 spatial tendon，具有可调弹性与阻尼 |

两套模型均保持 nq=13、nv=12、nu=6、neq=0；保留版另有两个 tendon，没有新增自由度、刚性闭链或固定长度约束。两套 XML 与 mesh 放在一起即可加载；不是只供 Python 脚本拼装的片段。

当前基础六关节 XML 原本就没有气弹簧物理力或固定长度约束。问题是源 STL 将气弹簧缸体和活塞杆固化在大腿 link 中，使外观看起来不能伸缩。两侧从源 mesh 各识别出 2 个独立连通部件、共 862 个三角面（缸体 708、杆 154），派生文件只移除这些面，其他原始面记录保留。筛选包围盒、源哈希和删除面索引均保存，不替换腿杆或整个机器人。

新空间弹簧显示为随端点长度变化的灰色元件，不是精细的两段套筒 CAD。这样仅靠 XML 就能伸缩，不需要额外脚本每帧搬动外观，也不引入未经确认的滑杆质量或闭链参数。

## 在 XML 中调参数

在保留版找到 `left_gas_spring` 与 `right_gas_spring`，两侧可以分别设置：

```xml
<spatial name="left_gas_spring"
         width="0.0075" rgba="0.65 0.65 0.65 1"
         stiffness="1000.0" damping="10.0"
         springlength="0.2 1" limited="false" frictionloss="0">
  <site site="left_gas_spring_upper"/>
  <site site="left_gas_spring_lower"/>
</spatial>
```

- `damping`：线性轴向阻尼，单位 N·s/m。设为 0 关闭粘性阻尼；例如改为 20 提高该侧阻尼。
- `stiffness`：等效压缩刚度，单位 N/m。设为 0 可单独测试阻尼。
- `springlength`：本实现用 `[0.2, 1.0]` 作为无弹性力区间；长度低于 0.2 m 时弹力向外支撑。在当前安装几何可达范围内，超过自由长度后不施加拉力。
- `1.0` 是使工作区间无拉力的死区上端，不是物理行程；`limited="false"` 不施加机械长度限位，不会把弹簧锁成定长杆。

采用 MuJoCo 的空间 tendon 弹性与阻尼语义，见[官方 XML 参考](https://mujoco.readthedocs.io/en/latest/XMLreference.html#tendon-spatial)与[被动力说明](https://mujoco.readthedocs.io/en/stable/computation/index.html)。在所测压缩区间，沿长度方向的力为 `k*(0.2-L) - c*Ldot`，其中阻尼功率为 `-c*Ldot²`。

默认 1000 N/m、10 N·s/m、0.20 m 是**未标定演示参数**，不是源模型的真实气体、预载或密封摩擦数据。该模型是只压缩支撑的等效线性弹簧阻尼器，不是气体热力学模型。真实预载、非线性曲线、摩擦和机械行程仍需测量后替换。

安装点来自同仓库原生 MJCF 的 `*_qitanhuang` sites，适配到标准 V4 的 f0/f1 坐标系；来源和坐标均写入 `gas_spring_variants.json`。这是明确登记的几何适配，尚不声称完成硬件安装点标定。

源 URDF 只提供 link 整体惯量，未提供气弹簧单独质量/惯量。两套暂保留源整体惯量，总质量均 18.3492386 kg；没有猜测并扣除气弹簧质量。因此“无弹簧版”表示移除其外观、碰撞部件和力学作用，尚不是已称重的真机拆件动力学模型。

## 实际验收

证据：[results.json](../../runs/m01_model/gas_spring_check_20261109_002615_952147/results.json)、[自由基座冒烟](../../runs/m01_model/gas_spring_check_20261109_002615_952147/free_root_smoke.json)、[收缩—伸展对照视频](../../runs/m01_model/gas_spring_check_20261109_002615_952147/retract_extend_comparison.mp4)。

诊断夹具固定基座、关闭重力，使用相同 PD80/3 和相同腿指令：两膝从 ±0.4 rad 收至 ±0.8 rad，再伸回。验收要求左右腿均缩短 >40 mm、保持阶段角误差 <0.04 rad、伸回角误差 <0.04 rad、无 NaN 或 MuJoCo 数值警告。该测试不是站立或运动控制策略。

| 指标 | 无弹簧 | 可压缩弹簧阻尼 |
|---|---:|---:|
| 左/右腿竖向缩短 | 59.29 / 59.29 mm | 54.11 / 54.11 mm |
| 收缩时左/右膝角 | +0.8000 / -0.8000 rad | +0.7701 / -0.7701 rad |
| 伸回时左/右膝角 | +0.4000 / -0.4000 rad | +0.3705 / -0.3705 rad |
| 两侧弹簧长度（初始→收缩） | 无 | 145.94 → 131.06 mm |
| 诊断峰值关节力矩 | 0.051 N·m | 2.574 N·m |
| 判据 | 通过 | 通过 |

保留版存在有限 PD 稳态误差，这是弹力与控制器平衡的结果；没有用强制写 qpos 冒充动态收缩。两侧长度随后回到约 147.25 mm。整个 4 s 过程实际积分并录像。

阻尼单独验证时将刚度暂设为 0，保持相同状态/速度，仅改变 c=0/10/50 N·s/m，测得总被动功率分别为 0/-0.0192235/-0.0961173 W，与 `-c*Σ Ldot²` 一致。阻尼确实耗能，且增大后耗能增加。

两份原始输出 XML 还各做了 2 s 自由基座、正常重力 CPU 步进，无 NaN 或数值警告；这不等于平衡成功。

## 生成与复验

在项目根目录，沿用现有 mjlab 环境：

```bash
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/build_gas_spring_variants.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_gas_spring_variants.py
```

生成器还接受 `--stiffness`、`--damping`、`--free-length`。重新生成会重建两份派生 XML，因此手工调好的参数应先另存；源模型与基础 `robot.xml` 不会被覆盖。复验脚本的对照和阈值针对上述默认演示参数；换参数后应重新解释其行程与误差，不能沿用旧成绩。

下一步仍是 M02 接口与最小任务；训练时必须明确选择哪个 XML，并将弹簧配置纳入世界模型状态/上下文和推理元数据。
