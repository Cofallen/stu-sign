# M01 纠正：完整并联闭链机构

日期：2026-11-09。当前正式模型采用上游原生并联 MJCF。此前六关节串联简化版不满足用户的完整机构要求，撤回其作为正式机器人模型的状态，保留代码与记录作为历史对照。

## 当前两份 XML

- [完整闭链，无气弹簧](../../assets/robots/infantry_parallel/robot_no_gas_spring.xml)
- [完整闭链，保留可压缩弹簧阻尼](../../assets/robots/infantry_parallel/robot_gas_spring_damped.xml)

两份均保留 **14 个显式 hinge、4 个闭环 connect、全部独立连杆**，nq=21、nv=20。每侧两个腿电机和一个轮电机，共 6 个驱动；其余 8 个显式 hinge 被动运动。所有关节均可转动，但闭链中的关节不是彼此独立的自由度。

当前数值设置为 physics_dt=0.001 s、implicitfast、Newton/100 次迭代；腿电机软件 ctrl 上限 ±30 N·m、轮电机 ±5 N·m，分别记录为仿真输入限幅，不视为真机额定参数。实际 XML 电机顺序见 [actuator_mapping.json](../../runs/m01_model/parallel_check_20261109_004237_385074/actuator_mapping.json)，后续任务必须按名字映射。

## 结构来源与此前错误

来源为 `upstream/fudan_rl_wheel_leg/mujoco/assert_now/infantry_binglian_yuntai/infantry_V2/meshes/mjmodel.xml`，commit `8204e853dfd2ed06d85a322e1a998c3d20a3be2c`。上游自身引用了部分 V4 mesh；本次以该 MJCF 的连杆、惯量、转轴和层级为准，不再用六关节 URDF 代替完整结构。

标准 V4 URDF 的大腿 STL 将部分后连杆随同固定外观合并在一个 link 中。恢复完整网格只恢复了外观，并未恢复这些被合并部件的关节。这正是前次交付遗漏机构运动的根因。新的并联版使用上游已经拆分的 `l20/l21/l22/l23`、`r20/r21/r22/r23` 等独立网格，不将这些杆件焊在 f0 上，也不再按猜测的 CAD 连通部件剪除完整机构。

本次所有 link 的质量和惯量保持原生并联模型，总质量 21.054625 kg；不是此前标准简化版的 18.3492386 kg。已接受的完整机身外观保留为单独视觉层，不参与质量、惯量或碰撞计算。机器人仍为白色机身、深灰轮子。

## 每个关节与闭环销接都能转动

| 左侧 | 右侧 | 角色 |
|---|---|---|
| lf0_Joint | rf0_Joint | 前主动腿轴 |
| l20_Joint | r20_Joint | 后主动腿轴 |
| l_wheel_Joint | r_wheel_Joint | 主动轮轴 |
| lf1_Joint | rf1_Joint | 被动前膝轴 |
| l21_Joint | r21_Link | 被动后连杆轴，上游右侧名称原样保留 |
| l22_Joint | r22_Joint | 被动后连杆轴 |
| l23_Joint | r23_Joint | 被动后连杆轴 |

另有 `Left_loop1/2`、`Right_loop1/2` 四个闭合销接。MuJoCo 用树形 hinge 加闭环点连接表示该机构，而不是把 XML body 层级写成环。connect 保持连接点重合，未使用 weld 锁住相对转动；见[官方 equality/connect 说明](https://mujoco.readthedocs.io/en/latest/XMLreference.html#equality-connect)。本次还直接测量了这四处的相对转角，确认不是只看 XML 中有没有 joint 标签。

默认姿态 equality Jacobian 秩为 8，14 个内部树形转动坐标减去 8 个独立闭环约束后，保留 6 个内部自由度（4 个腿驱动自由度与 2 个轮轴）。没有新增隐藏锁定，也没有让闭环连接脱开来换取运动。

## 修正原始 10 mm 残差

后连杆支路在本地 Z 轴有 0.01 m 的板件错层。上游 front/rear 的闭环 sites 都设在各自截面 Z=0，因而约束连接的是沿销轴相隔 10 mm 的两点。对这两个不同截面强行施加点重合，会造成无法消掉的轴向残差。

修正只将四个 `*_rear_site*` 的本地 Z 从 0 改为 -0.01 m，选取同一销轴上的共同标记点；**没有移动连杆、转轴锚点或修改杆长**。再求解 8 个被动角，使初始 XY 装配闭合。通过 body 参考旋转与 joint ref 的配套变换，将该装配态设为默认态，同时保持原来的绝对关节角语义。

20 组相同源关节坐标下，修正前后所有 body 位置/旋转矩阵的最大差约 7.77e-16；质量和惯量差均为 0。说明坐标重设没有悄悄改变机构几何或惯量。默认闭环距离约 1.02e-15 m。

完整修改记录、装配角、源 mesh 路径/哈希和两 XML 哈希见 [manifest.json](../../assets/robots/infantry_parallel/manifest.json)。

## 气弹簧能确认什么

位置依据是上游明确命名的四个 site，不是根据细长外形判断：

- 左侧：`left_lf0_qitanhuang` ↔ `left_lf1_qitanhuang`
- 右侧：`right_lf0_qitanhuang` ↔ `right_lf1_qitanhuang`

这些标记随各自真正可转动的 link 运动。无弹簧版不保留 tendon 或气弹簧执行器；保留版在这些原安装点之间使用可变长度 spatial tendon，设 `limited=false`，不会固定长度或增加闭环锁定。

**不能据此确认真实气弹簧的刚度、阻尼、预载或行程。** 默认仍是未标定的等效压缩弹簧：k=1000 N/m、c=10 N·s/m、自由长度 0.20 m。它不是完整气体热力学模型，也没有精细套筒 CAD。两侧阻尼可分别在 `left_gas_spring`、`right_gas_spring` 的 `damping` 字段修改。

旧版 STL 两个部件的几何筛选证据不足以作为完整真机机构识别结论。本版直接使用上游干净的分杆网格和显式安装点，不依赖旧筛选方法。

## 实际动态验证

记录：[完整机构动态结果](../../runs/m01_model/parallel_check_20261109_004237_385074/results.json)、[全部关节转动范围 CSV](../../runs/m01_model/parallel_check_20261109_004237_385074/joint_mapping.csv)、[闭环、惯量与相对转动复核](../../runs/m01_model/parallel_integrity_20261109_004606_083292/results.json)、[对照视频](../../runs/m01_model/parallel_check_20261109_004237_385074/parallel_retraction.mp4)。

测试固定基座并关闭重力，只向 6 个真实驱动轴发送诊断控制，不向 8 个被动轴施加 actuator 力矩，也不在运动中强写 qpos。两腿前后主动轴差动收缩、恢复，轮子正反转，持续 4 s。

| 检查 | 无气弹簧 | 保留弹簧阻尼 |
|---|---:|---:|
| 14 个显式关节均有旋转 | 通过 | 通过 |
| 最小显式关节角变化范围 | 0.2499 rad | 0.2431 rad |
| 被动关节直接驱动力矩 | 0 | 0 |
| 运动中最大闭环误差 | 3.45e-8 m | 4.42e-7 m |
| 左轮相对机身上收 | 44.80 mm | 40.60 mm |
| 气弹簧长度范围 | 无 | 约 148.53–166.97 mm，两侧一致 |
| 数值警告/NaN | 无 | 无 |

四个闭合销接的相对转角范围均约 0.4097 rad，确认这些闭合处同样能转动。另在完整并联模型上隔离验证 c=0/10/50 时阻尼功率符合 `-c*ΣLdot²`，阻尼增大后耗能增大。原始自由基座两 XML 还各做了 1 s 正常重力冒烟，无数值警告或 NaN。

这些是机构与数值检查，不是平衡控制成功或真机参数辨识。后续 M02 必须采用完整并联版，重新定义驱动顺序与状态，不能沿用旧简化版的六关节观测映射。

## 运行命令

```bash
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/build_parallel_variants.py
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_variants.py --video
PYTHONNOUSERSITE=1 /home/cofallen/miniconda3/envs/mjlab/bin/python -B scripts/m01_model/check_parallel_integrity.py --motion-run runs/m01_model/parallel_check_20261109_004237_385074
```

生成器支持 `--damping`、`--stiffness`、`--free-length`；或直接修改保留版 XML。验收阈值只对本次记录的配置负责，改参数后需复验。
