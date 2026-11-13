# 当前正式模型：完整并联机构

两份 XML：`robot_no_gas_spring.xml` 和 `robot_gas_spring_damped.xml`。均保留上游 14 个显式转动关节与 4 个闭合销接，所有独立连杆使用上游分开的网格，不使用串联简化代替。

源：yly-true/fudan_rl_wheel_leg，commit `8204e853dfd2ed06d85a322e1a998c3d20a3be2c`，原生 `mujoco/.../meshes/mjmodel.xml`。每个 mesh、修改和初始装配角见 manifest。原生质量/惯量保留；完整机身外观仅作为视觉层。

气弹簧安装点来自上游明确命名的 `*_qitanhuang` sites。阻尼可修改 `left_gas_spring/right_gas_spring` 的 `damping`。默认 k=1000 N/m、c=10 N·s/m、0.20 m 自由长度只是未标定的等效模型，不能当作真机参数。

许可范围沿用项目 THIRD_PARTY.md 的未确认状态，不自行附加许可证。生成、源对照和实际动态验收见 [完整机构报告](../../../docs/02_milestones/M01_parallel_linkage.md)。旧 `infantry_v4_standard_simplified` 仅保留历史对照。
