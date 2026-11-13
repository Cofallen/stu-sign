> **历史简化模型：不是完整闭链机构。** 当前正式文件在相邻 `../infantry_parallel/`。保留本目录用于复核旧记录，不用于完整机构训练。

# Fudan infantry V4 标准串联简化模型派生资产

新增两套可直接加载的版本：`robot_no_gas_spring.xml`（移除刚性气弹簧外观，无弹簧力）和 `robot_gas_spring_damped.xml`（可变长度等效弹簧阻尼，XML 内调 `damping`）。基础 `robot.xml` 保留作对照。生成及实际验收见 `docs/02_milestones/M01_gas_spring_variants.md`；变体的删除面记录、安装点与未标定参数见本目录 `gas_spring_variants.json`。

- 源仓库：https://github.com/yly-true/fudan_rl_wheel_leg
- 固定 commit：`8204e853dfd2ed06d85a322e1a998c3d20a3be2c`
- 源 URDF：`plane/resources/robots/infantry_V4/urdf/infantry_V4.urdf`
- 原始仓库保存在项目 `upstream/`；本目录不是上游原件。
- 生成入口：`scripts/m01_model/prepare_model.py`，依赖版本见模型审计 run 元数据。
- 内容：完整源 visual STL 的保形分块、源 collision mesh 的凸包、独立转换的 MJCF、源/输出 SHA256 manifest。
- 质量和惯量来自该标准版 URDF，不用加重版、并联版或其他机器人替换。
- 补充设置：自由基座、地面、直接力矩 motor、接触参数与碰撞 mask，均在生成脚本和 manifest 中明示。
- 正式外观使用全部源视觉三角面；通过每块不超过 190,000 面的二进制记录分块绕过 STL 面数限制，逐字节校验三角面记录。没有简化、重心平移或替换机身网格。
- 按用户选择，配色来自上游原生 MJCF：机身白色、轮子深灰色。manifest 分别记录原 URDF 颜色和当前颜色；仅引用外观，不引入并联模型物理参数。
- visual 归 group=2，contype/conaffinity=0，不参与碰撞；collision 归 group=3，默认隐藏。这一串联简化表示仍不能当作真实并联机构等价物。

上游 checkout 未发现仓库级 LICENSE，也未在所选资产中找到明确授权声明；资产许可范围待确认。文章的 CC BY-NC-SA 4.0 不自动覆盖这些文件。保留来源，不自行赋予 MIT/Apache 等许可证；对外发布前核查具体资产授权。详见项目 `THIRD_PARTY.md`。
