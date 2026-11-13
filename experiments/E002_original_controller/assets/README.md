# 随包完整闭链模型

`robot_no_gas_spring.xml` 与 `robot_gas_spring_damped.xml` 来自主工程正式 `infantry_parallel`，逐字节复制；所有 XML 引用的 24 个 STL 均放在同目录，非软链接。保留 14 个转动关节、4 个闭环销接、8 个被动轴、6 个真实驱动、全部独立连杆及原外观。nq=21、nv=20。

`manifest.json` 原样保留源几何、惯量、装配与视觉来源记录，其历史路径是来源信息，不是加载依赖。文件哈希和更完整的来源归属见上级 `SOURCE_MANIFEST.json` 与 `THIRD_PARTY.md`。

带弹簧版是未标定的等效可变长度 spatial tendon：k=1000 N/m、c=10 N·s/m、自由长度 0.2 m，不是经过真机确认的气体模型；STL 不会自行变成伸缩套筒。无弹簧版没有 tendon。不要混用旧串联版参数。
