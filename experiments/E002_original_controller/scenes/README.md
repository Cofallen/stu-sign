# 独立地形增量

training_terrain.xml仅放worldbody几何，controller.py --terrain training运行时通过load_play_scene合入原模型；不是可单独加载机器人场景的完整XML。默认flat不合并，原assets XML和网格不改。

10cm组(y0)顶面.1/.2m；20cm组(y2)顶面.2/.4m；每踏面长.8/宽1.4m。17°坡在y−2、中心x−3，长度2m、宽1.4m、厚.1m；原compiler使用radian，因此euler写.296705972839036。低端顶面接近地面，其下实体略埋地以连接地板。源几何由本项目直接写box，摩擦继承模型默认值，无外部资产或格式转换，不改机器人质量。

启动：本目录上一级用现有mjlab解释器python -B controller.py --terrain training --camera terrain。自由总览相机可调。验证限加载、几何尺寸/角度、原点平地站立；未验自主攀爬/上坡。

`single_step_10cm.xml` 是 `--terrain step10` 使用的独立场景增量。单个台阶顶面高 0.10 m，X 范围 [−3.6, −1.6] m，Y 范围 [−0.7, 0.7] m。台阶显式使用与地板相同的碰撞掩码 `contype=2 conaffinity=1`。自动短测命令见 [E002 README](../README.md)。

`step_up_course.xml` 是 `--step-up`、`--auto-step-up` 与 `--terrain step-up` 共用的可编辑多地形增量。中央两级顶面为 0.10/0.20 m，首级踏面长 2 m，给自动落点与制动留空间。Y=+2 m 车道顶面为 0.20/0.40 m，Y=−2 m 是 17° 斜坡。单位为米和弧度，box 的 `pos` 是中心，`size` 是半尺寸。所有地形显式设置地板碰撞掩码。修改中央首级前沿后可用 `--step-trial-x` 调整起跳触发位置，自动动作只验证该首级。
