# AI Formula · Unity 驾驶训练

[加速先行与减速绕行的双车对照](crossing_evidence/paired-crossing.mp4) 在同一时钟下比较两种选择，包含实际车速、A4 识别、车尾驶离时间与横移桶到达时间。[测试条件、原录像审计与留出结果](crossing_evidence/README.md)。

[七段避障实录](obstacle_films/index.html) 可连续播放，也能单独打开静止单桶、错列双桶、横移桶、直道动静混合、弯中单桶、弯中动静混合和碰桶复现。每段都同步展示跟车、A4 前视、车的局部选点与速度。跟车窗的小图使用场地固定坐标，显示裁判记录的车和桶的位置，方便辨别自车运动与锥桶横移。

原始闭环轨迹分别在 `risk_revision/runs` 和 `scenario_shots/runs`。三个新布局用冻结策略跑至驶离障碍，另用原种子重跑碰撞场景。[镜头采集记录](scenario_shots/README.md) 给出了标定、通过区间和失败复现核验。从已录相机帧重出全部视频，在本目录运行 `python -B assemble_obstacle_films.py obstacle_films --render`。若新场景原始帧还在独立工作树，添加 `--capture-root scenario_shots <该工作树的场景素材目录>`。只从七支已有 MP4 重建带章节的合集时省略 `--render`。

最新的 [A4 避让与圈速对照](risk_revision/index.html) 包含真实 Unity 前视、追车和同步轮速，两场空场赛与四个留出障碍布局的成对结果。可以选任一路障并拖动控制周期，查看 A4 看到了什么、车选了哪条线、何时收油以及它实际发出的两个轮速。页面下方列有训练和资格记录、车身回摆指标、单独放宽避让限速的对照。

在已启用的私有仿真工程跑本批实验时，依次使用 `python -B risk_trials.py baseline`、`python -B risk_curriculum.py`、`python -B risk_select.py`、`python -B risk_ablation.py` 和 `python -B risk_film.py`。各实验采用冻结求解器。最后执行 `python -B command_replay.py --root risk_revision --run decision_film --out risk_revision/commands.mp4`，再运行 `python -B risk_audit.py` 和 `python -B risk_results.py`。已有记录可直接离线运行最后的报告生成步骤。对移动障碍加大预测范围的独立试验见 [失败记录](reversal_diagnostic/README.md)。

打开 [驾驶回放](replay/index.html)。录像来自 Unity 实际机位。可按章节跳到单桶、连续双桶或横移路障，点击每个预训练试次查看车辆轨迹，也可在不同策略和传感器条件之间切换。页面支持本地文件打开。

## 车如何驾驶

| 环节 | 实现与可用信息 |
|---|---|
| 出生与计时 | 裁判从种子抽取外环位置、左右 0.30 m 内的偏移、前进方向 ±3.2° 的朝向偏差。车辆从静止起步。依次通过 32 个门并回到出生断面算一圈。|
| 感知 | 预训练可选择车前局部几何和锥桶视线检测。实际相机模式以 5 Hz 获取原装 640 × 360 光学 RGB，调用已安装的 A3 或 A4 单帧模型。使用 mask 在 2、3.5、5.5、8 m 的地面投影估计车道横向位置，框底中心投影成锥桶位置。缺失观测与实际推理帧数分开计数。 |
| 选线与收油 | 根据近处与远处的曲率变化选择横移目标。纵向指令依直线速度上限和观测曲率变化。预瞄、转向反馈和底层转速转换的形式由人指定，训练得到系数。记录中的两路 CAN 数值是仿真目标转速，速度读数来自车身运动。|
| 避障 | 连续观测配合仿真车身运动补偿得到锥桶相对位置和速度。局部规划器比较多种横移目标与三档速度比例，用预计到达时间估算能否完成横移。最小代价的动作继续由同一反馈控制器执行。运动障碍会在车到达前、到达时以及驶离时持续运动。 |
| 学习 | 第一阶段用真实 Unity 物理做交叉熵搜索，优先完圈，再比较计时。第二阶段要求每个候选在两个随机起点与布局接受评价。第三阶段固定 A4 权重，直接在相机闭环中练习速度、选线与避障参数的组合，使用第三个资格布局排序，再冻结并进入全新的盲测。 |
| 裁判 | 评分有整图和障碍物真值，策略获得局部观测、速度与偏航运动。车身代理长 1.16 m、宽 0.76 m。任一角越出两条外环车道，或车身代理进入给定锥桶半径 0.34 m 的保守外包圆，就终止。此判据严于比赛的四轮全出。 |

收油通过连续降低目标轮速实现。锥桶测速依赖仿真自车运动补偿，平地投影使用固定相机标定。独立刹车力、实时换电适应、轮胎滑移识别、红灯规则和行人行为需要分别验证。速度、路障预测范围、可执行动作、计分规则及观测频率都留在源码中，可按新的试验问题调整。

## 文件与证据

- [长训练、巡航与双侧空隙实验](long_training_evidence/README.md)。

- [本轮两版学习控制器、训练过程、完整跑圈和避障录像](learned_evidence/README.md)。

- [工作记录](WORKLOG.md)、[所有试车的参数与结果事件](evidence/events.jsonl)、[行驶轨迹](evidence/runs/)。
- [实际相机冻结策略](evidence/camera_policy.json)、[联合预训练策略](evidence/joint_policy.json)、[全部考核](evidence/summary.json)、[物理与视频核验](evidence/validation.json)。
- [车道与障碍决策](Road.cs)、[Unity 测试环境、裁判和摄像机](UnityRace.cs)、[RGB 识别适配](vision.py)、[A4 相机练习与预置盲测种子](camera_training.py)。
- [无障碍及首轮交通训练](train.py)、[第二轮联合训练](traffic_training.py)、[保留 / 关闭检测 / 关闭位移外推](ablations.py)。

## 在本机复验

独立实验工程使用项目原有 Unity 2022.3.22f1 和已安装依赖，仿真为 JanuaryLapIdentified + 耦合后轮接触，物理步长 0.01 s。以下 PowerShell 命令在本目录运行。使用新的、尚不存在的私有副本目录，保证与原编辑器分开。

```powershell
$env:AIFORMULA_LAP_COPY = 'E:/Mess/Projects/Programming/aiformula/workspace/my_private_lap_replay'
python -B isolated.py --copy $env:AIFORMULA_LAP_COPY
# 初次导入完成后，副本的 LabQueue/ready 会出现。
python -B camera_runs.py --release a4 --name my_new_camera_trial --seed 43210 --policy evidence/camera_policy.json --obstacles --capture .12
python -B render.py my_new_camera_trial my-run '新起点 A4 复验'
# 仅停止当前命令显式创建的私有副本。
Set-Content -Encoding ascii -LiteralPath "$env:AIFORMULA_LAP_COPY/LabQueue/stop" -Value 'stop'
```

`train.py` 的首轮搜索对应本地提交 `93f1f75` 的控制器。当前源码的第二轮和视觉练习依次运行 `traffic_training.py`、`camera_training.py`。在全新输出上重现历史阶段时，应使用相应源码版本与相同安装版物理。已有运行 ID 要求参数、控制器哈希和推理模式一致，重复运行会复用记录；新实验使用新 ID。每趟原始录像帧位于本地 `evidence/.work/frames`，实际视频文件及完整轨迹均存放在本实验目录。
