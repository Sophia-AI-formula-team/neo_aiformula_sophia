# V6.26 驾驶节点

输入是一张相机图、这张图的路面与标线掩码、检测框，以及拍摄时已经测到的两轮转速和陀螺。程序从零积累路面和障碍记忆。整个驱动在一个 Python 进程内工作。组件各自有 ROS 包，可以分别维护和测试，图像在进程间只传一次。

|包|负责的工作|
|---|---|
|concept_road|用标线、路面颜色和当前车道路面掩码恢复道路横截面。结合车轮与陀螺保存刚从镜头下方消失的路面。路口仍需找到沿赛道的边缘。|
|concept_obstacles|把同一个桶的嵌套检测框合并，结合图像和车辆运动，跟踪位置、速度及误差。|
|concept_feasibility|使用前进与倒车响应拟合，沿预测路径检查轮胎、路边、感知误差、障碍和减速可行性。|
|concept_candidates|生成候选动作、检查继续前进和失去下一帧时的后续动作，保存当选控制序列。恢复过程有独立的检查和执行逻辑。|
|concept_ranker|给白盒允许的前进候选加减分。四个宽度为12的网络输出平均分数修正，参与训练的输入与权重随包保存。|
|concept_drive|V6.27.2 按新图像触发规划，每秒最多请求 15 次。白盒、排序和恢复完成后，向物理指令门发送带采集时间的候选控制。车端验收要求每秒至少 5 条新指令。|

## 启动和操作

使用本田投影或实车标定投影时，前进和倒车候选按车上 `motor_controller` 的直行换算预测左右轮指令。规划的零纵向释放动作在实车输出为两轴全零。手柄仍可原地转向。车身速度和制动的响应系数源自 Unity，实车低速起步阈值仍需实测。

在 `/concept/decision` 的 JSON 中查看 `candidate_evaluation`。一次常规前进决策会显示 `mode=exact`、候选总数 `input_count` 和实际模拟数 `evaluated_count`。例如总数 75、实际模拟 55 表示有 20 个目标与速度均相同的重复请求共享了结果。所有 75 个编号仍参加原有的筛选和排序。没有执行前进搜索的决策会显示 0 次。

在已经构建并 source 的 ROS 2 工作区运行。完整的相机、状态和指令门由整机启动入口管理。

~~~bash
ros2 run concept_drive drive
ros2 param set /concept/drive target_speed_mps 2.0
ros2 service call /concept/drive/restart std_srvs/srv/Trigger '{}'
~~~

先暂停物理指令门，再重启驾驶。服务清空车道、障碍、回退计划、里程图和旧轮速，等待下一张图及对应的新测量。车载 GUI 可以用变化的 session ID 清空显示。驾驶器收到物理门正在执行自动输出的状态时，会拒绝清掉记忆。目标速度可在 (0,6] m/s 内调整；收到门的速度上限后，参与规划的速度取两者中较小的值。冻结训练验收的目标是 3.4 m/s。更换速度后，路线经过相同检查，圈速需要单独实测。

## 接口

- 相机使用 /concept/perception/frame，类型为 std_msgs/String。载荷是 concept.perception.v1，含 ROS stamp、光学 frame_id、geometry_profile、模型 SHA-256、无损的 640×360 BGR、路面和标线掩码、原检测框/类别/分数。编解码来自 concept_yolop。
- 车辆反馈使用 /concept/vehicle_state，类型为 std_msgs/String。t 为 ROS 采样秒数，leftRpm、rightRpm 为轮转速，wheelRadius 单位米，gyro 为 ROS 左转正的 rad/s。驱动选图像采集之前最近的一次有效样本，允许最大间隔 0.15 秒。相邻采样均值用于短时坐标变换，内部转弯按右转为正。
- /concept/auto_cmd 为 geometry_msgs/TwistStamped，header.stamp 是图像采集时间，frame_id 为 base_footprint；linear.x 单位 m/s，angular.z 按 ROS 左转为正。驱动没有 CAN 和电机发布器。输出由物理门独立裁决。
- /concept/decision 为 std_msgs/String，载荷为 concept.decision.v1。stamp 是图像采集时间，emitted_stamp 是发布时刻，capture_age_s 含推理、调度与规划延迟，sensor_age_s 是图像与轮速的时间差，processing_ms 仅计驾驶节点处理时间。target_speed_mps 与 effective_speed_mps 分别表示用户设定值和本帧实际采用的规划上限。
- selected_index 是最终候选；whitebox_index 是加神经网络分数之前的选择。每条 candidates 含 index、allowed、selectable、score、speed_mps、yaw_right_radps、clearance_m、path。allowed 表示该动作通过原白盒；selectable 表示当前控制阶段允许排序器或恢复逻辑选择该动作。受控恢复可独占某条已检查动作。没有候选时保留冻结驱动给出的控制和原因，两个索引可为空。
- road_rows 每行为 [前距,最左横距,最右横距,左侧误差,右侧误差]。selected_path、每条 path、lines[].points、edges.left/right 使用 [前距,右距] 米。lines[].kind 区分实线、虚线和证据不足。地图原点是本次驾驶开始位置，map.pose 为 [前距,右距,右偏航角]。map.parts 按 ego、adjacent 和 ego_mask 分层；map.changed 每行为 [前格,右格,本车道次数,相邻车道次数,本车道路面掩码次数,相对时间]，每格 0.2 m。map_semantics.changed_markings 给出相同格坐标和线型。map.static、map.dynamic、map.now 是冻结的静态位置估计、动态观测范围和当前观察。objects 保留规划器数值矩阵，tracks 提供人能读的 id / position / velocity / error_m 等。
- status 可能为 ok、blocked、invalid 或 late。blocked 表示运行时条件拦截输出，仍显示规划结果。图像超过 0.5 秒、时间异常、没有相应反馈或标定不匹配时，物理门收不到此帧的新控制。超过 0.5 秒的图像中断、更换相机/几何配置或规划计算中断后需要暂停并重启，以免沿用失效的历史。经过验证的有符号 0x711 轮速允许实车发送倒车指令。旧版无方向反馈仍会触发倒车互锁。

车端可采用本田安装位姿、现场光心高度 0.55 m 和实时 ZED 左相机 CameraInfo.P 投影。驾驶节点接受该配置 `honda-v1.4.0-ground`，在图像和反馈时效满足条件时向 `/concept/auto_cmd` 发布规划指令。冻结投影的焦距为 254.391622 像素，主点为 (330.01302,181.14964)，相机高 0.655 m。几何配置 unverified-resize 会触发输出互锁。calibrated-v626 能产生候选控制，frozen-unity-v626 仅在 ROS 使用仿真时钟时能产生候选控制。签发几何凭据的方式见 concept_yolop。物理上车还需单独验证动态响应、制动、通信延迟与轮速方向。

## 同帧核验

移动桶外圈的第一张归档图生成了 75 条轨迹，56 条通过白盒。白盒选第 42 条，网络为第 32 条的白盒分数 1.767989 加到 3.062521。最终控制为前进 1.87 m/s、向左转 0.196781084 rad/s。第 5 张图白盒负责控制，索引为空。test/data/moving-lap-start 保留这五张连续原图、同步识别与原始传感数据。参考决定仅供比较。

~~~bash
python3 -m pytest -q vehicle_concept/src/concept_drive/test
python3 vehicle_concept/src/concept_drive/tools/ros_roundtrip.py
python3 vehicle_concept/src/concept_drive/tools/replay_acceptance.py /path/to/eval-moving-lap/eval-moving-lap
~~~

最后一个命令接受多个包含 decisions.jsonl 和 frames/ 的原始档案，逐帧从 RGB、掩码、检测、测得的轮速和陀螺重建驾驶状态。完整档案的路径、分数、候选和控制差异见 REPLAY.json；它统计历史 Unity 驾驶记录的重放。四项旧正式验收为外圈移动桶整圈、双桶短段、混合短段、连续障碍短段。内圈整圈属于另存的诊断场。1,079帧中没有倒车帧。可用 ros_roundtrip.py 在开发机本地独立 DDS 域发送存档帧并验证真实 ROS 消息、重启、限速和未标定相机的影子输出。仿真时钟在这项联通测试中按帧推进，不能由此推算车载实时帧率。

PROVENANCE.json 列出冻结压缩包和各模块原始文件的 SHA-256，登记为 Foxy/Python 3.8 调整的兼容语法、原动态绑定与新包导入的对应关系。
