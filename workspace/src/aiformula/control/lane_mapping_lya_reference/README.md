# 第一圈建图，之后跑固定路线：保留 LYA 参考版

这个包做两件事：第一圈让现有 LYA 控制器带车跑，旁边用**完整车道 mask + VectorNav** 记下车道地图和车辆走过的顺序；停车后把它们整理成固定闭合路线，再跟踪这条路线。

**LYA 指令不是建图数据，也不是拿来训练地图。** 指令仅用于第一圈驾驶、第二圈安全参考和日志。第二圈不是重播第一圈的方向盘/速度命令，而是根据 **VectorNav + 当前 mask 对已有车道地图的匹配定位**，重新计算固定路线的跟踪命令。这是带外部定位先验的车道地图定位与受控增量建图，不是具有全局回环优化的完整 SLAM。

两个 ROS 包放在 `neo_aiformula_sophia/workspace/src/aiformula/control/` 下，与该仓库实际的 `trajectory_follower` 同级：

| 包 | 第一圈 | 第二圈 |
| --- | --- | --- |
| `lane_mapping_lya_reference` | 受管 LYA 驾驶，同时 mask + VectorNav 建图 | 固定路线为主；继续看 LYA 指令，差异超限或路线跟踪失效时回退至受限 LYA |
| `lane_mapping_fixed` | 同上 | 停车交接后关闭本次启动的 LYA 进程组，只跑固定路线 |

两包共享同一份建图、路线生成和控制代码，不复制算法。两种启动方式只能选一种运行。

## 先说清楚安全边界

默认仅向 `/lane_learning/cmd_vel` 发布，**没有接到电机**；默认 `DISARMED`，不会自动开车。本包不启动相机、VectorNav、CAN、电机或原来的全车 launch。

实车前必须完成这几项，而不是只把开关改为 `true`：

1. 确认手操硬急停确实绕过自主控制链路，能断驱动，并由安全员保持可操作。ROS Bool 急停是补充，不能替代它。
2. 确认电机接收到零速度、零角速度时真的输出零。当前 neo 电机源码已有完整停止命令的零 RPM 旁路，不能再把它描述为必然被仿射补偿改成非零；但源码检查不等于已验证车上安装版本、接线和实际停车。这两个新包没有修改电机控制器。
3. 确认电机端自身有断流停车机制。Python 回调、DDS、进程关闭或本机卡死都不是硬实时安全保证。
4. 停掉其他直接向同一电机命令话题发送的 LYA/控制器。这里只能停止本次 supervisor 启动的 LYA；不会搜寻或杀死其他用户进程。ROS 图检测能发现竞争发布者，但不能替代硬件互锁。
5. 检查实际车宽、路线安全边距、GNSS/INS 跨圈误差、VectorNav 安装朝向、相机标定和静态 TF。目前仍**假设天线至车体参考点的平移杆臂为零**。mask 匹配只能修正局部可观测、且在门限内的偏差；不能代替安装标定、修好错误地图，或保证全局重定位。

默认受管教师现在是 neo 实际使用的 `lya_0221`。它与第二圈固定路线默认继承唯一来源 `trajectory_follower/lya_profile.py` 的 `REFERENCE_SPEED_MPS`（原 `v_t`），不是在新包各处复制一个常数。本次源码值是 `2.0 m/s`；以后把该源值改为 4，新包的默认参考也随之为 4。LYA 原反馈律保留，例如本次 2.0 参考、正前方 1 m 误差、朝向相同时会输出约 `2.15 m/s`，不能说教师每帧恒定输出参考值。

第一圈 TEACH 以及参考版的 LYA FALLBACK 保留通过准入检查的原教师命令，不再暗中缩到 `0.8 m/s` 或附加新包的启动斜率限制。默认 `accepted_teacher_max_speed_mps=0` 只表示私有预览未指定线速度上限，**不是实车可无限速**；任何非私有输出必须同时具备三个部署确认和经过审核的正数教师上限，超限拒绝并停车，而非静默裁小。不能拿电机参数 `max_command_v=4.0` 当统一硬上限：当前 `STATE_BKUP` 路径绕开它。

第二圈固定的是本次运行所选的同源参考，不回放第一圈速度，也不按曲率偷偷生成慢速曲线；跟踪误差反馈、启动加速度限制和停车门限仍可能让实际指令不同于参考。原横向加速度限值 `0.35 m/s²` 不放宽，路线若无法在该参考速度下满足曲率/角速度/横向加速度限制就拒绝 ready/运行，而不是悄悄减速。因此某些弯道会明确不可行。这不是实车速度验收；HOLD/ESTOP 仍直接归零。

这里的 0.35 是 `v_ref² × |曲率|` 的名义参考可行性检查，**不是反馈后实际 `v × omega` 的严格上限**。repeat 角速度门限仍是 `0.4 rad/s`，没有随速度参考继承而提高；独立教师角速度准入 `accepted_teacher_max_yaw_rate_rps` 默认继承 LYA 当前 `MAX_YAW_RATE_RPS`（本次 2.0 rad/s）。这两个角色的范围不能混同。

`reference_speed_mps` launch 参数默认空，表示不覆盖源码当前默认值/配置；显式填写时统一覆盖 recorder、follower 和受管 LYA。也可在 `params_file` 用全局 `/**: {ros__parameters: {reference_speed_mps: 4.0}}` 为一次运行统一选择值，不能只改其中一个节点。repeat 的 `maximum_speed_mps=0` 是“使用本次有效参考”的内部哨兵，不是零速或另一个硬编码 2.0。改变参考不会自动获得实车许可，也不会放宽曲率、教师上限和停车门限。

继承发生在重新构建/启动时，运行中不热更新。改参考后，旧 bundle 的速度 policy/参考不匹配会被拒绝，必须重新生成并验证路线；不能偷偷切回另一 profile 降速使用旧结果。

## clone 和编译

目标环境是 **Ubuntu 20.04 + ROS 2 Foxy / Python 3.8**。先准备好系统 ROS 环境和车上原来的感知、定位、LYA 依赖。下面只编译明确列出的包，不让 colcon 遍历整个车辆项目。依赖安装需先获得车端环境修改授权；已有依赖时跳过 rosdep 安装。

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/lane_learning_ws/src
git clone --branch feat/causal-lane-teach-repeat \
  https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git \
  ~/lane_learning_ws/src/neo_aiformula_sophia
cd ~/lane_learning_ws

rosdep install --from-paths \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_lya_reference \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_fixed \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/trajectory_follower \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --ignore-src --rosdistro foxy -r -y

colcon build --symlink-install --base-paths \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_lya_reference \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_fixed \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/trajectory_follower \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --packages-up-to lane_mapping_fixed
source install/setup.bash
```

如果 `vectornav_msgs` 已在车上的已 source 工作空间中安装，可以省略上面两处 VectorNav 源码路径。neo 仓库的原生消息源码位于 `dependencies/vectornav/vectornav_msgs`，不是旧仓库的 sensing 目录。

`trajectory_follower` 是新包的实际依赖，不是只靠测试替身替代的可选项。上面包含它的源码构建路径，并须能在 `ros2 pkg executables trajectory_follower` 中找到 `lya_0221`。其 `tf_transformations`、`tf2_ros`、`tf2_geometry_msgs` 等运行依赖已在 package.xml 声明；安装仍须取得环境授权。当前修改只把原 `v_t` 提为共享参考参数，不更换 LYA 反馈律，也不启动其传感器/滤波输入链。

```bash
ros2 pkg executables lane_mapping_lya_reference
ros2 pkg executables lane_mapping_fixed
ros2 launch lane_mapping_lya_reference reference.launch.py --show-args
colcon test --packages-select lane_mapping_lya_reference lane_mapping_fixed
colcon test-result --verbose
```

### neo 接线修复与尚未验证的部分

2026-09-28 修正了迁移时暴露的接线问题：默认 mask 与 neo `topic_list.yaml` 一致；road detector 将输入的完整 Header 独立复制到 mask 和标注图；默认教师改为实际 `lya_0221`。严格 frame/标定检查保留，不能用猜测的 frame override 绕过。

`lya_0221` 仍需要 `/aiformula_sensing/gyro_odometry_publisher/odom`、`/filtered_lane_pose`、`/filtered_omega_t` 的真实发布者。supervisor 只管理自己启动的进程组，不能同时另起一个直发电机的 LYA。

新增 `test/neo_upstream_smoke.py` 会在私有 localhost/domain 启动真实已安装 LYA，检查继承当前参考及误差反馈、实际 STOPPED/子进程退出，并可用真实 cv_bridge/DDS 检查生产 mask 发布方法。**本次新增真实上游 DDS 结果待对应提交 CI 验证**；本地 Header 单测不等于模型推理/驱动/整车测试。以 [VALIDATION](VALIDATION.md) 和对应提交的证据为准。

## 需要什么输入

| 输入 | 默认话题/约定 |
| --- | --- |
| 完整二值车道 mask | `/aiformula_perception/road_detector/mask_image`，`mono8` 或 `8UC1`；不是 lane-line publisher 的 ROI |
| VectorNav | `/aiformula_sensing/vectornav/raw/common`，原生 `vectornav_msgs/CommonGroup` |
| 相机标定 | `/aiformula_sensing/zed_node/left/camera_info`，尺寸与 mask 一致 |
| 相机外参 | `base_footprint <- mask 对应相机 optical frame` 的全静态 TF 链 |
| LYA 原有输入 | 已运行的 gyro odometry、filtered lane pose、filtered omega 等；由原 LYA 自己订阅 |

VectorNav 必须提供 yaw/pitch/roll、LLA、NED 速度、INS 状态位；要求 Tracking 模式、GPS fix 且无时间/IMU/GPS 错误。航向由 `pi/2 - yaw_NED + yaw_offset_rad` 转成 ENU，停车时也能定位；不靠速度方向猜朝向。录制器和跟踪器使用相同 offset，结束后把原点与 offset 写入 bundle。

mask 只使用**当时已到达、且时间戳不晚于该图像**的 VectorNav；不插值借未来数据。全图投地后按像素投影敏感度筛选，不做 3 m ROI。默认地图网格 0.1 m，5 帧支持、其中至少 1 帧可靠投影才确认。建图候选、历史图、轨迹数量都有上限；达到容量导致失真风险时拒绝完成，而不是悄悄丢掉早期轨迹。

话题、frame 和 yaw offset 可用 launch 参数覆盖；其他参数在 `config/learning.yaml`。`params_file:=/绝对路径/learning.yaml` 可选择一份现场配置。对于 launch 明确暴露的参数，**launch 参数优先于 YAML**。

## 第二圈如何定位，以及如何继续降噪

第一圈仍以完整 mask + VectorNav 因果投影、多帧 consensus 建图。完成后冻结两份基准：车道参考地图与固定路线。第二圈并不关闭相机，也不只靠 GNSS 盲跑：

1. 每帧完整 mask 使用已保存的同一相机标定投影到地面，配对的 VectorNav 必须在该帧到达前已经收到，且传感器时间戳不晚于图像。
2. 以 VectorNav 为初值，把投影车道点对齐到冻结的参考车道图。鲁棒点到线匹配检查重叠、残差、几何可观测性和修正幅度。默认只修横向位置和朝向，每帧把沿车身前向的位置锚定到当前原始 VectorNav。平行线和近圆弧的沿线位置约束都可能不足，不能让上一帧修正悄悄累积成纵向漂移。全三自由度模式需要显式开启 `allow_longitudinal_correction` 且几何支持；默认关闭。
3. 通过检查后才接纳定位修正；新的 VectorNav 继续高频传播该修正。启动定轨迹还要求连续可信的 mask；图像断流、匹配拒绝或结果过期，两种版本都 HOLD 输出零。恢复输入不会自动重新开车。LYA 回退仅处理原有的控制分歧，不掩盖定位失效。
4. 只有已接纳匹配中的内点才进入独立的多帧降噪地图。默认在 10 秒窗口中取得至少 3 帧支持后，才累计到整圈增量图；已经确认的路段不会随窗口移出而消失。地图容量到达上限时会显式报告并停止接纳新格子，不淘汰已经确认的历史。**当前帧先定位、后更新**；该增量层不参与本轮匹配，也不移动固定路线、改写原 bundle。这样避免一帧错误同时污染地图和证明自身定位正确。

完整图像采用容量为 1 的待处理队列，忙时保留最新帧；匹配在独立 worker 中计算。结果提交前重新检查源时间、接收时间、当前定位和会话，过期计算不会更新定位或地图。20 Hz 命令与停车检查不在图像计算线程中执行；Python/操作系统仍不是硬实时保证。

默认 `visual_max_age_s=0.5`、图像与过去 VectorNav 最大间隔 `0.15 s`、连续成功帧数 `3`；VectorNav 自身仍必须满足 `0.25 s` 时效和 Tracking 健康检查。前者是视觉修正的新鲜度，不会把旧 VN 当成新定位。匹配参数通过 `visual_matcher_config_json` 提供，未知键或非法数值启动失败，不会静默采用另一个算法。

局部匹配默认最多使用 600 个降采样点、30 m 范围内的可靠投影，计算预算 40 ms。这是有界局部定位的范围，不是之前的 3 m 建图截断；原始建图仍按投影敏感度筛选完整 mask。超范围点数、预算超时和不可观测方向都会进诊断。

这是有界的局部匹配，不是任意位置的“找回自己”。初始定位偏差过大、地图无覆盖、车道消失或错误标定会拒绝运行。没有完成相机/地图标定与 consensus 哈希的旧测试 bundle 也会被拒绝；真实 recorder 保存的完整 bundle 包含这些信息。

## 第一圈到第二圈，按顺序操作

以下默认命令是隔离输出/台架演练。先启动车上现有传感器与感知节点，但不要再独立启动一个会直发电机的 LYA。

```bash
ros2 launch lane_mapping_lya_reference reference.launch.py
```

它启动 recorder、command selector 和一个受管 LYA。supervisor 把旧 LYA 的电机命令话题重映射到 `/lane_learning/lya_cmd`；只有 selector 决定最终输出。不要同时再运行固定版 launch。

另一个已 source 的终端：

```bash
ros2 topic echo /lane_learning/control_state
ros2 topic echo /lane_learning/recorder_state
```

1. 确认定位和 LYA 指令新鲜、无报错。显式允许第一圈跟随 LYA：

   ```bash
   ros2 service call /lane_fixed_follower/arm std_srvs/srv/Trigger '{}'
   ```

2. 跑一圈回到起点附近后，先切 HOLD 并确认实车停稳：

   ```bash
   ros2 service call /lane_fixed_follower/stop std_srvs/srv/Trigger '{}'
   ```

3. 停车且 VectorNav 持续新鲜后，结束第一圈并生成路线：

   ```bash
   ros2 service call /lane_lap_recorder/finish_lap std_srvs/srv/Trigger '{}'
   ```

   成功会返回 `bundle.json` 绝对路径。闭合、车道支持、宽度、曲率、自交等检查不通过就不发布 ready；不能靠改文件里的 `ready` 绕过校验。录制失败会留诊断与快照，重新录一圈需重新启动 recorder。

4. 检查生成路线与车道图，确认起终点连接和车辆所在位置。第一次调用 `start_repeat` **只加载冻结路线并保持 HOLD**：

   ```bash
   ros2 service call /lane_fixed_follower/start_repeat std_srvs/srv/Trigger '{}'
   ```

5. 等待新坐标原点下的定位、连续可信 mask 匹配和连续停稳证据到达。参考版还要求新的 LYA 参考命令。再次调用同一服务，成功后才进入 `REPEAT`：

   ```bash
   ros2 service call /lane_fixed_follower/start_repeat std_srvs/srv/Trigger '{}'
   ```

参考版把 LYA 当作安全参考，比较目标指令而非启动 ramp，并在相同比较速度下核对转向。确实分歧或固定控制拒绝时进入 `FALLBACK`，保留通过教师准入检查的原 LYA 指令；它不是“只取较慢者”的仲裁器，回退可能比固定候选快，所以上车前必须审核教师上限。LYA 的零速命令立即停车。**恢复一致也不会自动切回固定路线**，需一致维持恢复窗口，再由操作员调用：

```bash
ros2 service call /lane_fixed_follower/resume_reference std_srvs/srv/Trigger '{}'
```

定位过期、参考 LYA 断流、输出发布者竞争、日志故障等会 HOLD/ESTOP，不把它们当成可以继续开车的普通回退。`stop` 后只会停；不会自动重新 arm。

## 已有路线，不再重新建图

```bash
ros2 launch lane_mapping_lya_reference reference.launch.py \
  record:=false route_bundle_path:=/绝对路径/finished_xxx/bundle.json
```

仍需两次 `start_repeat` 完成加载、停稳、显式启动。保留受管 LYA，因为参考版第二圈需要它；固定版也需要在交接时确认它已停止。`manage_teacher:=false` 只用于你已明确管理私有 LYA 的测试场景；固定版默认不会在缺失停机确认时放行。

bundle、route、metadata 是一组文件。不要手改坐标原点或单独拷贝 route；哈希校验会拒绝损坏或不一致的组合。

## 实车输出必须显式打开

只有上面的物理验证实际完成，且现场配置已明确设置经过审核的正数 `lane_fixed_follower.ros__parameters.accepted_teacher_max_speed_mps` 后，才可以选择实车话题并声明三项条件。这里不代替现场选择一个未经批准的数值：

```bash
ros2 launch lane_mapping_lya_reference reference.launch.py \
  params_file:=/绝对路径/已审核的learning.yaml \
  command_output_topic:=/aiformula_control/game_pad/cmd_vel \
  enable_vehicle_output:=true \
  motor_zero_passthrough_verified:=true \
  hardware_stop_verified:=true
```

这些参数只是操作员声明，**不会替你测试急停、电机零指令或遥控所有权**。`accepted_teacher_max_speed_mps=0` 在非私有输出下必须拒绝，即使三个开关均为 true。仍需显式 `arm`；不要把验证开关写成无人检查的默认启动项。

软件急停（仍须保留手操硬急停）：

```bash
ros2 service call /lane_fixed_follower/estop std_srvs/srv/Trigger '{}'
```

也可由遥控桥接节点发布 `std_msgs/Bool` 到 `/lane_learning/emergency_stop`。True 锁定急停；撤销 True 本身不会恢复驾驶，必须确认安全后调用 `reset_estop`，再显式 arm/恢复。默认急停订阅使用 reliable/volatile，桥接器的 QoS 需要匹配；没有新增自动配置遥控器的逻辑。

## 看图和查日志

RViz Fixed Frame 设为 `lane_map`。第一圈看 `/lane_lap_recorder/consensus`（PointCloud2）与 `/lane_lap_recorder/trajectory`（Path）；第二圈看 `/lane_fixed_follower/route`（Path）和 `/lane_fixed_follower/pose`（mask 修正后的 PoseStamped）。地图/路径是 reliable + transient local；实时 pose 是 best effort。历史地图不是实时避障层。

第二圈还提供 `/lane_fixed_follower/raw_pose`（原始 VN）、`anchor_consensus`（冻结参考图）、`refined_consensus`（多帧降噪层）及 `visual_localization`（JSON 状态与匹配指标）；后三个名称同样位于 `/lane_fixed_follower/` 下。RViz 的俯视尺度固定，可同时对比原始和修正位姿，不随地图范围自动缩放。

已带固定尺度俯视配置，可在安装了 RViz2 的车端打开：

```bash
rviz2 -d "$(ros2 pkg prefix lane_mapping_lya_reference)/share/lane_mapping_lya_reference/rviz/learning.rviz"
```

CI 不代替 RViz GUI 的车端显示验收。

默认根目录 `~/.ros/lane_learning`，每次运行建立带时间和随机 ID 的新目录：

- `maps/<run>/trace.csv`：所有接收通过检查的 VectorNav 位姿与原始 LLA。
- `commands.csv`：观察到的 LYA 指令；`masks.csv`：图像、所用过去位姿、融合统计和耗时；`events.jsonl`：状态和拒绝原因。
- `finished_*/consensus.csv`、有序 `trace.csv`、`route.json`、`metadata.json`、`bundle.json`：可复用地图、路线、原点、标定、参数和源码/数据 SHA256。
- `follower-*/commands-*.jsonl`：每次输出命令、选择状态、位姿/参考时效与跟踪指标；有界队列，按文件轮换。
- 同一 follower 日志还包含源码 SHA256、实际匹配配置、mask 配对时间、匹配前后位姿、残差/覆盖/拒绝原因。正常退出时另存 `refined_consensus.csv` 与 `refinement_metadata.json`，标明是历史增量层及其原始地图/路线来源；不覆盖原 bundle，也不自动作为下一次的驾驶基准。
- `teacher_*/teacher.log*`：LYA 标准输出、生命周期和受管进程退出记录；按大小轮换。

全圈源码/数据快照不覆盖旧结果。持续运行日志会在**各自的新运行目录内部**轮换旧片段，长测试前请安排外部归档。另有 ROS 自身的 `~/.ros/log`。

测试通过证明的是被测范围内的逻辑/通信，不证明真实赛道定位精度或电机安全。实车验收应先架空/断动力确认命令所有权、零指令与急停，再在封闭场地低速检查两圈定位一致性和路线跟踪误差。

## 离线定位与降噪验证

`scripts/mask_localization_replay.py` 不启动 ROS 或输出车辆命令，输出到包内新建的 `.artifacts/mask_localization_<UTC>_<ID>/`，不覆盖旧实验。额外的离线依赖是 `rosbags`（读取真实 bag）和 `matplotlib`（作图）；不是节点运行依赖。

```bash
# 已知定位漂移与图像噪声的合成实验，包含真实误差和地图质量。
python3 scripts/mask_localization_replay.py --synthetic-only

# 真实 bag：仅以前 60 秒数据建参考图，之后匹配，不读取未来位姿或整圈地图。
python3 scripts/mask_localization_replay.py \
  --bag /绝对路径/rosbag2_2026_01_20-15_31_07 \
  --vectornav-msg-dir /绝对路径/vectornav_msgs/msg
```

真实单圈实验不是独立第二圈，也没有定位真值。离开前缀地图覆盖、地图不具线特征或匹配不可信时，必须如实记录拒绝。不得把“重复播放同一圈”包装成独立跨圈定位精度，也不得用匹配残差代替真实位置误差。该离线脚本检查算法和输入时序；运行节点另外要求停车采集、连续可信帧以及完整的安全状态机。

本次改动的已知效果与失败边界见 [VALIDATION.md](VALIDATION.md)。目前真实 bag 的连续定位未通过，不能据此启用实车定轨迹驾驶。
