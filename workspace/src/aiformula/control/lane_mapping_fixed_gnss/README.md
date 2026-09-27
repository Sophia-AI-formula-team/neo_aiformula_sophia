# lane_mapping_fixed_gnss

第三个独立 ROS 2 包，位于 `neo_aiformula_sophia/workspace/src/aiformula/control/`，与实际 `trajectory_follower`、原有两个 lane_mapping 包同级。
旧包保持不变。本包复用 `lane_mapping_lya_reference` 的纯算法、安全控制、日志和 LYA 进程管理；不实例化它的 VectorNav 定位节点。

## GNSS 的使用边界

GNSS **只用于定轨迹起点的粗定位，不用于建图**。只在 LYA **开始前停稳**、**结束后停稳** 两个显式窗口，各使用一条新到达、质量合格的原始 GPS 定位。收到后立即销毁订阅。窗口最长 2 秒；GNSS 无效只关闭定轨迹启动许可，**不阻止建图，不作废地图**。

- LYA 开启时：可选记录“定轨迹起点”的 GNSS 参考，单独存入运行日志，不进入地图文件。局部建图原点在显式 `begin_teach` 时由纯轮速/陀螺仪置零，与 GNSS 是否存在无关。
- LYA 建图全程：CAN 实测轮速 + 原始角速度积分 + 当前 mask 对过去 consensus 的匹配；**不读取 GNSS、INS 位置、融合速度或 orientation 四元数**。
- LYA 结束时：独立冻结/构建地图，同时单独采 GNSS 检查车辆是否处于“定轨迹起点”附近。不平移、不旋转、不把定位差分摊回地图。没有 GNSS 也可以生成几何有效的地图。
- 定轨迹：纯轮速/角速度先验 + mask 对冻结地图定位；独立 consensus 层持续降噪，不修改已审核的固定路线。没有 GNSS 订阅，也不回退到 GNSS 或 LYA。

两个静止 GNSS 位置不能确定初始朝向。因此地图坐标 `lane_teach_local` 是起跑时车头向前 x、向左 y、逆时针 yaw；**不是已知朝向的 ENU 地图**。GNSS 只粗筛“是否回到定轨迹起点附近”，精确地图位置和朝向还须 mask 匹配通过；不声称单靠 GNSS 已完成全局重定位。这仍不是完整的图优化 SLAM。

起点参考采样后应在 30 秒内、保持原位开始 `begin_teach`。若期间已移动超过 0.1 m、转动超过 0.15 rad 或运动链异常，该参考不能再代表定轨迹起点；地图仍可独立建立，但不能据此放行 repeat。重置局部原点后，首条新轮速也受 200 ms 断流保护，没有“等多久都可以重新从零开始”的空窗。

## 已核对的本田项目接口

依据 neo 基线的 Honda 派生源码 `workspace/src/aiformula/sensing/odometry_publisher/include/odometry_publisher/wheel.hpp`、`workspace/src/aiformula/vehicle/config/wheel.yaml` 和 [AI Formula 官方支持仓库](https://github.com/aiformula-support/aiformula)：

| 输入 | 默认话题 / 实际使用字段 |
|---|---|
| CAN | `/aiformula_sensing/vehicle_info`，`can_msgs/Frame`，ID 1809 (0x711)，左 `data[4]`、右 `data[0]`；轮径 0.254 m |
| 原始陀螺仪 | `/aiformula_sensing/zed_node/imu/data_raw`，只读取 `angular_velocity`，经已标定安装旋转转入车体系 |
| 完整 mask | `/aiformula_perception/pub_mask_image`，mono8；不是 lane publisher ROI |
| 内参 / 外参 | CameraInfo + 已到达的固定相机安装 TF |
| GNSS，仅端点 | `/aiformula_sensing/vectornav/raw/gps`，`vectornav_msgs/GpsGroup` 的 `fix`、`poslla`、`posu` |

轮速协议只证实无符号低 8 位前进 RPM；高字节非零、倒车、不认识的协议不能偷偷按有符号解析。本包不依赖旧 `gyro_odometry_publisher`：旧实现使用 ZED 姿态和前后 IMU 插值，不符合本包“仅角速度、只用过去”的契约。

ZED 可能在带时间戳的 `/tf` 发布固定 IMU 安装标定。本包启动时验证至少三个不同时间戳的相同矩阵后锁定；它不是运行姿态。也可设置 `gyro_mount_quaternion: [x,y,z,w]` 为实测 `base_from_imu` 单位四元数，默认全零表示从 TF 标定，**不能猜旋转轴**。`gyro_bias_radps` 应由停稳测量标定，默认 0 不等于确认无偏置。

### 必须修正传感器输出配置，不能换名冒充 GPS

neo 的 `dependencies/vectornav/vectornav/src/vectornav.cc` 中 `BO1.gpsField` 默认仅 FIX/POSU（0x210），**没有 GPS 经纬度**；实际 YAML/启动覆盖仍须现场核对。经授权在驱动启动时附加本包 `config/vectornav_endpoint_fields.yaml`（`BO1.gpsField=560`，即 0x230），保留原串口和其他设置。该参数是驱动启动配置，不是 follower 的参数；本包不启动或重配驱动。

没有采用旧 `vn_sensor_msgs` 的 NavSatFix：本地转换器使用 CommonGroup 的 INS position，并没有把构造的 covariance 赋给消息。新包使用原始 GpsGroup：严格要求 3D fix、字段有效，NED `posu` 标准差平方并重排为 ENU covariance。默认每点水平 sigma ≤1.5 m、首尾距离 ≤3 m；这是**验收门限，不是定位精度承诺**，不能把阈值放宽后称作高精度。

## 构建与运行（Ubuntu / ROS 2 Foxy）

需要 `vectornav_msgs`、`can_msgs` 和实际已安装的 `trajectory_follower` 及传感器发布者；本包不随启动安装它们。先按[参考包 clone 命令](../lane_mapping_lya_reference/README.md#clone-和编译)取得 `neo_aiformula_sophia`，不要单独复制一个目录遗漏共享算法。依赖安装需先取得环境修改授权；已有依赖可跳过 rosdep。

```bash
source /opt/ros/foxy/setup.bash
# 如已有车辆 overlay，先 source 其经过核对的 install/setup.bash。
cd ~/lane_learning_ws
rosdep install --from-paths \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_lya_reference \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_fixed_gnss \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install --base-paths \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_lya_reference \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_fixed_gnss \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --packages-up-to lane_mapping_fixed_gnss
source install/setup.bash
ros2 launch lane_mapping_fixed_gnss fixed_gnss.launch.py
rviz2 -d "$(ros2 pkg prefix lane_mapping_fixed_gnss)/share/lane_mapping_fixed_gnss/rviz/mapping.rviz"
```

### neo 基线的已知部署阻塞

- 默认 LYA executable 在 neo 中存在，电机话题重映射仍匹配；但其输出允许到 `4.0 m/s`、`0.45 rad/s`，本包仍拒绝超过原教师准入 `2.25 m/s`、`0.4 rad/s` 的命令。迁移未修改 LYA、安全参数或控制算法，超限会 HOLD，需要单独审核兼容方案。
- neo 的 `road_detector.publish_result` 没有保留 mask 的 `header.frame_id`。本包严格要求完整 mask 带真实 optical frame；空 frame 会拒绝建图/定位。需要在上游正确传递原始 header 并核对 CameraInfo/固定 TF，不能伪造 frame 或放松准入。本包没有 frame override。
- 受管 LYA 自己仍依赖原有 gyro odometry、filtered lane pose 和 filtered omega 发布者；新包自己的 raw gyro 里程计不会冒充该输入。

上述是实车集成阻塞，不影响只用合成原生消息的独立 DDS 测试。CI 通过不等于这些现场问题已解决。

默认只向 `/lane_learning_gnss/cmd_vel` 输出，不连接电机；启动后不自动开车。不要同时启动旧两包的 command selector 或第二个 LYA。launch 启动的 LYA 只向私有 `/lane_learning/lya_cmd` 发布。

实车接管前必须核实独立手操急停、下游超时看门狗以及**零指令确实透传到电机**。现有仿射电机补偿可能把零输入映射成非零，不能以“已经发零”代替物理停车验证。要绑定真实 command topic，必须显式同时打开 `enable_vehicle_output`、`hardware_stop_verified`、`motor_zero_passthrough_verified`；实际 remap 后的输出话题也检查。软件安全门限不是硬件急停替代品。

## 操作顺序

1. 保持车辆静止，等待连续轮速/原始角速度至少 1 秒。若要接着跑定轨迹，调用 `capture_start` 保存起点参考。GNSS 不可用仍可建图。
2. 调用 `begin_teach` 独立初始化局部建图原点，等待新原点下至少 1 秒停稳证据，再 `arm_teach`。核实急停后以 `hardware_stop_verified:=true` 启动的节点才允许 arm。LYA 驾驶第一圈，RViz 黄色点为实时确认的 consensus。
3. 回到起点，调用 `stop`，实际停稳至少 1 秒，再调用 `finish_lap`。地图 bundle 只按几何质量独立生成；第二次 GNSS 只判断能否进入定轨迹起点定位流程。
4. 地图/固定路线审核通过、首尾 GNSS 起点邻近检查通过后，调用 `prepare_repeat`：关闭本 launch 管理的 LYA，并等它的新 STOPPED 心跳。车辆停稳、连续三次可信 mask 匹配后，显式调用 `start_repeat`。缺 GNSS 时保留地图，但本 `_gnss` 版本不放行 repeat。
5. 手操急停随时有效；软件 `estop` 锁存，释放手操后还要显式 `reset_estop`，不会自动继续。

```bash
ros2 service call /lane_endpoint_follower/capture_start std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/begin_teach std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/arm_teach std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/stop std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/finish_lap std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/prepare_repeat std_srvs/srv/Trigger '{}'
ros2 service call /lane_endpoint_follower/start_repeat std_srvs/srv/Trigger '{}'
```

不能重启节点后直接加载旧图开车：两次 GNSS 不能恢复未知局部朝向和轮速里程计 epoch。本版只支持**同一连续进程里的 teach → repeat**，重启或运行中原始运动输入异常需重新建图。长直道的沿路方向仍可能不可观，mask 不是万能定位；匹配不可信就 HOLD，不能靠 GNSS 暗中补回。

## 因果、runtime 与日志

轮速只配已经到达且时间戳不晚于它的原始 gyro；mask 只配已到达且不晚于图像的 pose。没有未来插值。建图匹配先使用过去确认地图，然后才允许本帧投票；bootstrap 有时间/距离/帧数上限，匹配失败不会退回错误先验硬融合。

独立 bounded worker：一个运行任务、一个最新待处理 mask。控制为 20 Hz steady timer；过期结果、失效 generation、输入断流、移动中的首次定位均不能提交。图像、栅格、轨迹、队列、会话时长有界；预算是软实时保护，不是硬实时证明。`begin_teach` 后原始运动链异常锁存本次会话无效，不允许补造漏掉的运动继续驾驶；此限制不是由 GNSS 质量决定的。

`~/.ros/lane_learning_gnss/follower-<unique>/` 保存轮速/gyro源时间、GNSS起点参考/结束定位、状态转移、每帧匹配拒绝原因、输出命令和旋转 JSONL。日志写入失败 HOLD。成功地图只包含 route、consensus、trace、calibration、metadata 和逐文件 SHA256，**不含 GNSS 坐标或 anchors 文件**；构建前后的冻结几何 hash 必须一致。旧结果不覆盖。

## 验证边界

本地纯算法测试、真实 bag 输入审计、原生 Foxy DDS 验证分别记录，不混称实车验证。现有 `2026_01_20-15_31_07` bag 的原始 GPS 全部缺 POSLLA，不能证明本包真实 GNSS 首尾流程；不使用 INS position 伪造缺失值。CI 的合成端点只能验证软件隔离与时序，不证明现场卫星精度、轮胎打滑误差或车辆闭环安全。

运行本包测试：`python3 -m pytest test -q`（需先 source 安装的共享算法）；CI 脚本在独立 Foxy 环境构建三包并做 DDS 通信，绝不启动实车硬件。
