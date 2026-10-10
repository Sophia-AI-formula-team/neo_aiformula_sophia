# lane_mapping_fixed_gnss

第三个独立 ROS 2 包，位于 `neo_aiformula_sophia/workspace/src/aiformula/control/`，与实际 `trajectory_follower`、原有两个 lane_mapping 包同级。
本包复用 `lane_mapping_lya_reference` 的纯算法、安全控制、日志和 LYA 进程管理；不实例化它的 VectorNav 定位节点。三个包的共享接线/参考速度修复不改变这里独立的 CAN + raw gyro 运动链，旧两个包仍是 VectorNav CommonGroup 链，不能混用验收结论。

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
| 原始 IMU | `/aiformula_sensing/zed_node/imu/data_raw`；默认只读 `angular_velocity`，下述显式三轴模式另读原始 specific force，均经同一已标定安装旋转转入车体系 |
| 完整 mask | `/aiformula_perception/road_detector/mask_image`，mono8；不是 lane publisher ROI |
| 内参 / 外参 | CameraInfo + 已到达的固定相机安装 TF |
| GNSS，仅端点 | `/aiformula_sensing/vectornav/raw/gps`，`vectornav_msgs/GpsGroup` 的 `fix`、`poslla`、`posu` |

轮速协议只证实无符号低 8 位前进 RPM；高字节非零、倒车、不认识的协议不能偷偷按有符号解析。本包不依赖旧 `gyro_odometry_publisher`：旧实现使用 ZED 姿态和前后 IMU 插值，不符合本包“仅角速度、只用过去”的契约。

ZED 可能在带时间戳的 `/tf` 发布固定 IMU 安装标定。本包启动时验证至少三个不同时间戳的相同矩阵后锁定；它不是运行姿态。也可设置 `gyro_mount_quaternion: [x,y,z,w]` 为实测 `base_from_imu` 单位四元数，默认全零表示从 TF 标定，**不能猜旋转轴**。`gyro_bias_radps` 应由停稳测量标定，默认 0 不等于确认无偏置。

### 实验性、显式启用的原始三轴倾角模式

默认 `motion_attitude_mode: legacy_flat` 保留原平面链。`raw_imu_tilt` 是新增的
仿真可验证算法路径，**不是实车 IMU 校准或部署放行**；不改变车辆、相机或仿真参数。
启用前必须核对实际 `gyro_topic` 的发布者、单位、轴向、安装旋转及时间来源，再明确设置：

```yaml
lane_endpoint_follower:
  ros__parameters:
    motion_attitude_mode: raw_imu_tilt
    imu_specific_force_verified: true
    imu_source_stamp_semantics: driver_receipt_ros  # 或经核验的 hardware_sample
    gyro_bias_radps: 0.0
```

这些声明没有默认“已核验”值；漏填/未知值或非零标量 bias 会拒绝启动。
`true` 表示调用方已确认 `linear_acceleration` 是包含重力的原始 specific force，
静止、正确车体系下方向向上、单位 m/s²；它不是程序自动测得的证明。模式不会换话题，
不会把 VectorNav 重命名成 ZED，也不会将 ZED 挂载外参用于另一颗 IMU。
如使用 VectorNav，须另核对原始字段有效标志、converter 的 ENU 轴变换、消息 frame
以及真实 `base_from_imu`；不会读取任何 `orientation`、INS 位置/速度或 GNSS 建图。

`hardware_sample` 要求 header 已按经验证的时钟关系表示 ROS 时间域内的采样时刻；
`driver_receipt_ros` 则明确只表示驱动接收时刻，不能声称硬件同步。当前 VectorNav
驱动默认 `adjust_ros_timestamp=false` 时属于后一种，需要现场确认实际配置。
节点的 monotonic receipt 是**串行 ROS 回调取得所有权时刻**，不是 DDS 入队或传感器
采样时刻；上游延迟仍须测量，不能靠此声明证明相机/IMU 已同步。

启动必须持续停稳并发布零输出。初始化同时要求过去轮速、当前原始 IMU 稳定性和
本节点 DISARMED/HOLD/ESTOP 下已发布的零命令。零命令仅是命令证据，不证明车没动，
更不能证明本节点拥有外部电机控制权。新增初始化门槛均是**未实测工程假设**：

| 初始化检查 | 默认值 |
|---|---:|
| 连续窗口 / 最少样本 / 最大保存样本 | 1.0 s / 20 / 512 |
| specific force 模长 | 9.80665 ± 0.5 m/s² |
| 单样本相对窗口均值的最大加速度偏差 | 0.05 m/s² |
| 单样本角速度模长 / 对均值最大偏差 | 0.02 / 0.002 rad/s |
| R9 初始化端点方向/横向 bias 一致性角度预算 | 2.5e-5 rad |
| 车头水平投影平方模最小值 | 0.25 |

不满足静止/稳定窗口就丢弃候选窗口并继续等待，不回填早期姿态。
R9 在原有幅度检查之后，按 source 时间将候选窗口分成四个不重叠区间，每段至少
两条观测；缺少区间覆盖就继续等待，不能重标 source 或重置等待起点。前三段估计
一个固定 gyro bias，最后一段不参与 bias 估计，作为独立检查段；通过后也不会偷偷
改用全窗 gyro 均值。用该固定 bias 和过去角速度的零阶保持积分，把每条 specific-force
方向传到**窗口末 source 时刻**，然后才求最终 up；不把窗口中间的平均方向标成末时刻。
四段端点方向中心的最大两两夹角、检查段相对训练中心的夹角，以及训练/检查段
gyro 均值差在重力正交方向上的模长乘窗口时长，都必须不超过 `2.5e-5 rad`。
失败清除候选窗口，保持未就绪；失败指标与不重叠 source 范围留在初始化日志。

这个新增数值是**更严格的算法工程预算，不是本次数据拟合的实车噪声标定**。
其条件性设计依据是：相机高至少 0.5 m、作用距离至多 30 m 时，将 0.1 m 地图网格的
半格 0.05 m 分配给初始化方向一致性；平面最坏 pitch 关系
`(h²+R²) tan(θ) / (h−R tan(θ))` 在上述边界和 θ=2.5e-5 时约为 0.04508 m。
0.5 m 是设计包络，不是实测车高；代码没有改变或验证相机高度/30 m 范围，也没有
新增 ROI。包络外不声称这个米制误差界。分段一致性不能证明绝对倾角正确，
不能把这项检查当成真实传感器精度或全程漂移保证。

窗口合格才生成末时刻 up 和三轴静止 bias；常值旋转与陀螺 bias、水平加速度偏置与
倾角本身不可仅靠该窗口区分。初始化后只用**上一次**原始角速度减 bias 作因果
Rodrigues 传播，不用行驶中的加速度低通作“重力修正”，加速度仍检查格式/有限性。
角速度积分会漂移，初始化不是无偏或无限时长有效性的保证。

每条 wheel 绑定同一过去 gyro 的原始 source/receipt 与不可变倾角快照；每张 mask
只能从选中的过去 wheel/gyro 快照传播，不能拿随后到达的姿态替换。原 150 ms gyro、
200 ms receipt/断流及 500 ms mask 门限不变；重复预测不会刷新原始样本年龄。
`begin_teach`、arm、prepare/start repeat 和地图提交都要求对应倾角已就绪。
置零局部原点不重置 bias/倾角或源时间前沿；就绪后传感器/时间错误锁定 HOLD，
必须重启并重建图，绝不自动退回平面模式。GNSS 两个端点窗口保持原语义。

投影使用同一完整 mask、固定 K/安装外参及捕获时倾角，输出车头水平投影坐标；
仍假设地面经过 base 原点。未知底盘离地偏移、道路坡度/横坡、安装/时钟误差不能
靠重力方向消除。像素 J4 仍只是像素扰动敏感度，不包含倾角不确定度。
理想仿真 IMU 和纯 Python/回调测试不等于真实传感器、DDS、实车或整圈验收。

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
  src/neo_aiformula_sophia/workspace/src/aiformula/control/trajectory_follower \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install --base-paths \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_lya_reference \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/lane_mapping_fixed_gnss \
  src/neo_aiformula_sophia/workspace/src/aiformula/control/trajectory_follower \
  src/neo_aiformula_sophia/dependencies/vectornav/vectornav_msgs \
  --packages-up-to lane_mapping_fixed_gnss
source install/setup.bash
ros2 launch lane_mapping_fixed_gnss fixed_gnss.launch.py
rviz2 -d "$(ros2 pkg prefix lane_mapping_fixed_gnss)/share/lane_mapping_fixed_gnss/rviz/mapping.rviz"
```

### 当前 neo 接线与速度约定

**继承数值不等于提高已有准入门限。** raw-motion 原有速度门限是
`motion.DEFAULT_CONFIG.max_speed_mps=3.0`；有效 LYA 参考若为 4，本节点会启动失败并明确说明
4 与 3 的冲突，不改成 3、不擅自提高轮速门限，也不等开车后才中断整圈。要实际运行更高速度，
先由现场测量和 planner 审核运动准入及控制可行性；这不是车辆物理极速的证据。

- 默认受管教师为实际 `lya_0221`；它与固定路线默认继承 `trajectory_follower/lya_profile.py` 的 `REFERENCE_SPEED_MPS`（原 `v_t`）。本次源码值 2.0，不是新包另抄的固定常量：源改为 4，默认随之继承 4。原 LYA 反馈律未改，因此 2.0 参考加正前方 1 m 误差时可能输出约 2.15。
- 第一圈保留通过准入的原 LYA 命令，不再缩到 0.8 或追加新包启动 slew。第二圈固定本次所选参考，不回放第一圈速度，也不按曲率暗中减速；原 `0.35 m/s²` 横向限值不放宽，不可行路线拒绝生成/运行。跟踪反馈、启动加速度限制、故障停车仍保留。
- 0.35 是 `v_ref² × |曲率|` 的名义参考检查，不是反馈后实际 `v × omega` 严格上限。repeat 角速度仍限 0.4 rad/s；独立教师角速度准入继承 LYA 当前 `MAX_YAW_RATE_RPS`（本次 2.0 rad/s），两者不混同。
- `reference_speed_mps` launch 默认空，不覆盖当前源码/配置；显式值统一作用于本包和受管教师。`params_file` 也传给 supervisor，可通过全局 `/**.ros__parameters.reference_speed_mps` 统一覆盖。repeat `maximum_speed_mps=0` 表示使用有效参考，不是硬编码 2.0。
- 默认值在重新构建/启动时读取，运行中不热更新。修改参考后，旧 bundle 的速度 policy/参考不匹配就拒绝，需重生成并验证路线；不能换 profile 偷偷降速使用旧结果。
- road detector 已修复完整 Header 传递；本包仍严格检查真实 optical frame、CameraInfo 尺寸和固定 TF，不提供 frame override、不猜 frame。
- 受管 LYA 仍依赖原有 gyro odometry、filtered lane pose、filtered omega；本包自己的 raw gyro 里程计不替代它的输入。

真实安装的 LYA 与生产 mask 发布方法新增独立 DDS 集成测试；本次结果待对应提交 CI。Header 本地单测、历史合成 DDS 或编译通过都不是模型推理、全车接线和实车验证。

默认只向 `/lane_learning_gnss/cmd_vel` 输出，不连接电机；启动后不自动开车。不要同时启动旧两包的 command selector 或第二个 LYA。launch 启动的 LYA 只向私有 `/lane_learning/lya_cmd` 发布。

实车接管前必须核实独立手操急停、下游超时看门狗以及**零指令确实透传到电机**。当前 neo 电机源码已有完整停止命令的零 RPM 旁路，但不能以代码存在代替车上验证。要绑定真实 command topic，必须同时明确三个部署确认开关，并为 `accepted_teacher_max_speed_mps` 设置经过审核的正数上限。默认 0 只允许私有预览，不是实车无限速许可；也不能把电机 `max_command_v=4.0` 当统一硬上限，因为 `STATE_BKUP` 绕开它。

本包现在在 arm/prepare/start_repeat 前及每个控制 tick 检查实际 resolved/remapped 输出话题的 publisher。竞争发布者或 graph 查询失败会发零并 HOLD，竞争者消失不会自动恢复。**这不是遥控/自主仲裁器，不能阻止别人的消息到达电机**；真实遥控所有权、DDS 发现延迟和硬件急停仍待现场验证。

## R7 候选：视觉与相对运动共同求解

此前轮速/陀螺主要给视觉匹配初值，优化目标本身只有视觉残差。因此小的视觉
方向偏差可能沿着“旧地图—新定位—新地图”循环积累。R7 在共享定位器中增加
横向/航向的相对运动信息项，本包 worker 始终传入真实配对运动的源时间戳，
不是图像时间或接收时间；不增加 GNSS、INS 姿态、真值或未来观测。

运动项围绕上一次已提交定位加当前 raw-motion 增量，不把总校正拉回零。
信息矩阵按真实源时间间隔传播，视觉按整帧相关证据归一；不会把同一条漆线
上的数百像素冒充数百个独立传感器。视觉可观测性、覆盖、残差、创新、累计
修正及时间门限仍单独检查，运动项不能把缺失/不合格的 mask 变成有效定位。
首次运动参考采集仍是原视觉匹配，通过后初始化信息状态；后续才融合。
状态只在原提交检查通过后更新；候选 keyframe、拒绝帧和超时不能推进它。
第一圈到第二圈保留同一信息状态，不重设置信度或重新开放初次定位门限。

以下是当前候选的**算法工程假设，不是实车标定精度**，也不是车辆/仿真参数：

| 共享 matcher 参数 | 候选默认值 |
| --- | --- |
| `motion_visual_frame_sigma_m` | 0.05 m |
| `motion_lateral_noise_m_sqrt_s` | 0.05 m/√s |
| `motion_yaw_noise_rad_sqrt_s` | 0.002 rad/√s |
| `motion_initial_lateral_sigma_m` | 0.05 m |
| `motion_initial_yaw_sigma_rad` | 0.02 rad |
| `motion_visual_reference_period_s` | 0.1 s |

开发试验最初使用横向 0.1 m/√s，28 项独立合成检查中 2 项单侧偏置航向误差
超标。只比较一个预定替代值 0.05 后，原输入与原门限下 28 项通过；单侧航向
改善同时横向误差略增，原失败记录保留。该选择来自合成开发试验，不是原参数
通过、实车噪声标定或整圈验收；未使用 trial25 真值拟合参数。

这是在当前 raw-forward 固定约束下的二维短步信息模型，不是完整 IMU 预积分、
全姿态协方差或地图—位姿相关性估计；没有新增 gyro bias 在线估计，也没有
自动俯仰标定。持续陀螺偏置、轮胎滑移、错误地面模型仍须单独验证。
日志包含每帧 source stamp、实际 dt、预测/后验信息状态及是否初始化。
默认仍禁止实车输出；候选的独立合成、回放、实际两圈结果以 STATUS 为准。

范围：本次自动启用的是本包的 raw-wheel/raw-gyro worker 和仿真适配链。
旧 reference/fixed 的 CommonGroup follower 仍走旧调用，不把它的融合 INS
数据误称为此处原始陀螺统计模型。共享 API 不带新时间戳的历史调用保持旧行为；
一旦某定位器提交了运动信息，后续漏传时间戳就拒绝，不允许静默退回。

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

关键帧刷新先准备独立候选，不立即替换当前参考。候选继承已提交的跟踪修正和时间栅栏，
不会重新放开首次定位门限。候选几何匹配不合格时，最多再尝试一次真实的当前参考；
不是遍历历史参考挑结果，也不是退回里程计。两次尝试共用原整帧提交预算，只有最终
guard、预算和提交全部通过，才投一次地图票、追加一次轨迹并晋升候选。
日志 `reference_attempts` 保留每次尝试及其严格过去的 frontier；
`matched_against_frontier_ns` 表示本帧实际尝试的最后/所选参考，无尝试时为 null，
`active_anchor_frontier_ns` 则表示提交后仍生效的参考，两者在拒绝帧可以不同。
`pre_match_engine_ms` 是原提交预算时钟在首次匹配前的累计耗时（包括关键帧准备），
不是纯匹配时间；准备阶段已经超时的帧也保留这个值，并且实际参考仍为 null。
详见 [关键帧修复记录](../../../../../docs/agent-context/sessions/2026-10-11-planner-keyframe-005/SESSION.md)。

teach → repeat 不重置原始里程计，因此必须保留已提交的地图修正和源时间边界。
`TeachMapEngine.capture_repeat_state()` 冻结后仅捕获小型不可变状态；全图索引可由
`LaneMapLocalizer.from_tracking_state()` 在后台建立。同步便利接口
`repeat_localizer()` 使用同一契约。候选关键帧、未提交匹配、计数和降噪层不继承。
这不是对任意旧地图或新里程计 epoch 的重定位授权。
worker 仅为当前 engine 所有的冻结快照交接，并在加载 bundle 后核对精确几何和
来源前沿。已有修正不等于新定位确认：停稳、三条新 mask、原始准入门限和人工启动
要求仍保留。无 active matcher 的 bootstrap 状态保留原来的首次定位语义。

独立 bounded worker：一个运行任务、一个最新待处理 mask。控制为 20 Hz steady timer；过期结果、失效 generation、输入断流、移动中的首次定位均不能提交。图像、栅格、轨迹、队列、会话时长有界；预算是软实时保护，不是硬实时证明。`begin_teach` 后原始运动链异常锁存本次会话无效，不允许补造漏掉的运动继续驾驶；此限制不是由 GNSS 质量决定的。

`~/.ros/lane_learning_gnss/follower-<unique>/` 保存轮速/gyro源时间、GNSS起点参考/结束定位、状态转移、每帧匹配拒绝原因、输出命令和旋转 JSONL。日志写入失败 HOLD。成功地图只包含 route、consensus、trace、calibration、metadata 和逐文件 SHA256，**不含 GNSS 坐标或 anchors 文件**；构建前后的冻结几何 hash 必须一致。旧结果不覆盖。

## 验证边界

本地纯算法测试、真实 bag 输入审计、原生 Foxy DDS 验证分别记录，不混称实车验证。现有 `2026_01_20-15_31_07` bag 的原始 GPS 全部缺 POSLLA，不能证明本包真实 GNSS 首尾流程；不使用 INS position 伪造缺失值。CI 的合成端点只能验证软件隔离与时序，不证明现场卫星精度、轮胎打滑误差或车辆闭环安全。

运行本包测试：`python3 -m pytest test -q`（需先 source 安装的共享算法）；CI 脚本在独立 Foxy 环境构建三包并做 DDS 通信，绝不启动实车硬件。
