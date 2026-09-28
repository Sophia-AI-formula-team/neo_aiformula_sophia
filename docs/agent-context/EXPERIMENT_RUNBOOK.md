# 实验日：先确认数据和停车链，再谈定轨迹

适用于当前 `_gnss` 实验任务。先读 [AGENT_CONTEXT](../../AGENT_CONTEXT.md)、
[STATUS](STATUS.md) 和包 [README](../../workspace/src/aiformula/control/lane_mapping_fixed_gnss/README.md)。
本文件是检查/记录要求，**不是自动执行脚本或实车放行书**。

## 0. 先留档，避免“跑完才发现没录到”

在启动节点前创建 session 和独立 run ID。存档至少含以下内容：

| 证据文件（建议名） | 必须记录的内容 |
| --- | --- |
| `environment.json` | UTC 起止、匿名机器编号、Ubuntu/ROS/RMW/Python、ROS domain、时钟设置、CPU/资源限制、实际包前缀和 overlay 顺序 |
| `source.txt` + `changes.patch` | 任务与基线、实际 HEAD、branch、dirty status、diff；未跟踪源文件单独列出并保存，不能只存 git diff |
| `deployment.json` | 实际导入模块路径及 SHA256、可执行程序来源、启动命令、完整有效参数、remap 后话题、消息类型/frame、QoS、发布者数量 |
| `milestones.jsonl` | 启动、begin_teach、各次服务请求/返回、GNSS 窗口、停稳、LYA STOPPED、失锁/急停/结束；时间和原因，不轮转删除 |
| `metrics.json` | 定义/单位/样本数/拒绝数/统计窗、输入间隔、延迟与效果指标；没测填 null 并说明，不能写 0 |
| `commands.log` / 节点日志 | 本次命令、开始/结束、退出码、stdout/stderr；错误和取消不删 |
| `artifacts.json` | bag/标定/地图/路线的 SHA256、大小、起止和来源；是否脱敏/裁剪/缺初始化、原件保管状态 |
| `SESSION.md` | 假设、允许修改范围、实际步骤、前后差异、通过/失败/未执行、当前安全状态、回退和下一问题 |

**已知记录风险**：共享 RunJournal 默认仅保留 4 × 5 MiB，会删除早期记录。
不能只在结尾打包它；另存不轮转的 run manifest 和阶段快照，必要时使用有磁盘预算的外部日志收集。
空间不足立即记为证据不完整，不能悄悄继续。GNSS 节点启动日志已记录有效参数和速度来源，
现场仍须补实际源码哈希/overlay/依赖快照，并在轮转前保存；不能仅附 repo 中默认 YAML。

时间必须区分：源 header `source_ns`、接收时 ROS 时钟、接收时单调时钟，以及 clock domain/进程 epoch。
不同主机 monotonic 不能相减。LYA `Twist` 没有 header，只能记录本地接收新鲜度，不能编造源时间。
bag 片段若不含初始化，必须说明不能单独复现完整会话。

只读核对示例（结果存本地受控记录，审查后上传）：

```sh
git rev-parse HEAD
git status --short
git diff -- workspace/src/aiformula/control/lane_mapping_lya_reference workspace/src/aiformula/control/lane_mapping_fixed workspace/src/aiformula/control/lane_mapping_fixed_gnss
ros2 pkg prefix lane_mapping_fixed_gnss
ros2 node list
ros2 node info /lane_endpoint_follower
ros2 param dump /lane_endpoint_follower
ros2 topic info /aiformula_sensing/vehicle_info --verbose
ros2 topic info /aiformula_sensing/zed_node/imu/data_raw --verbose
ros2 topic info /aiformula_perception/road_detector/mask_image --verbose
ros2 topic info /aiformula_sensing/vectornav/raw/gps --verbose
```

节点尚未启动时相关查询失败应写“未启动”，不是捏造输出。不要收集整个环境变量表、
shell 历史或认证文件，以免把 token/个人资料带入证据。

## 1. 静态部署和输入核对（不连电机）

按包 README 在实际 Ubuntu/Foxy 环境构建，记录完整 colcon 命令、输出和退出码。
缺依赖报告明确包名/来源/方案，先取得安装授权，不擅自修改车端环境。
确认运行的是本次 build 的 overlay，不是旧 install 目录。

| 输入 | 当前实现契约 | 现场必须测/确认 |
| --- | --- | --- |
| `/aiformula_sensing/vehicle_info` | `can_msgs/msg/Frame`；ID 1809/0x711，左 RPM=data[4]、右=data[0]；默认轮径0.254m | 原始字节/单位/频率；仅已核实的低8位无符号前进协议。高字节/倒车/未知协议不猜 |
| `/aiformula_sensing/zed_node/imu/data_raw` | `sensor_msgs/msg/Imu`；只用 angular_velocity，经固定安装旋转 | 静止 bias、轴方向/符号、时间连续性；不读 orientation、不换 INS |
| `/aiformula_perception/road_detector/mask_image` | 完整 mono8/8UC1 mask | 来自 road detector，不是 ROI；尺寸/frame/编码和时间戳 |
| CameraInfo / TF | 匹配图像尺寸/frame，已到达的固定安装外参 | 实际相机内参/高度/俯角、地面假设；不要只相信文件名 |
| `/aiformula_sensing/vectornav/raw/gps` | 原生 `vectornav_msgs/msg/GpsGroup`，FIX/POSLLA/POSU=0x230，3D fix，有效不确定度 | Honda 默认0x210缺POSLLA；经授权用包内 `vectornav_endpoint_fields.yaml` 附加到驱动启动，不是 follower 参数 |

gyro TF 可有非零 stamp；当前实现要求 3 个不同时间戳的同一固定旋转再锁定，
或显式提供经过测量的 xyzw quaternion。不要随意填单位旋转解决等待。
旧 NavSatFix 转换器使用 INS CommonGroup 且缺协方差，不能替代原始 endpoint GPS。
GNSS 日志有精确经纬度，公开前脱敏。

输入间隔要看原始源时间和实际到达时间两套分布，记录 p50/p95/p99/max、失序、未来、丢包和拒绝原因。
当前 wheel 200 ms 限制不是调参优化目标。若超限，先查驱动/带宽/QoS/时间戳/执行阻塞和硬件，
不能放宽到几秒。`begin_teach` 后任一次 motion 断链使本会话失效；恢复消息不恢复旧地图行驶资格。

## 2. 隔离 DDS 和真实传感器影子验证

- 合成 smoke 只能在隔离 ROS domain/网络中执行，不能向真实车辆 domain 发布假 CAN/gyro/GPS。
  使用包内测试，不自行发一堆“看起来合理”的位姿给在线节点。
  它在隔离环境设置测试用确认并显式 arm，覆盖窗口、无 GNSS 建图、坏输入拒绝；
  这些确认不代表真实硬件已测。现有静止 smoke 不进入完整行驶 REPEAT，不能据此声称已验证第二圈。
- 真实传感器影子运行只保留私有 `/lane_learning_gnss/cmd_vel`，三个实车开关保持 false。
  不调用会 arm 的服务，不接电机输出；任何更高阶段都需新的操作者确认。
  这一状态只审计实际输入和未放行状态；当前节点未 arm/prepare 时不处理建图 mask，
  **此阶段不要求真实 consensus 或 TEACH/REPEAT 证据**，不要为完成清单私自开确认开关。
- 在隔离测试核对有界队列和源时间准入：未来/过期数据拒绝，超时后不自动恢复控制。
  真实输入只读审计保留源时间/接收时间，不向真实 domain 注入损坏数据。

实际运行 consensus、TEACH/REPEAT 的 GNSS 无订阅检查，放到下面另获授权的阶段。
无 GNSS 建图与坏 GNSS 禁 repeat 的已隔离证据应分别记录，不能混成一个成功结论。

影子验证不证明轮子停得住，也不等于完整移动建图/可用闭合路线验证。
合成损坏/未来消息仅在隔离测试，不为真实行驶实验故意注入断流。

## 3. 人工台架停车链与操作者许可

操作者明确批准台架条件后，分别记录：

1. 独立手操急停能切断驱动，以及恢复开关不会自动重新行驶。
2. 软件零命令 → 下游实际轮速/CAN 指令为零 → 物理车轮停止，三者分别留证。
   当前 neo 源码已有零命令旁路；须核对实际安装版本和物理输出，不能只看上游 Twist。
3. 下游命令断流超时有效；单个控制器拥有真实 command topic，无竞争发布者。
4. `enable_vehicle_output` / `hardware_stop_verified` / `motor_zero_passthrough_verified`
   的开启者、时间和证据。把参数设 true 本身不是验证。

当前任务不授权 agent 自动开启这三个开关、提速或发送真实电机指令。
无法证实停车链就返回 BLOCKED；不靠“有遥控急停”略过软件/电机零输出检查。

## 4. 有明确本次实车授权后，才做一圈和 repeat

不要把以下状态序列整体复制成无人看守脚本。具体服务名见包 README，每步核对返回与实际状态：

1. 连续停稳至少1秒，按需 `capture_start` 取起点 GNSS。随后30秒内保持位移≤0.1m、转角≤0.15rad。
2. `begin_teach` 独立重置局部地图原点；新原点后再取得至少1秒停稳证据，才由操作者决定 `arm_teach`。
3. LYA 驾驶第一圈，保存实时 mask、原始运动、consensus、车辆状态及拒绝事件。异常按停车流程处理，
   不通过删失败帧/循环 reset 延长旧会话。
   保存固定尺度过程画面，注明丢帧/抽帧倍率，不以后验地图替代。核对 GNSS 仅两个停稳窗口订阅，
   TEACH 中段和之后 REPEAT 不订阅；未到 REPEAT 就标该项未执行。
4. 回起点 `stop`，实测停稳至少1秒，再 `finish_lap`。保存独立几何 bundle 与第二次起点 GPS 判定。
   GNSS 坏则保留地图，禁止 repeat；不能反过来把 GPS 拿去改地图对齐。
5. 人工审阅地图/路线、清空危险区，才 `prepare_repeat`。必须是真实受管 LYA 的新 STOPPED 心跳、
   连续三次可信停稳 mask 匹配；不得伪造 STOPPED。最后由操作者显式 `start_repeat`。
6. 手操急停/软件 estop 均需记录；reset 不代表允许自动继续。运动链丢失后结束本 run，不能续用漏运动的地图。

LYA 起停只管理本 launch 所创建进程，不杀别的控制器、不假设其它发布者已停止。
速度默认继承 LYA 当前 `REFERENCE_SPEED_MPS`（当前 2.0），仅明确配置覆盖；第一圈保留通过
准入的教师原命令，repeat 原角速度门限 0.4 不变。软件默认值不是现场驾驶批准。
记录有效参考、教师正数准入上限、路线可行性和实际输出；上限 0 仅私有预览，不可接车辆输出。
`_gnss` 原轮速门限 3.0 不随参考自动升高，参考冲突在启动时报错；不得靠放宽门限消除错误。

## 5. 必须报告“效果”，不能只报运行速度

| 项目 | 可复核的记录要求 |
| --- | --- |
| 输入/实时性 | 每源有效/拒绝数、间隔与端到端延迟分位数、worker 耗时/超时/覆盖pending数、CPU/RSS、统计窗口；无统一时钟不能假报端到端延迟 |
| 地图效果 | 固定尺度在线 consensus、覆盖/空洞/离群点统计、首尾连续性；如报 overshoot 给出人工审核或独立参考定义/误差及样本数 |
| 路线 | bundle/hash、闭合/长度/曲率/走廊净空检查，驾驶路线冻结前后哈希；refinement 改了什么单独报告 |
| 定位 | 每帧匹配接受/拒绝原因、有效帧分母、连续通过数、最长失锁；独立参考下的位置/航向误差。匹配残差不是实际定位误差 |
| 控制 | 跟踪误差/指令饱和/停车距离与时延，参考来源及测量条件；没有真实实测就标未执行 |
| 对照 | 同输入/同代码基线/单因素变化，前后参数和指标，真实整圈与合成/裁剪回放分开 |

定轨迹阶段不能输入 GNSS。如需独立定位真值，仅按另行批准的离线/隔离记录方案评价，
不得连接到运行节点或据它修改在线地图。没有真值就明确“未测绝对误差”。
所有物理精度/净空/停车阈值由现场测量和后续计划确定，不能仅凭 CI 绿或视觉好看放行。

## 6. 收尾与回传

确认安全状态由谁负责，不擅自停止无关节点。保存最终参数/日志/地图和命令退出码，逐文件脱敏，
按 [协议](PROTOCOL.md) 封存、打包、上传当前实验分支、读取远端确认，发结果回执。
交接必须回答：“改了什么？什么证据支持？什么没过？下一步要 planner 决定什么？”
卡住同样回传，不能等一次成功才记录。未上传原始证据及原因写清，不能宣称包已包含完整复现。
