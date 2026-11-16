# 内外圈原包话题重放包（中文说明）

本交付是**一个ROS 2 bag**：`bag/metadata.yaml`和`bag/bag.db3`，sqlite3存储、CDR序列化、ROS 2标准消息。内圈inner_joint_07与外圈outer_joint_03各包含两物理圈；总消息41,997条，bag时间跨度约450.70秒，数据库1,549,475,840字节（约1.55 GB）。

数据来自既有Unity MCP实跑记录，按原semantic 0.4.0的订阅话题和标准消息类型导出。没有修改原包源码，没有启动Unity或重跑模型；这是从仿真接线日志还原的原生话题包，**不是现场DDS直接录制的bag**。原包入口为`semantic_local_planner_ros/map_assist`，来源与源码校验见`原包源码位置与校验.json`。

## 原包实际需要的话题

| 话题 | ROS消息类型 | 用途与来源 |
| --- | --- | --- |
| `/aiformula_sensing/gyro_odometry_publisher/odom` | `nav_msgs/msg/Odometry` | 实际传入原包的仿真里程计，8758条；不是重新估计的视觉里程计 |
| `/original_processed_point_a` | `geometry_msgs/msg/Pose2D` | 原始视觉选点A，semantic修正之前 |
| `/original_processed_point_b` | `geometry_msgs/msg/Pose2D` | 原始视觉选点B，semantic修正之前 |
| `/original_processed_point_c` | `geometry_msgs/msg/Pose2D` | 原始视觉选点C，semantic修正之前；三话题各1995条 |
| `/lane_stop_flag` | `std_msgs/msg/Bool` | 实际避障占用状态，本次两场均空闲；按初值/变化发布 |
| `/map_assist/branch_active` | `std_msgs/msg/Bool` | 实际分支占用状态，本次两场均false；按初值/变化发布 |
| `/map_assist/traffic_light_input` | `std_msgs/msg/String` | 原包真正收到的red/green/unknown，2001条；当前RGB灯牌策略，不是实体灯真值 |
| `/map_assist/controller_cmd_vel` | `geometry_msgs/msg/Twist` | 原生LYA实际回调给速度门控的命令，1870条 |
| `/map_assist/reset` | `std_msgs/msg/Bool` | 导出时显式加入的两段冷启动边界，true；不是原场景里发生的人工重置 |

正常冷启动无需外部reset；这里把两个独立worker的试验拼接成一个bag，因而每段开头发布reset，以免外圈沿用内圈地图。没有向原包接入bbox距离、官方节点真值、评价轨迹或相机外部定位。

相机不是MapAssist的直接订阅；另附下面的话题供复核/重新运行感知：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 640×360、bgr8，2012张实际detector.Input图像，已去除同一拍摄帧的重复保存记录 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 保留2012张原JPEG的精确字节 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 每帧已知K与分辨率；D未记录，保持空；R为无额外校正的单位阵，不能误当相机外参 |
| `/semantic_replay/camera_calibration` | `std_msgs/msg/String` | 原保存的K/R/t标定JSON，仅元数据；未猜测或生成TF |

**raw图像来源限制：原始记录是JPEG，image_raw是这些真实输入JPEG的解码像素，不能恢复未保存的无损像素。只保留记录到的2012张独立图像，不补帧、不使用overlay、不声称是相机全帧率录像。**语义输入重放使用已记录的ABC和颜色，所以无需再次运行感知；若重新运行感知，请停用相应已录输入，避免同一话题双发布。

## 一个bag中的两段时间线

| 段 | bag时间偏移 | 原始时长 | 实际圈数 | 图像 | 已接受ABC帧 | 真实颜色回调 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| inner_joint_07 | 1.000000秒 | 214.980秒 | 2 | 971 | 960 | 966 |
| outer_joint_03 | 220.979758秒 | 230.778秒 | 2 | 1041 | 1035 | 1035 |

原仿真时间加固定偏移写入bag，时间保持单调；第二段前有5秒段间隔。两个reset比各段零点提前0.1秒。Header时间同步平移，原始时间保存在图像/输入CSV和manifest中。两场坐标仍各自为原来的odom/base_link，重置明确区分两个会话；没有手工配准两场地图。

图像以原capture time写入，ABC/颜色按实际worker接收tick写入，保留检测延迟；过期视觉和未支持选点没有被伪造为有效ABC。相同tick内按占用状态→颜色/ABC→odom→LYA门控输入的顺序写入。ROS回调调度、异步匹配及墙钟计时会影响重跑结果，不承诺逐位一致。

## 直接用原包运行

在已有ROS 2环境中编译并source原semantic 0.4.0工作区。此目录只提供接线参数和bag，不重复复制源码。原包入口源码、官方图和参数文件的SHA256见源码索引。

终端一，在本交付目录运行：

```bash
source /你的ROS工作区/install/setup.bash
bash ./replay_original_map_assist.sh
```

该脚本直接启动原`map_assist`可执行程序；`replay_map_assist.yaml`采用实跑原参数，开启交通和速度门控，并使用附带的同一官方OSM。仅把重算速度输出接到`/semantic_replay/recomputed_cmd_vel`。其他重算输出仍为原`/map_assist/status`、`/map_assist/path`和`/processed_point_a/b/c`。没有重写semantic算法或包入口。

终端二，在同一目录运行：

```bash
ros2 bag info ./bag
ros2 bag play ./bag --clock 100 --rate 1.0 --delay 3
```

**按1倍速重放，不循环、不快进。** 原包的视觉超时、颜色投票和恢复延迟使用monotonic墙钟，单独设置use_sim_time不能使这些逻辑随倍速正确缩放。bag不另存/clock，由player发布，避免双时钟。两段起始reset会清除已有地图；需要只看一段时优先重新启动节点后从该段reset前播放。

这个重放入口直接消费bag中的原始ABC、颜色、odom及已录LYA输入，不另起lane_points或LYA节点。这里重放的是原记录的控制输入，用于语义地图/门控复查；不是算法改动后的新闭环车辆实验。要让LYA重新计算时，应另行接入原完整视觉控制链并从播放列表排除`/map_assist/controller_cmd_vel`，避免两个命令来源。

## 同包附带实际输出，方便对比

| 话题 | 内容 |
| --- | --- |
| `/recorded/map_assist/status` | 每次实跑原生状态，8758条 |
| `/recorded/map_assist/map_data` | 原始地图输出，每秒及路径版本变化时保留，428条；包含学习轨迹、参考线、官方图坐标系与绕行点 |
| `/recorded/map_assist/path` | 路径版本变化时的原参考路径，4条；位置来自实际输出，朝向未记录，导出填单位四元数，不能拿它当实测朝向 |
| `/recorded/map_assist/cmd_vel` | 原semantic实际发布的速度门控输出，2537条 |
| `/recorded/processed_point_a/b/c` | semantic实际新发布的修正后点，各1870条；原记录只保存x/y，theta导出为0 |
| `/semantic_replay/session` | 两段运行编号、时间偏移及合成reset说明 |

前缀/recorded的话题不回接原包输入。它们用来与重新计算的输出比较。当前记录应呈现首圈轨迹增长并闭合，随后official_unaligned/ambiguous_correspondence；两场红停绿行，但没有内部绕行。切绿是原测试在等待12秒后永久保留失败，再恢复第二圈行驶的动作。

## 验证与完整来源

- 已将全部41997条CDR消息读回并反序列化，计数、时间排序和里程计四元数检查通过。
- 全部2012张image_raw逐字节等于对应原JPEG解码结果；compressed逐字节等于原JPEG。
- 三个ABC话题数量一致；颜色回调数逐场等于原worker累计投票计数。原视觉接收结果和相应源行号见`vision_input_index.csv`。
- `image_index.csv`保留拍摄序号、原拍摄时间、原接收时间、bag时间和原图/解码SHA256。
- `manifest.json`保留话题类型、每话题计数、原大日志SHA256、段边界及数据限制；`verification.json`给出数据库SHA256；SQLite只读quick_check通过。
- 本机没有原生ROS 2/rclpy，仅有rosbags库；因此已验证标准bag结构与消息内容，尚未在ROS 2播放器/原生DDS节点下运行验证。将格式验证与原生运行验收分开。
- 仿真odom为原测试所用理想仿真里程计；协方差为原接线的全零，不代表真实设备高置信度。没有生成GNSS或猜测TF。

源运行配置、结果和来源哈希在`source_metadata/`。导出程序`export_semantic_rosbag.py`只读既有记录；其中固定本机来源路径，如需迁移应另行显式指定并复核来源。大bag不提交Git；文档、参数、源码索引和小验证记录可版本控制。
