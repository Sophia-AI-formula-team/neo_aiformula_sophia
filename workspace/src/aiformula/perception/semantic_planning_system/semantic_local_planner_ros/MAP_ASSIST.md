# map_assist：保留 odom 尺度的地图融合、视觉辅助与红灯改道

`map_assist` 仍在原 `semantic_local_planner_ros` 包内，独立于原语义门控运行。原语义方案保留。

## 启动：新 lane 用默认配置，旧 rosbag 指定测试配置

```bash
ros2 run semantic_local_planner_ros map_assist
```

上面这条裸命令自动读取包内 `config/map_assist.yaml`，适用于新 `lane_0927oa` 的正常冷启动：订阅 `/original_processed_point_a/b/c`，输出 `/processed_point_a/b/c`，使用系统时间，启用构图与方向辅助。初始假定尚未避障，随后服从真实 `/lane_stop_flag`；未知 OA 状态的中途重启应加 `--ros-args -p avoidance_initially_idle:=false`。红灯控制和速度转发默认关闭，需要这些功能时按下文车辆配置开启。

旧 rosbag 构图测试使用以下单行命令：

```bash
ros2 run semantic_local_planner_ros map_assist --ros-args --params-file "$(ros2 pkg prefix semantic_local_planner_ros)/share/semantic_local_planner_ros/config/map_assist_bag.yaml"
```

测试配置订阅旧 `/processed_point_a/b/c`，三点输出到 debug 话题，关闭驾驶纠偏、红灯控制及速度输出。录制开头应未进入避障，后续使用真实 flag；回放带 `--clock`。在 RViz 添加 `/map_assist/path`（Path，Transient Local），首圈结束后查看整条路线。`phase=guarding` 且 `map_fusion.available=true` 表示融合成功。

显式指定配置时不叠加默认车辆配置。两种启动方式都会自动定位包内官方 OSM；切换时结束当前 map，再用另一条命令启动，不需要修改代码或来回改话题。

## 0. 粗图怎样建立（本次修正）

```text
首圈 odom + 避障／岔路不可信标记
    → 临时测量轨迹（不是最终粗图）
    → 与官方外圈比较整体形状、方向和局部转弯特征
    → 将官方外圈节点对应到 odom 轨迹
    → 保留 odom 米制尺度和实测段长，用官方节点方向约束外圈形状
    → 对融合结果平滑，得到最终粗图
    → 保存每个外圈节点在最终粗图上的坐标和累计路程
```

官方地图用于形状和连接关系，不规定本次轨迹的实际尺寸。比较前将官方外圈归一到 odom 量纲；不能把 odom 轨迹放大／缩小成官方尺寸。小范围融合和平滑仍可能稍微改变总弧长和局部距离，但不采用官方总长或官方各段长度覆盖实测尺度。首圈最小行驶距离仅使用 `min_lap_distance_m` 的 odom 米数，不再乘官方周长。

融合独立于 `enable_traffic_control`：即使关闭红灯功能，仍执行地图比较、节点对应、融合和平滑。匹配不明确或节点特征不可靠时，状态为 `odom_only`，保留 odom 平滑轨迹供检查，但正常行驶仅透传有效视觉，不启用地图纠偏或拓扑绕行；通过 `/map_assist/reset` 重新学习。`map_fusion.available` 与 `reason` 明确报告结果。

外圈路口使用最终粗图的节点坐标和实测进度。内部节点先按官方图变到 odom 尺度，再把外圈节点的位移沿原图连接平滑传递到内部；所有连线长度重新用 odom 坐标计算。内部节点位置仍是推算，首圈没有经过的内部道路并未被测量。无新增连接，也没有承诺消除后续 odom 漂移。

## 1. 接入现有行驶链路

已按用户确认的 `lane_0529oa → withoutkalman_0312 → lya_follower_connected_omegat_global` 接线。OA 继续使用原来的 `oa_neo` 或 `oa_neov1.2`，本次没有修改、启动或替换 OA。

```mermaid
flowchart TD
    Perception[原车道线检测] -->|车道点云| OA[原 OA 节点]
    Cone[原 cone 检测] -->|cone_mask / cone_bbox_image| OA
    OA -->|正常时转发 lane_line_center / left / right| Lane[lane_0529oa]
    Lane -->|/original_processed_point_a、b、c| Points
    OA -->|避障三点 /processed_point_a、b、c| Filter[withoutkalman_0312]
    OA -->|/lane_stop_flag| Points
    OA -->|/lane_stop_flag| Lane
    YOLO[YOLOP 红绿灯状态] -->|占位 String：/map_assist/traffic_light_input| Points
    Odom[原 odom] --> Points
    OSM[官方拓扑文件] -.-> Points
    subgraph Assist[同一个 map_assist 节点]
        Points[记录粗图／辅助视觉／红灯路线决策]
        Velocity[速度转发与等待停车]
        Points -.->|等待／恢复状态| Velocity
    end
    Points -->|/processed_point_a、b、c| Filter
    Filter -->|/filtered_lane_pose、/filtered_omega_t| LYA[lya_follower_connected_omegat_global]
    LYA -->|/map_assist/controller_cmd_vel| Velocity
    Velocity -->|/aiformula_control/game_pad/cmd_vel| Motor[原底盘控制]
```

`a、b、c` 表示三个独立 `Pose2D` 话题，单位米，车体坐标 x 向前、y 向左。

接入改动只有两处重映射：视觉三点交给 `map_assist`；LYA 的速度输出交给同一节点做停车处理。OA 的避障三点仍直接送入 `withoutkalman_0312`。

**真正等待靠零速度实现。** 现用 LYA 存在最低线速度限制，零三点和停发三点都不能代替停车。正常时原样转发新 LYA 速度指令；等待时周期发送零 `Twist`；速度超过默认 0.5 秒未更新也发零。解除等待后要求新的有效视觉三点和新的 LYA 指令，不重放旧速度缓存。

## 2. 启动：一个开关选择原系统或地图辅助

在车辆 ROS2 工作区安装这套包后：

```bash
colcon build --symlink-install --packages-up-to semantic_local_planner_ros
source install/setup.bash
ros2 launch semantic_local_planner_ros map_drive.launch.py
```

`map_drive` 启动原来的三个节点和一个 `map_assist`。先结束手动启动的这三个进程，避免重复发布。相机、YOLOP、cone 检测、OA、里程计及底盘仍按原来的方式启动。

完全使用原行驶链路：

```bash
ros2 launch semantic_local_planner_ros map_drive.launch.py use_map_assist:=false
```

关闭模式只启动原三个节点，保持原话题，不启动 `map_assist`。这是启动时切换；已接入时单独结束 `map_assist` 不会自动恢复旧接线。

只关闭视觉的方向纠偏、保留红灯决策与停车功能：

```bash
ros2 launch semantic_local_planner_ros map_drive.launch.py enable_correction:=false
```

### 现有 OA 的初始 flag

当前两版 OA 都只在开始／结束避障时发布 flag，没有初始状态发布。裸命令读取的车辆 YAML 按正常冷启动设 `avoidance_initially_idle: true`；`map_drive.launch.py` 的启动参数仍默认 false，等待真实 flag。

如果是**已确认 OA 尚未进入避障的冷启动**，可显式声明初始空闲，先完成本次接入而不改 OA：

```bash
ros2 launch semantic_local_planner_ros map_drive.launch.py avoidance_initially_idle:=true
```

它只设置本节点的初始状态，不向外发布假 flag。之后始终遵从真实 `/lane_stop_flag`；收到 true 后不会超时自动恢复。在车辆已经避障时启动本节点，应显式设置 false，等待真实状态。后续若 OA 增加当前真实状态发布，就不再需要这一启动假设。

### 不使用 launch：逐终端手动启动

下列命令用于车辆上的 Linux/Bash。每个终端先加载现有 ROS2 环境及包含这些包的工作区 `install/setup.bash`。新增包需已构建。各节点只保留一个实例；不要同时运行旧启动命令或 `map_drive.launch.py`。

**终端 1：map_assist。** 以下 `avoidance_initially_idle:=true` 仅用于已确认 OA 空闲的冷启动；否则设 false，等待真实 flag。OSM 路径现在可自动定位；红灯停车／绕行仍需开启交通与速度功能，单独读取默认车辆 YAML 不会启用这两项。

```bash
MAP_ASSIST_SHARE="$(ros2 pkg prefix semantic_local_planner_ros)/share/semantic_local_planner_ros"

ros2 run semantic_local_planner_ros map_assist --ros-args \
  --params-file "$MAP_ASSIST_SHARE/config/map_assist.yaml" \
  -p "topomap_path:=$MAP_ASSIST_SHARE/config/aiformula_topomap.osm" \
  -p enable_traffic_control:=true \
  -p enable_velocity_gate:=true \
  -p avoidance_initially_idle:=true
```

**终端 2：视觉三点。** 新增的 `lane_0927oa` 已直接发布 `/original_processed_point_a/b/c`，不再需要重映射。它保留 `lane_0529oa` 的处理逻辑，只修改三个输出话题；文件放进原 `lane_points` 包，在该包 `setup.py` 的 `console_scripts` 中添加 `'lane_0927oa = lane_points.lane_0927oa:main',` 后重新构建。

```bash
ros2 run lane_points lane_0927oa
```

已有的 `map_drive.launch.py` 仍使用 `lane_0529oa` 加三点重映射，接线效果相同。手动方案使用 `lane_0927oa`，两者择一启动。

**终端 3：原三点处理节点。** 如果已经运行，就保留原实例。

```bash
ros2 run kalman_filter withoutkalman_0312
```

**终端 4：LYA。** 将它的输出交给 map_assist，再由 map_assist 输出到底盘话题。

```bash
ros2 run trajectory_follower lya_follower_connected_omegat_global --ros-args \
  -r /aiformula_control/game_pad/cmd_vel:=/map_assist/controller_cmd_vel
```

OA、相机、检测、odom 和底盘照旧运行。**不要把 OA 的 `/processed_point_*` 重映射成视觉输入，也不要将 `/lane_stop_flag` 接成 `/point_flag`。**

手动接线完成后检查：

```bash
ros2 node info /map_assist
ros2 topic info /original_processed_point_a --verbose
ros2 topic info /processed_point_a --verbose
ros2 topic info /map_assist/controller_cmd_vel --verbose
ros2 topic info /aiformula_control/game_pad/cmd_vel --verbose
ros2 topic echo /map_assist/status
```

| 话题 | 应有的发布方 | 应有的订阅方 |
|---|---|---|
| `/original_processed_point_a/b/c` | `lane_line_subscriber` | `map_assist` |
| `/processed_point_a/b/c` | `map_assist` 和原 OA（按 flag 分时发送） | `data_processing_node` |
| `/map_assist/controller_cmd_vel` | `trajectory_follower` | `map_assist` |
| `/aiformula_control/game_pad/cmd_vel` | 本套自动驾驶链路中为 `map_assist` | 原底盘控制节点 |

图中能看到发布／订阅关系只代表接线成立；是否有数据还应看 `ros2 topic hz` 和状态原因。若显示 `avoidance_state_unknown`，当前仍未确认 OA 状态；`waiting_fresh_visual_triplet` 表示视觉输入尚未满足；`controller_command_timeout` 表示尚未收到新 LYA 速度。

手动恢复原系统时，结束 map_assist 和 lane_0927oa，改用原 `lane_0529oa`；LYA 去掉速度重映射后重新启动。三点处理和 OA 的命令不变。只关闭 map_assist 不会自动恢复这两处连接。

## 3. 红灯决策

```mermaid
flowchart TD
    Light[接收红绿灯状态] --> Red{是否红灯}
    Red -->|绿灯| Normal[正常视觉行驶；解除等待后使用新数据]
    Red -->|红灯| Map{首圈粗图已建立}
    Map -->|否| Wait[零速度等待；绿灯后继续首圈]
    Map -->|是| Locate{融合成功且在已对应的外圈路口附近}
    Locate -->|否| Wait
    Locate -->|是| Block[临时禁用当前前方的外圈路段]
    Block --> Search[在剩余图中搜索经过内部节点的替代路线]
    Search --> Valid{路线连通且入口／跟踪条件满足}
    Valid -->|否| Wait
    Valid -->|是| Follow[按当前 odom 转换为车辆前方三点并跟踪]
    Follow --> End[到达原路线的后续节点]
    End --> Normal
```

- **第一圈：**沿视觉行驶并记录 odom。遇到红灯等待，只有明确绿灯解除；检测变成 unknown 或停止更新，不会自动解除已有红灯等待。
- **建图：**检测闭合圈后，先进行官方外圈对应和形状约束融合，再平滑。自动搜索起点和行驶方向，按局部转弯特征对应节点；最终保留 odom 米制尺度，不要求手动给起点或固定绕行节点。
- **质量检查：**对应误差太大、节点特征不可靠、形状近似对称而存在多个解释时，报告 `odom_only` 及不可靠原因，不用该结果执行地图纠偏或拓扑绕行。有效视觉继续正常透传。
- **第二圈起遇红灯：**在当前匹配的外圈附近找到路口，根据已识别行进方向确定原本下一条边，临时将其视为不可通行。从该路口到这条边的另一端，在剩余官方图上搜索经过内部节点的最短连通路线。每次按当前位置计算，不写死 `1→5→6→3` 等顺序。
- **跟踪替代路线：**在外圈节点已绑定最终粗图的 odom 图上搜索；入口、出口精确使用这份粗图的节点，内部节点用边界约束推算。每次新 odom 生成一组车体三点，经原处理节点和 LYA 执行。按顺序通过节点，进入 `detour_waypoint_radius_m` 容差范围即视为到达；三点只采样当前直线段，尚未生成平滑的路口转弯轨迹。偏离太远或目标不可向前执行时停车。
- **回到原路：**以替代路线出口重新设置外圈匹配进度，等待新视觉三点后恢复。完成前即使灯变绿，也先走完已进入的连接路线，避免在路段中途跳回外圈；同一次红灯不反复触发改道。
- **OA 仍优先：**true 时不输出任何地图／视觉三点，已有红灯决策暂停，新 LYA 避障速度指令照常通过；false 后再处理红灯或恢复路线。速度断流仍会停车。

**位置与道路边界：**融合对本次 odom 粗图施加有限的外圈形状约束，不是全局 SLAM，也没有消除长期 odom 漂移。官方文件只有稀疏连线，没有车道宽度和路口转弯曲线；本版只在路口附近启动替代路线，不能从任意路段直接穿过场地去连接内部节点。图中没有单行方向标签，当前按提供的无向连通关系搜索。内部节点是估计位置，实际路线可行性、容差和转弯跟踪需要现场验证。

## 4. 红绿灯输入留占位

参数文件：`config/map_assist.yaml`。

```yaml
traffic_light_topic: /map_assist/traffic_light_input
```

当前约定 `std_msgs/msg/String`，内容为 `red`、`green`、`unknown`。这是预留接口，没有假称已接上 YOLOP 的真实输出。若 YOLOP 输出的是检测框或自定义消息，还需要把识别结果转换成这一状态，或调整本节点订阅类型和回调；只改话题名不能改变消息类型。

以后改话题可以直接传启动参数：

```bash
ros2 launch semantic_local_planner_ros map_drive.launch.py traffic_light_topic:=/your/light_state
```

联调时手动输入状态：

```bash
ros2 topic pub --once /map_assist/traffic_light_input std_msgs/msg/String '{data: red}'
ros2 topic pub --once /map_assist/traffic_light_input std_msgs/msg/String '{data: green}'
```

## 5. 查看、重置与原单节点入口

```bash
ros2 topic echo /map_assist/status
ros2 topic echo /map_assist/path --once --qos-durability transient_local
ros2 topic pub --once /map_assist/reset std_msgs/msg/Bool '{data: true}'
```

状态包含 `phase`（learning / guarding / odom_only）、`map_fusion`、`topology_alignment`、`official_node_positions_odom`、`official_node_progress_m`、`topology_mapping`、`traffic_mode`、被禁用的 `blocked_edge`、自动选中的 `detour_node_ids` 和速度放行原因。融合成功、当前 odom 匹配有效（`route_matched`）、内部图可用是分别报告的条件。粗图及对应只存当前进程内存，重启或 reset 后重新学习。

原 `map_assist.launch.py` 仍只启动单个点辅助节点，默认不启用红灯控制和速度转发；新接入使用 `map_drive.launch.py`。不要同时运行原语义 mux 或另一套向最终点／最终速度话题发控制指令的链路。

常用配置保留原视觉纠偏的 15° 容差、1 米进度容差、连续 5 组异常才介入和 8° 最大修正。融合默认参数：节点搜索 4 米、转弯特征窗口 2 米、形状约束权重 0.25、融合位移上限 0.75 米；质量误差以 odom 米为单位。参数集中在同一 YAML 中。`route_node_ids` 可选地指定首圈外圈；为空时读取官方文件第一个闭合外圈，起点与行进方向仍自动估计。
