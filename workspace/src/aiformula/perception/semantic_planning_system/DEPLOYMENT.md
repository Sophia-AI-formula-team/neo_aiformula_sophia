# Semantic 0.4.5 车端部署

MapAssist 0.4.5 替换标准源码路径下的 `semantic_planner_core` 和 `semantic_local_planner_ros`。本分支的 `semantic_vehicle_045.launch.py` 读取车上已有节点的话题，用于车辆静止时核对 ROS 接线。速度输出为 `/semantic_drive/map_assist_045/diagnostic_cmd_vel`，状态和路径也在该命名空间内。实车速度输出切换须等部署 Agent 完成下列核对并回报审核。

## 部署 Agent 先核对，反馈后再修改

在车上已有节点保持原样且车辆静止时，先执行只读检查。把启动命令、节点名、完整话题、消息类型、发布者数量和 QoS 回报给用户。

```bash
ros2 node list
ros2 topic list -t
ros2 topic info --verbose /flag_stop
ros2 topic info --verbose /point_flag
ros2 topic info --verbose /aiformula_perception/yolop_a3/traffic_light/state
ros2 topic info --verbose /aiformula_control/game_pad/cmd_vel
ros2 topic info --verbose /semantic_drive/map_status
```

- **停车距离 Bool 发布端待查。** 0.4.5 默认订阅 `/flag_stop`，仓库里的车端 launch 和节点未查到对应发布端。确认真实节点、话题、`std_msgs/msg/Bool`、发送频率、True/False 的测量含义和失联时的行为。若无发布端，先提交接线方案和测量来源供用户审核。保留停车距离与 OA 两个独立输入。
- 确认 A4 的灯色话题、当前 `lane_points` 和 LYA 可执行项、OA 实际使用 `/point_flag` 还是其他话题，以及 `motor_controller` 的实际订阅和自动速度话题上唯一的发布者。若与仓库记录不同，先回报差异。
- 现有 motor 0.4.1 的语义门读取 `/semantic_drive/map_status`，接受旧 `stop_wait` 决策。0.4.5 发布 `ubs_auto_turn_v1`，并增加 `prepare`、`detour` 状态。把原样 0.4.5 状态接到旧门控会被拒绝。不要把状态标签改名绕过这项校验。反馈实际节点图后，单独审核速度所有权、状态兼容、OA 接管、失联停车和手柄优先。

## 车端节点与诊断接线

以下是 neo 当前车端 launch 可核对的接口。以部署 Agent 读取的活动节点图确认实际安装版本。

| 发布端 | 话题与类型 | 0.4.5 接收端或用途 |
| --- | --- | --- |
| A4（现有 `yolop_a3` ROS 节点） | `/aiformula_perception/yolop_a3/traffic_light/state` · `std_msgs/msg/String` | `traffic_light_topic`，灯色为 `red`、`green`、`unknown` |
| 实车停车距离节点，待确认 | `traffic_stop_flag_topic` 显式传入 · `std_msgs/msg/Bool` | 首圈红灯停车，建图后绕行失败时停车 |
| `gyro_odometry_publisher` | `/aiformula_sensing/gyro_odometry_publisher/odom` · `nav_msgs/msg/Odometry` | 在线建图与定位 |
| `lane_0215` | `/processed_point_a`、`_b`、`_c` · `geometry_msgs/msg/Pose2D` | 诊断 launch 旁路读取真实 ABC；原 `withoutkalman_0312` 继续订阅原话题 |
| `obstacle_avoidance.launch.py` 启用的 OA 节点 | `/point_flag` · `std_msgs/msg/Bool` | 使用仓库现有 remap，与 `withoutkalman_0312` 共用真实 OA 状态 |
| `lya_follower_connected_omegat_global` | `/aiformula_control/game_pad/cmd_vel` · `geometry_msgs/msg/Twist` | 诊断模式读取现有 LYA；原车电机端保持现有接线 |
| MapAssist 0.4.5 | `/semantic_drive/map_assist_045/diagnostic_cmd_vel` · `geometry_msgs/msg/Twist` | 只读观察速度；诊断路径和 ABC 同样使用 `/semantic_drive/map_assist_045/` |

后续审核的闭环方案应把 `lane_0215` 的原始 ABC 与 MapAssist 发给 `withoutkalman_0312` 的 ABC 分开，保留 OA 现有选择。LYA 输出改接 MapAssist 输入，由审核通过的速度链作为 motor 自动指令的唯一发布端。诊断 launch 保持这两处原接线，供确认真实输入和候选输出。

## 安装与静止核对

用户审核现场回报并批准部署后，再执行备份、切换源码和构建。

现场备份旧版同名包和对应安装目录，保留现有车端配置。切换环境前确认工作区内每个 ROS 包名只有一份，旧检查点文件保留在原处。0.4.5 在线建图在进程内保存，旧版 `checkpoint_path` 参数没有对应实现。

```bash
cd /home/workspace
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select semantic_planner_core semantic_local_planner_ros launchers
source install/setup.bash
```

通过上面的只读核对并得到用户确认后，在车端静止条件下使用 `semantic_vehicle_045.launch.py`。把 `CONFIRMED_BOOL_TOPIC` 替换为实际绝对话题。内圈选 `lane:=inner`，最小规划半径 **6.0 米**；外圈选 `lane:=outer`，最小规划半径 **5.5 米**。参数为现场几何核对候选值，车端绕行仍待验证。

```bash
ros2 launch launchers semantic_vehicle_045.launch.py lane:=inner traffic_stop_flag_topic:=CONFIRMED_BOOL_TOPIC
ros2 topic echo /semantic_drive/map_assist_045/status
ros2 topic info --verbose /semantic_drive/map_assist_045/diagnostic_cmd_vel
```

确认 `traffic_planner_version=ubs_auto_turn_v1`、`traffic_stop_flag` 和灯色来自现场发布端。若 OA 没有启动，状态会等待首次有效 OA 消息，保留该等待供现场确认。两套 Semantic 节点使用同名核心，核对时选择一个入口。旧版 `semantic_vehicle.launch.py` 和 `semantic_shadow.launch.py` 属于原接口，禁止把它们当作 0.4.5 启动步骤。

## 放行前仍待审核

- 现场停车距离节点与 DDS 连接，实车速度与状态接口适配，以及手柄接管和故障停车验证。
- 旧版独立 `race_map` 入口和磁盘检查点使用旧接口。0.4.5 原版没有对应兼容实现。若车端依赖这两个能力，先决定保留或迁移方案再切换。

## 原版本部署记录

以下记录对应上一代接口和历史验证。当前分支按本页上方 0.4.5 章节审核。

# Deployment

This repository offers two distinct deployment paths. The frozen Unity path
has four recorded closed-loop runs. The new native ROS path is a shadow wiring
entry and has no physical vehicle acceptance.

## Repository layout and versions

| Path relative to repository root | Purpose |
| --- | --- |
| `workspace/src/aiformula/perception/semantic_planning_system` | Semantic core and ROS packages, 0.2.0 for core/local planner |
| `workspace/src/aiformula/control/semantic_drive_bridge` | Native signal receiver and shadow command watchdog, 0.1.0 |
| `workspace/src/aiformula/launchers` | 0.1.0; dedicated `semantic_shadow.launch.py`; old allnodes defaults preserved |
| `codexws/unity_semantic_integration` | Versioned Unity source overlay, guarded installer and frozen evidence |

Core baseline: `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`.
Integration branch: `feat/semantic-neo-integration-20261012`, stacked on
`fix/semantic-distance-stop-20261012`. PR #15 imports the source, PR #16 fixes
distance approach, and the integration PR adds wiring/docs. No tag is moved.

## Unity deployment and replay

Follow [the Unity deployment guide](../../../../../codexws/unity_semantic_integration/DEPLOYMENT.md).
Use an existing compatible Simulator checkout with the original pinned
dependencies. The overlay is not a standalone Unity project. Verify the manifest
before applying; the installer refuses unexpected file drift and supports a
scoped rollback. Keep weights outside this Git delivery and verify their SHA.

Run through Unity MCP after exclusive editor ownership has been handed over.
Restore scene, Play state, focus override and worker ownership afterwards.
The target camera/light position remains fixed; inner and outer red/green starts
are separate cases. The trial stops at the end of the requested northeast
segment, rather than completing an entire circuit.

The frozen red cases stopped at 8.635939 m (inner) and 8.602903 m (outer), waited
for green, then continued. The video preserves recognition overlay, LLP/LYA,
semantic output and the vehicle view. These runs use ideal Unity odometry and
an in-process ROS transport shim. They do not test native DDS.

## Native ROS shadow deployment

Follow [launchers/SEMANTIC_DEPLOYMENT.md](../../launchers/SEMANTIC_DEPLOYMENT.md).
Build from the neo `workspace` with source roots restricted to the active
`src/aiformula` tree; do not build the entire parent project containing duplicate
packages. Configure the existing calibrated vehicle TF and LLP parameters, and
provide matching current mask, stamped signal capture and odometry producers.
Do not run `allnodes` or `map_drive` alongside the shadow entry.

The bridge itself targets Python >=3.8 with real rclpy and standard ROS messages.
The A3 inference package requires Python >=3.10 and its own compatible runtime.
A3 ROS wrapper availability does not prove stock Foxy compatibility. The new
launch does not install, download, or invent a metric signal producer.

Default final output is `/semantic_drive/shadow_cmd_vel`. The launch and bridge
reject the existing motor command topic. Keep the final output unconnected to
hardware. Read [INTEGRATION_ISSUES.md](INTEGRATION_ISSUES.md) for the missing source
provenance, final-consumption age and physical timeout requirements.

## Validation and rollback

Run scoped core tests and bridge policy tests. Record actual ROS distribution,
interpreter, package versions and commit for any native DDS smoke test; Windows
pure-policy tests are not a substitute. Check all resolved publishers using
`ros2 topic info -v`, verify fault/timeout zero output in the shadow topic, and
save the trace before claiming native acceptance.

For native rollback, stop the dedicated shadow launch and return to the pinned
pre-integration checkout/overlay. Never assume stopping a process stops hardware.
For Unity rollback, use the guarded overlay rollback against matching installed
hashes. Keep baseline recordings and tags immutable; record any new runs under a
new source revision instead of overwriting final63 evidence.
