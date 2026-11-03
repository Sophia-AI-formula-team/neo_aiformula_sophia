# race_map

独立运行命令：

    ros2 run semantic_local_planner_ros race_map

裸命令会读取包内 config/race_map.yaml。先构建并 source 工作区；不要同时启动 map_assist，两个节点会占用相同的话题。旧包已备份到 E:\1012\semantic_planning_system_backup_20261026。

## 接线

| 方向 | 话题 | 类型 | 来源／去向 |
| --- | --- | --- | --- |
| 输入 | /original_processed_point_a/b/c | geometry_msgs/msg/Pose2D | lane_0927oa 的视觉三点 |
| 输入 | /lane_stop_flag | std_msgs/msg/Bool | OA 避障状态；true 表示 OA 接管 |
| 输入 | /aiformula_sensing/gyro_odometry_publisher/odom | nav_msgs/msg/Odometry | 车辆里程计 |
| 输入 | /map_assist/traffic_light_input | std_msgs/msg/String | YOLOP 占位输入，消息内容须为 red 或 green |
| 输入 | /map_assist/controller_cmd_vel | geometry_msgs/msg/Twist | LYA 速度命令，经 remap 接入 |
| 输出 | /processed_point_a/b/c | geometry_msgs/msg/Pose2D | withoutkalman_0312；OA active 时本节点停止发布 |
| 输出 | /aiformula_control/game_pad/cmd_vel | geometry_msgs/msg/Twist | 原 LYA 速度转发或红灯停车零速度 |
| 输出 | /map_assist/path | nav_msgs/msg/Path | 完整参考图，RViz 可视化 |
| 输出 | /map_assist/status | std_msgs/msg/String | 建图、融合、红灯和避障状态 |

LYA 文件当前把速度发布到 /aiformula_control/game_pad/cmd_vel。要让 race_map 能拦截停车命令，必须在启动 LYA 的命令上加这个 remap，再由 race_map 把速度输出到原底盘话题：

    --ros-args -r /aiformula_control/game_pad/cmd_vel:=/map_assist/controller_cmd_vel

这项 remap 不在 race_map 的单行启动命令里，不能省略；否则 LYA 会绕过速度门控。视觉三点、里程计和 OA 话题按表连接，不需要改变消息类型。YOLOP 真实话题之后填入 traffic_light_topic 参数。

## 行为

1. 首圈用视觉三点正常行驶并用 odom 记录一圈。红灯稳定确认后停车等待，稳定绿灯后恢复；交通灯红、绿都按最近 0.5 秒内的多数结果确认。
2. OA flag 为 true 时，地图节点不发布控制点，不阻断 OA 的 LYA 速度命令；里程计继续累计距离，但 OA 期间的坐标不进入构图。有效段两端直接用直线连接。
3. 轨迹闭环后与包内拓扑图对齐、融合和平滑。闭环判定半径为 5 米；驶离起点区域的距离按现有逻辑扩大到 10 米。取消最小里程门槛，仍要求驶离起点后回到闭环范围、航向差不超过 25°、至少 30 个有效样本，并通过拓扑对齐质量检查。对齐 RMSE 阈值为 4 米。融合不可用时状态为 odom_only，不会据此规划拓扑绕行。
4. 后续圈继续沿用现有视觉基线和有界地图方向辅助。红灯稳定确认后，车辆沿当前视觉／地图辅助路线靠近拓扑路口；进入节点范围后按官方图选择避开外圈下一条边的内部节点路径。到达出口后恢复视觉。一次持续红灯只执行一次绕行，确认绿灯后才重新允许下一次红灯触发。
5. OA 避障始终优先于红灯与地图输出。若地图融合、定位或内部路线不可用，红灯保持等待，不伪造路线。

phase=learning 表示还在首圈；phase=guarding 且 map_fusion.available=true 表示首圈轨迹可用于地图辅助。detour_approach 表示已确认红灯，正在沿外圈接近拓扑节点；detour 表示正在走拓扑内部路。excluded_samples_omitted 可检查被 OA 排除的采样点数量。

该节点沿用原逻辑的视觉基线和有限方向修正，不是纯地图定位或全程纯地图控制。拓扑内部节点在首圈没有被实测，其位置由官方图和外圈对齐推算。此改动未在 ROS DDS 或实车上运行验证。