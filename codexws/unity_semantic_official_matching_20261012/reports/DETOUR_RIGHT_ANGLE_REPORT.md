# DetourTracker 直角拐点条件性失效复现

结果：**条件性失效已复现**。仅执行当前 commit `434fdc43d00399808adf7e548d491a5dfdc5bffa` 的原始 `DetourTracker`，源码哈希和 Git 状态前后不变。

## 证据边界

这是人工给定位姿的算法反例，不是实际车辆试验。没有导入/改写 MapAssist 状态，没有预置学习完成地图，没有 ROS、模型、LYA、plant 或 Unity 操作。最后一次 90° 航向是明确的外部反事实输入，不能当作车辆完成转弯的证明。真实 MCP 首圈学习和路口闭环验证待执行。

## 复现

折线路径为 `(0,0) → (5,0) → (5,5)`，单位米；参数使用 MapAssist 默认值：节点半径 0.8m、最大偏离 2m、lookahead/spacing 各 0.5m、入口半径 3m。

| 人工输入阶段 | (x,y,yaw) | tracker 段号 | 有ABC | 原始原因 |
| --- | --- | --- | --- | --- |
| entry | (0.0, 0.0, 0°) | 0 | True | tracking |
| straight_before_switch | (4.0, 0.0, 0°) | 0 | True | tracking |
| within_waypoint_radius_heading_still_straight | (4.3, 0.0, 0°) | 1 | True | tracking |
| overshot_corner_heading_still_straight | (5.1, 0.0, 0°) | 1 | False | target_behind_vehicle |
| same_stationary_pose_repeated | (5.1, 0.0, 0°) | 1 | False | target_behind_vehicle |
| counterfactual_external_heading_alignment | (5.1, 0.0, 90°) | 1 | True | tracking |

从 `(4.3,0,0)` 开始，节点距离 0.7m，tracker 切换到第二段，但车头仍朝东。之后若实际状态成为 `(5.1,0,0)`，下一段三个点都在 `x=5` 直线上，在车体坐标中 x 均为 -0.1m，触发 `target_behind_vehicle`。重复相同静止姿态仍被拒绝，tracker 状态不推进；没有自动原地旋转恢复的分支。

这个反例证明“越过拐点而未完成转向”会令选点不可用，不证明真实车辆必然到达该姿态。车辆惯性、角速度响应或先前转向也可能避免此状态或改变它，必须由闭环证据回答。

原实现位置：

- `semantic_planner_core/semantic_planner_core/traffic_route.py:248–254`：折线逐段跟踪，不平滑转角。
- 同文件 `314–343`：节点半径内换段；后方目标被拒绝；`347–349` 只在有效输出时提交状态。
- `semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py:736–766`：每新 odom 调 tracker，不可用时 `detour_motion_ready=False`。
- 同文件 `575–577、614–623`：不可用的 detour 通过原速度门控输出零。此处是静态阅读结论，本脚本没有执行 MapAssist。
- `semantic_local_planner_ros/tests/test_map_assist.py:494–509、1091–1095`：已有测试按下一段方向直接提供 yaw；这些测试覆盖状态机，不覆盖原LYA与plant产生该转向的过程。

## 8m 接近目标与 3m 绕行入口

两者坐标含义不同：接近参数是**采集时车头沿当前车体前向轴到信号的保守距离下界**；绕行入口是**当前 odom 参考点到已映射路口节点的二维欧氏距离**。核心默认目标是 8.5m，上一轮 Unity 运行覆盖为 8.0m；不能把覆盖值说成核心默认，也不能直接将这两个距离相减作为实车定位结论。

`_wait_for_signal("signal_target_reached")` 使用默认 `fault=False`（map_assist.py:410–411、433–434、468–473），**正常目标停车不会新设置 `signal_wait_latched`**。已有结构故障锁存是另一回事。正常目标等待仍可在所有条件满足时由 `_update_traffic` 进入 detour。

但是 `_update_traffic` 还要求 reference、融合、内部映射、当前匹配和新鲜 odom，并要求车体参考点在路口节点 3m 内（667–714）。若信号与该路口节点基本同址，且车头对准接近，约8m灯前停车通常仍在3m入口外；车辆会保持 `red_wait_near_junction`，没有为进入路口继续向前爬行的逻辑。若路口在信号上游足够远，或实际地图几何不同，则可能满足入口；**不能把该条件性几何冲突写成所有场景必然失败，更不能称目标停车被默认 fault latch 锁住**。冷启动没有 reference 时，原因首先是 `red_wait_first_lap`。

## 最小下一步

保持原 semantic/LYA/车辆参数，先在真实 MCP 场景完成学习并记录 `map_fusion.available`、`topology_mapping.available`、`route_matched`。仅在真实符合入口的位置触发红灯，保存 route node IDs、实际 odom/yaw、原ABC、v/w、节点距离和道路/车体边界；不要手工改车辆yaw或向控制器注入地图真值。先验证进入、一个真实转角和出口新视觉重获，再讨论是否需要轨迹平滑或入口协调修复。LEFT 感知来源标签不是左转完成证据。

JSON 记录全部人工输入、结果和源码哈希。复现命令：

```powershell
python "E:\Mess\Projects\Programming\aiformula\workspace\semantic_left_turn_20261012\analysis\reproduce_detour_right_angle.py"
```
