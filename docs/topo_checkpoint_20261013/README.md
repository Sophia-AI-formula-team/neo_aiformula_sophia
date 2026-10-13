# 2026-10-13：topo 闭环诊断、自动保存和重启恢复

**状态：本地完成，已做 Python 3.8 测试；未连接、编译或部署到车辆。**
基线为 PR24 的 `67aa5ad3abc0c93559f53306f54334353ba56eb5`。MapAssist 包
`semantic_local_planner_ros` 从 0.2.2 升至 **0.3.0**；车上最后确认仍是 0.2.2，motor 0.4.1。
本变更叠加在 PR24 之上，不将未合并 PR 描述成 main 已发布。

## 为什么改

10月12日最长会话采样950点、202.54米，但仍为 learning。旧代码不满足闭环条件时直接返回，
无法从状态明确知道是距离、朝向、输入还是排除状态卡住。多次重启又清空内存轨迹。
稀疏历史录包不能精确还原节点的每次闭环判定；本次增加实时诊断，不倒推或篡改昨日验收。

## 改了什么、怎么改

| 部分 | 实现 |
| --- | --- |
| 闭环诊断 | `map_assist` 的状态 JSON 新增 `closure`，在原有判断条件之外计算可观察的诊断快照；原闭环半径、航向和算法不变 |
| 终端显示 | 每5秒输出距起点/阈值、朝向差/阈值、累计路程、采样数、全部阻塞项及存档状态；不用额外启动诊断节点 |
| 错误解释 | 保留最近建图重置原因、参考线构建异常；ABC指出缺失、超时、非有限值、不在前方或时间不齐；已有odom但未闭环时fusion显示`first_lap_not_closed` |
| 保存 | `topo_checkpoint.py` 保存版本化JSON，含原始采样、排除标记、累计距离、离开起点标记、闭合点、匹配进度、最近odom位置/时间/坐标系 |
| 文件可靠性 | 内容SHA256、配置/官方地图指纹、格式/大小/采样数检查；临时文件+fsync+原子替换+目录fsync；Linux文件锁防止两个节点写同一文件 |
| 性能 | ROS回调复制状态，单独线程完成周期性JSON编码和写盘；定期存档不等待磁盘。闭环、重置、正常退出会等待完成，以保存边界状态 |
| 恢复 | 启动时读取并等待首个有效odom，连续性检查通过才恢复；未闭合首圈继续采样，已闭合图用同一融合算法和参数从采样重建并发布reference path |
| 接线 | 现有`semantic_vehicle.launch.py`直接设置默认存档路径；原`ros2 run semantic_vehicle_wiring semantic`无需新参数 |

`semantic_planner_core`、官方OSM、LYA、motor、CAN轮速算法、红绿灯投票与放行规则没有修改。
车辆入口依旧`enable_correction=false`，topo速度/三点依旧是私有诊断输出。
没有自动左转，也没有放宽闭环条件或加入GPS/SLAM重定位。

## 怎么看“具体卡在哪里”

继续使用原命令：

```bash
source /home/workspace/install/setup.bash
ros2 run semantic_vehicle_wiring semantic
```

部署新版后，终端会出现如下格式（以下是说明示例，不是实车测量）：

```text
Topo blocked: start=39.58/1.5 m, heading=100.7/25 deg, path=202.54/40 m, samples=950/30; blocked=within_start_radius,heading_aligned; checkpoint=saved
```

状态话题仍为 `/semantic_drive/map_status`。需要完整信息时：

```bash
ros2 topic echo /semantic_drive/map_status
```

`closure` 字段：

- `distance_to_start_m`、`heading_error_deg`：当前odom到**本次采样首点**的距离和最短绝对朝向差，缺输入时为null。
- `start_pose`、`current_pose`：`[x米,y米,yaw弧度]`，同一odom坐标系。
- `checks`：各条件true/false；`blockers`列出**所有未满足条件**，`blocker`只是列表第一项，不表示唯一根因。
- `odom_fresh/start_recorded/left_start_area`：有新鲜odom、有首点、曾离起点超过3米。
- `within_start_radius/heading_aligned`：回起点≤1.5米、朝向差≤25°。
- `enough_samples/enough_distance`：≥30点、累计≥40米。
- `recording_points_valid/not_excluded`：ABC有效，并且没有避障、岔路、恢复等待或交通等待；细节看`point_issues/exclusion_reason`。
- `restore_ready`：没有等待或拒绝恢复；具体看`persistence`。
- `build_error/last_learning_reset_reason`：构建失败或原算法重置的原因不会被普通tick状态覆盖。
- `state=complete`：已有闭合reference；它**不等于官方图融合成功**，后者仍看`map_fusion.available`。完成后`checks`仍显示当前实时条件，`blockers`为空，避免把之后离开起点当作首圈失败。

这些是状态时刻的快照；每个odom回调仍按原算法判定闭环。旧的“提前回起点但里程不足则重新学习”逻辑保留，原因会显示为`returned_before_min_lap_distance`。

## 保存在哪里、怎样恢复

车辆入口默认保存到运行用户的：

```text
~/.ros/aiformula/topo_map/checkpoint.json
```

以control用户运行即`/home/control/.ros/aiformula/topo_map/checkpoint.json`。按1秒间隔提交有改动的快照，
正常退出/闭环/重置额外保存。突然断电只能恢复**最近一次成功写盘**；若磁盘慢、写入失败或强制退出，
可能损失超过1秒的末尾数据，不能承诺零丢失或物理断电耐久性已验收。

`persistence.state` 会显示 `new_map`、`waiting_for_odom`、`restored`、`saved`、`save_failed` 或 `restore_rejected`。
另有`path/error/saved_samples/saved_distance_m`；等待恢复时当前采样数仍为0，已保存点数在persistence中。
成功恢复记录一次日志，后续自动保存会将状态从restored变成saved。

恢复必须满足：

1. JSON版本、校验和、官方OSM内容及关键建图参数相同。
2. odom frame和child_frame相同，源时间没有倒退。
3. 当前pose距离存档最后pose≤1.5米、朝向差≤25°。

通过检查后，允许semantic停机期间的时间间隔，不按连续运行的2秒odom间隔阈值清空恢复结果；
恢复后的正常实时odom跳变检查继续有效。未观测的停机连接段标记为excluded，不伪装成可信视觉轨迹。
已闭合地图从保存的完整采样/闭合点重建，保留排除标记和进度。

**适用于同一连续odom坐标下重启semantic。** 如果同时重启里程计、移动车辆或更换坐标系，可能拒绝续图。
相同frame名称和近邻pose也不能证明绝对坐标原点从未改变；这不是重定位算法，不能承诺断电后在任意位置续图。
拒绝时暂停topo记录，保留原文件供检查；交通投票/状态发布仍继续，恢复故障不新增motor停车条件。
同样，地图恢复不重放旧速度、旧红绿灯投票或旧手柄状态；会话重启遵循原有交通初始化行为。

读取失败/不兼容不会静默覆盖存档；写失败报告`save_failed`并重试。第二个写入者报告`checkpoint_in_use`，
不会覆盖首个节点的文件。**这只保护存档，不代替原系统避免重复启动节点的要求。**

确定要丢弃当前存档、重新建图时，沿用已有reset话题（这是主动清图操作，不是启动要求）：

```bash
ros2 topic pub --once /semantic_drive/reset std_msgs/msg/Bool '{data: true}'
```

它清空内存并写入空存档，下一次启动不会复活旧图；需要保留原图请先备份checkpoint.json。
原运行中的odom跳变等重置策略也会更新空存档，不新增历史地图轮换管理。
存档被其他进程占用时应先解决重复实例；发reset不能取得别人的文件锁。

裸MapAssist、旧shadow和bag配置默认`checkpoint_path=''`，不会写车辆存档。
`use_sim_time=true`即使配置了路径也禁用存档，防止仿真污染实车地图。

## 本地测试与证据

Python **3.8**，ROS消息/Node使用最小stub，运行真实MapAssist回调和原融合算法：

- 136项MapAssist/闭环/存档/launch测试，其中新增28项；覆盖半圈续图并真正完成闭环、已闭合reference重建一致、输入缺失、距离/朝向/坐标系/时间拒绝、损坏/版本/配置拒绝、写失败保留旧文件、文件锁、重置、退出保存，以及磁盘阻塞期间继续处理回调。
- 6项MapAssist→MotorGate生产回调联动回归（包括无ABC反复红绿切换）。
- 27项motor门控兼容性回归。

合计 **169 passed**。见 [test_receipt.json](test_receipt.json) 和 [source_manifest.json](source_manifest.json)。
另用主机Python 3.14做wheel内容检查，确认新模块进入安装包；3.8测试环境没有setuptools，这不是Foxy编译。
没有ROS安装环境或车辆连接，因此**没有本次原生DDS、Foxy colcon编译、真实断电恢复或实车一圈验收**。
昨日DDS/实车记录仍属于昨日版本，不挪用作本次验收。

## 后续上车范围

用户本次只授权本地和GitHub。车连上以后按新的部署指令再操作：

- 核对车上最后版本与manifest的基线；保存车上源/build/install及已有checkpoint。
- 仅更新manifest列出的运行文件：MapAssist主文件、两个新模块、setup.py/package.xml、semantic_vehicle.launch.py。
- 编译`semantic_local_planner_ros`和`launchers`，确认安装树包含两个新模块，再按原入口启用。
- 不用整仓覆盖车辆allnodes、相机标定、A3运行环境或其他人的实验修改；车辆allnodes本来就有平台覆盖层。
- 回滚应配套恢复上述运行文件；0.2.2不识别新存档，文件可以保留但不会自动读取。
