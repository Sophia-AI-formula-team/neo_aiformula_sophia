# 2026-11-19 MapAssist 红灯联锁修复

| JST | 安装与验证 |
| --- | --- |
| 15:34:47 | 核对 A3 检测 JSON、原像素适配器、车端 0.4.5 诊断状态和旧电机的状态、会话及速度订阅。 |
| 15:35:04–15:35:26 | 真实 ZED/A3 的静态对照收到 146 条带像素数的检测，145 条通过字段和帧龄核验，适配器发出 144 条 False。原事发时段没有 A3 数值录包，未能复现当时适配器掉线，也不能推断像素是否达到 600。 |
| 15:35:53 | 安装像素适配器诊断、兼容传感器 QoS、正式 stop_wait 和原版 session 发布选项。原 allnodes 增加按次选择电机速度话题的启动参数。 |
| 15:36:08 | 在 localhost DDS 域 75 静态运行真实安装的 MapAssist、OA 选择、LYA 和原版 motor。用合成 A3 检测及模拟里程计验证 599/600 边界。旧 motor 在 600 发全零软件 CAN，非红恢复非零。 |
| 15:36:21–15:36:38 | 使 commissioning 默认 OA 未运行时走正常车道，连续复测启动不依赖 OA 首帧、OA 选择和切回、停车、标志过期保持停车、非红恢复。motor 两个文件 SHA-256 一致。 |

## 已装车的自动接线

三条命令分别在原车环境、本机 YOLOP 环境和 Semantic 终端内使用。前两条需保持运行。本次只运行了隔离静态探针，正式模式留到用户安排的实车测试。

```bash
ros2 launch launchers allnodes.launch.py motor_cmd_topic:=/semantic_drive/map_assist_045/diagnostic_cmd_vel
yolop a3
semantic 5.5 vehicle_output:=true
```

6.0 米档使用 `semantic 6.0 vehicle_output:=true`。启动 Semantic 后由旧 motor 的 `stop_wait` 处理自动速度，旧手柄人工优先行为仍生效。像素适配器在 `/semantic_drive/map_assist_045/pixel_stop_status` 发布最新像素数、帧龄、处理统计与发布者计数，停更时留警告。

15:36:33–15:36:56 JST，再次接真实 ZED/A3 并运行已安装的像素适配器。采集 139 条带像素数的检测，像素范围 0–19，发布 137 条 False。启动时的一条约 3.39 秒旧帧被时效筛查挡住。600 像素实测由独立的合成输入联测覆盖。已结束全部静态试验进程。


复现时以当天 Semantic 0.4.5 安装为基础。将本目录的 `pixel_stop_flag.py`、`guarded_stop_wait.py` 和 `semantic_045_commissioning.launch.py` 放入 `~/semantic_045_ws/commissioning`。沿用同档案中的 [OA 选择器](../../commissioning/fresh_oa_switch.py) 和 [旧状态转换](../../stop_wait_preview.py)。`semantic_vehicle_045.launch.py` 为该工作区 `source/workspace/src/aiformula/launchers/launch/` 下的最终版本。将 `motor-input-launch.patch` 应用到原车 `allnodes.launch.py` 时显式指定目标文件。motor 的原文件校验值见 [motor.sha256](motor.sha256)。

## Neo ROS 源码入口

构建实验分支的 `launchers`、`semantic_local_planner_ros` 和 `semantic_planner_core`，然后加载工作区安装环境。使用 ROS 包内入口启动本次接线：

```bash
ros2 launch launchers semantic_045_commissioning.launch.py lane:=outer
```

保留 5.5 m 的 `outer` 和 6.0 m 的 `inner`。默认发布诊断速度及状态。上述已装车的正式接线对应 `vehicle_output:=true` 和 `allnodes` 的 `motor_cmd_topic` 参数。

[整套 ROS 启动](../../../../../../workspace/src/aiformula/launchers/launch/semantic_045_commissioning.launch.py) · [MapAssist 参数](../../../../../../workspace/src/aiformula/launchers/launch/semantic_vehicle_045.launch.py) · [车端接线脚本](../../../../../../workspace/src/aiformula/launchers/shellscript/mapassist_045)

2026-11-19 15:54 JST，基于实验分支 `0528b19` 将车端归档版本移入 ROS 源码树。上层入口改用 ament 查找已安装的脚本；下层 MapAssist 启动和四个接线脚本与先前车端归档一致。应用当天记录的非红恢复和像素到点绕行两个规划差异。工作站 ROS Jazzy 构建线下通过，两个启动入口均可从安装目录解析。46 项回归和 44 个子测试通过。本次提交期间车端 SSH 不可达，源码归档以 15:36 车端静态联测的版本为准。
