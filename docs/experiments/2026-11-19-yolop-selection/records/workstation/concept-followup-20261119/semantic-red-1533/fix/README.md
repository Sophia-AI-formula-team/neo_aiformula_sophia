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
