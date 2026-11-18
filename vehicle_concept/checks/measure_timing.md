# 车上被动测速

`measure_timing.py` 只订阅 ZED 左相机、B1.4 感知、决策和 `/concept/auto_cmd`。可以在电机输出关闭时测量。原来的命令及 JSON 字段继续适用。

```bash
python3 vehicle_concept/checks/measure_timing.py --seconds 60 --planning-hz 5 > timing-5hz.json
```

用 15 Hz 测试时，把 `--planning-hz` 改为 15。这个参数仅决定统计窗口的宽度；驾驶节点的真实设置显示在 `driver_planning_hz`。原来的 `processing_ms` 反映软件内计算；端到端耗时请看 `capture_to_observed_twist_s`。

| 字段 | 含义 |
|---|---|
| `camera`、`perception`、`decisions`、`auto_cmd` | 各话题实际收到的源时间范围、唯一时间戳数、向前更新数、重复、逆序、源帧频与间隔 |
| `capture_to_observed_twist_s` | 匹配同一个相机时间戳，从采集时间到本工具收到实际新 Twist 的时差，包含 DDS 传输和观察端调度 |
| `observed_twist_intervals_s` | 按本机单调钟计量，连续两条源时间前进的有效 Twist 到达的间隔。两种耗时都报 p50、p95、p99、最大值 |
| `cmd_with_camera`、`cmd_without_camera` | 实际新 Twist 中可以或无法与本窗口收到的相机源时间戳匹配的数量。重复、逆序、非有限指令各自计数 |
| `capture_to_twist_over_200ms`、`twist_intervals_over_200ms` | 分别统计超过 200 ms 的合拍指令耗时和实际更新间隔。严格按 200 ms 计数，轻微抖动也会出现 |
| `periods_without_new_twist` | 从第一条观测到的相机消息开始，划分完整的 200 ms（5 Hz）或 66.7 ms（15 Hz）窗口，并统计其中没有新 Twist 的窗口 |
| `rate_skipped_confirmed` | 同一驾驶会话相邻决策报告给出的主动抽帧累计数之差。旧版本或缺少两条可比报告时为 null |
| `statuses`、`reasons`、`invalid_messages` | 分别列出正常、shadow、过期、无效决策的次数和原因，以及观察端无法解析的消息数 |

`perception_without_decision` 统计观察窗口内有感知而没有同拍决策的情况。主动抽帧、观察端丢包或窗口边缘都可能贡献这一数字。相机、感知、指令的缺口仅能描述在该订阅端看到了什么。

在车上核实 ZED 采集时间和计算机使用同一时钟后，可加 `--vehicle-clock-confirmed`。工具还检查唯一的相机发布者、模拟时钟话题和源时间是否来自未来。未核实时 `eligible_for_vehicle_timing_evaluation` 为 false。容器测试、仿真和物理车辆应分别保存记录。脚本的零退出码表示采到了驾驶计算记录。
