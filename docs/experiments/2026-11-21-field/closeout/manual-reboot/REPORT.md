# 2026-11-21 手柄故障与重启回读

时间均为 JST。10:51 更换 Allnodes 入口以加载带手柄时间戳的 motor overlay。中午 OA 故障交还和冻结没有改 Allnodes 启动器或手柄、底盘代码。冻结回读与 10:51 的 field-launch SHA-256 相同。

12:10 的重启前 ROS 日志显示 can0 ready，joy_node 曾打开 Wireless Controller。未留存那次实际手柄动作及底盘指令，无法据此判断重启前故障的直接原因。

12:12–12:13 本机尝试 SSH，重启后重新连通。车机启动时钟曾比本机慢约一分半，最后查询已对齐。蓝牙服务运行，Wireless Controller 已配对和受信任，Connected 为 no，内核输入设备未见控制器。

12:13:54 JST 的最后回读确认 Allnodes 已启动并阻塞在 can0 预检，can0 为 DOWN，joy/motor 节点尚未启动。车机具备原有 can_bringup.sh，当前 Allnodes 等待该接口 up 和 carrier 1。仅做只读观察。

[最新回读](vehicle-current.log) · [ROS 被动采样](vehicle-manual-chain.log)
