# 2026-11-19 MapAssist 首圈和红灯排查

| JST | 观察 |
| --- | --- |
| 16:17:26 | 同车的另一份交接记录列出原车 allnodes 和独立 oa_2。allnodes 命令没有指定 motor_cmd_topic。 |
| 16:18:20 | 交接探针直接读取旧电机 gate_status，亦确认轮速反馈静止、CAN 全零。旧电机在该采样时刻报告 semantic_active=false、manual_held=false、command_timeout。随后结束这两组进程。这份记录没有采集红灯经过时的状态。 |
| 16:20:33–16:22 | SSH 多次无路由或超时。本机仍在实验网，目标邻居解析未成功。准备按时间提取车端 ROS 启动和节点日志，优先检查地图闭合、拓扑融合、停车旗标和旧电机模式。 |

## 验收字段

- 首圈闭合看 First lap complete 或 odom_reference_ready=true。phase=learning 表示首圈还在记录。
- 路线匹配看 route_matched。正式拓扑融合另看 Topology alignment accepted、map_fusion.available 和 phase=guarding。
- 红灯经过时对齐 A3 颜色投票、实际像素、flag_stop、MapAssist traffic_mode、旧 map_status 以及电机 gate_status。事后停车快照只说明当时的接管状态。

[16:17–16:18 停车交接原始取证](../concept-drive-fix/physical-handoff.log)

## 取回的本轮证据

| JST | 结果 |
| --- | --- |
| 16:08:59 | 原车 allnodes 启动。其电机最终日志保留了速度订阅 /aiformula_control/game_pad/cmd_vel，原车话题仍由 teleop 发布。 |
| 16:09:06 | Terminator 可见区和现存进程均确认 YOLOP 使用 A4Light 的 live 入口。A4Light 的安装路由有 /cone_mask、/cone_bbox_image、灯色和带时间戳的检测话题。 |
| 16:10:32–16:12:22 | MapAssist 五个进程组启动并在 SIGINT 后退出。组内没有 motor_gate_session，符合 commissioning 的诊断默认参数。已有 LYA 发至 MapAssist 私有入口。旧 motor 并未订阅 MapAssist 的受控速度。 |
| 16:10:34–16:10:41 | 像素适配器启动时没有收到新鲜检测。最后一条警告累计 11 条 stale_or_future_image 拒绝，latest_pixels=null。这些是启动阶段数据。 |
| 16:10:37–16:12:22 | map_assist 节点日志只有开始学习一行，无首圈闭合、拓扑融合附加成功。与 MapAssist 配套的 OA 选择器处理 1,431 组 ABC，未记录切向 /oa_point_*。配套 LYA 有 1,370 条控制日志。没有这轮的 MapAssist status/CAN 录包。 |
| 16:22:59–16:25:09 | SSH 恢复，保存 ROS 日志、Terminator 可见区匹配行、进程和录包索引。终端可见区直接确认运行的是独立 oa_2。取证时已无 MapAssist、OA、旧 motor 会话。 |
| 16:24:26–16:24:45 | 隔离域 196 使用车上实际 oa_2、MapAssist 和合成锥桶。2,448 像素时，裸 oa_2 在 /lane_stop_flag 发布 16 条 True，同时作为常规 /processed_point_a 额外发布者，MapAssist 的 avoidance_active=true 为零。加入四项重映射后，/point_flag 收到 15 条 True、/oa_point_a/b/c 各 30 条，MapAssist 收到 8 条 active 状态，选中 OA 的点，并在清掉障碍后切回正常车道。无实车 CAN 发布者。 |

原始车端运行中锥桶 ROI 是否超过 1,000 像素没有同步采样。实车经过红灯一刻的票选、像素数和运行状态也没有录包。已确认的两条接线断点为旧电机入口没有进入 Semantic 受控链，以及裸 oa_2 的旗标和 ABC 没有进入 MapAssist 的避障切换话题。ROS 记录覆盖上述启动会话，终端附件只含可见区相关行。

[终端可见区](terminal-visible.txt) · [首圈与 LYA 计数](selected-terminal-log-summary.txt) · [OA 隔离对照](oa-static-result.json)

16:26:42 JST 复核 ROS 启动目录和进程。16:24 的 MapAssist 日志属于上述隔离避障测试，16:26 两份目录来自启动参数预检；没有发现比 16:10 更新的 MapAssist 实车会话。16:10 的诊断启动可以并行学习，旧电机接管与首圈闭合是独立条件。首圈未记录闭合成功的具体原因无法从现存日志区分。
