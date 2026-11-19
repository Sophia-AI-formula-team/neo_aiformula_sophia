# 2026-11-19 MapAssist 红灯未停车

| JST | 记录 |
| --- | --- |
| 15:27:27–15:28:34 | 启动原车 allnodes、A3 和 5.5 m 的 Semantic 0.4.5 commissioning。 |
| 15:33:37–15:33:41 | 保存三份启动日志、运行参数和 ROS 现场快照。A3 的红灯状态可收，MapAssist 投票窗有 4/4 红灯。状态为 learning、route_matched=false、red_prepare_first_lap、traffic_stop_flag=false。采样窗中 /flag_stop 无可见发布者和消息。 |
| 15:33:41–15:33:56 | 对三组实验 launch 发 SIGINT。原 allnodes 的 joy_node 成为孤儿进程后单独结束。复查三组进程为空。 |
| 15:34 | 核对实际速度和状态接线。12 条 MapAssist 诊断速度为零，候选停车状态为 wait。输出分别留在 /semantic_drive/map_assist_045/diagnostic_cmd_vel 和 /semantic_drive/map_assist_045/stop_wait_candidate。原 motor 启动参数仍使用 /aiformula_control/game_pad/cmd_vel，安装程序监听 /semantic_drive/map_status 和 /semantic_drive/motor_session。原车速度入口在快照中只发现 teleop 发布者，正式 map_status 无发布者。 |

快照来自运行末尾。没有找到覆盖事发时段的 Semantic 录包，实际运动由哪个输入触发、600 像素阈值有没有达到，以及像素适配器在快照中为何不可见，均待查。车端原始证据位于 ~/aiformula_runtime/semantic-red-20261119-1533。
