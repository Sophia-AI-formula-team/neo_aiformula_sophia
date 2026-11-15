# outer_joint_03：panel-only 最终接线实跑

2026-11-16。本报告根据 completed run 的 `result.json`、events、worker ticks 与 trace，一次主 metrics 解析及 seq544–550 小段绿灯证据核对；没有重跑模型/回放/Unity，也没有改 source/live。主数据 `outer03_metrics.json`，恢复窗口 `outer03_green_transition_witness.json`。

## 实际通过与失败

- 同一原 semantic 0.4.0、原 LYA、最终 panel-only 接线完成 **2 个物理圈、563.1467 m、230.7783 s**。worker clean exit0，无强制结束。试验 `accepted=false` 正确保留，因为红灯内部绕行没有发生。
- 已越过此前 outer01/02 在约 96 m 的误红锁停区域。stop 前 1,036 个真实 detector receipts 全部标记 `calibrated_rgb_panel_only`：RGB red 62、RGB green 78、none/unknown 896；没有任何 a3_model source。62 个 red 的捕获时刻全部位于实体红阶段内，没有第一圈假红。
- 原包首次 odom reference 在 sim107.720 s 生成，长度281.356 m；随后 physical lap event t108.254 s 确认首圈。**这是里程计外圈闭合，不是官方拓扑正确对位。**
- 官方融合最终 `available=false / ambiguous_correspondence`，graph_frame=`official_unaligned`，official_node_positions_odom为空，topology_mapping不可用；`first_official_fusion` 与 `first_detour` 均为空。没有内部道路观测建图、没有真实红灯左转。

## 红停与绿恢复是本场真实链路结果

实体于 t108.254 s 转红。`package_observed_red` event t109.753 s 给出原 native 3 样本中 red=2/unknown=1，模式进入 wait，原因 `red_wait_map_fusion`。停车后当前 RGB red 持续可见，原 source 没有 crash。

稳定停车的 Body pose 为 (world x=72.1296539, z=-5.08980656)，实体灯 root=(75.31588,16.3327065)。水平 Body→灯 root **21.658 m**，沿当前车头方向投影 **21.396 m**。这是独立评价值，不进入控制；不是保险杠/相机/停止线距离。本次新包仍是确认红灯后立即停车，**没有实现旧 8–9 m 接近逻辑，也没有混入 bbox 距离模式**。harness 自带 `front_gap_evaluation_only` 使用固定 world-z=15 平面，故本报告不把该字段冒称真实灯牌距离。

等待12秒后，harness将 `redDetourFailed=true` 永久保留，并于 t122.427 s 切实体绿以继续检验第二圈。seq546/547/548 分别于 t122.909/123.129/123.348 收到当前 RGB green，confirmed_frames=2/3/4，车速均约零；seq548 已记录 native `normal/green_resume`，车尚未运动。seq549 记录原native green=3/3，同时开始实际运动；之后完成第二圈。该恢复来自停止位置的持续真实绿灯图像，不是车辆先移动才找回灯。

trace 附带的 native status 是此前异步 tick 快照，常在下一帧读取时因0.5秒窗口只剩2样本显示 unknown；例如seq548仍显示瞬时unknown，但状态已green_resume。故“first_native_green_after_scheduled_green”在主metrics的trace采样里晚到seq549；不得误解为绿灯必须移动后才触发。event/状态转换与seq546–548静止图像共同给出实际恢复证据。原unknown不释放red latch的语义未改。

## 官方图失败仍独立成立

red_detour_failed event中的matcher为原 joint_ordered_node_candidates，69候选，best RMSE=.407436m、alternative=.507322m、ambiguity_gap=.107533m，小于原.3m margin；alternative_node_spread=2.851812m，大于原1m位置分歧门槛。source按原契约拒绝对位，之后保持 `red_wait_map_fusion`。未放宽阈值、未部署迭代budget候选、未手工贴图或注入Unity定位。

结论：本场 **两圈、红停、停止位置转绿恢复通过**；**official topo融合、红灯内部左转失败**。odom闭环不能证明内部topo正确。
