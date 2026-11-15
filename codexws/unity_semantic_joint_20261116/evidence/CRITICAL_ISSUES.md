# 1115 原包问题分析：保留原行为后再实跑判定

日期 2026-11-16；源码提交 `6b71267`，MapAssist SHA256 `9b8590588e707173d6e623f144c41e6c1dd97531f92ce4282415eb4dc7eedb5b`。本文在任何算法修复之前写入。本审计未修改 semantic 算法、官方图、原 LYA、摄像机、灯位或车辆物理。

## 1. 红灯先停车，与近路口才能启动绕行之间存在不可达条件

代码证据：`map_assist.py::_update_light_vote` 在确认红灯后从 normal 进入 wait；`_velocity_hold_reason` 对 wait 返回停止原因，所以原门控输出零命令。`_update_traffic` 则要求 `reference`、已接受 fusion、mapped topology、可信且新鲜定位；其候选路口的进度距离必须小于 `detour_entry_radius_m + progress_tolerance_m`，实际二维距离必须不超过 `detour_entry_radius_m`。本包 YAML 的入口半径是 **3 m**。

确定的控制条件是：**如果真实红灯确认后车辆最终停在全部合格路口 3 m 范围之外，那么在红灯持续、OA不接管、没有外力移动的情况下，原包持续给零命令，不能自己驶入 3 m 入口以触发 detour。** 制动滑行可能暂时进入范围，因此“检测瞬间距路口大于 3 m”本身还不是一次完整失败证据；需记录实际停稳位置和 reason。原包没有接近路口阶段，也没有距离输入；旧版 8–9 m 停车策略和 bbox 测距没有在此实现里保留。

对本次验收的影响：即使首圈 topo 融合成功，第二圈红灯也可能停在 `red_wait_near_junction`，导致无法完成用户要求的内部路线行驶。这是与需求直接相关的可达性风险，不是 LYA 未接线造成的现象。

实跑记录至少需要：真实检测颜色与投票时刻、首次 wait 的 odom、停稳 odom、当前 progress、各候选映射路口及距离、`traffic_reason`、实际 v/w、detour 是否出现。不要为让演示成功而延迟真实检测、屏蔽远处红灯、移动灯/相机、注入近路口位姿或替换官方图。

本轮处置：保留原行为和失败视频；尚未选择或实施算法修复。若实跑证实，修复方案应作为独立版本解释“红灯所封闭的前向边、可安全靠近的路口入口、允许的接近动作”之间的规则，再验证，不能仅把 3 m 阈值无条件放大。

## 2. 官方图与外圈可能不能唯一对应

新的 `test_joint_alignment` 明确要求官方 OSM 加合成圆角外圈返回 `ambiguous_correspondence`；core 103 项测试中 5 个旧融合成功断言现在因同一原因失败。保留歧义拒绝可避免任意选图，但意味着“首圈 odom reference 闭合”不能自动等于“官方 topo 绑定正确”。失败时红灯保持 `red_wait_map_fusion` 是原包已实现的结果。

本次不修改官方图、不注入仿真标定、不放宽匹配门限。实跑应分别报告 odom 闭合、fusion 接受/拒绝、官方路口是否对应真实路口。内部节点是官方图与外圈锚点推估，不是首圈实际看见并建成了内部道路。

另有一个原 core 测试观察到最终平滑后节点误差 **0.4636301568 m**，超过原断言 **0.3 m**。这是实在的几何误差，不能和原因字符串差异合并成“旧测试已过时”而略过。实际赛道可容许误差尚未在本审计中测量。

## 3. 旧 ROS fixture 的同步假设和线程清理问题

ZIP 自带 `test_map_assist.learned_official_loop` 在提交异步 fusion 后立即要求融合成功；相关 failure 的实际状态是 `alignment_pending`。**这些同步断言失败本身不证明运行时 fusion 错误。** 新设计明确允许融合尚未完成时用闭合 odom reference 辅助视觉，故旧“无融合就完全不修正”断言也与新设计冲突。

旧 fixture 未注册 `close_fusion_worker` cleanup，合批测试累积后台任务。70 项断言在 23.360 秒完成后进程仍未退出，60 秒硬限被终止；其中新 joint 用例的真实融合等待发生 20 秒 timeout。隔离只运行新 joint 和 launch 后 **19/19 通过、2.287 秒正常退出**。这说明需要隔离解释之前的 timeout，不能将其直接标为 runtime bug；也不能丢弃原合批失败日志。

新 joint 成功用例使用非对称合成图，并直接把 node 置于图路口朝向。它证明真实融合、内部 route 创建及消息/所有权控制，但没有实际 LYA 车体运动。因此 **19 项通过不证明 Unity 已完成左转**。

本轮处置：不改原测试和算法；报告合批失败及隔离成功。Unity worker 在退出时调用原 `close_fusion_worker`，运行日志记录关闭时是否仍有融合任务。没有恢复旧 checkpoint 功能。

## 当前状态

worker 和接线已冻结；纯接线 11 项 smoke 通过。没有原算法修复，没有追加长测试。Unity 原行为两圈、第一视角/semantic 视频及上述候选风险的运行时判定，由主管继续并附加实际数据。

## 4. 首场实际 Unity：Joint C# 请求遗漏来源字段（接线问题）

实跑 `runs/inner_joint_01` 已结束：行程 `0.00168037415 m`，0 圈，首圈 reference 未建立，未调度红灯/绕行。第一帧实际推理 `11725.1553 ms`，捕获年龄 `12.0308211 s`，被既有 0.5 s TTL 拒绝；第二帧推理 `23.8259 ms`、年龄 `0.1918842 s`，已是有效新鲜输入。不能用第一帧的冷启动耗时推断稳态推理一直慢，也不能放宽 TTL 来接纳旧帧。

新鲜第二帧触发 worker exit 1。源码证据：staging `AIFormulaSemanticPlanning.cs` 在构造 vision 后，仅 `NeoStopWaitMode` 分支补充 `vision_commit=lane.commit`；Joint 分支遗漏该字段。worker 校验 `vision_commit` 与固定 LLP/LYA provenance，所以第二帧必然被拒绝。旧 stale 第一帧提前返回，因此推迟了这个接口缺口的暴露。

已用真实第二帧 candidate 按该 C# schema 重建请求，保留其 ABC、detected green、age、epoch、capture stamp，回放到冻结 worker；没有运行模型、Unity 或 fusion。因原始请求未留存，tick sequence 使用 619，odom 使用最新记录的近静止 odom，这一重建限制已经写在 receipt 中，不能称为逐字节原请求。

证据 `inner01_replay/receipt.json`：

- 原 C# schema：0.278 秒，exit 1，精确异常 `ValueError: Visual ABC provenance differs from the pinned LLP/LYA source`。
- 仅补实际 `candidate.commit`：0.282 秒，exit 0，`vision_result=accepted`，真实 MapAssist→原LYA→门控 `v=2.2248456404, w=0.6740196235`，shutdown clean。

**最小修复是 C# Joint 分支也发送实际 lane.commit；worker 来源校验正确，保持不变。** semantic 源码无需修改。主管已将此修复交给 C# 接线负责人。另一个前瞻接线问题：C# corridor 消费端需要在 Joint detour 时尊重 worker 的 `corridor_guard.applicable=false`，否则已不控制车辆的旧视觉 prior 可能错误拦截 topo 输出；这次未进入 detour，不能把它称为已实跑复现。

此次 C# Fail 的错误状态被 follower Stop 覆盖，原 worker异常未落盘，妨碍诊断。主管授权仅补 worker 异常证据输出：在 own output-root 保存 `error.json` 和白名单输入字段的 `failed_request.json`，stderr 同时记录异常；不得改算法/来源校验/门控行为。修改前保存冻结 worker 到 `baseline_worker`。后续实际帧回放会再次验证缺 commit 仍失败、准确 commit 仍通过。

已完成该纯诊断修改。旧 worker 原样保存在 `baseline_worker/semantic_joint_worker.74940f8e.py`；新 worker SHA256 为 `cff1e44ffd6fe5fc2184c6659e933277a6db79454c1f7c04afe23f5ac8591674`。`inner01_replay_diagnostics/receipt.json` 记录再次回放：缺 commit 仍报同一异常且 `error.json` / `failed_request.json` 实际存在；准确 commit 仍输出同样的 v/w 并 clean shutdown。两次子进程合计不到 1 秒；没有修改 semantic source、来源校验或控制行为。

## 5. inner04 实跑：里程闭合成功，可信数据缺口导致官方融合拒绝

`runs/inner_joint_04` 已完成，worker clean exit 0；累计行程 **308.3987 m**，实际完成 **1 圈**，没有达到 2 圈要求。第一圈实际计数事件 t=260.9938 s、行程 259.4106 m；原包更早在 t=260.6000 s 建立 **259.5646 m odom reference**。这证明里程轨迹闭合，**不证明 topo 建图已正确**：fusion 全程不可用，原因为 `excluded_gap_too_long`；`official_node_positions_odom` 为空，graph frame 为 `official_unaligned`，没有物理路口对位可评估。

该场在 sim t=168.4208 s 通过 MCP 将本 Unity 渲染从 fps=-1/vsync=1 改为 fps=30/vsync=0；physics step=0.01、time scale=1、相机、语义参数和时效门限保持不变。它是**混合渲染设置的失败基线**，不是固定设置验收，也不是独占 GPU 试验。实际检测时效统计：

| 区间 | 检测帧 | 推理中位数 / p95 | 捕获年龄中位数 / p95 | 超过 0.5 s 的帧 |
| --- | ---: | ---: | ---: | ---: |
| 调整前 | 80 | 692.37 / 4447.84 ms | 0.8794 / 4.5912 s | 48 |
| 跨过调整 | 1 | 1262.99 ms | 2.1253 s | 1 |
| 调整后 | 754 | 26.75 / 43.63 ms | 0.1299 / 0.1650 s | 0 |

966 个学习样本中 33 个 excluded，集中于开始阶段。不能仅按原始连续 excluded 样本数量判断安全性：`rough_route.RouteReference` 要求重采样段的**两端都可信**，交替不可信样本间的小段因此会合并。实际 reference mask 的最大连续不可信段为 **11.7996 m**，跨闭合段另有 **2.5942 m**。`route_fusion._fit_trace` 原门限为 `max_detour_gap_m + 2*spacing_m = 2.4 m`，拒绝有直接数据依据。没有证据支持放宽该保护。闭合时刻距首点约 1.40 m，原 `build_odom_reference` 也按设计将长于 0.4 m 的未观测闭合插值标为 excluded；干净重跑仍须检查该段是否超限，不能假设渲染改进必然使 fusion 接受。

最小下一步是保持固定 30 fps，从头采集新的第一圈；不要复用受延迟污染的 reference，不改 semantic 或门限。新场若仍被拒绝，再用新场证据区分闭合缺口、匹配歧义及真实几何错误。当前记录保留为独立失败视频。证据：`inner04_metrics.json`、`inner04_behavior.json`、`runs/inner_joint_04/render_cap_transition.json`。

## 6. inner04 转绿已恢复，随后感知将锥桶框报告为红灯而重新停车

第一次真实红灯确认在 t=262.1700 s（3 red / 3 samples），原包进入 `red_wait_map_fusion`。红灯 12 s 后 harness 将实际面板切回 green；在 t=280.0000 s（2 green / 3 samples）原包进入 `green_resume`，车辆随后真实行驶约 46 m。因此不能将本场概括为“绿灯不能继续”。

第二次等待在 t=298.5900 s：detector seq674/675/676 连续输出红灯，投票变为 2 red / 3 samples。该时刻评估日志中的物理面板为 green；`signal_truth` 只用于事后评价，没有接入控制。进一步核对原始 detection：三帧的 `class_id=10, traffic_light_state=red` 框，均与同帧 `class_id=91, traffic cone` 框**坐标完全相同**。例如 seq675，两者 bbox 同为 `[261.389526,180.926987,266.434479,188.844864]`，red light score=0.504321。`signal_panel.source=a3_model`，不是 Unity 真值注入或 topo 推算。保存的 `overlay_00675.png` 显示前方低矮目标处重叠 cone/light 标签，未见对应的灯牌。

这是需要独立定位的**感知输出类别冲突/二次分类适配问题**；从当前帧不能断定是模型原始多类输出还是 detector 后处理生成了重复类别。应先读实际 detector 的 class10 产生路径，保留图像与原始检测作为失败证据。最小修复应只约束不成立的 light 输入，且必须凭真实类别、框几何或像素证据，不能用仿真 light truth、车辆位置、手工路线或全局屏蔽红灯。semantic 在确认红后 unknown 不解除等待是原包已明确的设计，目前没有理由更改该安全语义。

第二次停车最终终止于 35 s 无物理进展，未出现 detour/左转，第二圈未完成。本节是修复前问题分析；本审计没有修改 detector、semantic、worker、LYA 或 Unity 源码。
