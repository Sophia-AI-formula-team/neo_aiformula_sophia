# 最终 panel-only 内外圈真实验证

2026-11-16。最终同一套六文件接线、原 semantic 0.4.0、官方 OSM、原 LYA、固定30fps/vsync0。以下是 Unity MCP 实际试验日志，不是离线重放。root负责运行，独立审计只在结束后做每场一次主 metrics 解析，没有重跑模型、改变 source 或放宽任何融合/控制门槛。

| 项目 | 外圈 outer_joint_03 | 内圈 inner_joint_07 |
| --- | --- | --- |
| 实际物理圈数 | 2 | 2 |
| 总行程 / 记录秒数 | 563.1467 m / 230.7783 s | 519.832642 m / 214.9798 s |
| 初次 odom reference | 281.356355 m | 259.519390 m |
| 真实红灯停车 | 通过 | 通过 |
| 停车位置看到真实转绿并恢复 | 通过 | 通过 |
| 官方 topo 融合 | 失败：ambiguous_correspondence | 失败：ambiguous_correspondence |
| 红灯驶入内部 / 左转绕行 | 未发生 | 未发生 |
| 源包干净结束 | exit0，clean=true | exit0，clean=true |
| 整体试验 accepted | false，红灯绕行失败永久保留 | false，红灯绕行失败永久保留 |

“两圈”不是绕行通过。每场首圈由真实 LYA 驶完，odom路径闭合；第二圈开头实体灯切红，真实 RGB→semantic 原投票触发停车。原融合拒绝对位后等12秒，harness保留redDetourFailed=true，再切实体绿，车辆从停止位识别绿灯继续跑完第二圈。两场都没有 detour node/path 或官方节点的可信 odom 对应。

## 内圈详细证据

`inner07_metrics.json` 对应最终完成的 inner_joint_07：首个 package reference sim99.590s，first physical lap event t100.087s；原包于 t100.728s 确认 red，进入 `wait/red_wait_map_fusion`。967个stop前detector receipts全部policy=`calibrated_rgb_panel_only`，source为 RGB red60、RGB green72、none/unknown835；没有model source。60个red的捕获时刻全部在实体红阶段内。

实体于 t113.390s 切绿。seq504/505/506在 t113.707/113.947/114.153s 连续当前 RGB green，车辆 actual_v分别约6.8e-8/1.7e-7/7.9e-8；seq506已为native `normal/green_resume`，seq507才开始实际前进。原native瞬时vote的trace快照可能显示unknown（窗口已只剩2样本），但状态已由先前合法多数转为green_resume；不能把异步快照误读成用unknown放行。seq508记录green=3/3。内部worker本场有6帧`light_only_unsupported_lane`，但新鲜支持点随后恢复，原TTL/原LYA完成两圈；这并不构成未部署四点短侧线候选的验收。

稳定停车Body坐标(world x=67.68032,z=-7.302632)，灯root=(75.31588,16.3327065)。水平Body→灯root24.838m、车头方向投影23.121m。外圈相同口径分别21.658m、21.396m。**这些只用于评价，不输入控制；不是保险杠/相机/停止线距离。新包仍确认红灯立即停车，未接旧8–9m接近策略，bbox距离实验未混入。**

内圈最终68个matcher候选，best RMSE=.0583205m、alternative=.2724613m、gap=.2341708m<原.3m margin，alternative_node_spread=1.3058268m>原1m分歧门槛，因此原包拒绝官方对位。`official_node_positions_odom`为空、graph_frame=`official_unaligned`，没有正确内部topo的证据。不能以best RMSE小替代歧义验收。

外圈完整交通与融合证据见 `OUTER03_FINAL_BEHAVIOR.md`；两场运行均已干净退出。各run的 `integration_hashes.json` 固定实际安装来源；本报告不把部署一致性或Python测试当作实车验收。

## 对功能的直接判断

这版源包能从两条实际行驶外圈建立并闭合里程计路径，能在当前仿真已知灯牌接线下红停绿行。但官方拓扑对位仍有歧义，内部节点仍仅是未对齐官方图，当前不能证明“正确建立semantic topo map”，也不能完成“建图后红灯拐进内部”。不通过手工定位、删候选、扩大融合阈值或未验证迭代budget掩盖失败。

后续算法修复应从记录的真实候选歧义和独立地面几何出发；此前15→75/100次完整诊断仍有其他合法候选，单加迭代次数不是已证明的完整修复。本次只归档问题与实际视频证据，不部署该算法候选。
