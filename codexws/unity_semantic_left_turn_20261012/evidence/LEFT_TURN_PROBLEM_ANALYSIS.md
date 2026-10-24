# 左转验证问题分析（修复前）

2026-10-12。代码基准 neo `434fdc43d00399808adf7e548d491a5dfdc5bffa`，semantic core `63eec2d`。本报告先于任何本轮算法或 Runtime 修改。静态风险与实测失败分开记载。

## 当前功能边界

当前 Unity/native shadow 接线是 MapAssist → 原 point processor → 原 LYA。MapAssist 没有无条件的“红灯左转”指令；已完成真实闭环学习、地图融合/匹配且靠近外圈节点时，红灯删除正常下一条边，选择含内部节点的替代路线。方向取决于图。另一套 costmap/primitive 链有 LEFT 偏好，但不是当前接线，不能用其单元测试代替车辆验证。

过去 final63 四场是首次运行的短区段，reference=None。它们验证红灯 8–9 m 停车、绿灯恢复及东北左弯跟踪，没有验证红灯触发路口左转绕行。LEFT 来源标签只是视觉车道推断来源。

## 待闭环验证的风险

1. `traffic_route.py` 的 DetourTracker 只在当前直线段上采 ABC，距端点 0.8 m 内才切到下一段。90 度拐点没有提前转向轨迹，越过节点时新目标可能落在车后并停车。现有测试直接设置每段正确航向，未验证 LYA/plant 如何转向。独立反例只证明算法输入边界；必须配合 Unity 闭环才能判定实际影响。
2. 视觉消费者 `AIFormulaVisionLaneFollower.cs` 在读取 semantic command 前仍要求最新视觉车道 supported；semantic_worker 和 C# consumer 无条件沿用视觉 corridor 期限。MapAssist 的 detour 原本由当前 odom 生成 ABC。转向过程中，视觉失去原车道可能截断有效的地图输出。修复若需要，应严格依据当前 detour 模式和实际地图 ABC 来源，保留 odom/命令年龄、手动/E-STOP/owner 保护；禁止全局移除视觉保护。
3. 8–9 m 的停车目标不自动满足映射节点 3 m 的 detour 入口。正常 `signal_target_reached` 不会默认锁存 fault；需用实测融合节点与停车位姿计算，不能错误归因为必然锁存。
4. 拓扑内部节点是推估坐标，稀疏边不是测量的可行驶车道。平滑折线本身不能证明道路/车体空间安全；不得用评测道路真值来偷偷生成控制路线。

## 执行方案

先仅增加独立 Editor 诊断 recorder，不改原 recorder、Runtime、semantic、相机、灯位、物理或 LYA。用真实 RGB 全链路检查内外圈首圈学习前提。真实失败视频和 trace 保存后，再判断问题属于试验接线、感知或算法。不能将注入地图、逐段重设 yaw、伪造灯色/ABC 或模拟轨迹标为完整功能通过。

Unity 与另一对话保持独占协调，本轮全部 Editor 操作使用 MCP。原 final63 证据和 Git PR 堆栈保持独立。

## 真实首圈基线与配置对照

- `baseline_inner_learning`：97.92993 m，42.3825 s，停车位姿约 (12.9934,38.6627)，`Lost predicted lanes`，地图未生成。
- `baseline_outer_learning`：106.0802 m，43.9891 s，同样在西北入口失去支持，地图未生成。
- 两场失败视频 `baseline_learning_failure.mp4`：1280×960、15fps、1298帧、86.5333s，完整解码无错误；主画面75%，下方为真实识别、LLP/LYA、semantic与轨迹。SHA256 `481a341d5fe040cdcade21b5e39af6682d89014c197c7e4ddcc4c4dd7e4bf26b`。
- 内圈 seq284 原始 LEFT 有7点，2.5–5.5 m，RIGHT 为0；左线与之前直线身份差0.4304 m/14.73°。最后 paired标定 seq273，宽度4.5044 m；到 seq284消费时先验已过1.5秒。当前真实lane mask并非完全没有车道，但不能仅从点数断言边界身份。
- A对照：将独立 recorder 的组件恢复选项与signal approach/focus/启动headroom解耦，关闭原 `recover_projected_components`。`native_inner_learning` 在15.65583 m失去支持，地图未生成。说明不能简单关闭全部恢复；seq284的7点本来也依赖当前组件恢复。

下一步 B 对照只拆开接线开关：保留现有 LP 当前组件恢复，单独关闭 CurrentCorridor 选择，调用已经存在的当前观测选点方法。默认路径与旧输出保持；试验模式仍有当前像素、geometry、观测跨度、命令新鲜度与E-STOP保护。这是可复测候选配置，不是已完成修复或左转通过。semantic、原LLP/LYA类及车辆物理仍不修改。

## B 实际首圈结果与新的融合问题

2026-10-12 JST06:17，`component_native_inner_learning` 使用真实 RGB、实际原 LYA/Unity plant 完成首圈：263.707336 m，100.903816 s 时建立 reference，reference_length_m=264.161226。终止时 phase=`odom_only`，leftTurnValidated=false；核心 source 仍保持 63eec2d，默认行为的288帧 before/after 回放逐字段、内部状态完全等价。

新阻断是地图融合而非首圈跟踪。实际 `topology_alignment`：rmse_m=3.198397389，scale=0.788109463，phase_fraction=0.573721476，direction=1，alternative_rmse=4.051410560，ambiguity_gap=0.853013171；官方稀疏弦长215.730473 m，学习闭合长度264.177919 m。`map_fusion.reason=rmse_exceeds_limit`，`topology_mapping.reason=fusion_unavailable`，没有 official_node_positions_odom，因此不能启动真实地图绕行。

默认 alignment_max_rmse_m=3.0。本轮不直接扩大门槛或把 odom_only 当作可绕行地图；先检查完整轨迹是否仍在真实道路、官方图与连续道路的表示差异，以及匹配算法的对应关系。外圈独立闭环正在进行，待对照后提出必要的最小改动。上述首圈完成仅满足参考轨迹学习，不构成地图融合或左转验收。

外圈 B 实测也完成学习：286.079956 m，109.911716 s 建立 reference=286.527265 m；地图融合误差3.412707965 m，同样被3.0 m门槛拒绝。两场共3162个中心采样对完整赛道 perimeter−五岛区审计均在道路内，内圈最小中心净距1.600688 m、外圈1.331477 m。条件轮廓在水平姿态/名义关节假设下也均在内，最小1.173587/0.903841 m；不能代替完整实际车体连续扫掠证明。

独立视频 `component_native_learning_and_fusion_failure.mp4` 包含A关闭全部恢复失败、B内圈和B外圈学习。1280×960、15fps、3381帧、225.4s，完整解码零错误，SHA256 `89c8c149b774ef4a7e69cf88ab67d1070b71d6fbf20d4277dee066947e853e6c`。编码器只重放真实记录，不推断左转通过。

先增加只读接线诊断：Runtime 可选 DiagnosticRouteOutput 默认null；stdio worker 可选 sidecar 导出实际 build_fused_route 输入（包括闭合样本和excluded标记）及原函数输出。原函数以原实参调用一次，返回同一对象，不改变核心、门槛、路线、LLP/LYA或物理。导出错误只单独记录，不能成为控制输入。独立 recorder v06 让同一真实worker/epoch完成第一圈后在第二次接近东灯时切真实panel红色，记录red wait/entry/exit及当前节点距离。若融合/绕行未完成则判失败，并单独保存视频。此诊断接线先于任何融合算法修复；后续算法方案必须依据exact样本再报告。
