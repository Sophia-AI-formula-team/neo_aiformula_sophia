# 视觉绕行目标所有权：修复前问题与最小方案

日期：2026-10-12。状态：**静态问题报告和接口提案；未实施控制候选，未修改核心、LLP、LYA、场景或车辆参数，未进行本报告对应的运行验证。**

用户选择是保留官方 OSM、修复自动路口关联并验证真实左转；未授权仿真路口坐标标定。本文不启用历史 calibrated-map 候选，不使用 CAD、Unity 路口/道路坐标、人工 world anchors、portal 或固定转弯半径生成控制路径。现有 ideal Unity odometry 必须继续如实标注，不能称物理传感器验证。

配套报告：[官方自动关联问题](OFFICIAL_AUTO_MATCH_PROBLEM_AND_PLAN.md)、[原 tracker 条件性反例](DETOUR_RIGHT_ANGLE_REPORT.md)、[真实 capture/render pose 不一致](CAPTURE_RENDER_POSE_PROBLEM.md)。这些问题相互独立：路口 ID 关联成功不证明内部道路几何可走；有局部几何也不代表当前核心会使用它。

## 1. 已确认的源代码问题

| 边界 | 当前实现及位置 | 对左支路控制的影响 |
| --- | --- | --- |
| 完整观测到 LLP | `RoadDetector/worker.py:175–185` 输出原始 lane-mask PNG，并将原数组交给 `Pipeline.process()`。`lane_publisher.py:55–77` 使用左右半图和历史线附近 ROI；`:405–425` 将投影限制为前向 0–10m、横向 ±4m，按前向 x 排序，最后把每侧拟合为一条直线。 | 完整 mask 仍可读取；最终 clouds/raw_clouds 已经过筛选。横向支路及真实弯曲边界可能被剔除或压直，不能把 clouds 缺线当作路口正证据。 |
| LEFT 的语义 | `upstream_adapter.py:84–109` 按可用云/数量选 center、left_only、right_only；原 `lane_0215.py:144–164` 从单侧线按宽度偏移出当前中心，`:166–204` 实际将三点 y 截到 ±2.5m。 | left_only 不表示选择左支。把左支线塞进左边界会触发原偏移，可能得到错误中心；超范围目标也可能被静默截断。 |
| 数值目标入口 | `upstream_adapter.py:187–221` 的 `selection` 接受 `(selected_clouds, indices, mode, reason)`，调用原选点节点，再将 ABC 送原目标计算/LYA；`withoutkalman_0312.py:43–69` 用 AB/BC 方向计算目标和角速度前馈。 | 可以复用原 ABC/LYA 数值接口；不存在语义化的 `choose_left_branch` 或视觉分支走廊接口。数值可接收不等于目标已被观测/验证，亦不证明车辆能跟踪。 |
| Unity → semantic 协议 | `AIFormulaSemanticPlanning.cs:118–125` 只发送选定三点、当前 corridor 元数据、灯色与距离；`semantic_worker.py:310–355` 按帧时效检查后接受恰好三个有限点。当前 corridor 来源是既有当前车道宽度/身份检查，不是支路几何。 | 缺少官方分支意图、匿名支路候选、边界像素来源、局部可见范围及观测不确定度。不得把新分支伪装成旧 corridor 来源。 |
| detour 所有权 | `map_assist.py:983–1003` 在 wait/perception_suspended 时保持等待，在 detour 时只调用 `_process_detour()`；`:736–766` 由 `detour_tracker.update(*odom)` 发布 ABC。`:1038–1039` 不允许普通视觉 triplet 在 detour 模式输出。 | 新鲜视觉 ABC 即使被接收，也不会接管 detour。仅替换 `selected_points` 不能修复红灯绕行。直接在 transport 覆写发布器/状态同样是行为修改，不能称已有接线开关。 |
| 官方图边几何 | `anchored_topology.py:77–166` 把外节点锚定，内部节点来自相似变换和逆边长加权位移插值；`traffic_route.py:112–121` 将节点坐标直接连接为 OpenTopoRoute。 | 这是拓扑节点位置估计和节点间直线，不是实测内部道路中心线、宽度、通行空间或转弯曲率。外节点匹配成功不能把这些 chords 提升为道路测量。 |
| 原逐段 tracker | `traffic_route.py:314–343` 到节点半径内才切下一段，ABC 在当前段内压缩，不能跨节点提前预瞄；后方目标被拒绝。 | 折线拐点前预瞄缺少后续方向。原反例 `(0,0)→(5,0)→(5,5)` 在外部给定姿态 `(5.1,0,0)` 时重复拒绝 `target_behind_vehicle`。这证明条件性失效，不证明真实车辆必然失败；更不能用人工改变 yaw 证明通过。 |

另有执行层契约：`AIFormulaVisionLaneFollower.cs:149–158` 在取 semantic 命令之前检查当前 lane 支持超时。未来真正独立的支路观测不能通过给原 ego-lane 的 unsupported 结果改标签来规避它；需要显式、可审计的活跃目标来源与对应有效性检查，保留超时停车。

`MapAssist._on_branch()` 只是将当前段排除出学习并暂停地图修正；它不是左转命令，也不能越过红灯 wait/detour 状态所有权。

### 条件性进入时机缺口：approach 阶段不会选择绕行

`map_assist.py:668–669` 明确要求 `traffic_mode == "wait"` 且当前票决为 red；approach 会直接返回，因而不会枚举附近可信路口或调用 `alternative_route()`。`:430–437` 在有效红色测距尚有 remaining 时保持 approach，达到距离目标才转 wait。注意 wait 表示零速门控状态，不证明车辆实际速度已经为零；`:668–731` 也没有独立的“实际已刹停”检查。

因此，**若**真实可信路口位于当前 approach 行进段，车辆可能先经过可进入位置，随后才在灯前目标距离处进入 wait，届时近路口/方向/前方目标条件可能已不成立。这是源代码支持的条件性风险，**不是本场景已经发生的结论**。不能用手工 CAD 或 Unity 路口坐标决定它；目前 capture 实跑是 green 首圈诊断，没有触发或验证该红灯时序。灯前车头距离与 odom 到路口站点距离的基准不同，不能把 8–9m 与 entry radius 直接相减来证明问题。

最小方案应将“准备意图”和“提交控制所有权”分开：允许在新鲜、非故障的确认红灯 approach 中，用真实已确认且仍在前方的路口观测准备官方绕行意图；只有匹配/方向无歧义、当前实测入口走廊覆盖安全制动范围、入口速度/姿态可接受、新 ABC 和新命令齐全且 OA/E-STOP/时效允许时，才原子提交视觉 detour。已经经过或在身后的入口应拒绝，不能全局跳配、掉头补救或前移虚拟入口。进入前准备失败保留原 approach/目标停车策略；接管后几何丢失则明确 hold。不得仅把条件改成 `mode in ("wait", "approach")` 就启用旧图 chord tracker，也不得靠取消红灯零速门控完成转向。现有 fault latch 和未知灯色拒绝规则保留。

## 2. 最小新增接口：官方意图 + 实测局部几何

以下均为提案字段/状态，不是现有已实现能力。默认保持关闭；先影子输出和记录，再在审核后的核心版本中接管。

1. **官方层只给意图。** 自动路口关联已消除歧义后，提供 `junction_id / incoming_edge / intended_outgoing_edge / direction / association_epoch / confidence`。不得把官方绘图比例或内部节点 chord 当作局部米制几何。未知或多候选时不授权左转。
2. **感知层给匿名实测走廊。** 从同帧完整 lane mask 在原 LLP 压缩之前旁路读取；保留真实像素组件，使用已声明的传感器模型投影。候选提供两条独立实测边界、共同可见范围、弧长有序中心采样、方向/截断/不确定度、原始像素索引或可复核支持。中心是两条测量边界导出的估计，不能标为直接检测到的中心线。仅缺线、单个横向碎片、斑马线、阴影或单侧固定宽度补线，不足以证明可左转。
3. **绑定同一次采集。** capsule 至少含 schema、sequence、epoch、capture clock/time、RGB/mask 身份、相机标定身份、曝光时姿态来源/有效性。图像 render pose 与 Rigidbody odom 若不同必须分别标明，不用 arrival pose 替代。odom 只负责把历史观测移到当前车体坐标；它不能创造道路边界。双时钟 freshness、跨 epoch、重复帧、重置、位姿缺失均拒绝。
4. **明确新目标所有者。** semantic 接收意图与观测走廊，验证该候选与当前入口连续且方向相符后，进入显式 `visual_detour` 所有权；保留 OA/E-STOP、速度/消息时效和交通进入门控。每次输出都应记录 `target_source / observation_sequence / valid_until / geometry_source / reason`。正常 vision、视觉 detour、旧图 tracker 不能同时发布有效 ABC；不允许未知时退回未实测的图 chord。
5. **复用原 LLP/LYA，不修改其算法。** 感知旁路可提供真实双边界及共同中心给现有 selection 入口，并选择观测范围内三个点。必须在原 LP 横向限幅之前检查目标处于其契约范围，记录输入与最终 ABC；不能以静默截断后的点冒充原测量。持续大角度或不足三点则 hold。原数值目标计算、LYA、车体动力学、相机、原 LLP 输出和上游源码保持不变。
6. **局部连续预瞄，但不制造曲线。** 在已观测走廊中按弧长取 ABC，允许跨真实连续小段，局部单调推进、有界窗口，拒绝跨交叉口跳到另一支路、目标在后方或几何突变。不借用历史手工 R=3/portal 路径。连续预瞄是算法改动；它不能把未观察的角落补成弯道，也不能单靠重采样保证动力学可行。
7. **未知即保持安全停止。** 有效观测范围须覆盖考虑当前速度、执行延迟和不确定度的停车距离；没有可靠制动包络时不能宣称足够。边界不成对、截断、关联歧义、过期、投影未获支持、前方不可见、出口未确认时明确 hold，禁止扩展 TTL、虚构 supported、反复刷新旧点、强制 normal/green 或取消 fault latch。出口必须由新视觉与真实 odom 关联到原参考及方向一致性确认，不只按预计出口节点直接宣布完成。

**不需要假称已知完整内部路径。** 可以研究按可见局部走廊逐步推进，但每一步都必须能在已验证范围内停车。尚未看到的内部段和出口始终标 unknown；如果前向相机无法观测足够边界，就应保留功能未完成，不能用官方拓扑坐标补齐。

上述所有权和安全握手不是纯接线已具备的功能。若必须修改 semantic 核心，先审阅本报告并定稿接口，再以 **0.3.0** 独立版本实现/记录。保留当前 0.2.0/final63 证据、官方 OSM 原字节、默认原行为和回滚点；不得只因改动放在 wrapper 中而省略版本与问题报告。

## 3. 地面平面来源：传感器假设与路口真值标定的区别

用户拒绝的是用仿真路口标定替代自动匹配。由现有要求判断，这**不等于禁止已经存在的相机 K/R/t 和明确声明的局部平地投影假设**；但也不构成对任意新增世界平面/场景真值读取的授权。默认只保留现有传感器合同，不新增手工世界 y、坡度、路口坐标或模型 mesh/raycast 控制输入。若需要超出已有合同的新标定，先明确提出来源和用途。

当前 `AIFormulaRoadDetector.cs:152–163` 从相机相对车辆的位置/旋转构造 K/R/t；`lane_publisher.project():413–415` 用 `scale=-t[2]/ray_z`，隐含**车辆投影坐标 z=0 为地面**。相机的局部安装高度是相对车辆原点的距离，不自动等于距实际路面的高度。`AIFormulaRgbCameraTopic.cs:50` 使相机随车辆姿态变化；仅有固定外参不会自动补偿车体相对路面的 pitch/roll、悬架变化或道路坡度。把每帧 body-z=0 变换到世界，得到的是随车体变化的假设平面，不证明它是同一个物理路面。

可接受的真实数据约束及限制：

- **固定传感器标定**：已测相机内参、畸变处理、相机到车体的刚性外参，以及有明确 road/body datum 的安装高度。只读取已有数值和来源，不由灯/路口真值反推。若高度基准未知，保留 scale/height uncertainty；不能给无根据的厘米级精度。
- **同刻姿态与时间**：使用当前已记录的曝光 render pose、实际 odom 和完整姿态作为不同来源，保留 clocks/epoch；若用于仿真相机解释，标为仿真姿态证据，不能包装成真实 IMU。姿态本身不提供地面法向或高度。
- **多帧真实像素重投影**：用同一静态地面边界/纹理在不同帧的重投影一致性检查局部平面，匹配由图像数据决定，不能由官方节点或 CAD 给对应。可报告观测残差、跨度、失败率与敏感度；重投影一致并不唯一证明高度/尺度，纯转动、低视差、重复虚线和误匹配存在退化。
- **显式平面模型与不确定度**：若只有已有局部平地假设，标 `plane_source=existing_local_flat_ground_assumption`，限制有效距离/姿态并保持未知误差。若未来由真实图像和可靠运动估计平面，另标 observation-derived，记录输入、可观测性与验证。不得把官方图拟合残差当作相机平面验证。
- **允许先只做相机/车体局部观测**：无获支持的世界平面时，仍可保留原始 mask、方向/开口候选及局部假设下的几何审计；不要发布高置信世界路口锚点。其能否支持安全的米制控制，要由误差和可见停车范围回答。

官方 OSM 提供拓扑和分支意图，不提供相机标定、地面高度、道路边界或理想像素。CAD/真实 Unity 道路几何仅能用于隔离的事后评测，不进入特征生成、候选选择、投影校正或控制目标。

## 4. 修改前后验收要求

先保存本报告、当前源码/接口基线和既有失败证据；随后才讨论新增代码。不得回标旧失败为成功。

- 离线有意义测试覆盖：弯但无支路、真实双边界支路、单侧/缺线/斑马线、近地平线/截断、退化平面、时钟/pose错配、旧帧/跨epoch、目标后方、过短预瞄、交叉路径误跳、双所有者、unknown停车、绿恢复及退出误匹配。人工位姿反例只标算法证据。
- 回归保证：原 LLP/LYA 源码和默认路径不变；正常红灯 8–9m 停车/绿继续、现有 fault/OA/TTL/focus/E-STOP 行为保持。相机、灯位、车辆、物理参数和另一对话的修改均受保护。
- 实际功能验收必须有**真实 MCP 内圈、外圈**自动学习/路口关联后左入、内部通过、退出并恢复视觉；每场记录新鲜 mask、候选/实际 ABC、控制所有权、odom/姿态、v/w、guard 原因及全真实时序视频。失败保存日志和可运行视频。仅离线测试、拟合通过、目标图像好看或三点可计算均不能称闭环通过。
- 原生 ROS/DDS 和真实车辆验收另列，Unity 成功不替代它们。新核心如果实施，发布/部署/交接文档应标 0.3.0 的实际状态、默认开关、依赖、来源边界和回滚方法。

## 源码定位

本报告静态读取的安装根目录：

`C:\Users\PC\OneDrive - Sophia Univ. Students\Main\SOPHIA\Y4T1\Research\Demo_LaneLineTrainer\Simulator\Assets`

- [完整 mask 与 lane pipeline 入口](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/RoadDetector/worker.py>)
- [LanePublisher 投影/裁剪](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/VisionLaneFollower/lane_publisher.py>)；[现有云选择接线](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/VisionLaneFollower/upstream_adapter.py>)
- [原选点节点](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/VisionLaneFollower/upstream/workspace/src/aiformula/perception/lane_points/lane_points/lane_0215.py>)；[原三点目标计算](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/VisionLaneFollower/upstream/workspace/src/aiformula/perception/kalman_filter/kalman_filter/withoutkalman_0312.py>)
- [semantic C# 协议](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs>)；[Python transport](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/SemanticPlanning/semantic_worker.py>)；[执行层支持守卫](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs>)
- [MapAssist 所有权](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/SemanticPlanning/source/semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py>)；[内部节点插值](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/SemanticPlanning/source/semantic_planner_core/semantic_planner_core/anchored_topology.py>)；[原路由与逐段 tracker](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/StreamingAssets/SemanticPlanning/source/semantic_planner_core/semantic_planner_core/traffic_route.py>)
- [相机标定构造](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs>)；[实际 RGB capture](<C:/Users/PC/OneDrive - Sophia Univ. Students/Main/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets/AIFormula/Runtime/Perception/AIFormulaRgbCameraTopic.cs>)

行号对应本次静态审查；并行新增 capture 诊断代码可能移动行号，应按函数名复核。本文没有执行旧反例脚本、重投影计算、模型、Unity 或测试。
