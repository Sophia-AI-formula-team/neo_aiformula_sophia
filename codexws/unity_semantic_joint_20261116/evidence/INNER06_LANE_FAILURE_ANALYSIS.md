# inner06 第二圈丢失有效 ABC：修复前只读分析

日期：2026-11-16。只读真实 `runs/inner_joint_06` 和对照 `inner_joint_05` 的记录、第一视角 RGB/overlay、现有 LLP 适配源码及 MapAssist。没有重跑模型、修改算法/接线/门限、控制 Unity 或执行新的 driving test。

## 判定

**停车的直接原因是当前帧可用投影只剩 4 个单侧点，达不到现有适配器 5 点 / 2 m 支撑要求，因而不再产生可信 ABC；原 semantic 的视觉时效门控随后正确发零。** 这不是新的红灯锁停，也不是 native reference correction 把车辆转向错误方向。

更细的归因应表述为“视觉/投影的有效支撑变短 + 接线选点策略不能使用该短单侧观测”，不能简单写成模型完全没有检测到车道。seq667 的原始 RGB 仍有左弯边界和右侧虚线；overlay 也保留相应分割。左侧 cloud 仍有 4 个本帧有效点，右侧有 1038 个 component pixels，但投影后不满足输出条件。当前日志没有逐像素投影拒绝原因，无法在不重建 mask 的情况下进一步断言是 ROI、地面范围、单连通片长度中的哪一个精确过滤条件造成右侧空 cloud。

## 原场次结果与时间顺序

`inner06/result.json`：完成 1 圈，总行程 303.726624 m，185.03648 s；结束原因是 `No physical progress for 35 simulation seconds after motion control started`。第二圈计划红灯先因原 0.4.0 的 `ambiguous_correspondence` 等待 12 s；实体灯切绿后原状态已恢复 `normal / green_resume`，随后在东北弯道失去有效 ABC。最终姿态约 Unity `(x=64.34938,z=33.37657,yaw=326.19794°)`。没有 detour 或红灯内转成功。

| detection seq | t / 位置 | 真实感知及门控 |
| --- | --- | --- |
| 655–657 | 146.34–146.75 s，z26.00–27.07 | center 有效，left 8–9 点/right 3–4 点；采用既有短双侧 center 选点。 |
| 658–663 | 146.97–148.10 s，z27.62–30.50 | right 投影不足后转成 left_only，仍有 6–7 点；原 LYA 输出随左弯增长。 |
| 664 | 148.336 s，(65.668,31.085), yaw334.828° | counts `[5,4,2]`，left_only，仍可选真实三点。 |
| 665–666 | 148.506–148.712 s | counts `[6,4,3]` / `[6,5,4]`，短双侧 center 有效。seq666 最后有效 ABC 为 `(2,.484619),(2.5,.597904),(3.5,.824475)`。 |
| 667 | 148.932 s，(64.924,32.480), yaw328.854° | left 4 点；right `projected_output_insufficient`、component_pixels1038。counts `[4,0,0]`，`supported=false/default_straight/no_observed_triplet`。 |
| 668 | 149.136 s | 仍 `[4,0,0]`，无新可信 ABC；原上个命令仅在自身 TTL 内有效。 |
| 669 | 149.344 s | `visual_points_stale`、native v/w=0；车辆正在制动。 |
| 670 以后 | 149.544 s 至终止 | left 持续 4 点，right 后续 `too_few_component_pixels`；车辆停稳后视角不再变化，没有新可信三点可恢复。 |

seq667 的 4 个真实 left cloud 点（base_link）为：

```text
(3.0, 2.9226463), (3.5, 3.1061120), (4.0, 3.2895780), (4.5, 3.4730439)
```

它们只覆盖 1.5 m。`default_straight` 中的 `[3,0],[4,0],[5,0]` 是原候选模块诊断 fallback，**supported=false，不能当成本次车实际驶向直线的控制指令**；Joint worker 没有接纳该假定直线。seq827 的停机 evidence 仍为 4 个 left 点，说明这次是持续观测支撑不足，不是单帧超时后已有有效帧却无法恢复的接线 bug。

## 为什么不是 semantic reference 修正

第二圈行程 280–305 m、失效与制动附近的 50 个 inner06 detection 记录，`correction_deg` 全为 **0**。原 MapAssist `map_assist.py:889–905` 在 correction=0 时原样发布传入 ABC 的 x/y；此场没有 topo detour，官方 fusion 也未被接受。末端速度门控是 `waiting_fresh_visual_triplet`，traffic mode 为 `normal`。因此没有证据支持改 semantic、关掉 reference correction 或改变红灯规则来修复这次停车。

trace 的新 detection candidate 与同一行 semantic 输出可能相差一个消息周期；必须按 `semantic.last_vision_sequence` / source sequence 对齐。比如 seq667 行里的 semantic 仍指向 seq666，不能直接把这一行两套 ABC 的差异说成语义修正。

这一段 detector 捕获年龄最大约 0.2791 s，seq667–670 约 0.129–0.137 s，均未达到原 0.5 s capture TTL。最终失效来自 supported=false 没有新 ABC，随后旧 ABC 自然超时；不能用“GPU 推理太慢”概括此次停止，也不能放宽 TTL 继续用旧弯道命令。

## inner05 同路段对照

inner05 成功穿过同一东北弯道，第二圈相同附近 43 个 detection 的 `correction_deg` 也全为 0。seq575–579 的 left cloud 为 6、6、5、5、5 点，right 也一度投影不足/消失，但单侧支撑仍满足原 2 m 要求，所以 `observed_side_recovery` 能持续输出真实 ABC。

例如 inner05 seq577 在 `(64.40135,33.05260,yaw324.6751°)` 仍有 5 点；inner06 seq668 在 `(64.651,32.918,yaw326.983°)` 只剩 4 点。两场位姿和朝向并不完全相同，约数度的视角差、采样时刻和连通片选择可以改变投影可见范围。该对照证明现有链在邻近位姿能通过，不证明任一姿态必须有相同输出，也不证明 v2 滤灯直接破坏了车道。v2 不改 mask/LLP 选点内容；时序和轨迹差异仍应如实保留。

已目视核对的原文件：

- `runs/inner_joint_06/rgb_00667.jpg`
- `runs/inner_joint_06/overlay_00666.png`
- `runs/inner_joint_06/overlay_00667.png`
- `runs/inner_joint_05/overlay_00577.png`

图像中车辆仍面向赛道左弯，左边界可见，未出现支持“已转进草地/反向/翻覆”的证据。仅凭这几张图不作毫米级车道中心误差验收。

## 代码中的确定拦截位置

源码根为现有 Simulator 的 `Assets/StreamingAssets/VisionLaneFollower/`，不是 semantic 包：

- `upstream_adapter.py:22–38`：`observed_triplet` 仅为 n≥7、n=6、n=5 返回 `(2,4,6)/(1,3,5)/(0,2,4)`，并要求 3 个选中点覆盖至少 2 m。n=4 必然 None。
- `upstream_adapter.py:41–79`：现有 3/4 点短 center 只允许两侧同时支持，并检查共覆盖、中心平均、宽度稳定和边界方向一致。它不会帮助本场 right=[] 的单侧情况。
- `upstream_adapter.py:82–109`：找不到可用 center 或任一侧 triplet 后返回 unsupported default_straight。
- `lane_publisher.py:391–400`：既有 component recovery 已启用，且在无三点支撑时尝试其他当前片段；但 recovery 自身仍要求旧 5/6/7+、2 m 契约。没有证据表明忘记开启 recovery。
- `lane_publisher.py:405–425`：真实当前像素经相机投地面，限定前向 0–10 m、横向 ±4 m；原像素前向跨度不足 1 m 或最终不足 3 个 0.5 m 采样点时返回空。仅有 component pixel 数量不能保证可投影的前向支撑。

## 最小接线候选（尚未实现，也未证明能跑过弯道）
若要保留原模型、LYA 和 semantic，最窄候选位于**现有 Unity adapter 的观测选点策略**：新增独立默认关闭、Joint 实验显式启用的“当前 4 点单侧短支撑”选项，且只在现有正常/center/5+侧线策略均无法提供 ABC 时尝试。仅选本帧已有的 4 个有效投影点中的 3 个，例如 `(0,1,3)`，覆盖现有完整 1.5 m，不生成第 5 点，不外推、不重复历史点。再送原生单侧 lane-point 分支及原 LYA；保留原 lane width/补偿、frame/epoch/TTL/finite 检查以及所有标准模式的默认行为。

这属于明确的观测准入策略扩展，**不是一个无行为变化的漏字段修复**。原来侧线 2 m 的支撑长度会在该特定 opt-in 分支缩至 1.5 m，必须先解释为什么 4 个真实点足够、给当前 mask 来源与几何质量界限，再拿当前失败帧云和更差的不足/乱序/nonfinite/错误侧输入做窄验证；不能悄悄把全局 `observed_triplet` 的门限降低。没有实际新运行前，不能称其已解决第二圈。

另一个方向是让投影 recovery 从本帧可见片段恢复原有 ≥2 m 支撑，这能保留原准入契约，但需要原始 mask 和详细投影诊断来证明没有跨错误边界/外推；本次仅有 overlay 与 clouds，不能保证它比上述窄策略更小、更正确。当前不建议调整摄像机/ROI/投影范围、开另一套 corridor 模式、改单侧固定偏移、降低车速或修改 LYA 来掩盖问题。

执行状态：问题报告完成，所有文件只读分析；未修改 frozen v2、native LYA、模型、semantic、场景或控制参数。下一步由主管选择是否审核该窄候选或保留失败结果，不能把本分析当 live 验收。

## 主管批准的 staging 实施边界

报告先保存后，主管批准只在 staging 和归档实现/回放上述窄候选，live 待独立审查。新增 `allow_short_observed_side=False` 参数，C# 仅增加非持久 `AllowShortObservedSide`；Joint 保存原值、显式启用并退出恢复，configuration 记录。默认关闭时旧选择逐字保留。

新 fallback 放在所有旧 normal/center/≥5 点 side 尝试之后，仅接受当前恰 4 个 shape=(4,3) 有限点、前向 x>0 严格递增、完整跨度≥1.5 m；left 全部 y>0/right 全部 y<0，保持原投影横向 |y|≤4 m；三段相邻 heading 的最大差≤15°，避免折返/不一致碎片。选 `(0,1,3)` 并保持原生侧线分支/宽度补偿和原 LYA；不生成或缓存点，不改变 capture/command TTL。完整原 mask 识别/投影/fragment 规则和固定源码保持不变。

独立审查指出原生侧线 A/B/C 固定偏移及转向差分会受非等距选点影响，必须记录真实 seq666→667 的 target、omega_t 和 v/w 连续变化，不能仅验证 finite。为排除几乎重复的局部 x 导致病态短段，新分支还要求输入四点保持原 projector 的 0.5 m 采样间距（仅浮点 1e-6 容差）；不插值或重新采样来通过它。真实 seq667 满足这一条件。

验证只做冻结 upstream source provenance 校验、真实 seq667 cloud 经原 callback→LYA，以及必要边界/默认关闭行为；不运行模型或 Unity，不声称实际车辆已通过弯道。实现和验证结果在独立 receipt 中记录。

## 回放结论：不放行当前 `(0,1,3)` 候选

`audit/short_observed_side_verification.json` 记录 0.913 s 的真实 clouds→冻结原生 lane_points→withoutkalman→LYA 回放。14 项边界检查完成，827 个已保存 cloud 的默认关闭选择与原实现完全相同；这些只能证明边界/默认行为，不能替代实际控制品质。

| 原生输出 | 最后有效 seq666 | seq667 短单侧候选 | 差值 |
| --- | ---: | ---: | ---: |
| v | 2.243535 m/s | 2.247369 m/s | +0.003834 |
| w | 0.831470 rad/s | 0.999041 rad/s | +0.167571 |
| theta1 | 0.222809091 rad | 0.118751023 rad | -0.104058068 |
| theta2 | 0.222809148 rad | 0.220433623 rad | -0.002375525 |
| omega_t | 0.000000567 rad/s | **1.016826003 rad/s** | +1.016825436 |

seq667 的输入 left cloud 为原 projector 的近乎严格直线，原 4 点在 x 上均匀相隔 0.5 m。新选 `(0,1,3)` 的 x 间距变为 0.5 m / 1.0 m。原 lane_points 分别对 A/B/C 减 `2.6/1.5, 2.6/1.4, 2.6/1.3`，得到 `(3,1.18931295),(3.5,1.24896915),(4.5,1.47304392)`；原 withoutkalman 再用 `(theta2-theta1)/0.1`，产生上述 1.016826 rad/s。这个量有直接代码与数值来源，不能当成当前真实路面的曲率观测。

最终 w 未触及原 2 rad/s 饱和，且跨帧 Δw 仅约 0.168，并不消除中间人工曲率的问题。主管明确要求如果不等距选择引入该问题就不能为了完成两圈放行，因此**当前短侧线候选判为不适合部署**。不插入等距假点、不改变原补偿/LYA、不调 gains、不接受 default_straight 来续跑。

独立审查还发现新 fallback 对左右各 4 点但 paired-center 质量检查失败的情况会优先取左，可能绕过短 center 的一致性要求；虽然本场确为 `[4,0,0]`，该边界也必须在任何后续重新设计中明确拒绝。本候选已因更直接的控制几何问题不予放行，不继续叠加检查来宣称它可用。

本实现尚未进入 live 或归档部署 manifest。候选代码与回放原件保留为独立失败证据，正式 staging/归档保持原 v2。继续真实重复实验可检验原观测策略的稳定性；若未来仍要修单侧短线，需先重新定义与原生补偿一致的几何接口，而不是隐式修改采样含义。
