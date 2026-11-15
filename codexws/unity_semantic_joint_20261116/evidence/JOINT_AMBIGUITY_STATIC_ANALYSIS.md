# inner05 joint matcher 歧义：只读源码核对

日期：2026-11-16。范围：只读当前 0.4.0 源码、既有测试定义和已完成的 inner05 结果；未运行 fusion 重放、模型或测试，未改源码、门限、官方图、staging 或 live。父任务另行负责 v2 感知接线；该工作不属于本分析的 matcher 修复。

**后续更新：审计 agent 已完成唯一一次原输入重放，见末节。原来选中的两个 seed 候选追加迭代后，最大几何差从 1.4846 m 缩至 0.002623 m，出现明确的未充分迭代证据。以下静态结论保留其当时边界；现在不能再把这对候选的差异解释为已证明的物理不可观测性。完整候选集是否仍歧义尚未验证。**

**再更新：随后获批的完整 75/100 次预算诊断仍返回 ambiguous，发现另一组相差约 38 m 的近成本候选。仅把 15 改成更多迭代不能宣称修复；见最后一节。**

## 结论

**现有证据不能证明这是重复 alias 误判，也不足以提出可证明正确的最小算法 bugfix。** 最终同方向候选已经按实际几何差异过滤；完全相同的几何副本不会触发本次歧义。日志中两个候选只在首节点的相位接近，不能推出全部外圈锚点或内部道路位置等价。

当前结果符合源码的保守拒绝条件：找到另一个通过硬几何门槛、与最佳候选的组合评分差不到 0.3、但至少一个比较点相差超过 1 m 的候选。该事实支持“现有输入和评分尚未证明唯一可靠对位”，**不等于证明物理赛道在任何观测方式下都不可识别**。是否还有固定迭代预算造成的数值残差，需要逐候选诊断才能进一步区分。

特别纠正字段解释：`alternative_node_spread_m=1.4846015816` 是所有外圈投影点和全部内部 comparison points 的**最大值**。现有结果没有保存取得最大值的点类型或 ID，所以不能把这个数单独称为“内部节点误差/内部点 spread”。

## 基线与实测事实

源码位于本仓库 `workspace/src/aiformula/perception/semantic_planning_system/`，导入提交 `6b71267e24e81459220c7e6d30d84c3f7a00e591`。

| 文件 | SHA256 |
| --- | --- |
| `semantic_planner_core/semantic_planner_core/route_fusion.py` | `f79be602021295dd59bd76a22d8bc475303f0da30212d1e4f363558912b1a633` |
| `semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py` | `9b8590588e707173d6e623f144c41e6c1dd97531f92ce4282415eb4dc7eedb5b` |
| `semantic_local_planner_ros/config/map_assist.yaml` | `9505bcec3c51f8ec8639b6694ecaacd7cb59ac3b3f3f574a5952c8e25aa53107` |
| `semantic_planner_core/tests/test_joint_alignment.py` | `983e64102d3b70a7b51bd39d3d072c4ef7502e31768576dd4ec97021ac5092b5` |

既有结果文件：`runs/inner_joint_05/result.json`，SHA256 `1f6e96835bc757e057dae1ee9f54a322c091af64ddd52b4d863b8145cefd342f`。下列值直接读取其 `terminalStatus.status.topology_alignment`，不是新融合重放。

| 指标 | 实测值 |
| --- | ---: |
| 实际圈数 / 总行程 | 2 / 519.3135 m |
| `accepted` / 红灯绕行 | false / 未进入 detour |
| 红灯失败原因 | 12 s `red_wait_map_fusion` |
| 后续第二圈 | 实体灯切绿后恢复并完成；不是红灯转弯成功 |
| odom reference | ready，259.4229306066 m |
| fusion reason | `ambiguous_correspondence` |
| best / alternative node RMSE | 0.1592974061 / 0.2227218016 m |
| best / alternative direction | 1 / 1 |
| best / alternative phase fraction | 0.5681089020 / 0.5690348731 |
| 首节点圆周相位距离 | 0.2402225950 m |
| 组合评分差 / 允许的歧义 margin | 0.06627544246 / 0.3 |
| 最大比较点差 / position tolerance | 1.4846015816 / 1.0 m |
| 外圈锚点数 / 内部 comparison points 数 | 5 / 2 |
| 最终 candidate 数 | 69 |
| `same_transform_alternatives_checked` / `search_exhaustive` | true / false |

`official_node_positions_odom` 与 `official_node_progress_m` 为空，故没有可声称已正确接受的 topo 路口对位。第一圈里程闭合和第二圈车辆运行均不能替代该项验收。

## 源码证据链

1. `map_assist.py:721–733` 传入完整官方外圈，同时把**不属于外圈且有邻接边**的官方节点作为 `comparison_points`。本场为 2 个内部节点；它们是官方先验中的节点，不是第一圈视觉实际测得的内部路口观测。
2. `route_fusion.py:148–258` 联合选择各外圈节点在可信 odom 线段上的候选投影，并约束循环顺序、节点间进度及 chord/angle。每节点只保留最多 6 个分离候选，搜索并非穷举。
3. `route_fusion.py:426–477` 从各 seed 交替做 joint correspondence 与 similarity fitting，最多 15 次；最终再投影一次。当前返回值没有 `converged`、迭代数或最终 transform step。
4. `route_fusion.py:496–518` 的评分为 Huber node RMS + `0.2 * feature_window_m * angle_RMS` + `0.1 * Huber chord-length RMS`。所以 `ambiguity_gap_m` 是组合评分差，不能直接说成 RMSE 差，也不是已标定概率。
5. `route_fusion.py:526–544,696–701` 的 `_same_basin` 只用于粗种子去重。在当前 `node_search_m=4` 下，它允许相位/outer RMS 约 8 m、旋转 12 度、比例 15% 的范围；这明显宽于最终 1 m 几何一致性契约。
6. `route_fusion.py:724–749` 还在同一 transform 下逐个排除最佳对应附近 1 m 的进度位置，搜索不同有序对应。这是明确实现的歧义探测，不是重复 append 本身导致报错。
7. `route_fusion.py:754–759` 最终 alternative 必须不是最佳对象本身、满足 RMSE/最大节点距离/角度硬门槛，并且**方向不同，或 `_candidate_disagreement > 1 m`**。本场两者方向相同，故实际几何差异条件一定成立。
8. `_candidate_disagreement` 在 `route_fusion.py:556–562` 比较每个外圈 ID 的投影位置，再比较两个 transform 对每个内部 `comparison_point` 的映射，取所有距离的最大值。对于同方向、全体比较几何均相同的候选，无论 Python 对象是否不同、来自多少 seed，差值都为零，不能成为本场拒绝的 alternative。
9. `_to_alignment` 在 `route_fusion.py:576–588` 仅用第一个官方节点生成 phase；通过硬门槛后，另一个上述候选的评分差 ≤ 0.3 才返回 `ambiguous_correspondence`。本场 0.06627544246 满足它。`build_fused_route:915–916` 在 shape deformation 前就 fallback，所以后续路线平滑不是本次歧义的直接来源。

## 为什么近 phase 不证明 alias

同方向只描述绕行顺序；phase 只约束外圈第 0 节点的位置。其余 4 个外圈锚点可以沿线发生不同位移，transform 的尺度/旋转也可能不同，导致内部节点偏移。即使所有外圈投影非常接近，内部映射仍可能不同——现有 `test_inner_node_disagreement_is_not_hidden_by_equal_outer_projections` 就明确保护这种情况。

如果选中的 alternative 来自同-transform 探测，两个内部点的 transform 映射完全相同，最大差只能来自外圈对应点；如果来自不同 seed 的 refinement，外圈和内部两种差都可能存在。当前 `topology_alignment` 只有汇总结果，没有 alternative 来源或 transform，无法从日志确定属于哪一种。

本包 `test_official_nodes_and_synthetic_rounded_track_remain_ambiguous` 明确要求官方图 + 合成圆角外圈拒绝；`test_symmetric_loop_is_not_accepted_as_unique` 同样防止把低残差当唯一解。`test_different_start_and_reverse_still_search_expected_direction` 允许 accepted 或 ambiguous，但要求方向与跨起点行为一致。这些是读取到的测试契约，**本轮没有运行它们，也不将测试定义冒充当前测试通过证据**。

## 尚未证实的问题与不可直接采取的修复

15 次上限而不报告收敛状态，是值得诊断的数值不确定性：若不同 seed 尚在走向同一解，最终留存的候选可能包含求解预算残差。但这不自动证明该候选“不合法”：当前契约本就比较满足门槛的近评分几何，并未要求 alternative 必须是另一个严格局部极小值。即使进一步优化会合并两个点估计，也不能仅靠这一事实排除低成本的空间不确定区域。

以下修改现在均缺少正确性证明，不应为了使演示通过而做：

- 在最终候选上复用宽松 `_same_basin`，或仅按 direction/phase 聚类；这会隐藏超过当前 1 m 契约的真实几何差。
- 忽略内部 `comparison_points`、提高 position tolerance、降低 ambiguity margin、只选最低 RMSE，或取消同-transform alternative 检查。
- 用 Unity 路口/车辆真值给官方图额外对位，或手工修改官方图。
- 宣称无条件增加迭代数就修复了 alias。它可能改变数值结果，但还需说明候选含义、实时开销与安全判定，不是已证明的最小修复。

当前没有发现方向符号、phase 环绕、重复闭合节点或 map/odom 单位混用可以从代码直接证明为本场根因的错误。不能把“尚未发现”扩大成对整个 matcher 的正确性证明。

## 下一步最小诊断建议（本报告未执行）
由单独负责重放的 agent 在不修改部署源码、不调整参数的前提下，从原始闭环输入捕获 best 和选中 alternative 的完整诊断：

1. 来源（refined seed / constrained same-transform）、direction、scale、rotation、translation、各 outer ID 的 progress/projected position/residual，以及每一项评分。
2. 每个外圈和内部点的分项距离，明确 1.4846 m 最大值的点 ID/类型；先确认是否实际几何相等，再谈 alias。
3. 每个 seed 的 15 次迭代收敛轨迹、最终 step、终止原因；区分尚未收敛、不同局部匹配与沿连续曲线仍有近成本几何候选。
4. 原始 learned samples、excluded/trust 和官方图/配置/source hashes，确保重放使用真正的第一圈 closure 输入；不能用第二圈累计轨迹替换它。

若证明两个候选在全部 relevant 几何上一致，而当前判定仍显示 >1 m，才有具体可定位的比较/单位 bug；修复应保持现有 1 m 契约并加入该真实失败输入的窄回归。若两组几何真实不同且都低成本，则是当前观测/模型的可辨识性不足，单纯 deduplicate 不是修复。若仅发现数值未收敛，则先独立报告求解预算问题与代价，再评估有界收敛改进，不能混成几何证据已充分。

验证状态：静态核对与已存结果读取完成；未做新 fusion/单测/Unity 验证；semantic 源码保持原样。

## 后续独立审计证据：当前两候选明显向同一解收敛

主管随后授权单次、单线程、60 s 上限的候选诊断。`joint_topo_audit` 执行了 `audit/inspect_recorded_fusion.py`，产生 `audit/inner05_fusion_candidates.json`，总耗时 2.6716 s；本报告作者只读该 JSON 做点距离计算，没有再次执行 matcher。该脚本根据 first-closed tick sequence 1981 / sim_time 99.2799977809 的 `learned_samples` 加上同 tick 原始 current odom，恢复 959 个 closure candidate samples，保留原 OSM/门限。7 个原 runtime 摘要与原算法重放误差全部为 0；`route_fusion.py` hash 与上表相同。

捕获结果确认 best 与 alternative **均来自 refined seed**，并非 same-transform constrained probe。原来最大差 1.4846015816 m 来自外圈 `10000000002`，外圈 `10000000001` 为 1.0046794684 m；内部节点 `10000000005/6` 分别为 0.3659424183 / 0.8769003817 m。至此可排除“本场最大 spread 是内部点”这一表述。

审计进程从实际 best/alternative 的原 15 次结果分别开始，对该函数的内存副本增加迭代预算；它不重写源码，也不改原 1e-5 收敛阈值、评分、投影门限或官方图。两组追加实验各自从同一原结果开始，并非先 +15 再 +60。

| 追加迭代预算 | 两候选最大几何差（5 外圈 + 2 内部） | best / alternative RMSE | best / alternative 末 step | 两候选追加计算耗时 |
| --- | ---: | --- | --- | ---: |
| 0（原结果） | 1.4846015816 m | 0.1592974061 / 0.2227218016 m | 原结果未记录 | — |
| 15（共约 30） | 0.2818099131 m | 0.0155611658 / 0.0463454074 m | 0.0019477739 / 0.0055424419 | 0.05474 s |
| 60（共约 75） | 0.0026231885 m | 0.0011158140 / 0.0011877437 m | 0.0000169212 / 0.0000504166 | 0.20098 s |

这证明原来被选中的一对“相差超过 1 m”候选未充分优化；追加 15 次时，它们的相互差已低于原 1 m 门限，追加 60 次时只剩约 2.6 mm。二者末 step 仍未小于原 1e-5，故不能写“已达到原数值收敛标准”，但已具有明确的同解趋近证据。该结果比静态“可能未收敛”的推测更强，不能继续无条件归因为真实几何可观测性不足。

局限同样明确：本次仅续算实际 best/alternative，**没有重算所有 69 个候选，也没有在新的最佳候选上重新进行 same-transform 替代对应探测**。因此不能推断全局 ambiguity 已消失、原 fusion 会 accepted，更不能推断 topo 物理路口已对齐或红灯已可绕行。即使这两个 seed 合流，其他 seed、反向候选或约束探测仍可能触发原规则。

当前有证据支持的下一步是独立评估**有界 refinement 收敛改进**，在完整候选选择链中保留原歧义判定、原几何门槛和全部 alternatives，而非简单删除当前 alternative。先报告该 15 次截断对原候选的实际影响，再测完整重放的结果与计算预算；只有全链路证据才能决定是否存在足够窄且有效的修复。主管后续运行期间本 agent 不再执行 matcher。

## 完整候选链的后续预算诊断：另一远距离对应仍在

主管另行分配的单次≤60 s 窗口中，审计 agent 运行 `inspect_full_fusion_budget.py`，只在审计内存副本把每个 seed 的预算设为 75 或 100，保持原 1e-5 收敛阈值、官方图、投影/评分/eligibility 和 same-transform alternative 机制。两次各耗时 7.6137 / 9.8673 s，源码未写入。证据 `inner05_full_fusion_budget_75.json` 与 `_100.json`；本报告作者只读这两份结果，没有重复计算。

| 总预算 | best RMSE | 新 alternative RMSE | best / alternative phase | 组合评分差 | 最大几何差 | 完整结果 |
| --- | ---: | ---: | --- | ---: | ---: | --- |
| 75 | 0.0011158140 m | 0.2485593130 m | 0.567496745 / 0.441760068 | 0.258043889 | 37.930115 m | ambiguous |
| 100 | 0.0011090802 m | 0.2484646524 m | 0.567497234 / 0.441664209 | 0.257824769 | 37.953148 m | ambiguous |

这两次最终仍有 69 个候选，且照常检查同-transform alternative。原先近 phase 的两候选在增加预算后几乎合并，但新的最佳不等价候选来自另一个 refined seed；与最佳的相位差约 0.126 圈，几何差约 38 m，仍通过原硬门槛且评分差小于 0.3。因此**原 15 次截断的问题确实影响了报告出来的候选，但它不是完整拒绝的唯一原因**。

审计记录 best 已在 100 次预算内到达停止阈值，新 alternative 仍未达到，故不能声称全优化已严格收敛或已经证明数学上存在两个最终极小值。但“15→100 即可得到 accepted”已被这次完整重放反证；不能删除该远距离候选、调 margin 或安装预算修改来制造成功。当前仍没有可证明能完成官方 topo 对位的最小 source 修复；保持原 0.4.0，继续报告实测限制，与感知/四点候选问题分别记录。
