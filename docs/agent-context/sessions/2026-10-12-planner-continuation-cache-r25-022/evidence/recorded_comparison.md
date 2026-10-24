# R23 / R25 已保存严格录制回放比较

只读 JSON/ZIP 比较，2026-10-12；未重新提取、运行 estimator、测试或 Unity，未修改 R/T 或已冻结 runner/PLAN。这里比较的是同一固定录制输入的两次离线回放，不是 live 行驶。

## 结论

没有发现非预期非计时变化。两份完整报告结构、键集合、列表顺序与类型一致；完整递归比较只有 19 个叶子值不同：15 个计时叶子、4 个预期的 paint_geometry.py 来源哈希叶子。除此之外，全部值相同，浮点按解析后的 IEEE-754 binary64 字节比较（包含 signed zero，不用容差）。

两者均尝试 seq151–162 共 12 帧，seq151–161 为 bootstrap_odometry 接受，seq162 为 processing_deadline 拒绝；11/12 不是原 111 帧完成。两者 seq162 的候选 matcher 本身 accepted=true，reason=matched_observable_subspace，budget_exceeded=false；engine 未提交该帧。

## 比较规则与精确范围

- 只隔离三种计时字段：processing_ms、elapsed_ms、pre_match_engine_ms。具体为 12 个 result.processing_ms、seq162 两处 elapsed_ms（顶层 matcher metrics 与 reference_attempts[0].metrics）、seq162 一处 pre_match_engine_ms。没有宽泛删除所有 metrics 或所有包含 time 的字段；source stamp、motion_interval_s、motion_visual_interval_s 等仍精确比较。
- 两处派生 matcher 布尔 budget_exceeded 单独核验，均为 false，未发生变化。所有 accepted/reason/counts/phase、all_prefix_frames_accepted 等也保留比较，没有作为“计时派生”隐藏；拒绝行为相同。
- results 下非计时叶子共 615 个（包含 233 个 float 叶子）全部一致。每帧原始 prior、最终 result.pose_xyyaw、motion predicted/posterior covariance、创新量、质量指标、计数、reference frontier/attempt/selection 与决定逐值一致。
- 完整报告共 1697 个非计时 float 叶子逐 binary64 相同。162 条 sensor_state_comparisons、candidate_initialization、所有 motion/attitude/teach/matcher configs、plane、native 身份、完整输入 SHA、原 receipts/ACK-tail 语义、因果检查计数及元数据全部一致，来源 paint 哈希除外。
- 两报告的 non_timing_mapping_exact / all_non_timing_mapping_exact 为 false 且一致；这些值比较的是当前 boundary candidate 与历史录制算法，不表示 R23 对 R25 不一致。未将其改写为历史复现成功。
- 两份 execution_01.json 各 25 项 integrity checks 全 true。子进程总耗时和 runner 输出哈希属于外围执行记录，不参与几何逐位等价主张。

## 原始逐帧 engine 时间（ms；仅列证据，不推断性能增益）

| seq | 决定 | R23 processing_ms | R25 processing_ms | 非计时 float 叶子数 |
|---:|---|---:|---:|---:|
|151|接受|37.96059999149293|38.406599953304976|13|
|152|接受|50.82499998388812|46.28930002218112|14|
|153|接受|45.01100000925362|63.31210001371801|14|
|154|接受|49.48839999269694|53.9499000296928|14|
|155|接受|47.59780003223568|49.89729996304959|14|
|156|接受|54.49790001148358|52.06850002286956|14|
|157|接受|53.56770002981648|57.32780002290383|14|
|158|接受|54.50950004160404|53.03239997010678|14|
|159|接受|58.18220000946894|53.79829998128116|14|
|160|接受|54.58509997697547|72.71850004326552|14|
|161|接受|74.805699987337|70.7593999686651|14|
|162|processing_deadline|106.27520002890378|117.21639998722821|80|

seq162 matcher elapsed_ms：34.6718 → 39.5834；pre_match_engine_ms：4.740500007756054 → 6.4922000165097415。matcher 40ms / engine 80ms 均未变。这是两次单次运行，不提供隔离负载统计，也不能证明 R25 快或慢的确定原因。

## 来源变化完整清单

两份 source_snapshot_01.zip 都有 23 个同名同序成员（20 个 core/support 源 + T 的三个适配源）。直接解压字节比较，仅 paint_geometry.py 一个成员不同；其余 22 个成员逐字节相同。

- R23 paint：7f8023c67829b2e4fbf33d636ee71aed30f72a7b4a8558af7f13a17f3539ce90
- R25 paint：c21aa8de5176894fcc4eebc62d16488e3c8d4c71ffed9bfe55de76b8d998b206
- 报告四处预期差异：source_sha256 的 paint 项，以及 source_line_ending_hashes 的 paint.raw / .lf / .crlf；不是额外运行时源变更。

## 输入报告 SHA256

- r23_observed_01/r23_observed_01.json：1778bc9b3197779878689701a6f09f84139d205a09f92e3d743d61c2a694694e
- r25_observed_01/r25_observed_01.json：9162b803568e0383012c617d8f25ff621e95902ad582908ee9f15d36745c3e9c
- r23_observed_01/execution_01.json：2693ed5bd59899eea70f045eacab33b5b2882238d218bf9ca9c26f4882549b59
- r25_observed_01/execution_01.json：da0a46bd854fb7b6c11be4ca9467baf4b707b44c59cf08889703989b473f784e

边界：保存的报告没有包含完整每帧内部地图/所有临时数组；本审计证明其实际导出的所有非计时结果和元数据一致，不能据此声称未导出的全部内部状态逐字节一致。更不把本地全套单测通过或这段失败前缀当成整圈成功。
