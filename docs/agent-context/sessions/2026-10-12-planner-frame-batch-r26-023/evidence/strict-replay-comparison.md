# R25 / R26 严格录制输入回放比较

只读已保存的 JSON、JUnit、日志及 ZIP；2026-10-12。此审计没有运行 estimator、pytest、profile 或 Unity，也没有修改 R/T、原runner或旧报告。

## 结果

R26 严格回放仍失败：原111可用mapping帧中只执行seq151–165共15帧，14接受，seq165因 processing_deadline 拒绝。25/25 integrity checks全true说明来源与回放合同符合，不表示111帧或整圈完成。子进程exit0同样不是算法成功。

R25和R26共同接受前缀为seq151–161（11帧，均bootstrap_odometry）。这11个完整results对象的396个非计时叶子、其中153个float叶子全部一致。JSON类型、key集合、列表顺序逐一比较；float用struct.pack('>d')检查binary64字节，包括signed zero，无容差。仅排除精确字段名 processing_ms、elapsed_ms、pre_match_engine_ms；本11帧实际上只排除了11个processing_ms。没有删除metrics、source时间、prior或covariance，也不隐藏accepted/reason/counts。

四配置对象、observation_plane、graph_acceleration、candidate_initialization全部逐值/逐float字节相同。R25的162条sensor_state_comparisons与R26对应前162条也完全相同：2176叶子、1320个float。R26继续多处理三条状态，不能把不同长度全报告声称逐位相同。

## 同一帧计时证据

|seq|R25 engine processing_ms|R26 engine processing_ms|该帧非计时float数|
|---:|---:|---:|---:|
|151|38.4066|28.2594|13|
|152|46.2893|33.7574|14|
|153|63.3121|33.2417|14|
|154|53.9499|36.2007|14|
|155|49.8973|37.0898|14|
|156|52.0685|37.3301|14|
|157|57.3278|37.9989|14|
|158|53.0324|39.0620|14|
|159|53.7983|40.3657|14|
|160|72.7185|39.9480|14|
|161|70.7594|40.8060|14|

这些是各版本原来唯一一次执行的同帧读数，不是新增基准、重复统计或因果成本归因；不跨不同帧计算速度比例。

首次keyframe / match发生于seq162：

|同一seq162|R25|R26|
|---|---:|---:|
|engine processing_ms|117.21639998722821|71.14610000280663|
|matcher elapsed_ms|39.5834|26.202|
|pre_match_engine_ms|6.4922000165097415|3.6244000075384974|
|matcher budget_exceeded|false|false|
|candidate matcher accepted|true|true|
|engine决定|processing_deadline拒绝|matched_past_consensus接受|

这两次seq162的候选matcher reference metrics，除elapsed_ms外93叶子/37个float逐位相同，budget_exceeded也保留比较且相同。原始prior、mask、stamp、过去frontier一致。R26在提交前预算内通过，开始提交候选：confirmed_cells121→131、trace11→12、首次promotion，并输出修正pose；R25未提交而返回raw prior。因此seq162的整个result确实不同，所有差异都保存在COMPARE_01.json，不能将它纳入“前11帧相同”或宣称两者最终pose相同。

R26新增尾段（R25没有这些尝试，禁止跨帧速度比较）：

|seq|engine processing_ms|matcher elapsed_ms|pre_match_engine_ms|engine决定|
|---:|---:|---:|---:|---|
|163|74.76170000154525|30.5875|0.16430002870038152|接受|
|164|80.59440000215545|31.3004|4.272799997124821|接受|
|165|81.97209995705634|33.7198|4.739099997095764|processing_deadline拒绝|

seq165 matcher本身matched_observable_subspace、budget_exceeded=false；engine拒绝后导出confirmed_cells仍151、trace仍14、promotions仍2，与seq164相同。只是可见导出计数，不把未保存的内部数组声称全量逐字节证明。

### 已知80ms边界必须保留

seq164 accepted而最终processing_ms=80.59440000215545，不能写成“所有接受帧总耗时小于80ms”。封存ZIP内mapping.py（SHA a2171210d68c1d3d55e67cf2e826914f109f382280a7dbdc67a05571da574b88）在491–499行进入commit_guard后检查deadline，500–537行提交/返回，179行在结果生成时读取processing_ms。因此这是已知pre-commit80ms门，不是function-return WCET保证；并非据此新造R26放松门限结论。严格回放也不覆盖live适配层全部200ms端到端准入。

## 来源与负面状态

- 两份source ZIP均23个同名同序成员；21个字节相同，仅paint_geometry.py与continuation_support.py两处预期变化。
- R26 paint：b8ece7fb614423277ad4a5e27fa5463d1d8c1198357156390b6cfa03d3d75a77。
- R26 continuation：178cb6dbaf5af39ae285bd00eb90a7ee2c95b93205c1f78971dc41f15e9b19e4。
- 20 source/support、16实际runtime模块、8 external pins执行前后均匹配。native9与原fixture、配置、plane .007、原receipt/consume/source-now/ACK-tail语义保持不变。
- all_prefix_frames_accepted=false、observed_decisions_reproduced=false、all_non_timing_mapping_exact=false、historical_state_equivalence_required=false均原样保留。后两者区分候选算法和历史录制算法，不应误写成历史状态完全复现。
- R26实际检查165 raw-attitude states、15 recomputed priors、1809 motion samples；未执行的其余96 mapping帧没有任何通过证据。

## R26全部25项integrity checks

- process_exit_zero: true
- source_unchanged: true
- pins_unchanged: true
- report_present: true
- native_binary_unchanged: true
- fixture_hash_exact: true
- all_loaded_sources_exact: true
- exact_sixteen_loaded_sources: true
- all_configs_exact: true
- plane_exact: true
- native_identity_exact: true
- explicit_candidate: true
- boundary4: true
- continuation_representation_exact: true
- certified_model_exact: true
- source_mode_worktree: true
- motion_receipts_and_tail: true
- sequence_mask_stamp_exact: true
- original111: true
- original_budgets: true
- causal_check_counters: true
- no_historical_state_success_claim: true
- first_failure_stop: true
- no_shortened_success: true
- counts_consistent: true

所有原25项逐项保留，source/pins/report SHA另由本审计直接核验。完整值、每帧差异、类型/浮点计数与来源哈希见COMPARE_01.json。

## 关键证据SHA256

- R25 report：9162b803568e0383012c617d8f25ff621e95902ad582908ee9f15d36745c3e9c。
- R26 report：39265568d52e301896a01360fb7f773097a7a6927fb76928e26d58ef80558d85。
- R26 execution：be66701044f577ad48ddbef42449e6c3299b5f42aded0d1e322f331a18624a9b。
- R26 source ZIP：905bc463762692e20d294a7a363a576f3b3fa71d3b0862c21e98b3a95e62b902。

边界：这是固定已录制仿真输入的失败离线前缀，不是live transport、新Unity行驶、freeze/repeat或两圈完成。未保存的全内部地图/临时数组不在本JSON比较证明范围。
