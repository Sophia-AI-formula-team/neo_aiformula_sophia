# R22 严格固定录制输入重放结果：失败，来源完整性通过

主代理按获批 manifest 执行一次。原 trial28 schema3 111 帧输入，原 receipts/source/consume、capture ACK tail、相机与 authored plane 0.007 m 不变；matcher40ms、engine80ms不变。没有新 Unity 实跑，也不是 live transport 或完整圈结果。

## 实际结果

- **12 attempts / 11 accepted**，seq151–161接受，seq162首次失败，未运行剩余99帧、未重试。
- 终止原因 **`processing_deadline`**；engine **100.99060001084581ms > 80ms**。
- 当帧候选 matcher 本身接受 `matched_observable_subspace`，**28.6062ms < 40ms**，`budget_exceeded=false`；因此不能将整帧称为接受。
- `pre_match_engine_ms=5.924199998844415`。未由本次数据细分余下 engine 时间，不能将差额直接归因于某个函数。
- 当帧 overlap/inlier ratio **0.8079710144927535**，rank2，84 nearest / 77 finite-covered / 7 finite rejected；RMSE .007337900218845346→.00445548326653866m。这些是被 engine deadline 否决的候选诊断，不是有效地图更新或轨迹精度真值。
- 计数保留 accepted11、bootstrap_odometry11、rejected1；`trace_points=11`，`confirmed_cells=121`，`active_anchor_frontier_ns=null`。当帧候选过去 frontier18669999605，当前 image stamp18779999603。没有把候选 pose/后验写成一次成功 visual commit。
- 子进程 exit0；driver 内完整性 **25/25 true**。exit0表示报告成功生成，不表示算法完成111帧。

## 来源与合同核对

25项检查逐项覆盖源码/pins前后不变、binary不变、fixture SHA、实际加载16模块完整字典、全部四份配置、plane/native身份、显式候选、v4及representation、worktree来源、原 motion receipts/ACK tail、逐帧sequence/mask/stamp、111完整目标、原预算、重算运动/prior/raw attitude检查、首失败停止、无缩短成功和计数一致。`historical_state_equivalence_required=false`只允许候选算法结果改变，不取消上述输入/因果校验。

本轮为 `confirmed_graph_and_all_continuation_finite_models_v4`。终帧 `continuation_permission_intervals=0`，不能拿本次结果声称新 continuation 许可已在该帧产生收益。另行保存的 synthetic 固定 query 证据不等于此次 recorded-prefix 成功。

原 R21 对照是 13 attempts / 12 accepted、seq163 processing deadline；不同运行的单次 wall-time 不足以归因本次更早失败。没有改善/退化的统计计时宣称。

## 固定证据

- 输入 `../trial28_analysis/observed_prefix.json` SHA256 `ce1b36e4ec3de47be49893ba60a21c6f8b32596eaafc966a4dc6e467f293c421`。
- 本次报告 `r22_observed_01.json` SHA256 `377425c14f7e42df33a046b1fe4895db61ef56e5059c4a20b2aa0f06319c1a1c`。
- `start_01.json` 含唯一实际命令、获批manifest、源zip SHA及UTC；`execution_01.json` 含25检查、全部源/pins前后、终帧完整结果、stdout/stderr摘要。
- 原 `source_snapshot_01.zip`、pending/approved manifest、stdout/stderr和报告保留不改。本摘要不替代原件，也不新增实验。

结论：R22 在固定已录制输入上仍未通过原80ms engine门，尚无新实际圈或两圈成功证据。
