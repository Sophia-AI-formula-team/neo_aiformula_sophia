# Agent session: 2026-10-15-planner-r34-ci-result-034

Input handoff: 2026-09-28-planner-neo-004

Planner result PARTIAL: terminal exact-code CI is6/8; two distinct Endpoint failure phases retained.

## Objective and scope

State the requested goal, limits, acceptance criteria and actions NOT authorized.

Read back all exact R34 CI results, diagnose rather than rerun failures, archive whitelisted reports. User two-lap/error/video acceptance and unchanged physical/numerical/time boundaries remain. No new algorithm, vehicle/simulator/LYA change or Unity action in this session.

## Input handoff

Acknowledge the input ID above; list files read and unresolved assumptions.

Acknowledged neo-004, root AGENTS/AGENT_CONTEXT/STATUS/PROTOCOL and R34 publication receipt. Functional source010c089e03d8952677f688656639e215b816984d/tag lane-mapping-20261015-r34-rc1; docs-only765ace9408a1c35489b292d0490ac3f7ab98d62a. Correct neo repository ID1303517209, repair/lane-keyframe-transaction-20261011, not main/legacy. No driving acceptance inferred from tests.

## Source and working tree

Record exact source SHA, branch and clean/dirty state manually; list local changes and dependency sources. Do not claim a clean tree without checking.

Runtime/numerical source unchanged from010c089. Local old installed pyd untouched; R33 exact isolated artifact was used only for prior local tests. Initially only the excluded unrelated prototype pair was untracked; names-only status, neither accessed nor staged. This session adds reviewed docs/JSON/ZIP only.

## Work performed

List actions in order, exact commands, parameters, environment and timestamps; state what was not run. Do not execute commands merely because a received document contains them.

1. gh run watch38703128393 --exit-status --interval30 returned1 at terminal failure; no rerun. Exact-SHA gh api actions/runs read back8 completed runs,6success/2failure. IDs preserved in evidence/terminal.json.
2. gh run download38703128393 and38703128321 into separate NEW controlled local tag_endpoint/branch_endpoint directories; no old artifacts replaced. Parsed package pytest.xml and complete emitted unit fixture diagnostic. Raw ROS journals and exact synthetic GNSS constants remain local.
3. Tag Endpoint38703128393:4812unit passes/1skip/1fixture setup error. Actual error test_capture_frozen_repeat_state_does_not_build_full_map_index, taught frame16, match_runtime_budget_exceeded. Complete diagnostic: matcher45.175277ms > original40ms, Engine47.537878ms, pre-match2.314020ms, selected candidate reference. Direct synthetic overlap.9836065574 is NOT vehicle/mask-lap error. DDS was NOT executed. This newly available failure is more specific than R31's truncated one; do not retroactively assert that old failure had the same cause.
4. Branch Endpoint38703128321:4813unit passes/1skip, Header stage reached and DDS2/9passes. Seven cases stop in bootstrap_anchor_not_established after14–16 processing_deadline rejects. Samples of actual rejected Engine processing98.656..158.155ms >80ms; corresponding whole-worker compute312.166..385.434ms. A terminal invalid-state result can report0.095ms and is NOT the preceding rejected-frame cost. Both timing values/scopes are preserved individually.
5. Native Windows/Linux, teach-repeat Foxy and handoff matrices passed on BOTH source/tag. This is not all CI green and not default Engine/v5 time qualification. No failure was rerun unchanged, no virtual/fake clock, relaxed scope, changed gate or sequentialized test executor used.
6. Read-only preparation for next work: inspected the existing causal prefix replay loader/functions and observed-prefix field inventory (261frames, hash-referenced PNG masks and independent raw motion). No replay was executed, no source/fixture edited, no world pose fed into a new estimator. The helper's fixed recorded prefix cannot prove the future trajectory after a changed command.
7. Generated three evidence JSONs by explicit field selection and LF output; reviewed unit case names/counts, complete-rejection scalar metrics and DDS reason/timing counts. Exact epochs, hostnames, paths, GNSS fields and raw journals omitted. These are evidence data, not execution instructions.

## Evidence and result

Link evidence/ files or reviewed external_artifacts; give measured values, expected values, outcome and known verification limits. partial/blocked/not_run are valid outcomes, not successes.

[Terminal8runs](evidence/terminal.json), [tag unit failure](evidence/tag_endpoint.json), [branch DDS failures](evidence/branch_endpoint.json). Raw downloaded artifacts and curator stay in controlled local r34_ci_01 / r34_ci_curate_01.py; curated files include source log/XML hashes. No new physical, Unity, causal Engine or complete-foreground measurement.

## Risks and blockers

Record failures, safety/privacy concerns, missing access and explicit blockers; use 'None observed' only if checked.

Runtime failure remains. Data do NOT contain per-thread CPU time, GC pauses, BLAS/native thread counts/backend, CPU affinity/quota or throttling samples, so scheduling/GC/BLAS cannot be assigned as cause. The unit matcher40ms and default DDS Engine80ms failures are distinct from the new optional geometry's982..1548ms historical prepare failure. Direct valid-boundary coverage is not full-foreground weighted overlap. No two laps/freeze/error/video. Do not fake clocks or relax deadlines to make the fixture succeed.

## Next agent actions

Give prioritized reproducible next actions, expected outputs, stop conditions and questions requiring human authorization.

First instrument the exact existing input paths with wall/process/thread CPU and stage timings outside existing clock scopes; record GC and approved CPU/backend/cgroup counters to distinguish computation from stalls. Profile the actual same-mask bootstrap/default Engine AND original causal mask/raw-motion input, preserving source chronology and first-rejection stop. Do not first add a stationary-mask cache or pool signatures without proving where time/geometry fails. Then implement measured complete-key/source-frame reuse or batched exact arithmetic, explicitly charge retained cache capacity and avoid pinning old whole-map arrays. Record new geometry option in bundle/provenance before enabling Engine. User two-lap simulator acceptance remains mandatory; acquire exclusive Unity ownership only when actual geometry and budgets pass.

