# Agent handoff — 2026-10-15 JST

Current user task: update Unity wiring to latest neo, use original LYA for inner
and outer runs, verify red-stop/green-go, topo closure and restart recovery, and
provide videos. Keep bbox distance separate. Analyze necessary fixes before edits.

The other active chat owns SLAM/lane mapping. This task owns semantic topo.
Shared Editor access is the only reason for resource coordination; the semantic
task has not been delegated to that chat. Never use its unpublished R30 sources.

Upstream baseline is neo PR25 commit
`ed8e397ae41732877dfef334f0b89f98c0b20ae8`. The current production pin is
`3c6fbac2ff9f8dd23b247a187e966648876400b1`, MapAssist 0.3.1. The export is
byte-identical to this fixed commit, not unchanged 0.3.0. The only runtime
production edit relative to upstream is the `topo_checkpoint.validate_state`
predicate accepting finite signed progress after a closed reference exists.
Core route geometry, closure thresholds, official-map acceptance, traffic policy
and motor gate are unchanged. Keep the old bbox branch PR22, legacy stop-distance
evidence and failed left-turn investigation unchanged.

The problem was analyzed before correction. `outer_closure_02` actually closed
1028 samples / 285.624625 m into a 286.991534 m reference, then clean process
exit reported checkpoint save failure `negative_progress`; it remains rejected.
The final negative numeric value was not recorded. Separately, official fusion
RMSE 3.453969 m exceeded the unchanged 3 m limit. Do not conflate learned closure
with official-map fusion or erase either failure. The
[production report and 118-test archive](../../workspace/src/aiformula/perception/semantic_planning_system/semantic_local_planner_ros/docs/checkpoint_signed_progress_20261015/README.md)
documents the original reproduction and precise fix. Windows storage shims limit
those tests; they do not prove Unity recovery or native POSIX durability.

The four final cases passed: inner/outer traffic03 and inner/outer recovery03.
Consult [RESULTS.md](RESULTS.md), [MEDIA_RESULTS.md](MEDIA_RESULTS.md) and immutable
per-run hashes. Inner uses component-only; outer uses baseline. Both recovery
cases close a real lap, restore the partial and closed map, and resume LYA.
Official-map fusion is still unavailable; do not claim left-turn success.

At 2026-10-15 01:05 JST, shared Unity/CPU resources were explicitly returned to
the SLAM chat. The semantic task was not transferred. The actual final Editor
was PID 9828, birth FILETIME 134364639991484353, in clean Edit at
Assets/Scenes/LaneTeachRepeatValidation.unity. A later agent must reacquire actual
ownership before Refresh, scene switching or Play; this document grants no lease.
All observed model/semantic/media processes exited, and the independent final
process query found none. Per-case PID/birth receipts are in evidence/cleanup.

Both original-scene and MotorControllerValidation snapshots retain all 116
parameters and complete components/bodies/globals; only realtime differs.
Of 220 original scoped files, 219 are unchanged. The one existing-file change is
AIFormulaSemanticPlanning.cs; additions are the independent SemanticPlanningNeo20261015
tree and AIFormulaNeoTopoTrial.cs (plus Unity-generated metadata). Native LLP/LYA,
camera, light pose, scenes and physics remain unchanged. Rebuilding the package
with stage_install.py produced 135 files byte-identical to the tested installation.
See evidence/cleanup/parameter_cleanup_comparison.json, source_preservation.json
and deployment_rebuild.json. The peer SLAM checkout was never modified.

See [DEPLOYMENT.md](DEPLOYMENT.md) for executable staging/install and MCP runner
commands. The packaged exporter must pin 3c6fbac; check `provenance.json` before
installation. Model environment and bridge are provided by the existing Simulator.
Do not regenerate the historical SemanticPlanning bundle or mix any signal-range
payload into stop_wait. The new worker receives atomic color-only observations
with real capture age/sequence and no distance; original LYA command age is checked
independently of its true receipt time at the native motor gate.

Restart acceptance requires a stationary DiagnosticHold, clean old-worker
checkpoint acknowledgement, changed boot identity, and an actual production
restore event. The worker captures `restore_event` before a later save can replace
the transient persistence state; test the receipt and preserved samples/reference,
not just a later `saved` string. Body pose, clock, detector identity and lane epoch
remain continuous. Never teleport or rebuild a lap to substitute for this test.
A recovery case drives an actual full lap with stationary holds for the two
restarts and can establish closure plus recovery together. Do not add a redundant
no-restart full-lap requirement. Preserve the earlier rejected closure-only case.

The failed `inner_closure_02` stopped on genuine missing lane support. The analyzed
optional `component-only` mode uses the existing current-mask component fallback
with CurrentCorridor disabled; it does not change LLP/LYA source or support/TTL
thresholds. Baseline stays default, options are recorded, and original runtime
flags are restored. Fixed-mask replay addresses that recorded support failure,
and the new inner traffic03/recovery03 cases passed independently. Keep the
runtime first-frame-headroom option and optional panel-RGB correction explicit in
reports as well. Review current source manifests because the final package export
may update the staged runner/harness independently of this document.

Do not describe stdio simulation, synthetic tests or a learned closed reference as
native DDS, vehicle validation or successful official map fusion. Preserve every
failed run and explain its exact blocker.
