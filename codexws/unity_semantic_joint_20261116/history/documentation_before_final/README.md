# Unity semantic joint-map experiment — 2026-11-16

Latest staged signal policy: **calibrated_rgb_panel_only**. The single default-off
request option is `signal_panel_only`; JointTrial temporarily enables it with RGB
panel recognition and restores both original settings. Raw model detections,
colors and overlay remain visible; only current calibrated RGB candidates with
the existing two-frame confirmation enter the signal policy. Missing RGB mode
is rejected; missing calibration yields unknown without a model-color fallback.
The active v1/v2 guard has been replaced, and both five-file versions remain in
`history/`. The final installer contains six payload files and no identity helper.
Original rollback bytes remain in `before/`.

Thirteen boundary checks and 3268 saved frames passed: default-off full output
matches the pre-v1 helper, and panel-only core output matches that original helper
with model color set to unknown. Independent signal replay preserves stationary
green recovery opportunities in inner05/06; mixed-render inner04 lacks that proof.
These are JPEG replay results, not new live acceptance. Root records final
inner/outer outcomes, including unresolved map ambiguity and lane-support loss.

This opt-in, `COLCON_IGNORE` archive connects the imported 1115 semantic package
to the existing simulator's RGB perception, LLP, and original LYA. It is separate
from Neo stop/wait and the bbox-distance experiment. It does not install ROS
packages, model weights, physics settings, or camera calibration.

The semantic source is **0.4.0**, imported at
[`6b71267e24e81459220c7e6d30d84c3f7a00e591`](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/6b71267e24e81459220c7e6d30d84c3f7a00e591),
in [the existing source directory](../../workspace/src/aiformula/perception/semantic_planning_system/).
No duplicate semantic source tree is included here. Source deployment/version
instructions remain in that package.

The first lap uses the physical green lamp and the new package's own map builder.
After the package reports an odometry reference and the vehicle physically
returns through its starting plane, the harness switches the existing physical
lamp red. Only subsequent camera detections reach the planner. The semantic
package's actual path points go through original LYA and its own velocity gate.
No evaluator route, map alignment, physical lamp truth, or lap count enters
control.

The harness attempts at least two physical laps. A persistent 12-second red wait
is recorded as a failed red detour before switching the physical lamp green to
try completing the second lap. The final result retains that failure even if
green recovery succeeds. Learned odometry closure, official graph fusion,
red-triggered detour, physical turn, and completed laps are separate evidence.

| Contents | Purpose |
| --- | --- |
| `overlay/Assets/...` | Four C# files plus three narrowly scoped perception Python files |
| `before/Assets/...` | Known original transport, follower, detector, perception worker and RGB helper for guarded rollback |
| `transport/semantic_joint_worker.py` | Standalone JSON/ROS-message adapter; no source copy |
| `trials/run_joint_trial.py` | Bounded MCP runner with fixed input hashes |
| `manifest.json`, `install.py` | Hash-guarded comparison, installation, and rollback |
| [DEPLOYMENT.md](DEPLOYMENT.md) | Exact integration procedure and output schema |
| [AGENT_HANDOFF.md](AGENT_HANDOFF.md) | Source/evidence index and pending verification |
| [PROBLEM_ANALYSIS.md](PROBLEM_ANALYSIS.md) | Reasons for adapter-only changes |
| [RESULTS.md](RESULTS.md) | Actual inner03/04 outcomes and rendering-load diagnostic |

Current transport pins:

- C# adapter: `cf506d82e22f1cf10ea26011229bad71c3e368a97121d9673039bb25ea0911a8`.
- Joint command consumer: `ad9dfd2e2353babe55cc9b7e30f26040b20821a8249fd4ffd5ea6c1dfc626a08`.
- Harness: `215d0cb19a78ca13b4a8996493c9fad9f8dee0b6bbbb77abba5d030faeb3f7f2`.
- Perception request adapter: `5140b0e12b0ae78db5f42b744d562caf6c905d02322650d4e41739885fedabb4`.
- Diagnostic worker: `cff1e44ffd6fe5fc2184c6659e933277a6db79454c1f7c04afe23f5ac8591674`.
- Semantic MapAssist: `9b8590588e707173d6e623f144c41e6c1dd97531f92ce4282415eb4dc7eedb5b`;
  the imported algorithm remains unchanged.

**Validation status: corrected live trials pending.** `inner_joint_01` stopped
before completing a lap because the C# Joint branch omitted `vision_commit`.
Actual-frame replay reproduced that failure and accepted the same points after
adding their real provenance. The corrected C# sends that field and respects
the Joint detour's explicit corridor applicability. The newer worker adds only
`error.json`/whitelisted `failed_request.json` diagnostics; its control behavior
and source checks remain unchanged. See the reviewed small receipts and issue
reports in [evidence/README.md](evidence/README.md).

`inner_joint_02` then stopped after 0.603 m with no lap completed: the semantic
watchdog correctly commanded zero for stale visual points, but the original
visual follower permanently shut down the Joint owner. The archived Joint-only
consumer now holds zero/brake on an expired semantic command and can resume only
from a later permitted fresh command. Ownership/manual/fault/pipeline-error and
finite checks remain; the Neo/legacy branch and original LYA are unchanged.
`inner_joint_03` then exposed a harness startup-timing defect, corrected without
changing source or TTLs. `inner_joint_04` completed one physical lap and a
259.5646 m odometry reference, but official fusion rejected
`excluded_gap_too_long`; no red detour occurred. The recorded green recovery did
not complete the second lap. Its rendering settings changed mid-run, so it is a
mixed-render diagnostic, not fixed-cap acceptance. See [RESULTS.md](RESULTS.md).
The subsequent perception adapter review is described below. Fixed-30 inner05,
the outer start, full two-lap/turn acceptance, and
final video review remain pending. This archive conveys no real-vehicle
deployment or driving acceptance.

Historical v1 staging: root identified A4 class-wise NMS retaining conflicting
cone/light boxes. The detector now forwards a nonpersistent, default-false
`signal_panel_reject_conflicting_below_horizon_lights` option. Only JointTrial
enables it and restores its original value. The matching Python predicate is
included with exact baseline/final hashes. It only rejects a higher-score cone's
strongly overlapping class-10 light when its center ray is below the calibrated
horizon and no current same-color RGB panel identity supports it. Raw detections,
overlay and model-state evidence remain.

The audit agent's 17 checks passed: all 839 recorded frames retain identical
default-off dictionaries, all 61 baseline red outputs during the physical-red
segment remain, and the three recorded conflicting false-red frames are rejected
when enabled. Four conflicting model-green frames also become unknown; recorded
JPEG replay is not identical to original inference PNG or a live timing test.
See the [reviewed analysis](evidence/PERCEPTION_INPUT_ANALYSIS.md) and
[receipt](evidence/staged_perception_verification.json). Live complete-map/two-lap
acceptance remains pending. The run-04 results and their original hashes remain
unchanged in `RESULTS.md`.
