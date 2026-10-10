# Agent session: 2026-10-11-planner-observability-007

Input handoff: 2026-09-28-planner-neo-004

Planner-owned R3 core checkpoint: partial, not a completed lap or vehicle release.

## Objective and scope

Repair the actual neo packages; use immutable tags; demonstrate a new-scene
first-lap map and second-lap fixed route with logs/video. Do not change vehicle,
camera, physics or simulation parameters. Inputs remain full masks and causal
raw wheel/gyro, never teacher-command integration, future observations, GNSS,
reference course or simulation truth. No forced 3 m ROI or speed override.
Only Sophia-AI-formula-team/neo_aiformula_sophia (ID 1303517209) is authorized.
This repair branch does not imply a main merge, field-driving permission,
dependency installation or public raw-video upload.

## Input handoff

Read AGENTS, AGENT_CONTEXT, STATUS, neo-004, PROTOCOL, package documentation,
and sealed R1/R2 records. Local simulation does not update the old field baseline
automatically. Remaining assumptions: ideal binary masks, flat-ground projection,
teacher's existing 2.6 m single-line lane-width assumption, limited longitudinal
observability, host-dependent deadlines and same-process teach/repeat continuity.
None is a measured real-vehicle result or physical operating approval.

## Source and working tree

Base code: 751edef54952902783c19ebb148072ccc7f17152 (R2).
Entry HEAD: 4f79ab173789bac5a0a15a9f48fe069f92a71b56 (R2 receipt).
Branch: repair/lane-keyframe-transaction-20261011. Development used a dirty tree;
it was not represented as clean tagged runtime evidence.

Production changes: mask_localization.py, mapping.py, worker.py, follower_node.py.
No controller, motion, anchors, GroundLookup, LYA profile, YAML/launch, message,
dependency or entrypoint changed. New runtime code uses NumPy and Python 3.8
standard-library-compatible operations only.

Final matcher Windows-byte SHA256:
028199bc7b60ceba998a5ad16812e76631c0d473eecc9e3771332c407f76c083;
LF-normalized: 9f3962a6def177a1ecac796589829fae32c4eda2e47344d372a932463163bd68.
Replay reports include all four module hashes and line-ending equivalence.
A separate receipt will identify the exact code/evidence commit, tag and CI.

## Work performed

1. Reproduced R2 trial16 from 106 original PNGs, 357 chronological motion batches
   and 3922 wheel/gyro samples. Prior digests are checks, not pose inputs.
2. Fixed PCA neighborhoods that bridged disconnected paint and visibility-clipped
   thick bars. Current full-mask geometry supplies direction; only reliable
   positions enter fitting. Old-map centers/votes remain confirmed points.
3. Past direction context only fills missing anchor PCA, never replaces valid
   geometry. Source frontier must precede the current image. Context is published
   only with admitted map/pose state. Cache reuse requires identical complete
   local dependencies and never reuses borrowed directions.
4. Fit/overlap now use reliable-point density in a connected current paint
   cross-section. The numeric 0.55 stays, but this IS a changed statistical
   denominator/weighting, not the same gate semantics. Eligibility is determined
   before correspondence success. Full support adds no position votes/inliers.
5. Repaired a draft false admission: remote/near U bridges produced 27/48 instead
   of correctly rejecting 27/69. Connectivity now stays inside the tangent slab;
   independent all-distance/set and slower generic graph oracles agree exactly.
6. Rejected unconditional hint replacement and inverse-J fit weights after worse
   drift. J remains the existing caller's projection screen, not calibrated
   covariance. Longitudinal correction stays disabled.
7. Retained exact lexicographic grouping and removed unused J aggregation.
   Rejected dense nearest, alternate flood order and squared-distance variants
   without stable benefit. No deadline was widened.
8. Discard pending worker results on denied/false commit or guard failure, with
   real-matcher regression against late proposal reuse.

New algorithm settings: connectivity 0.22 m, normal compatibility 30 degrees,
support capacity 4000. These are not vehicle/physics measurements. Original
support minima, 0.12 m/0.04 rad tracking innovation, total correction, 40 ms
matcher/80 ms engine and sensor-age limits remain. Checks are not preemptive
hard-real-time scheduling.

Final local command from neo root (existing Python environment):

```powershell
$env:PYTHONPATH='workspace/src/aiformula/control/lane_mapping_lya_reference;workspace/src/aiformula/control/lane_mapping_fixed_gnss;workspace/src/aiformula/control/lane_mapping_fixed;workspace/src/aiformula/control/trajectory_follower'
python -B -m pytest workspace/src/aiformula/control/lane_mapping_lya_reference/test workspace/src/aiformula/control/lane_mapping_fixed_gnss/test workspace/src/aiformula/control/lane_mapping_fixed -q --junitxml=r3-core-local.xml
```

Actual: 878 passed, one existing Linux process-group skip on Windows, 39.91 s,
exit 0. Actual JUnit filename was r3_core_suite_01.xml; rerun to a new output.
Earlier 842/1 and intermediate failures remain local. New Foxy/colcon CI was not
run before this checkpoint and is not inferred from R2's green CI.
The first pre-pack handoff unittest command (33 tests, 4.983 s, exit 1) correctly
reported two failures and one error because session007 was still draft/unpacked;
one platform-specific check skipped. No validation was suppressed; seal/pack and
rerun are required before publication. The portable script uses .py.txt because
the evidence whitelist excludes .py; Python executes this text file directly.

[REPLAY](evidence/REPLAY.txt) gives the exact portable R2 command. For R3 omit
`--revision` to load the checked-out files, verify source hashes, and use a new
output path. Never overwrite the baseline, disable clocks or widen budgets.
The tool only replays captured sensors; it cannot operate a vehicle.

## Evidence and result

Same final source: replay23 accepted 105/106, terminal match 40.3902 ms exceeded
40 ms. Then consecutive replays24/25/26 accepted 106/106; terminal match times
32.3755/33.3320/31.6327 ms, whole-frame 62.5381/61.4377/61.8370 ms. All four
reports are retained. This fixed observed prefix is not the changed closed loop.

Held-out common-105-frame scoring, aligned once at first TEACH: lateral RMSE
7.633 cm versus R2 7.366 cm; max lateral 14.766 versus 21.178 cm. Yaw RMSE
1.044 versus 1.087 degrees, max yaw 2.176 versus 2.037 degrees. Mixed, not uniformly
improved accuracy. Raw motion was better laterally on this short straight;
parallel markings do not correct longitudinal drift. Truth is evaluation-only,
read after replay. The rejected first draft had 24.15 cm lateral RMSE.

The included fixture/tool reproduce core failure/fix. Detailed diagnosis-figure
generators and held-out truth scoring are retained locally, not packaged as
independently reproducible scripts. Do not claim all diagnostics are one-command.

Actual trial17 followed 192 adapter unit passes and 28 compiled source checks,
but failed in WARMUP before Start, motion commit, image or map. Unity Mono
Stopwatch was wrongly assumed to share Windows QPC's origin. A native same-process
Edit probe confirmed approximately 236601 s offset despite equal 10 MHz frequency,
not a true 200 ms processing delay. The backend stopped, scene E-stopped/returned
to Edit. Initial physics settling is not described as zero physical displacement.
No zero-image video exists. The adapter correction/new live run require new IDs.

Latest driven run remains failed trial16: 28.457 m, 105/106 mapping frames, no
route/repeat. Its local fixed-scale online-consensus video is 41.3 s, 1280x800,
10 fps, SHA256 f6598775390027f01fe13b07087d90bc6192e75332a6052a948d1bc2e96bedcf.
Large videos/raw simulator logs remain local, not ordinary Git attachments.

## Risks and blockers

- No complete first lap, frozen-route feasibility, stopped reacquisition,
  second lap, physical motor or emergency-stop acceptance.
- Prefix success is not target CPU timing or new-trajectory map accuracy.
- Trial17 transport bug escaped offline/source checks; compiled runtime must
  exercise the correction without relaxing watchdogs.
- Separately user-authorized simulator work introduced opt-in contact changes.
  Our lane scene retains disabled modes; original parameters/scene bytes were
  checked. Preserve the original 878-file baseline; its 12 peer-attributed
  differences are not a clean preservation pass or changes made by this task.
- No automatic field-baseline update or main merge. Clone-main lacks the packages.

## Next agent actions

Planner: verify native-clock adapter during an explicitly coordinated Editor
lease, capture new source/parameter snapshots, and run a fresh first/second lap.
Avoid blocking Editor queries while driving. Keep every failure, delivered-command
journal and fixed-scale contemporaneous consensus replay. Stop on faults; never
reset/patch time or import truth to stitch a successful lap.

Publish this checkpoint with immutable R3 tag on the authorized repair branch.
Read back repository ID and commit, verify Foxy/handoff CI, then append a receipt
with exact results. Future field agents must select the tested tag explicitly,
read its receipt and still obtain physical operating approval under the runbook.

