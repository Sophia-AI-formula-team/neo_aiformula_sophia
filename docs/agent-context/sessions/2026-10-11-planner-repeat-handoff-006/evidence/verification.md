# R2 verification checkpoint

No physical two-lap completion or vehicle acceptance is claimed. No raw motion,
camera image, GNSS coordinate, personal path or credential is in this checkpoint.

## Reproducible package commands

From the canonical neo checkout, use Python3.13 on Windows for the recorded local
results. Set PYTHONDONTWRITEBYTECODE=1 and put these four package roots under
workspace/src/aiformula/control on PYTHONPATH: lane_mapping_lya_reference,
lane_mapping_fixed, lane_mapping_fixed_gnss, trajectory_follower.

```text
python -B -m pytest -p no:cacheprovider workspace/src/aiformula/control/lane_mapping_lya_reference/test workspace/src/aiformula/control/lane_mapping_fixed_gnss/test -q --tb=short -rs
```

Final691 passed,1 skipped,17.76s,exit0. The only skip is test_supervisor.py:62,
Target Linux process-group test; Linux/Foxy CI is a separate later result.
Root independently repeated the full command after source review:691 passed,
1 same skip,18.05s,exit0. All three session files matched their staged Git blob
bytes before packaging; metadata/privacy text was inspected, not merely scanned.

```text
python -B -m pytest -p no:cacheprovider workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_tracking_state.py workspace/src/aiformula/control/lane_mapping_fixed_gnss/test/test_repeat_handoff.py workspace/src/aiformula/control/lane_mapping_fixed_gnss/test/test_keyframe_transaction.py -q --tb=short
git diff --check
```

Focused76 passed,2.34s,exit0. Diff check exit0; CRLF conversion warnings do not
indicate test failures.16 new state cases,18 new handoff cases,42 existing R1
transaction cases. The fake matcher follows the real public state protocol;
the bundle fixture explicitly owns the same frozen snapshot in engine/worker.

## Failures retained

| Attempt | Result | Resolution |
| --- | --- | --- |
| Pre-edit mapping/worker |34 passed,1.16s,exit0|Baseline|
| First handoff focused |81 passed,1 failed,8.49s,exit1|Refreshed active had only2 votes; add3 real admitted frames to mature refinement before proving it does not transfer|
| Handoff focused retry |82 passed,7.11s,exit0|Original refinement assertion retained|
| First full core |673 passed,1 skipped,18.00s,exit0|Before async state API|
| State API focused |75 passed,1 failed,2.60s,exit1|Empty engine correctly forbids freeze; fixture adds actual legal bootstrap frame|
| State focused retry |76 passed,2.34s,exit0|Freeze gate unchanged|
| State API full |690 passed,1 failed,1 skipped,17.98s,exit1|Old FakeMatcher lacked capture/from protocol; add it without removing assertions|
| Final full |691 passed,1 skipped,17.76s,exit0|Windows-only skip above|

Before runtime, independent adapter review also caught incorrect `.correction`
property spelling and raw anchor row-order equality. Correct public property is
correction_se2. Exact comparison now uses the production voxel normalizer, not
approximate equality or altered dense fixture geometry. A provisional synchronous
full-index capture was replaced with immutable state capture plus worker index
construction; the200ms budget was not enlarged. One README patch had a missing
heading and failed without modifying content; reapplied to an observed paragraph.

## Actual-core regression oracle

17 real synthetic engine admissions establish a stationary two-line map while
raw lateral drift increases0.05m per frame to0.6m. Old fresh repeat matcher:
122/122 inliers,zero RMS,but initial_innovation_gate rejects0.6>0.45m. New
same-session handoff keeps correction y=-0.6m and accepts the next current frame.
Tracking0.12m and total1m/.25rad limits remain. Tests also reject excessive new
innovation, stale source time, foreign snapshot/anchor/hash/frontier; preserve
frozen geometry and independent refinement; do not reuse pending/candidate state.

## Simulation adapter evidence (local, not uploaded)

160/160 adapter tests passed,0 skips,14.001s in independent suite; captured root
verify_local report also passes160 tests,13.770s. It returns aggregate exit1
because the separate preservation command flags12 concurrently authorized
simulator source/scene changes against the immutable878-file baseline. These
are not rebaselined away; safe old-mode runtime verification remains required.
The full-index builder thread and unchanged zero-heartbeat behavior are tested.
An isolated100000-point normalized anchor check p95=61.62ms across7 runs does not
prove the full callback meets200ms on all hardware/load.

Actual trial14 used R1, not R2:101/101 map frames accepted,26.627889m sampled
travel,1011 confirmed cells;seq200 fails the old paired teacher's longitudinal
support at source21.929999509826303s. No route/repeat. Final v=w=0, E-stop/Edit
snapshot and backend shutdown recorded. TEACH backend p95=31.4043ms.

Its causal fixed-scale video is local demo_14/replay.mp4,3292460bytes,
SHA2560b46b2c2831d8cf549529d100a4ade99b6f83f8b92e2a31ea6cb45c51a022680;
1280x800,10fps,240frames.200/200 exact PNG/reply pairing; no future map or truth
path. Terminal label is FAIL / NOT full-lap pass, not a successful demo.

New full LYA frontend supports200/200 old trajectory frames, all right_only with
the original assumed2.6m lane width/asymmetric offsets. It uses current image
target/omega, discards vendored speed output, and leaves full mask mapping intact.
Local replay report SHA256
82b4aebad7847d551e5f0511c65208d3a641e9eef2e1efbca946bb08c2dd44e7.
This replay is not the new policy's closed loop or a map/repeat lap.
