# 2026-10-11-planner-repeat-r2

- Planner publication receipt; RESULT_READY for the R2 code checkpoint only.
  The complete first-lap mapping / second-lap fixed-route request is still partial.
- Input: neo-004 and the R1 checkpoint. Exact code/evidence commit:
  [751edef54952902783c19ebb148072ccc7f17152](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/751edef54952902783c19ebb148072ccc7f17152).
- Branch: `repair/lane-keyframe-transaction-20261011`. Annotated immutable tag:
  [lane-mapping-20261011-r2](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/lane-mapping-20261011-r2).
  Branch and peeled tag were read back at that exact commit from repository
  ID 1303517209. R1/baseline tags were not moved. No main merge or old-repo write.
- Sealed [session006](../sessions/2026-10-11-planner-repeat-handoff-006/SESSION.md)
  and [archive](../bundles/2026-10-11-planner-repeat-handoff-006.zip): 13,913 bytes,
  SHA256 `434edf7a53ff12688f76b14d1a8d2a9c0bdbeeae61a5e8d2b0885760a2b635b6`.
  All three reviewed record files match the staged Git blobs. An inherited extra
  blank EOF in the sealed SESSION was retained, not represented as a clean
  whole-record whitespace check. Runtime-code diff checks passed.
- Repair: retain only the committed same-session localization correction and
  timestamp fences across frozen-map index construction. No pending proposal,
  refinement, old confirmation count or restarted-session seed crosses over.
  Original innovation, total-correction and three-new-stationary-frame gates stay.
- Final local production tests: 691 passed, one target-Linux process-group test
  skipped on Windows. Simulator adapter: 160 passed, no skips. These are not a lap.

## Exact R2 remote verification

All six runs succeeded on 751edef. Branch/tag triggers are duplicate executions
of the same code, not six independent algorithm validations. Logs and four
uploaded artifact manifests/reports were read back; no workflow was rerun or
dispatched to conceal a failure.

| Workflow | Branch / tag runs | Actual checks in each run |
| --- | --- | --- |
| Lane Foxy | [38066225203](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066225203) / [38066225224](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066225224) | 4 packages built; 370 unit tests, 4 Header tests, 63 DDS checks, 26 actual neo upstream checks; 4 synthetic control scenarios and 10 safety checks |
| GNSS Foxy | [38066225206](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066225206) / [38066225947](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066225947) | 5 packages built; 692 unit tests (370 + 322), 4 Header tests; 9 stationary DDS scenarios / 151 checks, all exit 0 |
| Handoff | [38066225214](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066225214) / [38066224961](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38066224961) | Windows/Linux and Python 3.8/3.13; 33 tests in every job |

All remote unit tests reported zero failures, errors and skips. Lane artifact
IDs: 11674393440 / 11675073679. GNSS: 11674178715 / 11674368486.
The earlier R1 receipt's phrase "3 synthetic control demos" was a counting
mistake: the runner contains four scenarios plus ten safety checks. That sealed
historical receipt is not rewritten; this correction does not change its result.

## Actual runtime follow-up, including failures

The full pinned LYA image frontend is now the simulation teacher; mapping still
consumes the independent full mask, never its ROI/targets/commands. No change to
vehicle, camera, simulation parameters or safety gates was made by this task.

- Trial15: stopped before Start. Oldest gyro in a contiguous 16-sample batch was
  160 ms old against the unchanged 150 ms consumer gate; latest was 10 ms old.
  This exposed image-request backpressure in the adapter. No motion or map.
- Trial16, unchanged retry: startup passed, 105 mapping frames admitted
  (9 bootstrap, 96 matched), then seq357 safely HOLD. The candidate required
  yaw innovation 0.043014 rad against 0.04; the old active reference failed final
  inliers. This is not a completed lap, not a successful repeat handoff, and not
  grounds to weaken admission. Logs and exact PNGs are retained locally.
- Both runs stopped in Edit mode and their backend processes were stopped.
  No route, repeat driving, physical vehicle, ROS-on-Windows or real camera
  inference acceptance is claimed.

Next owner remains this planner: diagnose the observed geometry and adapter
backpressure, make a separately versioned repair, preserve failed inputs and
rerun the complete two-lap workflow. Main still lacks these packages; a future
operator must explicitly check out the documented tag/branch and read STATUS.
