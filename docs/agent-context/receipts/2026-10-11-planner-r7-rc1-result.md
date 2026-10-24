# R7 RC1 checkpoint result — not final handoff or deployment approval

Planner, 2026-10-11. The task remains active; draft session
`2026-10-11-planner-lap-validation-009` is not sealed. This append-only checkpoint
prevents the local/CI pass from being mistaken for a successful two-lap run.

## Exact version and scope

- Canonical repository ID: `1303517209`, `Sophia-AI-formula-team/neo_aiformula_sophia`.
- Code commit: `6bbee374ef4a4b60b20a3d5344adf7fe214d8960`.
- Annotated tag: `lane-mapping-20261011-r7-rc1`; tag object `ab25fac18915716807719f0f95a69bb4fa7b761e`.
- Branch: `repair/lane-keyframe-transaction-20261011`; not main and not field approval.
- Shared matcher, raw-wheel/raw-gyro worker and Python simulation adapter changed.
  No car, simulation, LYA speed, vehicle control or safety-gate parameter changed.
- The old CommonGroup follower remains the explicitly documented legacy path.

## Verification, without conflating levels

Local native regression: 1534 passed, 1 Windows platform skip, 47.54 seconds.
Simulation adapter: 213 passed, 16.176 seconds. Original 878-file preservation
baseline still reports 14 previously known peer changes; these were not reset.
Trial25-to26 scene/preset/component/body values and all 14 current peer hashes
were independently equal. This is not a clean-original-baseline assertion.

The initial motion lateral noise assumption `.1 m/sqrt(s)` failed 2 of 28
synthetic development checks. One predeclared `.05` alternative passed those
unchanged inputs and thresholds; original failures were retained. After fixing
`.05`, 27 new synthetic conditions passed, giving 55 in total. These are not
image/ROS/closed-loop/hardware tests, and the assumption is not measured noise.

The fixed recorded trial25 prefix accepted 530/530 frames versus 529/530 before,
with 6327 recomputed causal motion samples, zero resets and unchanged 40/80ms
budgets. Over 529 common accepted capture sequences, lateral RMSE changed from
1.20735 to .92185 m, yaw .0322685 to .00651351 rad. Both remain worse than raw
(.77011 m and .00225417 rad). The recorded development prefix is not blind data.

All eight CI runs and sixteen jobs passed for the same code SHA:

- [Foxy reference/fixed, tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162800)
- [Foxy reference/fixed, branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162816)
- [Foxy GNSS, tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162801)
- [Foxy GNSS, branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162944)
- [Native, tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162795)
- [Native, branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162818)
- [Handoff, tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162813)
- [Handoff, branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38086162756)

Each native job: 423 tests plus 954 subtests, not 1377 independent tests. Each
reference/fixed run: 1176 unit tests, 4 Header checks, 63 DDS and 26 upstream
checks. Each GNSS run: 1535 unit tests and 4 Header checks; runtime reported
passed=true without printing the check count, so its exact count is unknown.
Installed-prefix and actual native-call checks passed. Handoff matrix: 33 each.

## Actual trial26 — FAILED

1119 captured replies/PNGs: 143 WARMUP, 975 TEACH, 1 HOLD. Of 976 mapping
attempts, 975 were accepted; final sequence1119 hit `match_total_correction_gate`
at 1.00104721m. Match max29.7668ms, map-engine max68.3958ms: not a timeout.
No completed first lap, frozen route, REPEAT or second lap.

The last accepted sequence1118 had raw lateral error -.00310m, corrected
+.99380m, with a +.99691m algorithm correction. In the last100 captures,
corrected position error was worse than raw in every frame. Therefore the
1m gate must not be relaxed to hide this additional drift. The next diagnostic
must cover curved geometry and causal map/pose correlation, not only a frozen
straight oracle map or changed motion weights.

Safety readback: EStop/brake true, armed false, requested v/w zero, residual
body speed approximately2.58e-6m/s (not mathematically zero). Own backend closed,
Editor in clean Edit, zero console errors. Original physical parameters retained.

Video: 1251 decoded frames, 125.1s at10fps, 1280x800, fixed120m square centered
at(20,40), zero clipped samples. Actual received sampled consensus, not a final
map replayed backward; final1757 preview points represent14051 confirmed cells.
Strict renderer failure retained. A separate diagnostic wrapper accepts only
this trial's16 individually verified one-ULP leaves in10 replies; no tolerance
was added to production. Commands/map points/phase/header/PNG hash were exact.

## Reviewed local evidence identities (raw files not uploaded)

| Local evidence basename | SHA256 |
| --- | --- |
| r7_rc1_ci_readback_01.json | 66272ba5625ef66507c7abf6c377470f3faa7d12cf934a70dc606c003bf14843 |
| r7_motion_prefix_01.json | aac7185f15e43deb017f8537543cc2b41e021a43f0d33a4a5b5b738361ea37be |
| r7_motion_quality_01.json | 55fdd53946a0c90aa3e3688726dca428312befaa213cb249917e6fd79b95b049 |
| trial26_preflight_audit_01.json | f66d6b8ed2e5f31ecb1ea75f1da4b4b6660a998bea434da8698bfe07f19064ff |
| trial26_audit_01.json | d7d6692d68d73b6598ffaccd74f7322953f819c44d0d0f581f15fd1ec9ac6708 |
| trial26 drift_audit_01.json | 4da2e431fcb8d49f3f1d391d022ef63344d75ed693cdc9f809e25fb94279ab56 |
| trial26 observed_prefix.json | d882272d4210076056d6530fb2646517bcc4d842a95704e9bff383723d88770b |
| demo26 replay.mp4 | b8bf8a073c66e9d2523d32a5300643e1d5855455407b328ad42bc56a185ad6a4 |

Original PNGs, raw sensor/event logs, exact simulated truth and video remain
local. No raw bag/video/private paths/GNSS coordinates were published. A final
reviewed session package and new receipt remain due when the active task ends.
