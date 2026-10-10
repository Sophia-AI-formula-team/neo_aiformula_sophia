# R9 RC1 — measured consensus / startup consistency; FAILED quality checkpoint

Role: planner. Base is R8 RC1 `ae7e27b909319a49285d50b3e4b3b7aa4789ddaf`.
User requested fixes in neo, immutable tags and a new-scene two-lap test, with
no vehicle or simulator parameter changes. Session009 remains an ongoing draft.
This checkpoint does not replace the field baseline or authorize vehicle output.

## Changes and boundaries

The teach engine now explicitly keeps the equal-frame mean of actual projected
measurements per confirmation cell, instead of reporting the rounded cell center.
Voting keys, one vote per frame, qualification, TTL and capacity remain unchanged.
Geometry is prepared without persistent mutation, then bound to the owner,
generation, source stamp and exact keys before commit. Default key-only callers
retain old behavior. Frozen maps and repeat consume the same measured positions.
This single mean still loses within-cell endpoint extent; no invented padding or
extra support points were added. It is not strong rollback against MemoryError
after matcher commit and is supported under the serialized worker contract.

Raw-IMU opt-in startup gains the independently declared four-block model check,
held-out gyro-bias validation and transport to the actual endpoint time. See
[the predeclared design](2026-10-11-planner-r9-design.md). The angular budget is
an engineering assumption, not measured sensor calibration or an absolute tilt
guarantee. Constant acceleration/bias and yaw remain unobservable at startup.
The moving path remains gyro-only; no ground-truth orientation enters mapping.

No matcher quality/time thresholds, vehicle, physics, camera calibration, scene,
LYA speed, controls or stop gates were changed. The 80ms gate remains a
pre-commit check, not a worst-case function-return guarantee.

## Tests and retained failures

Final core regression: **1804 passed / 1 platform skip, 53.32s**, with native
required. Adapter regression: **249 passed, 19.12s**. These are code checks,
not lap success. Focused measured-geometry32, startup/IMU/worker129 and
transaction42 checks are included in the evidence.

Retained earlier attempts:

- A full-suite command named a nonexistent fixed-package test directory:
  no tests collected, exit1. Corrected two-suite command used existing directories.
- Native02:1803 passed/1 skipped/1 failed. The old handoff oracle expected every
  endpoint covered. Independent literal grouping gives122 scan representatives,
  historical extent ending6.975m and two current x=7m endpoints outside support.
  The new strict oracle requires122 nearest /2 excluded /120 inliers and exact
  120/122 ratio. Initial-innovation rejection, zero residual and repeat retaining
  its0.6m correction remain required; no production admission was relaxed.
- Adapter01:232 passed/2 failed/14 errors. Two old CLI assertions omitted the new
  explicit false candidate flag; the historical Git-blob test polluted subsequent
  imports. Tests now restore module/path state even on failure. Provenance checks
  remain enabled. Candidate replay remains schema3-only and explicitly named.
- Development01 computed its first case then could not serialize the new nested
  initialization dataclass. Its partial output/traceback are retained. A tested
  serializer-only wrapper reran to a new02 output; no input/gate was changed.

## Quality: improvement, but NOT accepted

All four unchanged deployed-camera development circles accepted509/509 frames.
Lateral RMSE R8 -> R9, in left-level / left-tilted / right-level / right-tilted:
0.14384 ->0.06564m, 0.12610 ->0.04711m, 0.14403 ->0.07341m,
0.07917 ->0.02810m. All pass the predeclared0.10m absolute criterion, but
all remain worse than raw odometry at about0.01405m.
All2036 recorded input rows are float64-bit-identical, including masks, raw pose,
normal, bias and clocks. Thus this fixture isolates map representation effects;
it does not establish initialization equivalence on other histories.

The eight predeclared additional cases each accepted201/201, but only **2/8**
passed their complete quality criteria. All six post-start gyro-bias cases failed
the required20% yaw improvement; H3/H4 also exceeded the0.01rad absolute yaw limit.
Their lateral improvement passed. Unbiased H5/H6 passed absolute gates but yaw
remained worse than raw. H7/H8 maximum lateral errors reached0.25413/0.24975m.
No thresholds or cases changed after these outcomes and no retries were selected.
These synthetic runs use declared2000ms diagnostic budgets, not runtime40/80ms.
This set becomes development if subsequently used to tune or repair the algorithm.

The actual trial27 recorded prefix evaluated with current raw-IMU/core code also
**failed**, now at sequence406 (105/106 map attempts accepted), insufficient
overlap. Startup qualified at a later sensor window and all source/receipt/ACK
checks remained active. This is candidate evaluation, not historical equivalence
or closed-loop driving: it cannot substitute for a new scene run.

Local artifact identities:

- Development02 SHA256 `5472f3e50e0ac5405709c80b2606c1cb07d7ca12483ecc2ec8183f685a5763c5`.
- Exact-input comparison `047d643f2d412c07fa2515bb70fc4464887fbe06ddc5a26c86440206a9c5168f`.
- Eight-case result `1a4afff2c1523afe30d46f70b5a739d571fc0f026997a32e5326a7fd1310ad3c`.
- Candidate actual-prefix `5b355b8436abaa21f77334209227f86d4957839d29e5cb76e48662df11147303`.

## Safe state and next action

No R9 live trial has occurred at this checkpoint. The selected new scene is
clean Edit and its backend is closed. Scene SHA remains
`542b69e811b5eca13456a7d39f32c8b47290a2f7211865337497242bd5d3aa98`;
all116 preset fields/root pose match. Original878-file preservation still reports
the same14 known independent peer changes, all unchanged since the prior audit;
the baseline was not reset.

Next: diagnose the exact reference/scan support at the failed actual prefix,
separately from persistent-bias/map correlation. Do not increase gyro noise,
extrapolate support, shrink the overlap denominator, or weaken a stop gate merely
to pass. Complete first-lap mapping and second-lap fixed-route driving remain
unmet; no new successful video exists. Raw videos/images/private paths stay local.
