# R28 Endpoint GNSS artifacts — mixed result, not stable acceptance

Read-only JSON/log/XML/hash review. No Python, test/model execution, encoding,
Unity, source edit or remote mutation. Original downloaded artifacts remain intact.
Only this REVIEW.md and redacted REVIEW.json are new; root owns the production
receipt. Parent reports exact code `8fcd4c0533a13d95400b9f377959733ed798f744`
and overall7/8 workflows; this review independently covers only these two
Endpoint artifacts: branch38171022797 success, tag38171022715 failure
(job114563478846, step6, identity supplied by parent).

## Actual cases and assertions

| Case | Branch | Tag | Recorded checks branch/tag |
| --- | --- | --- | ---: |
| default_guard | PASS | PASS | 11/11 |
| normal | PASS | FAIL bootstrap | 25/13 |
| no_gnss_build | PASS | FAIL bootstrap | 12/10 |
| startup_gap | PASS | PASS | 16/16 |
| future_gps | PASS | FAIL bootstrap | 17/15 |
| missing_bits | PASS | FAIL bootstrap | 17/15 |
| uncertainty | PASS | FAIL bootstrap | 17/15 |
| future_wheel | PASS | FAIL bootstrap | 18/13 |
| future_gyro | PASS | FAIL bootstrap | 18/13 |

Branch9/9 cases,151 recorded checks alltrue; allcase exit0. Tag2/9 cases,
121 recorded checks alltrue but seven case exit1; **121 is a reached prefix,
not complete DDS acceptance**. All18 individual case reports match the combined
runtime reports. A success at the same code/gates does not erase the failed tag
or prove timing stability.

In the failed tag, invalid-start-GNSS rejection for future_gps/missing_bits/
uncertainty **was reached**, as were GNSS/orientation no-pose-influence checks.
The tag did not reach five accepted-camera consensus and its later mapping
assertions. normal did not reach manual-estop/release/end-window checks;
future_wheel/future_gyro did not reach their later raw-motion injections/recovery.
Do not describe all negative tests as passed, nor all GNSS injections as unexecuted.

The successful normal case explicitly proves the stationary fixture cannot be
claimed as a driven route and an incomplete lap cannot start REPEAT. It finishes
HOLD/END_ANCHOR_PENDING, bundle null, with the expected route-construction refusal
“trajectory must have shape N x 3 with N >= 8”. This is not a valid frozen route
or second lap. Both artifacts declare synthetic actual ROS messages, private
output, fixture teacher speed0.2m/s, physical_stop_tested=false and
production_speed_default_tested=false. Fixture speed is not user LYA configuration.

## New failure diagnostics

All seven tag failures report engine phase invalid and terminal
bootstrap_anchor_not_established; accepted1, bootstrap_odometry1, processing
deadline13 or14, terminal-failure1. Worker thread alive and worker_failure null.
Independently sampled safety still says TEACH/teacher command selected; this is
not a transaction-atomic combined state or a REPEAT permission.

| Tag case | Deadline count | Last logged engine/worker ms | Latest received mask age ms | Last completed source age ms |
| --- | ---: | ---: | ---: | ---: |
| normal | 14 | 108.460/343.909 | 89.594 | 618.363 |
| no_gnss_build | 14 | 109.219/342.274 | 57.601 | 670.939 |
| future_gps | 13 | 116.867/383.779 | 98.636 | 714.163 |
| missing_bits | 14 | 0.091/260.114 | 34.122 | 342.637 |
| uncertainty | 14 | 126.000/386.642 | 91.988 | 704.092 |
| future_wheel | 13 | 117.327/380.628 | 37.982 | 758.213 |
| future_gyro | 14 | 110.235/373.664 | 79.494 | 695.193 |

missing_bits last completed event is terminal, not processing_deadline; its
0.091ms engine return is not healthy fast mapping. All other last completed
events still show a prior processing_deadline, while the engine sample already
shows terminal failure. **Journal can lag; ages are diagnostic-read ages, not
commit ages.** No causal commit-age conclusion is inferred.

Observed tag journals contain104 teach_mask rows:7 accepted,96 deadline,
1 terminal; diagnostics record seven terminal states, showing the asynchronous
difference. Successful branch journals contain35 rows:30 accepted and5 deadline;
several successful cases log only4 rows although the predicate read accepted>=5.
Do not substitute journal length for live engine counters or invent missing rows.

Recorded worker compute range: branch164.931–251.973ms; tag260.114–411.319ms.
Worker-minus-engine range: branch114.872–159.787ms; tag229.256–300.983ms.
These measure completed logged worker jobs, not every incoming frame or WCET.
Source inspection confirms worker compute includes projection/extraction, engine
and optional consensus export but excludes queue wait, transport and callback
journal delay. Engine duration is nested; worker compute is not the complete
simulator LaneSession200ms scope. Neither branch PASS nor these times establishes
full200ms or lap acceptance. Original gates are not changed by this review.

## Build, unit and native provenance

Both artifacts independently contain five build JobEnded rc0:
trajectory_follower, vectornav_msgs, lane_mapping_lya_reference, fixed and gnss.
Unit logs collect and pass3340 reference tests plus940 GNSS tests =4280, no
reported errors/failures/skips. Fixed has0 collected tests: pytest command rc5
is handled as colcon job rc0, not an additional test population. Header XML and
logs independently record4 passed,0 errors/failures/skips in each run.
Build/unit completion is separate from failed tag DDS.

Both native.json records show installed_prefix_verified=true, real_calls_passed=true,
nine API versions1 and same identities:
C `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`;
Linux binary `472a3316cfeefd8514b59a9d2306ad92fb26f5edb26e024118ce7236cf81a948`,
Python3.8.10. These are installed native verification facts, not proof of every
DDS extractor call path, runtime stability or vehicle acceptance.

## Evidence preservation

REVIEW.json records72 exact hashes for both runtime/native reports,18 individual
reports,18 process/error logs,18 command journals, build/test event logs,
unit outputs and Header XML/logs. The four local source files used to interpret
timing/diagnostics have recorded hashes and an empty scoped Git diff against8fcd.
The review does not copy source timestamps, coordinates, raw masks, hostname,
domain IDs, personal paths or CI absolute paths. Original artifacts are unchanged.
Warnings in logs are not treated as a root cause; actual engine deadline counts
and latched reason are the evidence. Preserve both runs and continue bounded
runtime/geometry work; no rerun-to-pass or deployment authorization follows.
