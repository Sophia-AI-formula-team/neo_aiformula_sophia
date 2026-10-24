# R28 PCA moments integration — partial development checkpoint

## Objective and scope

Continue the authorized geometry/runtime repair directly in neo without changing vehicle, simulation, camera, LYA speed or admission thresholds. Complete first-lap mapping and second-lap repeat remain the requested acceptance, not yet achieved.

## Input handoff

Consumed neo-004, current STATUS, AGENTS and PROTOCOL. Prior session023 remains sealed. Canonical repository ID is 1303517209; repair/lane-keyframe-transaction-20261011 remains the authorized branch. No vehicle operation is authorized by a test result.

## Source and working tree

Base e43792a9ce119384d7a6bcfaa502b8980df65f06, previous production geometry b8ece7fb614423277ad4a5e27fa5463d1d8c1198357156390b6cfa03d3d75a77. Existing excluded untracked prototype pair is preserved and not read, imported, tested or staged. Simulator Tools dependency loaders add the one explicit local_moments dependency; three current-source count assertions and seven new loader tests accompany this addition. Historical selected-source module sets stay unchanged. No Assets, scene, vehicle, camera, physics or simulation parameter is changed.

## Work performed

Root created a bounded private dispatcher reusing the existing optional LOCAL_MOMENTS_API_VERSION=1, preserving original mean/covariance reductions and literal NumPy fallback. It changes only temporary PCA statistics computation, not support/radii/eigh/signs/positions/validity or timing scopes. Independent static AST review matched the tested diagnostic helper and test bodies to production. Complete production regression and one actual-recorded-frame extraction profile have now run.

The endpoint-GNSS smoke harness has a separate diagnostics-only change: report camera journal/fault/phase/timing information and stop an already latched bootstrap failure earlier. It does not change success predicates, timeouts, node behavior or map gates; all eight pure-mock tests passed in the full regression. This is not a DDS repair or test pass.

## Evidence and result

One peer-authorized static CPU run: 29 independent arithmetic tests pass, 0 skip, then all22 original/candidate masks compare byte-exact across all29 observation fields plus five derived model arrays. Actual native moments calls were exercised. Source/input hashes unchanged; root subprocess wall 2.8384678 s, exit0. This is Windows NumPy2.4.4 equivalence, not runtime or Foxy compatibility acceptance.

One complete production run: 4279 passed, one original Linux-only process-group test skipped, zero failures/errors. All107 explicit test-file arguments and196 source pins match approval; before/after hashes identical. Actual native9 calls and identities passed, but this invocation reports installed_prefix_verified=false. Root subprocess wall99.982971 s under110 s watchdog; no retry or simultaneous Unity/model/encoding. Both new test groups (29+8) actually ran. Reviewed public reports in evidence retain raw local output digests; original absolute-path/hostname JUnit and manifests remain local.

One causal seq163 diagnostic: exactly one plain extraction196.799100 ms and one cProfile extraction236.311300 ms, identical all29 fields;34 actual native moments calls. Original PNG, raw wheel/three-axis IMU and attitude reconstruction remain exact; no future packet consumed, native/source/input pins unchanged. Root wall1.317011 s under19 s watchdog. The two differently instrumented timings are not a speedup comparison. Engine, filled-mask projection, LYA and full adapter are outside the timed scope. Extraction alone nearly consumes the full200 ms allowance; this is not real-time acceptance.

R27/R28 ambiguous-closure holdout is rejected: 9/12 normal rectangles pass the fixed90% coverage standard;3 fail. Severe normals45→0 by rejecting all45 original severe roots; pooled metrics are not a substitute for per-case coverage. Candidate remains diagnostic-only and is not used in neo. Full rejected result and read-only review stay immutable locally; a reviewed redacted summary will be packaged.

R26 CI has completed:6/8 runs successful; both GNSS DDS runs fail stationary bootstrap after80ms deadlines, while builds/unit tests pass. No future-GNSS fault injection is reached in those failed startup cases. This is not an endpoint test pass.

## Risks and blockers

No new Unity first lap, map freeze, second lap, video or driving accuracy result. Last actual run remains failed trial28. Original matcher40ms, engine80ms and complete adapter200ms gates are retained. Peer owns Unity/Assets and CPU outside explicit bounded test windows; no parallel model/replay/encoding. New caller compatibility on Foxy, strict111-frame engine replay and adapter loader regression are not yet verified.

R29 fixed-arm joint primitive remains diagnostic-only. Its first33-test run has32 passes/1 failed test: helper and independent oracle correctly return an empty oblique band, but an extra expected polygon-size assertion was wrong. Exact binary64 values give band [+2^-55,-2^-55]. Original failed fixture/result stay immutable; a new34-test preparation preserves it as an explicit counterexample and adds a distinct positive fixture, not yet executed. Neither an empty fixed straight arm nor absent line witness can exclude an arbitrary curve. Test driver stopped at this failure; adapter tests and packaging were not executed in that CPU window.

## Next agent actions

Run adapter loader regression and independently prepared joint tests in the next bounded CPU window. Publish this reviewed partial checkpoint/tag to canonical neo and inspect its exact-SHA Foxy CI, retaining failures. Strictly replay the original111-frame engine once with unchanged gates; profile identifies local window handling and corner consistency as major remaining extraction costs, not just PCA moments. Continue geometry repair by making actual incident models and support provenance consistent, without substituting certificate witness positions into observations or claiming physical uniqueness. Request a fresh protected-state Unity lease for two real laps only after original runtime gates pass. Preserve every failed attempt, exact source version and logs.
