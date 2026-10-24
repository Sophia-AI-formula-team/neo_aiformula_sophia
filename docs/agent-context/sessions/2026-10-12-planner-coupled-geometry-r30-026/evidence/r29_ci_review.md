# R29 exact-SHA CI review

Result: **7/8 workflows succeeded, 1 failed**, all original push attempts `1`, for `9b8636fc3c1aa9e9712142b3924a722211df19bd`. This is a code/build/isolated-DDS checkpoint, not geometry repair, reliable runtime, deployment or driving approval.

## Identity and inspection scope

Live GitHub API verified canonical repository `Sophia-AI-formula-team/neo_aiformula_sophia`, immutable ID `1303517209`, public. All eight named runs have this exact SHA. Branch is `repair/lane-keyframe-transaction-20261011`; tag is `lane-mapping-20261012-r29-rc1`. No legacy repository is used.

Initial state inspection was one snapshot of all eight IDs. The four already-terminal native/handoff runs were reviewed while the four Foxy runs completed; only the latter received a second final snapshot after that independent work. No workflow was dispatched, rerun, cancelled or otherwise changed. Eight terminal artifacts from six runs were downloaded to fresh run-ID directories; the two handoff runs publish no artifact. GitHub API archive digests are recorded, but ZIP bytes were not independently hashed; extracted selected members and standalone native binary hashes were independently checked.

Reviewer activity was read-only Git/GitHub/PowerShell text, XML/JSON and explicit-member SHA inspection, plus these two new diagnostic reports. No Python/tests/AST import/build/native execution/solver/profile/Unity/Play/vehicle operation, no production edit or excluded-prototype content access. Exact committed workflows and both `scripts/ci_verify.sh` were read, not current dirty-tree substitutes. Original downloads and failures remain local and unchanged.

## Terminal runs

| Workflow | Branch | Tag |
| --- | --- | --- |
| Lane exact native graph | [38181799010](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181799010): success | [38181798956](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181798956): success |
| Agent handoff integrity | [38181799059](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181799059): success | [38181798972](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181798972): success |
| Lane teach-repeat, Foxy | [38181799027](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181799027): success | [38181798954](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181798954): success |
| Endpoint GNSS, Foxy | [38181799031](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181799031): **failure** | [38181798986](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38181798986): success |

The branch is 3/4 workflows successful and tag 4/4. A tag success does not replace or erase the same-SHA branch failure.

## Native and handoff checks

Both native workflow runs completed Ubuntu and Windows Python3.13 jobs. Each stdout reports **646 main tests passed plus 954 subtests passed**, not 1600 independently named main tests. JUnit has `tests=1600`, exactly 646 unique `testcase` elements, no failure/error/skip elements. This difference is accounted for by the explicitly reported successful subtests.

All nine native API version entries are `1`, `real_calls_passed=true`, including finite-current-union real calls. Source C SHA is `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`. Downloaded standalone binaries match their own metadata hash. The two Linux binaries match `4d6ef1f112b0058a044e0e2d4c284ebf0c8b71e315b54c80353d945e10f28c5c`; Windows branch and tag binaries have different hashes, separately retained in REVIEW.json. No claim of byte-identical Windows reproducible builds or cause of that difference is made. Standalone jobs explicitly have `installed_prefix_verified=false`.

Both handoff runs completed all four OS/Python combinations (Ubuntu/Windows ×3.8/3.13). Every job reports **33 tests, OK**. This checks packager, sealed records, local links and exact bundles; repeated matrix runs are not 264 distinct tests and do not authorize hardware.

## Foxy builds and units

All four Foxy jobs installed dependencies, built their selected packages and verified the installed native prefix. Lane builds `vectornav_msgs`, real `trajectory_follower`, `lane_mapping_lya_reference`, `lane_mapping_fixed`; Endpoint also builds `lane_mapping_fixed_gnss`. Even the failed Endpoint branch successfully completed those earlier steps.

All four installed native reports have nine version entries `1`, real-call success, `installed_prefix_verified=true`, same C SHA and reported Python3.8 binary SHA `472a3316cfeefd8514b59a9d2306ad92fb26f5edb26e024118ce7236cf81a948`. These Foxy binaries are not in the uploaded artifacts, so this reviewer verified report bytes, not a downloaded binary or a second local native call.

Each Lane/Endpoint colcon log reports **3365 lane-reference tests passed** (1134 warnings), visibly including `test/test_boundary_fit_indexed.py` and all 25 progress dots. `lane_mapping_fixed` collects zero local items; do not assign it a fabricated package test count. Both Endpoint logs additionally report **940 GNSS tests passed** (170 warnings), giving 4305 units each. Each of the four separate Header publication JUnit files records **4 passed, zero failures/errors/skips**, with a deliberately doubled inference boundary; this is not perception-model inference testing.

Artifact completeness limitation: colcon generates `build/package/pytest.xml`, but the workflow uploads `build/*/test_results`, not those generated pytest.xml paths. None of the downloaded Foxy artifacts contains the package-unit JUnit files. Unit counts above come from recorded package stdout/GitHub logs and the completed count gate, not a falsely claimed inspection of absent unit XML. Header4 XML is present. Do not sum repeated runs or main tests/subtests into a new distinct coverage total.

## Lane DDS / upstream scope

Both Lane runs report 63/63 checks in `runtime.json`: real localhost DDS, stationary synthetic lanes, nonempty recorder consensus, reject stationary non-lap, explicit arming, stale pose/mask/teacher and E-stop latches, synthetic frozen-bundle handoff, reference fallback and fixed-only owned-state boundaries. The core fixed handoff uses a simulated STOPPED confirmation. It expressly does not run a complete camera lap, vehicle dynamics or physical stops.

Each independent `neo_upstream_runtime.json` reports 26/26 check occurrences: source-default case 13, YAML-override case 11, plus two completed-case guards. There are 15 distinct top-level check names, not 26 distinct categories. Real installed `lya_0221` runs on remapped private command topics; the real supervisor stops its owned process with recorded exitcode `-2`, not a fabricated STOPPED publisher. Current source default2m/s and private startup YAML override4m/s propagate as expected. The4m/s case is only private supervisor/LYA propagation, not GNSS3m/s motion-gate, physical command acceptance or driving approval. Only source-default case doubles inference to verify production full-mask pixels/Header publication; neither case tests inference or a vehicle. ROS graph absence is sampled diagnostic evidence, not hardware isolation proof.

Both control-demo metrics are byte-identical (`a87d76343817230457a98cd2c4fbc593a01151ea52aa4b4826e0106127483ba1`), `all_pass=true`, with four ideal-unicycle scenarios (0.4/0.8m/s fixture caps ×0/0.25m initial offset), two requested ideal laps each. They start from synthetic stadium trace and ideal observed boundaries; they are not new Unity-camera first-lap modelling/second-lap driving, production speed parameters, geometry acceptance or the requested new scene video.

## Endpoint branch failure preserved

Endpoint tag: all9 cases passed, 151/151 recorded checks true. Endpoint branch: only `default_guard` and `startup_gap` passed; the other seven case processes exit1. Their **121 reached checks are true**, but later checks are not reached. No false check is recorded because the strict wait throws before the next check; this is seven real scenario failures, not a successful121-check workflow.

Every failed case raises `terminal engine: bootstrap_anchor_not_established; waiting for five actual stationary camera commits`. `runtime_smoke.py` stops the wait as soon as the actual engine is terminal; no admission is fabricated. The following are original diagnostic snapshots, not rerun timing:

| Failed case | Reached checks | Processing-deadline rejects | Accepted count if reported | Last completed engine ms | Adapter ms | Last completed reason |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| normal |13 |15 |1 |107.744841 |324.100306 |processing_deadline |
| no_gnss_build |10 |17 |not recorded |86.156707 |307.302361 |processing_deadline |
| future_gps |15 |15 |1 |117.188728 |331.291323 |processing_deadline |
| missing_bits |15 |15 |1 |0.089027 |216.035276 |bootstrap_anchor_not_established |
| uncertainty |15 |15 |1 |108.928449 |326.237137 |processing_deadline |
| future_wheel |13 |17 |not recorded |87.573500 |304.831790 |processing_deadline |
| future_gyro |13 |16 |1 |124.758670 |337.997958 |processing_deadline |

Each terminal snapshot reports `worker_failure=null`, worker thread alive, engine invalid, runtime GNSS disabled, and the independent safety snapshot still TEACH. That state label is not an accepted camera commit, valid map, successful full lap or proof of physical actuation. The `missing_bits` terminal-event engine time0.089ms does not negate the preceding15 deadline rejections. Diagnostic ages are expressly independent read-only samples with possibly lagging journal, not frame admission ages; do not infer which sampled age caused a particular rejection.

Build/native/4305 units/Header4 succeed before this runtime failure. Same SHA passing on a different tag runner does not establish host-load causation, a harmless flake, WCET, reliable80/200ms service or a geometry repair. Preserve both outcomes and original logs. Future wheel/gyro fault-specific checks after camera bootstrap remain unproved in this failed branch, even though the tag reaches them.

## Evidence hashes and boundaries

REVIEW.json records exact public run URLs/IDs/attempts, GitHub artifact IDs/digests, selected downloaded native/JUnit/runtime/unit-log SHA values, scenario counts and timings without raw coordinates or private absolute paths. Primary Endpoint failure `runtime.json` SHA: `fef8a169ea9c1ee98303fdcf93ae4e2a3178eccfacc1c09dcbe19dc654bcccd8`; tag success SHA: `6c10afccf2d3b8fe2ff507292883a0a2522bf057fb8ffc2e273929fe79a1565e`. Original raw artifact coordinates, paths, command journals and synthetic route remain local, not copied into these redacted summaries.

Static reading encountered one PowerShell parse error before its command executed and two overbroad/truncated display selections; corrected narrow read-only inspections established the counts above. The initial memory search no-match exit was not a GitHub failure. None triggered a CI/local test rerun or production change.

Do not weaken matcher40ms, engine80ms or full-adapter200ms, geometry/motion/GNSS/safety gates or vehicle/simulator parameters to make these results green. New cause/fix studies need distinct source and execution records. This review does not approve real sensor timing, localization, motor zero, remote arbitration, hardware E-stop, a successful first-lap map/second-lap drive or new video. R29 remains an allocation/diagnostic checkpoint with a preserved runtime blocker.

