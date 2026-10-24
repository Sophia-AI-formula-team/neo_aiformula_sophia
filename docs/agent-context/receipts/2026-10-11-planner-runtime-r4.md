# 2026-10-11-planner-runtime-r4

Planner receipt: RESULT_READY for an auditable partial code checkpoint,
not a runtime acceptance, deployment permission or completed first/second lap.
The SHA below identifies the existing code/evidence checkpoint, not the later
commit containing this append-only receipt.

- Exact code/evidence commit:
  [d1c5d33df28f9dea4fcd8e814c6e9a5f93c3eef8](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/d1c5d33df28f9dea4fcd8e814c6e9a5f93c3eef8).
- Branch: repair/lane-keyframe-transaction-20261011; immutable annotated tag
  [lane-mapping-20261011-r4](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/lane-mapping-20261011-r4).
  Tag object: f04be2d7941d9e467419006f22ed9795569079de.
  The read-only CI receipt verifies canonical repository ID 1303517209 and the
  annotated tag target at that exact commit. No main merge or old-repo write was performed.
- Sealed [session008](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/d1c5d33df28f9dea4fcd8e814c6e9a5f93c3eef8/docs/agent-context/sessions/2026-10-11-planner-runtime-008/SESSION.md)
  and [ZIP](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/d1c5d33df28f9dea4fcd8e814c6e9a5f93c3eef8/docs/agent-context/bundles/2026-10-11-planner-runtime-008.zip):
  5,650,245 bytes; SHA256
  9f4637f0b7ef15bad04d9b98fb91628068d371376c3830a0ddd844bc3b25e4c5.
  The local ZIP byte count and hash were independently checked while preparing
  this receipt; the sealed session and bundle were not edited.
- Local core suite: 926 passes / 1 existing Linux-only process-group skip on
  Windows, 46.26 s, exit 0. Adapter suite: 208/208 passes, 20.743 s, unit exit 0.
  The adapter aggregate verification remains exit 1: its original 878-file
  preservation manifest still detects 12 known peer-attributed differences.
  These are not attributed to this optimization, hidden, or rebaselined; the
  aggregate is not a clean preservation pass.

## R4 remote CI — all six completed successfully

Source: r4_ci_readback_01.json, verified 2026-10-10 18:18:10 UTC.
All runs below use d1c5d33. Branch/tag duplicates and the reference suite repeated
by the GNSS workflow are not additional unique algorithm tests.

| Workflow/ref | Run | Actual result |
| --- | --- | --- |
| Lane Foxy / branch | [38074893106](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893106) | SUCCESS; 4 packages built in 21.6 s; 568 reference tests passed in 35.73 s; 4 road-detector header tests; 63 DDS checks; 26 neo upstream runtime checks. |
| Lane Foxy / tag | [38074893148](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893148) | SUCCESS; 4 packages built in 21.9 s; 568 reference tests passed in 35.67 s; 4 header tests; 63 DDS checks; 26 upstream runtime checks. |
| Endpoint GNSS Foxy / branch | [38074893103](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893103) | SUCCESS; 5 packages built in 12.5 s; 927 tests passed (568 reference + 359 GNSS; 18.61 s / 8.38 s); 4 header tests; all 151 checks across 9 DDS scenarios passed. |
| Endpoint GNSS Foxy / tag | [38074893175](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893175) | SUCCESS; 5 packages built in 21.7 s; 927 tests passed (568 + 359; 35.04 s / 16.11 s); 4 header tests; all 151 checks across 9 DDS scenarios passed. |
| Agent handoff / branch | [38074893099](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893099) | SUCCESS; Ubuntu/Windows x Python 3.8/3.13: 33 tests per environment, all four environments passed. |
| Agent handoff / tag | [38074893188](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38074893188) | SUCCESS; same four-environment matrix: 33 tests per environment, all passed. |

Each Foxy unit result has zero errors, failures and skips. The fixed package has
zero separately collected tests; this is not an extra passing test suite. Its
interfaces are exercised by the shared regression/DDS checks. The previous R3
NumPy 1.17.4 empty-array test-oracle failure is repaired in this new checkpoint;
the immutable R3 tag and its failed CI records remain unchanged.

The DDS checks use actual ROS messages on synthetic private-localhost topics.
The 26 upstream checks run the real neo LYA and production road-detector
publication method with inference bypassed. They are not a network-inference
benchmark or vehicle test. GNSS scenarios are default_guard, normal,
no_gnss_build, startup_gap, future_gps, missing_bits, uncertainty, future_wheel
and future_gyro. Their fixture teacher speed is 0.2 m/s; every scenario explicitly
records physical_stop_tested=false and production_speed_default_tested=false.
These checks do not authorize GNSS mapping or runtime GNSS use.

Artifact review is asymmetric and explicit: the two GNSS ZIPs (IDs 11678218315
and 11677973686, together 474,270 bytes) were read in memory, their GitHub SHA256
digests verified, and evidence/runtime.json parsed. No archive was saved.
The two Lane artifact manifests were read (IDs 11677888878 and 11677344334);
their archive bytes were not downloaded or hash-verified. Lane DDS/upstream
counts above come from successful job-log completion summaries.

## Offline optimization and six-run ablation — timing remains unstable

R4 adds exact current-scan geometry memoization and bounded bucket-pair batching;
it does not relax the 40 ms matcher or 80 ms engine limits. Three retained
batched development-prefix replays accepted 107/107, with all 107 estimated
poses numerically and float64-byte identical to pinned R3. That does not erase
the earlier cache03 pre-match 81.892 ms rejection or the worse p95 timings
documented in session008. The initial readonly-cache test failed (15 pass / 1
fail); the bytes-backed repair then passed. Those early test outcomes exist as
contemporaneous tool/conversation records, not reconstructed failure artifacts.

A subsequent six-run alternating ablation is bound to the exact R4 revision
and the same 107-mask schema2 fixture. Both normal and cache-cleared modes use
the same explicit diagnostic wrapper; no deployment source, clock or budget
was changed. Every run reached all 107 frames, then stopped at its first
rejection if present. The six outcomes must be read together:

| Run/mode | Accepted | Engine p50 / p95 / max, ms | Matcher p50 / p95 / max, ms | Sequence410 result |
| --- | --- | --- | --- | --- |
| 1 / normal cache | 107/107 | 14.2743 / 26.1851 / 65.2187 | 6.9780 / 10.9025 / 34.4222 | accepted |
| 2 / clear each match | 107/107 | 20.0307 / 33.0144 / 78.2935 | 10.3483 / 16.0599 / 39.0414 | accepted |
| 3 / normal cache | 106/107 | 20.9940 / 37.9056 / 81.3571 | 10.4940 / 16.7530 / 35.9731 | processing_deadline |
| 4 / clear each match | 106/107 | 20.1605 / 38.0278 / 77.0159 | 10.1236 / 15.8386 / 40.1611 | match_runtime_budget_exceeded |
| 5 / normal cache | 106/107 | 20.5611 / 38.4889 / 78.1666 | 9.7336 / 17.4258 / 40.5032 | match_runtime_budget_exceeded |
| 6 / clear each match | 106/107 | 19.5183 / 36.9275 / 78.0274 | 9.8317 / 16.6842 / 40.5819 | match_runtime_budget_exceeded |

Each engine distribution contains 107 samples; each matcher distribution contains
99 logged samples, without bootstrap zero-imputation. pre_match_engine_ms is
elapsed engine time before matching, not a pure preparation timer. All common
accepted poses are byte-identical to run1; failed final-frame outputs are not
claimed equal to its accepted pose. Normal-cache and cache-cleared modes each
passed the full prefix only once out of three attempts. This small host-sensitive
ablation establishes neither stable runtime margins nor a general speedup.

The fixture is a development prefix used for repair, not a held-out dataset.
Schema2 retains recorded causal mask/motion and timing fields but lacks image
HTTP-arrival and individual MMF/commit-guard read instants. Offline replay does
not reconstruct watchdog scheduling, lock interleavings, transport timing or
a changed policy's closed-loop trajectory. No simulator truth, future pose,
course or GNSS is supplied as an estimator input.

## Actual closed-loop trial18 — failed before the first lap

The latest actual trial18 preceded R4 and used the R3 core with the corrected
independent-motion/native-QPC adapter. It passed stationary warmup and explicit
Start, then accepted 106 of 107 mapping attempts. Sequence410 entered HOLD for
40.7357 ms matching against the unchanged 40 ms limit. Engine time was 75.4226 ms;
backend processing was 97.4254 ms. Teacher support was still valid. These three
timings describe distinct boundaries and must not be substituted for each other.

Sampled travel was 28.76994 m (28.770 m rounded), with 1186 confirmed cells.
There was no completed first lap, frozen route or second-lap repeat. The failure
video preserves the fixed 60x60 m view and only consensus available at each
logged reply. Raw logs, video and simulator world-pose streams remain local.

HOLD sent zero; the saved post-halt snapshot confirms E-stop and released control
ownership, but still records 0.1030073 m/s residual deceleration. It proves neither
complete physical settling nor a physical-vehicle emergency stop. The scene then
returned to Edit and the backend exited. The original 111 registered parameter
values remain exact; the two added opt-in values are zero. Scene-byte and current
113-value snapshot checks do not cancel the 12 independently attributed source
manifest differences.

## Evidence identity and acceptance boundary

- r4_ci_readback_01.json SHA256:
  aa344a4289fdde311b59197b5db05a5a1a1babc8ff26ab348fbf46f17637f330.
- cache_ablation_01_summary.json SHA256:
  fc9d7f84fd1a0242c7eb83187ab426a0c28260ede441dab80b30e49799674be2.
- Trial18 schema2 fixture SHA256:
  ade7b16987374e874358f9ac88babd1b42f8c9e4f92317933debaf1d4aa3ec2b.

This receipt records stronger build, unit and synthetic ROS/DDS evidence than R3;
it does not turn session008 or trial18 into runtime success. No R4 live lap or
vehicle deployment acceptance is established here. The next acceptance requires
a newly logged, coordinated actual run: complete first-lap mapping, valid route
freeze, then teacher-off fixed-route repeat under the unchanged safety gates.
Keep failures and contemporary fixed-scale consensus video. Do not use future
inputs, truth, GNSS mapping, altered vehicle/physics parameters or loosened
deadlines to manufacture completion. No tests or Unity actions were rerun to
prepare this receipt, and it creates no deployment permission.
