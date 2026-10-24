# Agent session: 2026-10-11-planner-runtime-008

Input handoff: 2026-09-28-planner-neo-004

R4 code/evidence checkpoint; result PARTIAL. The original planner completed
the source integration and final local review below. This is not a completed
two-lap demonstration; new Foxy CI and closed-loop runtime remain separate.

## Objective and scope

Continue the authorized new-scene teach-map / fixed-route experiment without
changing vehicle, camera or physics parameters. Repair proven integration or
runtime defects while preserving causal full-mask + raw wheel/gyro inputs,
original safety gates and every failed attempt. Only the neo organization
repository is authorized; no main merge, field-driving approval, raw video,
world-pose stream or GNSS publication is implied.

This session preserves compact failure evidence and a portable schema2 sensor
fixture, then integrates and tests the R4 runtime optimization in neo. The
evidence-preparation subtask did not operate Unity or alter production source;
the original planner owns the actual code change, regression and publication.

## Input handoff

Read AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL, the existing neo-004 handoff,
sealed session007 structure and the R3 receipt. Session007 is read-only.
The evidence preparation subtask explicitly returned ownership to the planner
before final additions, review and sealing. There was one writer at each stage.

Existing limitations remain: ideal semantic masks rather than network output,
flat-ground projection, the teacher's single-side lane-width assumption,
limited longitudinal observability, host-dependent deadlines and same-process
continuity. None has become a measured real-vehicle capability.

## Source and working tree

Base: d49f3c3352f2f7bfb670ea66bcc70cf4e18aae0e, immutable R3 checkpoint.
Branch: repair/lane-keyframe-transaction-20261011. At draft preparation the
matcher and test_cell_grouping.py were modified and test_scan_geometry_cache.py
was untracked. The core remains under active optimization; this is not a clean
final R4 source claim. Individual replay reports bind their own four-module
hashes; source_revision=null means worktree, not an immutable commit.

Final integration predecessor:6cec83368c1bad1b4c5df5e2444ccc27df0f295b.
Runtime source identities and all six modified/new Python files are recorded
in evidence/final_checks.json as raw and LF SHA256. The final matcher is
ddc1a027378246d643f65de60636a95e7e51b21a4cb064369d1d9183c35cc58a,
identical to the three included batched replay sources. The Git commit/tag
identifies the corresponding full source and tests; no circular self-SHA is
invented in this session. New dependencies: none. Physical parameters: unchanged.

## Work performed

1. Reviewed R3 CI readback. Four/five-package Foxy builds succeeded, but the
   reference tests failed on NumPy 1.17.4's empty Nx2 np.unique(axis=0) oracle.
   The production empty path already exists. Reference: 519 passes / 1 failure;
   GNSS combined: 878 passes / 1 failure. DDS and later runtime checks were not
   reached. Handoff CI passed. The test-only repair preserves explicit empty
   expectations and nonempty NumPy oracles; it requires a new tag and new CI.
2. Preserved trial17's zero-image clock failure and its same-process native-QPC
   probe. Unity Mono Stopwatch and native QPC had different origins despite
   equal 10 MHz frequencies. The approximately 236601 s offset is not latency
   and must not be subtracted as a production calibration constant.
3. Reviewed trial18 after the adapter's native-QPC repair. It started, taught
   106 frames and then rejected image410 for 40.7357 ms matching against 40 ms.
   Teacher support remained valid. Engine75.4226 ms and backend97.4254 ms are
   separate timing boundaries. No first lap, frozen route or repeat completed.
4. Recorded the first scan-cache readonly test failure: 15 passed / 1 failed
   because write access could be re-enabled. After immutable bytes-backed
   storage, the test agent reported 16 passed in1.02 s. These are explicitly
   contemporaneous conversation/tool-output records, not reconstructed logs;
   no separate original failure or pass artifact was saved.
5. Reviewed schema2 export/replay: 410 images' motion prefixes, 4505 samples,
   107 exact mask PNGs. Original per-sample source-now/receipt/consumption and
   capture-acknowledged tail are retained. Raw/teach pose digests are checks,
   never estimator pose inputs. The 16 helper tests' saved summary is included;
   their raw log is withheld because it embeds a personal environment path.
6. Copied reviewed sensor/replay reports and two utility sources byte-for-byte.
   Prepared selected trial/profile summaries with source SHA256. Raw truth,
   original videos, personal-root profiler paths and terminal pose snapshots
   remain local. No new heavy computation or Unity action was performed here.
7. Added the later bounded bucket-pair batch reports after planner review.
   The planner reports unchanged graph operations/order plus a configuration
   alias fix; index configuration is captured independently of the mutable
   public configuration. The current narrow suite is reported as18 tests;
   full-suite evidence was subsequently supplied by the planner below. Independently compared all107
   reported poses for each of three runs against the pinned R3 replay, both
   numerically and as little-endian float64 bytes: exact. No core was rerun.
8. Planner integrated bounded bucket-pair batching: original neighbor pairs,
   subtraction/sum order, radius, bucket capacities and graph/forest traversal
   remain unchanged. At most65536 pair distances per temporary batch; >4000
   anchors keep the original sparse path. A standalone permanent oracle suite
   covers boundary floats, negative cells, resource bounds, large-map fallback
   and PCA equivalence. Its first draft had28 passes/2 failures because the
   test-only object omitted _component_sizes; corrected fixture bookkeeping
   produced30/30, without changing production geometry or weakening assertions.
9. Planner integrated exact current-scan geometry memoization and independent
   index-configuration snapshots. Only the scan anchor is bytes-backed; other
   existing arrays remain flag-readonly. No internal shared-geometry mutation
   path was found. Cached unrequested normals are not reliable positions/votes.
   Reset and tracking-state rebuild clear the cache. Rejected fits cannot
   commit pose/map changes; all cache checking stays inside the40ms timer.
10. Added pre_match_engine_ms to the existing pre-match deadline read, with no
    extra clock read or changed admission. A preparation timeout retains this
    elapsed-engine metric but no invented reference attempt/frontier. Added
    regression coverage;59 mapping/transaction tests passed.
11. Full local core suite:926 passes,1 existing Linux-only process-group skip
    on Windows,46.26s,exit0. Included sanitized XML removes its hostname and
    replaces the checkout-root prefix in skipped-test text. An initial draft
    missed that root prefix; privacy review caught it before packaging. Both
    initial generated draft files were moved to local diagnostics, not published;
    the original raw XML is unchanged.
    Simulator adapter suite:208/208 passes,20.743s,unit exit0; aggregate runner
    remains exit1 for the12 known peer differences against878 protected files.
    No rebaseline, full-pass relabeling or physical/simulation parameter change.

## Evidence and result

See evidence/REPLAY.txt for the relative-path procedure and evidence_review.json
for file hashes, review coverage and exclusions. The schema2 replay now requires
the accompanying motion_channel helper; the old single-file recipe is not enough.

| Evidence | Recorded result and boundary |
| --- | --- |
| r3_ci_readback_01.json | Failed Foxy tests, successful builds/handoff CI; not ROS runtime approval. |
| trial17_clock_probe_01.json, trials17_18.summary.json | Zero-image failure followed by actual failed trial18; no fabricated zero-frame mapping video. |
| observed_prefix.json, schema2_audit.summary.json | Simulator masks/motion only, no raw truth/course/GNSS/controller input; timing scheduler is not reconstructed. |
| r3_causal_01.json | R3 offline replay accepts107/107 even though the actual runtime timed out. This alone demonstrates why offline acceptance is not runtime proof. |
| scan_cache_review.summary.json | Early cache01/02 accept107/107; cache03 accepts106/107 and hits processing_deadline at81.892 ms >80 ms before matching. Full early reports remain local; exact hashes, sizes and all-frame summaries retain the failure. Initial readonly failure and bytes-backed repair are disclosed without inventing missing logs. |
| r4_batched_01/02/03.json | Each107/107; all107 poses numerically and byte-identical to pinned R3. Recorded matcher SHA ddc1a027378246d643f65de60636a95e7e51b21a4cb064369d1d9183c35cc58a; not final publication approval. |
| components_batch_01.json | 21 adversarial cases, two capacity failures and three actual index calls; graph/geometry equivalence and microbenchmarks only. The captured profiled path still failed; no runtime pass inferred. |
| profile_r4_01.summary.json | Selected path-redacted hotspots; profiler overhead and failed profiled replay are not a runtime benchmark. |
| final_checks.json, r4_core_suite.sanitized.xml | Final source/test hashes,926 passes/1 platform skip and208 adapter passes; preserved partial aggregate result and previous failed test drafts. |

Trial18 aggregate sampled travel28.76994 m,1186 confirmed cells; TEACH backend
p95=50.9421 ms. Zero output, E-stop and ownership release were recorded, but the
post-halt sample still had0.1030073 m/s residual deceleration. It is not proof of
complete physical settling. The original111 registered parameters remain exact;
two added opt-in values are zero. The 878-file preservation baseline still has
12 peer-attributed changes; it was not silently rebaselined or called clean.

The fixture is a development prefix used for diagnosis and optimization, not a
held-out validation set. Prior pass/fail timings vary by host load. Schema2 lacks
image HTTP-arrival and individual MMF/commit-guard read instants; no evidence
here reproduces the continuous watchdog, lock interleavings, transport scheduler,
or a changed policy's new closed-loop trajectory.

All-frame timing is mixed, not uniformly faster. Engine p95: pinned R3=27.5622ms;
early cache01/02/03=28.8901/26.9867/27.3224ms; later batch01/02/03=
30.8241/33.6850/27.5165ms. Matcher p95: R3=11.4140ms; batch=
15.3818/13.2793/13.7273ms. The final-frame matcher times improve to
35.1754/35.6026/37.0453ms, but that does not erase worse p95. Complete p50/p95/max,
sample counts and both pose comparisons are in scan_cache_review.summary.json.
Engine statistics include bootstrap and rejection; matcher statistics use only
logged matcher samples. pre_match_engine_ms is elapsed engine time before
matching, not a pure preparation timer. No zero imputation or timeout relaxation.

## Risks and blockers

No completed lap, fixed-route feasibility, reacquisition, second lap, new R4
Foxy/ROS runtime, physical motor or emergency-stop acceptance. R4 changes after
the retained final hashes require new evidence. Passing replay01/02 must never
hide replay03 or the actual trial18. The shared Unity Editor remains leased
to the independently authorized physics task, with explicit coordination;
the planner will not operate it until clean Edit ownership is returned.

The copied scripts are diagnostic utilities, not deployment entrypoints. Their
helper contains runtime transport definitions but replay uses only constants and
pure timestamp validation; it does not start an MMF reader or live sender.
Remote publication/CI cannot be claimed from these local records. The following
append-only receipt will provide the actual R4 commit/tag, remote readback and
CI result; it cannot turn this partial session into a completed two-lap result.

## Next agent actions

Publish immutable tag lane-mapping-20261011-r4 on the existing repair branch,
verify canonical neo URL/repository ID before push, and read fresh Foxy/build/
communication CI. Do not move R1-R3 tags or write main. Record receipt separately.
After the coordinated Editor release, verify scene and original parameters,
start a NEW actual run and continue first-lap mapping then teacher-off fixed-route
repeat under the original gates. Preserve fresh logs and fixed-scale contemporaneous
consensus video whether it passes or fails. No reusing a failed run's motion chain,
future data, ground truth input, altered physics or claimed hard-real-time proof.

