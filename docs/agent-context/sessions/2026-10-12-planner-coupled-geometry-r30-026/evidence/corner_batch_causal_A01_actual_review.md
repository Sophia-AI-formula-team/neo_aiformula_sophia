# Independent review of actual A_run_01

Decision: the retained records substantiate ONE valid A-only diagnostic prefix.
They do NOT substantiate runtime completion or a speedup. Actual processing
stopped at the original first engine rejection: 15 processed / 14 accepted of
111 mapping images, sequence 165, `processing_deadline`. No blocking receipt,
source/hash, saved-public-result or stopping discrepancy was found in this audit.

Reviewer performed only named static PowerShell source/JSON/metadata/hash reads
and arithmetic on existing logged values. No Python/import/AST/unit/benchmark,
solver, Unity, production/A artifact modification or excluded-file/alias read,
hash or import. A-only driver (399 lines), APPROVED and A_PLAN were fully read;
PENDING/receipts/report/all15 JSONL rows were parsed, not executed. Original
loader/replay lines 656–946 and stop_once lines 436–445 were read as source.
One initial PowerShell formatting expression failed to parse before data work;
it was corrected for read-only inspection, not a replay retry.

## Permission and exact current sources

- PENDING is false/false/null; APPROVED is true/true with
  `root-r30-current-negative-batch-A-causal-once-20261012`.
  Independent JSON comparison finds EXACTLY four changed authorization fields:
  `state`, `execution_authorized`, `root_reviewed`, `approval_id`. No other scope,
  source, interpreter, inventory or deadline field changed.
- Literal scope is A-only, current worktree HEAD
  `9b8636fc3c1aa9e9712142b3924a722211df19bd`, Python 3.13, one child,
  60-second primary / single 5-second stop / zero retries. Parent repo snapshots
  before/after are exactly equal, including that HEAD.
- All FOUR recorded maps (parent before/after and child before/after) have 28
  entries and equal APPROVED pins. Independent current hashes of those exact
  28 allowed files also match, with no mismatch. Before those hashes, literal
  basename/file-link and directory-link checks were performed within the named
  roots; no excluded prototype or alias was read/hashed.
- The inherited capture02 manifest has 23 pins; ONLY corner_brackets and
  paint_geometry changed from it. The five additions are the corrected test,
  unit02 parent/child/approval evidence and A driver. BASE d006a980 is unchanged.
- Corrected unit02 parent/child receipts are hash-bound: actual 21 methods,
  exact method ledger true, failures/errors/skips empty, source before/after
  equal, exit0 and child stopped. This is existing unit evidence, not tests run
  by this reviewer and not actual all111 OFF/ON equality.
- Actual original loader count is 1. Its 17 executed core raw-source hashes
  exactly match their APPROVED full paths; report hashes equal child hashes.
  Driver retains the actual module objects and checks registry/object identity
  after restoring wrappers; recorded `module_registry_unchanged` is true.
  Native C SHA9dae4a7a and CP313 binary SHAf2f97127 match the report and pins.

This is byte/source identity for this retained execution, not a fresh Linux/
Foxy/ROS/Orin run or a guarantee against every hypothetical concurrent file edit.

## Original input and actual causal prefix

- Fixture SHA ce1b36e4 matches the pinned current file and report. Independent
  JSON counts: 261 sensor frames / 111 embedded PNG mapping images / 2868 raw
  motion samples. No bag scan or substitute input was performed.
- Actual mapping images are exactly the first 15 mapped rows, sequences
  151–165, with matching original image stamps and mask SHA fields. This audit
  separately decoded only those 15 base64 PNG byte payloads for SHA verification;
  all15 match their original declared SHA. No image/model processing occurred.
- Sensor comparison rows are exactly the first165 original sensor sequences;
  original fixture motion count through that prefix is 1809, equal to report
  `recomputed_motion_samples`. All165 recorded raw-odometry/raw-attitude flags
  are true; all15 recomputed prior flags are true. These are the original
  replay's numerical comparisons; this audit did NOT reintegrate the sensors.
- Raw sensor-loop source SHA5e57fef0 is unchanged. The original per-sample
  source-now/receipt/consume/capture-ack semantics remain; engine call still
  receives original motion stamp and boundary observation. No OFF engine loop,
  future-frame model update, new odometer or GNSS input was introduced by A.
- Calibration selection remains recorded: effective/recorded observation-plane
  offset both0.007m, candidate_override null. Source default matcher comparison
  and existing past-frontier checks remain. No car/simulation parameter write
  exists in A's workflow; this is not an independent live scene-state attestation.

## Public results, failure and stopping

- All15 saved JSON public results structurally/value-exact match their own
  report results, with identical sequence/stamp order. This is the driver’s
  public dictionary equality contract, not an archive of full private map state
  or a new IEEE-bit oracle for every JSON float.
- Sequences151–161: 11 bootstrap accepts. 162–164: 3 visual accepts. 165: the
  first and only rejection, reason processing_deadline. Source replay loop
  lines853–854 breaks immediately on rejection; no later mapping row is saved.
- Parent/child failure fields null; exit0 means complete DIAGNOSTIC evidence,
  not completed mapping. `runtime_pass_claim=false`, B flags false, profile false,
  complete-adapter/repeat-budget flag false. A neither invokes B nor an OFF
  engine. Prepared B source being present is not B execution/pass evidence.
- Parent PID19928 / birth134362458031671409 and child PID67516 /
  birth134362458034933853 agree across start/owned/child-start/child/parent
  records; start and APPROVED SHA chains agree. Exact child bootstrap executes
  SHA-verified SAME driver bytes. Source stop_once kills at most that owned
  process once and waits5s; normal A needed no kill/timeout/retry. The original
  owned child PID was absent on this review's read-only process query.
- Child wall3.5915846s; parent wall3.9410533s; stdout/stderr empty. All8 parent
  output artifact hashes independently match. Timeout cleanup was NOT exercised
  and is not proven by this normal-exit example.

## Timing: preserve distinct clock scopes

| Recorded scope | Actual values | Boundary |
| --- | --- | --- |
| Matcher | seq165: 33.7319ms <40ms; visual seq162–165: 26.5475/31.2754/31.2352/33.7319ms | original selected matcher clock, not complete callback |
| Original engine result | seq165: 81.18699997ms, rejected at original80ms gate | original deadline precedes later commit/result work |
| Engine forwarding wall | seq165: 81.42210002ms | includes wrapper/return wall, distinct from internal engine clock |
| Extraction forwarding wall | 15 logged rows: mean145.23759333ms, range134.471099998–163.630400028ms | one current-prefix observation, not matched OFF/ON gain |
| Image.open-start through engine return | mean198.53420668ms, range172.991400003–248.150300002ms | includes PNG validation/convert, projection/extraction/engine; excludes earlier base64, motion, output IO, transport/control |
| Parent/child wall | 3.9410533s /3.5915846s | source loading/setup/sensor/IO included; not per-frame WCET |

Five of15 partial decode-to-engine subtotals already exceed200ms, but the
complete adapter clock is not exercised/measured here. Do not turn a partial
subtotal into either whole-callback acceptance or a physical-target WCET result.
Sequence164 is accepted despite returned engine time80.92630003ms: the original
admission check is earlier than commit/result work. That does NOT authorize
loosening80ms, moving clocks or claiming every returned engine time passes80.
Forwarding wrappers do not alter arguments/objects; diagnostic copies/IO happen
AFTER the process return. Their clock overhead is still part of this run where
it falls inside the measured scope. No profile was used.

## Limits and next evidence

Report `changed_model=true`, `historical_state_equivalence_required=false` and
`all_non_timing_mapping_exact=false`; all15 per-row historical non-timing flags
are also false. Own-report equality is NOT historical geometry/state equality.
No private-map archive is retained, so none can be inferred equal. There is no
paired OFF engine or all111 observation-byte equality / kernel speed measurement
in A. Older pristine capture/profile timings have different clocks/conditions;
comparing them cannot establish causally that this batch patch is faster.

The proper result is a source-bound current A prefix still blocked by engine80,
not a completed runtime model. Separate B correctness/cost, unchanged-gate full
callback and requested new-scene first-lap/second-lap logs/errors/video remain.
No ROS communication/build, physical vehicle safety, completed laps or delivery
success is established by A or this static review.

## Independently matched artifact SHA256

- run_A_once.py: `5dfaf136c616ecba42615f24df83c4624d1c5bd34529fbabd7209da6a5762d6d`
- A_PLAN.md: `2aa75adcd54d2334c0bf62c133a33f908883d24bb80e827c28daadfd2cdf46df`
- PENDING: `23b4b95b8a2644d823053e94fd35f6a36293d50abb7be2c660a781e8821faab1`
- APPROVED: `163fac26006c747d8a461868c9b121d1e13cb2a19efed2df872d0cfb41def7bb`
- parent result: `35fd8f17d96790ed088a2bfbf5fb44d5eb860ae75224a7fe3dd39d6fe6e785e6`
- child result: `3a87bb346998ad99dc35e9e08b37d6cd44e283b5cdcb3e80d06cb29cbc930111`
- replay report: `8979f0a12550e916f1bf6576e164f78d974b153ef3b4019ce9df879bf6f83b85`
- frames JSONL: `e645e49b97fcff61c9563364fc1e6f354d95185182d969c3a402e413fbdba17d`
- start: `9cfa3a0b04212605cd2f9779a753beadf77f6e6e260b86d8b7007d5ba0ff9862`
- child-start: `ffb1930b29948e4ca4e1cb1c72aa2cec7a836d9c946e4962b03d53a96307be3b`
- owned-child: `4c0e6a233b2650df3f08a4a282a616635367431e85a1c8353639cd88a87db632`
- each empty stdout/stderr: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`
