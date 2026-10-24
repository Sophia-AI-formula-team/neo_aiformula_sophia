# Current original-input pipeline: one diagnostic profile

One authorized execution completed; no retry or production edit. The profiled replay stopped at its original first rejection: sequence 162, 11 accepted / 12 processed of 111 recorded mapping inputs. This is an earlier instrumentation-affected stop, not an improvement over pristine capture02's 14 accepted / 15 processed at sequence 165, and not a runtime/lap success. Current paired proposal/joint helpers were not imported or enabled.

## Actual identity and stopping evidence

- Original fixture: 261 frames, 111 embedded original mapping PNGs, 2868 causal raw motion samples; only the prefix through the first actual rejection was processed (1776 motion samples). Camera, vehicle, simulation, teach/matcher parameters and source clocks were untouched. Original engine 80 ms and matcher 40 ms gates were retained.
- Original 22 source/input bindings plus this driver: 23 pins, four parent/child before/after maps identical. Actual original loader executed once; its 17 production module names/raw hashes and native C/binary identity matched. No excluded prototype basename was read, hashed, imported or tested.
- Owned child PID 65776: exit 0, elapsed 4.3348014 s, stopped and no longer present on independent readback. No timeout, stop attempt, cleanup timeout or retry. Child profiled replay elapsed 3.9735627 s. Eight receipt-bound output artifact hashes independently matched. The 60+single5 timeout branch was not exercised and is not a WCET proof.
- The whole original replay was profiled, including setup, raw motion, original PNG decoding, projection, extraction, engine and original report output. Three forwarding wrappers also measured stage walls. Frame logging ran after engine return. Profile and wrapper overhead are present; native/NumPy library internals can be coarsely attributed by cProfile. No application worker process/thread was added; this is not an attestation about internal BLAS threads.

## Measured bottlenecks

All following measurements are from the one **profiled 12-frame prefix**, not pristine runtime. Cumulative rows overlap and must not be added together.

| Path | Calls | Cumulative ms | Interpretation |
| --- | ---: | ---: | --- |
| Boundary extraction | 12 | 2311.793 | Dominant complete-pipeline work |
| `_directions` | 12 | 1689.766 | 73.09% of extraction; self 309.606 ms |
| `_direction_paths` | 12 | 704.268 | Includes exact bracket checks below |
| `certify_corner_brackets` | 1488 | 576.023 | Repeated original split certificates; no permission to remove them |
| Indexed finite bounds | 386 | 254.024 | Existing R29 indexed path actually used |
| Quadratic patch fitting | 386 | 228.614 | Existing full support/order retained |
| Engine `process` | 12 | 884.825 | Original 80 ms clock starts within this |
| `BoundaryConsensus.prepare` | 12 | 789.232 | 89.20% of aggregate engine; self 118.811 ms |
| Owner domain advance | 2964 | 341.973 | 43.33% of prepare, already frame-array validated |
| Inner advance body | 2964 | 295.044 | Includes union and interval construction |
| `_current_union` wrapper | 2964 | 99.641 | Wrapper self 72.004 ms; actual native union only 7.764 ms |
| Contiguous cell runs | 12 | 85.836 | Original topology/run walk and pairwise signed normal tests |
| Continuation preparation | 12 | 88.253 | Existing CSR immutable frame path used |
| Boundary transformation | 12 | 15.565 | Validates/freezes unchanged arrays again; secondary here |
| Filled-map `prepare_geometry` | 12 | 7.358 | Not the primary observed bottleneck |
| Local reference snapshot | 1 | 8.403 | Only one preparation before instrumentation stop |
| Permission capacity prepare | 12 | 11.570 | Whole historical re-export branch did **not** execute |

The capacity branch finding follows the caller graph: `_certified_model` ran once, only from `snapshot_near`; `_continuation_permissions` ran once beneath it. Do not blame a hypothetical capacity fallback for this measured engine residual.

Detailed owner costs include 5928 input-vector validations (30.985 ms), 20314 Interval post-init validations (70.758 ms) and 3317 DomainState post-init validations (33.642 ms). Some overlap with the advance rows. Native union consumes only about 7.8% of the wrapper's cumulative time. Rebuilding C is not the first justified contact. Existing `_compact_confirmed_domain` already returns the unchanged state when there is nothing to compact; do not propose that optimization again.

The 12 instrumented stage wall means/ranges:

| Stage | Mean ms | Min ms | Max ms |
| --- | ---: | ---: | ---: |
| Project foreground with sensitivity | 2.772 | 1.788 | 10.925 |
| Extract boundary | 192.907 | 187.589 | 198.431 |
| Engine forwarding call | 73.739 | 48.270 | 124.554 |
| Projection start to engine return | 269.531 | 251.706 | 324.319 |

The last pipeline wall is not the adapter's complete request latency: PNG decode, preceding motion handling, transport, LYA and delivery are outside it. They are not declared free. Aggregate cProfile includes their replay equivalents where exercised, but not actual transport/control. Original pristine R28 extraction 196.799 ms is separate historical evidence, not a current R29 timing result.

Sequence 162's original engine result measured 123.882700 ms; pre-match 8.998400 ms and selected matcher 35.504100 ms. Its remaining 79.380200 ms includes post-match preparation/admission and other engine work; the aggregate profile identifies prepare as dominant but does not independently isolate each last-frame substage. Bootstrap sequence 159 returned processing_ms 80.049900 ms accepted: the original admission deadline precedes commit/result work, so return time can exceed 80 even when that earlier check passed. This existing clock scope was not changed.

## Smallest useful implementation contacts

1. **Prioritize private, sealed owner-domain batching in the existing prepare path.** `_validate_frame_batch` already validates packed arrays once; the remaining 2964 per-owner calls reconvert origin/frame normals, repeat frozen scalar configuration checks, perform Python endpoint-normal screening and construct/validate many immutable Interval/DomainState objects. A candidate can validate proven frame/config invariants once and process ordered numeric owner results in a sealed batch, leaving ordinary public `advance_domain`, unsupported layouts and strict FP/error paths on the literal original route. Keep each creation chart, expiry, one-vote/frame union, saturation, exact interval/capacity result and independently owned immutable storage. Do not silently treat previous objects as certified just because they have the right type, bypass malformed/forged-state handling or let one retained owner pin the entire frame outside storage accounting.
2. **Reduce prepare's small-array Python overhead using exact ordered batches.** Group normalization, incident normal screening and per-group selection repeatedly perform small NumPy reductions. `_group_frame_means`, footprint packing and continuation batches are already implemented and measured cheap; do not replace them gratuitously. Batch only remaining arithmetic with independent literal output and first-warning/first-exception comparisons. Two separate `np.dot` calls and OR short circuit in `_current_union` are deliberate semantics; do not replace them with unverified fused reductions or a tolerance. Public fallback remains necessary.
3. **Extraction must also improve before complete runtime readiness.** `_direction_paths` includes 1488 certificate calls, 342 reaching Fraction conversion after the negative check; the original exact witness/labels must remain authoritative. A frame-local batch of the existing outward negative test, or immutable reuse of repeated binary64 label conversions, is an experiment contact, not an accepted predicate change. Preserve accepted-candidate order and the first witness, exact boundary comparisons and all support labels. The separately prepared unresolved-row gather candidate targets `_directions`/bounds/fit memory work, but does not remove this certificate cost and does not directly repair engine80.

Do **not** start with whole-map export caching, a snapshot spatial index or transform array sharing solely on speculation: their measured contributions here are secondary. They may matter on longer maps, which this early profiled prefix does not reach. Do not promise speed or claim that caching diagnoses geometric drift.

Minimal next evidence for root's chosen production change: independent literal old/new byte equivalence for all original observation/reference/domain/update fields; original owner/member order, duplicate supports, topology, saturated/expired intervals, read-only storage, signed zero/NaN/strict FP and warning/first-error behavior; then one unprofiled source-bound original-prefix measurement including extraction+engine+complete adapter scope with the unchanged 40/80/200 ms gates. A first rejection remains a failure/prefix, never permission to retry until passing. Production geometric effectiveness and actual first-lap/second-lap errors still require the requested scene run; none is supplied by this profile.

## Frozen diagnostic hashes

- Driver: `3e8beaced3f886e675fb120566667baa4d75610d8d1aae1079ffe7dd7b2d365c`.
- PLAN: `fa9dc79148567c4b74688783e625ef00e299cadf8d94ee78bc6aa15ee68b7f14`.
- Root result: `f497df19ead207478c1453382c972ed0e6f84c7e475a76053a2ad9701156492f`.
- Child result: `e6a00748b68eb53ed73fde5888466a22da9ad2106ebacab4535ff6f35c87cd95`.
- Detailed pstats JSON: `ebbfb00ba5ee30d92a7ad5221912e2d363cf1a608f5f2a647dbfbb000e9556c3`.
- Native profile file: `aa9ee8aca57889532ccb0c051ab4de116d01324780dc3ba3ea8099670887780c`.
- Original replay report: `888412acf25e8c91eb4f1316a6f1e8687056ee696c91774682c915ca80534000`.
