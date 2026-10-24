# R21 runtime-equivalence checkpoint: still incomplete

## Objective and scope

Planner continues the requested actual first-lap mapping and second-lap fixed
route in the new Unity scene. No car, camera, physics, simulator, LYA speed,
causal input or original .55 / 5+1 / 40 / 80 / 200 ms gate was changed.
This checkpoint is NOT a completed lap, runtime pass or deployment approval.

## Input handoff

Consumed neo-004. Root reread AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL and the
active handoff.

## Source and working tree

Base is R20 commit51890caaac3be996b6ffef53eee7c64cd8043859,
on repair/lane-keyframe-transaction-20261011, only in neo repository1303517209.
Existing untracked quantized_curve_fit.py and test_quantized_curve_fit.py were
excluded and preserved. Sealed017 and all failed outputs remain immutable.

Root owns the two production changes and architectural decisions. Delegates
provided independent old-body tests and evidence; root reviewed their full
test/runner sources and this evidence before packaging.

## Work performed

- corner_brackets: outward IEEE interval arithmetic rejects only a provably
  impossible label band or half-face intersection before Fraction conversion.
  All uncertain/positive cases still run the old exact solver, with unchanged
  witness order and all original labels. No approximate positive certificate.
- paint_geometry: preallocated whole-frame footprint packing replaces repeated
  per-owner concatenation. Pair-then-own ordering, copied float bits, reliability,
  once-per-frame votes, roots, fragments, edges and gates are unchanged.
- Added154 independent exact-oracle corner tests and26 packing cases, including
  signed zeros, strict NumPy error mode, extreme values and capacity-scale arrays.
- No geometric ambiguity was bypassed to claim a successful test.

Production SHA256: corner e0254dc27fe52c028508ebc5afccce5b1592f514373abdca16dd7ae5e803b0b1;
paint 5f97044c4a8aaab4dd23a2f52976266c2f679d544313b549366e54bbb21dc217.

## Evidence and result

Independent focused commands used the selected new and existing pytest files:
corner310 pass; footprint packing129 pass. These overlap the full suite.
The reviewed one-shot driver sequence was extraction comparison, full regression,
then strict replay. Exact command arrays and hashes remain in the local original
start/summary records; published reviewed_summary.json binds the drivers/results.

1. Extraction comparison: fixed actual seq151-162, one extraction per variant
   per frame, alternating order, no warmups/retries. All12 x28 observation fields
   matched all bytes, with immutable inputs and exact source/native identity.
   Median160.8019 to150.4418 ms; candidate was slower on160/162 and max236.1730 ms.
   This is paired diagnostic timing, NOT a real-time distribution.
2. Full Windows core:3597 passed,1 Linux-only process-group skip,0 failures/errors;
   pytest102.12s, process103.1896s. All183 source/test files unchanged. The
   installed eight native API functions were actually exercised successfully.
   No ROS/DDS or Unity in this local suite.
3. Strict original111-frame fixture:12 accepted of13 attempted; stopped at
   seq163 processing_deadline87.1215 ms > original80. Visual32.187 ms <40,
   overlap.6758620689655174, rank2. Failed frame not committed, trace12,
   confirmed filled cells131. All21 integrity checks passed. No retry.
   Source archives, raw causal motion receipts, original camera and .007m
   observed-paint plane are pinned. No new trajectory or shortened-success claim.

An extra post-full summary assertion erroneously required installed_prefix_verified
even though the literal native verifier command had not supplied its optional
--install-prefix. That read-only assertion failed; the tool orchestration
incorrectly continued to strict without checking that extra command's return code.
The required full suite/native/source checks had actually passed. This procedural
error is preserved separately, not erased or described as a passing assertion.
Future orchestration must gate each command's return code before proceeding.

## Geometry investigation and negative findings

Original saved left69/right13 histories show temporal ID adjacency union can
create apparent forks: A->C and A->B->C occurred in different frames. All15
blocked owners had at most one raw predecessor/successor per original frame.
Non-cooccurrence or a small angle alone does NOT prove physical identity.

A proposed model-invalid-gap identity repair has92/31 conservative candidate
gaps; only6/11 caused real same-record claim conflicts, and NONE touched the
current branch blockers. It was not integrated as a claimed fix.

All-support white/black quadratic feasibility was also falsified as a universal
corner repair: three83-90degree bad corner normals satisfy every original label
strictly, and a same-patch LP remains wrong. Some good circle controls violate
the same quadratic family. No such refit/rejection was integrated. Bad endpoint
roots are inside the original half-face outer footprint; this does not establish
their membership in the actual feasible corner polygon.

Next geometric work must preserve explicit observed path hypotheses and resolve
quantized endpoint side assignment. These are pending designs, not validated
changes. No truth/future data has entered production.

## Risks and blockers

R20 exact CI6/8 workflows and14/16jobs passed. Both GNSS DDS runs failed:
repeated engine80.815-166.790ms deadlines preceded bootstrap failure; this is
from actual command JSONL, not inferred from wait timeouts. R20 receipt contains
all scenario counts and remote hashes. R21 CI was not yet run when sealing018.

No Unity operation, LYA drive, frozen route, second lap or new video occurred
during018. Latest actual drive remains failedtrial28. Original R20 four
development cases remain0/4, with corrected lateral error worse than raw on
two pitched cases. Do not combine unequal-prefix RMSEs as improvement.

## Next agent actions

Reviewed code, tests, summary and this record contain no credentials, personal
paths, raw masks, precise GNSS, bags or binary payloads. Only the compact reviewed
summary is packaged; complete local raw evidence is retained, not silently lost.
The first local packaging validation rejected this report's combined headings;
the required sections were separated before packing. No source or test changed.
Root selects the next candidate; no autonomous car/Unity start is authorized by
this record. Next: repair identified geometric representation, reduce real
pipeline latency, then fresh protected-state preflight and actual two-lap test.
