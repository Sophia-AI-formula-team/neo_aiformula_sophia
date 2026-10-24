# R30 corner negative-screen batching: independent static review 01

Role: planner review subagent. Input: root's narrowly scoped read-only review
request; no new driving, editor, test, timing, or publication authority.

## Baseline and reviewed files

Branch: `repair/lane-keyframe-transaction-20261011`.
Git HEAD: `9b8636fc3c1aa9e9712142b3924a722211df19bd`.
The current working tree is intentionally dirty and is NOT a tested/published
R30 baseline. Root owns all core edits and session026; reviewer edits only this
new exclusive local report.

Reviewed current files, relative to the repository root:

- `workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/corner_brackets.py`
  SHA256 `fdb09f4527b701c6b6175d37c6e8ea72737ecf5d4b59b5eb2cdc54a4498585e3`.
- `workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/paint_geometry.py`
  final SHA256 `6a037de6efd8f49a5a4fcd8cc262aa06f7a6d1a3aaf6d544bd65edc4203058c5`.

I read root AGENTS/context, current STATUS lead and active handoff index,
neo-004, the full handoff protocol, session026 metadata and full446-line draft.
I inspected branch/HEAD/dirty filenames, the two-file HEAD diff, the complete
current and HEAD corner module, the full current `_direction_paths`, public
extractor/projector and configuration routing relevant to dtype/bounds, and
the final changed integration block. Only the two allowed source files were
hashed. Excluded prototype contents were not read, hashed, imported, edited,
tested or staged. Dirty status exposed names only.

## Finding and its correction

An initial integration draft allocated `candidate_normals` with forced
`dtype=np.float64`. This is equivalent on the public extraction path, whose
projection and covariance/eigh results are binary64. It nevertheless made the
stated unsupported-input fallback less literal: old per-slot normals retain
`pair.dtype` until the original certificate's `_array` validation/conversion.

I recommended only `dtype=pair.dtype`, retaining original arithmetic and
validation for unsupported/private non-binary64 callers. Root applied that
one-line correction; I reread it and froze the final SHA above. The earlier
paint SHA `cddc7dcd15200aef20e9789f2161081771dd75639f3033f7860d07eb5d1d4691`
was the pre-correction draft, not the reviewed final integration baseline.
Reviewer made no core edits.

## Necessary-negative predicate equivalence

`_definitely_disjoint_batch` (corner lines140-184) batches the unchanged
`_dot_enclosure` (lines103-109), not a new solver or positive certificate.
For each native binary64 row, the same two products, separate nextafter
roundings, two-component addition and outer rounding are used. The extra
leading dimension does not introduce a dot reduction or reassociation.

The original scalar side dimension is still side2; its label reduction over
K is batch axis2 instead of scalar axis1. Axis and normal broadcasts produce
the same pairwise two-component terms. Radius and footprint lower/upper use
the identical nesting of addition, multiplication by0.5 and nextafter.
The same four derived arrays must be finite on BOTH sides of each row, then
the same three strict comparisons are ORed and reduced over those two sides.
There is no across-row finite guard or comparison reduction: one bad row
cannot suppress or reject a different row.

Let exact side label extrema be L=max(n.W) and U=min(n.B). The stored lower
is at most L and stored upper at least U. The exact half-face footprint
q=v+u*a0+w*a1, |u|,|w|<=0.5, is contained in the outward stored footprint
interval. Each of `lower>upper`, `lower>footprint_upper`, or
`upper<footprint_lower` therefore excludes an original necessary condition.
Every True row can only remove a junction for which the old scalar cheap
negative test also returns True. Every False row remains UNKNOWN, not feasible.
This argument inherits the original module's binary64/NumPy directed-enclosure
assumptions; it is not a new cross-platform execution proof.

## Guards and exceptional values

- Exact base ndarray and native float64 dtype are required for all five
  inputs. Lists, ndarray subclasses, float32, integer, Boolean, complex,
  wider floats and nonnative float64 do not enter the fast path; return None.
- Whites must be `(R,2,K,2)`,1<=K<=512. Other four shapes must match the same
  R and K, with R<=128. Shape dimensions are intrinsically nonnegative.
- R=0 is explicitly an empty Boolean result after all dtype/shape guards;
  no empty-label max/min is executed. The source's original128-junction
  block and unchanged validated K<=512 ensure canonical arrays are bounded.
- Strided, negative-strided and read-only base ndarrays are accepted without
  input conversion or writes. Every computed temporary is separate.
- Signed zeros do not change any strict-comparison truth value. There is no
  reciprocal, division, signbit predicate or signed-zero-dependent geometry
  added by this screen.
- NaN propagation or overflow that leaves any of the four derived arrays
  nonfinite makes that row False, exactly as the literal scalar predicate.
  The guard is deliberately derived-output finite checking, NOT an assertion
  that every nonfinite input necessarily produces False: a -Inf white bound
  can be masked by max or a +Inf black bound by min. The scalar does the same.
  In actual certification any nonfinite original input is already rejected
  by unchanged `_array`; either early batch rejection or survivor rejection
  thus still has the old public None outcome. No nonfinite input is admitted.
- Malformed/unsupported shapes return None, leaving each original candidate
  to `certify_corner_brackets`; no candidate is discarded merely for missing
  fast-path eligibility.

## Integration, label and cut preservation

`_direction_paths` still derives accepted candidates from the unchanged PCA,
minimum support, spread/thickness/variance, signed angle and stability tests.
`np.flatnonzero(accepted)` retains the original ascending slot order.
`side_ids[candidates]` gathers the same complete two-side K face occurrences
in the same order; duplicates, labels and candidate junctions are not pruned.

Normals are still constructed and signed with the original PER-SLOT
`column_stack`, `sum(black-white,axis=1)` and `sum(normals*outward,axis=1)<0`
operations. Their final dtype is now preserved. The batch axes for each row
are the same first/second face vectors in the same order as the old per-slot
`stack`; junction coordinates are unchanged.

Only a True necessary-negative row is marked rejected without an exact call.
Every other row calls the unchanged original certificate with complete
original vertex/axes/normals/W/B. That certificate still validates/copies its
inputs, repeats the original negative screen, converts original values to
Fractions, checks unit/independent bases and rechecks all labels and original
footprint for every positive witness. No rounded batch witness is substituted.

Passing rows still save the original signed normals via `.copy()` under the
same original contour row ID. Score, quality, unique local-peak selection,
tie handling, cut ordering, returned path contents/closed flags and final
`cut_evidence` support IDs/normals are untouched. Analytic callers with no
W/B still take the old mean-line footprint route. No mask/crop, future input,
owner/continuation state, map coordinate, vote, gate, reference speed, vehicle
or simulator parameter was changed in this diff.

## Remaining validation and cost risks

No remaining static semantic blocker was found for the final two-file diff.
This is static eligibility for bounded tests, NOT an executed equivalence,
speedup, WCET, replay, installed-Foxy, first-lap, frozen-route or second-lap
acceptance. Root retains responsibility for those checks.

Required next evidence before accepting this change:

1. Independent literal old scalar predicate versus every batch row; mixed,
   empty/1/128, K1/6/512, strided/read-only, signed zero, NaN/Inf/extreme values,
   unsupported dtype/shape fallback and Fraction checks of every finite True.
2. Off/on `_direction_paths` full returned paths and cut support/normal arrays
   plus original complete synthetic extractor field parity. Explicitly check
   fallback complete packets/order with float32 and final `pair.dtype` route.
3. One newly pinned causal original-input replay/full-pipeline measurement,
   with unchanged matcher40ms, engine80ms, applicable whole-worker and adapter
   limits. Do not rerun until a favorable timing sample is obtained.

There are new bounded batch allocations and a Python per-candidate normal
construction loop followed by a survivor loop. At canonical K=6 allocation
is small; at maximum R128/K512 each W or B gather alone is2MiB before
enclosure temporaries. Every survivor still pays its original scalar negative
check in addition to the batch screen. Fewer cheap-negative calls in a prior
profile therefore does not itself prove a latency improvement; full caller
cost and peak memory need measurement. No native rebuild or dependency change
is justified by this static review.

## Execution and safe state

Commands were read-only PowerShell file reads/rg, scoped git show/diff/status/
branch/HEAD, allowed two-file SHA256 reads and this apply_patch report write.
No Python import/execution, pytest, model, benchmark, encoder, backend, Unity
MCP operation, process launch/stop, staging, commit, tag, push or CI dispatch.
Root's shared CPU/Unity handback remains undisturbed. This report is not a
physical vehicle or Unity driving permission and does not close user acceptance.
