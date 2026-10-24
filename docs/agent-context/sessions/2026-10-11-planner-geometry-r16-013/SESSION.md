# R16: causal identity continuity and equivalent extraction optimization

Reviewed partial development checkpoint. Original logs remain local; the sanitized
[evidence index](evidence/summary.json) records source identities, outcomes and hashes.
This checkpoint does not pass the two-lap acceptance or authorize vehicle use.

## Objective and scope

Repair boundary-record identity continuity and the extraction cost exposed by
real ROS DDS. Preserve full-mask evidence, causal past-only references, original
5 frame + 1 reliable confirmations, whole-foreground mass denominator, finite
support, original geometric gates and 40/80 ms matcher/engine budgets. No
vehicle, simulator, controller speed, GNSS-use policy, ROI or safety-limit
changes are authorized by this record.

A complete teaching lap followed by a repeat lap remains unachieved. Actual
trial28 remains the latest recorded failed drive; the results below are
offline tests, causal recorded-input replay, synthetic development or isolated
profiling—not a new driving trial.

## Input handoff

- Consumed task: `2026-09-28-planner-neo-004`.
- Base commit: `77f2f17410634e562e78d4370ee47b8c21375e34`.
- Base annotated tag: `lane-mapping-20261011-r15-rc1`.
- Canonical repository: `Sophia-AI-formula-team/neo_aiformula_sophia`,
  immutable ID `1303517209`.
- Branch: `repair/lane-keyframe-transaction-20261011`.
- Read current AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL, consumed handoff and
  formal session013 template. Planner owns implementation and this record.

## Source and working tree

Results are tied to distinct source snapshots, not one claim about the current
working tree:

- Initial R16 identity candidate paint SHA256:
  `6bb0730232290a81579a7d2d0cc53e0d8d02077ae8bf0d1b0de6d569ff95ef85`.
  This owns the 2657/1 core result, original recorded replay and four development
  results. Their full manifests are indexed separately.
- Isolated extraction optimization probe paint SHA256:
  `848091e964cab2ea29eb07529c0770a7947252379e7c6b6cd0c868843abc1f6b`.
  Its extraction profile is not a whole-suite result. A later independent
  40-case packed-helper equivalence run and final native core suite passed on
  this unchanged source.
- Native C SHA256:
  `a39fc90d8373056bc98dc45f936e60cc5d2ef6639aea2417e0e35fde35be6705`;
  local Windows binary SHA256:
  `0674d769ef5f70bfd4c3ef054fefbe96778e74d0a375f32630a8c7a023451165`.
  Native real-call verification passed all seven APIs.
- Parent/peer-owned production and test edits remain uncommitted at this
  checkpoint. Both core commands explicitly exclude
  `test_quantized_curve_fit.py`, an independent unintegrated prototype.
  Neither suite validates that prototype. The final suite does cover the
  actual packed finite-face helper through the new equivalence tests.
- This session is prepared for tag `lane-mapping-20261011-r16-rc1`; its exact
  eventual commit is resolved by that immutable tag, not guessed here. Packed-helper equivalence
  and final-source native core regression are now verified over the stated
  scopes. ROS DDS, full runtime and driving acceptance are not implied.

## Work performed

R16 identity association preserves ordinary same-cell priority. A new
adjacent-cell association requires an independently resolved current-frame
same-cell neighbour seed, consistent repeated historical directed adjacency,
signed-normal compatibility, and strict overlap of the **same actual raw-face
pair in both old and current tangent charts**. Contact, gap or a union hull is
not overlap evidence. Cross-cell claims cannot seed one another, copy old
votes, erase branch edges or add localization support. All viable competing
claims—including exact nearest-distance ties—reserve their old records before
a unique association can be accepted. Record-ID order is preserved when a
committed fused mean changes its search bucket; preparation does not mutate
the published index.

Independent regressions cover ambiguity, seedless cycles, competing claims,
finite gaps, opposite normals, reliability ownership, TTL, capacity, stale
tokens, one-frame voting and committed index migration. Ordinary same-cell
nearest-mean behavior is deliberately unchanged and can still associate
disjoint observations within a cell; the new rule does not prove universal
physical identity.

The R15 DDS source audit isolated added finite-face-domain work as a concrete
performance candidate. The R16 performance repair packs only selected faces
in original row/member order before endpoint arithmetic, instead of computing
padded Q-by-K endpoint tensors. It retains connected-run checks, actual
segment clipping and finite-domain arithmetic; special signed-zero reductions
are covered by the new independent literal oracle. This is not a reason to enlarge runtime
budgets or waive DDS tests.

A separate saved-data design diagnostic examined repeatedly observed
within-record model segments missing from a graph made only of cross-record
mean-node chords. Its proposed fixed creation-chart domain remains a design,
not an integrated solver result in R16.

## Evidence and result

| Evidence | Verified result | Limit |
| --- | --- | --- |
| Identity focused tests | 85 passed: 40 identity + 45 legacy-domain equivalence, 1.32 s | Unit fixtures, not timing or physical identity proof |
| Initial core native suite | 2657 passed, 1 skipped; JUnit 2658, zero failures/errors, 80.837 s; 156 source hashes unchanged | Windows offline; Linux process-group test is the sole skip; independent unintegrated curve-fit test explicitly excluded |
| Packed finite-face helper equivalence | 40 passed, no failure/error/skip, JUnit 0.659 s; unchanged paint `848091…` | Literal old helper, finite-value bytes/invalid masks and all DDS-observation bytes; not universal equivalence, ROS or timing acceptance |
| Final native core suite | 2697 passed, 1 skipped; JUnit 2698, zero failures/errors, 71.391 s; process72.2914663 s; 157 source hashes unchanged | Same Linux-only skip and unintegrated-prototype exclusion; seven native APIs reverified; not ROS DDS or driving acceptance |
| Original R16 recorded candidate | 105 accepted / 106 attempts of 111 available frames; seq151–255 committed, seq256 rejected `match_insufficient_overlap` | Original 40/80 ms gates; not a full lap |
| Terminal reference comparison | Fresh overlap 0.5307820299500832; active 0.4209650582362729; engine 58.7387999985 ms | Both below original 0.55; engine timing excludes projection/extraction and delivery |
| Four fixed development cases | 0/4 passed; each 7 accepted / 8 attempted / 509 planned, frame7 overlap failure | Left 0/+0.35°: 0.4876033058; right 0/+0.35°: 0.5121951220; unchanged 2000 ms diagnostic budget, not runtime proof |
| Creation-chart COUNTERFACTUAL | Fresh covered mass 32 → 34.75 of D=60.5; overlap 0.5289256198 → 0.5743801653 | Pure fixed-prediction geometry competition, no new solve, rank/inlier admission, pose or accepted frame |
| Extraction profile01 → profile02 | First unprofiled call 297.3686000332 → 146.6981999693 ms; profiled finite-face helper approximately 165 → 27 ms | Separate single-fixture probes, not an isolated distribution or robust runtime/DDS acceptance |

The counterfactual uses immutable record-creation charts and only earlier
successful commits. All 103 queries compete against both the original graph
and 90 certified finite-interval records; graph wins ties. Six newly covered
queries add 2.75 mass, no previously covered query is lost. Each domain
requires 5 distinct frames plus 1 frame with both endpoints reliable.
Unassigned foreground mass remains in D. No future observation or truth was
fed into this analysis. A favorable counterfactual overlap cannot stand in for
successful localization or quality.

The extraction probes use the same synthetic DDS mask (1322 faces, 1308
model-valid). Each report verifies profiled versus unprofiled output bytes
within that source version. Those flags alone do **not** prove old-versus-new
whole-extractor equivalence. The later independent 40-case run compares the
packed helper with an embedded literal pre-packed body: random windows up to
512 faces, original row/member order, disconnections, selected/unselected
NaN/Inf, degenerate and extreme inputs, radius boundaries, signed-zero bounds,
strided/Fortran layouts and empty dimensions. Finite output bytes and invalid
masks match; input arrays remain unchanged. A literal DDS full-extractor case
compares every observation array byte and scalar and its immutable ownership.
This is ordinary non-trapping IEEE arithmetic: identical warning counts or
NaN payload bytes are not asserted by the helper-level tests.

The new test source SHA256 is
`20d7855cdc5ce98871a2e41af9a3b8976f054e206f0613949b052ddf99ef1184`;
JUnit SHA256 is
`8378d3447fd9a12663442417efb59e020dd5dc932d1f8022cafc51336bb5edf2`.
Read-only review found no threshold change or private-fixture/Git dependency
in the new test. The parent subsequently completed `r16_full_native_02`
with exit0 at `2026-10-11T10:21:10.833635+00:00`: 2697 passed / 1 skipped.
The terminal pytest line rounds elapsed time to71.40 s; JUnit records71.391 s,
and the process wrapper records72.2914663 s. The157 source hashes, paint848 and
test20d785 identities stayed unchanged. Native real calls again passed all
seven APIs on the same C/binary hashes. Both the initial and final reports are
retained; neither repairs the recorded/synthetic failures.

R15 publication readback at `2026-10-11T09:43:06.5051983Z` is **6/8 success**,
not green: native, Lane Foxy and handoff branch/tag runs passed; GNSS Foxy
branch `38128963174` and tag `38128963098` failed actual DDS smoke. Each GNSS
run had 2618 passing unit tests plus four header tests, but DDS passed only
7/9 branch cases and 3/9 tag cases. Failures waited unsuccessfully for five
stationary camera commits; many journal results were
`commit_guard_rejected`, followed by the original 5 s bootstrap latch.
Worker compute was roughly 365–559 ms, journal source age roughly 400–660 ms,
with the original 500 ms node freshness limit. The exact failed guard clause
was not logged, so freshness expiration is strongly supported, not established
for every rejection. The earlier R14 9/9 DDS comparison had worker medians
128–155 ms on different CI runners; it is not a controlled benchmark.

Timing must include receipt/queue, projection/extraction, engine, export and
publication/command delivery. Worker and engine durations measure different
portions; their difference is not an isolated extractor cost. Neither a
58.74 ms engine result nor a 146.70 ms local extraction probe proves successful
end-to-end runtime or repairs the prior DDS failures.

## Risks and blockers

- Four development cases and the original recorded candidate still fail under
  unchanged gates. No full teaching or repeat lap has passed.
- Initial core results do not cover later performance edits. The new 40-case
  packed-helper run and final2697/1 core run supply separate evidence for
  paint848. ROS DDS and end-to-end runtime revalidation remain pending;
  offline unit success cannot inherit or erase the R15 DDS failures.
- The association rule repairs a narrow identity defect, not all ambiguous
  physical identity or finite support loss.
- Fixed-chart support is model evidence, not raw-point ground truth. With
  immutable creation chart `(o0,t0)`, observed domain parameter
  `s=t0·(q-o0)` and current line `(p,t)`, use
  `F(s)=p+((s-t0·(p-o0))/(t0·t))*t`.
  This preserves the same world-chart parameter as the fitted line changes.
  Require the original **directed** angular gate, finite nonzero denominator
  and endpoints; do not reinterpret old limits relative to the current mean.
- Five versioned frozen fields (model segments, owner, frame votes, reliable
  votes, last stamp) are sufficient as a localization snapshot payload only
  when the internal creation chart, per-location certificate and record
  geometry commit together. They are not complete raw-history proof and do
  not assert every historical segment lay exactly on the current model line.
  Bias of that line remains an independent risk. Frozen repeat must not
  reconstruct or transport domains using later observations.
- Planner reviewed this text and the summarized evidence for secrets and privacy.
  Raw logs, images, absolute personal paths and coordinate-bearing captures remain local.
  Artifact hashes index retained originals; the package does not contain those originals.

## Next agent actions

1. Preserve the 40-case packed-helper result and its exact source identities;
   keep its finite-byte/invalid-mask and DDS-fixture scope explicit.
2. Preserve both completed core attempts and complete any remaining
   final-source adapter verification, then actual ROS DDS at unchanged
   source-age and bootstrap limits. Report queue-to-output
   timing separately from engine time; do not retry selectively for a green
   sample.
3. If implementing fixed-chart support, explicitly version geometry hashing,
   subset ownership, bundles and repeat identity. Add causal commit/rollback,
   gap/branch/opposite-side, stamp, saturation and capacity tests; never add
   fake graph nodes or delete denominator mass.
4. Re-run the prescribed quality and recorded-input candidates only after the
   source is fixed and CPU access is coordinated. Gate pass alone is not
   accuracy; truth remains offline scoring only.
5. Verify the immutable checkpoint/package and new CI result before selecting the
   next baseline. Sealed R15 records are not overwritten. No drive is authorized
   merely by receiving this development record.

Reproduction commands (repository root, package roots on PYTHONPATH):

```text
python -B -m pytest workspace/src/aiformula/control/lane_mapping_lya_reference/test workspace/src/aiformula/control/lane_mapping_fixed_gnss/test -q -ra -p no:cacheprovider --ignore=workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_quantized_curve_fit.py
python -B workspace/src/aiformula/control/lane_mapping_lya_reference/scripts/verify_native.py --source workspace/src/aiformula/control/lane_mapping_lya_reference/native/lane_graph_native.c --output <new-identity-json>
```

`LANE_REQUIRE_NATIVE=1`; local recorded-fixture path was supplied only for the
pre-existing optional native tests. No CI threshold, GNSS policy, car/scene,
physics, controller or camera parameter was edited. No new Unity trial was run.

