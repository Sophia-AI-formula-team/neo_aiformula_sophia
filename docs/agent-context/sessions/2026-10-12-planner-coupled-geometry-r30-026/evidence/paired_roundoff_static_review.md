# R30 generator-only roundoff change: independent static review

Prepared 2026-10-12 by `indexed_bounds_review`. This is a source preparation and
static review, not a test, solver, replay, runtime or Unity result. The reviewer
ran no Python, AST evaluation, import, native code, tests, profiler, NPZ load or
Unity operation. Only explicit allowed text reads, PowerShell literal comparison
and SHA-256 checks were used. The excluded prototype pair was not read, hashed,
imported, tested or edited. No car or simulator parameter was changed.

## Frozen scope and source identity

`R` denotes the canonical neo staging checkout; `L` its
`workspace/src/aiformula/control/lane_mapping_lya_reference` package. `V` denotes
the existing `r30_joint_curve_fit_01` diagnostic directory. Paths below are
relative to these named roots and disclose no private workstation path.

| File | SHA-256 | Role |
| --- | --- | --- |
| `L/lane_mapping_lya_reference/paired_curve_fit.py` | `597703b0a408b462807df57ece9e17787f322a167632ea44558c1765b3daa47c` | New root-owned proposal helper, read completely, not edited by reviewer |
| `V/source_revision_03/paired_curve_fit.py` | `d367d48e4b0e9afbd5c6fe7e18f349211ff7906dcc74b0a8d4706c2ea05a8679` | Frozen previous helper, read completely |
| `L/lane_mapping_lya_reference/footprint_geometry.py` | `eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17` | Existing outward arithmetic primitives; no change |
| `L/test/test_paired_curve_fit.py` | `15333c87ba50ace1f2f0ecd130bfdd75e26e266b252d600af3b84261179092be` | Existing 58 methods in `PairedCurveProposalTests`; unchanged |
| `L/test/test_paired_curve_roundoff.py` | `2abc09acc54128b1e561f603809278a5005107fafb0379a7669152eb6697749d` | New independent 11-method test, 15,157 bytes; prepared and read back only |

The original `REVIEW.md` and all old test/results/receipts remain untouched. The
new helper, original test and footprint pins were identical before and after
this reviewer's new test preparation. The root owns any subsequent execution,
integration and publication. This review does not authorize an execution lease.

## Exact change boundary, not a sampled guard comparison

PowerShell literal reverse-change comparison found exactly one new
`_definite_linear_violations` definition, exactly one new full-scan call and
exactly one new pool call. Removing that definition and replacing those two
expressions with their original expressions reconstructs the ENTIRE frozen
`d367` source string exactly. No newline normalization or selective mathematical
rewrite was needed for that complete comparison.

The separate function-string check also found `_array`, `_number`, `_pair`,
`_chart_exact`, `_inside`, `_locations`, `_chart_float`, `_chart_interval`,
`_labels`, `_pair3`, `_fit_at_q` and `propose_pair` unchanged. The class/constants,
imports, remaining restricted-generator algebra and control flow are covered by
the whole-file reverse comparison, not merely inferred from those function
checks.

The two intentional changes are:

- Full scan: `rows @ current > limits` becomes definite positive lower bounds
  from an outward enclosure of `row[0]*beta[0]+row[1]*beta[1]-limit`.
- Pool check: the same enclosure replaces the old raw dot-product comparison
  for the pool's at-most-three original row IDs.

The helper still uses the same eigen/rank refusal, unchanged nonnegative raw
multiplier check, basis ordering, initial solve, first violated occurrence,
24-pivot cap and diagnostic counters. A nonfinite or inverted enclosure raises
`ValueError`, rather than silently returning no violations. The public exception
handler and already-appended partial diagnostics are unchanged.

No observation or label is rounded again to manufacture an easier final gate.
No residual epsilon, changed support selection, deduplication, chart bound,
half-face domain, gate threshold or final predicate has been added. The original
five exact initializer slots, finite-chart intersection, 27 adjacent beta64
packets and all-WB interval/Fraction verification are byte-identical to `d367`.

## Soundness conclusion and its necessary boundary

Within the footprint primitives' stated ordinary IEEE binary64 assumptions,
each scalar multiply/add/subtract is rounded outward. The new helper's strictly
positive lower bound therefore implies a positive exact residual of the
SUPPLIED already-rounded float64 linear rows and limits. It preserves original
row order and multiplicity via `flatnonzero`; no row is declared definitely
violated merely because the rounded dot product happened to be positive.

The converse is intentionally false. An empty returned index array does NOT
certify feasibility, KKT optimality or even nonpositive exact linear residuals.
Three explicit independently hand-derived cases in the new test demonstrate
positive exact residuals that remain uncertain:

- `1 - nextafter(1,-infinity) = 2^-53`;
- `(2^53)*1 + 1*1 - 2^53 = 1`, whose addition loses a unit when rounded;
- the smallest positive binary64 times one, with exact residual `2^-1074`.

Such a result is allowed only because this is a proposal generator. The model's
final unchanged `_labels` predicate must check the SAME actual rounded
coefficient packet against EVERY original white/black occurrence, and the
caller must separately verify the actual rounded joint, original domain,
root/angle/J/ownership and all existing downstream admission gates. A partial
or unknown result may not be promoted to a complete pair. The numeric helper
still does not authenticate primary producers or prove a shared rounded zero.

The final WB predicate covers nominal polynomial-extension signs of saved
binary64 world points, including off-extent points and duplicates. It is not
equivalence to the old rounded fit program, a physical lane-width proof or a
proof that the extension hypothesis is correct for the original geometry. The
new linear enclosure likewise refers to already-rounded linear rows, not the
original FP Gram/rhs, exact unrounded chart expansion or physical geometry.

Under those explicit conditions I found no new false-admission path in this
two-touchpoint proposal change. Using this helper's empty pivot list as a final
feasibility certificate WOULD be a soundness bug; the new test deliberately
shows that misuse is false. This static conclusion does not certify an as-yet
unreviewed extractor integration or another caller that bypasses the final
labels/joint/source gates.

The new enclosure is not an API validator for arbitrary private-helper shapes.
Actual public callers already supply finite float64 `N x 2` rows, `N` limits and
two coefficients, with `N <= 1024`. Empty private row arrays work as vacuous
generator scans; malformed direct private calls are outside that validated
public contract. There is no newly relaxed public shape/role/count contract.

Overflow is conservative but can lose proposals: even a finite exact final
residual, e.g. `maxfloat*2 + maxfloat*(-2) = 0`, can have overflowing
intermediates, and the helper must raise unknown rather than call it feasible.
The public result remains `unknown` with retained prefix/counters. It is not a
global infeasibility certificate, root veto, point deletion or evidence that
no alternate five-location proposal exists. A completed diagnostic may still
contain only `None` coefficient packets; its header is not map admission.

## Independent prepared tests and portable loader contract

The new test imports the subject through its normal package name. It contains
one class, `PairedCurveRoundoffTests`, and 11 unique `test_` methods. Its direct
oracle uses its own `Fraction.from_float` substitutions for linear residuals
and saved-world WB signs; it does not use the subject's chart, label or domain
helpers to manufacture expected mathematical signs. It does not import the old
test, diagnostic solver, joint helper, collector, native library or prototype.

Prepared coverage:

1. Hand dyadic exact residuals and ordered definite-violation IDs.
2. Closed exact equalities, signed zero and empty rows.
3. Positive one-ULP, large cancellation and subnormal residuals that remain
   uncertain, explicitly NOT feasible.
4. Clearly positive/negative residuals at power-of-two scales `2^-500`, `1`,
   `2^500`, with no fixed absolute epsilon.
5. Read-only strided input arrays, repeated original rows, preserved IDs,
   dtype/shape/strides/bytes/writeability and underlying storage.
6. Product/add/subtraction overflow and NaN/infinity refusal, including an
   exactly zero but intermediate-overflowing nominal residual. Refusal is also
   checked under the caller's `np.errstate(all='raise')`.
7. An SPD zero-pivot generator that returns an uncertain, exactly infeasible
   vector, followed by a one-ULP black-label violation that the unchanged final
   all-original-WB predicate rejects through exact substitution. Six repeated
   off-extent white/black occurrences are retained.
8. An intentionally unsafe mocked generator output: all 27 actual neighboring
   packets are independently proven to violate the last original off-extent
   black label. Each packet must invoke the real unchanged full-label predicate
   with byte-identical six-occurrence arrays, and no packet may be accepted.
   This is a guard fixture, NOT evidence that the actual generator emits that
   unsafe vector for that center objective.
9. A clearly definite violation with zero allowed pivots still returns
   `pivot_budget_unknown`, with actual work counters retained.
10. A mocked new enclosure error exercises the real public exception path:
    partial first case only, no false both-complete flag, next side not started,
    zero writes/drops/gate/domain changes and no joint/global-no claim.
11. A prepared AST call-site guard binds the new helper ONLY to the two
    restricted-generator sites; constants remain `512/24/16384`, and both
    subject/test avoid the unavailable Python-3.8 `math.nextafter` API.

The AST guard is source in the test file for the root's future execution. This
reviewer did not parse/execute AST or run that test. PowerShell line matching
counted the 11 unique methods. Existing 58 tests and all their assertions remain
byte-identical, with no skip, fallback, xfail or oracle relaxation introduced.
The intended future combined suite is 58 + 11 = 69, NOT a present pass count.

For the root's verified-source loader, use a fresh registered test module name
such as `r30_independent_paired_curve_roundoff_test_01`, a real `__file__`, and
`sys.modules[name] is module` BEFORE execution, because `inspect.getmodule`
needs that normal registry. As with the old test, use verified raw bytes and
the supplied test-source marker `__independent_test_source_sha256__`; do not
re-read an unverified `.pyc` or create a second subject instance. The existing
package/footprint/fit subject identities remain the same registered modules.

## Operational caps, limitations and next evidence gate

This change does not increase the explicit per-side maxima: at most 24 pivots,
25 full scans, one initial system plus at most seven basis attempts per pivot
(169 systems), pool size at most three, and 27 rounded packets. With five slots
and two sides those bounds repeat for up to ten side fits per pair. Every final
packet still checks up to 512 white and 512 black occurrences; exact fallback
work and Fraction arithmetic are not free.

The new predicate adds two endpoint-product stacks and outward elementwise
add/subtract scans. It runs once per full scan plus on each numerically eligible
pool proposal, up to seven pool checks per pivot. The `full_scans` counter still
does not count those small pool scans; it is not a total work or wall-time
counter. Per-pair caps do not bound total cuts per frame, queue delay, projector,
consensus, engine, matcher, commit, DDS or whole runtime chain. No strict WCET,
80-ms engine repair or vehicle-safe runtime claim follows from these caps.

Fewer spurious pool/pivot failures could improve proposal completion, but each
scan costs more than the old raw 2D dot product and newly surviving generators
may incur all 27 WB checks and a separate joint certificate. Thus either total
cost or output counts may increase, decrease or shift. Do not promise speed or
stable basis/pivot/counter output from this static review. The old 58 hand-dyadic
assertions should be retained exactly and measured in a fresh combined gate,
not weakened to preserve an expected count.

The root reported an old `d367` single 58-test pass and old actual-03 completion
of 22 cases / 40 cuts / 80 models / 1,528 supports / 3,056 labels, with two
proposal regressions versus the exact diagnostic. Those are root-reported
historical outcomes, not independently re-executed or relabeled by this
reviewer. They do NOT validate source `5977`. The old actual-03 artifacts must
remain frozen rather than rerun or described as this new change's outcome.

The next evidence gate belongs to the root: authenticate this new source/test,
run a single bounded 69-method combined suite with unchanged before/after
bindings, preserve failure/prefix evidence, then separately audit the same
original 22 packets under a newly approved lease. Only those new results can
establish whether the named regressions improve and what additional work is
incurred. Any later production integration additionally needs independent
owner/support/cut/joint/gate and full-output/runtime evidence. No geometric
repair, one-lap mapping, second-lap fixed driving or demo success is claimed here.
