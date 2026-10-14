# Independent actual04 and unit05 evidence review

Status: post-execution independent static/readback review. The provisional driver
was inspected while its new pins were pending, then its final changed gate and
whole-file relations were inspected after actual69 receipts arrived. Root ran
actual04 before this report was written. This report is not retrospectively an
execution prerequisite, not a rerun, and not authorization to adopt a model.

## Outcome

Every one of exact policy02's 24 positive cuts remains positive in actual04.
Every one of actual03's 22 positives and all four originally protected positives
also remains. Actual04 has 25 positive / 15 unknown cuts, not merely a larger
total with hidden regressions. No preservation or evidence-integrity blocker was
found for retaining this bounded diagnostic result.

The two actual03 regressions recover and one previously unknown cut becomes
positive. q ordinals are zero-based and are the same five exact original slots:

| Cut | Policy02 | Neo03 | Neo04 | Neo04 complete WB + verified joint q |
| --- | --- | --- | --- | --- |
| longcap_phase_0.0 / 1 | positive | unknown | positive | q1 |
| arc_0.6_phase_0.0 / 2 | positive | unknown | positive | q2, q3 |
| arc_0.6_phase_0.0 / 1 | unknown | unknown | positive | q2, q3 |

The last row is the **only** new positive relative to policy02. It is not
longcap cut2. Actual03's two losses remain recorded in its unmodified evidence;
actual04 must not rewrite the earlier 24-to-22 result.

Fifteen cuts remain unknown. This is not completed geometry repair, map
adoption, production/runtime readiness, ROS/Foxy build, scene verification,
two-lap success, trajectory-error evidence, or a video result.

## Frozen identities and driver differences

- run_actual_04.py:
  63b9dd53e2a2de042c633b2589a0e5896419738f74e89a3c4e8cf8601f4ec39f
- actual_run_04/result.json:
  169c875a04cf8c72fde429eb28cb77a5bbc9fe6c404727746f2aabc60e7b14ae
- actual_run_04/summary.json:
  a15029f1d69a500e978e9fff98455833ac7682c583db0531c062bb0c4a1939eb
- paired_curve_fit.py, executed current revision:
  597703b0a408b462807df57ece9e17787f322a167632ea44558c1765b3daa47c
- footprint_geometry.py, unchanged:
  eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17

The provisional driver observation was
f944ec37e7cb9373a522279096b7ad597dc9c659b7b2bd22fdc9bbef5554dbcb
(903 lines), with None new test/driver/receipt/count pins. Its neo gate would refuse
pending values in parent history before OUT.mkdir and child launch. Historical
authentication precedes this gate; pending rejection must not be described as
preventing every historical read/hash or as an observed executed rejection.

The final driver is 907 lines. Its changed gate was read in full. In-memory LF
normalization, replacing only the reviewed neo unit-pin/gate blocks, source
pin, study/output/path names, and corresponding comments, makes the complete
final driver exactly equal to frozen run_actual_03.py. No unlisted change to
support handling, charts/domains, final WB checking, joint builder/verifier,
candidate adoption, counters, or owned-child stop logic was found.
Nothing was written back to either driver.

The finalized gate explicitly requires 69 unique test IDs divided into exactly:

- 58 test_paired_curve_fit.PairedCurveProposalTests IDs;
- 11 r30_independent_paired_curve_roundoff_test_01.PairedCurveRoundoffTests IDs.

It uses the actual registry name, not the source basename as an invented test
prefix. It authenticates five before/after bindings and pinned actual parent/
child receipts. Missing/non-SHA pins, wrong group counts, duplicate IDs, skips,
failure/errors, incomplete child stop, binding changes, or absent evidence
refuse the new study. Old65/domain20 remain necessary independent gates and
cannot stand in for the new69.

The parent creates a new actual_run_04 directory with exist_ok=false; saved
outputs use exclusive creation. The child is one literal own Popen PID; primary
wait60s and one scoped stop wait5s, with stop_attempted set before kill/wait.
There is no all-Python/process-tree kill, second stop wait, retry, or concurrent
study worker. Preflight authentication and final hashing/serialization are
outside those wait bounds, so this is not a strict whole-program WCET proof.

## Generator-only source change

Current fit597 was read in full (348 lines) and hashed. Original frozen d367 is
preserved under source_revision_03. Exact text comparisons show these source
blocks are unchanged between d367 and597:

- _locations: all original P planes, all original finite chart slabs, five q;
- _labels: complete original nominal WB checking with exact uncertain fallback;
- _fit_at_q: same full supports and 27 adjacent coefficient packets;
- propose_pair: same limits, copied original charts, partial counter reports,
  no mutation/adoption/veto, and unknown rather than global no.

The new _definite_linear_violations encloses the supplied rounded two-term
linear residual using outward binary64 interval primitives and chooses a pivot
only when the lower residual endpoint is positive. Nonfinite or inverted
enclosures raise an unknown path. This is a proposal-generation decision,
not a declaration that uncertain rows are feasible. The unchanged final all-WB
predicate and separate actual-rounded joint proof still decide certification.
It is not an epsilon, clipped support, expanded chart, or relaxed final gate.

The new11 artificial fixtures use their own Fraction linear/world-label
substitution. They explicitly exhibit positive one-ULP/subnormal residuals that
may be uncertain for the generator while not exact-feasible, and verify that
the final original WB/27-packet checks still reject invalid packets.
No optimality, original rounded-fit-program feasibility, or runtime claim is
derived from the generator or fixtures.

## Unit05 actual receipt checks

- run_units_05.py:
  99667a5b2556bd849102e598173fa83637fe0c07bf4976e857e0f6d8d6f45b5f
- test_paired_curve_fit.py, unchanged58:
  15333c87ba50ace1f2f0ecd130bfdd75e26e266b252d600af3b84261179092be
- test_paired_curve_roundoff.py, new11:
  2abc09acc54128b1e561f603809278a5005107fafb0379a7669152eb6697749d
- units_run_05/result.json:
  c289239be6b1d12ce89aca15ddf7dab0f8a545d7301ea1b6f53b74c4b0c62e36
- units_run_05/unit_result.json:
  fcbb58273e3b475342e8ffa05db61bcfc00d2f390c2c81f57ed650207e067c23

Both complete receipts, the complete unit05 driver, and the complete new11 test
source were read. All 69 unique actual IDs match all 69 statically declared test
methods, with exact58/11 class distributions and zero failure/error/skip.
All five current hashes agree; parent and child before/after maps agree and
the parent authenticates the child digest. The actual04 gate source map has a
different JSON key order from the unit receipts, but exactly the same five
name-to-hash entries; semantic map equality was checked entry by entry.

Unit parent reports exit0, failure=null, retries0, own child15752 stopped;
observed parent1.4699218s / child receipt1.3435737s. Its closed package executes
only fit/footprint authenticated bytes, excludes production __init__/mapping/
extraction, checks source module and test subject identities, and records zero
map/native/Unity actions. The tests contain AST checks executed by root's unit
batch; this reviewer only read that source and did not execute AST or Python.
Artificial69 success is not ROS/Foxy build or actual/runtime acceptance.

## Whole actual evidence, provenance and domains

PowerShell parsed all 28 JSON files in each actual01/02/03/04 folder plus both
units05 files: 114 whole JSON files. All 100 parent-declared JSON/receipt digest
relationships across the four study folders match. All 22 actual04 case files
equal their full summary records. Parent-after, summary-before/after, start-before
and initial-before maps each equal the parent-before map: 132 stable bindings.

At this review's current-source checkpoint all 105 bound .py/.json hashes match.
Two actual04 text logs were additionally read and hashed:

- stdout.txt:
  6e36703d899552e36aaaaefdc0cb69348f6bb64a5b4b59677e4fce4109f369e1
- stderr.txt, empty:
  e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855

The other25 NPZ/ZIP/historical-log entries were not independently opened or
hashed by this reviewer. Their historical receipt-map equality is checked, but
raw binary payload validation relies on the authenticated study loader.
No excluded prototype pair was read, hashed, imported, staged, or altered.
Any later intentional source change must be kept separate from these132
recorded run-time before/after identities.

All 80 complete original side records equal each of actual01/02/03: 240 whole
packet comparisons. They retain actual primary owner/incident role/seed identity,
selected support-ID original order, every occurrence (including repeats), all
support descriptor shapes/digests, original chart packet descriptors/values,
label reports, and unchanged-original objective reports. All320 support
descriptor shapes and all80 sequence lengths match their declared full counts.

There are 1,528 center and 3,056 WB occurrences, not sampled or deduplicated
supports. These are serialized descriptor/sequence checks, not a reviewer
reread of NPZ payload bytes or rerun of every nominal label polynomial.

All200 exact q values/order agree with policy02, policy03, and raw neo reports.
Independent exact BigInteger rational substitution checked all800 original P
plane signs and all400 original chart-membership predicates; all passed.
The 1,200 protected cut-field comparisons against01/02/03 all match, including
original P/Q/P-intersect-Q, preserve/adoption flags, eligible primary identity,
drop/map/gate counters, and unchanged radius search. Original totals remain:
P4yes/31no/5unknown; Q0yes/1no/39unknown; P-intersect-Q0yes/1no/39unknown;
original nominal WB side labels69yes/11no/0unknown.

No unknown is changed to no, missing primary to ribbon, hull/extent repair,
normal normalization, domain growth, point deletion, label-margin epsilon,
3m ROI, or parameter override is used to obtain these findings. Nominal WB
polynomial-extension certification is not captured original rounded-fit-space
program certification; those original rounded fit labels remain unknown.

Initial inventory contains22/40 not_processed skeletons, named counters zero,
and original P/preserve flags matching final records. Start/child/parent/initial/
summary driver identities agree. Parent66024 and own child34096 identities
agree. Parent reports exit0, failure=null, retries0, stopped=true,
stop_attempted=false, and structured_actual_evidence_valid=true.

## Actual counters and complete certificates

Every cut has complete planned five-q coverage and counters. All400 side
diagnostic reports have actual nonnegative integer counters, and all2,000
adapter/raw-helper counter equalities, per-cut sums, and summary sums agree.

| Recorded scope | Count |
| --- | ---: |
| Complete side label packets yes | 293 |
| restricted_float_pool_unknown sides | 104 |
| no_certified_27_packet sides | 3 |
| pivot_budget_unknown sides | 0 |
| Complete paired WB hypotheses | 102 |
| Joint one-box calls, all yes and independently_verified | 306 |
| QP systems / pivots / full scans | 2,315 / 434 / 730 |
| Adjacent rounding packets / exact label substitutions | 7,992 / 848 |

For all102 paired hypotheses, both original chart origins/normals/scales/bounds
are unchanged: 816 field comparisons. Every joint report has boxes_searched=1,
status=yes, and independently_verified=true. The prescribed radius fractions
remain1e-4/1e-6/1e-8 and the per-side pivot cap remains24.

All three no_certified_27_packet reports remain unknown after27 attempts:

| Case/cut/q/side | Another q certifies this cut |
| --- | --- |
| longcap_phase_0.0 / cut3 / q0 / left | yes |
| arc_0.6_phase_0.0 / cut3 / q1 / right | yes |
| arc_0.6_phase_0.35 / cut3 / q2 / right | yes |

They each record27 exact label substitutions and are not silently promoted.
Their cuts are positive only through other fully certified q hypotheses.

The reviewer checked complete serialized reports and their identities/counts,
not a rerun/rebuild of all306 exact interval operators. The authenticated driver
calls both the pinned exact D certificate builder and independent verifier.
Actual04 does not execute production paired_curve_joint.py; separate nominal
joint unit receipts are not substituted for actual04 joint evidence.
Exactly two raw neo source execution records returned, with correct fit597/
footprinteb03 hashes; mapping/extraction/production-init remain excluded.

## Remaining15 unknown and preservation comparison

Each remaining unknown has zero paired full-WB hypotheses and zero joint calls.
All78 failed sides on these cuts are restricted_float_pool_unknown, not
finite-domain exclusions, rank failures, joint failures, 27-packet failures,
missing support, or pivot-budget failures.

| Remaining cut | Failed pool sides |
| --- | ---: |
| longcap_phase_0.0 / 0 | 5 |
| longcap_phase_0.0 / 2 | 5 |
| longcap_phase_0.35 / 1 | 5 |
| longcap_phase_0.35 / 2 | 5 |
| longcap_phase_0.35 / 3 | 8 |
| shortcap_1_phase_0.0 / 0 | 5 |
| shortcap_1_phase_0.0 / 1 | 5 |
| shortcap_1_phase_0.35 / 0 | 5 |
| shortcap_1_phase_0.35 / 1 | 5 |
| shortcap_5_phase_0.0 / 0 | 5 |
| shortcap_5_phase_0.0 / 1 | 5 |
| shortcap_5_phase_0.35 / 0 | 5 |
| shortcap_5_phase_0.35 / 1 | 5 |
| arc_0.6_phase_0.35 / 1 | 5 |
| arc_0.6_phase_0.35 / 2 | 5 |

Restricted-pool failure is still an unknown bounded numerical/search outcome,
not global infeasibility. Rejected KKT candidates/multipliers are not persisted
in full, so exact per-basis causes cannot be uniquely reconstructed from these
receipts. No claim that all failures are solely floating equality roundoff.

| Baseline | Positive cuts retained by04 | Unknown to positive | Positive to unknown |
| --- | ---: | ---: | ---: |
| policy01 (19 positive) | 19 | 6 | 0 |
| policy02 (24 positive) | 24 | 1 | 0 |
| neo03 (22 positive) | 22 | 3 | 0 |

The original protected four remain L_corner cuts3/4 at phases0.0/0.35 with
complete original packets unchanged. Ten no-cut cases are not geometric success.
Actual04 is25=4protected+21additional positive cuts;15 remain unknown.

Observed child4.891625s / parent5.3329389s is one offline diagnostic batch.
Compared with neo03's4.6478167s /5.0890563s and policy02's8.1100438s /8.5483045s,
neither identical work, statistical speedup, per-frame latency, nor strict WCET
is established. Existing40ms/80ms/200ms runtime budgets remain untouched.

## Boundaries and readback limitations

This worker used only PowerShell read/hash/JSON/regex/exact-sign operations and
new D-only report writes. It did not execute Python/AST/tests/NPZ/solver/native/
Unity or edit R, old artifacts, old tests, old reports, or root-session026.

A first in-memory driver normalization overlapped root's live gate update and
returned unequal; rereading final63b9 and normalizing only reviewed gate blocks
passed. A first unit-gate map comparison incorrectly treated JSON key order as
identity; entry-by-entry semantic comparison passed. These were review/readback
issues, not actual04 or unit05 failures; no study/test batch was rerun.

The study reports map writes/adoptions/gate changes/dropped occurrences and
mapping/extractor/native/Unity calls zero. It does not enable the helper in
extraction or change car/simulator/LYA speed parameters. Remaining runtime
integration, root/angle/J/ownership gate coverage, causal end-to-end latency,
new-scene first-lap mapping/second-lap fixed trajectory, error metrics, logs and
video are still separate acceptance work. Do not call this a scene or two-lap
pass, nor silently discard the remaining unknown geometry.
