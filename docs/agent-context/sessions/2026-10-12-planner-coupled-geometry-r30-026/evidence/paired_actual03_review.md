# Independent actual03 evidence review

Status: post-execution, static-only review of the single recorded actual_run_03.
This review was not an execution prerequisite and does not authorize adoption.
The study is diagnostic evidence, not runtime, ROS/Foxy, vehicle, or two-lap acceptance.

## Outcome and blocking regression

The recorded neo-helper03 study is complete for the frozen 22 cases / 40 cuts /
80 primary roles / 1,528 center occurrences / 3,056 white/black occurrences.
It retains all four originally positive cuts and all 19 policy01 positive cuts.
It does **not** retain all 24 exact-policy02 positive cuts: 22 remain positive,
two become unknown, and no previously unknown policy02 cut becomes positive.

This effect regression blocks treating d367 as a no-effect-loss replacement for
exact policy02 or declaring the geometry repaired. It is not an input-integrity,
support-deletion, gate-relaxation, domain-expansion, or original-four loss finding.

The two regressions stop at proposal generation, before joint certification:

| Cut | Successful q in policy02 | Five neo03 side outcomes, q0 to q4 | Both-WB pairs / joint calls |
| --- | --- | --- | --- |
| longcap_phase_0.0 / 1 | q1 | Y/F; F/Y; Y/F; F/B; F/F | 0 / 0 |
| arc_0.6_phase_0.0 / 2 | q2, q3 | F/Y; F/Y; F/Y; F/Y; F/Y | 0 / 0 |

Y = a complete binary64 coefficient packet passed independent all-label checking.
F = restricted_float_pool_unknown. B = pivot_budget_unknown.
For longcap cut1 this is six F and one B failed sides; for arc cut2 five F.

The frozen source returns F when no enumerated restricted-pool basis passes its
finite-answer, nonnegative-multiplier, and strict floating pool-feasibility checks.
It returns B when a still-violated iterate reaches the unchanged 24-pivot cap.
These are bounded search/numerical unknowns, not proofs of infeasibility.
The persisted reports do not retain rejected KKT answers/multipliers, so this
review cannot identify which individual comparison rejected each basis or prove
that rounding alone caused a particular failure. Do not infer a global geometric
impossibility, relax a label comparison, increase the cap, or delete support.

## Version and receipt identities

Historical executed sources are the byte-identical frozen copies:

- source_revision_03/paired_curve_fit.py:
  d367d48e4b0e9afbd5c6fe7e18f349211ff7906dcc74b0a8d4706c2ea05a8679
- source_revision_03/footprint_geometry.py:
  eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17

Both frozen files were independently read in full and hashed. At the pre-edit
checkpoint all 104 bound current .py/.json files matched their historical pins.
After that checkpoint the root intentionally changed the repository fit source
to 597703b0a408b462807df57ece9e17787f322a167632ea44558c1765b3daa47c.
This later hash was independently observed, but its candidate source was not
reviewed or executed for this report. It must not be substituted into actual03,
and the post-run change is not evidence of a change during the recorded run.
The recorded 131 before/after bindings remain equal.

Primary evidence:

- run_actual_03.py:
  b16d41fd9aa0ac242e17eadd670d440126d67ae9fe20d43d619137ff3c608174
- actual_run_03/result.json:
  438df0101f9bdb5c51995976cc71ec5e2dc3b2970488cdc706fe764243450c3b
- actual_run_03/summary.json:
  b47a0567b9d313a58fd4d431ccd603dc5fc8e796b66b0b074e032a0df7cf3489
- actual_run_03/stdout.txt:
  6e36703d899552e36aaaaefdc0cb69348f6bb64a5b4b59677e4fce4109f369e1
- actual_run_03/stderr.txt:
  e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855

The parent result, child_started, start, initial inventory, and summary all bind
the same driver. Parent PID 17732 / child PID 12992 identities agree; the result
receipt authenticates the parent result hash. Parent reports exit0, no failure,
zero retries, own_child_stop_attempted=false, and own_child_stopped=true.

## Whole-JSON and source closure checks

PowerShell read-only parsing covered all 28 JSON files in each actual01/02/03
directory plus both units03 JSON files: 86 files, without opening NPZ or ZIP data.
All 75 parent-declared JSON/receipt digest relationships across the three
actual-policy folders matched. All 22 separate actual03 case files matched their
complete summary case records, not merely counts or abbreviated console text.

The parent-after, summary-before, summary-after, start-before, and initial-before
binding maps each equal parent-before. The 131 historical bindings are stable.
The policy02-to-03 relation is 124 unchanged shared bindings, one replaced
actual-driver entry, and seven added neo-study bindings; it is not a claim that
the old policy02 driver belongs to the new map unchanged.

At the earlier checkpoint, 104 current .py/.json bindings were hashed.
Two actual03 text logs were subsequently read and hashed as above.
The 25 NPZ/ZIP/other historical text-log entries were not independently opened or
hashed by this reviewer. Their historical before/after receipt equality is
checked, but their binary contents rely on the authenticated study's own loader.
Excluded prototype files were not read, hashed, imported, staged, or altered.

Initial inventory has 22 not_processed case skeletons / 40 not_processed cut
skeletons, not an empty case array. All named processing/counter/mutation fields
are initially zero; original P status and preserve-required flags match final
records. No initial candidate is represented as already accepted.

## Original support, domains, and labels

The full 80 original side packets in actual03 are semantically identical to
actual01 and actual02: 160 whole-packet comparisons. This includes primary owner
and incident-role identity, original selected-ID sequence, full support array
descriptors and byte digests, original model descriptors, saved model values,
label reports, and unchanged-original objective reports. There are no ribbon
fallbacks, missing-owner substitutions, shortened support sequences, reordered
or deduplicated occurrences, or model/chart normalization in these records.

Counts and descriptor shape checks independently yield 1,528 center occurrences
and 3,056 WB occurrences. All 320 support descriptors match the declared complete
support shapes; all 80 selected-ID sequence lengths match their occurrence count.
These comparisons authenticate serialized descriptors and sequence evidence,
not an independent reread of raw NPZ arrays or a rerun of nominal label algebra.

All 200 exact q slots have the same rational values, ordinal, and order as
policy02 and agree with the neo helper's unencoded original report.
Direct BigInteger rational substitution checked 800 original-P plane signs
and 400 original finite-chart membership predicates. Every one passed.
No epsilon, hull replacement, normalization, extent growth, clamp, or point
discard was applied by this check.

There are 800 original-cut field comparisons against the two older policies,
including original P, Q, P-intersect-Q, preserve-required flags, adoption flags,
gate/drop/map counters, eligible-primary identity, and declared radius search.
All match. Original status totals remain:

| Predicate | yes | no | unknown |
| --- | ---: | ---: | ---: |
| Original P | 4 | 31 | 5 |
| Original Q | 0 | 1 | 39 |
| Original P intersect Q | 0 | 1 | 39 |
| Original nominal all-WB side labels (80 roles) | 69 | 11 | 0 |

Unknown remains unknown. Original rounded-fit labels remain unverified because
fit-space WB chart inputs were not captured; nominal saved-world polynomial
labels must not be renamed original rounded-fit-program certification.

All four protected positives are L_corner_phase_0.0 cuts3/4 and
L_corner_phase_0.35 cuts3/4. They remain candidate-positive, and their complete
original model packets are unchanged. No candidate is adopted.

## Candidate reports and measured work

All 40 cut reports have complete five-slot coverage and complete counters.
All 400 helper-side reports carry actual nonnegative integer counters.
The five counters per side agree between raw helper and adapter reports:
2,000 equality checks. Side sums equal every cut and the final summary.
The report uses null/unknown rather than zero when coverage would be absent;
no absent-counter or partial exception report occurred in this run.

| Recorded scope | Count |
| --- | ---: |
| Complete helper side packets with yes labels | 284 |
| restricted_float_pool_unknown sides | 107 |
| pivot_budget_unknown sides | 9 |
| Complete paired all-WB hypotheses | 93 |
| Joint one-box calls | 279 |
| QP linear-system attempts | 1,461 |
| QP pivots | 416 |
| QP full scans | 709 |
| Adjacent rounding packet attempts | 7,668 |
| Exact nominal label substitutions | 383 |

All 93 paired hypotheses retain original origin/normal/scale/bounds for both
models: 744 unchanged chart-field comparisons. All 279 joint reports have
status=yes, independently_verified=true, and boxes_searched=1. Their three
declared radius fractions remain 1e-4, 1e-6, and 1e-8. The radius list and
24-pivot budget are unchanged, and are not new physical safety gates.

This reviewer read the complete certificate records and checked these report
relations; it did not rerun the Python certificate builder or verifier or
independently rebuild all 279 interval operators. The authenticated actual
driver performs both exact builder and independent-verifier checks.

Actual03 executes only the two neo sources listed above in a closed synthetic
package. Both logged raw-source executions returned successfully and their
hashes match before-bindings and frozen copies. It does not execute production
paired_curve_joint.py. Joint checks here still use the pinned D exact certificate
and independent verifier, as in policy02. The separate 30-method neo joint
checker receipt is not actual03 joint evidence.

Remaining 18 unknown cuts have zero complete paired-WB hypotheses and zero
joint calls. Among their failed sides there are 87 F and 8 B. Thus no remaining
cut is unknown due to a failed joint certificate or a failed 27-packet result;
the failure stage is the bounded proposal QP pool/cap. Successful other sides
may still have performed rounding attempts. Across all 400 sides there are no
rank, outside-chart, malformed-coverage, no_certified_27_packet, or independent
label-check failure reasons.

| Remaining unknown cut | F failed sides | B failed sides |
| --- | ---: | ---: |
| longcap_phase_0.0 / 0 | 5 | 0 |
| longcap_phase_0.0 / 1 (regression) | 6 | 1 |
| longcap_phase_0.0 / 2 | 5 | 0 |
| longcap_phase_0.35 / 1 | 5 | 0 |
| longcap_phase_0.35 / 2 | 5 | 0 |
| longcap_phase_0.35 / 3 | 8 | 0 |
| shortcap_1_phase_0.0 / 0 | 5 | 0 |
| shortcap_1_phase_0.0 / 1 | 4 | 1 |
| shortcap_1_phase_0.35 / 0 | 5 | 0 |
| shortcap_1_phase_0.35 / 1 | 4 | 1 |
| shortcap_5_phase_0.0 / 0 | 5 | 0 |
| shortcap_5_phase_0.0 / 1 | 3 | 2 |
| shortcap_5_phase_0.35 / 0 | 4 | 1 |
| shortcap_5_phase_0.35 / 1 | 3 | 2 |
| arc_0.6_phase_0.0 / 1 | 5 | 0 |
| arc_0.6_phase_0.0 / 2 (regression) | 5 | 0 |
| arc_0.6_phase_0.35 / 1 | 5 | 0 |
| arc_0.6_phase_0.35 / 2 | 5 | 0 |

## Policy comparison

| Frozen study | Candidate-positive cuts | Unknown cuts | Child diagnostic seconds | Parent diagnostic seconds |
| --- | ---: | ---: | ---: | ---: |
| Exact policy01 | 19 | 21 | 4.3699471 | 4.7557226 |
| Exact domain-aware policy02 | 24 | 16 | 8.1100438 | 8.5483045 |
| Neo float proposal03 + exact joint | 22 | 18 | 4.6478167 | 5.0890563 |

Relative to policy01, all 19 positive cuts remain; three formerly unknown cuts
become positive: longcap_phase_0.0 cut3, longcap_phase_0.35 cut0,
arc_0.6_phase_0.35 cut3. Relative to policy02, 22 positives remain, two regress,
16 unknowns remain, and zero new positives appear.

The 18 new positives beyond the original four are:
the two longcap cuts above; L_corner cuts0/1/2/5 at both phases;
narrow_U cuts0/1 at both phases; arc_0.6_phase_0.0 cuts0/3;
and arc_0.6_phase_0.35 cuts0/3.
Ten cases have no cuts and are not counted as geometric successes.

Wall times are one offline observed batch, not repeated performance estimates,
per-frame timing, a speedup certificate, or worst-case runtime guarantees.
The proposal algorithms and counters differ; lower systems/time must not hide
lost positives or be compared as identical work.

## Artificial-unit gate and limits

The units03 parent and child receipts were read in full:

- units_run_03/result.json:
  69612de92a2aceccebf949e6efe341048e70d40055891c7f34a4017668fa650b
- units_run_03/unit_result.json:
  c3ec9a6fb971a7d02cbe65c65eaf923448e32dbc4e674d78846e66c1c5d561cb
- run_units_03.py:
  70f5e83a954f078f590be95340d2b25a197de8433378a3c3d667343d6cfec302
- test_paired_curve_fit.py:
  15333c87ba50ace1f2f0ecd130bfdd75e26e266b252d600af3b84261179092be

All 58 unique executed test IDs match all 58 statically declared methods;
zero failures/errors/skips, successful/evidence-valid flags true.
Parent/child before/after maps and the child receipt digest agree. Four current
test/driver or historical frozen-source hashes match those maps.
The parent identifies child67248 stopped, exit0, zero retries; observed parent
1.4005806s / child receipt1.2783423s. This is artificial independent nominal
proposal testing, not actual packets or runtime. The existing65 and domain20
gates agree across start/initial/final gate records; their original receipts and
older review remain preserved rather than being rerun.

The actual study records mapping/extraction imports, extractor calls, native
calls, Unity calls, map writes, adoptions, gate changes, and dropped labels as
zero. No car, simulator, LYA speed, ROI, radius, or safety-gate parameter was
changed by this review. No ROS build, real-time watchdog, actual causal mapping
replacement, full runtime pipeline, closed-loop lap, trajectory error, or video
claim follows from these artifacts. The original 40ms/80ms/200ms budgets are
not waived by an offline batch result.

## Review mechanics and read-only failures

This reviewer used PowerShell UTF-8 full reads, JSON parsing, regex method
inventory, file hashes, and exact rational sign/membership comparisons only.
No Python, AST, import, test execution, solver, NPZ read, native code, Unity,
production edit, old-evidence rewrite, or root-session026 edit was performed.

Read-only review commands had recoverable formatting/schema mistakes:
a parser typo in a method-inventory expression; an attempted nonexistent
locations array / start_receipt key; and an overstrong empty-initial-cases
assertion. They were corrected using the observed schema and then passed.
These were review/readback failures, not extractor, unit, or actual-study
failures; no batch was rerun and no source/data was written by those commands.

Disposition: retain actual01/02/03 and all old receipts unchanged. Record the
two regressions explicitly. A new bounded numerical proposal generator may be
studied separately, with the original final all-WB and joint certificates kept
strict; its new source hash must get fresh static review and unit/actual
evidence before any adoption claim.
