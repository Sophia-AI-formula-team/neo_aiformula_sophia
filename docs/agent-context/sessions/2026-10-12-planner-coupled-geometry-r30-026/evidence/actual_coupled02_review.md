# Actual policy02 evidence review (independent read-only review)

This is a post-execution evidence review, not retroactive pre-run permission.
The reviewer performed only PowerShell UTF-8 JSON/source reads, rg, SHA256,
JSON comparisons and direct exact rational substitution using built-in
System.Numerics.BigInteger. No Python, AST/compile/import, solver, certificate
implementation, tests, extractor/native code, NPZ payload, Unity, encoding,
production edit or original-artifact mutation was run by this reviewer.

## Verdict

No blocking evidence inconsistency found within the frozen D-only diagnostic
contract. Policy02 yields 24/40 candidate-certifiable cuts (4 originally
certifiable cuts preserved plus 20 additional candidates), compared with
policy01 19/40 (4 + 15). All 19 policy01 positives remain positive; exactly five
previous UNKNOWN cuts become positive. Sixteen remain UNKNOWN.

This is not 24 successfully mapped/accepted vehicle corners, a runtime adoption,
two completed laps, physical map accuracy, localization accuracy or trajectory
tracking success. Neither the recorded root certificate nor the review creates
a map observation or authorizes rejection of an original accepted model.

## Read evidence and source identity

- actual_run_02/result.json SHA256:
  `c0a155db1db47a77656d9b0c45a21df8928f731b51cb36c7577d941ab5125a05`.
- actual_run_02/summary.json SHA256:
  `99a71527b23aa5ac1ee9cd9e743ff28a4eba5f76c83a34b294f457704ac0f8b0`.
- run_actual_02.py driver:
  `a7a4356bc4e1c8eec49c907b92ab8c4eee128967f98088d750037a0631cc88a2`.
- Frozen policy01 result.json:
  `cd379e4bde33e6062a0ff555d1b349cbef7db451361ce1a0b440d130070a47ea`.
- Frozen policy01 summary.json:
  `5ab2e1af0ba1c2fbd086af798828403ae0e3245b3dd29eebfc7085ef762c768e`.

All 28 JSON files in actual_run_02 and all 28 in actual_run_01 were read fully
and parsed; the two unit02 JSON receipts were also read/parsed (58 in that
comparison batch). Each of the 22 case files semantically equals its complete
case record embedded in summary02. Each parent result's declared 24 output-JSON
digests plus its linked result_receipt were checked in both policies: 50
current JSON/receipt hash relationships matched. These are current readbacks
of the frozen records, not a second actual study.

Additionally, all 22 original R29 consumer audit JSON files were read/parsed.
The 120 whole original P/Q/P-intersect-Q reports in policy02 equal those
original audit reports, not merely their status flags.

Policy02's 125 before/after binding entries agree between parent result and
child summary, with identical keys/digests. Current contents of all 98 .py/.json
bound files match those bindings. No excluded prototype filename occurs.
The remaining 27 NPZ/zip/text-log bound files were intentionally NOT read or
hashed by this reviewer; their before/after equality is recorded evidence from
root's executed driver, not an independent payload check here.

The exact relationship to policy01 bindings is 118 shared unchanged entries,
one replaced driver key, and seven added policy02 keys. The missing old-driver
key is run_actual_01.py, not changed source: its current SHA256 remains
`a23bddf2033355af2ef647040fba02e0c98de360af6c88e85638380cf405399e`.
The seven new keys are the new actual driver, two policy sources, test source,
unit driver and two actual unit receipts. Thus “125 matched” must not be
restated as all 119 old keys being a subset. Old policy01 output JSON remains
consistent with its own linked parent receipt; those output files are not
themselves entries in the new 125-source/input set.

The new unit02 actual receipts contain 20 executed test IDs, no failure/error/
skip, success and matching source-before/source-after pins. Their hashes match
the actual02 nested unit gate:
parent `df199e1a0f64075231f8fbfc2c5158eed3638d2cd33e465b06a201ea8a9e1b5c`,
child `f68d5767277ec945cb59b50b980f1638e23013b70b6c8cd7de729257dc806eca`.
The unit02 driver is
`b9b8b5ef9a2b545ba727ea76021e2084171c19cc3c139184b5c3481dd3a17d74`.
This confirms root's recorded execution, not an execution by this reviewer.
The previous prepared test/review files retain their historical wording.

## Inventory and original-source protection

Recomputed totals from every cut/side, not copied only from top-level counters:

| Quantity | Policy01 | Policy02 |
| --- | ---: | ---: |
| Processed cases | 22 | 22 |
| Cuts / paired primary roles | 40 / 80 | 40 / 80 |
| Center occurrences | 1528 | 1528 |
| White/black occurrences | 3056 | 3056 |
| Latent q hypotheses / q-side chart cases | 200 / 400 | 200 / 400 |
| q outside original P | 0 | 0 |
| q-side outside an original finite chart | 207 | 0 |
| Candidate-certifiable cuts | 19 | 24 |
| Originally certifiable cuts retained | 4 | 4 |
| Additional candidate-certifiable cuts | 15 | 20 |
| UNKNOWN cuts | 21 | 16 |

These are occurrences per incident-model role; reuse of the same source model
or support across cuts does not make them distinct independent observations.

All 80 complete original side packets are semantically identical to policy01,
including owner/seed role, original selected support-ID sequence, all four
full-support descriptors, all original model descriptors/model values,
original label reports and original zero-change center-objective diagnostics.
Counts and descriptor shapes agree for all 320 support descriptors. No order
change, deduplication, ribbon substitution, truncation, dropped occurrence or
new extent occurs. Original chart packet retention is recorded for all roles.

Ten protected original cut fields are identical to policy01 at every cut
(400 comparisons), including full P/Q/PQ reports, preserve flags, adoption/map/
gate/drop flags, eligible-primary flags and unchanged radius search. The
full-domain results still are:

- Original P: 4 YES, 31 NO, 5 UNKNOWN.
- Original Q: 0 YES, 1 NO, 39 UNKNOWN.
- Original P intersect Q: 0 YES, 1 NO, 39 UNKNOWN.
- Original nominal all-WB models: 69 YES, 11 NO (80 roles).

The candidate certificate's domain is original P plus the original finite
charts. No Q/PQ certificate is replaced by a candidate P certificate; candidate
positivity does NOT imply Q/PQ positivity. Their original negative/unknown
reports remain untouched.

The four original-preserve cuts are L_corner_phase_0.0 ordinals 3/4 and
L_corner_phase_0.35 ordinals 3/4. Each original P and both original label reports
are YES, and each still has a candidate positive diagnostic. All original
side/chart descriptors stay identical. candidate_adoption_permitted is false
on every cut; map_writes/adoptions/gate_changes/dropped occurrences are zero.

## Independent exact q-domain substitution

No policy helper or solver was called. Fraction strings were parsed as exact
BigInteger numerator/positive-denominator pairs. Direct addition,
multiplication, subtraction and division were evaluated without floating
tolerances, and rational signs were checked.

For each of the 200 saved exact q hypotheses:

1. The q index and exact q value equal the corresponding ordered saved location.
2. The location's four original planes equal the unchanged original P context.
3. All four original-P inequalities hold: 800 exact sign checks.
4. All four saved finite-chart inequalities hold: 800 exact sign checks.
5. Independently, x=(ny*(qx-ox)-nx*(qy-oy))/scale is computed from each ORIGINAL
   model and tested against its unchanged closed bounds: 400 exact chart checks.

All checks succeed. Thus the improvement from 207/400 off-chart q-side cases
in policy01 to 0/400 in policy02 is verified using the unchanged original
chart values, not by growing chart bounds or adding q locations.

A feasible latent q is still NOT a root of the rounded polynomials. The actual
driver separately records both full-WB tests and a rounded-model root proof.
Of the 200 hypotheses, 102 have two complete label-positive beta64 packets.
For those 204 models, every origin/normal/scale/bounds field equals its original
counterpart (816 checks). All direct base and independent fixed full-label
reports are YES with the complete original WB count. No in-extent-only filter
is used.

Those 102 hypotheses generate exactly 306 one-box reports (the same three
prescribed radii per hypothesis). All 306 record YES, one box searched and
independently_verified=true; no certificate implementation was rerun here.
The latent-q equality-after-rounding field remains explicitly unverified in
the side fit report; the separate existence proof is what supplies joint
positivity. This review corroborates identity/domain/report consistency, not
an independent recomputation of all interval/Krawczyk algebra.

## Cut-by-cut comparison and remaining UNKNOWN causes

All 19 policy01 positives remain positive. Exactly these five convert
UNKNOWN -> candidate-certifiable:

| Case | Cut ordinal | Certifiable q ordinals (zero-based) |
| --- | ---: | --- |
| longcap_phase_0.0 | 1 | 1 |
| longcap_phase_0.0 | 3 | 1 |
| longcap_phase_0.35 | 0 | 2 |
| arc_0.6_phase_0.0 | 2 | 2, 3 |
| arc_0.6_phase_0.35 | 3 | 2, 3 |

The 16 remaining cuts have 80 q hypotheses and ZERO joint calls/box reports.
All are unresolved before two complete beta64 all-WB packets are available;
none currently fails due to an out-of-chart q, a failed Krawczyk box, timeout,
missing owner, domain shape repair or label deletion.

Here P means exact reason no_certified_restricted_pool (search UNKNOWN, NOT a
certificate of true global infeasibility). R27 means
no_certified_packet_in_27_ULP_candidates (real QP solution was obtained, but
none of the prescribed 27 adjacent binary64 packets passed all labels).

| Remaining case | Cut | Exact failed-side/q pattern |
| --- | ---: | --- |
| longcap_phase_0.0 | 0 | left P at q0-4 |
| longcap_phase_0.0 | 2 | left P at q0-4 |
| longcap_phase_0.35 | 1 | left P at q0-4 |
| longcap_phase_0.35 | 2 | left P at q0-4 |
| longcap_phase_0.35 | 3 | left P at q0-4; right P additionally at q0, q1, q4 |
| shortcap_1_phase_0.0 | 0 | left P at q0-4 |
| shortcap_1_phase_0.0 | 1 | left P at q0-4 |
| shortcap_1_phase_0.35 | 0 | left P at q0-4 |
| shortcap_1_phase_0.35 | 1 | left P at q0-4 |
| shortcap_5_phase_0.0 | 0 | left P at q0-4 |
| shortcap_5_phase_0.0 | 1 | left P at q0-4 |
| shortcap_5_phase_0.35 | 0 | left P at q0-4 |
| shortcap_5_phase_0.35 | 1 | left P at q0-4 |
| arc_0.6_phase_0.0 | 1 | right P at q0, q1, q4; right R27 at q2, q3 |
| arc_0.6_phase_0.35 | 1 | right P at q0-4 |
| arc_0.6_phase_0.35 | 2 | left P at q0-4 |

Across these 16 cuts there are 83 failed-side proposals: 81 P and 2 R27.
Across ALL 40 cuts/400 side proposals there are 294 YES and 106 UNKNOWN:
104 P + 2 R27. Thus 23 P failures occur in cuts which also contain a different
successful q. Those failures must not be erased when reporting a positive cut.

The ten processed zero-cut cases remain processed zero-cut cases, not missing
runs and not invented junction results: both phases of shortcap_3, parallel,
circle_0.6, circle_8.0 and arc_8.0.

## Compute/resource receipt and scope

Recomputed sums: 306 candidate joint calls, 2480 reported QP systems,
28 reported pivots, 324 reported full scans. Five q hypotheses, the 24-pivot
cap, 27 rounding packets and three prescribed boxes have not changed.
More hypotheses reach the joint stage; actual work is therefore larger than
policy01's 75 joint calls/1143 QP systems/12 pivots/155 scans.

Child diagnostic elapsed time is 8.1100437999703 s versus policy01
4.36994709999999 s; parent02 elapsed time is 8.54830450000009 s. These are
whole offline study timings with exact algebra/serialization and ARE NOT
extractor, matcher, mapper or vehicle-cycle latency measurements. This is an
effect improvement under the same search upper bounds, not a speedup claim.

The parent02 receipt reports failure=null, exit_code=0, retries=0,
own_child_stop_attempted=false, own_child_pid=23556, own_child_stopped=true,
primary_wait_s=60 and single_stop_wait_s=5. The child_started record matches
that child and driver. This is recorded normal completion; no live OS process
or strict all-path worst-case timeout proof was attempted by this reviewer.
All map/adoption/gate/production import/extractor/native/Unity counters are zero.

## Reviewer readback problems, not actual-study failures

Initial overly broad raw/deep JSON output was truncated by tool-output limits;
later complete JSON parsing and compact derived readbacks succeeded. One
PowerShell metadata printer mistakenly passed -join to Select-Object and
stopped that printer. An initial overstrong “all119oldkeys remain in125”
assertion stopped the comparison before geometric checking; the exact expected
one-driver replacement was identified and verified as described above.

A temporary rational function named RP collided with PowerShell's rp alias
(Remove-ItemProperty); parameter binding stopped with required Name missing
before any command operation. No file/property mutation was performed.
All rational helpers were renamed ReviewRat*/ReviewPlaneSign/ReviewChartX;
the final complete read-only arithmetic comparison then succeeded.
One naive empty aggregate array count returned PowerShell's @($null) count
rather than an actual box count; an explicit per-q sum confirmed ZERO boxes
across the 80 remaining UNKNOWN q hypotheses.

None of these reviewer readback retries ran actual02, tests or any model code.
No original result/source/manifest was changed, and no earlier receipt was
backdated or silently repaired.

## Remaining acceptance

The exact candidate contract is nominal all-WB polynomial extension plus a
rounded-model common root in original P/original finite charts. It does not
prove physical lane truth, observation noise reduction, independent sensor
localization, closed-loop tracking, future runtime latency or two laps.

Before adoption, resolve or explicitly preserve the 16 UNKNOWN cuts without
turning search failure into NO/deleting accepted observations/loosening gates.
Then perform scoped production implementation, original observational-parity
and runtime budget checks, unchanged-car/simulator new-scene first-lap mapping
and second-lap fixed-trajectory validation with logs/error metrics/video, and
the required neo repository version control. No such acceptance is claimed
by this document.
