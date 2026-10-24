# Independent R27/R28 holdout review — REJECT general integration

This is a read-only review of the single immutable `run_01`. It is not another
algorithm execution. No retune/retry, source/input change, generator, extractor,
engine, Unity operation or write inside run_01 occurred during this review.

## Decision

The fixed positive criterion fails: **9/12 normal rectangles pass; 3/12 fail the
original per-case 90% side-coverage requirement.** All 24 cases were scored.
Candidate severe signed-normal errors (>30 degrees) are 0 versus baseline 45,
with no newly severe face in any case. This does not override coverage failure.

All 45 baseline severe faces were removed from the admitted set, not repaired
at those same faces. Overall, admitted roots are 5447→5185: 36 gained, 298 lost
(253 lost faces were not baseline severe), 5149 common. Covered side faces are
5319→5103: 36 gained, 252 lost, 5067 common. The fixed pooled denominator is 5543:
coverage 95.9589%→92.0621%.
A pooled ratio cannot replace the predeclared per-case gate.

| Failed positive case | Baseline side coverage | Candidate exact count/coverage | Result |
| --- | ---: | ---: | --- |
| rect_000_p0125 | 95.1872% | 166/187 = 88.7701% | FAIL |
| rect_090_p0125 | 95.1872% | 166/187 = 88.7701% | FAIL |
| rect_101_p0125 | 95.8904% | 197/219 = 89.9543% | FAIL |

The 101-degree case is below .9 without tolerance or rounding exemption.
Both axis-aligned phase-.125 cases had no baseline severe roots, yet lose
12 covered roots each. This is useful positive-coverage counterevidence, not
a reason to change masks, thresholds or select another phase.

## Integrity and actual execution evidence

- APPROVED manifest: `1a671891561f47d31a420113ab6c04d9a147255da19cf0704221be2485b94021`; snapshot approval is
  byte-identical to the root-approved file and execution record.
- 423 seal entries independently rehashed: no mismatch. 14 algorithm source
  files, 11 harness files and all 25 source ZIP members match their frozen pins.
  Original source files still match; PENDING and BOUND hashes are unchanged.
- Actual generator result/log: 13 passed, 0 failures/errors/skips (0.114 s in the
  original log). These are the holdout generator tests, not R28 candidate tests.
- All 24 input generations completed. The 48 baseline/candidate process logs
  show exit 0, no timeout; inputs before/after and source-after records match.
  Each used its own Python -I -B invocation. All 24 individual scores match the
  scoring summary. Every unresolved final normal/model/fragment flag is invalid.
- All 11 raw array members were independently compared as NPY bytes for all 24
  pairs: exact, including previous/next/contour IDs. Original raw-reason-bit
  equality is reported by the hash-verified sealed scorer. No model-dependent
  denominator removal occurred in that scorer.
- Output seal predates scoring. Only the expected final execution/scoring files
  lie outside the pre-score seal; their current hashes are in REVIEW_01.json.
  No generator or analytic-truth module appears in estimator import records.
- Original whole-harness duration 17.867384 s
  includes processes and scoring; **not extractor runtime or vehicle acceptance**.

## Position and angle distributions

Values are pooled from sealed individual roots, not averages of per-case RMSEs.
Position is analytic-boundary distance in mm; angles are signed errors summarized
as RMSE / absolute P95 / absolute maximum in degrees. No truth was recomputed or
fed to an estimator in this review.

| Population | Roots | Position mm: RMSE / P95 / max | Angle deg: RMSE / P95 / max |
| --- | ---: | ---: | ---: |
| All baseline | 5447 | 5.253284 / 11.250000 / 24.905808 | 6.825921 / 4.972614 / 93.719227 |
| All candidate | 5185 | 4.793096 / 11.250000 / 17.966892 | 0.879851 / 1.266624 / 24.396804 |
| Common baseline | 5149 | 4.818524 / 11.250000 / 17.966892 | 1.376826 / 2.966442 / 24.396804 |
| Common candidate | 5149 | 4.803734 / 11.250000 / 17.966892 | 0.854866 / 1.214316 / 24.396804 |

All-admitted populations differ because of deletion/addition. The common-root
row controls that survivor-set difference but is still this reused static test,
not new runtime evidence. Normal-rectangle pooled coverage is
96.3429%→91.6571%;
normal severe count is 27→0. The 12-case pooled score also cannot hide 3 failures.

## Adverse cases: extraction evidence only

| Case | Admitted roots | Side coverage baseline→candidate | Severe count | Admitted gained/lost |
| --- | ---: | ---: | ---: | ---: |
| black_gap | 160→120 | 96.2500%→75.0000% | 6→0 | 0/40 |
| U_notch | 386→374 | 96.7742%→93.2551% | 0→0 | 0/12 |
| FOV_open_sides | 380→380 | 100.0000%→100.0000% | 0→0 | 0/0 |
| J_horizon_ribbon | 254→254 | 94.3320%→94.3320% | 0→0 | 0/0 |

Raw observed masks/topology/projection/J arrays are unchanged. The reviewed hook
also checks returned ordered domain adjacencies are a subset of original ones;
all children completed without violating this check. FOV and J cases have every
saved observation NPY member byte-identical between baseline and candidate.
Black-gap and U outputs change model/validity fields while retaining the raw
fields. Black-gap coverage falls to 75%; no positive 90% gate was invented for
this adverse case, and it is not omitted.

These facts **do not prove absence of downstream black-gap filling, erroneous
temporal association, graph links, domain votes, mapping failures or unsafe
driving**. No mapper/consensus/route/control loop was exercised. Preserving raw
links is an extraction contract, not a guarantee that all changed model support
or a future mapping pipeline is safe. No general adverse-geometry PASS is given.

## Complete fixed population

| Case | Candidate covered / fixed denominator | Side coverage baseline→candidate | Severe baseline→candidate | Normal-rectangle criterion |
| --- | ---: | ---: | ---: | --- |
| 000 rect_000_p0125 | 166/187 | 95.1872%→88.7701% | 0→0 | FAIL |
| 001 rect_000_p0625 | 168/185 | 99.4595%→90.8108% | 4→0 | PASS |
| 002 rect_090_p0125 | 166/187 | 95.1872%→88.7701% | 0→0 | FAIL |
| 003 rect_090_p0625 | 168/185 | 99.4595%→90.8108% | 4→0 | PASS |
| 004 rect_011_p0125 | 200/219 | 95.4338%→91.3242% | 3→0 | PASS |
| 005 rect_011_p0625 | 202/219 | 94.0639%→92.2374% | 3→0 | PASS |
| 006 rect_037_p0125 | 244/260 | 97.3077%→93.8462% | 3→0 | PASS |
| 007 rect_037_p0625 | 244/261 | 96.5517%→93.4866% | 3→0 | PASS |
| 008 rect_068_p0125 | 224/242 | 97.1074%→92.5620% | 2→0 | PASS |
| 009 rect_068_p0625 | 225/243 | 97.1193%→92.5926% | 2→0 | PASS |
| 010 rect_101_p0125 | 197/219 | 95.8904%→89.9543% | 2→0 | FAIL |
| 011 rect_101_p0625 | 202/218 | 93.5780%→92.6606% | 1→0 | PASS |
| 012 slanted_cap_minus25 | 244/260 | 97.3077%→93.8462% | 3→0 | reported only |
| 013 slanted_cap_plus25 | 243/261 | 95.4023%→93.1034% | 1→0 | reported only |
| 014 arc_R2_left | 221/233 | 96.9957%→94.8498% | 1→0 | reported only |
| 015 arc_R2_right | 220/232 | 96.1207%→94.8276% | 2→0 | reported only |
| 016 arc_R5_left | 186/208 | 91.8269%→89.4231% | 2→0 | reported only |
| 017 arc_R5_right | 192/208 | 96.1538%→92.3077% | 1→0 | reported only |
| 018 arc_R13_left | 174/194 | 93.2990%→89.6907% | 0→0 | reported only |
| 019 arc_R13_right | 166/194 | 87.6289%→85.5670% | 2→0 | reported only |
| 020 black_gap | 120/160 | 96.2500%→75.0000% | 6→0 | reported only |
| 021 U_notch | 318/341 | 96.7742%→93.2551% | 0→0 | reported only |
| 022 FOV_open_sides | 380/380 | 100.0000%→100.0000% | 0→0 | reported only |
| 023 J_horizon_ribbon | 233/247 | 94.3320%→94.3320% | 0→0 | reported only |

## Evidence anchors

- `run_01/outputs_sealed.json`: `e572144a373996f306020234eab3a49c3ab9bc0c152ea64d923cd061b38d8bb1`
- `run_01/scoring.json`: `31289a88d53dcd96d19578865151da783a8bc8fa4a8b8d8dd4e03c0eb00c5fb1`
- `run_01/source_snapshot.zip`: `2426779893e2538a53a928955745a5650cbd03efb76bb9efb0ba49875993d16a`
- Full per-case metrics, source/input checks and exact counts: `REVIEW_01.json`.

Preserve this failed holdout result unchanged. Further model work requires new
explicit design/validation decisions; this review authorizes no integration or
additional execution.
