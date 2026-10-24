# R26 piecewise short-closure: static and recorded-result review

Status: rejected development estimator; not integrated. This review reads the
frozen source and existing JSON only. No extractor, engine, fit, test, or Unity
call was made for this review. CPU remained available to the other task.
The prototype, prepared runner, manifest, results and repository were not edited.

## Scope and identity

This is a changed geometric estimator, not an equivalent optimization. It cuts
computational fitting domains while retaining the original raster observations.
The diagnostic used the original 22 development masks, not a new holdout.

All paths below are relative to this diagnostic directory unless stated otherwise.

| Artifact | SHA-256 |
|---|---|
| `piecewise_closure.py` | `a13ac9f704691c29ccfc07acfe4337656a13b29e0e7e007307449eb552ca8da6` |
| `run_once.py` | `6bec8e219a7cb4b74566ced68abc268546e6236cb8310b0e70ff674cb05d8527` |
| `source_manifest_PREPARED_01.json` | `bdd5a539b6351b343a117153caab3fcbf9b853621aa21c48cfa9182d43cf1d78` |
| `result_01.json` | `5dec02d85e6d2c9a7a6b329761346ed295bd92e8d29a923386f8ccc1d68dab00` |
| `arc_8.0_phase_0.35.json` | `038fbc56fb71c8daab5bc10c0a0c3d64b9db94130434929e2cc9bb467f02d772` |
| `source_01.zip` | `0586fb33c5aa5f581a51a464cfa2c727c656ded5e67d4a70958f487105d349c3` |

The complete 11-module source identity remains in the prepared manifest.
No withdrawn quadratic-label guard or unrelated curve-fit prototype was loaded.
Raw geometry, projection, sensitivity, stable/metric flags, W/B pixels and
original previous/next/contour arrays were byte-exact to the saved input.
Truth was used only for the already-completed post-extraction scoring.

## Actual result, not an acceptance claim

The one run completed 22/22 cases without an exception; integrity was true.
There were 22 extractor calls and zero engine, matcher or Unity calls.
No case was all-unknown and no declared rectangle-coverage check failed.

| Scored metric | Stored R22 | R26 candidate |
|---|---:|---:|
| Analytically assigned roots | 9,940 | 10,066 |
| Position RMSE (m) | 0.008215398400802475 | 0.008123005265956962 |
| Position p95 (m) | 0.015946521420944163 | 0.015376164874062863 |
| Position maximum (m) | 0.04379469343283469 | 0.03999999999999915 |
| Normal RMSE (degrees) | 1.9080007058099584 | 1.6174459811128936 |
| Normal p95 (degrees) | 3.8583928342018714 | 3.0954075217438732 |
| Normal maximum (degrees) | 60.02334681886688 | 80.79041056836871 |
| Normal error above 30 degrees | 3 | 1 |

There were 4 lost faces (2 side, 2 cap) and 130 gained faces (129 side, 1 cap).
All these lost/gained faces were analytically assigned. There were 9,946 common
selected face identities, of which 9,936 were assigned/scored; the other 10
remained analytically ambiguous. These denominators must not be conflated.

The aggregate improvement does not justify integration: the changed estimator
newly admits a cap with an 80.7904-degree direction error. This is not fixed by
counting fewer total severe errors, or by preserving raw bytes. There is no
runtime, pose, map, full pipeline, completed-lap or vehicle acceptance evidence.

## Concrete failure: arc_8.0_phase_0.35, face 2

This mask has 644 raw faces and one closed original contour. The original
direction-domain algorithm also returned one closed 644-face domain. The
candidate enumerated 40 closure hypotheses, selected 2, and produced open
domains of lengths 323, 2, 317, 2. Face 2 was the last original contour face
(index 643); it became the last face (index 316) of the 317-face long domain.

The posthoc analytic cap faces are exactly [0, 2, 376, 378], all raw-stable.
The newly admitted face 2 is cap primitive 3, position error
0.007751700338249332 m, signed-normal error +80.79041056836871 degrees.

### Recorded assignment explains how the cap entered a long arm

The first selected closure is [0, 380], start 0, count 2, radius 0.45 m,
cost 12.68439460161828. It contains true cap 0 and true side 380.

Its left fitting arm is [390, 389, 4, 385, 384, 2], with fitted normal
[-0.41030018625644343, -0.9119505234155676] and line cost
5.787260554099706. Thus cap 2 is used as side evidence and is not isolated.
Its right arm is [381, 1, 382, 383, 3, 386], line cost 5.5108396863986835.

A competing actual-cap closure [2, 0] exists: start 643, count 2,
radius 0.45 m, cost 12.960815469319366. Its left arm is
[6, 390, 389, 4, 385, 384] and right arm [380, 381, 1, 382, 383, 3].
Both are line fits with costs about 5.7872605540997. It loses to [0, 380].
These hypotheses do not use the same observations or assignments, so their
cost ordering is not evidence that [0, 380] identifies the physical cap.
This actual failure is not an exact-cost tie.

The second selected closure is [378, 376], start 325, count 2,
radius 0.45 m, cost 13.880338521343281. Both are actual cap faces.
Both selected short domains end with normal/model/fragment validity false.
The unresolved issue is incorrect domain membership, not a selected cap
passing its own insufficient-support check.

### Fallback and old-model boundary

This case has only one original contour. The existing ribbon proposal function
returns early when fewer than two stable contour IDs are present. Therefore
face 2 was not resurrected by ribbon fallback; it was admitted by the main
estimator on the newly cut long domain.

Face 2 is absent from both old R20 and R22 model-score lists and is present in
the recorded gained list [2, 643]. The reviewed JSON does not retain its old
per-face direction/model/J rejection stage or old fit coefficients. It proves
old non-admission, not whether its old cause was direction support, W/B root,
fit-domain, or J rejection. No such cause is inferred or fabricated here.
Existing NPZ arrays were not loaded and no old fit was recomputed.

The previously severe common side face 637 improved from 32.77993711917406 to
9.89219879544932 degrees, and from 0.04379469343283469 to
0.021665595325911193 m. Old cap faces 0 and 376, whose errors were
+60.02334681886688 and -36.428972244850286 degrees, were removed from the model.
These improvements are real but do not cancel the new face-2 counterexample.

## Five static findings, with actual-result boundaries

1. **A complete candidate arm is not the unchanged original target window.**
   The arm traversal fixes each seed/radius window before fitting, stops on
   invalid/opposite-polarity/arc-length conditions, and does not trim points
   after seeing residuals. Nevertheless, changing the seed, one-sided domain,
   and cuts changes the original two-sided target support. The final estimator
   refits on the new domain; it is incorrect to call the overall geometry
   unchanged merely because no candidate arm was residual-trimmed.

2. **No common-evidence uncut comparison.**
   The score is the sum of two arm fits and a closure fit. Different candidates
   may use different observations, assignments, counts, and first-valid radii.
   An uncut explanation is not a competing hypothesis. Opposing supported
   arms do not uniquely identify a cap: a short U-turn, re-entrant shape or
   smooth reversal can satisfy the same necessary conditions. The declared
   residual/complexity prior is not a calibrated likelihood or certificate.
   Face 2 supplies a concrete wrong-membership result from this mechanism.

3. **Conflict/tie handling does not express the full ownership problem.**
   The exact-cost check only intersects closure faces, while suppression also
   uses immediate closure neighbours. Equal-cost disjoint closures with
   intersecting neighbourhoods can therefore be resolved by enumeration order.
   A selected cut can also remove faces used to qualify another arm; the
   consumed set does not encode all arm ownership. Closed-contour cyclic start
   invariance and post-partition support sufficiency are not established.
   These are static risks, not the claimed cause of the unequal-cost face-2 case.

4. **Raw-only short closures are not generally protected after all proposals.**
   On multi-contour inputs, the existing ribbon path can consider stable faces
   invalidated by the main estimator. It is not informed of the prototype's
   selected closure ownership. Its original W/B/PCA/radius gates remain, but
   those do not establish that a previously isolated closure stays raw-only.
   This risk was not exercised by the one-contour face-2 case.

5. **Runtime cost remains structurally unbounded by this diagnostic.**
   Cached arms still require roughly O(n R P) traversal/storage, with R original
   radii and P patch cap; repeated set/tuple construction, PCA and least-squares
   fits remain. Up to (k-1)n candidates feed a worst-case quadratic global
   exact-cost comparison. The input face cap is not a latency guarantee.
   No 80 ms engine or 200 ms end-to-end claim follows from these 22 offline masks.

## General corrections worth testing, not a face-specific fix

A model-selection revision should form actual conflict components first, freeze
a common causal evidence window per component before scoring, and compare uncut
and alternative partitions on the same observations/weights and same N.
Complexity must account for the complete explanation; training faces cannot
disappear from the cost simply because they were reassigned or isolated.
Equal-best conflicting explanations should remain unresolved/uncut without a
face-ID or contour-start tie-break. The final joint partition must satisfy the
unchanged support gates after all cuts. Fallback must obey the same declared
ownership decision. None of this licenses new angle, radius, vote or overlap
thresholds, truth-fed selection, or tuning a score epsilon to face 2.

A separate set-valued experiment may instead keep all qualifying assignments
and mark their closure union unresolved, with post-ALL-proposals non-admission.
That removes the discrete winner and the incomplete exact-cost tie rule; it
does not prove the hypothesis family is complete or physically correct.
The union may absorb genuine side faces, shorten remaining arms below k, or
spread over short U-turns/smooth reversals. The original full evidence mass must
remain, with lost/gained side/cap coverage and all-unknown outcomes reported.
It is an explicit abstention model, not identification of physical caps or
proof that ordinary rectangles retain sufficient direction coverage.
