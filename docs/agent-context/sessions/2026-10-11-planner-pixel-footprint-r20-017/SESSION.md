# Agent session: 2026-10-11-planner-pixel-footprint-r20-017

Input handoff: 2026-09-28-planner-neo-004

Development checkpoint only: R20 remains unaccepted for driving. Logs and
analysis are handed off for continued repair, not a successful two-lap result.

## Objective and scope

Continue geometric repair directly in neo, then actual new-scene TEACH lap and
REPEAT lap with logs/error metrics/video. This draft is NOT acceptance. Preserve
vehicle/physics/simulator/camera/LYA speed, causal inputs, original .55 overlap,
5+1 votes, six-face/PCA/angle/J limits and 40/80 ms budgets. No GNSS mapping,
future input, raw-ROI substitute, gap filling, force push or main publication.

## Input handoff

Planner acknowledges neo-004; AGENTS, AGENT_CONTEXT, STATUS and PROTOCOL read.
R19 remains a failed checkpoint, not driving approval. The new per-face model is
explicitly a local straight tangent inside its own WB pixel footprint, not an
exact reconstruction of the original quadratic or physical paint. Root/PCA/J
do not prove cap/curvature accuracy; independent geometric tests and unchanged
four-case development/recorded-prefix checks are required before any Unity run.

## Source and working tree

Base 01442bbf75cb68213dff3c05b88f3bef8639a877; branch
repair/lane-keyframe-transaction-20261011; published R19 rc1 tag. Older untracked
quantized_curve_fit.py and its test remain excluded/preserved. R20 main edits
owned by root: footprint helper, observation arrays and finite-domain integration.
Delegates own finite_union hotloop, logging-only reanchor metrics, ribbon margin
rounding, independent tests and simulator exact-source dependency allowlist.

## Work performed

1. R19 exact CI readback: 4/8 workflows passed, four Foxy builds succeeded but
   failed one exact Fraction minimum_margin assertion; no DDS tests reached.
   Separate append-only receipt records all URLs; sealed016 unchanged.
2. Per-operation outward interval whole-quad J helper developed independently,
   including interval homography composition from the existing already-rounded
   projector components. Diagnostic tests 47 passed. It does not certify physical
   calibration or the old per-query floating evaluation error.
3. Integrated new per-face model candidate: clips tangent at existing WB root to
   its projected quad; whole-quad J decides fragment reliability independently.
   No raw face, point model, topology link or node/inlier count changed. Evidence
   is unioned per original owner before the original temporal vote update.
4. First focused consensus run: 147 passed, 3 failed. Two new tests incorrectly
   expected 5 reliable votes after existing confirmed saturation to 1; fixed
   expectations and added pre-confirmation exact one-vote assertions. One old
   injected J-hole fixture needed to invalidate its newly added fragment fields;
   fixed fixture only, retaining all original assertions. Original XML retained.
5. First full Windows/native regression: 3416 passed, 1 failed, 1 Linux-only skip;
   104.388 s process; all source identities unchanged. Sole failure was the new
   test expecting reliable counters to rise before confirmation: original code
   saturates each counter at its own threshold immediately, so reliable remains
   1 from its first vote. Corrected expectation; original frame count still
   checked as 1/2/3/4, then5. Focused187 passed after fixture correction. This is
   not a complete full-suite green result yet. Added precise new support-policy
   diagnostic string, leaving v3 storage schema unchanged.
6. Current-source adapter focused52 passed; original two historical R8/R12
   native-source identity checks still reject the newer local native module;
   not bypassed or marked as skips. New helper is explicitly loaded/archived.
7. Four-case once-wrapper first failed before any child estimator import due to
   passing a string to a Path-only hash helper. Preserved setup_failure_01;
   corrected only its Path conversion and started first actual four-case run.
   First case left_0 accepted69/509 (old14), frame69 fails original overlap:
   newest reference .5495867769 < .55; active .5371900826. No threshold change.
   Completed four cases: 0/4 pass; accepted 69/435/13/413 of 509 each.
   left_0 fails overlap; right_0 final inlier mass33.75/61.5=.54878049;
   both pitched cases fail combined finite-domain capacity, not visual matching.
   The 2000 ms diagnostic budget is inherited from this offline runner, not a
   runtime acceptance limit. No capacity or geometric threshold was increased.
8. First and only strict recorded R20 prefix: 11/12 accepted/attempted of111;
   seq162 engine81.3592 ms exceeds original80 ms, visual21.8383 ms passes40 ms.
   Failed frame did not commit. All21 integrity checks pass. This is recorded
   input, not a fresh trajectory; the11 accepted steps are bootstrap steps.
   LaneSession's separate200 ms full processing/freshness limit also includes
   extraction and teacher frontend and has not been measured by this replay.
9. Independent fixed-mask audit22 cases/44 extractions: no exceptions, old24
   fields and raw topology byte-exact in all cases. This is NOT geometry pass:
   some cap/side corners retain old60-90 degree wrong normals; finite fragments
   increase their positional influence (long-cap example1.266 to6.691 mm,
   8 m arc example1.090 to21.533 mm). Six short-cap cases create no cap fragments.
   Completecircle .6 m cases have maximum sampled error2.739/2.774 mm; largest
   sampled error over all22 cases52.673 mm. Scoring is posthoc analytic truth,
   uses fixed affine synthetic cameras, and checks three samples per fragment.
10. Passive terminal query captures of original left_0/right_0 prefixes both
    reproduce every original non-timing row and input bit,70/14 calls; no retry.
    Saved nearest primitives/domain histories are for diagnosing, not alternate
    query fallback or permission to extend geometry. Literal saved-query
    classification reproduces all four output arrays byte-for-byte. Left newest
    reference rejects24 for finite alpha,17 for certificate owner branch degree,
    one for primitive direction; right terminal loses0.5 mass when selection
    changes from a valid graph chord to a nearer branch-blocked certificate.
    This is not permission to remove branch protection. Historical physical
    identity/co-occurrence remains under investigation.
11. Full post-fixture Windows/native regression:3417 passed,1 Linux-only skip,
    zero errors/failures;181 source files unchanged; native8 real API calls pass.
    Pytest100.13 s/process101.250 s. Original failed full run remains retained.
12. No new Unity run, frozen route or two-lap video in this session yet.

## Evidence and result

Reviewed small evidence is packaged for the failed R20 checkpoint. Raw masks,
NPZ histories, full local logs and source archives remain local. Independent
whole-quad helper tests: 47 passed. Finite
union literal-old AST/oracle equivalence:109 passed. Ribbon margin old regression
6 failed/68 passed, fixed74 passed, original hard-label gates unchanged. New
per-face focused first-run XML r20_footprint_consensus_01.xml retains3 failures.
Final full regression3417 passed/1skip is not a sum of focused counts. Neither
these unit counts nor R19 CI establishes a lap or runtime acceptance. This
checkpoint's GitHub CI has not yet been read; do not inherit prior CI status.

## Risks and blockers

Unaccepted geometric approximation, caps/curves and temporal coverage remain.
Corrected lateral RMSE is worse than raw over both long pitched accepted
prefixes: left .022108 versus .014636 m; right .018802 versus .014466 m.
Do not compare different-length prefix RMSE as like-for-like improvement.
R20 strict80 ms runtime limit still fails. Original trial28 remains latest
actual drive; there is no real TEACH/REPEAT success or replacement video.
Do not publish absolute local paths/raw bags/full source masks in small bundle.

## Next agent actions

Continue exact failed-query topology analysis and all-support W/B label
consistency diagnosis of legacy quadratic fitting; profile the fixed failure
prefix and full LaneSession chain. Fix identified geometry and representation
causes without threshold changes; recheck full regressions, original four
geometry cases and strict recorded prefix with exact source identities. Only if the relevant
offline gates pass, fresh Unity ownership/runtime/protected-state preflight and
one bounded new actual trial. Preserve all failed attempts; stop/HOLD on failure.

