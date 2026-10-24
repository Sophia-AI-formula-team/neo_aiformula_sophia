# R12 RC1: explicit quantized paint boundaries; acceptance remains failed

Role: planner. Parent code: `408762fb7d1428c73b6cd3d6bc0946c92f0d9a35`.
Candidate tag: `lane-mapping-20261011-r12-rc1` on the existing repair branch.
This record does not claim that the tag has already been pushed. Publication
and CI must be read back separately. No main/field-baseline promotion.

## Changes

Full masks now supply signed, directed white/black pixel faces, separately
from filled-paint consensus. Contour-only direction fits cannot mix opposite
stripe sides. Quadratic position estimates must have a unique outward root
inside the actual observed white/black bracket, retain raw and model J4 checks,
and cannot extrapolate beyond the fitted observed interval. This is a local
model of quantized observations, not recovered subpixel truth.

Consensus votes separately for contiguous directed records and actual observed
record adjacency; original 5-frame/1-reliable-frame gates remain. Matching uses
finite confirmed node chords. Raw historical pixel faces are retained for
audit, not silently substituted as a precise target. Full graph degrees survive
subsetting so a cropped branch cannot acquire false smooth-joint certification.
Every foreground half-mass remains in the original overlap denominator,
including missing or uncertified sides. No current/future map votes enter the
reference used to validate the current frame.

Schema3 bundles, the raw-motion worker and TEACH/REPEAT handoff carry the explicit
boundary identity and graph arrays. Ordinary legacy ROS entries are not migrated
by this change. This is not GNSS mapping or a complete loop-closure SLAM system.

Integer grouping, batched means and small contractions have independent exact
numeric oracles. Existing guarded map admission now additionally revokes pending
localization proposals in a finally clause on both active/candidate owners.
Six precommit-exception tests fail before and pass after that fix. It does not
promise rollback if an error occurs after persistent stores begin committing.

## Evidence and failures

Full Windows native core: 2396 passed, 1 Linux-only process-group skip, 67.22 s.
The seven compiled APIs were actually called; 147 selected source/config/native
files were unchanged across the run. Focused batch-mean/equivalence/exception
suite:63 passed. Extractor exact-equivalence suite:67 passed. These are software
checks, not timing or closed-loop acceptance.

Full simulator adapter attempt:318 tests,21 failures,7 errors,83.923s. Its
original878-file protection check also remains false with14 existing peer-owned
differences. Neither failure is hidden or rebaselined. Subsequent analysis
separates old internal-call fixtures, wrong fixture matchers and a real
missing-boundary-reference preparation guard. The320px mask has74 reliable
confirmed boundary nodes, below original100, despite116 filled cells; it is
not sufficient startup evidence. Repairs are pending a fresh test attempt.

On the identical 97 accepted recorded-simulation frames, fixed-origin posthoc
scoring without spatial/time refitting gives:

| Estimate | Lateral RMSE m | Yaw RMSE rad |
| --- | ---: | ---: |
| Raw wheel/gyro | 0.004596479 | 0.000646559 |
| R10A | 0.220024830 | 0.019233853 |
| R11 | 0.063455734 | 0.007308036 |
| R12 | 0.006386860 | 0.000940494 |

R12 reduces false visual drift, but raw is still better. Longitudinal RMSE
remains about0.28456m for all four; total localization improvement is unproven.
Truth was used only in scoring, not by the estimator.

Four frozen full-mask synthetic development cases (left/right, flat/pitched)
ALL stop on the eighth attempted frame for insufficient overlap. Left0.487603,
right0.512195 are below unchanged0.55. Only7/509 masks per case were accepted.
The latest committed past reference raises the left test only to0.528926;
refresh frequency alone is not a geometric repair. Eight reused holdout-family
cases have not been run for R12.

A no-solver audit reproduced all103 query representatives by bytes. Uncertified
mass10.5 includes7 invalid direction,2 out-of-bracket roots,1 unreliableJ,
and0.5 direction mismatch. Of the direction-invalid mass,6.5 belongs to tiny
raster components with only4-5 same-side faces: do not invent the required6.
All27 finite-support rejections are true graph terminals. Some queries are near
actual past faces, but latest face extent does not prove5-frame support and
cannot justify extending every endpoint.

Recorded candidate04:97/98 accepted, seq248 deadline. Candidate05:101/102,
seq252 deadline. After the exact mean/copy optimization, candidate06:104/105,
seq255 deadline84.1913ms above the original80ms gate. No failed frame committed.
These are distinct, documented source revisions, not retries until a timer passes.
Extraction is outside the engine stopwatch and must also be included in future
end-to-end image latency measurements. Profiling with2000ms diagnostic budgets
is explicitly NOT runtime admission.

## Still required

Correct the local quantization fit and independently observed finite support;
do not clamp roots, bridge unseen gaps, reduce denominators or relax gates.
Bound work as the accumulated map grows. Rerun fixed quality and runtime tests,
then the saved new Unity scene with unchanged physics/vehicle/LYA parameters.
Actual driving remains trial28 (R10A), failed before a complete lap. There is
no new complete first lap, frozen driving route, second lap, or two-lap video.

Also unresolved for deployment: ordinary reference/fixed ROS entries retain the
older VectorNav chain; `_gnss` TEACH rejection currently logs rather than having
the simulator's complete HOLD policy. Do not infer safe vehicle operation from
the worker/core tests. Hardware axis/time/specific-force assertions remain
unverified; GNSS is endpoint-only and never builds or aligns the map.

Reviewed small evidence is indexed in session
`2026-10-11-planner-boundary-r12-010`. Raw images, truth/log streams, profiler
dumps and videos remain local; no raw bag or exact GNSS location is published.
