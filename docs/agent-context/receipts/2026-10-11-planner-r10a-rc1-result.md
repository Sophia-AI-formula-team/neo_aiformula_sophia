# R10A RC1: CI passed, actual trial28 failed

Role: planner. This is a development result, not a field deployment approval.

## Exact version and CI

Core commit `023ccc0b18003dffdfb7262e87539077e53fb4c7`, immutable tag
`lane-mapping-20261011-r10a-rc1`. The canonical neo repository's branch/tag
runs all completed successfully: native graph 38092616201 / 38092602367,
teach-repeat Foxy 38092616213 / 38092602534, endpoint GNSS Foxy
38092616227 / 38092602429, handoff integrity 38092616245 / 38092602401.
These are build/test results, not closed-loop acceptance.

## Actual simulation result (retained failure)

Trial28 recorded 261 masks/replies and 111 mapping attempts: 110 accepted,
1636 confirmed cells, 110 trace points. No first lap, route freeze or REPEAT.
At sequence261 the candidate failed the original final-inlier threshold
(0.5492063492 < 0.55); the active reference then had visual rank1 and rejected
with `match_unobservable_geometry`. Candidate/active match times were
31.5434 / 20.5413 ms, engine 69.7242 ms: this failure was not a deadline.
No threshold was relaxed and no failed map was reused for a new trial.

One exact causal replay preserved the recorded six-field camera, 0.007 m
observation plane, PNGs and raw sensor chain. All 111 non-timing mapping
results reproduced exactly. Source modules match the pinned Git contents
(four files differ only by CRLF/LF transport); the native binary is identical.

Independent post-output, same-capture-sequence scoring uses one initial rigid
alignment only. Across accepted sequences151-260, lateral RMSE was raw
0.004630 m versus mapped 0.271754 m; yaw RMSE 0.000608 versus 0.020137 rad.
The first visual-only acquisition, sequence160, introduced +0.0157295 rad
yaw and -0.0472032 m lateral correction. The final accepted mapped lateral
error was +0.559805 m versus raw -0.004738 m. Thus the mapping correction
made localization worse; this is not necessary odometry compensation.
The measured longitudinal raw error is a separate limitation, not corrected
by this lateral/yaw estimator.

A diagnostic-only one-expression seed-plane residual ablation accepted the
recorded prefix but still had 0.263561 m common-prefix lateral RMSE and
+0.523260 m final error. It is not adopted as a production repair merely
because its acceptance count improved. Its source override is explicitly
recorded; it is not the same algorithm as the exact baseline.

## Stop, preservation and artifacts

The zero/HOLD response arrived 140 ms after the failed capture. Physical
velocity was still 0.102347 m/s 530 ms later, so this was not an instant stop.
The subsequent explicit stop, EStop and StopMotion were verified; the final
snapshot before StopMotion was already near zero. Editor returned to clean
Edit mode and the owned backend was closed.

All 116 runtime preset values and three Rigidbody physical fields were
float32-bit identical. The scene hash and the 14 previously documented peer
file differences remained unchanged; the original 878-file protection audit
is not relabelled all-green or reset. No vehicle, physics, speed or LYA
parameter was changed for this test.

Local originals include backend/Unity logs, exact fixture/replay, stop and
preservation audits, and a 30.8 s / 308-frame failure video. It displays
runtime-received consensus at a fixed 120 m scale; all 261 masks pair exactly
and no consensus point is clipped. Seven frames contain ten map-coordinate
serialization differences of exactly one ULP; a hash/bit-pinned diagnostic
whitelist permits only those leaves. The original strict-render failure and
unchanged source logs remain retained. Raw logs, personal paths and video
are not automatically published in this receipt.

Next: repair the erroneous first visual acquisition and its geometric cause,
validate accuracy as well as gates, then publish a new candidate tag before
another actual trial. Two-lap acceptance remains unmet.
