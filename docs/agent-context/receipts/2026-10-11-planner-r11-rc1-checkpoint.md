# R11 RC1: preserve bootstrap motion information, geometry still fails

Role: planner. Candidate tag: `lane-mapping-20261011-r11-rc1`.
This is a versioned repair checkpoint, not two-lap or vehicle acceptance.
Parent: R10A commit `023ccc0b18003dffdfb7262e87539077e53fb4c7`.

## Repair and unchanged boundaries

The same-session TEACH engine previously committed bootstrap masks and raw
poses but no motion information. Its first visual match was unconstrained by
that motion history; only afterwards was the information state initialized
around the already-shifted visual pose. Trial28's first match introduced
0.0157295 rad of erroneous yaw, propagated through subsequent displacement.

R11 creates the existing initial engineering covariance at the first admitted
bootstrap frame and propagates it with the existing source-time F/Q model.
Only guarded map commits advance this immutable state. A fresh keyframe is
bound to its engine-owned bootstrap source and map frontier. An explicit,
keyword-only seed regularizes its first visual fit; it does not fabricate a
visual acceptance timestamp, confirmation count or tracking gate. Ordinary
fresh localization against another map remains visual-only. A map already
committed without motion provenance cannot acquire invented bootstrap
information midway through the session.

No motion/visual noise value, PCA residual, geometry gate, 40/80 ms budget,
vehicle, physics, camera, speed or LYA parameter changed. This information
model remains an engineering regularizer, not measured covariance or a
complete map/pose cross-correlation model.

## Tests and retained failures

29 new tests cover independent covariance propagation, source/frontier
validation, rejection/guard/deadline rollback, stale candidates, initial
visual gates, legacy behavior and handoff isolation. Focused suite: 174 passed.
Full native core suite: 1918 passed, one platform-specific skip, 53.09 s.
Two initial new-test fixture errors were fixed without changing the algorithm
or weakening assertions; their failed outputs remain local.

The first simulator-adapter regression run was 288 passed / 1 failed: an old
handoff-corruption fixture built legacy geometry, then enabled motion halfway
through. R11 correctly rejects that missing history. The fixture repair must
start a fresh, causal motion-aware bootstrap and retain the receiver-corruption
assertions. That narrow repair also adds an explicit legacy-to-motion rejection
test. Focused handoff tests: 19 passed. Subsequent full adapter regression:
290 passed, 16.94 s (`r11_adapter_full_02.xml`), zero failures/errors/skips.
The first failed run is retained, not replaced by this result.

## One candidate replay, then post-output scoring

The original trial28 six-field camera and 7 mm plane, raw-IMU chain, PNGs,
source/receipt clocks and inputs were unchanged. The worktree candidate is
explicitly distinguished from an exact historical replay. No pose substitution,
future input, seed-plane residual ablation, gate change or retry was used.

106 of 107 attempts were accepted. Sequence257 failed: candidate final inlier
mass was 36.25 / 66.083333 = 0.5485498, below the unchanged 0.55; the active
reference then failed overlap. No full recorded prefix or actual lap passed.

Post-output evaluation compares only the common accepted sequences151-256,
with one initial rigid alignment and no fitted scale/re-alignment:

| Metric | R10A | R11 | Raw motion |
| --- | ---: | ---: | ---: |
| Lateral RMSE, m | 0.255642 | 0.079087 | 0.004624 |
| Final lateral error, m | 0.525897 | 0.187582 | about -0.0047 |
| Yaw RMSE, rad | 0.0198725 | 0.0079246 | about 0.00062 |

First-match yaw innovation reduced to 0.000117928 rad and lateral innovation
to -0.00412284 m. This confirms the state repair, not a geometric cure: the
candidate still degrades the near-ideal raw estimate and rejects earlier.

An independent replay of all 107 attempts reproduced every non-timing field.
The failure contains genuinely unobserved finite support, filled-paint width
incorrectly treated as zero-width-line residual, and four points misassociated
with an adjacent bar. The latter correctly reject. A future geometry repair
must distinguish these causes; it cannot remove unmatched mass or lower the
threshold to disguise them.

## Next work and evidence boundary

No new actual driving trial uses this candidate yet. The simulator remains in
clean Edit mode with its backend closed. Evaluator-only pose logging was added
to its separate truth file, with no wire/estimator changes: 12 source checks,
44 compiled adapter checks, actual readonly-field JSON serialization, and an
empty error console were verified. The extra serialization cost is not yet
measured in driving. No 100 Hz file I/O was added.

Next is a causal, semantically correct paint-boundary observation model that
does not confuse paint width or confirmation clipping with vehicle motion.
Filled consensus remains the map/display; physical observation support and
gates cannot be invented. Raw logs/video remain local pending a reviewed small
handoff package. CI success, when available, will not change these unmet
accuracy and two-lap criteria.
