# Real first-lap fusion failure and diagnostic plan

This report is diagnostic. No semantic, LYA, vehicle, camera, or installed worker source was changed. The proposed snapshot helper beside this document has not been installed, imported, or tested. No acceptance threshold is changed.

## Confirmed result

`component_native_inner_learning` completed a real Unity first lap from `(x=68.76,z=-10)`: travelled 263.707336 m, learned at simulation time 100.9038158 s, 916 recorded samples, reference length 264.1612262 m. The result's `accepted=true` means the **green learning trial** completed. `phase=odom_only`, `map_fusion.available=false`, `topology_mapping.available=false`, and `leftTurnValidated=false` remain explicit.

Core closure conditions are `MapAssist._try_complete_lap`, `map_assist.py:934–978`: leave the 3 m start area; return within 1.5 m with heading within 25 degrees; at least 30 samples and 40 m; current recording points valid. The first lap has no fixed 400 m requirement or hard shape requirement before fusion. Its 20,000-sample limit is far above this lap's 916 samples. No start/route preseed was needed. `route_matched=false` in the first learned status is expected: it is cleared at closure and requires the next original odometry update to match (`map_assist.py:973`, `rough_route.py:165–207`).

## The actual failing gate

`route_fusion.py:253–275` normalizes the official five-chord polygon by measured/official perimeter, uniformly samples 128 points, and runs an orientation-preserving similarity fit. `topology_alignment.py` searches both directions and all cyclic offsets with local continuous phase refinement. The reported fit is:

- RMSE 3.198397389 m, versus the unchanged 3.0 m maximum.
- Alternate RMSE 4.051410560 m; ambiguity gap 0.853013171 m exceeds 0.3 m.
- Phase 0.5737214765, direction +1, normalized-to-official scale 0.7881094633.
- Official chord perimeter 215.7304734 m; measured preparation perimeter 264.1797170 m.

The fitting code returned before `_node_anchors`; there are **no verified physical node correspondences**. There is no evidence here of a missed phase search or reflection/sign bug. There is also insufficient evidence to declare that the supplied official map describes the wrong course: it is explicitly a sparse topology/chord map, not surveyed lane geometry. The demonstrable failure is its geometric similarity gate against the actual smooth driven loop.

## Fixed-transform decomposition (one run; no refit)

`diagnose_first_lap_fixed_alignment.py` applied the already recorded transform to motion samples with `t <= learnedStatus.ros_time_sec=100.789997747`. It excluded all 26 later E-stop/tail samples. This is an **approximation**, not the original 20 Hz RouteSample/trust list: the recorder has 1,458 motion rows up to that point, while the worker's exact selected samples and excluded flags were not exported.

The approximation closes at 264.1928139 m, within 0.0131 m of the core preparation length. Its fixed-transform RMSE is 3.198519643 m, close to the actual 3.198397389 m. Residual decomposition gives 1.698633354 m along the target chord and 2.710197896 m perpendicular to it. Distance to the nearest outer chord, ignoring ordering, still has RMS 2.703621713 m. The largest correspondence residual is 5.600824226 m near actual Unity `(x=64.105758,z=34.077856)`; 5.592727376 m is perpendicular to official edge `10000000003 -> 10000000004`.

Thus equal-arc correspondence contributes, but it does not explain the main residual; the sparse polygon itself differs from the smooth driven outline. Nearest-chord distance is only a diagnostic lower bound, never a replacement acceptance metric or road-clearance test.

## Why a 3.2 m threshold is not a fix

Passing the first gate would expose additional untested assumptions. `_node_anchors` (`route_fusion.py:84–136`) searches ±4 m around phase seeds, using a 2 m feature window. It requires matching turn sign, observed turn at least 25% of the official chord turn (and at least 0.10 rad), reasonable turn difference, correct cyclic order, and distinct nodes.

The five official chord turns are approximately 85.22°, 79.80°, 99.10°, 49.63°, and 46.25°. For a constant-radius arc, the angle between the two feature-window chords is `window/R`. With the current 2 m window, the minimum-turn gate excludes radii above approximately 5.38, 5.74, 4.63, 9.24, and 9.91 m respectively. These are analytic observability limits, **not measured radii or proof of the next failure**. A broad real bend can be observable over a longer distance and remain invisible to this short window.

The test called `test_actual_official_map_nodes_are_observable_in_both_directions` (`tests/test_route_fusion.py:152`) records straight segments of the official polygon itself. It does not cover this actual rounded lap. Changing smoothing, fusion weight, or route start cannot cure an unaccepted initial fit. Reducing ambiguity gates, enlarging match/cross-track tolerances, or globally accepting a 3.2 m residual would not validate node identity.

## Exact next-run export, without changing core behavior

Do not claim exact recovery of `RouteSample.excluded` from recorder pose/status. `recorded_samples` increments reveal when sampling occurred only approximately: motion and semantic response timestamps differ, and one status can cover multiple odometry callbacks. `self.map.samples` alone also omits the final closure sample appended by `_try_complete_lap` with its local `excluded` value.

The proposed `fusion_snapshot_capture.py` wraps only the worker module's imported `build_fused_route` symbol, opt-in for diagnostics. It copies the actual materialized `candidate_samples`, official IDs/points, and arguments at the exact call; calls the original function once with the same objects; captures returned reference points, trust flags, cumulative distance, alignment and node results; and returns the exact original result. Original exceptions are re-raised. Diagnostic copy failures are recorded separately. There is no route injection, threshold change, fabricated observation, or control output.

Install only through an explicit diagnostic worker flag. Include worker/tick/epoch, simulation and odometry stamps, source-tree/file hashes, official OSM hash and active configuration in its context. Drain after constructing the normal command response and export once to the diagnostic log/sidecar, avoiding file I/O or large stdout writes inside fusion. Capture is bounded to four build attempts; diagnostics are not re-consumed by control. A worker restart/reset needs a distinct epoch in every snapshot. The wrapper needs focused identity/exception/disabled-parity tests before installation; it is currently a proposal only.

## Candidate repair design to evaluate offline first

Keep the current gate and default behavior intact. Evaluate an opt-in alternative that aligns **ordered observed turns across multiple physical windows**, rather than declaring every official chord point a metric correspondence. Requirements:

1. Use the exact newly exported measured samples/trust mask. Never use CAD, world route reference, synthetic completion, or preseeded route points.
2. Detect broad turns from trusted measured geometry across a bounded scale range. Require coherent turn sign and accumulated heading, stable observed locations under neighboring window sizes, and independently distinguishable turn events. Do not count the same broad bend as multiple nodes.
3. Associate official node IDs by cyclic turn order and direction. Retain competing phase/direction hypotheses and reject ambiguity, missing or extra required features, excluded turns, or crossings through untrusted spans. A full circle or symmetric outline may legitimately remain unidentifiable.
4. Retain measured segment lengths and actual curve geometry. Validate correspondences and local edge directions on held-out measured segments; do not lower the existing chord-RMSE threshold and call that validation. A new feature-based quality gate needs its own explicit units, negatives, and evidence.
5. Separate a geometrically identifiable outer bend from a physically observable junction. A rounded bend apex is not proof of the branch entrance. Sparse topology IDs and straight internal edges do not establish a drivable corridor or detour road clearance. If that evidence is absent, keep topology navigation unavailable and stop on red.
6. Only after offline acceptance on exact real samples plus negative cases, attempt a new live learning run. Then require post-closure odometry matching, valid fusion/mapped topology, actual detour entry/exit, and vehicle evidence. The 8 m signal stop target and 3 m detour-entry radius remain different geometric conditions; do not make them pass by weakening either.

This is not a parameter-only repair with known safe settings. The smallest justified immediate change is precise diagnostic export; feature association requires evidence before a core change.

## B wiring and turn-sign review

The existing coordinate chain is consistent: optical-to-base maps image-left to positive base `y`; base `x` is forward. `AIFormulaSemanticPlanning.cs:105–106` maps Unity odometry to `(z,-x,-yaw)` and `AIFormulaVisionLaneFollower.cs:160` applies `-CommandW` to Unity. `clouds_to_command` feeds the real left/right/center arrays to native callbacks and uses the native generated ABC; it does not negate or swap left/right. No new B-specific transport sign inversion was found.

However, native `lane_0215.py:160–169` shifts LEFT ABC by unequal offsets `2.6/1.5`, `2.6/1.4`, `2.6/1.3` (about 1.7333, 1.8571, 2.0 m), while RIGHT uses a constant +1.3 m. Even a straight LEFT observation gains a negative slope/curvature bias after these unequal shifts. `withoutkalman_0312.py:50–61` converts that geometry into heading and omega; native LYA combines lateral/heading errors, so the sign of a single output is not proof of a left/right wire fault. Curve association and native asymmetric offset behavior must be distinguished from a coordinate bug. Neither native parameters nor LYA were changed.
