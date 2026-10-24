# R10A RC1: explicit observation-plane correction

Planner repair branch: `repair/lane-keyframe-transaction-20261011`.
Base `58c99c7306f9ca9b7a7155b6b00915ad82d303fd`; new immutable checkpoint
`lane-mapping-20261011-r10a-rc1`. This is not a completed-lap or vehicle release.

## Correction and scope

The simulator's authored semantic white-paint mesh is7mm above its physical
contact ground. The old mapper treated both as the same body-origin plane.
Independent authoring-source, complete-mesh and clean-Editor checks establish
this difference;25mm is spawn clearance and is explicitly NOT a calibration.

The shared `TiltedGroundLookup` now accepts separate keyword-only
`plane_offset_m=0.0`: `up_body dot point_body = plane_offset_m`. Signed finite
real values only; offset participates in immutable calibration identity.
Zero-offset/level behavior remains byte-exact, including the old empty result
for nonpositive camera height. Nonzero-offset/dynamic path requires the camera
above its plane. K/T and the existing J4, active pixels, gates and bounds remain.
No ROS node/default, vehicle, scene, camera, LYA, speed or physics change.

The local simulator adapter declares `.007` and a named authored source as
separate camera fields, requiring the raw-IMU protocol and unchanged calibration
throughout each session. There is no runtime worldheight/pose/raycast input.
Nominal body contact and no dynamic heave/bank compensation remain assumptions.
The actual upstream LYA frontend gets its original four K/T camera fields and
unchanged original projection; this separate teacher assumption is logged.

## Validation and retained failures

- Core focused161pass; full native1889pass/1platformskip,53.93s.
- Full local adapter283pass,19.14s; includes real upstream frontend through
  raw-session.step, unchanged target/omega, calibration cache, zero HOLD and
  oversized JSON integer rejection normalized to ValueError.
- Unity compiled44adapter checks and6independent full-mesh checks passed,
  no Console errors, clean Edit scene. Before explicit import, the Editor was
  still using an older assembly;40old checks did NOT validate the new code and
  the missing new method failed. That attempt is retained, not a success.
- First adapter-plane fixture incorrectly called camera before selecting its
  first motion protocol:3fail/11pass. Corrected explicit sensor setup15pass.
- First replay-new-test fixture lacked attitude tracker config:54pass/4errors;
  fixed the test substitute only. Focused58pass, then25plane-only tests after
  adding direct huge-integer ValueError regression. No production gate changed.

One predeclared old-trial27 candidate run, explicit `.007` override, processed
107/107mapping frames through sequence407, all accepted. Original fixture SHA
`014b73194902fcc389bb6098288c6f2093de3d24a6b4ed603a8398dab632109a` unchanged;
candidate report SHA `02868f56a4dd6a32da849660f32615707ec81f2966990db89728c3358a556ad2`.
Original40/80ms budgets, finite confirmed support, votes and geometric gates
remain. Final map1369confirmed cells/107trace points, final inlier ratio
.56978233, overlap.71728553, engine25.8635ms/match11.8091ms.
This is changed-calibration candidate evaluation, NOT exact historical replay,
runtime timing or closed-loop proof. It retains original/selected calibration
and raw sensor/source/receipt/ACK/PNG provenance; no recorded pose substitution.

## Open work

R9 eight-case quality remains2/8; the general bias/curved-lane issue is not
claimed fixed by this simulator-only surface correction. Final recorded-prefix
correction magnitude is still about.499m, not an accuracy measurement.
The partial-front-edge normal issue has a separate unimplemented design; no
direction override, support extension, denominator or motion-weight change is
included here. New actual trial28 must test first-lap mapping and REPEAT.
No full lap has yet completed. Session009 is still an ongoing local draft;
current local adapter snapshots are not yet a sealed GitHub handoff bundle.
