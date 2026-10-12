# Capture binding: actual interpolation mismatch

Written before any v2 capture protocol or observer pose-admission change.

The MCP `official_capture_outer_01` ran the unchanged semantic/controller with
opt-in capture-only metadata. It completed first-lap learning in 112.213 wall
seconds / 286.089 m, then stopped both workers normally. This is capture and
learning evidence, not junction association or left-turn acceptance.

At genuine sequence 78, capture before and after are exactly identical, but
Rigidbody position `(72.526039, -0.05000019, 14.779480)` differs from the render
root `(72.526054, -0.05000019, 14.777753)`. Their rotations also differ slightly.
Unity interpolation is the expected source. The v1 capsule therefore says
`body_render_pose_mismatch`, `valid=false`, with no usable flat body pose or
capture time. This rejection is correct for the v1 claim of exact Body/image
co-location. It must not be repaired by rounding, tolerance, receipt pose, or
silently calling the Body pose the exposure pose.

Minimal next change: independently identify the same Capture render root as
the image pose (`source=unity_render_root`), retain full quaternion/height and
all original Body audit values, and require stable before/after time/calibration
and render pose. This supplies actual image-time geometry without changing the
camera, interpolation, Body, model, LLP, LYA, core, or map. The new consumer must
not mistake image pose for a simultaneous native Rigidbody odometry sample.

Projection remains a separate question. Original K/R/t and the existing local
ground-plane assumption must stay visible. A measured render pose alone does
not prove the road plane. No scene mesh, raycast, CAD, fixed junction coordinates,
portal, or arbitrary turning radius may manufacture junction observations.
Full world projection needs an explicitly supported plane model and uncertainty;
unknown geometric uncertainty cannot be reported as bounded accuracy.

Restoration: fresh after02 matches before01 in all non-clock snapshot fields,
116 parameters, 3 bodies, 175 components, globals/camera/spawn; 343 source files
have only the three reviewed capture changes. Initial after01 read the tuning
registry before `SerializePreset()` initialized its read-only parameter list;
that 0-vs-116 diagnostic mismatch is preserved. No preset was loaded/applied.
