# R10A design: explicit observation-plane calibration, not a vehicle change

Planner decision: implement and test one independently established projection
model correction before considering changes to association or information weights.
Base R9 RC1 is `58c99c7306f9ca9b7a7155b6b00915ad82d303fd`; it failed quality
and the actual recorded prefix. This plan does not assert the correction fixes
either failure, and does not authorize gate weakening or driving a real vehicle.

## Evidence and rejected alternative

Read-only authoring source, saved scene hierarchy/mesh and live Editor queries
agree: semantic WhiteMarkings are drawn at world y approximately -0.043m,
while the unchanged physical contact ground is approximately -0.05m. All32358
vertices of the active layer6 marking mesh share the same height. The7mm
separation is authored geometry, not a shader displacement, learned correction,
track-coordinate input, or a value fitted to pose/trajectory errors.

The Edit vehicle is spawned25mm above contact ground by the vehicle authoring
tool. A retained Play stop snapshot puts its origin within a few micrometers
of physical ground. Therefore the proposed25mm fixed offset was REJECTED:
spawn clearance is not camera calibration or runtime suspension height.
The source contact model's upright nominal support point is body local y=0.
Neither a world pose nor a runtime contact/raycast result may enter mapping.

The current core instead assumes `normal_body dot point_body = 0` for paint.
With horizontal body and nominal ground-contact origin, the0.655m camera height
above body corresponds to0.648m above this rendered paint. Ignoring the7mm
plane separation conditionally inflates ground-ray distance by about1.08%.
This is a geometric discrepancy, not a demonstrated cause of all map drift.

## Minimal implementation

1. Add an explicit finite signed `plane_offset_m` to TiltedGroundLookup,
   default0 preserving all legacy results and ROS defaults. Intersection is
   `normal_body dot point_body = plane_offset_m`. Keep the original K/T
   byte-for-byte; do not hide the correction inside camera installation data.
   Include offset in the lookup's immutable calibration identity. Camera must
   remain above the selected plane. No dynamic height or road-bank estimation.
2. The new-scene adapter must declare the authored7mm observation-surface offset
   separately from camera K/T, with source/provenance in logs. It is static
   simulator sensor calibration, not a change to the simulated scene, physical
   plane, camera, wheel, LYA, timestep or solver. An incompatible calibration
   must fail explicitly, not trigger an automatic guessed fallback.
3. Historical records remain unchanged. A diagnostic calibration override, if
   needed for the old schema3 prefix, must be explicit candidate-only and logged
   alongside the original calibration and artifact identity. It is NOT exact
   historical reproduction. Do not patch old camera JSON or PNG files.

## Validation before a new live attempt

- Independent scalar ray/plane oracle for positive/negative offset, tilted body,
  original four-neighbour sensitivity, invalid geometry and empty/full masks.
- Zero-offset legacy bit-equivalence, unchanged K/T and offset cache identity.
- Adapter rejects wrong type/nonfinite/mid-session calibration changes and
  keeps all prior-source/receipt/ACK boundaries. Separate nominal-contact and
  no-heave assumptions must remain visible.
- One exact-prefix candidate test with documented input calibration change,
  original40/80ms and every visual gate. Retain failures; no pose truth input.
- Full regressions, a new immutable tag, unchanged-parameter audit, then a
  uniquely logged simulator trial when justified. No two-lap result is assumed.

Do not bundle a change to overlap denominator, past-direction priority, gyro
bias dynamics, or vision/motion weights into this calibration correction.
Remaining curved-lane/bias quality deficiencies retain their failed status.
