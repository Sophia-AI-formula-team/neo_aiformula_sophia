# Deployment Maintainer Instructions

Scope: this YOLOP_A3_Deploy package only. Read README.md, context.md, handoff.md
and weights/manifest.json before changes. Inspect repository status and preserve
unrelated edits. Do not alter sibling YOLOP/A2 or launch wiring incidentally.

## Invariants

- Tag a3-v14 is local V14 step 16,384. Do not substitute the remote A1/FCOS model.
- Keep one shared encoder, one model-only checkpoint and independent lane/object
  outputs. No second detector, teacher, dataset, optimizer or training loop here.
- Match the serialized native input contract exactly. Do not change resize,
  normalization, padding, restoration or score/segmentation thresholds silently.
- RAW lane output at 0.5 is the default. Do not enable closing, thickening or
  component filtering to make screenshots or gate numbers look better.
- Preserve public Python output fields and ROS names/encodings/source headers.
  cone_mask is a box mask for person/car/cone; traffic lights are excluded.
- Cone colors and light states are HSV-derived semantics, not learned subclasses.
- Fail on missing/incompatible weights or invalid V14 contract. Do not download
  or substitute a different weight as fallback.

## Required Verification

1. Run tools/verify_weights.py and pytest tests; use a workspace-local temp path.
2. For weight/model changes, compare both task outputs against the selected
   source at full input resolution and verify one encoder traversal per frame.
3. Build/install the wheel in an isolated directory, run tools/demo.py and inspect
   the separate masks and overlay. Never edit a manifest hash to hide mismatches.
4. Record exact checkpoint hash, input contract, metric definition, dataset scope
   and failed gates. Packaging tests do not prove accuracy, DDS or vehicle safety.
5. Keep licenses and third-party notices; keep credentials, local usernames,
   external paths, data and training checkpoints out of the release.

## Integration And Versioning

Start in shadow mode using DEPLOYMENT.md. Do not auto-enable live vehicle control,
change watchdog policy or launch extra publishers on existing live topics.
No Jetson FPS or TensorRT claim without target-device measurements. Do not mix
desktop dependency pins with incompatible JetPack/ROS binaries.

Use a scoped branch and reviewed PR. Keep immutable release tags; never force-move
a3-v6 or a3-v14. Update manifest, verification, context and handoff together.
Rollback means the entire tagged runtime plus weight, not a renamed checkpoint.
