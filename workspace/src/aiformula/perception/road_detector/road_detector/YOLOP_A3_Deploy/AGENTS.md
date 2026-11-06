# A4 Deployment Maintainer Instructions

Scope: this package only. Read README, context, handoff, TEMPORAL and the weight
manifest. Inspect Git status and preserve unrelated edits.

- Current identity is A4 / 4.0.0 / tag a4, with unchanged V76 aligned step1024
  weights from a3-v76. Keep one shared encoder, one
  model-only checkpoint and independent lane/object outputs.
- No training loop, optimizer, teacher, dataset or second detector.
- Match serialized FP32 min640/max1152 resize, normalization, padding and
  native restoration. Do not substitute architecture defaults or V14 weights.
- Raw probability >=0.5 is the default; no closing, thickening or gap joining.
- Temporal history must be causal, previous RAW probability only. Never feed
  ground truth, future frames, recursively fused masks or an uncalibrated warp.
- Keep predict(image) explicitly single-frame. predict_frame requires calibrated
  metric geometry and capture time. Preserve observable reset/mode status.
- Preserve public fields, ROS topics/encodings and independent source Headers.
  cone_mask contains person/car/cone rectangles, not traffic lights.
- Cone colors/light states are HSV rules, not learned classes.
- Fail on missing/incompatible weights; no downloads or legacy fallback.
- Run verify_weights.py, package tests, source/export native-resolution parity
  for both heads, one-encoder checks, wheel install and CLI demos.
- Record exact hashes, input contract, evaluation scope and failed quality gates.
- Keep licenses, private data/paths/credentials and training state out of release.
- Do not alter sibling YOLOP/A2, launcher, controller or watchdog incidentally.
- Start ROS with all shadow remaps. Do not enable vehicle motion or duplicate
  publishers. Target-device timing, DDS and real calibration require their own tests.
- Explain publication versus activation: the model and temporal API are shipped,
  but real-camera temporal input integration is unfinished. The bundled ROS node
  is single-frame, unlike the calibrated Unity temporal demos. See MODE_GUIDE_ZH.md.
- Keep existing folder/distribution/Python/ROS names as compatibility interfaces;
  do not rename serialized source architecture or historical hashes to A4.
- Tags a3-v6/a3-v14/a3-v76/a4 are immutable; rollback the entire runtime and weight.
  Update manifest, verification, context and handoff together.
