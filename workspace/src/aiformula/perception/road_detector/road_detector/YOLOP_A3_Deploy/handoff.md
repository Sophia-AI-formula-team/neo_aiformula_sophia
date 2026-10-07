# A3 Deployment Handoff

## Current Result

This is the selected v6 epoch 24 candidate, not a completed accuracy acceptance.
The run completed 30 sampled epochs without a crash and stopped on configured
plateau. Best epoch 24 passes all 22 relative protection checks. Last epoch 30 fails the real
person protection check and was NOT selected.

| Metric | Before v6 | Selected epoch24 |
|---|---:|---:|
| Real lane IoU | 61.789% | 61.918% |
| Real lane recall | 77.121% | 77.238% |
| Empty-frame FP,lower is better | 1.099% | 1.133% |
| Dash-gap recall proxy | 73.455% | 73.566% |
| Real small-object recall | 17.635% | 20.486% |
| Unity small-object recall | 64.476% | 65.374% |

Real person/light/cone recall: 45.07%/49.37%/20.10%.
Unity person/light/cone recall: 93.80%/98.30%/78.65%.
These are fixed score 0.5 / IoU 0.5 validation measurements, not mAP or vehicle rates.
Real distant lanes, empty-frame false positives and small cones remain problems.
Only real lane IoU passes the separate A1/real-object absolute acceptance checks.

See verification/training_summary.json for unrounded values,all relative gates,
failed absolute flags,baseline and selected measurements. No thresholds or
acceptance tolerances were relaxed for this publication.

## What Changed For Publication

- Stripped best24 to one inference-only state dictionary;all328tensors exactly
  equal the selected training checkpoint. See weights/manifest.json.
- Public PerceptionModel and existing ROS wrapper now select SharedFPNPerception.
  Previously the local public entrypoint rejected the new checkpoint because it
  still selected the obsolete SSD loader.
- Extracted common output dataclasses/rendering without changing numerical
  behavior. No old SSD model,ONNX,second model or training dependencies included.
- Preserved existing ROS topics,mask semantics,color rules and source headers.
- Added installation/shadow/remap/rollback documentation and package tests.

## Verification

- 20 local tests: bundled checkpoint/public entrypoint, single encoder traversal,
  invalid inputs, mask values/shapes, color semantics and mocked ROS callbacks.
- 8 full-resolution CUDA diagnostic views: four real, two Unity, two negative.
  Masks, overlays, boxes, colors and light states are exactly identical to the
  original trained implementation. Runtime imports the published package.
- Export preserves all 328 tensors and removes optimizer/scaler/RNG/history.
- Wheel build and isolated installation pass; installed outputs exactly match.
  The default CLI smoke and wheel license notices are verified. See
  verification/package_smoke.json.
- Desktop environment and observed smoke timings:verification/runtime_smoke.json.
  Its small sample is not a sustained throughput benchmark or an Orin result.
- Original training had 104 tests and an exact interrupted/resumed GPU proof.
  Those training tests are not bundled and are not claimed as ROS/device QA.

No live DDS,ROS2 build on this Windows host,Jetson CUDA,TensorRT,full-lap latency,
controller behavior or moving-vehicle acceptance was performed.

## Next Owner

1. Follow DEPLOYMENT.md offline and shadow commands before touching live topics.
2. Verify target Python/ROS/cv_bridge/NumPy ABI and TorchVision CUDA ROI/NMS ops.
3. Replay a held-out real bag with distant lanes,markings,cones,people and actual
   red/green lights;inspect every output and measure timestamp-to-publication lag.
4. Audit spatial resolution and complete annotations for distant lanes/small
   objects before merely extending the same training recipe.
5. Validate stopped-input/inference-failure watchdog behavior and avoidance ROI
   thresholds in controlled tests. Do not treat a detected cone as a proven trigger.
6. Only after accuracy,latency and safety gates pass should a separate reviewed
   integration change select A3 for driving. Keep the previous deployment ready.

## Resume And Rollback

Deployment shared_best.pth is not resumable optimizer state. The local training
workspace retains runs/shared_adaptation_v6/last.pth (epoch30) and best.pth (epoch24),
with optimizer87slots,AMP,RNG,cursors,code/data fingerprints and history. Use that
workspace's unchanged training entrypoint for exact resume;the normal plateau
guard intentionally prevents blindly continuing an exhausted recipe.

No automatic training or deployment promotion is included here. To roll back
inference,stop A3 and restore the previously recorded perception launch command.
Legacy YOLOP/A2 and their launch wiring were not modified by this publication.
