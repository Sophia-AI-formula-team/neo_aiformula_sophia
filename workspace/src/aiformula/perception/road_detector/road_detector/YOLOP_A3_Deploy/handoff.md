# A3 V14 Deployment Handoff

## Selected Version

User selected **local V14 step 16,384**, not remote A1/FCOS or the best-gap step.
Tag `a3-v14` replaces the previous A3 package. Tag `a3-v6` preserves the former
PR #13 code and epoch24 weight together. Local training finished normally after
176.73 minutes, step_budget, exit 0. Automated quality qualification did not pass.

## Results

Fixed diagnostics, raw masks at 0.5; object confidence/IoU thresholds both 0.5.
32 real lane images, 16 empty, 594 geometry, 101 object images. These are repeated
diagnostic holdouts, not an independent final or vehicle test.

| Metric | Original A1 | Local V14 final |
|---|---:|---:|
| Real lane recall | 97.85% | 82.45% |
| Real lane IoU | 59.09% | 74.72% |
| Empty false-pixel rate | 0.398% | 0.187% |
| Geometry gap recall | 7.17% | 17.91% |
| Real object micro F1 | N/A, lane-only | 55.56% |

Training-sentinel gap recall is 91.32%, exposing poor holdout generalization.
Real person/car/light/cone recall is 39.13/64.71/53.33/24.24%; Unity person/light/
cone recall is 95.65/82.64/63.18%. Real small-object recall is 26.51%. Micro F1
is not mAP. The original A3 native reference real F1 was 56.14%.

Real recall >=95%, clean continuity and object nonregression remain unmet. Empty
FP <=0.35% and IoU improvement on these sentinels do not establish full acceptance.
The old >=85% dash-gap gate was not retested in its old definition; geometric
gap recall cannot be directly compared with v6's 73.57% dark-pixel proxy.
Full unrounded data: verification/training_summary.json. No claim of Orin FPS.

## Deployment Changes

- Exported only model tensors and provenance; no optimizer/scaler/RNG/history.
- Removed the experimental `core.` wrapper prefix with exact tensor equality.
- Added mandatory V14 input/restoration metadata validation at load time.
- Default lane output is RAW at 0.5, matching the reviewed local final images.
  Older v6 morphology is no longer enabled by default. This is intentional.
- Preserved one shared encoder, independent masks, ROS topics, headers and HSV
  color/light semantics. Legacy architecture loaders remain for older explicitly
  supplied checkpoints; they are not fallbacks for missing V14 metadata.
- Added AGENTS.md, release/rollback instructions and contract regression tests.

## Verification And Remaining Risks

See verification/runtime_smoke.json for fixed-frame reference/package parity and
single encoder counts; verification/package_smoke.json for tests, wheel, isolated
installation and CLI smoke. These files describe this exact weight, not v6.
The package tests use real weights with reduced-size CPU structural checks;
separate CUDA parity uses the full native input contract.

Exact parity uses deterministic algorithms with TF32 disabled. The default CLI
retains Torch's backend settings: its smoke frame differed by 8 lane pixels,
with identical detection count/semantic labels and maximum box shift 0.030 px.
This is recorded in package_smoke.json, not represented as bitwise CLI parity.

No live ROS2/DDS or colcon build was performed on Windows. No Orin/TensorRT,
sustained full-bag latency, light-state accuracy, watchdog or moving-vehicle
acceptance is implied. Callback failures emit no fabricated empty result; stale
data must be handled by a validated downstream watchdog.

## Next Owner

1. Verify weights, install runtime and run the offline demo/tests in DEPLOYMENT.md.
2. Use all shadow remaps before touching vehicle topics. Check actual publisher
   count, source timestamp age, frame_id, resolution and avoidance ROI behavior.
3. On Orin verify the JetPack/Torch/TorchVision/ROS ABI and CUDA ROI/NMS operators;
   measure sustained end-to-end timing on representative held-out bags.
4. For further training, audit complete real small-object assignment coverage and
   compare detector-only against joint training. For lanes, compare lane-only
   against joint training on independent maps/camera poses. Do not only add epochs.
5. Keep one backbone/one weight/two outputs. Do not conceal gap failures with
   thickening, broad closing, relaxed thresholds or validation relabeling.

## Resume And Rollback

This model-only weight is not an exact-resume checkpoint. Full local training
state remains separately in `YOLOP_A3/runs/contract_v14/local/a3/last.pth`, with
87 optimizer slots, four RNG states and source/data fingerprints. Do not restart
the exhausted recipe or edit fingerprinted sources from this deployment package.

With the vehicle stopped, restore the entire `a3-v6` tagged checkout and reinstall
its runtime, then verify its weight and shadow outputs. Code and weights must be
rolled back together. No vehicle launch switch is performed by either release.
