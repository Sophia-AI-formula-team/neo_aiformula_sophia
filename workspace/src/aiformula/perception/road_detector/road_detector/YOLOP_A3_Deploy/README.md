# YOLOP A3 V76 Deployment

User-selected stable release: **a3-v76**, Python package **0.76.0**.

One MobileNetV3 Large encoder, one FCOS detector, one continuous-lane decoder,
one 49-parameter temporal gate, **one model-only weight file**.
This replaces the old A3 deployment package; a3-v14 and a3-v6 remain immutable.

## Choose The Input Mode

| Entry | Inputs | Lane behavior |
|---|---|---|
| `predict(image)` | One uint8 BGR image | Exact frozen single-frame image model; clears temporal history |
| `predict_frame(image, ...)` | Image, sequence ID, capture time, calibrated metric ground-plane projection | Previous raw lane probability aligned into the current frame |
| Bundled ROS node | Image topic | Explicit **single-frame mode**; logs this at startup |

The ROS node does not guess camera calibration or derive motion from timestamps.
To use V76 temporal fusion, integrate the documented Python frame contract with
your calibrated, timestamp-aligned geometry source. See [TEMPORAL.md](TEMPORAL.md).
Objects always use the current image; the temporal gate does not track objects.

## Output Contract

- Native-resolution lane mask, uint8 0/255; raw threshold 0.5, no morphology.
- Person/car/cone box mask, uint8 0/255; traffic lights are excluded from this mask.
- Boxes and JSON detections; person=1, car=3, traffic light=10, cone=91.
- Cone colors and light states use existing HSV rules, not learned subclasses.
- Separate lane/object/combined overlays and explicit `temporal_status`.

Input resize is min side 640, max side 1152, then bottom/right padding to a
multiple of 32. A 1280x720 image becomes 1137x640, padded to 1152x640.
Do not use the architecture's old experimental 360/640 defaults.

## Start Here

1. [DEPLOYMENT.md](DEPLOYMENT.md): install, verification, ROS shadow topics, rollback.
2. [TEMPORAL.md](TEMPORAL.md): geometry, reset and streaming API.
3. [handoff.md](handoff.md): measured results and next-owner tasks.
4. [JETSON.md](JETSON.md): target-device environment and performance checks.
5. [weights/manifest.json](weights/manifest.json): exact source/export hashes.

The wheel contains runtime code. Keep this package checkout for the weight,
verification tools and separate ROS overlay. No dataset or training code is shipped.
