# YOLOP A4 Deployment

Current release: **A4**, tag **a4**, package version **4.0.0**.

**当前 ROS 节点逐帧独立识别，并未启用时序增强。** 这与之前 V14 的运行方式相同，
但使用的是本次新权重，不是退回 V14。时序模块和接口已发布，实车的标定与同步位姿
尚未接入；因此直接启动 ROS 节点不能复现 Unity 演示中的时序增强效果。
详见[单帧与时序模式说明](MODE_GUIDE_ZH.md)。

A4 renames the previously published a3-v76 release without retraining, changing
weight bytes or enabling temporal inference. The existing YOLOP_A3_Deploy folder,
yolop-a3-runtime distribution, yolop_a3_runtime imports and ROS package/topic names
are retained to keep current installation and caller contracts intact.

One MobileNetV3 Large encoder, one FCOS detector, one continuous-lane decoder,
one 49-parameter temporal gate, **one model-only weight file**.
Historical tags a3-v76, a3-v14 and a3-v6 remain immutable.

## Choose The Input Mode

| Entry | Inputs | Lane behavior |
|---|---|---|
| `predict(image)` | One uint8 BGR image | Exact frozen single-frame image model; clears temporal history |
| `predict_frame(image, ...)` | Image, sequence ID, capture time, calibrated metric ground-plane projection | Previous raw lane probability aligned into the current frame |
| Bundled ROS node | Image topic | Explicit **single-frame mode**; logs this at startup |

The ROS node does not guess camera calibration or derive motion from timestamps.
To use A4 temporal fusion, integrate the documented Python frame contract with
your calibrated, timestamp-aligned geometry source. See [TEMPORAL.md](TEMPORAL.md).
Objects always use the current image; the temporal gate does not track objects.

Single-frame means every video frame is processed independently, not that only
one image is processed. The Unity temporal comparisons had accurate camera poses;
the current ROS wrapper receives images only. The model and API are packaged,
but the real-camera temporal input integration is not complete ([issue #34](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/issues/34)).

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
