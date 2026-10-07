# YOLOP A3 Shared Deployment

**Experimental candidate, not approved for autonomous driving.** This package
publishes the selected `shared_adaptation_v6` epoch 24 model. It passes 22 relative
protection checks but still fails the A1 lane-recall/empty-frame false-positive
and real-object recall acceptance criteria. It does not replace vehicle startup.

One MobileNetV3 encoder traversal feeds an FPN/RPN/ROI object detector and an
amodal lane decoder. There is **one checkpoint**, not an A1 model plus another
YOLO detector. The public `PerceptionModel` now points to this shared architecture,
not the obsolete initial A3 SSD experiment.

## Contents

- `weights/shared_best.pth`: 79,781,607 bytes, 328 tensors, FP32, selected epoch 24.
- `weights/manifest.json`: SHA256, architecture and source-checkpoint provenance.
- `yolop_a3_runtime/`: inference only; no datasets, teachers or training loops.
- `ros2/yolop_a3_perception/`: ROS2 adapter with separate lane/object outputs.
- `tools/demo.py`, `tools/verify_weights.py`, `tests/`: offline checks.
- [DEPLOYMENT.md](DEPLOYMENT.md): installation, shadow mode, topics, switching and rollback.
- [context.md](context.md) and [handoff.md](handoff.md): architecture, results, next steps.
- [JETSON.md](JETSON.md): target environment and outstanding Orin acceptance.

Keep this repository directory alongside its weights. Installing the Python wheel
installs the runtime only; it does not embed/download the checkpoint or ROS wrapper.

## Offline Quick Start

In an existing compatible Python 3.10-3.13 environment with Torch/TorchVision,
NumPy and OpenCV already installed:

```sh
cd workspace/src/aiformula/perception/road_detector/road_detector/YOLOP_A3_Deploy
python -m pip install --no-deps .
python tools/verify_weights.py
python tools/demo.py --image /absolute/path/frame.jpg --output /absolute/path/new-a3-demo --device cuda
python -m pytest tests -q
```

Use `--device cpu` for functional CPU smoke. GPU latency is not implied by CPU
success. `demo.py` refuses existing output folders and writes RGB, combined overlay,
separate lane/obstacle masks, individual overlays and detection JSON.

```python
import cv2
from yolop_a3_runtime import PerceptionModel

model = PerceptionModel("/absolute/path/weights/shared_best.pth", device="cuda")
result = model.predict(cv2.imread("frame.jpg"))  # uint8 HxWx3 BGR
cv2.imwrite("lane.png", result.lane_mask)
cv2.imwrite("obstacles.png", result.obstacle_mask)
cv2.imwrite("overlay.jpg", result.combined_overlay)
```

Both masks are native-input-resolution uint8 0/255 images. The obstacle mask is
filled person/car/cone boxes, not instance segmentation. Lights are excluded.
Cone color and red/green/unknown light state use crop-based color rules, not
separate learned state classes.

## Evidence And Limits

The run ended normally after 30 sampled epochs and 12,000 updates; best epoch 24 was retained.
Real lane recall improved 77.12% -> 77.24%, not a resolution of distant-lane misses.
Real person/light/cone recall is 45.07%/49.37%/20.10%. Full selection metrics and
failed absolute acceptance flags are in [training_summary.json](verification/training_summary.json).

20 package tests pass. The wheel build, isolated installation and default CLI
smoke also pass; see [package_smoke.json](verification/package_smoke.json).
Eight real/Unity/negative diagnostic views have exactly matching masks, overlays,
boxes and semantic outputs against the original trained implementation, with one
encoder visit per frame. See
[runtime_smoke.json](verification/runtime_smoke.json). These are desktop
packaging tests, not independent accuracy tests, live ROS2 or Orin acceptance.

Original YOLOP/A1/A2,vehicle launch files and training workspaces are unchanged.
No ONNX/TensorRT engine or second checkpoint is included. Retained licenses:
first-party/original YOLOP MIT and TorchVision BSD-3-Clause notices.
