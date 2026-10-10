# YOLOP A3 Shared Deployment

**Tag `a3-v14`: local V14 step 16,384, selected by the user for deployment testing.**
This replaces the previous A3 package, preserved at tag `a3-v6`. It is a research
snapshot, not approved for autonomous driving; recall, continuity and object
acceptance are still incomplete. It does not replace vehicle startup wiring.

One MobileNetV3 encoder traversal feeds an FPN/RPN/ROI object detector and an
amodal lane decoder. There is **one checkpoint**, not an A1 model plus another
YOLO detector. The public `PerceptionModel` now points to this shared architecture,
not the obsolete initial A3 SSD experiment.

## Contents

- `weights/shared_best.pth`: 79,763,687 bytes, 328 tensors, FP32, local final step 16,384.
  The historical filename is retained; this is the requested final snapshot, not
  an automatically qualified best checkpoint. Only one weight file is included.
- `weights/manifest.json`: SHA256, architecture and source-checkpoint provenance.
- `yolop_a3_runtime/`: inference only; no datasets, teachers or training loops.
- `ros2/yolop_a3_perception/`: ROS2 adapter with separate lane/object outputs.
- `tools/demo.py`, `tools/verify_weights.py`, `tests/`: offline checks.
- [DEPLOYMENT.md](DEPLOYMENT.md): installation, shadow mode, topics, switching and rollback.
- [context.md](context.md), [handoff.md](handoff.md), [AGENTS.md](AGENTS.md): architecture, results and maintainer instructions.
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

V14 defaults to the **raw lane mask at 0.5**, with no closing or component
filtering. Input is aspect-preserving min_size=640/max_size=1280, not 640-square
letterbox. The loader validates the serialized input/restoration contract.

## Evidence And Limits

The local run completed 16,384 updates in 176.73 minutes, exit 0 (step budget).
Fixed diagnostics: real lane recall **82.45%**, IoU **74.72%**, empty false pixels
**0.187%**, geometry-gap recall **17.91%**. Training gap recall was 91.32%, exposing
a large generalization gap. Real object micro F1 is **55.56%**; person/light/cone
recall is **39.13%/53.33%/24.24%**. These are not full acceptance or mAP.
The geometry-gap definition differs from the old v6 dark-pixel proxy; do not
compare 17.91% directly with the old 73.57%. Full measurements and failed/pending
gates are in [training_summary.json](verification/training_summary.json).

Package tests cover the bundled checkpoint, input contract, raw masks, single
encoder traversal and mocked ROS callback. Export/parity, wheel/isolated install
and CLI results are recorded in [runtime_smoke.json](verification/runtime_smoke.json)
and [package_smoke.json](verification/package_smoke.json). They are desktop
packaging evidence, not live ROS2, Orin throughput or vehicle acceptance.

Original YOLOP/A1/A2,vehicle launch files and training workspaces are unchanged.
No ONNX/TensorRT engine or second checkpoint is included. Retained licenses:
first-party/original YOLOP MIT and TorchVision BSD-3-Clause notices.
