# YOLOP_A2 Deploy

Inference-only lane segmentation and object detection, installed beside the
existing `../yolop`. Adding this directory does not select it in the vehicle
launch configuration. Training data/code, optimizer state, virtual environments,
and generated bag/video outputs are not included.

The included checkpoints are `weights/lane_epoch_282.pth` (lane epoch 282) and
`weights/object_detector_usable.pth` (selected object epoch 20). Thresholds
default to 0.50. See [context.md](context.md) for architecture, topics, hashes,
and limitations, and [handoff.md](handoff.md) for verification and integration.
The package's existing MIT notice is retained in [LICENSE](LICENSE).

## Install and run

From this directory, use Python 3.10-3.13. Declared versions are torch 2.11.0
and torchvision 0.26.0. Validation used Python 3.12, Windows, and an RTX 4070 Ti.
ARM/Jetson, older ROS Python environments, and live ROS have not been validated.

```bash
python -m pip install -e .
python tools/demo.py --source /path/to/images --output inference/demo --device auto
```

Outputs include separate binary lane/obstacle masks, overlays, detection JSON,
a summary, and a contact sheet. Checkpoints use restricted `weights_only=True`
loading. Keep the weights directory: weights are not bundled into the wheel.

## ROS2

Use the same Python environment for ROS2 and the runtime. From this directory:

```bash
python -m pip install -e .
colcon build --base-paths ros2/yolop_a2_perception --symlink-install
source install/local_setup.bash
ros2 run yolop_a2_perception yolop_a2_node --ros-args \
  -p lane_weights:="$PWD/weights/lane_epoch_282.pth" \
  -p object_weights:="$PWD/weights/object_detector_usable.pth" \
  -p input_image_topic:="/aiformula_sensing/zed_node/left_image/undistorted"
```

The wrapper is nested under the existing road_detector package to keep this
deployment beside yolop. Build it explicitly using the command above; an outer
colcon scan may not discover the nested package. Select only one lane-topic
publisher in the integration launch before running the pipeline.

## Verification and bag tools

```bash
python -m pip install -e '.[test]'
python -m pytest -q
python tools/smoke_test.py --source /path/to/seven-case-fixtures --device cuda:0
```

Smoke fixtures are not distributed here. Expected image stems are
`01_red_cones`, `02_yellow_cone`, `03_green_cones`, `04_blue_cones`,
`05_person`, `06_red_traffic_light`, and `07_green_traffic_light`.
For arbitrary images, use demo.py.

Offline bag tools require an external ROSBagel checkout and
`python -m pip install rosbags==0.11.1` in the inference environment:

```bash
python tools/rosbag_video.py --bag /path/to/bag \
  --rosbagel-root /path/to/ROSBagel --output inference/lap.mp4 --device cuda:0
python tools/rosbag_lane_masks.py --bag /path/to/bag \
  --rosbagel-root /path/to/ROSBagel --output inference/lane_masks --device cuda:0
```

Video uses all images in topic order with a constant average FPS, not variable
per-frame timing. Lane-mask names and the manifest retain source timestamps.
