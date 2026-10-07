# Deployment Guide

## Status And Environment

This is an offline/shadow candidate. Do not connect its outputs to moving-vehicle
control until the remaining gates in handoff.md pass. The upload does not modify
main launch wiring or stop/start any existing ROS node.

Tested reference:Windows,x86-64,Python3.12.10,Torch2.11.0+cu130,
TorchVision0.26.0,RTX4070Ti. Package requires Python>=3.10,<3.14.
Stock ROS2 Foxy/Python3.8 is NOT compatible with this Python package.
Use a ROS/Python environment whose rclpy and cv_bridge match the interpreter.

The dependency pins in pyproject.toml describe the desktop reference,not a
Jetson installation recipe. On Orin first select NVIDIA's matching
JetPack/PyTorch/TorchVision binaries and a compatible NumPy/cv_bridge stack.
Do not let pip replace the platform Torch or system OpenCV blindly.
See [JETSON.md](JETSON.md). Other version combinations are unverified.

## Checkout And Install

After fetching/checking out this PR's branch,from the repository root:

```sh
export A3="$PWD/workspace/src/aiformula/perception/road_detector/road_detector/YOLOP_A3_Deploy"
python -c "import sys,torch,torchvision,cv2,numpy; print(sys.version); print(torch.__version__,torchvision.__version__,torch.cuda.is_available())"
python -m pip install --no-deps "$A3"
python "$A3/tools/verify_weights.py"
```

Build tooling needs setuptools>=77 and wheel. For an offline installation,
preinstall those and add `--no-build-isolation`. This package does not install
CUDA,JetPack,ROS2,or a compatible platform Torch wheel for you.
Do not rename a different checkpoint to shared_best.pth;verify its checksum.

Bundled SHA256:
```text
12f8202587f33972ec09f6f513817ec6a30aa35310256fd356119d547d4edc02
```

The file has one architecture tag and one shared state dictionary. It contains
no optimizer/AMP/RNG state and cannot be used to resume the original training
trajectory. Training resume checkpoints remain in the separate training workspace.

## Offline Smoke

```sh
python "$A3/tools/demo.py" --image /absolute/path/frame.jpg --output /absolute/path/new-a3-output --device cuda
python -m pytest "$A3/tests" -q
```

The tests require pytest. They exercise the actual bundled model,mask contracts,
single encoder traversal,and a mocked ROS callback. They do NOT start DDS.
Check real distant lanes,dashes,zebra crossings,arrows,empty road views,small
colored cones,people and light states on your own held-out images as well.

## ROS2 Build

Use the same compatible Python environment for runtime and ROS wrapper. After
sourcing your installed ROS distribution,verify:

```sh
python -c "import rclpy,cv_bridge; from yolop_a3_runtime import PerceptionModel; print(PerceptionModel.__module__)"
```

Expected module:`yolop_a3_runtime.shared_runtime`.
Build only this wrapper in a separate overlay workspace,not inside the nested
road_detector package:

```sh
mkdir -p "$HOME/a3_shadow_ws"
cd "$HOME/a3_shadow_ws"
colcon build --base-paths "$A3/ros2" --packages-select yolop_a3_perception
source install/setup.bash
```

If an environment's colcon executable selects a different Python,resolve that
interpreter/ABI mismatch before running the node. Changing PYTHONPATH alone does
not repair an incompatible cv_bridge or rclpy binary.

## Shadow Run

Use all of these output remaps together so the existing vehicle consumers cannot
accidentally use the candidate:

```sh
ros2 run yolop_a3_perception yolop_a3_node --ros-args \
  -p weights:="$A3/weights/shared_best.pth" -p device:=cuda \
  -r pub_mask_image:=/a3_shadow/lane_mask \
  -r pub_annotated_mask_image:=/a3_shadow/lane_overlay \
  -r /cone_mask:=/a3_shadow/obstacle_mask \
  -r /cone_bbox_image:=/a3_shadow/object_overlay \
  -r /red_pixels_count:=/a3_shadow/red_pixels_count \
  -r /traffic_light/state:=/a3_shadow/traffic_light_state \
  -r /aiformula_perception/yolop_a3/detections:=/a3_shadow/detections
```

Default input:`/aiformula_sensing/zed_node/left_image/undistorted`.
Override it with `-p input_image_topic:=/your/image_topic`.
Input QoS is best-effort,keep-last,depth1. Image output publishers use depth1.
Use `ros2 topic info -v`,`ros2 topic hz` and image viewers to verify actual
publisher identity,QoS,source timestamps,frame IDs,dimensions and rate.

## Topic Contract

| Default topic | Type / encoding | Meaning |
|---|---|---|
| pub_mask_image | sensor_msgs/Image,mono8 | Lane-only 0/255 mask |
| pub_annotated_mask_image | sensor_msgs/Image,bgr8 | Lane overlay |
| /cone_mask | sensor_msgs/Image,mono8 | Filled person/car/cone rectangles,0/255 |
| /cone_bbox_image | sensor_msgs/Image,bgr8 | Object boxes on source RGB |
| /red_pixels_count | std_msgs/Int32 | Crop-derived red evidence count |
| /traffic_light/state | std_msgs/String | red,green,or unknown;red takes precedence |
| /aiformula_perception/yolop_a3/detections | std_msgs/String | JSON detections,color/state,timing,source header |

All four output Image messages preserve the input header. Detection JSON includes
the source sec/nanosec and frame_id. Int32/String state topics have no ROS Header;
do not assume they independently encode freshness.

Object class IDs follow COCO plus cone:person1,car3,traffic_light10,traffic_cone91.
The detector may return other COCO categories in JSON;only1/3/91 fill cone_mask.
The avoidance consumers oa_neo*.py subscribe to cone_mask/cone_bbox_image and
apply their own ROI/pixel threshold (oa_neo.py defaults1000/500 enter/exit).
Whole-image mask area or a detected box does not prove avoidance activation.

## Controlled Integration And Rollback

Only after offline/shadow and target-device acceptance,with the vehicle stopped:

1. Record the current deployment version,launch command,topics and weights.
2. Stop only the perception publisher being replaced;never run legacy/A2/A3 on
   the same live mask or light topics simultaneously.
3. Run A3 with its default object topics and explicitly remap
   `pub_mask_image:=/aiformula_perception/road_detector/mask_image`.
   This matches main's launchers/config/topic_list.yaml and lane_line_publisher.
4. Keep its debug overlay separate,for example
   `pub_annotated_mask_image:=/a3_debug/lane_overlay`;do not collide with the
   lane_line_publisher's own annotated image.
5. Verify there is exactly one intended publisher and inspect masks,source
   timestamps,TF/intrinsics,avoidance ROI and controlled light-state behavior
   before enabling any motion. No automatic controller switch is provided.

To roll back:stop the A3 node,restart the previously recorded legacy/A2 command,
and verify publisher identities and output freshness again. No repository launch
file needs reverting because this package changes none.

## Failure Behavior

Missing/incompatible weights fail startup. A callback error is logged and emits
no new result for that frame;it does NOT publish a fabricated empty mask or a
vehicle stop command. Downstream stale-data watchdog/fail-safe behavior must be
validated separately before live use. A stopped camera or dead inference process
must not leave the vehicle acting on indefinitely stale perception.
