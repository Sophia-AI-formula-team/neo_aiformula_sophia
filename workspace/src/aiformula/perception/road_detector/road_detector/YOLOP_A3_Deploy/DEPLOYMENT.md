# A4 Deployment Guide

## Vehicle profiles (not part of the original tag a4)

Build both standalone archives from this checkout:

```sh
python tools/package_vehicle.py --out /path/to/output
```

Extract into separate `A4/` and `A4_lite/` folders under
`/home/control/aiformula_runtime/` (or set `a4_bundle_parent`).
Each contains the same runtime and verified A4 checkpoint. The included
`profile.json` selects plain or lean inference; do not edit it to change a
bundle's identity. The ROS adapter is included; an environment with compatible
`torch`, `torchvision`, `cv2`, `numpy`, `rclpy` and `cv_bridge` is required
at `venv/bin/python3`. These are source/weight bundles, not JetPack images
or prebuilt Python environments. Do not reuse the V14 Foxy Python 3.8 venv.

Read-only identification on the vehicle: inspect `/proc/<PID>/cmdline`
and the startup log, then hash the checkpoint from the actual process
arguments. Folder and ROS node names alone do not identify the model.
V14 tag: `a3-v14`, exported checkpoint SHA256
`a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c`
(architecture `a3_mobilenet_shared_fpn_contract_v14`). The original training
snapshot with SHA starting `aad5df29` is not the packaged file.
A4/A4_lite share the A4 hash below; distinguish them by the validated profile
and logged `lean_runtime` setting. Archived `a3-v6`, `a3-v14`, `a3-v76`
and `a4` tags remain immutable.

```sh
# Prints Python, checkpoint hash, profile and remaps; does not start ROS.
./venv/bin/python3 run_vehicle.py --profile A4_lite --identity
# Offline bag test in an isolated ROS_DOMAIN_ID: all seven outputs are shadow.
./venv/bin/python3 run_vehicle.py --profile A4_lite --use-sim-time true
```

The neo launcher can select either bundle with `yolop_profile:=A4` or
`yolop_profile:=A4_lite` after shadow acceptance. Leave the existing V14
installation in place; select `yolop_profile:=V14` to roll back. The
runner rejects pre-existing publishers on all seven selected outputs,
but graph discovery is not a process lock: start only one profile.

| Vehicle output (matches neo main V14 remaps) | Type |
|---|---|
| /aiformula_perception/road_detector/mask_image | Image mono8 |
| /aiformula_visualization/road_detector/annotated_mask_image | Image bgr8 |
| /cone_mask | Image mono8 |
| /cone_bbox_image | Image bgr8 |
| /aiformula_perception/yolop_a3/detections | String JSON |
| /aiformula_perception/yolop_a3/traffic_light/state | String |
| /aiformula_perception/yolop_a3/red_pixels_count | Int32 |

Input is `/aiformula_sensing/zed_node/left_image/undistorted`. Four image
headers and detection JSON source timestamp/frame ID are preserved.
LaneLinePublisher consumes the standard lane mask. Avoidance consumes
the two existing cone topics. Optional semantic modules configured for
global `/traffic_light/state` or `/red_pixels_count` are **not** implicitly
rewired; neo's vehicle launcher uses the namespaced perception outputs.

### Lite mode

The ROS boolean startup parameter `lean_runtime` defaults to false. True
precomputes the frozen encoder normalization coefficients once in FP32,
transfers the default binary mask instead of a float probability image, and
omits the unpublished combined preview. The four published image streams,
objects, signal outputs and source headers remain active. No frame skipping,
lower resolution, detector cutoff changes, CUDA Graph or model conversion.
Python users can use `PerceptionModel(weights, lean_runtime=True)`;
`predict(image)` still returns all overlays. No hardware speedup is assumed.

Use a separate candidate checkout/environment. Verify the actual running
interpreter and installed package, not just the launch filename: the older neo
launcher points to a V14 installation by default. Do not apply this to V14.
Check the A4 weights with `python tools/verify_weights.py`, then compare
without ROS, on a directory of representative PNG/JPEG camera frames:

```sh
python tools/compare_lean.py --images /path/to/frames --out lean_orin.json
```

The tool uses one model at a time, compares every published content field
except timing, and reports warm model-to-host response times. Test the ROS
candidate only with the seven shadow remaps below, adding
`-p lean_runtime:=true` to that node's ROS arguments. Confirm its startup log;
this parameter is not a top-level argument of the legacy launch file. Restore
the default by restarting with false. Measure actual camera timestamp to output,
misses including drops, longest gap, GPU/CPU, power, temperature and memory
under the same load on Orin for at least 10 minutes per mode. Prefer bag
playback before connecting to a vehicle stack. Do not promote on offline
timing alone.

## Status And Environment

This is the A4 package (version 4.0.0, tag a4), renamed from a3-v76 with identical
weights and inference behavior. First run it on isolated shadow
topics. Publication does not modify main launch wiring or start/stop any ROS node.
The bundled ROS wrapper runs single-frame inference and says so at startup.
For calibrated temporal inference use predict_frame and TEMPORAL.md.

**Model/API publication is complete; real-camera temporal integration is not.**
Single-frame processes every video frame independently, as the V14 ROS node did,
but uses the new image-model weights. Unity temporal demos had accurate camera
poses; this image-only ROS wrapper does not receive that geometry. Starting it
does not reproduce the temporal demo effect. See [MODE_GUIDE_ZH.md](MODE_GUIDE_ZH.md)
and [integration issue #34](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/issues/34).
Existing folder/Python/ROS names are retained as compatibility interfaces.

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

Use a clean dedicated deployment checkout. Fetch/check out the release tag,
then from the repository root:

```sh
git fetch origin tag a4
git switch --detach a4
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
b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3
```

The file is V76 aligned step 1024, including its frozen image model. It has one
architecture tag, an explicit input contract and one shared state dictionary. It contains
no optimizer/AMP/RNG state and cannot be used to resume the original training
trajectory. Training resume checkpoints remain in the separate training workspace.

The default lane output is raw softmax foreground >=0.5. Closing and connected
component filtering are disabled. Do not enable the old morphology or change
min_size640/max_size1152 while claiming parity with the selected V76 model.

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
This FCOS model has exactly those four classes; only1/3/91 fill cone_mask.
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

To restore the previous A3 package, stop its node, then in the clean dedicated
deployment checkout:

```sh
git fetch origin tag a3-v14
git switch --detach a3-v14
python -m pip install --no-deps --force-reinstall "$A3"
python "$A3/tools/verify_weights.py"
```

Restart the previously recorded shadow command and verify publishers/freshness.
Roll back the entire runtime and weight together, not only shared_best.pth. The
v14 tag preserves the previous runtime, input contract and weight together.
Alternatively restart the recorded legacy/A2 command. No launch file needs
reverting because this publication changes none.

## Failure Behavior

Missing/incompatible weights fail startup. A callback error is logged and emits
no new result for that frame;it does NOT publish a fabricated empty mask or a
vehicle stop command. Downstream stale-data watchdog/fail-safe behavior must be
validated separately before live use. A stopped camera or dead inference process
must not leave the vehicle acting on indefinitely stale perception.
