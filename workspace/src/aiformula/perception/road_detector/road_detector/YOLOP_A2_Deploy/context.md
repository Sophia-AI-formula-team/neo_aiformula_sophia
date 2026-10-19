# YOLOP_A2 Deployment Context

## Location and scope

Repository path:
`workspace/src/aiformula/perception/road_detector/road_detector/YOLOP_A2_Deploy`.
This is a sibling of the original `yolop`. Training remains in the separate
local YOLOP_A2 project; this package only performs inference.

## Model structure

`yolop_a2_runtime.runtime.DualPerceptionModel` loads two independent models:

- A1-derived lane-only YOLOP, kept at lane epoch 282.
- TorchVision Faster R-CNN MobileNetV3-FPN extended with traffic-cone class 91.
  Selected object epoch 20 follows cone-head, RPN, and hard-negative calibration.

Object training did not retrain the lane model. Cone colors and traffic-light
states come from pixels inside detected boxes. Lane masks are independent of
object masks. Obstacle masks rasterize selected bounding boxes; they are not
instance segmentation. Traffic lights are excluded from obstacle masks.

## Checkpoints

| File | Bytes | SHA256 |
| --- | ---: | --- |
| `weights/lane_epoch_282.pth` | 20921608 | `ac2336709034f89b8a7d69d0d7af158e677964699d41cc3520c08d16dc691bfe` |
| `weights/object_detector_usable.pth` | 77867961 | `5ef0574078014c0ac9d97c21539a31435e1c02499d57a944c02e07589a231fe0` |

`weights/manifest.json` records original hashes and calibration. Runtime
checkpoints exclude optimizer state and training metrics. These are ordinary
Git files, so a normal clone includes the weights without an extra download.

## ROS contract

Default input: `/aiformula_sensing/zed_node/left_image/undistorted`,
`sensor_msgs/Image`. Image outputs preserve the source header.

| Output | Type / encoding |
| --- | --- |
| `pub_annotated_mask_image` | `sensor_msgs/Image`, `bgr8` |
| `pub_mask_image` | `sensor_msgs/Image`, `mono8` |
| `/cone_bbox_image` | `sensor_msgs/Image`, `bgr8` |
| `/cone_mask` | `sensor_msgs/Image`, `mono8` |
| `/red_pixels_count` | `std_msgs/Int32` |
| `/traffic_light/state` | `std_msgs/String`: red, green, unknown |
| `/aiformula_perception/yolop_a2/detections` | `std_msgs/String` JSON |

Lane topic names are relative and depend on the node namespace. The existing
`perception/obsticle_avoidence` consumer applies its own trapezoid ROI and
pixel-count thresholds. Whole-mask counts alone do not prove avoidance triggers.

## Quality and limitations

Held-out cone metrics at score/IoU 0.50:

| Box area | Precision | Recall | F1 |
| --- | ---: | ---: | ---: |
| >=1024 px | 0.739 | 0.646 | 0.689 |
| >=4096 px | 0.853 | 0.879 | 0.866 |

Small/distant cones remain weak. Real-bag review found construction barriers
classified as people at bag-relative 102.243 s and 110.364 s, and a small
tree-canopy lane false positive at 110.364 s. This publication preserves the
verified model outputs; it does not fix those training-data limitations.

The seven-image smoke covers colored cones, a person, and red/green lights.
It is a regression check, not a real-world accuracy estimate. The 16-frame
bag review contained no traffic lights. Live vehicle acceptance is pending.
