# A3 V14 Context

## Architecture

One MobileNetV3 encoder traversal feeds both the multiscale lane decoder and
TorchVision Faster R-CNN FPN/RPN/ROI detector. There is one loaded model and one
weight file, not two YOLO networks. Cone detection has an independent calibrated
predictor. This package does not contain the remote A1/FCOS experiment.

Runtime architecture tag: `a3_mobilenet_shared_fpn_contract_v14`.
Training wrapper tag: `a3_shared_contract_v14`, arm `a3`.
Export removes only `core.` from the 328 tensor names; all FP32 values are exact.
The underlying neural topology remains the shared independent-v4 layout; V14
identifies the new weights and explicit input contract, not another backbone.

Input is uint8 BGR, converted to RGB [0,1], ImageNet-normalized and resized with
TorchVision min_size=640/max_size=1280. Normalized padding is bottom/right to a
multiple of 32. Lane logits are cropped to the resized image, bilinearly restored
to source resolution, then thresholded at 0.5. No morphology is applied by default.
Boxes are restored to the original image coordinates. Do not replace this with
A1's square letterbox or silently shrink input dimensions.

Lane targets are continuous/amodal boundaries, ignoring road symbols, zebra
crossings and occluders. These are intended semantics, not current guarantees.
The obstacle mask is filled person/car/cone rectangles; it excludes lights.
Cone color and light red/green/unknown state use crop HSV rules, not separately
learned state classes. ROS outputs remain independent and preserve image headers.

## Provenance

Release tag `a3-v14`, package version 0.3.0; prior package tag `a3-v6`.
Selected local final snapshot: `contract_v14/local/a3/snapshots/16384.pth`.
Source SHA256: `aad5df29a3a97f959ac7f582b2d45255bd23cb2436c7e0820eb69549ad6d48a9`.
Model-only SHA256: `a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c`.
The source still has `deployment_candidate=false`; user-selected publication is
not an assertion that automated quality gates passed.

V14 repaired the native input contract, added 1,200 RGB renders/200 poses with
near-field/near-horizontal views, and retained real/empty/object replay. The
synthetic pool has 4,866 RGB/811 poses; real lane/empty/real object replay has
987/235/1,247 records. Holdouts remained 594 geometry/32 real/16 empty/101 object
images. Geometry-gap labels differ from the old dark-pixel proxy. Test was unused.

Local A3 reached 16,384 steps in 176.73 minutes and stopped normally. The remote
route starts from A1 with a new FCOS head and differs in architecture,
initialization and input contract; host comparisons are not pure GPU experiments.

## Boundaries

Only this deployment directory changes in neo. Original YOLOP/A1/A2, vehicle
launch wiring, external Unity, training sources and optimizer checkpoints stay
unchanged. No datasets, bags, credentials, training loops or old ONNX graphs are
distributed. MIT and TorchVision BSD-3-Clause notices are retained.

The runtime wheel installs code, not bundled weights or the ROS overlay. Keep
this checkout beside the installed runtime and provide an absolute weights path.
Read DEPLOYMENT.md, handoff.md and AGENTS.md before integration.
