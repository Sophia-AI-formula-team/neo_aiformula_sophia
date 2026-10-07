# A3 Context

## Objective And Architecture

The user needs lane completion and object perception without A2's two encoder
passes. A3 uses one MobileNetV3 body with shared multiscale features. The object
path is TorchVision Faster R-CNN style FPN/RPN/ROI;the lane decoder consumes
intermediate body features from the same traversal. This is not YOLOP+YOLO and
not the obsolete initial A3 YOLOP/SSD candidate.

Architecture tag:`a3_mobilenet_shared_fpn_independent_v4`.
The selected training stage is v6;the architecture tag describes the checkpoint
layout,not its training stage. Both names are intentional.

The model transforms input with aspect-preserving min_size640/max_size1280 and
returns masks/boxes at the original image resolution. The lane head targets
continuous/amodal lane boundaries while ignoring visible dashes,markings and
occluders. This is the intended semantics,not a guarantee of current accuracy.
Existing lane morphology/component filtering remains identical to validation.

Cone color and traffic-light state are crop-based HSV semantics. Detection and
lane outputs are independent. The obstacle mask contains filled boxes,not pixel
segmentation. Traffic lights are excluded from avoidance masks.

## Provenance

Selected checkpoint:shared_adaptation_v6 epoch24,9600updates. The full run ended
normally at30sampled epochs/12000updates after10956.672seconds,exit0,plateau.
Each sampled epoch is400lane updates with200object updates,not a full data pass.

v6 warm-started from lane_preservation_v5 best3 with a new optimizer. MobileNet
body12..16 and lane/RPN/ROI heads trained jointly. Early body,FPN and norm buffers
were preserved. Learning rates were2.5e-6/1.25e-6/5e-7 for lane/object/shared tail.
Correct-GT lane/person/cone reference preservation each had weight5. All teachers
and deterministic CPU ROI training machinery are absent from this package.

Lane data:8584train/1867validation,mixing manual real labels and asymmetric Unity
RGB/completed-lane masks. Object data:6062train/1488validation. Validation was
used repeatedly for model selection;it is not an independent final test set.
No data,images,bags,training scripts or optimizer checkpoints are uploaded here.

## Boundaries

A1,A2,and existing main perception/launch files are unchanged. This upload is
an independent experimental package. It does not supersede A2 PR11 or integration
PR9 and does not automatically enable A3 on a vehicle.

The model-only file is79,781,607bytes and is loaded once. Manifest SHA256 and
the source checkpoint hash provide lineage. Do not substitute the old SSD
shared_best.pth,old ONNX graph,or the last epoch30 checkpoint.

MIT and TorchVision BSD-3-Clause notices are retained. Training-source assets
keep their own licensing;those assets are not distributed in this package.
