# A4 Context

Release identity: A4 / 4.0.0 / tag a4. This is a naming clarification of a3-v76,
not a new training run. Weight bytes, numerical inference, source architecture
and provenance are unchanged. Legacy folder, distribution, Python and ROS names
remain compatible. See MODE_GUIDE_ZH.md for the actual deployment status.

## Selected Source

The selected snapshot is V76 aligned step 1024, built on the frozen V73 image model.
Architecture: `a3_shared_fcos_temporal_probability_v76`.
Source SHA256: `1fc29a6cd4bb5de3390570ce805adca5167303176cecdd867eddb6d36752f1ca`.
Export SHA256: `b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3`.

The user selected this version as stable on 2026-11-06 and authorized publication
and merge. The source's automated direction gate remains recorded as failed
(gap improvement below 2 percentage points); export does not rewrite that history.

## Runtime

The shared MobileNetV3 Large features feed a multiscale lane decoder and a compact
four-class FCOS detector (width 32, depth 2). This intentionally replaces V14's
Faster R-CNN/RPN/ROI layout. Old checkpoints require the old tagged package;
there is no legacy checkpoint fallback in this version.

The serialized input contract is min_size=640/max_size=1152, RGB [0,1] with
ImageNet normalization and bottom/right normalized-zero padding to 32.
Lane logits are resized to padded shape, cropped to resized content, then
bilinearly restored to native shape with align_corners=False before thresholding.
The temporal gate works on native floating probabilities, not binary masks.
The combined checkpoint has 341 tensors and 3,403,740 parameters (49 temporal).

Each stream stores only the previous raw probability, capture timestamp and
metric plane projection. No labels, future images or stored lane map are inputs.
History is cleared by explicit reset or independent-image prediction and bypassed
on first frame, changed sequence/size or non-increasing/>0.25s capture time.
Object output is never temporally modified.

## Integration Boundary

The Python input/output API remains compatible through PerceptionModel;
SharedFPNPerception is only an import alias, not a second model.
predict_frame is a new explicit API. The bundled ROS image-only wrapper runs the
single-frame base and reports that fact. It does not fabricate motion.
Source Headers are independently copied into every image output.

This release updates only the A3 deployment package and its handoff records.
Legacy road_detector, default launchers, A1/A2, motor/controller/watchdog and
source training/Unity projects remain unchanged.
MIT and TorchVision BSD-3-Clause notices are retained.
