# Jetson Orin Validation Boundary

Target:Jetson AGX Orin. RAM,JetPack,power mode and ROS/Python versions must be
recorded on the target. No new Orin measurement or TensorRT engine is bundled.

Use a PyTorch wheel compatible with the actual JetPack and a matching TorchVision
build with CUDA operators. NVIDIA's [Jetson PyTorch installation guide](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
documents the JetPack-specific wheel requirement. Do not install the desktop
CUDA13.0 wheel pins over an existing Jetson environment.

The package needs Python>=3.10,<3.14. Stock Foxy/Python3.8 cannot import this
package. Resolve that environment boundary explicitly;do not mix binary rclpy or
cv_bridge modules from a different interpreter. Preserve a platform-compatible
NumPy/OpenCV ABI and install this runtime with --no-deps only after preparing it.
Other than the recorded desktop versions,version combinations are unverified.

Before running inference,verify CUDA and the TorchVision extension:

```sh
python -c "import torch,torchvision; from torchvision.ops import nms; print(torch.__version__,torchvision.__version__); print(nms(torch.tensor([[0.,0.,2.,2.]],device='cuda'),torch.ones(1,device='cuda'),0.5))"
```

Then run the checksum check,package tests,offline demo and isolated shadow node
from DEPLOYMENT.md. Tests use reduced-size CPU inputs for fast structural QA;
the deployment runtime still uses min_size640/max_size1152 unless deliberately
changed and revalidated. Do not silently reduce that size and reuse accuracy claims.

Old A3 SSD/Faster R-CNN ONNX exports are incompatible with this FCOS checkpoint.
No ONNX or TensorRT artifact is provided here. A future export must include the
correct transform,shared features,lane decoder,FCOS and detection postprocessing,
and compare both outputs against this exact model. Do not reuse a desktop engine.

Record module/RAM,JetPack,driver,Torch/TorchVision,power mode,temperatures and input
resolution. Measure sustained end-to-end median/p95/p99 latency,FPS,memory,
dropped frames and source timestamp age over a representative full bag.
The local RTX4070Ti diagnostic smoke is only packaging evidence. Exact cases and
environment are recorded in verification/runtime_smoke.json.

Keep current FP32 behavior as the reference. FP16/INT8 or other optimizations
require target-device output/accuracy regression tests for lanes and objects.
No accelerator conversion or quantization has been validated in this package.
Temporal warping/gating is FP32; blindly calling half() is not supported.
Benchmark single-frame and calibrated temporal modes separately. The ROS wrapper
is image-only and reports single-frame; installing the weight does not supply
the calibrated motion needed by the temporal API.
