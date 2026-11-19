#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/foxy/setup.bash
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
for profile in A4 A4_lite; do
    PYTHONPATH="$PWD/bundles/$profile:$PWD/bundles/$profile/ros2/yolop_a3_perception:$PYTHONPATH" foxy-env/bin/python3 - "$profile" <<'PY'
import hashlib,sys
import cv2,numpy as np,rclpy,cv_bridge,torch,torchvision
from yolop_a3_runtime import PerceptionModel
from yolop_a3_perception.node import main
profile=sys.argv[1]
torch.set_num_threads(2)
model=PerceptionModel(f'bundles/{profile}/weights/shared_best.pth',device='cuda',lean_runtime=profile=='A4_lite')
result=model.predict(np.zeros((360,640,3),dtype=np.uint8),combined_overlay=profile=='A4')
for mask in (result.lane_mask,result.obstacle_mask):
    assert mask.shape == (360,640) and mask.dtype == np.uint8
print(profile,torch.__version__,torchvision.__version__,torch.cuda.get_device_name(),
      *(hashlib.sha256(x.tobytes()).hexdigest() for x in (result.lane_mask,result.obstacle_mask)))
PY
done
