from pathlib import Path
import sys

import numpy as np
import pytest
import torch
from torchvision.ops.misc import FrozenBatchNorm2d

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionModel
from yolop_a3_runtime.lane_processing import LaneRuntimeConfig


@pytest.mark.parametrize("config", [LaneRuntimeConfig(), LaneRuntimeConfig(prediction_threshold=.63),
                                     LaneRuntimeConfig(postprocess_enabled=True)])
def test_optional_lean_preserves_cpu_outputs(config):
    torch.set_num_threads(2)
    nets = [PerceptionModel(ROOT/"weights/shared_best.pth", device="cpu", lane_config=config, lean_runtime=flag)
            for flag in (False, True)]
    assert any(isinstance(m,FrozenBatchNorm2d) for m in nets[0].model.base.encoder.modules())
    assert not any(isinstance(m,FrozenBatchNorm2d) for m in nets[1].model.base.encoder.modules())
    img = np.random.default_rng(171).integers(0,256,(79,143,3),dtype=np.uint8)
    for net in nets:
        net.model.base.detector.transform.min_size = (128,)
        net.model.base.detector.transform.max_size = 256
    ref, lean = [net.predict(img) for net in nets]
    for key in ("lane_mask","obstacle_mask","lane_overlay","object_overlay","combined_overlay"):
        np.testing.assert_array_equal(getattr(ref,key),getattr(lean,key))
    assert ref.detections == lean.detections
    assert nets[1].predict(img,combined_overlay=False).combined_overlay is None
