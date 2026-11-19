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


def test_grid_cache_is_bounded_and_invalidates_on_shape_change():
    net = PerceptionModel(ROOT/"weights/shared_best.pth", device="cpu", lean_runtime=True)
    base = net.model.base
    with torch.inference_mode():
        maps = [torch.zeros((1, 1, y, x)) for y, x in ((16,24),(8,12),(4,6),(2,3),(1,2))]
        from torchvision.models.detection.image_list import ImageList
        batch = ImageList(torch.zeros((1, 3, 64, 96)), [(61, 94)])
        sizes = [v.shape[-2]*v.shape[-1] for v in maps]
        grid = base.split_anchor_grid(batch, maps, sizes)
        assert base.split_anchor_grid(batch, maps, sizes) is grid
        changed = ImageList(torch.zeros((1, 3, 64, 128)), [(61, 124)])
        new_grid = base.split_anchor_grid(changed, maps, sizes)
        assert new_grid is not grid
        assert base.split_anchor_grid(changed, maps, sizes) is new_grid
        base.cache_anchor_grid = False
        reference = base.split_anchor_grid(changed, maps, sizes)
        assert all(torch.equal(a,b) for a,b in zip(reference[0],new_grid[0]))
