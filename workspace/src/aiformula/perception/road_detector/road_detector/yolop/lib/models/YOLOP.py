from collections import OrderedDict

import torch
import torch.nn as nn
from torch.nn import Upsample

from lib.models.common import BottleneckCSP, Concat, Conv, Focus, SPP
from lib.utils.utils import initialize_weights


# Encoder + lane head only. The lane head layout matches the original YOLOP
# lane branch so legacy multitask checkpoints can still seed this model.
LANE_ONLY = [
    [25],  # lane output index
    [-1, Focus, [3, 32, 3]],  # 0
    [-1, Conv, [32, 64, 3, 2]],  # 1
    [-1, BottleneckCSP, [64, 64, 1]],  # 2
    [-1, Conv, [64, 128, 3, 2]],  # 3
    [-1, BottleneckCSP, [128, 128, 3]],  # 4
    [-1, Conv, [128, 256, 3, 2]],  # 5
    [-1, BottleneckCSP, [256, 256, 3]],  # 6
    [-1, Conv, [256, 512, 3, 2]],  # 7
    [-1, SPP, [512, 512, [5, 9, 13]]],  # 8
    [-1, BottleneckCSP, [512, 512, 1, False]],  # 9
    [-1, Conv, [512, 256, 1, 1]],  # 10
    [-1, Upsample, [None, 2, "nearest"]],  # 11
    [[-1, 6], Concat, [1]],  # 12
    [-1, BottleneckCSP, [512, 256, 1, False]],  # 13
    [-1, Conv, [256, 128, 1, 1]],  # 14
    [-1, Upsample, [None, 2, "nearest"]],  # 15
    [[-1, 4], Concat, [1]],  # 16
    [16, Conv, [256, 128, 3, 1]],  # 17
    [-1, Upsample, [None, 2, "nearest"]],  # 18
    [-1, BottleneckCSP, [128, 64, 1, False]],  # 19
    [-1, Conv, [64, 32, 3, 1]],  # 20
    [-1, Upsample, [None, 2, "nearest"]],  # 21
    [-1, Conv, [32, 16, 3, 1]],  # 22
    [-1, BottleneckCSP, [16, 8, 1, False]],  # 23
    [-1, Upsample, [None, 2, "nearest"]],  # 24
    [-1, nn.Conv2d, [8, 2, 3, 1, 1]],  # 25 raw logits
]

LEGACY_LAYER_MAP = {
    **{idx: idx for idx in range(17)},
    **{src: dst for src, dst in zip(range(34, 43), range(17, 26))},
}


class LaneOnlyModel(nn.Module):
    def __init__(self, block_cfg):
        super().__init__()
        layers, save = [], []
        self.seg_out_idx = block_cfg[0][0]

        for i, (from_, block, args) in enumerate(block_cfg[1:]):
            block = eval(block) if isinstance(block, str) else block
            module = block(*args)
            module.index, module.from_ = i, from_
            layers.append(module)
            save.extend(
                x % i for x in ([from_] if isinstance(from_, int) else from_) if x != -1
            )

        self.model = nn.Sequential(*layers)
        self.save = sorted(save)
        self.names = ["background", "lane"]
        initialize_weights(self)

    def forward(self, x):
        cache = []
        lane_logits = None

        for i, block in enumerate(self.model):
            if block.from_ != -1:
                x = (
                    cache[block.from_]
                    if isinstance(block.from_, int)
                    else [x if j == -1 else cache[j] for j in block.from_]
                )
            x = block(x)
            if i == self.seg_out_idx:
                lane_logits = x
            cache.append(x if block.index in self.save else None)

        return lane_logits


def _strip_module_prefix(state_dict):
    clean_state = OrderedDict()
    for key, value in state_dict.items():
        clean_key = key[7:] if key.startswith("module.") else key
        clean_state[clean_key] = value
    return clean_state


def _remap_legacy_key(key):
    if not key.startswith("model."):
        return None

    key_parts = key.split(".")
    if len(key_parts) < 3:
        return None

    try:
        layer_idx = int(key_parts[1])
    except ValueError:
        return None

    if layer_idx not in LEGACY_LAYER_MAP:
        return None

    key_parts[1] = str(LEGACY_LAYER_MAP[layer_idx])
    return ".".join(key_parts)


def get_lane_state_dict(model, checkpoint_or_state_dict):
    state_dict = checkpoint_or_state_dict.get("state_dict", checkpoint_or_state_dict)
    state_dict = _strip_module_prefix(state_dict)
    model_state = model.state_dict()
    loadable_state = OrderedDict()

    for key, value in state_dict.items():
        if key in model_state and model_state[key].shape == value.shape:
            loadable_state[key] = value
            continue

        remapped_key = _remap_legacy_key(key)
        if remapped_key and remapped_key in model_state and model_state[remapped_key].shape == value.shape:
            loadable_state[remapped_key] = value

    return loadable_state


def load_lane_only_weights(model, checkpoint_or_state_dict, strict=False):
    loadable_state = get_lane_state_dict(model, checkpoint_or_state_dict)
    incompatible = model.load_state_dict(loadable_state, strict=strict)
    return {
        "loaded_keys": sorted(loadable_state.keys()),
        "missing_keys": list(incompatible.missing_keys),
        "unexpected_keys": list(incompatible.unexpected_keys),
    }


def get_net(cfg, **kwargs):
    return LaneOnlyModel(LANE_ONLY)


if __name__ == "__main__":
    model = get_net(None)
    dummy_input = torch.randn(1, 3, 256, 256)
    lane_logits = model(dummy_input)
    print(lane_logits.shape)
