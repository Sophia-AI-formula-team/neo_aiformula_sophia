"""Inference-only cached affine coefficients, after loading the exact weights."""
from torch import nn
from torchvision.ops.misc import FrozenBatchNorm2d


class _CachedAffine(nn.Module):
    def __init__(self, norm):
        super().__init__()
        scale = norm.weight * (norm.running_var + norm.eps).rsqrt()
        offset = norm.bias - norm.running_mean * scale
        self.register_buffer("scale", scale.reshape(1, -1, 1, 1))
        self.register_buffer("offset", offset.reshape(1, -1, 1, 1))

    def forward(self, x):
        # Retain torchvision's multiply followed by add; no convolution folding.
        return x * self.scale + self.offset


def cache_encoder_norms(module):
    for name, child in module.named_children():
        if type(child) is FrozenBatchNorm2d:
            setattr(module, name, _CachedAffine(child))
        else:
            cache_encoder_norms(child)
