from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Any

import torch
from torch import nn


def _autopad(kernel_size: int, padding: int | None = None) -> int:
    return kernel_size // 2 if padding is None else padding


class HardSwish(nn.Module):
    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return inputs * nn.functional.hardtanh(inputs + 3, 0.0, 6.0) / 6.0


class Conv(nn.Module):
    def __init__(
        self,
        input_channels: int,
        output_channels: int,
        kernel_size: int = 1,
        stride: int = 1,
        padding: int | None = None,
        groups: int = 1,
        activation: bool = True,
    ) -> None:
        super().__init__()
        self.conv = nn.Conv2d(
            input_channels,
            output_channels,
            kernel_size,
            stride,
            _autopad(kernel_size, padding),
            groups=groups,
            bias=False,
        )
        self.bn = nn.BatchNorm2d(output_channels)
        self.act = HardSwish() if activation else nn.Identity()

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.act(self.bn(self.conv(inputs)))


class Bottleneck(nn.Module):
    def __init__(
        self,
        input_channels: int,
        output_channels: int,
        shortcut: bool = True,
        groups: int = 1,
        expansion: float = 0.5,
    ) -> None:
        super().__init__()
        hidden_channels = int(output_channels * expansion)
        self.cv1 = Conv(input_channels, hidden_channels, 1, 1)
        self.cv2 = Conv(hidden_channels, output_channels, 3, 1, groups=groups)
        self.add = shortcut and input_channels == output_channels

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        output = self.cv2(self.cv1(inputs))
        return inputs + output if self.add else output


class BottleneckCSP(nn.Module):
    def __init__(
        self,
        input_channels: int,
        output_channels: int,
        repeats: int = 1,
        shortcut: bool = True,
        groups: int = 1,
        expansion: float = 0.5,
    ) -> None:
        super().__init__()
        hidden_channels = int(output_channels * expansion)
        self.cv1 = Conv(input_channels, hidden_channels, 1, 1)
        self.cv2 = nn.Conv2d(input_channels, hidden_channels, 1, 1, bias=False)
        self.cv3 = nn.Conv2d(hidden_channels, hidden_channels, 1, 1, bias=False)
        self.cv4 = Conv(2 * hidden_channels, output_channels, 1, 1)
        self.bn = nn.BatchNorm2d(2 * hidden_channels)
        self.act = nn.LeakyReLU(0.1, inplace=True)
        self.m = nn.Sequential(
            *[
                Bottleneck(
                    hidden_channels,
                    hidden_channels,
                    shortcut,
                    groups,
                    expansion=1.0,
                )
                for _ in range(repeats)
            ]
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        first = self.cv3(self.m(self.cv1(inputs)))
        second = self.cv2(inputs)
        return self.cv4(self.act(self.bn(torch.cat((first, second), dim=1))))


class SpatialPyramidPooling(nn.Module):
    def __init__(
        self,
        input_channels: int,
        output_channels: int,
        kernel_sizes: tuple[int, ...] | list[int] = (5, 9, 13),
    ) -> None:
        super().__init__()
        hidden_channels = input_channels // 2
        self.cv1 = Conv(input_channels, hidden_channels, 1, 1)
        self.cv2 = Conv(hidden_channels * (len(kernel_sizes) + 1), output_channels, 1, 1)
        self.m = nn.ModuleList(
            nn.MaxPool2d(kernel_size=size, stride=1, padding=size // 2)
            for size in kernel_sizes
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        inputs = self.cv1(inputs)
        return self.cv2(torch.cat([inputs] + [pool(inputs) for pool in self.m], dim=1))


class Focus(nn.Module):
    def __init__(
        self,
        input_channels: int,
        output_channels: int,
        kernel_size: int = 1,
        stride: int = 1,
        padding: int | None = None,
        groups: int = 1,
        activation: bool = True,
    ) -> None:
        super().__init__()
        self.conv = Conv(
            input_channels * 4,
            output_channels,
            kernel_size,
            stride,
            padding,
            groups,
            activation,
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        slices = (
            inputs[..., ::2, ::2],
            inputs[..., 1::2, ::2],
            inputs[..., ::2, 1::2],
            inputs[..., 1::2, 1::2],
        )
        return self.conv(torch.cat(slices, dim=1))


class Concat(nn.Module):
    def __init__(self, dimension: int = 1) -> None:
        super().__init__()
        self.dimension = dimension

    def forward(self, inputs: list[torch.Tensor]) -> torch.Tensor:
        return torch.cat(inputs, dim=self.dimension)


LANE_BLOCKS = (
    (25,),
    (-1, Focus, (3, 32, 3)),
    (-1, Conv, (32, 64, 3, 2)),
    (-1, BottleneckCSP, (64, 64, 1)),
    (-1, Conv, (64, 128, 3, 2)),
    (-1, BottleneckCSP, (128, 128, 3)),
    (-1, Conv, (128, 256, 3, 2)),
    (-1, BottleneckCSP, (256, 256, 3)),
    (-1, Conv, (256, 512, 3, 2)),
    (-1, SpatialPyramidPooling, (512, 512, (5, 9, 13))),
    (-1, BottleneckCSP, (512, 512, 1, False)),
    (-1, Conv, (512, 256, 1, 1)),
    (-1, nn.Upsample, (None, 2, "nearest")),
    ((-1, 6), Concat, (1,)),
    (-1, BottleneckCSP, (512, 256, 1, False)),
    (-1, Conv, (256, 128, 1, 1)),
    (-1, nn.Upsample, (None, 2, "nearest")),
    ((-1, 4), Concat, (1,)),
    (16, Conv, (256, 128, 3, 1)),
    (-1, nn.Upsample, (None, 2, "nearest")),
    (-1, BottleneckCSP, (128, 64, 1, False)),
    (-1, Conv, (64, 32, 3, 1)),
    (-1, nn.Upsample, (None, 2, "nearest")),
    (-1, Conv, (32, 16, 3, 1)),
    (-1, BottleneckCSP, (16, 8, 1, False)),
    (-1, nn.Upsample, (None, 2, "nearest")),
    (-1, nn.Conv2d, (8, 2, 3, 1, 1)),
)

LEGACY_LAYER_MAP = {
    **{index: index for index in range(17)},
    **{source: target for source, target in zip(range(34, 43), range(17, 26))},
}


class LaneOnlyModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.seg_out_idx = LANE_BLOCKS[0][0]
        layers: list[nn.Module] = []
        saved_indices: list[int] = []

        for index, (sources, block_type, arguments) in enumerate(LANE_BLOCKS[1:]):
            block = block_type(*arguments)
            block.index = index
            block.from_ = sources
            layers.append(block)
            source_indices = [sources] if isinstance(sources, int) else sources
            saved_indices.extend(source % index for source in source_indices if source != -1)

        self.model = nn.Sequential(*layers)
        self.save = sorted(saved_indices)
        self.names = ["background", "lane"]
        _initialize_weights(self)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        cache: list[torch.Tensor | None] = []
        lane_logits: torch.Tensor | None = None
        current: torch.Tensor | list[torch.Tensor] = inputs

        for index, block in enumerate(self.model):
            if block.from_ != -1:
                if isinstance(block.from_, int):
                    current = cache[block.from_]
                else:
                    current = [
                        current if source == -1 else cache[source]
                        for source in block.from_
                    ]
            current = block(current)
            if index == self.seg_out_idx:
                lane_logits = current
            cache.append(current if block.index in self.save else None)

        if lane_logits is None:
            raise RuntimeError("lane model did not produce logits")
        return lane_logits


def load_lane_checkpoint(
    path: str | Path,
    *,
    device: torch.device,
) -> LaneOnlyModel:
    checkpoint_path = Path(path)
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"lane checkpoint not found: {checkpoint_path}")

    model = LaneOnlyModel().to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    loadable_state = _get_lane_state_dict(model, checkpoint)
    incompatible = model.load_state_dict(loadable_state, strict=False)
    if incompatible.missing_keys:
        raise ValueError(
            f"lane checkpoint is incomplete: {len(incompatible.missing_keys)} missing keys"
        )
    model.eval()
    return model


def _initialize_weights(model: nn.Module) -> None:
    for module in model.modules():
        if isinstance(module, nn.BatchNorm2d):
            module.eps = 1e-3
            module.momentum = 0.03
        elif isinstance(module, (nn.Hardswish, nn.LeakyReLU, nn.ReLU, nn.ReLU6)):
            module.inplace = True


def _get_lane_state_dict(
    model: nn.Module,
    checkpoint_or_state_dict: dict[str, Any],
) -> OrderedDict[str, torch.Tensor]:
    raw_state = checkpoint_or_state_dict.get("state_dict", checkpoint_or_state_dict)
    clean_state = OrderedDict(
        (
            key[7:] if key.startswith("module.") else key,
            value,
        )
        for key, value in raw_state.items()
    )
    model_state = model.state_dict()
    loadable_state: OrderedDict[str, torch.Tensor] = OrderedDict()

    for key, value in clean_state.items():
        if key in model_state and model_state[key].shape == value.shape:
            loadable_state[key] = value
            continue
        remapped_key = _remap_legacy_key(key)
        if (
            remapped_key is not None
            and remapped_key in model_state
            and model_state[remapped_key].shape == value.shape
        ):
            loadable_state[remapped_key] = value
    return loadable_state


def _remap_legacy_key(key: str) -> str | None:
    if not key.startswith("model."):
        return None
    parts = key.split(".")
    if len(parts) < 3:
        return None
    try:
        layer_index = int(parts[1])
    except ValueError:
        return None
    if layer_index not in LEGACY_LAYER_MAP:
        return None
    parts[1] = str(LEGACY_LAYER_MAP[layer_index])
    return ".".join(parts)
