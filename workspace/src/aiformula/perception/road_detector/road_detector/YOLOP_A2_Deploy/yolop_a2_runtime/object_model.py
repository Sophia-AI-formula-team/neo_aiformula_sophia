from __future__ import annotations

from pathlib import Path
from typing import Any

import torch
from torch import nn
from torchvision.models.detection import (
    FasterRCNN_MobileNet_V3_Large_FPN_Weights,
    fasterrcnn_mobilenet_v3_large_fpn,
)
from torchvision.ops.misc import FrozenBatchNorm2d


COCO_CATEGORIES = tuple(
    FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1.meta["categories"]
)
TRAFFIC_CONE_ID = len(COCO_CATEGORIES)
OBJECT_CATEGORIES = COCO_CATEGORIES + ("traffic cone",)
OBJECT_CHECKPOINT_FORMAT = 1


class ExtendedFastRCNNPredictor(nn.Module):
    def __init__(self, base_predictor: nn.Module) -> None:
        super().__init__()
        self.base_predictor = base_predictor
        for parameter in self.base_predictor.parameters():
            parameter.requires_grad = False

        input_features = self.base_predictor.cls_score.in_features
        self.cone_score = nn.Linear(input_features, 1)
        self.cone_box = nn.Linear(input_features, 4)

    def forward(self, features: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        base_scores, base_boxes = self.base_predictor(features)
        flat_features = features.flatten(start_dim=1)
        return (
            torch.cat((base_scores, self.cone_score(flat_features)), dim=1),
            torch.cat((base_boxes, self.cone_box(flat_features)), dim=1),
        )


def load_object_checkpoint(
    path: str | Path,
    *,
    device: torch.device,
) -> tuple[nn.Module, dict[str, Any]]:
    checkpoint_path = Path(path)
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"object checkpoint not found: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    if checkpoint.get("format_version") != OBJECT_CHECKPOINT_FORMAT:
        raise ValueError(
            f"unsupported object checkpoint format in {checkpoint_path}: "
            f"{checkpoint.get('format_version')!r}"
        )
    if tuple(checkpoint.get("categories", ())) != OBJECT_CATEGORIES:
        raise ValueError(f"object category contract mismatch in {checkpoint_path}")

    model = _build_object_detector()
    model.load_state_dict(checkpoint["model_state"], strict=True)
    model.to(device)
    model.eval()
    return model, checkpoint


def _build_object_detector() -> nn.Module:
    model = fasterrcnn_mobilenet_v3_large_fpn(
        weights=None,
        weights_backbone=None,
        min_size=640,
        max_size=1280,
        box_score_thresh=0.05,
    )
    _replace_batch_norm_with_frozen(model)
    for parameter in model.parameters():
        parameter.requires_grad = False
    model.roi_heads.box_predictor = ExtendedFastRCNNPredictor(
        model.roi_heads.box_predictor
    )
    return model


def _replace_batch_norm_with_frozen(module: nn.Module) -> None:
    for child_name, child in tuple(module.named_children()):
        if isinstance(child, nn.BatchNorm2d):
            setattr(
                module,
                child_name,
                FrozenBatchNorm2d(child.num_features, eps=child.eps),
            )
            continue
        _replace_batch_norm_with_frozen(child)
