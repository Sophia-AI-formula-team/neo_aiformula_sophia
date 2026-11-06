"""One MobileNet encoder, lane decoder and a replacement four-class FCOS head."""
from collections import OrderedDict

import torch
from torch import nn
from torch.nn import functional as F
from torchvision.models import mobilenet_v3_large
from torchvision.models.detection import FCOS
from torchvision.models.detection.anchor_utils import AnchorGenerator
from torchvision.models.detection.fcos import FCOSHead
from torchvision.ops import FeaturePyramidNetwork
from torchvision.ops.feature_pyramid_network import LastLevelMaxPool
from torchvision.ops.misc import FrozenBatchNorm2d

LABELS = (1, 3, 10, 91)
FEATURE_LAYERS = (1, 3, 6, 12, 16)
ARCHITECTURE = "a3_shared_fcos_v22"


def lane_conv(in_channels, out_channels):
    return nn.Sequential(nn.Conv2d(in_channels, out_channels, 3, padding=1, bias=False),
                         nn.GroupNorm(8, out_channels), nn.ReLU(inplace=True))


class LaneDecoder(nn.Module):
    def __init__(self):
        super().__init__()
        channels = (16, 24, 40, 112, 960)
        widths = (16, 32, 48, 64, 96)
        self.lateral = nn.ModuleList(nn.Conv2d(c, w, 1) for c, w in zip(channels, widths))
        self.fuse = nn.ModuleList(nn.Sequential(lane_conv(widths[i] + widths[i + 1], widths[i]),
                                                lane_conv(widths[i], widths[i])) for i in range(4))
        self.output = nn.Conv2d(widths[0], 2, 1)

    def forward(self, features):
        skips = [layer(value) for layer, value in zip(self.lateral, features)]
        result = skips[-1]
        for index in range(3, -1, -1):
            result = F.interpolate(result, size=skips[index].shape[-2:], mode="bilinear", align_corners=False)
            result = self.fuse[index](torch.cat((result, skips[index]), 1))
        return self.output(result)


class ObjectPyramid(nn.Module):
    def __init__(self, width):
        super().__init__()
        self.out_channels = width
        self.fpn = FeaturePyramidNetwork([24, 40, 112, 960], width, extra_blocks=LastLevelMaxPool())

    def forward(self, features):
        return self.fpn(OrderedDict((str(i), x) for i, x in enumerate(features[1:])))


class SharedPerception(nn.Module):
    def __init__(self, min_size=640, max_size=1152, width=32, depth=2):
        super().__init__()
        if min_size < 32 or max_size < min_size or width % 32 or depth < 1:
            raise ValueError("Invalid image size or FCOS width/depth")
        self.config = dict(min_size=min_size, max_size=max_size, width=width, depth=depth)
        self.encoder = mobilenet_v3_large(weights=None, norm_layer=FrozenBatchNorm2d).features
        self.lane_head = LaneDecoder()
        anchors = AnchorGenerator(((4,), (8,), (16,), (32,), (64,)), ((1.,),) * 5)
        self.detector = FCOS(ObjectPyramid(width), num_classes=4, anchor_generator=anchors,
                             head=FCOSHead(width, 1, 4, num_convs=depth),
                             min_size=min_size, max_size=max_size, score_thresh=.05,
                             nms_thresh=.5, detections_per_img=100, topk_candidates=200)
        self.register_buffer("label_map", torch.tensor(LABELS, dtype=torch.long))

    def extract(self, tensor):
        features = []
        for index, layer in enumerate(self.encoder):
            tensor = layer(tensor)
            if index in FEATURE_LAYERS:
                features.append(tensor)
        return features

    def raw_heads(self, normalized_tensor):
        features = self.extract(normalized_tensor)
        lane = self.lane_head(features)
        maps = list(self.detector.backbone(features).values())
        return lane, maps, self.detector.head(maps)

    def restore_lanes(self, logits, batch, native):
        logits = F.interpolate(logits, size=batch.tensors.shape[-2:], mode="bilinear", align_corners=False)
        return [F.interpolate(value[None, :, :size[0], :size[1]], size=original,
                               mode="bilinear", align_corners=False)[0]
                for value, size, original in zip(logits, batch.image_sizes, native)]

    def prepare(self, images):
        native = [image.shape[-2:] for image in images]
        batch, _ = self.detector.transform(images, None)
        return batch, native

    def forward(self, images, *, detect=True):
        batch, native = self.prepare(images)
        features = self.extract(batch.tensors)
        lanes = self.restore_lanes(self.lane_head(features), batch, native)
        if not detect:
            return lanes, []
        maps = list(self.detector.backbone(features).values())
        outputs = self.detector.head(maps)
        anchors = self.detector.anchor_generator(batch, maps)
        sizes = [value.shape[-2] * value.shape[-1] for value in maps]
        split = {key: list(value.split(sizes, dim=1)) for key, value in outputs.items()}
        predictions = self.detector.postprocess_detections(
            split, [list(a.split(sizes)) for a in anchors], batch.image_sizes)
        for row in predictions:
            row["labels"] = self.label_map[row["labels"]]
        predictions = self.detector.transform.postprocess(predictions, batch.image_sizes, native)
        return lanes, predictions


class DenseExport(nn.Module):
    """Static normalized NCHW input; native-coordinate restoration stays in adapter."""
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, tensor):
        lane, _, outputs = self.model.raw_heads(tensor)
        return lane, outputs["cls_logits"], outputs["bbox_regression"], outputs["bbox_ctrness"]
