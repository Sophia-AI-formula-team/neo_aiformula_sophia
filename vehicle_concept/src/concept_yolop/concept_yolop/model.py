"""Inference equations of the frozen MobileNet B1.4 checkpoint. See SOURCES.md."""
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


class NativeCudaConv2d(nn.Conv2d):
    def forward(self, image):
        # cuDNN 8.6 on Orin is slow for the 112-to-48 decoder fusion.
        # PyTorch's CUDA convolution keeps this operation in FP32.
        with torch.backends.cudnn.flags(enabled=False, deterministic=True, allow_tf32=False):
            return super().forward(image)


def lane_conv(in_channels, out_channels, convolution=nn.Conv2d):
    return nn.Sequential(convolution(in_channels, out_channels, 3, padding=1, bias=False),
                         nn.GroupNorm(8, out_channels), nn.ReLU(inplace=True))


class LaneDecoder(nn.Module):
    def __init__(self, native_fusion=False):
        super().__init__()
        channels = (16, 24, 40, 112, 960)
        widths = (16, 32, 48, 64, 96)
        self.lateral = nn.ModuleList(nn.Conv2d(c, w, 1) for c, w in zip(channels, widths))
        self.fuse = nn.ModuleList([
            nn.Sequential(
                lane_conv(widths[i] + widths[i + 1], widths[i],
                          NativeCudaConv2d if native_fusion and i == 2 else nn.Conv2d),
                lane_conv(widths[i], widths[i]))
            for i in range(4)
        ])
        self.output = nn.Conv2d(widths[0], 2, 1)

    def forward(self, features):
        skips = [layer(value) for layer, value in zip(self.lateral, features)]
        result = skips[-1]
        for index in range(3, -1, -1):
            result = F.interpolate(result, size=skips[index].shape[-2:], mode='bilinear', align_corners=False)
            result = self.fuse[index](torch.cat((result, skips[index]), 1))
        return self.output(result)


class ObjectPyramid(nn.Module):
    def __init__(self):
        super().__init__()
        self.out_channels = 32
        self.fpn = FeaturePyramidNetwork([24, 40, 112, 960], 32, extra_blocks=LastLevelMaxPool())

    def forward(self, features):
        return self.fpn(OrderedDict((str(i), x) for i, x in enumerate(features[1:])))


class B14(nn.Module):
    def __init__(self, config, native_fusion=False):
        super().__init__()
        if (config['backend'], config['road_mode'], config['width'], config['depth']) != ('mobile', 'separate', 32, 2):
            raise ValueError('Unexpected B1.4 network configuration')
        self.encoder = mobilenet_v3_large(weights=None, norm_layer=FrozenBatchNorm2d).features
        self.lane_head = LaneDecoder(native_fusion)
        self.road_head = LaneDecoder(native_fusion)
        anchors = AnchorGenerator(((4,), (8,), (16,), (32,), (64,)), ((1.,),) * 5)
        self.detector = FCOS(ObjectPyramid(), num_classes=4, anchor_generator=anchors,
                             head=FCOSHead(32, 1, 4, num_convs=2), min_size=config['min_size'],
                             max_size=config['max_size'], score_thresh=.05, nms_thresh=.5,
                             detections_per_img=100, topk_candidates=200)
        self.register_buffer('label_map', torch.tensor((1, 3, 10, 91), dtype=torch.long))

    @staticmethod
    def restore(logits, batch, native):
        logits = F.interpolate(logits, size=batch.tensors.shape[-2:], mode='bilinear', align_corners=False)
        size = batch.image_sizes[0]
        return F.interpolate(logits[:, :, :size[0], :size[1]], size=native, mode='bilinear',
                             align_corners=False)[0]

    def forward(self, image):
        native = image.shape[-2:]
        batch, _ = self.detector.transform([image])
        x = batch.tensors
        features = []
        for i, layer in enumerate(self.encoder):
            x = layer(x)
            if i in (1, 3, 6, 12, 16):
                features.append(x)
        lane = self.restore(self.lane_head(features), batch, native).argmax(0)
        road = self.restore(self.road_head(features), batch, native).argmax(0)
        maps = list(self.detector.backbone(features).values())
        outputs = self.detector.head(maps)
        anchors = self.detector.anchor_generator(batch, maps)
        sizes = [value.shape[-2] * value.shape[-1] for value in maps]
        split = {key: list(value.split(sizes, dim=1)) for key, value in outputs.items()}
        detections = self.detector.postprocess_detections(
            split, [list(a.split(sizes)) for a in anchors], batch.image_sizes)
        detections[0]['labels'] = self.label_map[detections[0]['labels']]
        return road, lane, self.detector.transform.postprocess(detections, batch.image_sizes, [native])[0]
