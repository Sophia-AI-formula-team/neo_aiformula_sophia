"""One MobileNet feature traversal feeding detection and amodal lane heads.

The detector layout preserves A2's TorchVision parameter names for exact transfer.
No teacher or training dependency is loaded by this module.
"""
from collections import OrderedDict

import torch
from torch import nn
from torch.nn import functional as F
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.ops.misc import FrozenBatchNorm2d
from torchvision.models.detection.roi_heads import RoIHeads
from torchvision.ops import boxes as box_ops

ARCHITECTURE = "a3_mobilenet_shared_fpn_v2"
REFINED_ARCHITECTURE = "a3_mobilenet_shared_fpn_refined_v3"
CALIBRATED_ARCHITECTURE = "a3_mobilenet_shared_fpn_independent_v4"
CONTRACT_ARCHITECTURE = "a3_mobilenet_shared_fpn_contract_v14"
INPUT_CONTRACT = dict(version=1, name="a3_original_native", min_size=640,
                      max_size=1280, size_divisible=32, resize="torch_bilinear",
                      padding="bottom_right_zero_normalized", normalize="imagenet",
                      restore="torch_logits_crop_resize_align_corners_false", threshold=.5)
LANE_LAYERS = ("1", "3", "6", "12", "16")


class ExtendedPredictor(nn.Module):
    def __init__(self, base):
        super().__init__()
        self.base_predictor = base
        self.cone_score = nn.Linear(base.cls_score.in_features, 1)
        self.cone_box = nn.Linear(base.cls_score.in_features, 4)

    def forward(self, features):
        scores, boxes = self.base_predictor(features)
        flat = features.flatten(1)
        return torch.cat((scores, self.cone_score(flat)), 1), torch.cat((boxes, self.cone_box(flat)), 1)


class RefinedPredictor(ExtendedPredictor):
    def __init__(self, base):
        super().__init__(base)
        self.cone_delta=nn.Sequential(nn.Linear(base.cls_score.in_features,128),nn.ReLU(),nn.Linear(128,5))
        nn.init.zeros_(self.cone_delta[-1].weight)
        nn.init.zeros_(self.cone_delta[-1].bias)

    def forward(self, features):
        scores,boxes=super().forward(features)
        delta=self.cone_delta(features.flatten(1))
        return torch.cat((scores[:,:-1],scores[:,-1:]+delta[:,:1]),1),torch.cat((boxes[:,:-4],boxes[:,-4:]+delta[:,1:]),1)


class IndependentPredictor(RefinedPredictor):
    def forward(self, features):
        baseline,boxes=ExtendedPredictor.forward(self,features)
        delta=self.cone_delta(features.flatten(1))
        # Keep the original class posterior available; cone adaptation must not
        # change a pedestrian/light confidence via a shared softmax denominator.
        scores=torch.cat((baseline,baseline[:,-1:]+delta[:,:1]),1)
        return scores,torch.cat((boxes[:,:-4],boxes[:,-4:]+delta[:,1:]),1)


class IndependentConeRoIHeads(RoIHeads):
    @classmethod
    def from_heads(cls,old):
        return cls(old.box_roi_pool,old.box_head,old.box_predictor,old.proposal_matcher.high_threshold,
                   old.proposal_matcher.low_threshold,old.fg_bg_sampler.batch_size_per_image,
                   old.fg_bg_sampler.positive_fraction,old.box_coder.weights,old.score_thresh,
                   old.nms_thresh,old.detections_per_img)

    def postprocess_detections(self,class_logits,box_regression,proposals,image_shapes):
        baseline=class_logits[:,:92].softmax(-1)
        cone=torch.sigmoid(class_logits[:,92:]-torch.logsumexp(class_logits[:,:91],1,keepdim=True))
        scores=torch.cat((baseline[:,:91],cone),1)
        sizes=[p.shape[0] for p in proposals]
        boxes=self.box_coder.decode(box_regression,proposals)
        all_boxes,all_scores,all_labels=[],[],[]
        for box,score,shape in zip(boxes.split(sizes),scores.split(sizes),image_shapes):
            box=box_ops.clip_boxes_to_image(box,shape)[:,1:].reshape(-1,4)
            label=torch.arange(1,92,device=score.device).expand(score.shape[0],91).reshape(-1)
            score=score[:,1:].reshape(-1)
            keep=torch.where(score>self.score_thresh)[0]
            box,score,label=box[keep],score[keep],label[keep]
            keep=box_ops.remove_small_boxes(box,1e-2)
            box,score,label=box[keep],score[keep],label[keep]
            keep=box_ops.batched_nms(box,score,label,self.nms_thresh)[:self.detections_per_img]
            all_boxes.append(box[keep]);all_scores.append(score[keep]);all_labels.append(label[keep])
        return all_boxes,all_scores,all_labels


def freeze_norm(module):
    for name, child in tuple(module.named_children()):
        if isinstance(child, nn.BatchNorm2d):
            setattr(module, name, FrozenBatchNorm2d(child.num_features, eps=child.eps))
        else:
            freeze_norm(child)


def conv(input_channels, output_channels):
    return nn.Sequential(nn.Conv2d(input_channels, output_channels, 3, padding=1, bias=False),
                         nn.GroupNorm(8, output_channels), nn.ReLU(inplace=True))


class LaneDecoder(nn.Module):
    def __init__(self):
        super().__init__()
        channels = (16, 24, 40, 112, 960)
        widths = (16, 32, 48, 64, 96)
        self.lateral = nn.ModuleList(nn.Conv2d(c, w, 1) for c, w in zip(channels, widths))
        self.fuse = nn.ModuleList(nn.Sequential(conv(widths[i] + widths[i + 1], widths[i]),
                                                conv(widths[i], widths[i])) for i in range(4))
        self.output = nn.Conv2d(widths[0], 2, 1)

    def forward(self, features):
        skips = [layer(value) for layer, value in zip(self.lateral, features)]
        result = skips[-1]
        for index in range(3, -1, -1):
            result = F.interpolate(result, size=skips[index].shape[-2:], mode="bilinear", align_corners=False)
            result = self.fuse[index](torch.cat((result, skips[index]), 1))
        return self.output(result)


class SharedFPN(nn.Module):
    def __init__(self):
        super().__init__()
        self.detector = fasterrcnn_mobilenet_v3_large_fpn(
            weights=None, weights_backbone=None, min_size=640, max_size=1280, box_score_thresh=.05)
        freeze_norm(self.detector)
        self.detector.roi_heads.box_predictor = ExtendedPredictor(self.detector.roi_heads.box_predictor)
        self.lane_head = LaneDecoder()

    def extract(self, tensor):
        detector_features, lane_features = OrderedDict(), []
        # Execute every encoder block exactly once; no forward-hook cache/state.
        for name, layer in self.detector.backbone.body.items():
            tensor = layer(tensor)
            if name in LANE_LAYERS:
                lane_features.append(tensor)
            if name in self.detector.backbone.body.return_layers:
                detector_features[self.detector.backbone.body.return_layers[name]] = tensor
        return lane_features, detector_features

    def forward(self, images, *, detect=True):
        original_sizes = [image.shape[-2:] for image in images]
        images, _ = self.detector.transform(images, None)
        lane_features, features = self.extract(images.tensors)
        logits = self.lane_head(lane_features)
        logits = F.interpolate(logits, size=images.tensors.shape[-2:], mode="bilinear", align_corners=False)
        lanes = [F.interpolate(logit[None, :, :size[0], :size[1]], size=original,
                               mode="bilinear", align_corners=False)[0]
                 for logit, size, original in zip(logits, images.image_sizes, original_sizes)]
        detections = []
        if detect:
            features = self.detector.backbone.fpn(features)
            proposals, _ = self.detector.rpn(images, features, None)
            detections, _ = self.detector.roi_heads(features, proposals, images.image_sizes, None)
            detections = self.detector.transform.postprocess(detections, images.image_sizes, original_sizes)
        return lanes, detections


def load_shared_fpn(path, device="cpu"):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    if payload.get("architecture") not in {ARCHITECTURE,REFINED_ARCHITECTURE,CALIBRATED_ARCHITECTURE,CONTRACT_ARCHITECTURE}:
        raise ValueError("Expected A3 shared MobileNet FPN checkpoint")
    if payload["architecture"] == CONTRACT_ARCHITECTURE and payload.get("input_contract") != INPUT_CONTRACT:
        raise ValueError("Missing or changed V14 input contract")
    model = SharedFPN()
    if payload["architecture"]==REFINED_ARCHITECTURE:
        model.detector.roi_heads.box_predictor=RefinedPredictor(model.detector.roi_heads.box_predictor.base_predictor)
    elif payload["architecture"] in {CALIBRATED_ARCHITECTURE, CONTRACT_ARCHITECTURE}:
        model.detector.roi_heads.box_predictor=IndependentPredictor(model.detector.roi_heads.box_predictor.base_predictor)
        model.detector.roi_heads=IndependentConeRoIHeads.from_heads(model.detector.roi_heads)
    model.load_state_dict(payload["model"], strict=True)
    return model.to(device).eval()
