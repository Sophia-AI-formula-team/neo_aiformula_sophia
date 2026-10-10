"""Inference-only adapter for the pretrained shared MobileNet architecture."""
import time

import cv2
import numpy as np
import torch
from torchvision.models.detection import FasterRCNN_MobileNet_V3_Large_FPN_Weights

from .color_semantics import classify_traffic_cone, classify_traffic_light
from .lane_processing import LaneRuntimeConfig, overlay_lane_mask, postprocess_lane_probability
from .outputs import Detection, PerceptionOutput, render
from .shared_fpn import load_shared_fpn

CATEGORIES = tuple(FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1.meta["categories"]) + ("traffic cone",)


class SharedFPNPerception:
    def __init__(self, weights, *, device="auto", score_threshold=.5, lane_config=None):
        if not 0 <= score_threshold <= 1:
            raise ValueError("score_threshold must be between 0 and 1")
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.model = load_shared_fpn(weights, self.device)
        self.score_threshold = score_threshold
        self.lane_config = lane_config or LaneRuntimeConfig()

    @torch.inference_mode()
    def predict(self, image):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
            raise ValueError("image must be nonempty uint8 H x W x 3 BGR")
        began = time.perf_counter()
        inputs = torch.from_numpy(cv2.cvtColor(image, cv2.COLOR_BGR2RGB).transpose(2, 0, 1).copy()).to(self.device).float() / 255
        lanes, results = self.model([inputs])
        probability = lanes[0].softmax(0)[1].cpu().numpy()
        lane = postprocess_lane_probability(probability, self.lane_config)
        detections, red_pixels = [], 0
        result = results[0]
        for box, label, score in zip(result["boxes"].cpu().tolist(), result["labels"].cpu().tolist(), result["scores"].cpu().tolist()):
            if score < self.score_threshold or not 0 < label < len(CATEGORIES):
                continue
            color, state = None, None
            name = CATEGORIES[label]
            semantic = name.replace(" ", "_")
            if label == 91:
                color = classify_traffic_cone(image, tuple(box)).label
                semantic = f"{color}_traffic_cone" if color != "unknown" else "traffic_cone"
            elif label == 10:
                evidence = classify_traffic_light(image, tuple(box))
                state = evidence.label
                red_pixels += evidence.pixel_count
                semantic = f"{state}_traffic_light"
            detections.append(Detection(tuple(box), label, name, score, semantic, color, state))
        detections = tuple(detections)
        states = {item.traffic_light_state for item in detections}
        state = "red" if "red" in states else "green" if "green" in states else "unknown"
        lane_overlay = overlay_lane_mask(image, lane)
        object_overlay, obstacle = render(image, detections)
        combined, _ = render(lane_overlay, detections)
        return PerceptionOutput(lane * 255, lane_overlay, obstacle, object_overlay, combined,
                                detections, state, red_pixels, {"total": (time.perf_counter()-began)*1000})
