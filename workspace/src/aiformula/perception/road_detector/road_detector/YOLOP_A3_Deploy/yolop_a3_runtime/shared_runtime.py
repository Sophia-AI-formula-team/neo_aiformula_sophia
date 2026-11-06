"""Inference-only adapter for the pretrained shared MobileNet architecture."""
import time

import cv2
import numpy as np
import torch

from .color_semantics import classify_traffic_cone, classify_traffic_light
from .lane_processing import LaneRuntimeConfig, overlay_lane_mask, postprocess_lane_probability
from .outputs import Detection, PerceptionOutput, render
from .temporal import load_model

CATEGORIES = {1: "person", 3: "car", 10: "traffic light", 91: "traffic cone"}


class TemporalPerceptionModel:
    def __init__(self, weights, *, device="auto", score_threshold=.5, lane_config=None):
        if not 0 <= score_threshold <= 1:
            raise ValueError("score_threshold must be between 0 and 1")
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.model = load_model(weights, self.device)
        self.score_threshold = score_threshold
        self.lane_config = lane_config or LaneRuntimeConfig()

    @torch.inference_mode()
    def predict(self, image):
        return self._predict(image)

    @torch.inference_mode()
    def predict_frame(self, image, *, sequence, timestamp, plane_to_image):
        """Use metric ground-plane projection at this image's capture time."""
        return self._predict(image, dict(sequence=sequence, timestamp=timestamp,
                                         plane_to_image=plane_to_image))

    def reset(self):
        self.model.reset()

    def _predict(self, image, geometry=None):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
            raise ValueError("image must be nonempty uint8 H x W x 3 BGR")
        began = time.perf_counter()
        inputs = torch.from_numpy(cv2.cvtColor(image, cv2.COLOR_BGR2RGB).transpose(2, 0, 1).copy()).to(self.device).float() / 255
        if geometry is None:
            lanes, results = self.model([inputs])
        else:
            lanes, results = self.model.forward_frame(inputs, **geometry)
        probability = lanes[0].softmax(0)[1].cpu().numpy()
        lane = postprocess_lane_probability(probability, self.lane_config)
        detections, red_pixels = [], 0
        result = results[0]
        for box, label, score in zip(result["boxes"].cpu().tolist(), result["labels"].cpu().tolist(), result["scores"].cpu().tolist()):
            if score < self.score_threshold or label not in CATEGORIES:
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
                                detections, state, red_pixels, {"total": (time.perf_counter()-began)*1000},
                                temporal_status=self.model.status)
