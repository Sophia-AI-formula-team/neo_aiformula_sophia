"""Image-only supplement for luminous red/green flag panels.

The A3 detector is retained. A supplementary panel needs compact square colour,
an adjacent dark border, contrast against its surrounding background, and two
independent frames at a consistent image location. No world pose, signal truth,
fixed image coordinates, previous colour replay or route data is consumed.
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np
from signal_panel_range import project_panel_geometry


def _model_boxes(detections):
    boxes = []
    for detection in detections or ():
        if isinstance(detection, dict):
            label = detection.get("class_id")
            box = detection.get("box_xyxy", detection.get("bbox", detection.get("box")))
        else:
            label = getattr(detection, "class_id", None)
            box = getattr(detection, "box_xyxy", None)
        if label == 10 and box is not None and len(box) == 4:
            boxes.append([float(value) for value in box])
    return boxes


def panel_candidates(image_bgr, detections=(), calibration=None):
    """Current whole-image candidates; an explicit calibration adds identity proof.

    Runtime callers must pass the current camera, or {} when unavailable.
    None preserves the uncalibrated API for existing offline callers only.
    """
    if not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image_bgr must be uint8 HxWx3")
    height, width = image_bgr.shape[:2]
    if min(height, width) < 16:
        return []
    hue, saturation, value = cv2.split(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV))
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    vivid = (saturation >= 110) & (value >= 65)
    masks = {"red": vivid & ((hue <= 9) | (hue >= 170)),
             "green": vivid & (hue >= 38) & (hue <= 85)}
    model_boxes = _model_boxes(detections)
    candidates = []
    max_side = max(10, int(min(height, width) * 0.2))
    for color, mask in masks.items():
        count, components, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        for index in range(1, count):
            x, y, box_w, box_h, area = (int(v) for v in stats[index])
            if min(box_w, box_h) < 3 or max(box_w, box_h) > max_side or area < 9:
                continue
            aspect, fill = box_w / box_h, area / (box_w * box_h)
            # Each rasterized dimension has up to one pixel of edge uncertainty.
            # Require that its aspect interval overlaps the same geometric gate.
            if (box_w + 1) / (box_h - 1) < 0.6 or (box_w - 1) / (box_h + 1) > 1.65 or fill < 0.65:
                continue
            pad = max(2, round(min(box_w, box_h) * 0.3))
            outer = max(pad + 2, round(max(box_w, box_h) * 0.75))
            # Cropped-off candidates cannot provide four-sided background proof.
            if x < outer or y < outer or x + box_w + outer > width or y + box_h + outer > height:
                continue
            region = np.s_[y-pad:y+box_h+pad, x-pad:x+box_w+pad]
            background = np.s_[y-outer:y+box_h+outer, x-outer:x+box_w+outer]
            ring = np.ones((box_h + 2*outer, box_w + 2*outer), dtype=bool)
            ring[outer-pad:outer+box_h+pad, outer-pad:outer+box_w+pad] = False
            background_luma = float(np.median(gray[background][ring]))
            background_color_fraction = float(np.mean(mask[background][ring]))
            if background_luma < 65 or background_color_fraction > 0.15:
                continue
            # Colour is measured in the component. Border evidence must come
            # from different, darker pixels; a plain coloured square is rejected.
            colored_region = mask[region]
            # A rasterized border can mix with background at any panel size.
            # Use one photometric model: >=18% attenuation from background,
            # with separate color/core contrast and two current dark sides.
            # Both this uint8 sample and the background estimate have a
            # half-level quantization interval. Compare the possible border
            # lower edge with the background upper edge, not rounded centers.
            dark_limit = max(75.0, (background_luma + 0.5) * 0.82) + 0.5
            dark = (value[region] < dark_limit) & ~colored_region
            mixed_border = bool(np.any(dark & (value[region] >= max(75.0, background_luma * 0.72))))
            dark_count = int(np.count_nonzero(dark))
            border_sides = [int(np.count_nonzero(dark[:, :pad+1])),
                            int(np.count_nonzero(dark[:, pad+box_w-1:])),
                            int(np.count_nonzero(dark[:pad+1, :])),
                            int(np.count_nonzero(dark[pad+box_h-1:, :]))]
            if dark_count < max(2, math.ceil(min(box_w, box_h) * 0.4)) or sum(n > 0 for n in border_sides) < 2:
                continue
            component = components[y:y+box_h, x:x+box_w] == index
            # Measure the observed bright core consistently across raster sizes;
            # the 25-level contrast requirement is unchanged.
            component_value = float(np.percentile(value[y:y+box_h, x:x+box_w][component], 75))
            dark_value = float(np.median(value[region][dark]))
            if component_value - dark_value < 25:
                continue
            # Verify the observed object can be the known-height panel before
            # allowing it to acquire a color track. A cone below the calibrated
            # horizon cannot masquerade as that panel, even if its red base is
            # a compact, dark-bordered blob. No fixed pixel ROI is involved.
            if calibration is not None:
                geometry = project_panel_geometry([x, y, x+box_w, y+box_h], calibration, image_bgr.shape)
                if not geometry["valid"]:
                    continue
            center_x, center_y = x + box_w/2, y + box_h/2
            associated = any(left-pad <= center_x <= right+pad and top-pad <= center_y <= bottom+pad
                             for left, top, right, bottom in model_boxes)
            candidates.append(dict(state=color, bbox=[x, y, x+box_w, y+box_h], pixels=area,
                                   fill=round(fill, 4), aspect=round(aspect, 4), dark_pixels=dark_count,
                                   dark_sides=border_sides, background_luma=round(background_luma, 2),
                                   background_color_fraction=round(background_color_fraction, 4),
                                   color_value=round(component_value, 2), dark_value=round(dark_value, 2),
                                   model_associated=associated, mixed_border=mixed_border))
    return sorted(candidates, key=lambda item: (item["model_associated"], item["pixels"] * item["fill"]), reverse=True)


def _prediction_error(previous, current, dt=None):
    ax, ay, ar, ab = previous["bbox"]
    bx, by, br, bb = current["bbox"]
    dx, dy = (ax+ar-bx-br)/2, (ay+ab-by-bb)/2
    velocity = previous.get("image_velocity_px_s")
    if velocity is not None and dt is not None and 0 < dt <= 0.75:
        dx += velocity[0] * dt
        dy += velocity[1] * dt
    return math.hypot(dx, dy)


def _matches(previous, current, dt=None):
    ax, ay, ar, ab = previous["bbox"]
    bx, by, br, bb = current["bbox"]
    scale_a, scale_b = max(ar-ax, ab-ay), max(br-bx, bb-by)
    return min(scale_a, scale_b) / max(scale_a, scale_b) >= 0.5 and _prediction_error(previous, current, dt) <= max(8.0, 1.5*max(scale_a, scale_b))


class SignalPanelRGBTracker:
    def __init__(self):
        self.last_sequence = -1
        self.last_time = None
        self.tracks = []

    def update(self, image_bgr, model_state, detections, sequence, now=None, calibration=None):
        """One observation per real frame; explicit A3 states remain available."""
        if type(sequence) is not int or sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        if sequence <= self.last_sequence:
            return dict(state="unknown", source="none", fresh=False, reason="duplicate_or_out_of_order",
                        panel=None, candidates=[], model_state=model_state)
        arrived = time.monotonic() if now is None else float(now)
        if not math.isfinite(arrived) or self.last_time is not None and arrived < self.last_time:
            raise ValueError("arrival time must be finite and monotonic")
        old_tracks = self.tracks if self.last_time is not None and arrived - self.last_time <= 0.75 else []
        dt = None if self.last_time is None else arrived - self.last_time
        self.last_sequence, self.last_time = sequence, arrived
        candidates = panel_candidates(image_bgr, detections, calibration)
        tracks = []
        for candidate in candidates:
            prior = [item for item in old_tracks if item["state"] == candidate["state"] and _matches(item, candidate, dt)]
            previous = min(prior, key=lambda item: _prediction_error(item, candidate, dt)) if prior else None
            confirmed_frames = min(1000, 1 + (previous["confirmed_frames"] if previous else 0))
            # Velocity only associates two already observed same-color panels.
            # It cannot produce a missing candidate, color, bbox or distance.
            velocity = None
            if previous is not None and dt is not None and dt > 1e-6:
                ax, ay, ar, ab = previous["bbox"]
                bx, by, br, bb = candidate["bbox"]
                velocity = [(bx+br-ax-ar)/(2*dt), (by+bb-ay-ab)/(2*dt)]
            tracks.append(dict(candidate, confirmed_frames=confirmed_frames, image_velocity_px_s=velocity))
        self.tracks = tracks
        confirmed = [item for item in tracks if item["confirmed_frames"] >= 2]
        rgb_red = next((item for item in confirmed if item["state"] == "red"), None)
        rgb_green = next((item for item in confirmed if item["state"] == "green"), None)
        # Any currently evidenced red dominates a concurrent green result.
        if model_state == "red":
            state, source, panel = "red", "a3_model", rgb_red
        elif rgb_red is not None:
            state, source, panel = "red", "rgb_panel", rgb_red
        elif model_state == "green":
            state, source, panel = "green", "a3_model", rgb_green
        elif rgb_green is not None:
            state, source, panel = "green", "rgb_panel", rgb_green
        else:
            state, source, panel = "unknown", "none", None
        return dict(state=state, source=source, fresh=True, reason="current_image_evidence" if state != "unknown" else "insufficient_panel_evidence",
                    panel=panel, candidates=tracks, model_state=model_state,
                    definition="A3 detection or current RGB square+dark-border+background proof in two frames")


def draw_panel_overlay(overlay_bgr, evidence):
    """Add one independently labelled box; retain the original model overlay."""
    result = overlay_bgr.copy()
    panel = evidence.get("panel")
    if evidence.get("source") != "rgb_panel" or panel is None:
        return result
    left, top, right, bottom = panel["bbox"]
    color = (80, 240, 80) if evidence["state"] == "green" else (80, 80, 255)
    cv2.rectangle(result, (left-2, top-2), (right+2, bottom+2), color, 1)
    label = "RGB panel: " + evidence["state"]
    cv2.putText(result, label, (max(0, min(left-3, result.shape[1]-155)), min(result.shape[0]-5, bottom+16)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(result, label, (max(0, min(left-3, result.shape[1]-155)), min(result.shape[0]-5, bottom+16)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    return result
