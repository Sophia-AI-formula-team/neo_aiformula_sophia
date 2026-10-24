import cv2
import numpy as np


def component_diagnostics(binary_mask):
    binary_mask = binary_mask.astype(np.uint8)
    num_labels, _, stats, _ = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
    component_areas = stats[1:, cv2.CC_STAT_AREA].astype(np.float32) if num_labels > 1 else np.array([], dtype=np.float32)

    return {
        "num_components": float(component_areas.size),
        "avg_component_size": float(component_areas.mean()) if component_areas.size else 0.0,
        "largest_component_size": float(component_areas.max()) if component_areas.size else 0.0,
        "raw_num_components": float(component_areas.size),
    }


def filter_lane_components(binary_mask, cfg):
    binary_mask = binary_mask.astype(np.uint8)
    filtered = np.zeros_like(binary_mask)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)

    kept_areas = []
    raw_components = max(num_labels - 1, 0)

    for component_idx in range(1, num_labels):
        x = stats[component_idx, cv2.CC_STAT_LEFT]
        y = stats[component_idx, cv2.CC_STAT_TOP]
        width = stats[component_idx, cv2.CC_STAT_WIDTH]
        height = stats[component_idx, cv2.CC_STAT_HEIGHT]
        area = stats[component_idx, cv2.CC_STAT_AREA]

        aspect_ratio = width / max(height, 1)
        touches_bottom = (y + height) >= (binary_mask.shape[0] - cfg.POST.BOTTOM_TOUCH_MARGIN)

        keep = area >= cfg.POST.MIN_COMPONENT_AREA and (
            height >= cfg.POST.MIN_COMPONENT_HEIGHT
            or aspect_ratio <= cfg.POST.MAX_COMPONENT_ASPECT_RATIO
            or touches_bottom
        )
        if not keep:
            continue

        filtered[labels == component_idx] = 1
        kept_areas.append(float(area))

    return filtered, {
        "num_components": float(len(kept_areas)),
        "avg_component_size": float(np.mean(kept_areas)) if kept_areas else 0.0,
        "largest_component_size": float(np.max(kept_areas)) if kept_areas else 0.0,
        "raw_num_components": float(raw_components),
    }


def apply_lane_postprocess(prob_map, cfg, threshold=None):
    threshold = cfg.LOSS.PRED_THRESHOLD if threshold is None else threshold
    binary_mask = (prob_map >= threshold).astype(np.uint8)

    if not cfg.POST.ENABLED:
        diagnostics = component_diagnostics(binary_mask)
        return binary_mask, diagnostics

    processed = binary_mask
    if cfg.POST.VERTICAL_CLOSE:
        kernel_w, kernel_h = cfg.POST.VERTICAL_KERNEL
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_w, kernel_h))
        processed = cv2.morphologyEx(processed, cv2.MORPH_CLOSE, kernel)

    if cfg.POST.COMPONENT_FILTER:
        processed, diagnostics = filter_lane_components(processed, cfg)
    else:
        diagnostics = component_diagnostics(processed)

    return processed.astype(np.uint8), diagnostics
