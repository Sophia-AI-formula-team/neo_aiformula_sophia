from pathlib import Path

import cv2
import numpy as np


def overlay_lane_mask(img, lane_mask, color=(0, 0, 255), alpha=0.5, resize_to=None):
    overlay = img.copy()
    lane_mask = lane_mask.astype(bool)

    if lane_mask.any():
        color_array = np.array(color, dtype=np.float32)
        overlay[lane_mask] = (
            overlay[lane_mask].astype(np.float32) * (1.0 - alpha) + color_array * alpha
        )

    overlay = overlay.astype(np.uint8)
    if resize_to is not None:
        overlay = cv2.resize(overlay, resize_to, interpolation=cv2.INTER_LINEAR)
    return overlay


def probability_heatmap(prob_map):
    prob_map = np.clip(prob_map, 0.0, 1.0)
    heat = (prob_map * 255.0).astype(np.uint8)
    return cv2.applyColorMap(heat, cv2.COLORMAP_TURBO)


def save_lane_visual_bundle(output_dir, stem, image_bgr, target_mask=None, pred_prob=None, pred_binary=None):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(output_dir / f"{stem}_original.png"), image_bgr)

    if target_mask is not None:
        cv2.imwrite(str(output_dir / f"{stem}_mask.png"), target_mask.astype(np.uint8) * 255)

    if pred_prob is not None:
        cv2.imwrite(str(output_dir / f"{stem}_pred_prob.png"), probability_heatmap(pred_prob))

    if pred_binary is not None:
        pred_binary_uint8 = pred_binary.astype(np.uint8)
        cv2.imwrite(str(output_dir / f"{stem}_pred_binary.png"), pred_binary_uint8 * 255)
        overlay = overlay_lane_mask(image_bgr.copy(), pred_binary_uint8)
        cv2.imwrite(str(output_dir / f"{stem}_overlay.png"), overlay)
