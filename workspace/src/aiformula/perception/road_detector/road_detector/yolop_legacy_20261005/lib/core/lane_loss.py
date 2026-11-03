import torch
import torch.nn as nn
import torch.nn.functional as F


def _build_valid_mask(shapes, mask_shape, device):
    batch_size, height, width = mask_shape
    valid_mask = torch.ones((batch_size, height, width), dtype=torch.bool, device=device)

    if shapes is None:
        return valid_mask

    valid_mask.zero_()
    for batch_idx, shape in enumerate(shapes):
        pad_w, pad_h = shape[1][1]
        pad_w = int(round(pad_w))
        pad_h = int(round(pad_h))
        valid_mask[batch_idx, pad_h:height - pad_h, pad_w:width - pad_w] = True

    return valid_mask


def get_lane_foreground_logit(lane_logits):
    if lane_logits.ndim != 4:
        raise ValueError("Lane logits must have shape [B, C, H, W].")
    if lane_logits.size(1) == 1:
        return lane_logits[:, 0]
    if lane_logits.size(1) == 2:
        return lane_logits[:, 1] - lane_logits[:, 0]
    raise ValueError("LaneSegLoss expects logits with shape [B, 1, H, W] or [B, 2, H, W].")


def lane_prob_from_logits(lane_logits):
    return torch.sigmoid(get_lane_foreground_logit(lane_logits))


def lane_pred_from_logits(lane_logits, threshold=0.5):
    return (lane_prob_from_logits(lane_logits) >= threshold).long()


def compute_binary_segmentation_metrics(pred_mask, target_mask, roi_mask=None, eps=1e-6):
    pred_mask = pred_mask.long()
    target_mask = target_mask.long()

    if roi_mask is None:
        roi_mask = torch.ones_like(target_mask, dtype=torch.bool)
    else:
        roi_mask = roi_mask.to(device=target_mask.device, dtype=torch.bool)

    pred_fg = (pred_mask == 1) & roi_mask
    pred_bg = (pred_mask == 0) & roi_mask
    target_fg = (target_mask == 1) & roi_mask
    target_bg = (target_mask == 0) & roi_mask

    tp = (pred_fg & target_fg).sum().float()
    fp = (pred_fg & target_bg).sum().float()
    fn = (pred_bg & target_fg).sum().float()
    tn = (pred_bg & target_bg).sum().float()
    total = (tp + fp + fn + tn).clamp_min(1.0)

    lane_iou = tp / (tp + fp + fn + eps)
    background_iou = tn / (tn + fp + fn + eps)
    dice = (2.0 * tp) / (2.0 * tp + fp + fn + eps)
    pixel_acc = (tp + tn) / total
    positive_recall = tp / (tp + fn + eps)
    positive_precision = tp / (tp + fp + eps)
    pred_positive_ratio = (tp + fp) / total
    target_positive_ratio = (tp + fn) / total
    mean_iou = (lane_iou + background_iou) / 2.0

    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "tn": tn,
        "pixel_acc": pixel_acc,
        "lane_iou": lane_iou,
        "background_iou": background_iou,
        "mean_iou": mean_iou,
        "dice": dice,
        "positive_pixel_recall": positive_recall,
        "positive_pixel_precision": positive_precision,
        "pred_positive_ratio": pred_positive_ratio,
        "target_positive_ratio": target_positive_ratio,
        "valid_pixels": total,
    }


class LaneSegLoss(nn.Module):
    def __init__(self, cfg, lane_pos_weight=None):
        super().__init__()
        self.cfg = cfg
        self.bce_weight = cfg.LOSS.LANE_BCE_WEIGHT
        self.dice_weight = cfg.LOSS.LANE_DICE_WEIGHT
        self.hard_negative_ratio = cfg.LOSS.HARD_NEGATIVE_RATIO
        self.pred_threshold = cfg.LOSS.PRED_THRESHOLD

        if lane_pos_weight is None:
            lane_pos_weight = cfg.LOSS.LANE_POS_WEIGHT
        self.register_buffer("pos_weight", torch.tensor(float(lane_pos_weight), dtype=torch.float32))

    def forward(self, lane_logits, lane_target, shapes=None, roi_mask=None, compute_metrics=True):
        lane_target = lane_target.long()
        if roi_mask is None:
            roi_mask = _build_valid_mask(shapes, lane_target.shape, lane_target.device)
        else:
            roi_mask = roi_mask.to(device=lane_target.device, dtype=torch.bool)

        lane_fg_logit = get_lane_foreground_logit(lane_logits)
        lane_target_float = lane_target.float()

        bce_map = F.binary_cross_entropy_with_logits(
            lane_fg_logit,
            lane_target_float,
            reduction="none",
            pos_weight=self.pos_weight,
        )
        bce_loss = self._reduce_loss_map(bce_map, lane_target, roi_mask)

        dice_loss = lane_fg_logit.new_zeros(())
        if self.dice_weight > 0:
            dice_loss = self._dice_loss(lane_fg_logit, lane_target_float, roi_mask)

        total_loss = self.bce_weight * bce_loss + self.dice_weight * dice_loss

        metrics = {}
        if compute_metrics:
            with torch.no_grad():
                pred_mask = lane_pred_from_logits(lane_logits, threshold=self.pred_threshold)
                metric_tensors = compute_binary_segmentation_metrics(pred_mask, lane_target, roi_mask)

            metrics = {
                "lane_bce": bce_loss.detach(),
                "lane_dice": dice_loss.detach(),
                "loss": total_loss.detach(),
            }
            metrics.update(
                {
                    key: (value.detach() if isinstance(value, torch.Tensor) else value)
                    for key, value in metric_tensors.items()
                    if key not in {"tp", "fp", "fn", "tn"}
                }
            )
        return total_loss, metrics

    def _reduce_loss_map(self, loss_map, lane_target, roi_mask):
        if self.hard_negative_ratio > 0:
            reduced = self._reduce_with_hard_negatives(loss_map, lane_target, roi_mask)
            if reduced is not None:
                return reduced

        masked_loss = loss_map[roi_mask]
        if masked_loss.numel() == 0:
            return loss_map.mean() * 0.0
        return masked_loss.mean()

    def _reduce_with_hard_negatives(self, loss_map, lane_target, roi_mask):
        sample_losses = []

        for sample_loss, sample_target, sample_roi in zip(loss_map, lane_target, roi_mask):
            positive_mask = (sample_target == 1) & sample_roi
            negative_mask = (sample_target == 0) & sample_roi

            positive_loss = sample_loss[positive_mask]
            negative_loss = sample_loss[negative_mask]

            if positive_loss.numel() == 0 or negative_loss.numel() == 0:
                masked_loss = sample_loss[sample_roi]
                if masked_loss.numel() > 0:
                    sample_losses.append(masked_loss.mean())
                continue

            max_negatives = max(int(self.hard_negative_ratio * positive_loss.numel()), 1)
            max_negatives = min(max_negatives, negative_loss.numel())
            hard_negative_loss, _ = torch.topk(negative_loss, max_negatives)
            sample_losses.append(torch.cat((positive_loss, hard_negative_loss), dim=0).mean())

        if not sample_losses:
            return None
        return torch.stack(sample_losses).mean()

    @staticmethod
    def _dice_loss(lane_fg_logit, lane_target, roi_mask, eps=1e-6):
        probs = torch.sigmoid(lane_fg_logit)
        mask = roi_mask.float()

        intersection = (probs * lane_target * mask).sum(dim=(1, 2))
        denominator = ((probs + lane_target) * mask).sum(dim=(1, 2))
        dice = 1.0 - ((2.0 * intersection + eps) / (denominator + eps))
        return dice.mean()


def get_loss(cfg, device=None, lane_pos_weight=None):
    loss = LaneSegLoss(cfg, lane_pos_weight=lane_pos_weight)
    if device is not None:
        loss = loss.to(device)
    return loss


__all__ = [
    "LaneSegLoss",
    "compute_binary_segmentation_metrics",
    "get_lane_foreground_logit",
    "get_loss",
    "lane_pred_from_logits",
    "lane_prob_from_logits",
]
