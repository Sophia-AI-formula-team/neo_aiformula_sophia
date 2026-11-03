import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from tqdm import tqdm

from lib.core.lane_loss import compute_binary_segmentation_metrics, lane_prob_from_logits
from lib.utils.augmentations import letterbox
from lib.utils.lane_postprocess import apply_lane_postprocess
from lib.utils.plot import save_lane_visual_bundle
from lib.utils.utils import describe_input_batch, gpu_memory_stats_mb, move_inputs_to_device, time_synchronized


IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


def restore_lane_map(array, shape, interpolation):
    if isinstance(array, torch.Tensor):
        array = array.detach().cpu().numpy()

    if array.dtype == np.float16:
        array = array.astype(np.float32, copy=False)

    (orig_h, orig_w), (_, pad) = shape
    pad_w, pad_h = pad
    pad_w = int(round(pad_w))
    pad_h = int(round(pad_h))

    cropped = array[pad_h:array.shape[0] - pad_h, pad_w:array.shape[1] - pad_w]
    if cropped.size == 0:
        raise ValueError(
            f"restore_lane_map produced an empty crop for shape={shape}, array_shape={array.shape}"
        )
    return cv2.resize(cropped, (orig_w, orig_h), interpolation=interpolation)


def restore_lane_mask(mask, shape):
    restored = restore_lane_map(
        mask.astype(np.uint8) if isinstance(mask, np.ndarray) else mask,
        shape,
        cv2.INTER_NEAREST,
    )
    return restored.astype(np.uint8)


def load_vis_config_entries(config):
    if not config.TEST.VIS_USE_FIXED_SET:
        return []

    vis_config_path = Path(config.TEST.VIS_CONFIG)
    if not vis_config_path.is_absolute():
        vis_config_path = Path(__file__).resolve().parents[2] / vis_config_path
    if not vis_config_path.exists():
        return []

    vis_entries = []
    for line in vis_config_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        for token in stripped.replace(",", " ").split():
            vis_entries.append(token)
    return vis_entries


def resolve_vis_config_samples(config, logger=None):
    entries = load_vis_config_entries(config)
    if not entries:
        return [], []

    dataset_root = Path(config.DATASET.ROOT)
    image_ext = config.DATASET.DATA_FORMAT.lstrip(".")
    mask_ext = config.DATASET.LANE_MASK_FORMAT.lstrip(".")
    default_splits = [config.DATASET.TRAIN_SET, config.DATASET.TEST_SET]

    samples = []
    missing = []
    for entry in entries:
        normalized = entry.replace("\\", "/").strip()
        split_override = None
        identifier = normalized

        if ":" in normalized:
            prefix, remainder = normalized.split(":", 1)
            if prefix in {config.DATASET.TRAIN_SET, config.DATASET.TEST_SET, "train", "val"}:
                split_override = config.DATASET.TRAIN_SET if prefix == "train" else config.DATASET.TEST_SET if prefix == "val" else prefix
                identifier = remainder
        elif "/" in normalized:
            prefix, remainder = normalized.split("/", 1)
            if prefix in {config.DATASET.TRAIN_SET, config.DATASET.TEST_SET, "train", "val"}:
                split_override = config.DATASET.TRAIN_SET if prefix == "train" else config.DATASET.TEST_SET if prefix == "val" else prefix
                identifier = remainder

        stem = Path(identifier).stem
        search_splits = [split_override] if split_override is not None else default_splits
        matched_sample = None
        for split in search_splits:
            image_path = dataset_root / split / config.DATASET.IMAGE_DIR / f"{stem}.{image_ext}"
            lane_path = dataset_root / split / config.DATASET.LANE_MASK_DIR / f"{stem}.{mask_ext}"
            if image_path.exists() and lane_path.exists():
                matched_sample = {
                    "entry": entry,
                    "split": split,
                    "stem": stem,
                    "image": str(image_path),
                    "lane": str(lane_path),
                }
                break

        if matched_sample is None:
            missing.append(entry)
        else:
            samples.append(matched_sample)

    if logger is not None:
        logger.info("=> visualization config resolved %d samples from '%s'", len(samples), config.TEST.VIS_CONFIG)
        if missing:
            logger.warning(
                "=> %d visualization entries were not found in %s/%s: %s",
                len(missing),
                config.DATASET.TRAIN_SET,
                config.DATASET.TEST_SET,
                ", ".join(missing[:10]),
            )

    return samples, missing


def build_vis_output_dir(output_dir, config, tag):
    return Path(output_dir) / config.TEST.VIS_OUTPUT_DIR / Path(tag)


def preprocess_visualization_sample(config, image_bgr, lane_mask):
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    target_shape = tuple(config.MODEL.IMAGE_SIZE) if isinstance(config.MODEL.IMAGE_SIZE, (list, tuple)) else (config.MODEL.IMAGE_SIZE, config.MODEL.IMAGE_SIZE)
    resized_shape = max(target_shape)

    h0, w0 = image_rgb.shape[:2]
    resize_ratio = resized_shape / max(h0, w0)
    if resize_ratio != 1:
        interpolation = cv2.INTER_AREA if resize_ratio < 1 else cv2.INTER_LINEAR
        image_rgb = cv2.resize(image_rgb, (int(w0 * resize_ratio), int(h0 * resize_ratio)), interpolation=interpolation)
        lane_mask = cv2.resize(
            lane_mask,
            (int(w0 * resize_ratio), int(h0 * resize_ratio)),
            interpolation=cv2.INTER_NEAREST,
        )

    h, w = image_rgb.shape[:2]
    (image_rgb, lane_mask), _, pad = letterbox(
        (image_rgb, lane_mask),
        target_shape,
        auto=False,
        scaleup=False,
    )
    shapes = (h0, w0), ((h / h0, w / w0), pad)

    image_rgb = image_rgb.astype(np.float32) / 255.0
    image_rgb = (image_rgb - IMAGENET_MEAN) / IMAGENET_STD
    image_tensor = torch.from_numpy(np.ascontiguousarray(image_rgb.transpose(2, 0, 1))).unsqueeze(0)
    target_tensor = torch.from_numpy(np.ascontiguousarray((lane_mask > 0).astype(np.int64))).unsqueeze(0)
    return image_tensor, target_tensor, shapes


def save_configured_visualizations(config, model, device, output_dir, tag, logger=None):
    samples, _ = resolve_vis_config_samples(config, logger=logger)
    if not samples:
        return 0

    vis_output_dir = build_vis_output_dir(output_dir, config, tag)
    vis_output_dir.mkdir(parents=True, exist_ok=True)
    use_channels_last = bool(getattr(config, "CHANNELS_LAST", False)) and device.type == "cuda"

    saved_count = 0
    model.eval()
    with torch.no_grad():
        for sample in samples:
            image_bgr = cv2.imread(sample["image"], cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
            lane_mask = cv2.imread(sample["lane"], cv2.IMREAD_GRAYSCALE)
            if image_bgr is None or lane_mask is None:
                if logger is not None:
                    logger.warning("=> skipping visualization sample '%s' because the image or mask could not be read", sample["entry"])
                continue

            input_tensor, _, shapes = preprocess_visualization_sample(config, image_bgr, lane_mask)
            input_tensor = move_inputs_to_device(input_tensor, device, channels_last=use_channels_last)

            lane_logits = model(input_tensor)
            lane_prob = lane_prob_from_logits(lane_logits)[0]
            restored_prob = restore_lane_map(lane_prob, shapes, cv2.INTER_LINEAR).astype(np.float32)
            restored_pred, _ = apply_lane_postprocess(
                restored_prob,
                config,
                threshold=config.LOSS.PRED_THRESHOLD,
            )
            restored_gt = (lane_mask > 0).astype(np.uint8)

            save_lane_visual_bundle(
                vis_output_dir,
                f"{sample['split']}_{sample['stem']}",
                image_bgr,
                target_mask=restored_gt,
                pred_prob=restored_prob,
                pred_binary=restored_pred,
            )
            saved_count += 1

    if logger is not None:
        logger.info("=> saved %d visualization bundles to '%s'", saved_count, vis_output_dir)
    return saved_count


def train(cfg, train_loader, model, criterion, optimizer, scaler, epoch, num_batch, num_warmup,
          writer_dict, logger, device, rank=-1):
    batch_time = AverageMeter()
    data_time = AverageMeter()
    channels_last = bool(getattr(cfg, "CHANNELS_LAST", False)) and device.type == "cuda"
    log_metrics = rank in [-1, 0]
    perf_log = bool(getattr(cfg.TRAIN, "PERF_LOG", False))
    logged_batch_info = False

    loss_sum = torch.zeros((), device=device)
    sample_count = 0

    model.train()
    start = time.time()

    for i, (inputs, target, paths, shapes) in enumerate(train_loader):
        num_iter = i + num_batch * (epoch - 1)
        if num_iter < num_warmup:
            lf = lambda x: ((1 + math.cos(x * math.pi / cfg.TRAIN.END_EPOCH)) / 2) * \
                           (1 - cfg.TRAIN.LRF) + cfg.TRAIN.LRF
            xi = [0, num_warmup]
            for j, group in enumerate(optimizer.param_groups):
                group["lr"] = np.interp(
                    num_iter,
                    xi,
                    [cfg.TRAIN.WARMUP_BIASE_LR if j == 2 else 0.0, group["initial_lr"] * lf(epoch)],
                )
                if "momentum" in group:
                    group["momentum"] = np.interp(
                        num_iter,
                        xi,
                        [cfg.TRAIN.WARMUP_MOMENTUM, cfg.TRAIN.MOMENTUM],
                    )

        data_time.update(time.time() - start)

        if not cfg.DEBUG:
            inputs = move_inputs_to_device(
                inputs,
                device,
                channels_last=channels_last,
            )
            target = target.to(device, non_blocking=True)

        if log_metrics and perf_log and not logged_batch_info:
            batch_info = describe_input_batch(inputs)
            logger.info(
                "Input batch: batch_size=%d shape=%s dtype=%s memory_format=%s",
                batch_info["batch_size"],
                batch_info["shape"],
                batch_info["dtype"],
                batch_info["memory_format"],
            )
            logged_batch_info = True

        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast(device_type=device.type, enabled=device.type != "cpu"):
            lane_logits = model(inputs)
            total_loss, _ = criterion(lane_logits, target, shapes, compute_metrics=False)

        scaler.scale(total_loss).backward()
        scaler.step(optimizer)
        scaler.update()

        if log_metrics:
            batch_size = inputs.size(0)
            batch_time.update(time.time() - start)
            sample_count += batch_size
            loss_sum += total_loss.detach() * batch_size

            if i % cfg.PRINT_FREQ == 0 or i == len(train_loader) - 1:
                window_count = max(sample_count, 1)
                loss_avg = (loss_sum / window_count).item()
                samples_per_sec = sample_count / max(batch_time.sum, 1e-6)
                msg = (
                    "Epoch: [{0}][{1}/{2}]\t"
                    "Batch {batch_time.val:.3f}s ({batch_time.avg:.3f}s)\t"
                    "Speed {speed:.1f} samples/s\t"
                    "Data {data_time.val:.3f}s ({data_time.avg:.3f}s)\t"
                    "Loss {loss:.5f}"
                ).format(
                    epoch,
                    i,
                    len(train_loader),
                    batch_time=batch_time,
                    speed=samples_per_sec,
                    data_time=data_time,
                    loss=loss_avg,
                )
                if perf_log:
                    mem_stats = gpu_memory_stats_mb(device)
                    msg += "\tGPU alloc/resv {:.0f}/{:.0f}MB".format(
                        mem_stats["allocated_mb"],
                        mem_stats["reserved_mb"],
                    )
                logger.info(msg)

                if writer_dict is not None:
                    writer = writer_dict["writer"]
                    global_steps = writer_dict["train_global_steps"]
                    writer.add_scalar("train_loss", loss_avg, global_steps)
                    writer.add_scalar("train_samples_per_sec", samples_per_sec, global_steps)
                    writer.add_scalar("train_batch_time", batch_time.avg, global_steps)
                    writer.add_scalar("train_data_time", data_time.avg, global_steps)
                    if perf_log and device.type == "cuda":
                        writer.add_scalar("train_gpu_allocated_mb", mem_stats["allocated_mb"], global_steps)
                        writer.add_scalar("train_gpu_reserved_mb", mem_stats["reserved_mb"], global_steps)
                    writer_dict["train_global_steps"] = global_steps + 1

                batch_time.reset()
                data_time.reset()
                loss_sum.zero_()
                sample_count = 0

        start = time.time()


def validate(epoch, config, val_loader, model, criterion, output_dir,
             writer_dict=None, logger=None, device="cpu", rank=-1, eval_tag=None):
    metric = BinarySegmentationMeter()
    losses = AverageMeter()
    inf_time = AverageMeter()

    model.eval()

    for _, (img, target, paths, shapes) in tqdm(enumerate(val_loader), total=len(val_loader)):
        if not config.DEBUG:
            img = move_inputs_to_device(
                img,
                device,
                channels_last=bool(getattr(config, "CHANNELS_LAST", False)) and device.type == "cuda",
            )
            target = target.to(device, non_blocking=True)

        with torch.no_grad(), torch.amp.autocast(device_type=device.type, enabled=device.type != "cpu"):
            start = time_synchronized()
            lane_logits = model(img)
            elapsed = time_synchronized() - start
            inf_time.update(elapsed / img.size(0), img.size(0))

            total_loss, _ = criterion(lane_logits, target, shapes, compute_metrics=False)
            losses.update(total_loss.item(), img.size(0))

            lane_prob = lane_prob_from_logits(lane_logits)

        for sample_idx in range(img.size(0)):
            restored_prob = restore_lane_map(lane_prob[sample_idx], shapes[sample_idx], cv2.INTER_LINEAR).astype(np.float32)
            restored_gt = restore_lane_mask(target[sample_idx], shapes[sample_idx])
            restored_pred, component_stats = apply_lane_postprocess(
                restored_prob,
                config,
                threshold=config.LOSS.PRED_THRESHOLD,
            )
            metric.add(restored_pred, restored_gt, component_stats)

    lane_result = metric.summary(config.LOSS.PRED_THRESHOLD)

    if writer_dict is not None and rank in [-1, 0]:
        writer = writer_dict["writer"]
        global_steps = writer_dict["valid_global_steps"]
        writer.add_scalar("valid_loss", losses.avg, global_steps)
        writer.add_scalar("valid_pixel_acc", lane_result["pixel_acc"], global_steps)
        writer.add_scalar("valid_lane_iou", lane_result["lane_iou"], global_steps)
        writer.add_scalar("valid_dice", lane_result["dice"], global_steps)
        writer.add_scalar("valid_positive_recall", lane_result["positive_pixel_recall"], global_steps)
        writer.add_scalar("valid_positive_precision", lane_result["positive_pixel_precision"], global_steps)
        writer.add_scalar("valid_pred_positive_ratio", lane_result["pred_positive_ratio"], global_steps)
        writer.add_scalar("valid_avg_components_per_image", lane_result["avg_components_per_image"], global_steps)
        writer_dict["valid_global_steps"] = global_steps + 1

    if logger is not None and lane_result["pred_to_target_ratio"] > config.POST.FP_WARNING_RATIO:
        logger.warning(
            "False-positive-heavy prediction: pred_positive_ratio %.4f vs target_positive_ratio %.4f (ratio %.2f).",
            lane_result["pred_positive_ratio"],
            lane_result["target_positive_ratio"],
            lane_result["pred_to_target_ratio"],
        )

    if config.TEST.PLOTS and rank in [-1, 0]:
        save_configured_visualizations(
            config,
            model,
            device,
            output_dir,
            eval_tag if eval_tag is not None else f"epoch_{epoch:04d}",
            logger=logger,
        )

    model.float()
    return lane_result, losses.avg, [inf_time.avg]


class BinarySegmentationMeter(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.tp = 0.0
        self.fp = 0.0
        self.fn = 0.0
        self.tn = 0.0
        self.component_count_sum = 0.0
        self.raw_component_count_sum = 0.0
        self.component_size_sum = 0.0
        self.image_count = 0

    def add(self, pred_mask, target_mask, component_stats=None):
        pred_tensor = torch.as_tensor(pred_mask, dtype=torch.long)
        target_tensor = torch.as_tensor(target_mask, dtype=torch.long)
        metrics = compute_binary_segmentation_metrics(pred_tensor, target_tensor)
        self.tp += metrics["tp"].item()
        self.fp += metrics["fp"].item()
        self.fn += metrics["fn"].item()
        self.tn += metrics["tn"].item()
        self.image_count += 1

        if component_stats is not None:
            self.component_count_sum += component_stats["num_components"]
            self.raw_component_count_sum += component_stats["raw_num_components"]
            self.component_size_sum += component_stats["avg_component_size"]

    def summary(self, threshold, eps=1e-6):
        tp = self.tp
        fp = self.fp
        fn = self.fn
        tn = self.tn
        total = max(tp + fp + fn + tn, 1.0)

        lane_iou = tp / (tp + fp + fn + eps)
        background_iou = tn / (tn + fp + fn + eps)
        dice = (2.0 * tp) / (2.0 * tp + fp + fn + eps)
        pixel_acc = (tp + tn) / total
        positive_recall = tp / (tp + fn + eps)
        positive_precision = tp / (tp + fp + eps)
        pred_positive_ratio = (tp + fp) / total
        target_positive_ratio = (tp + fn) / total
        mean_iou = (lane_iou + background_iou) / 2.0
        pred_to_target_ratio = pred_positive_ratio / max(target_positive_ratio, eps)
        divisor = max(self.image_count, 1)

        return {
            "pixel_acc": pixel_acc,
            "lane_iou": lane_iou,
            "background_iou": background_iou,
            "mean_iou": mean_iou,
            "dice": dice,
            "positive_pixel_recall": positive_recall,
            "positive_pixel_precision": positive_precision,
            "pred_positive_ratio": pred_positive_ratio,
            "target_positive_ratio": target_positive_ratio,
            "threshold": threshold,
            "avg_components_per_image": self.component_count_sum / divisor,
            "avg_raw_components_per_image": self.raw_component_count_sum / divisor,
            "avg_component_size": self.component_size_sum / divisor,
            "pred_to_target_ratio": pred_to_target_ratio,
        }


class AverageMeter(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count if self.count != 0 else 0
