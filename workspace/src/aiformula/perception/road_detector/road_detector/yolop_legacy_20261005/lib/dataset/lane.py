from pathlib import Path
import random
import warnings

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from ..utils.augmentations import augment_hsv, letterbox, random_perspective


class LaneDataset(Dataset):
    def __init__(self, cfg, is_train, inputsize=640, transform=None):
        self.cfg = cfg
        self.is_train = is_train
        self.transform = transform
        self.inputsize = inputsize
        self.data_format = cfg.DATASET.DATA_FORMAT.lstrip(".")
        self.lane_mask_format = cfg.DATASET.LANE_MASK_FORMAT.lstrip(".").lower()
        self.dataset_root = Path(cfg.DATASET.ROOT)
        self.image_dir_name = cfg.DATASET.IMAGE_DIR
        self.lane_mask_dir_name = cfg.DATASET.LANE_MASK_DIR

        indicator = cfg.DATASET.TRAIN_SET if is_train else cfg.DATASET.TEST_SET
        self.img_root = self.dataset_root / indicator / self.image_dir_name
        self.lane_root = self.dataset_root / indicator / self.lane_mask_dir_name
        self.db = self._build_db()

    def _build_db(self):
        if not self.dataset_root.exists():
            raise FileNotFoundError(f"Dataset root does not exist: {self.dataset_root}")
        if not self.img_root.exists():
            raise FileNotFoundError(f"Image root does not exist: {self.img_root}")
        if not self.lane_root.exists():
            raise FileNotFoundError(f"Lane root does not exist: {self.lane_root}")

        lane_paths = sorted(
            path
            for path in self.lane_root.iterdir()
            if path.is_file() and path.suffix.lower() == f".{self.lane_mask_format}"
        )
        db = []

        for lane_path in lane_paths:
            image_path = self.img_root / f"{lane_path.stem}.{self.data_format}"
            if image_path.exists():
                db.append({"image": str(image_path), "lane": str(lane_path)})

        image_count = sum(
            1
            for path in self.img_root.iterdir()
            if path.is_file() and path.suffix.lower() == f".{self.data_format.lower()}"
        )
        if image_count and len(db) != image_count:
            warnings.warn(
                (
                    f"Matched {len(db)} lane masks with format .{self.lane_mask_format} under {self.lane_root}, "
                    f"but found {image_count} images under {self.img_root}. "
                    "Images without matching lane masks will be skipped."
                ),
                stacklevel=2,
            )

        if not db:
            raise FileNotFoundError(
                f"No lane samples found under {self.lane_root} with mask format .{self.lane_mask_format} "
                f"matching image format .{self.data_format}"
            )

        return db

    def __len__(self):
        return len(self.db)

    def estimate_positive_fraction(self, max_samples=16):
        sample_count = min(max_samples, len(self.db))
        if sample_count == 0:
            return 0.0, 0

        if sample_count == 1:
            sample_indices = [0]
        else:
            sample_indices = np.linspace(0, len(self.db) - 1, sample_count, dtype=int)

        positive_pixels = 0
        total_pixels = 0

        for idx in sample_indices:
            lane_label = cv2.imread(self.db[int(idx)]["lane"], cv2.IMREAD_GRAYSCALE)
            if lane_label is None:
                raise FileNotFoundError(f"Unable to read lane mask: {self.db[int(idx)]['lane']}")
            positive_pixels += int((lane_label > 0).sum())
            total_pixels += int(lane_label.size)

        positive_fraction = positive_pixels / max(total_pixels, 1)
        return positive_fraction, sample_count

    def __getitem__(self, idx):
        data = self.db[idx]
        img = cv2.imread(data["image"], cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if img is None:
            raise FileNotFoundError(f"Unable to read image: {data['image']}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        lane_label = cv2.imread(data["lane"], cv2.IMREAD_GRAYSCALE)
        if lane_label is None:
            raise FileNotFoundError(f"Unable to read lane mask: {data['lane']}")

        target_shape = tuple(self.inputsize) if isinstance(self.inputsize, (list, tuple)) else (self.inputsize, self.inputsize)
        resized_shape = max(target_shape)
        h0, w0 = img.shape[:2]
        resize_ratio = resized_shape / max(h0, w0)

        if resize_ratio != 1:
            interp = cv2.INTER_AREA if resize_ratio < 1 else cv2.INTER_LINEAR
            img = cv2.resize(img, (int(w0 * resize_ratio), int(h0 * resize_ratio)), interpolation=interp)
            lane_label = cv2.resize(
                lane_label,
                (int(w0 * resize_ratio), int(h0 * resize_ratio)),
                interpolation=cv2.INTER_NEAREST,
            )

        h, w = img.shape[:2]
        (img, lane_label), ratio, pad = letterbox(
            (img, lane_label),
            target_shape,
            auto=False,
            scaleup=self.is_train,
        )
        shapes = (h0, w0), ((h / h0, w / w0), pad)

        if self.is_train:
            (img, lane_label), _ = random_perspective(
                combination=(img, lane_label),
                targets=np.zeros((0, 5), dtype=np.float32),
                degrees=self.cfg.DATASET.ROT_FACTOR,
                translate=self.cfg.DATASET.TRANSLATE,
                scale=self.cfg.DATASET.SCALE_FACTOR,
                shear=self.cfg.DATASET.SHEAR,
            )
            augment_hsv(
                img,
                hgain=self.cfg.DATASET.HSV_H,
                sgain=self.cfg.DATASET.HSV_S,
                vgain=self.cfg.DATASET.HSV_V,
                color_format="rgb",
            )

            if self.cfg.DATASET.FLIP and random.random() < 0.5:
                img = np.fliplr(img)
                lane_label = np.fliplr(lane_label)

        img = np.ascontiguousarray(img)
        lane_mask = (lane_label > 0).astype(np.int64)
        lane_mask = torch.from_numpy(np.ascontiguousarray(lane_mask))

        if self.transform is not None:
            img = self.transform(img)

        return img, lane_mask, data["image"], shapes

    @staticmethod
    def collate_fn(batch):
        img, lane_mask, paths, shapes = zip(*batch)
        return torch.stack(img, 0), torch.stack(lane_mask, 0), paths, shapes
