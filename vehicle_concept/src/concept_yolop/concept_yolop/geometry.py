"""Recognition can run with any rectified 16:9 image. Metric driving needs a measured view."""
import json
from pathlib import Path

import numpy as np

UNVERIFIED = 'unverified-resize'
CALIBRATED = 'calibrated-v626'
UNITY = 'frozen-unity-v626'
PROFILES = (UNVERIFIED, CALIBRATED, UNITY)
FX, FY, CX, CY, HEIGHT = 254.391622, 254.391622, 330.01302, 181.14964, .655


class CalibrationReceipt:
    """Verify held-out ground marks, then bind them to the live rectified stream."""
    def __init__(self, path):
        data = json.loads(Path(path).expanduser().read_text(encoding='utf8'))
        self.topic, self.frame_id = data['image_topic'], data['frame_id']
        self.width, self.height = int(data['width']), int(data['height'])
        self.projection = np.asarray(data['projection'], dtype=float).reshape(3, 4)
        if (not self.frame_id or self.width <= 0 or self.height <= 0 or self.width * 9 != self.height * 16
                or not np.isfinite(self.projection).all() or not data['mount_id']):
            raise ValueError('Incomplete rectified camera registration')
        # A fixed ray-to-ground transform is what matters to the existing planner.
        # Use actual camera pixels and independent vehicle-relative measured positions.
        marks = np.asarray(data['ground_marks'], dtype=float)
        if marks.ndim != 2 or marks.shape[1] != 4 or len(marks) < 4 or not np.isfinite(marks).all():
            raise ValueError('Need measured ground samples [image_u,image_v,right_m,forward_m]')
        u, v, right, forward = marks.T
        if (np.any(u < 0) or np.any(u >= self.width) or np.any(v < 0) or np.any(v >= self.height)
                or np.any(forward <= 0) or forward.min() > 3.0 or forward.max() < 7.0
                or right.min() > -.5 or right.max() < .5):
            raise ValueError('Ground check must cover nearby, distant and both lateral sides')
        u = (u + .5) * 640 / self.width - .5
        v = (v + .5) * 360 / self.height - .5
        if np.any(v <= CY):
            raise ValueError('Ground projection crosses the optical horizon')
        estimated_z = HEIGHT * FY / (v - CY)
        error = np.hypot((u - CX) * estimated_z / FX - right, estimated_z - forward)
        if np.max(error) > .10:
            raise ValueError('Measured camera projection differs from V6.26 by more than 0.10 m')
        self.max_error = float(np.max(error))

    def matches(self, topic, image, info):
        if info is None:
            return False
        return (topic == self.topic and image.header.frame_id == self.frame_id == info.header.frame_id
                and (image.width, image.height) == (info.width, info.height) == (self.width, self.height)
                and np.allclose(np.array(info.p).reshape(3, 4), self.projection, rtol=0., atol=.001))
