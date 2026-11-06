"""Current-to-previous ground-plane sampling in native pixel-center coordinates."""
from functools import lru_cache
import numpy as np
import torch


@lru_cache(maxsize=2)
def pixels(height, width, device):
    y, x = torch.meshgrid(torch.arange(height, device=device, dtype=torch.float32),
                          torch.arange(width, device=device, dtype=torch.float32), indexing='ij')
    return x, y


def validate_matrix(matrix):
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (3, 3) or not np.isfinite(value).all() or abs(np.linalg.det(value)) < 1e-12:
        raise ValueError('Finite nonsingular 3x3 homography required')
    return value


def sampling_grid(previous_plane, current_plane, height, width, *, aligned=True, device='cuda', near=.05, far=5000.):
    if not 0 < near < far:
        raise ValueError('Positive increasing camera clip range required')
    previous, current = validate_matrix(previous_plane), validate_matrix(current_plane)
    transfer = previous @ np.linalg.inv(current) if aligned else np.eye(3)
    x, y = pixels(height, width, str(device))
    h = torch.as_tensor(transfer, dtype=torch.float32, device=device)
    denominator = h[2, 0] * x + h[2, 1] * y + h[2, 2]
    safe = denominator.abs() > 1e-7
    divisor = torch.where(safe, denominator, torch.ones_like(denominator))
    px = (h[0, 0] * x + h[0, 1] * y + h[0, 2]) / divisor
    py = (h[1, 0] * x + h[1, 1] * y + h[1, 2]) / divisor
    valid = safe & (px >= 0) & (px <= width - 1) & (py >= 0) & (py <= height - 1)
    for plane, xx, yy in ((current, x, y), (previous, px, py)):
        inverse = torch.as_tensor(np.linalg.inv(plane)[2], dtype=torch.float32, device=device)
        inverse_depth = inverse[0] * xx + inverse[1] * yy + inverse[2]
        # Valid positive ground depth; near/far bounds match the audited camera.
        valid &= (inverse_depth >= 1 / far) & (inverse_depth <= 1 / near)
    grid = torch.stack((2 * (px + .5) / width - 1, 2 * (py + .5) / height - 1), -1)
    grid = torch.where(valid[..., None], grid, torch.full_like(grid, 2.))
    return grid[None], valid[None, None].float()
