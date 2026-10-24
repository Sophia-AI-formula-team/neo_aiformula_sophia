"""Current positive-pixel rays only; no line fitting, resampling or map input."""
import hashlib
import json


def rays(uv, inverse, rotation, translation, np):
    direction = np.column_stack((uv, np.ones(len(uv)))) @ inverse.T @ rotation.T
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = -translation[2] / direction[:, 2]
        points = direction * scale[:, None] + translation
    valid = (direction[:, 2] < -1e-8) & (scale > 0) & np.isfinite(points).all(axis=1)
    return points, valid


def project_current_pixels(mask, camera, parameters, np, cv2):
    image = np.asarray(mask)
    if image.ndim != 2 or min(image.shape) < 2 or max(image.shape) > 4096 or not np.isfinite(image).all():
        raise ValueError("finite_two_dimensional_mask_required")
    k = np.asarray(camera["K"], float).reshape(3, 3)
    r = np.asarray(camera["R"], float).reshape(3, 3)
    t = np.asarray(camera["t"], float).reshape(3)
    if (not np.isfinite(k).all() or not np.isfinite(r).all() or not np.isfinite(t).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0 or abs(np.linalg.det(k)) < 1e-9
            or not np.allclose(k[2], [0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0)
            or abs(np.linalg.det(r)-1) > 1e-5 or t[2] <= 0):
        raise ValueError("invalid_existing_camera_K_R_t")
    binary = (image > 0).astype(np.uint8)
    v, u = np.where(binary)
    if len(u) > parameters.max_positive_pixels:
        raise ValueError("positive_pixel_budget_exceeded")
    if not len(u):
        raise ValueError("no_current_positive_pixels")
    uv = np.column_stack((u, v))
    inverse = np.linalg.inv(k)
    points, valid = rays(uv, inverse, r, t, np)
    keep = valid & (points[:, 0] >= 0) & (points[:, 0] <= parameters.max_forward_m) & (np.abs(points[:, 1]) <= parameters.max_lateral_m)
    _, labels = cv2.connectedComponents(binary, connectivity=8)
    flags = {}
    for component in np.unique(labels[v, u]):
        member = labels[v, u] == component
        flags[int(component)] = []
        if not keep[member].all():
            flags[int(component)].append("ground_domain_or_horizon_truncated")
        selected = uv[member]
        if ((selected[:, 0] == 0) | (selected[:, 0] == image.shape[1]-1)
                | (selected[:, 1] == 0) | (selected[:, 1] == image.shape[0]-1)).any():
            flags[int(component)].append("image_boundary_truncated")
    uv, points = uv[keep], points[keep]
    if not len(points):
        raise ValueError("no_pixels_in_ground_domain")
    bounds = np.zeros((len(points), 2))
    known = np.ones(len(points), dtype=bool)
    half = parameters.pixel_quantization_half_width
    for dx, dy in ((-half, -half), (-half, half), (half, -half), (half, half)):
        other, accepted = rays(uv + [dx, dy], inverse, r, t, np)
        known &= accepted
        bounds = np.maximum(bounds, np.where(accepted[:, None], np.abs(other[:, :2]-points[:, :2]), 0))
    observations = []
    for pixel, point, bound, bounded in zip(uv, points, bounds, known):
        x, y = (int(number) for number in pixel)
        observations.append(dict(pixel_id=y*image.shape[1]+x, uv=[x, y], ground_xyz=point.tolist(),
            component_id=int(labels[y, x]), pixel_quantization_bound_xy_m=bound.tolist() if bounded else None,
            uncertainty="unknown_total; quantization_only"))
    calibration = dict(K=k.tolist(), R=r.tolist(), t=t.tolist())
    evidence = dict(observed_points=observations, positive_pixel_count=int(len(u)),
        projected_pixel_count=len(observations), rejected_projection_pixel_count=int(len(u)-len(observations)),
        image_size=[int(image.shape[1]), int(image.shape[0])],
        calibration_canonical_sha256=hashlib.sha256(json.dumps(calibration, sort_keys=True).encode()).hexdigest(),
        projection_assumptions=dict(ground_plane="existing base z=0", axes="x forward, y left, z up",
            total_uncertainty="unknown camera/segmentation/ground errors", quantization_is_not_total_error=True))
    return evidence, binary, labels, flags
