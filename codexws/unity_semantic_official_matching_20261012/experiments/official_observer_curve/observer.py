"""Default-disabled anonymous curve evidence. Never emits a junction identity."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObserverParameters:
    max_forward_m: float = 10.0
    max_lateral_m: float = 4.0
    max_positive_pixels: int = 50000
    max_contour_pixels: int = 150000
    max_fragments: int = 2048
    max_split_work: int = 500000
    max_candidate_fragments: int = 128
    min_fragment_pixels: int = 8
    min_fragment_span_m: float = .6
    max_local_residual_m: float = .15
    min_inlier_fraction: float = .8
    max_main_heading_deg: float = 20.0
    min_side_heading_deg: float = 45.0
    max_parallel_difference_deg: float = 10.0
    min_side_separation_m: float = 1.5
    max_side_separation_m: float = 6.0
    min_side_overlap_m: float = 1.0
    min_main_before_m: float = .5
    min_main_continuation_m: float = 1.0
    min_main_support_pixels: int = 12
    max_fragment_gap_m: float = .5
    pixel_quantization_half_width: float = .5


def _empty(status, reason):
    return dict(schema="anonymous_lane_curve_geometry/v1", enabled=status != "disabled",
        status=status, reason=reason, frame="base_link_at_capture", junction_recognized=False,
        junction_status="unknown", temporal_identity="unbound_capture_metadata_required",
        observed_points=[], curves=[], fragments=[], fragment_gap_relations=[], side_boundary_candidates=[])


def _validate(p):
    if type(p) is not ObserverParameters:
        raise ValueError("ObserverParameters_required")
    for name, value in vars(p).items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid_positive_parameter:" + name)
        if (name.startswith("max_") and (name.endswith("pixels") or name.endswith("fragments"))
                or name in ("min_fragment_pixels", "min_main_support_pixels", "max_split_work")) and type(value) is not int:
            raise ValueError("integer_budget_or_pixel_count_required")
    if (p.min_fragment_pixels < 3 or p.min_inlier_fraction > 1
            or not p.max_main_heading_deg < p.min_side_heading_deg < 90
            or p.max_parallel_difference_deg >= 45 or p.min_side_separation_m >= p.max_side_separation_m):
        raise ValueError("inconsistent_thresholds")


def _angle(a, b, np):
    return math.degrees(math.acos(float(np.clip(abs(a @ b), 0, 1))))


def _runs(pixel_ids, available):
    """Partition the closed contour at missing projections; never bridge a gap."""
    if all(key in available for key in pixel_ids):
        return [(pixel_ids, True)]
    absent = next(i for i, key in enumerate(pixel_ids) if key not in available)
    ordered = pixel_ids[absent+1:] + pixel_ids[:absent+1]
    result, current = [], []
    for key in ordered:
        if key in available:
            current.append(key)
        elif current:
            result.append((current, False))
            current = []
    if current:
        result.append((current, False))
    return result


def _pieces(ids, by_id, p, np, work):
    """Split by measured chord residual; keep every original ordered support."""
    stack, result = [ids], []
    while stack:
        current = stack.pop()
        work[0] += len(current)
        if work[0] > p.max_split_work:
            raise ValueError("curve_split_work_budget_exceeded")
        if len(current) < 3:
            result.append(current)
            continue
        xy = np.asarray([by_id[key] for key in current])
        chord = xy[-1]-xy[0]
        length = float(np.linalg.norm(chord))
        residual = np.linalg.norm(xy-xy[0], axis=1) if length < 1e-9 else np.abs((xy-xy[0]) @ np.array([-chord[1], chord[0]]))/length
        index = 1+int(np.argmax(residual[1:-1]))
        if residual[index] > p.max_local_residual_m:
            stack.extend([current[index:], current[:index+1]])
        else:
            result.append(current)
        if len(result)+len(stack) > p.max_fragments:
            raise ValueError("fragment_budget_exceeded")
    return result


def _describe(ids, curve_id, component, index, flags, by_id, p, np):
    unique = list(dict.fromkeys(ids))
    xy = np.asarray([by_id[key] for key in unique])
    center = xy.mean(axis=0)
    _, vectors = np.linalg.eigh((xy-center).T @ (xy-center)/max(1, len(xy)))
    axis = vectors[:, -1]
    if axis[0] < 0:
        axis = -axis
    residual = np.abs((xy-center) @ np.array([-axis[1], axis[0]]))
    along = xy @ axis
    keep = residual <= p.max_local_residual_m
    span = float(along.max()-along.min())
    quality = (len(unique) >= p.min_fragment_pixels and span >= p.min_fragment_span_m
        and float(keep.mean()) >= p.min_inlier_fraction and float(np.quantile(residual, .9)) <= p.max_local_residual_m)
    return dict(fragment_id=index, curve_id=curve_id, component_id=component, ordered_pixel_ids=ids,
        pixel_ids=unique, inlier_pixel_ids=[key for key, valid in zip(unique, keep) if valid],
        support_endpoint_pixel_ids=[ids[0], ids[-1]], endpoint_kind="unknown_support_or_partition_extreme",
        truncation=list(flags), local_linear_support=bool(quality), observed_span_m=span,
        direction_xy=axis.tolist(), centroid_xy=center.tolist(), residual_p90_m=float(np.quantile(residual, .9)),
        inlier_fraction=float(keep.mean()), heading_to_forward_deg=math.degrees(math.acos(float(np.clip(abs(axis[0]), 0, 1)))),
        status="local_positive_pixel_support" if quality else "unknown_or_insufficient_local_support")


def _gap_relations(fragments, by_id, p, np):
    result = []
    for i, a in enumerate(fragments):
        for b in fragments[i+1:]:
            if a["curve_id"] == b["curve_id"] or _angle(np.array(a["direction_xy"]), np.array(b["direction_xy"]), np) > p.max_parallel_difference_deg:
                continue
            choices = [(float(np.linalg.norm(by_id[x]-by_id[y])), x, y)
                       for x in a["support_endpoint_pixel_ids"] for y in b["support_endpoint_pixel_ids"]]
            gap, x, y = min(choices)
            delta = by_id[y]-by_id[x]
            if not 0 < gap <= p.max_fragment_gap_m or _angle(delta/gap, np.array(a["direction_xy"]), np) > p.max_parallel_difference_deg:
                continue
            result.append(dict(fragment_ids=[a["fragment_id"], b["fragment_id"]], endpoint_pixel_ids=[x, y],
                gap_m=gap, status="unconfirmed_gap_relation", gap_is_observed=False, inserted_pixel_count=0))
    return result


def _side_pairs(useful, by_id, p, np):
    main = [f for f in useful if f["heading_to_forward_deg"] <= p.max_main_heading_deg]
    side = [f for f in useful if f["heading_to_forward_deg"] >= p.min_side_heading_deg]
    candidates = []
    for i, a in enumerate(side):
        for b in side[i+1:]:
            if set(a["inlier_pixel_ids"]) & set(b["inlier_pixel_ids"]):
                continue
            axis = np.array(a["direction_xy"])
            difference = _angle(axis, np.array(b["direction_xy"]), np)
            if difference > p.max_parallel_difference_deg:
                continue
            pa = np.asarray([by_id[k] for k in a["inlier_pixel_ids"]])
            pb = np.asarray([by_id[k] for k in b["inlier_pixel_ids"]])
            sign = 1 if min(pa[:, 1].min(), pb[:, 1].min()) > 0 else -1 if max(pa[:, 1].max(), pb[:, 1].max()) < 0 else 0
            if not sign:
                continue  # Bars crossing the whole driving corridor are not side edges.
            normal = np.array([-axis[1], axis[0]])
            qa, qb = pa @ axis, pb @ axis
            overlap = float(min(qa.max(), qb.max())-max(qa.min(), qb.min()))
            ca, cb = float(pa.mean(axis=0) @ normal), float(pb.mean(axis=0) @ normal)
            width = abs(cb-ca)
            if overlap < p.min_side_overlap_m or not p.min_side_separation_m <= width <= p.max_side_separation_m:
                continue
            repeated = False
            for other in side:
                if other is a or other is b or _angle(axis, np.array(other["direction_xy"]), np) > p.max_parallel_difference_deg:
                    continue
                po = np.asarray([by_id[k] for k in other["inlier_pixel_ids"]])
                qo, co = po @ axis, float(po.mean(axis=0) @ normal)
                independent = min(abs(co-ca), abs(co-cb)) > 2*p.max_local_residual_m
                if (independent and min(ca, cb)-p.min_side_separation_m <= co <= max(ca, cb)+p.min_side_separation_m
                        and min(qa.max(), qb.max(), qo.max())-max(qa.min(), qb.min(), qo.min()) >= p.min_side_overlap_m):
                    repeated = True
                    break
            if repeated:
                continue
            lo, hi = float(min(pa[:, 0].min(), pb[:, 0].min())), float(max(pa[:, 0].max(), pb[:, 0].max()))
            before, after = {1: set(), -1: set()}, {1: set(), -1: set()}
            for f in main:
                points = np.asarray([by_id[k] for k in f["inlier_pixel_ids"]])
                side_sign = 1 if points[:, 1].min() > 0 else -1 if points[:, 1].max() < 0 else 0
                if not side_sign:
                    continue
                for key, point in zip(f["inlier_pixel_ids"], points):
                    if point[0] <= lo-p.min_main_before_m:
                        before[side_sign].add(key)
                    if point[0] >= hi+p.min_main_continuation_m:
                        after[side_sign].add(key)
            if not all(len(group[sgn]) >= p.min_main_support_pixels for group in (before, after) for sgn in (-1, 1)):
                continue
            candidates.append(dict(status="unconfirmed_paired_side_fragment_evidence", junction_recognized=False,
                side="left" if sign > 0 else "right", boundary_fragment_ids=[a["fragment_id"], b["fragment_id"]],
                boundary_inlier_pixel_ids=[a["inlier_pixel_ids"], b["inlier_pixel_ids"]],
                main_before_pixel_ids={str(k): sorted(v) for k, v in before.items()},
                main_continuation_pixel_ids={str(k): sorted(v) for k, v in after.items()},
                observed_axis_overlap_m=overlap, observed_separation_m=width, parallel_difference_deg=difference,
                uncertainty="unknown; no_station_or_world_anchor; independent_temporal_validation_required"))
    return candidates


def observe_lane_geometry(mask, camera, *, enabled=False, parameters=None):
    if enabled is False:
        return _empty("disabled", "explicit_opt_in_required")
    if enabled is not True:
        return _empty("unknown", "enabled_must_be_bool")
    import numpy as np
    import cv2
    from pixel_projection import project_current_pixels
    p = ObserverParameters() if parameters is None else parameters
    try:
        _validate(p)
        evidence, binary, labels, flags = project_current_pixels(mask, camera, p, np, cv2)
        by_id = {item["pixel_id"]: np.array(item["ground_xyz"][:2]) for item in evidence["observed_points"]}
        contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if sum(len(c) for c in contours) > p.max_contour_pixels:
            raise ValueError("contour_pixel_budget_exceeded")
        curves, fragments, split_work = [], [], [0]
        width = binary.shape[1]
        for curve_id, contour in enumerate(contours):
            uv = contour[:, 0, :]
            ids = [int(v)*width+int(u) for u, v in uv]
            component = int(labels[int(uv[0, 1]), int(uv[0, 0])])
            runs = _runs(ids, by_id)
            curves.append(dict(curve_id=curve_id, component_id=component, original_contour_pixel_ids=ids,
                observed_runs=[dict(ordered_pixel_ids=run, closed=closed) for run, closed in runs],
                truncation=flags[component], status="ordered_positive_pixel_boundary_not_road_topology"))
            for run, closed in runs:
                arcs = [run]
                if closed and len(run) > 2:
                    xy = np.asarray([by_id[k] for k in run])
                    split = int(np.argmax(np.linalg.norm(xy-xy[0], axis=1)))
                    if 0 < split < len(run):
                        arcs = [run[:split+1], run[split:]+run[:1]]
                for arc in arcs:
                    for piece in _pieces(arc, by_id, p, np, split_work):
                        if not piece:
                            continue
                        fragments.append(_describe(piece, curve_id, component, len(fragments), flags[component], by_id, p, np))
                        if len(fragments) > p.max_fragments:
                            raise ValueError("fragment_budget_exceeded")
        useful = [f for f in fragments if f["local_linear_support"]]
        result = _empty("observed_geometry", "anonymous_curve_support_only")
        result.update(evidence, curves=curves, fragments=fragments, parameters=vars(p).copy())
        if len(useful) > p.max_candidate_fragments:
            result["reason"] = "candidate_budget_exceeded; raw_geometry_retained"
        else:
            result["fragment_gap_relations"] = _gap_relations(useful, by_id, p, np)
            result["side_boundary_candidates"] = _side_pairs(useful, by_id, p, np)
        return result
    except (ValueError, KeyError, TypeError, AttributeError, np.linalg.LinAlgError) as error:
        return _empty("unknown", type(error).__name__+":"+str(error))
