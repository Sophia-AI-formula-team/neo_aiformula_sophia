"""UV-ordered, quantization-aware curve hypotheses. Default off, no control."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObserverParameters:
    max_forward_m: float = 10.
    max_lateral_m: float = 4.
    max_positive_pixels: int = 50000
    max_contour_pixels: int = 150000
    max_curve_arms: int = 256
    max_pair_cells: int = 2000000
    max_footprint_pixels: int = 500000
    min_curve_pixels: int = 8
    tangent_half_window_px: float = 64.
    adaptive_tangent_max_half_window_px: float = 256.
    max_adaptive_tangent_support_checks: int = 2000000
    min_tangent_chord_arc_ratio: float = .85
    pixel_quantization_half_width: float = .5
    max_main_heading_deg: float = 20.
    min_side_heading_deg: float = 45.
    max_parallel_difference_deg: float = 10.
    min_side_separation_m: float = 1.5
    max_side_separation_m: float = 6.
    min_pair_support_span_m: float = 1.
    min_matched_sections: int = 3
    min_black_cells: int = 2
    min_main_support_pixels: int = 12
    min_main_before_m: float = .5
    min_main_continuation_m: float = 1.


def _empty(status, reason):
    return dict(schema="anonymous_quantized_curve_geometry/v1", enabled=status != "disabled",
        status=status, reason=reason, junction_recognized=False, junction_status="unknown",
        confirmation_eligible=False, confirmation_reason="unconfirmed_semantics_and_unknown_total_model_error",
        capture_metadata_admitted=False, frame="existing_local_plane_at_capture_assumption",
        observed_points=[], curves=[], pair_hypotheses=[], paint_support_groups=[])


def _validate(p):
    if type(p) is not ObserverParameters:
        raise ValueError("ObserverParameters_required")
    integer_names = {"max_positive_pixels", "max_contour_pixels", "max_curve_arms", "max_pair_cells",
        "max_footprint_pixels", "min_curve_pixels", "min_matched_sections", "min_black_cells", "min_main_support_pixels",
        "max_adaptive_tangent_support_checks"}
    for name, value in vars(p).items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid_positive_parameter:"+name)
        if name in integer_names and type(value) is not int:
            raise ValueError("integer_count_required:"+name)
    if (p.min_curve_pixels < 3 or p.min_matched_sections < 3 or p.min_black_cells < 2
            or p.min_tangent_chord_arc_ratio > 1
            or p.tangent_half_window_px != 64
            or p.adaptive_tangent_max_half_window_px not in (64, 128, 256)
            or not p.max_main_heading_deg < p.min_side_heading_deg < 90
            or not 0 < p.max_parallel_difference_deg < 45
            or p.min_side_separation_m >= p.max_side_separation_m):
        raise ValueError("inconsistent_parameters")


def _angle(a, b, np):
    return math.degrees(math.acos(float(np.clip(abs(a @ b), 0., 1.))))


def _runs(ids, available):
    if all(key in available for key in ids):
        return [(ids, True)]
    split = next(i for i, key in enumerate(ids) if key not in available)
    result, run = [], []
    for key in ids[split+1:]+ids[:split+1]:
        if key in available:
            run.append(key)
        elif run:
            result.append((run, False))
            run = []
    if run:
        result.append((run, False))
    return result


def _straight_observed_window(uv, half_pixel, np):
    """Estimated endpoint line must intersect every observed pixel's square.

    No fitted/invented support, no skipping a bend, and no missing UV step.
    """
    steps = np.abs(np.diff(uv, axis=0))
    contiguous = bool(len(steps) and np.all(np.max(steps, axis=1) == 1))
    chord = uv[-1]-uv[0]
    length = float(np.linalg.norm(chord))
    if length <= 0:
        return dict(contiguous_observed_pixels=contiguous, strip_pass=False,
            strip_max_residual_px=None, strip_quantization_bound_px=None)
    normal = np.array([-chord[1], chord[0]])/length
    residual = float(np.max(np.abs((uv-uv[0]) @ normal)))
    bound = float(half_pixel*np.sum(np.abs(normal)))
    return dict(contiguous_observed_pixels=contiguous, strip_pass=residual <= bound,
        strip_max_residual_px=residual, strip_quantization_bound_px=bound)


def _describe_arm(ids, arm_id, component, flags, observations, p, np, cv2, adaptive_work=None):
    adaptive_work = [0] if adaptive_work is None else adaptive_work
    uv = np.asarray([observations[k]["uv"] for k in ids], float)
    xy = np.asarray([observations[k]["ground_xyz"][:2] for k in ids], float)
    cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(uv, axis=0), axis=1))]
    estimated = cv2.approxPolyDP(uv.astype(np.float32).reshape(-1, 1, 2),
        math.sqrt(2)*p.pixel_quantization_half_width, False)[:, 0, :].tolist()
    samples = []
    for i, key in enumerate(ids):
        lo = max(0, int(np.searchsorted(cumulative, cumulative[i]-p.tangent_half_window_px, side="right"))-1)
        hi = min(len(ids)-1, int(np.searchsorted(cumulative, cumulative[i]+p.tangent_half_window_px, side="left")))
        chord, uv_chord = xy[hi]-xy[lo], float(np.linalg.norm(uv[hi]-uv[lo]))
        arc = float(cumulative[hi]-cumulative[lo])
        length = float(np.linalg.norm(chord))
        qlo = observations[ids[lo]]["pixel_quantization_bound_xy_m"]
        qhi = observations[ids[hi]]["pixel_quantization_bound_xy_m"]
        sample = dict(pixel_id=key, estimated_tangent_xy=None, tangent_quantization_bound_deg=None,
            tangent_support_endpoint_ids=[ids[lo], ids[hi]], tangent_uv_arc_px=arc,
            tangent_chord_arc_ratio=uv_chord/arc if arc else 0., role="unresolved",
            tangent_is_estimated=True, total_model_error="unknown")
        if arc > 0 and length > 0 and qlo is not None and qhi is not None:
            error = float(np.linalg.norm(np.asarray(qlo)+np.asarray(qhi)))
            if error < length and uv_chord/arc >= p.min_tangent_chord_arc_ratio:
                direction = chord/length
                angle_bound = math.degrees(math.asin(error/length))
                heading = math.degrees(math.acos(float(np.clip(abs(direction[0]), 0, 1))))
                sample.update(estimated_tangent_xy=direction.tolist(), tangent_quantization_bound_deg=angle_bound,
                    estimated_heading_to_forward_deg=heading)
                if heading+angle_bound <= p.max_main_heading_deg:
                    sample["role"] = "main_direction_support_under_assumptions"
                elif heading-angle_bound >= p.min_side_heading_deg:
                    sample["role"] = "side_direction_support_under_assumptions"
        # Preserve the exact old descriptor. Additional support may reduce only
        # quantization error; all original pair/width/span gates remain unchanged.
        sample["baseline_tangent"] = dict(sample)
        sample.update(baseline_half_window_px=64., used_half_window_px=64.,
            tangent_support_order_index_range=[lo, hi],
            tangent_support_identity="inclusive slice of parent ordered_pixel_ids",
            adaptive_tangent_used=False, adaptive_attempts=[],
            adaptive_target_bound_deg=p.max_parallel_difference_deg/2)
        baseline_bound = sample["tangent_quantization_bound_deg"]
        if baseline_bound is None or baseline_bound > p.max_parallel_difference_deg/2:
            previous_endpoints = (lo, hi)
            for window in (128., 256.):
                if window > p.adaptive_tangent_max_half_window_px:
                    break
                a = max(0, int(np.searchsorted(cumulative, cumulative[i]-window, side="right"))-1)
                b = min(len(ids)-1, int(np.searchsorted(cumulative, cumulative[i]+window, side="left")))
                attempt = dict(half_window_px=window, observed_support_count=b-a+1,
                    support_order_index_range=[a, b], endpoint_pixel_ids=[ids[a], ids[b]],
                    accepted=False, reason="unresolved")
                sample["adaptive_attempts"].append(attempt)
                if (a, b) == previous_endpoints:
                    attempt["reason"] = "no_new_observed_endpoints"
                    break
                previous_endpoints = (a, b)
                adaptive_work[0] += b-a+1
                if adaptive_work[0] > p.max_adaptive_tangent_support_checks:
                    raise ValueError("adaptive_tangent_support_budget_exceeded")
                strip = _straight_observed_window(uv[a:b+1], p.pixel_quantization_half_width, np)
                attempt.update(strip)
                uv_arc = float(cumulative[b]-cumulative[a])
                uv_length = float(np.linalg.norm(uv[b]-uv[a]))
                ratio = uv_length/uv_arc if uv_arc else 0.
                attempt.update(uv_arc_px=uv_arc, chord_arc_ratio=ratio)
                if not strip["contiguous_observed_pixels"]:
                    attempt["reason"] = "missing_or_repeated_uv_step"
                    break
                if not strip["strip_pass"] or ratio < p.min_tangent_chord_arc_ratio:
                    attempt["reason"] = "bend_or_turnaround_not_quantized_straight"
                    break
                ground_chord = xy[b]-xy[a]
                ground_length = float(np.linalg.norm(ground_chord))
                qa = observations[ids[a]]["pixel_quantization_bound_xy_m"]
                qb = observations[ids[b]]["pixel_quantization_bound_xy_m"]
                if qa is None or qb is None or ground_length <= 0:
                    attempt["reason"] = "unbounded_or_zero_ground_chord"
                    continue
                error = float(np.linalg.norm(np.asarray(qa)+np.asarray(qb)))
                attempt.update(ground_chord_length_m=ground_length, endpoint_error_norm_m=error)
                if error >= ground_length:
                    attempt["reason"] = "quantization_exceeds_ground_chord"
                    continue
                bound = math.degrees(math.asin(error/ground_length))
                attempt["tangent_quantization_bound_deg"] = bound
                if bound > p.max_parallel_difference_deg/2:
                    attempt["reason"] = "direction_bound_target_not_met"
                    continue
                direction = ground_chord/ground_length
                heading = math.degrees(math.acos(float(np.clip(abs(direction[0]), 0, 1))))
                role = "unresolved"
                if heading+bound <= p.max_main_heading_deg:
                    role = "main_direction_support_under_assumptions"
                elif heading-bound >= p.min_side_heading_deg:
                    role = "side_direction_support_under_assumptions"
                sample.update(estimated_tangent_xy=direction.tolist(), tangent_quantization_bound_deg=bound,
                    tangent_support_endpoint_ids=[ids[a], ids[b]], tangent_uv_arc_px=uv_arc,
                    tangent_chord_arc_ratio=ratio, estimated_heading_to_forward_deg=heading, role=role,
                    used_half_window_px=window, tangent_support_order_index_range=[a, b],
                    adaptive_tangent_used=True, straight_strip_evidence=strip)
                attempt.update(accepted=True, reason="more_observed_same_line_support_meets_bound")
                break
        samples.append(sample)
    return dict(arm_id=arm_id, component_id=component, ordered_pixel_ids=ids, samples=samples,
        estimated_uv_polyline=estimated, polyline_kind="estimated_chords_between_observed_vertices",
        uv_approximation_bound_px=math.sqrt(2)*p.pixel_quantization_half_width,
        approximation_bound_is_not_segmentation_accuracy=True,
        truncation=list(flags), endpoint_kind="support_extreme_not_physical_termination",
        raw_points_replaced=False, gap_filled=False)


def _raster_line(a, b):
    """Integer Bresenham footprint between two actual positive pixel centers."""
    x, y = a
    end_x, end_y = b
    dx, dy = abs(end_x-x), -abs(end_y-y)
    sx, sy = 1 if x < end_x else -1, 1 if y < end_y else -1
    error = dx+dy
    result = []
    while True:
        result.append((x, y))
        if x == end_x and y == end_y:
            return result
        doubled = 2*error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


def inspect_footprint(mask, uv_a, uv_b, min_black_cells=2):
    """Paint-support evidence only; black pixels never certify drivable space."""
    pixels = _raster_line(tuple(uv_a), tuple(uv_b))
    if any(x < 0 or y < 0 or y >= mask.shape[0] or x >= mask.shape[1] for x, y in pixels):
        raise ValueError("footprint_outside_actual_image")
    white = [bool(mask[y, x] > 0) for x, y in pixels]
    runs = []
    for value in white:
        if not runs or runs[-1]["white"] != value:
            runs.append(dict(white=value, count=1))
        else:
            runs[-1]["count"] += 1
    status = "unresolved_footprint"
    if white[0] and white[-1]:
        if all(white):
            status = "same_positive_paint_footprint"
        elif len(runs) == 3 and not runs[1]["white"] and runs[1]["count"] >= min_black_cells:
            status = "separated_positive_paint_footprints"
        elif len(runs) > 3:
            status = "additional_positive_bands_ambiguous"
    return dict(status=status, endpoint_uv=[list(uv_a), list(uv_b)], sampled_pixels=len(pixels),
        white_pixels=sum(white), black_pixels=len(white)-sum(white), runs=runs,
        black_pixel_means="no_positive_lane_mask_evidence_not_drivable_space")


def _selected(arm, role, observations, np):
    selected, seen = [], set()
    for sample in arm["samples"]:
        key = sample["pixel_id"]
        if sample["role"] != role or key in seen:
            continue
        item = observations[key]
        q = item["pixel_quantization_bound_xy_m"]
        if q is None:
            continue
        y = item["ground_xyz"][1]
        sign = 1 if y-q[1] > 0 else -1 if y+q[1] < 0 else 0
        if not sign:
            continue
        selected.append(dict(sample, xy=np.asarray(item["ground_xyz"][:2]), bound=np.asarray(q), side=sign, uv=item["uv"]))
        seen.add(key)
    return selected


def _span(samples, np):
    if len(samples) < 2:
        return 0.
    return max(0., float(np.linalg.norm(samples[-1]["xy"]-samples[0]["xy"])
        - np.linalg.norm(samples[0]["bound"]+samples[-1]["bound"])))


def _normal_separation_interval(delta, normal, position_bound, angle_bound_deg):
    """Quantization-only bound for every normal in the angular cone.

    ``position_bound`` is the summed endpoint axis-aligned error box. Its
    Euclidean norm bounds its projection onto every possible unit normal.
    """
    dx, dy = (float(value) for value in delta)
    nx, ny = (float(value) for value in normal)
    qx, qy = (float(value) for value in position_bound)
    values = (dx, dy, nx, ny, qx, qy, angle_bound_deg)
    if (not all(math.isfinite(value) for value in values) or min(qx, qy, angle_bound_deg) < 0
            or not math.isclose(math.hypot(nx, ny), 1., rel_tol=0., abs_tol=1e-8)):
        raise ValueError("invalid_normal_separation_inputs")
    cone = min(90., float(angle_bound_deg))
    radius = math.hypot(dx, dy)
    theta = math.atan2(nx*dy-ny*dx, nx*dx+ny*dy)
    low, high = theta-math.radians(cone), theta+math.radians(cone)
    extrema = [radius*abs(math.cos(low)), radius*abs(math.cos(high))]
    # Absolute cosine has maxima/zeros every pi/2. Include internal extrema.
    step = math.pi/2
    for index in range(math.ceil(low/step), math.floor(high/step)+1):
        extrema.append(0. if index % 2 else radius)
    angular = [min(extrema), max(extrema)]
    expansion = math.hypot(qx, qy)
    return dict(interval_m=[max(0., angular[0]-expansion), angular[1]+expansion],
        angular_only_interval_m=angular, position_bound_l2_m=expansion,
        normal_cone_deg=cone)


def _pair(a, b, selected, observations, binary, p, work, np):
    aa, bb = selected[a["arm_id"]], selected[b["arm_id"]]
    work[0] += len(aa)*len(bb)
    if work[0] > p.max_pair_cells:
        raise ValueError("pair_work_budget_exceeded")
    pa, pb = np.asarray([s["xy"] for s in aa]), np.asarray([s["xy"] for s in bb])
    distance = np.sum((pa[:, None, :]-pb[None, :, :])**2, axis=2)
    ab, ba = np.argmin(distance, axis=1), np.argmin(distance, axis=0)
    indices = [(i, int(j)) for i, j in enumerate(ab) if int(ba[j]) == i]
    b_order = [j for _, j in indices]
    monotonic = len(indices) >= p.min_matched_sections and (
        all(y > x for x, y in zip(b_order, b_order[1:])) or all(y < x for x, y in zip(b_order, b_order[1:])))
    records, width_pass, tangent_pass, normal_pass, side_pass = [], True, True, True, True
    width_compatible, tangent_compatible = True, True
    for i, j in indices:
        sa, sb = aa[i], bb[j]
        da, db = np.asarray(sa["estimated_tangent_xy"]), np.asarray(sb["estimated_tangent_xy"])
        if da @ db < 0:
            db = -db
        axis = da+db
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            continue
        axis /= norm
        normal = np.array([-axis[1], axis[0]])
        delta = sb["xy"]-sa["xy"]
        width = abs(float(delta @ normal))
        angle_bound = sa["tangent_quantization_bound_deg"]+sb["tangent_quantization_bound_deg"]
        separation = _normal_separation_interval(delta, normal, sa["bound"]+sb["bound"], angle_bound)
        width_interval = separation["interval_m"]
        angle = _angle(da, db, np)
        along = abs(float(delta @ axis))
        along_bound = float((sa["bound"]+sb["bound"]) @ np.abs(axis)) + float(np.linalg.norm(delta))*math.sin(math.radians(min(90., angle_bound)))
        footprint = inspect_footprint(binary, sa["uv"], sb["uv"], p.min_black_cells)
        work[1] += footprint["sampled_pixels"]
        if work[1] > p.max_footprint_pixels:
            raise ValueError("footprint_work_budget_exceeded")
        record = dict(pixel_ids=[sa["pixel_id"], sb["pixel_id"]], width_interval_m=width_interval,
            estimated_normal_separation_m=width,
            normal_direction_quantization_bound_deg=separation["normal_cone_deg"],
            width_angular_only_interval_m=separation["angular_only_interval_m"],
            width_position_expansion_m=separation["position_bound_l2_m"],
            estimated_tangent_difference_deg=angle, tangent_quantization_bound_deg=angle_bound,
            normal_correspondence_residual_m=along, quantization_correspondence_bound_m=along_bound,
            footprint=footprint)
        records.append(record)
        width_pass &= width_interval[0] >= p.min_side_separation_m and width_interval[1] <= p.max_side_separation_m
        tangent_pass &= angle+angle_bound <= p.max_parallel_difference_deg
        # Interval overlap can preserve a competing explanation, never accept a pair.
        width_compatible &= width_interval[1] >= p.min_side_separation_m and width_interval[0] <= p.max_side_separation_m
        tangent_compatible &= max(0., angle-angle_bound) <= p.max_parallel_difference_deg
        normal_pass &= along <= along_bound
        side_pass &= sa["side"] == sb["side"] == aa[0]["side"]
    matched_a, matched_b = [aa[i] for i, _ in indices], [bb[j] for _, j in indices]
    spans = [_span(matched_a, np), _span(matched_b, np)]
    footprint_states = [r["footprint"]["status"] for r in records]
    enough = len(records) >= p.min_matched_sections
    span_pass = min(spans) >= p.min_pair_support_span_m
    same_paint = enough and span_pass and all(s == "same_positive_paint_footprint" for s in footprint_states)
    separated = enough and all(s == "separated_positive_paint_footprints" for s in footprint_states)
    checks = dict(monotonic_real_correspondence=monotonic, actual_support_span=span_pass,
        same_lateral_side=bool(side_pass), quantized_parallelism=bool(tangent_pass),
        normal_correspondence=bool(normal_pass), complete_width_interval=bool(width_pass),
        actual_separated_paint_footprints=separated)
    matched_ids = list(dict.fromkeys(k for r in records for k in r["pixel_ids"]))
    xlo = min((observations[k]["ground_xyz"][0]-observations[k]["pixel_quantization_bound_xy_m"][0] for k in matched_ids), default=0.)
    xhi = max((observations[k]["ground_xyz"][0]+observations[k]["pixel_quantization_bound_xy_m"][0] for k in matched_ids), default=0.)
    return dict(arm_ids=[a["arm_id"], b["arm_id"]], component_ids=[a["component_id"], b["component_id"]],
        matched_sections=records, conservative_observed_support_spans_m=spans,
        original_component_ids_used_for_independence=False, same_paint_footprint_explanation=same_paint,
        checks=checks, observed_x_interval_m=[xlo, xhi], side=aa[0]["side"],
        competing_interpretation_compatible=bool(monotonic and span_pass and side_pass
            and normal_pass and separated and width_compatible and tangent_compatible),
        status="unknown", geometry_supported_under_assumptions=False, junction_recognized=False)


def _finish_pairs(pairs, arms, observations, p, np):
    parents = {a["arm_id"]: a["arm_id"] for a in arms}
    def find(key):
        while parents[key] != key:
            key = parents[key]
        return key
    for pair in pairs:
        if pair["same_paint_footprint_explanation"]:
            a, b = (find(k) for k in pair["arm_ids"])
            parents[b] = a
    main = {}
    for arm in arms:
        for sample in _selected(arm, "main_direction_support_under_assumptions", observations, np):
            main[sample["pixel_id"]] = sample
    for pair_id, pair in enumerate(pairs):
        pair["pair_id"] = pair_id
        lo, hi = pair["observed_x_interval_m"]
        before, after = {1: [], -1: []}, {1: [], -1: []}
        for key, sample in main.items():
            if sample["xy"][0]+sample["bound"][0] <= lo-p.min_main_before_m:
                before[sample["side"]].append(key)
            if sample["xy"][0]-sample["bound"][0] >= hi+p.min_main_continuation_m:
                after[sample["side"]].append(key)
        pair["main_before_pixel_ids"] = {str(k): sorted(v) for k, v in before.items()}
        pair["main_after_pixel_ids"] = {str(k): sorted(v) for k, v in after.items()}
        pair["paint_group_ids"] = [find(k) for k in pair["arm_ids"]]
        pair["checks"]["not_same_paint_support_group"] = pair["paint_group_ids"][0] != pair["paint_group_ids"][1]
        pair["checks"]["actual_main_before_after"] = all(len(group[side]) >= p.min_main_support_pixels for group in (before, after) for side in (-1, 1))
    alternatives = [pair for pair in pairs if pair["competing_interpretation_compatible"]
                    and pair["checks"]["not_same_paint_support_group"]]
    for pair in pairs:
        groups = set(pair["paint_group_ids"])
        competing = [other["arm_ids"] for other in alternatives if other is not pair
            and groups != set(other["paint_group_ids"]) and groups & set(other["paint_group_ids"])
            and pair["side"] == other["side"]]
        pair["competing_pair_arm_ids"] = competing
        pair["checks"]["no_competing_boundary_pair"] = not competing
        pair["failed_checks"] = [key for key, value in pair["checks"].items() if not value]
        pair["geometry_supported_under_assumptions"] = not pair["failed_checks"]
        pair["status"] = "paired_curve_support_under_local_plane_assumption" if not pair["failed_checks"] else "unknown"
        pair["total_model_error"] = "unknown; geometry support is not junction confirmation"
    equivalent = {}
    for pair in pairs:
        key = tuple(sorted(pair["paint_group_ids"]))
        equivalent.setdefault(key, []).append(pair)
    for key, members in equivalent.items():
        representative = next((pair for pair in members if not pair["failed_checks"]), members[0])
        for pair in members:
            duplicate = pair is not representative
            pair["pair_equivalence_id"] = "paint_groups:"+":".join(str(value) for value in key)
            pair["duplicate_of_pair_id"] = representative["pair_id"] if duplicate else None
            pair["checks"]["unique_pair_representative"] = not duplicate
            pair["failed_checks"] = [name for name, passed in pair["checks"].items() if not passed]
            pair["geometry_supported_under_assumptions"] = not pair["failed_checks"]
            pair["independent_support"] = not pair["failed_checks"]
            if duplicate:
                pair["status"] = "duplicate_paint_pair_explanation"
    groups = {}
    for key in parents:
        groups.setdefault(find(key), []).append(key)
    return [dict(group_id=key, arm_ids=value, meaning="same_positive_footprint_explanation_not_official_edge_id") for key, value in groups.items()]


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
        observations = {item["pixel_id"]: item for item in evidence["observed_points"]}
        contours, _ = cv2.findContours(binary.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if sum(len(c) for c in contours) > p.max_contour_pixels:
            raise ValueError("contour_pixel_budget_exceeded")
        arms, adaptive_work = [], [0]
        for contour in contours:
            uv = contour[:, 0, :]
            ids = [int(v)*binary.shape[1]+int(u) for u, v in uv]
            component = int(labels[int(uv[0, 1]), int(uv[0, 0])])
            for run, closed in _runs(ids, observations):
                sections = [run]
                if closed and len(run) > 2:
                    xy = np.asarray([observations[k]["uv"] for k in run])
                    split = int(np.argmax(np.linalg.norm(xy-xy[0], axis=1)))
                    if 0 < split < len(run):
                        sections = [run[:split+1], run[split:]+run[:1]]
                for section in sections:
                    if len(set(section)) < p.min_curve_pixels:
                        continue  # All positive pixels remain in observed_points.
                    arms.append(_describe_arm(section, len(arms), component, flags[component], observations, p, np, cv2, adaptive_work))
                    if len(arms) > p.max_curve_arms:
                        raise ValueError("curve_arm_budget_exceeded")
        selected = {a["arm_id"]: _selected(a, "side_direction_support_under_assumptions", observations, np) for a in arms}
        side = [a for a in arms if len(selected[a["arm_id"]]) >= p.min_matched_sections]
        pairs, work = [], [0, 0]
        for i, a in enumerate(side):
            for b in side[i+1:]:
                pairs.append(_pair(a, b, selected, observations, binary, p, work, np))
        groups = _finish_pairs(pairs, arms, observations, p, np)
        result = _empty("observed_geometry", "quantized_curve_hypotheses_only")
        result.update(evidence, curves=arms, pair_hypotheses=pairs, paint_support_groups=groups,
            parameters=vars(p).copy(), work=dict(pair_distance_cells=work[0], footprint_pixels=work[1],
                adaptive_tangent_support_checks=adaptive_work[0]))
        return result
    except (ValueError, KeyError, TypeError, AttributeError, np.linalg.LinAlgError) as error:
        return _empty("unknown", type(error).__name__+":"+str(error))
