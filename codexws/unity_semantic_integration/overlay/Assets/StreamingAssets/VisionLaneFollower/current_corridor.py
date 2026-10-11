"""Opt-in current-mask recovery and identity-checked native LEFT targets.

Original pixels only. Historical state is a scalar lane-width calibration;
no historical cloud, target, pose, route or signal truth is published. Paired
LEFT identity is a short-lived scalar admission reference, never a target.
"""
import json
import math
import cv2
import numpy as np
import lane_publisher as lp


def indices(cloud, long_only=False):
    p = np.asarray(cloud, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all():
        return None
    if not np.all(p[:, 0] > 0) or not np.all(np.diff(p[:, 0]) > 0):
        return None
    n = len(p)
    ix = (2, 4, 6) if n >= 7 else (1, 3, 5) if n == 6 else (0, 2, 4) if n == 5 else None
    if ix is None and not long_only:
        ix = (0, 1, 2) if n == 3 else (0, 1, 3)
    span = 2.0-1e-6 if n >= 5 else 1.0
    return ix if ix and p[ix[-1], 0]-p[ix[0], 0] >= span else None


def paired(clouds):
    left, right, center = (np.asarray(c, dtype=float) for c in clouds)
    ix = indices(center)
    if not ix or not indices(left) or not indices(right) or min(len(left), len(right)) <= ix[-1]:
        return None
    x = center[:, 0]
    if any(x[0] < p[0, 0] or x[-1] > p[-1, 0] for p in (left, right)):
        return None
    ly, ry = (np.interp(x, p[:, 0], p[:, 1]) for p in (left, right))
    if np.max(np.abs(center[:, 1]-.5*(ly+ry))) > 1e-4:
        return None
    width = ly-ry
    if not np.isfinite(width).all() or not np.all(width > 0):
        return None
    mean = float(width.mean())
    dx = x-x.mean()
    slopes = [float(dx @ (y-y.mean()) / (dx @ dx)) for y in (ly, ry)]
    angle = abs(math.degrees(math.atan(slopes[0])-math.atan(slopes[1])))
    spread = float(np.ptp(width)/mean)
    return dict(width_m=mean, width_min_m=float(width.min()), width_max_m=float(width.max()),
                variation_ratio=spread, heading_difference_deg=angle,
                trusted=spread <= .05 and angle <= 5.0)


def center_cloud(left, right):
    center = left[(left[:, 0] >= right[0, 0]) & (left[:, 0] <= right[-1, 0])].copy()
    if len(center):
        center[:, 1] = .5*(center[:, 1]+np.interp(center[:, 0], right[:, 0], right[:, 1]))
    return center


def line_identity(cloud):
    """Fit current observed points; retain only scalar geometry and domain."""
    if not indices(cloud):
        return None
    p = np.asarray(cloud, dtype=float)
    x, y = p[:, 0], p[:, 1]
    dx = x-x.mean()
    slope = float(dx @ (y-y.mean()) / (dx @ dx))
    return dict(slope=slope, intercept=float(y.mean()-slope*x.mean()),
                x_min=float(x[0]), x_max=float(x[-1]))


def match_left_identity(cloud, reference):
    current = line_identity(cloud)
    if current is None or reference is None:
        return dict(accepted=False, reason='no_current_left_or_identity')
    low = max(current['x_min'], reference['x_min'])
    high = min(current['x_max'], reference['x_max'])
    slope = current['slope']-reference['slope']
    intercept = current['intercept']-reference['intercept']
    distance = max(abs(slope*low+intercept), abs(slope*high+intercept))
    angle = abs(math.degrees(math.atan(current['slope'])-math.atan(reference['slope'])))
    accepted = high-low >= 1.0 and distance <= .25 and angle <= 10.0
    return dict(accepted=accepted, reason='current_left_identity_match' if accepted else 'left_identity_mismatch',
                overlap_x=[low, high], overlap_span_m=high-low,
                max_position_difference_m=distance, heading_difference_deg=angle,
                current_line=current)


class GroundProbe(lp.LanePublisher):
    """New area policy; original path/coverage/geometry/projection gates."""
    def evaluate(self, p, side, ref):
        result = dict(status='ok', component_pixels=int(len(p)), bottom=int(p[:, 1].max()) if len(p) else -1,
                      mode='scaled_current_component', pixels=p, line=None)
        if len(p) < max(20, lp.rounded(180*self.h*self.w/(1920*1080))):
            result['status'] = 'too_few_component_pixels'
        elif len(np.unique(p[:, 1]))/max(1, self.h-self.top) < .1:
            result['status'] = 'insufficient_row_coverage'
        elif p[:, 1].max()-p[:, 1].min()+1 < max(1, lp.rounded(.05*self.h)):
            result['status'] = 'insufficient_component_vertical_span'
        else:
            path = self.path(p, side, ref)
            if len(path) < 12:
                result['status'] = 'insufficient_continuous_lane_path'
            else:
                result['pixels'], result['line'] = path, lp.fit(path)
                if not lp.geometry(result['line'], side):
                    result['status'] = 'rejected_side_geometry'
        return result

    def fragments(self, mask, side, camera):
        scale = self.h*self.w/(1920*1080)
        parts = [p for p in lp.components(cv2.bitwise_and(mask, self.roi(side, None)), max(8, lp.rounded(50*scale)))
                 if len(np.unique(p[:, 1])) >= 3]
        parts = sorted(parts, key=lambda p: -len(p))[:20]
        if len(parts) < 2:
            return []
        centers = np.asarray([p.mean(axis=0) for p in parts])
        tolerance = max(3., .006*self.w)
        seeds = [lp.fit(p) for p in parts]
        for i in range(len(parts)):
            for j in range(i):
                du, dv = centers[i]-centers[j]
                if abs(dv) >= max(6, .02*self.h):
                    a = du/dv
                    seeds.append(np.asarray([a, centers[i, 0]-a*centers[i, 1]]))
        seen, accepted = set(), []
        for line in seeds:
            if not lp.geometry(line, side):
                continue
            close = np.flatnonzero(np.abs(centers[:, 0]-centers[:, 1]*line[0]-line[1]) <= tolerance)
            members = tuple(int(i) for i in close if np.quantile(np.abs(parts[i][:, 0]-parts[i][:, 1]*line[0]-line[1]), .9) <= 2*tolerance)
            if len(members) < 2 or members in seen:
                continue
            seen.add(members)
            p = np.concatenate([parts[i] for i in members])
            if len(p) < max(50, lp.rounded(180*scale)):
                continue
            if len(np.unique(p[:, 1]))/max(1, self.h-self.top) < .1:
                continue
            line = lp.fit(p)
            if not lp.geometry(line, side) or np.quantile(np.abs(p[:, 0]-p[:, 1]*line[0]-line[1]), .9) > 2*tolerance:
                continue
            path = self.path(p, side, line)
            if len(path) < 12:
                continue
            line = lp.fit(path)
            if not lp.geometry(line, side):
                continue
            cloud = lp.project(path, camera)
            if not indices(cloud):
                continue
            accepted.append(dict(status='ok', component_pixels=int(len(p)), bottom=int(p[:, 1].max()),
                mode='current_fragment_group', pixels=path, line=line, cloud=cloud, fragment_count=len(members)))
        return accepted


class CurrentCorridor:
    MAX_CALIBRATION_AGE = 1.5

    def __init__(self):
        self.identity = None
        self.sequence = -1
        self.stamp = None
        self.width_samples = []
        self.domain = None

    def begin(self, request, shape):
        sequence, stamp = int(request['sequence']), float(request['sim_time'])
        if sequence <= 0 or not math.isfinite(stamp) or stamp < 0:
            self.width_samples = []
            return False
        try:
            k = np.asarray(request['camera']['K'], float).reshape(3, 3)
            r = np.asarray(request['camera']['R'], float).reshape(3, 3)
            t = np.asarray(request['camera']['t'], float).reshape(3)
            if (not np.isfinite(k).all() or not np.isfinite(r).all() or not np.isfinite(t).all()
                    or k[0, 0] <= 0 or k[1, 1] <= 0 or abs(np.linalg.det(k)) < 1e-12 or t[2] <= 0):
                raise ValueError('Invalid calibration')
        except (KeyError, TypeError, ValueError):
            self.width_samples, self.domain = [], None
            return False
        identity = json.dumps([request['epoch'], list(shape), request['camera']], sort_keys=True)
        if identity != self.identity or (self.stamp is not None and (sequence < self.sequence or stamp < self.stamp)):
            self.identity, self.sequence, self.stamp = identity, -1, None
            self.width_samples, self.domain = [], None
        if sequence <= self.sequence or (self.stamp is not None and stamp <= self.stamp):
            return False
        self.sequence, self.stamp = sequence, stamp
        return True

    def calibrate(self, clouds, quality):
        # v2 intentionally admits trusted native 3/1m paired support. This
        # changes opt-in calibration only, never default/native thresholds.
        if quality and quality['trusted'] and indices(clouds[2]):
            if self.width_samples and self.width_samples[-1]['sequence'] >= self.sequence:
                return
            prior = self.calibration()
            if prior and abs(quality['width_m']-prior['width_m']) > .05*prior['width_m']:
                return
            recent = [s for s in self.width_samples if self.stamp-s['stamp'] <= self.MAX_CALIBRATION_AGE]
            trial_widths = [s['width_m'] for s in recent[-2:]]+[quality['width_m']]
            if np.ptp(trial_widths) > .05*np.mean(trial_widths):
                return
            self.width_samples = recent
            self.width_samples.append(dict(sequence=self.sequence, stamp=self.stamp,
                left_identity=line_identity(clouds[0]),
                paired_x_range=[float(clouds[2][0][0]), float(clouds[2][-1][0])], **quality))
            self.width_samples = self.width_samples[-3:]

    def calibration(self):
        if len(self.width_samples) < 3:
            return None
        recent = self.width_samples
        # Calibration time is window completion (latest observation). Formation
        # requires a compact window; runtime validity is <=1.5s since completion.
        # The consumer separately includes current-frame and transport age.
        if (self.stamp-recent[-1]['stamp'] > self.MAX_CALIBRATION_AGE
                or recent[-1]['stamp']-recent[0]['stamp'] > self.MAX_CALIBRATION_AGE):
            return None
        widths = [r['width_m'] for r in recent]
        return dict(width_m=float(np.median(widths)), dispersion_m=float(np.ptp(widths)),
            calibration_first_sequence=recent[0]['sequence'], calibration_last_sequence=recent[-1]['sequence'],
            calibration_first_stamp=recent[0]['stamp'], calibration_last_stamp=recent[-1]['stamp'],
            calibration_age_at_capture=self.stamp-recent[-1]['stamp'],
            calibration_window_age_at_capture=self.stamp-recent[0]['stamp'],
            calibration_max_age_sec=self.MAX_CALIBRATION_AGE,
            calibration_sequences=[r['sequence'] for r in recent])

    def left_identity(self):
        calibration = self.calibration()
        if calibration is None:
            return None
        latest = self.width_samples[-1]
        return dict(line=latest['left_identity'], paired_x_range=latest['paired_x_range'], **calibration)

    def ground_mask(self, mask, camera):
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        if self.domain is None:
            v, u = np.indices(mask.shape)
            p = np.column_stack((u.ravel(), v.ravel(), np.ones(u.size))).astype(np.float32)
            k = np.asarray(camera['K'], np.float32).reshape(3, 3)
            r = np.asarray(camera['R'], float).reshape(3, 3)
            t = np.asarray(camera['t'], float)
            if not np.isfinite(k).all() or not np.isfinite(r).all() or not np.isfinite(t).all() or t[2] <= 0:
                raise ValueError('Invalid optical camera calibration')
            ray = (p @ np.linalg.inv(k).T).astype(float) @ r.T
            with np.errstate(divide='ignore', invalid='ignore'):
                scale = -t[2]/ray[:, 2]
                points = ray*scale[:, None]+t
            keep = (scale >= 0) & np.isfinite(points).all(axis=1) & (points[:, 0] >= 0) & (points[:, 0] <= 10) & (np.abs(points[:, 1]) <= 4)
            self.domain = keep.reshape(mask.shape).astype(np.uint8)*255
        binary = (mask > 0).astype(np.uint8)*255
        clean = np.zeros_like(binary)
        for p in lp.components(binary, max(3, lp.rounded(100*max(.01, mask.size/(1920*1080))))):
            clean[p[:, 1], p[:, 0]] = 255
        return cv2.bitwise_and(clean, self.domain)

    def alternatives(self, mask, camera, publisher, before):
        probe = GroundProbe()
        probe.h, probe.w, probe.top, probe.center, probe.scale = publisher.h, publisher.w, publisher.top, publisher.center, publisher.scale
        mask = self.ground_mask(mask, camera)
        choices = [[], []]
        for side in (0, 1):
            minimum = max(8, lp.rounded(50*probe.h*probe.w/(1920*1080)))
            parts = sorted(lp.components(cv2.bitwise_and(mask, probe.roi(side, None)), minimum), key=lambda p: -p[:, 1].max())
            results = [probe.evaluate(p, side, None) for p in parts]
            results += probe.fragments(mask, side, camera)
            for result in results:
                if result['status'] != 'ok':
                    continue
                line, previous = result['line'], before[0][side]
                tracked = previous is not None and before[1][side] <= 5
                if tracked and all(abs((line[0]-previous[0])*v+line[1]-previous[1]) > max(4*probe.margin(v), .4*probe.w)
                                   for v in (probe.top, probe.h-1)):
                    continue
                cloud = result.get('cloud')
                if cloud is None:
                    cloud = lp.project(result['pixels'], camera)
                if indices(cloud):
                    result['cloud'] = cloud
                    choices[side].append(result)
        return probe, choices

    def resolve(self, mask, request, clouds, publisher, before):
        if not self.begin(request, mask.shape[:2]):
            return dict(clouds=[[], [], []], indices=(2, 4, 6), mode='default_straight', reason='corridor_nonfresh_request')
        quality = paired(clouds)
        prior = self.calibration()
        if quality:
            quality['calibration_consistent'] = prior is None or abs(quality['width_m']-prior['width_m']) <= .05*prior['width_m']
        self.calibrate(clouds, quality)
        base = dict(source_sequence=self.sequence, source_frame_stamp=self.stamp)
        # Preserve existing 5+/2m center priority exactly. Quality below controls
        # calibration only; this branch is not advertised as universally trusted.
        long_ix = indices(clouds[2], True)
        if long_ix and min(len(clouds[0]), len(clouds[1])) > long_ix[-1]:
            return dict(clouds=clouds, indices=long_ix, mode='center', reason='adapter_opt_in_observed_center',
                        evidence=dict(base, target_type='observed_center', pair_quality=quality))
        if quality and quality['trusted'] and quality['calibration_consistent']:
            return dict(clouds=clouds, indices=indices(clouds[2]), mode='center', reason='adapter_opt_in_trusted_short_center',
                        evidence=dict(base, target_type='observed_center', pair_quality=quality))
        probe, choices = self.alternatives(mask, request['camera'], publisher, before)
        pairs = []
        for left in choices[0]:
            for right in choices[1]:
                ll, rl = left['line'], right['line']
                if any((rl[0]-ll[0])*v+rl[1]-ll[1] < .65*probe.spacing(v) for v in (lp.rounded(.78*probe.h), probe.h-1)):
                    continue
                center = center_cloud(left['cloud'], right['cloud'])
                cs = [left['cloud'], right['cloud'], center]
                q = paired(cs)
                calibration = self.calibration()
                consistent = q and (calibration is None or abs(q['width_m']-calibration['width_m']) <= .05*calibration['width_m'])
                if q and q['trusted'] and consistent:
                    pairs.append((q['heading_difference_deg'], q['variation_ratio'], left, right, cs, q))
        if pairs:
            _, _, left, right, cs, q = min(pairs, key=lambda item: item[:2])
            self.calibrate(cs, q)
            for side, result in enumerate((left, right)):
                publisher.lines[side], publisher.lost[side] = result['line'].copy(), 0
            return dict(clouds=[c.astype(np.float32).tolist() for c in cs], indices=indices(cs[2]), mode='center',
                reason='adapter_opt_in_ground_observed_center', evidence=dict(base, target_type='observed_center', pair_quality=q,
                    raw_sources=[self.source(left), self.source(right)]))
        calibration = self.calibration()
        current_right = np.asarray(clouds[1], dtype=float)
        source = dict(kind='original_current_right', point_count=len(current_right))
        chosen = None
        if not indices(current_right) and choices[1]:
            ambiguous = False
            for i, a in enumerate(choices[1]):
                for b in choices[1][:i]:
                    ca, cb = a['cloud'], b['cloud']
                    x = ca[(ca[:, 0] >= cb[0, 0]) & (ca[:, 0] <= cb[-1, 0]), 0]
                    if (len(x) < 2 or x[-1]-x[0] < 1 or calibration is None
                            or np.max(np.abs(np.interp(x, ca[:, 0], ca[:, 1])-np.interp(x, cb[:, 0], cb[:, 1]))) > .05*calibration['width_m']):
                        ambiguous = True
            if not ambiguous:
                chosen = min(choices[1], key=lambda c: (float(c['cloud'][0, 0]), -float(c['cloud'][-1, 0]-c['cloud'][0, 0])))
                current_right, source = chosen['cloud'], self.source(chosen)
            else:
                source = dict(kind='ambiguous_current_right_candidates', candidate_count=len(choices[1]))
        if calibration and indices(current_right):
            if chosen is not None:
                publisher.lines[1], publisher.lost[1] = chosen['line'].copy(), 0
            publisher.lines[0], publisher.lost[0] = None, 6
            return dict(clouds=[[], current_right.astype(np.float32).tolist(), []], indices=indices(current_right),
                mode='right_only', reason='adapter_opt_in_current_right_inferred_center', width_m=calibration['width_m'],
                evidence=dict(base, target_type='inferred_center_from_current_right', raw_right_source=source,
                    **calibration))
        identity = self.left_identity()
        reference = identity['line'] if identity else None
        raw_left_match = match_left_identity(clouds[0], reference)
        left_choices = []
        if raw_left_match['accepted']:
            left_choices.append((np.asarray(clouds[0], dtype=float), None, raw_left_match))
        else:
            for alternative in choices[0]:
                match = match_left_identity(alternative['cloud'], reference)
                if match['accepted']:
                    left_choices.append((alternative['cloud'], alternative, match))
        ambiguous = False
        for i, (a, _, _) in enumerate(left_choices):
            for b, _, _ in left_choices[:i]:
                lo, hi = max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0])
                x = np.unique(np.concatenate((a[:, 0], b[:, 0])))
                x = x[(x >= lo) & (x <= hi)]
                if hi-lo < 1 or np.max(np.abs(np.interp(x, a[:, 0], a[:, 1])-np.interp(x, b[:, 0], b[:, 1]))) > .25:
                    ambiguous = True
        if left_choices and not ambiguous:
            current_left, alternative, match = min(left_choices, key=lambda item: (
                item[2]['max_position_difference_m'], -item[2]['overlap_span_m']))
            if alternative is not None:
                publisher.lines[0], publisher.lost[0] = alternative['line'].copy(), 0
            publisher.lines[1], publisher.lost[1] = None, 6
            left_source = self.source(alternative) if alternative is not None else dict(
                kind='original_current_left', point_count=len(current_left))
            return dict(clouds=[current_left.astype(np.float32).tolist(), [], []],
                indices=indices(current_left), mode='left_only',
                reason='adapter_opt_in_current_left_identity_checked_native',
                identity_checked_left=True,
                evidence=dict(base, target_type='current_left_identity_checked_native',
                    raw_left_source=left_source, left_identity_match=match,
                    raw_left_identity_match=raw_left_match, identity_reference=identity['line'],
                    identity_paired_x_range=identity['paired_x_range'],
                    native_left_width_m=2.6, **calibration))
        return dict(clouds=[[], [], []], indices=(2, 4, 6), mode='default_straight', reason='adapter_opt_in_no_current_corridor',
            evidence=dict(base, target_type='unsupported', current_right_count=len(current_right),
                          raw_right_source=source, calibration=calibration,
                          raw_left_identity_match=raw_left_match,
                          left_identity_candidate_count=len(left_choices), left_identity_ambiguous=ambiguous,
                          reason='no_current_right' if not indices(current_right) else 'width_calibration_unavailable'))

    @staticmethod
    def source(result):
        return dict(kind=result['mode'], component_pixels=result['component_pixels'],
                    fragment_count=result.get('fragment_count'), point_count=len(result['cloud']),
                    observed_x_range=[float(result['cloud'][0, 0]), float(result['cloud'][-1, 0])])
