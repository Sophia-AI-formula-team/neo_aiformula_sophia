"""OpenCV port of pinned lane_line_publisher.cpp (not a native ROS node).

Defaults and branch ordering follow commit 05f1651. Eigen QR is replaced by
NumPy least squares; native C++ numerical parity has not been established.
Projected-component recovery is adapter-only, explicit opt-in; defaults keep this pinned port.
"""
import math
import cv2
import numpy as np


def rounded(x):
    return int(math.copysign(math.floor(abs(x) + .5), x))


def components(mask, minimum):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
    return [np.column_stack(np.where(labels == i)[::-1]) for i in range(1, n)
            if stats[i, cv2.CC_STAT_AREA] >= minimum]


def fit(p):
    if len(p) < 3:
        return None
    u, v = p.astype(float).T
    dv = v - v.mean()
    variance = dv @ dv
    if variance < 1e-9:
        return None
    a = float(dv @ (u-u.mean()) / variance)
    return np.array([a, float(u.mean()-a*v.mean())])


def geometry(line, side):
    if line is None:
        return False
    a = line[0]
    return 7 <= math.degrees(math.atan2(1, a)) <= 173 and .08 <= abs(a) <= 10 and (a < 0 if side == 0 else a > 0)


class LanePublisher:
    def __init__(self):
        self.lines = [None, None]
        self.lost = [0, 0]

    def spacing(self, v):
        return (40+100*self.ratio(v))*self.scale

    def ratio(self, v):
        return np.clip((v-self.top)/max(1, self.h-1-self.top), 0, 1)

    def margin(self, v):
        return (60+120*self.ratio(v))*self.scale

    def roi(self, side, tracking):
        out = np.zeros((self.h, self.w), np.uint8)
        other = self.lines[1-side]
        clamp = lambda x: max(0, min(self.w-1, x))
        for v in range(self.top, self.h):
            gap = self.spacing(v)
            lo, hi = (0, clamp(math.floor(self.center-gap/2))) if side == 0 else (clamp(math.ceil(self.center+gap/2)), self.w-1)
            if tracking is not None:
                pred = tracking[0]*v+tracking[1]
                lo = max(lo, clamp(math.floor(pred-self.margin(v))))
                hi = min(hi, clamp(math.ceil(pred+self.margin(v))))
            if other is not None:
                pred = other[0]*v+other[1]
                if side == 0:
                    hi = min(hi, clamp(math.floor(pred-gap)))
                else:
                    lo = max(lo, clamp(math.ceil(pred+gap)))
            if lo <= hi:
                out[v, lo:hi+1] = 255
        return out

    def select(self, mask, side, tracking):
        items = sorted(components(cv2.bitwise_and(mask, self.roi(side, tracking)), 50),
                       key=lambda p: -p[:, 1].max())
        index = 0
        while index < len(items):
            bottom = items[index][:, 1].max()
            best, score = np.empty((0, 2), int), math.inf
            while index < len(items) and items[index][:, 1].max() >= bottom-max(2, self.h//100):
                p = items[index]
                index += 1
                if tracking is not None:
                    value = np.abs(p[:, 0]-(p[:, 1]*tracking[0]+tracking[1])).mean()
                else:
                    line = fit(p)
                    if not geometry(line, side):
                        continue
                    band = p[p[:, 1] >= rounded(.82*self.h)]
                    if not len(band):
                        band = p[p[:, 1] >= np.sort(p[:, 1])[int(.75*(len(p)-1))]]
                    anchor = self.center-band[:, 0].max() if side == 0 else band[:, 0].min()-self.center
                    if anchor < 0:
                        continue
                    value = anchor+.2*(self.h-1-p[:, 1].max())-min(200, .01*len(np.unique(p[:, 1])))
                if value < score:
                    best, score = p, value
            if len(best) and math.isfinite(score):
                return best
        return np.empty((0, 2), int)

    def path(self, p, side, reference):
        rows = []
        for v in np.unique(p[:, 1]):
            u = np.sort(p[p[:, 1] == v, 0])
            runs = np.split(u, np.where(np.diff(u) > 1)[0]+1)
            candidates = [float(run.mean()) for run in runs if run[-1]-run[0]+1 <= max(3, rounded(.09*self.w))]
            if candidates:
                rows.append((int(v), candidates))
        if len(rows) < 12:
            return np.empty((0, 2), int)
        if reference is not None:
            line = reference
        else:
            seed = []
            for v, candidates in rows:
                choices = [u for u in candidates if (u <= self.center if side == 0 else u >= self.center)] or candidates
                seed.append((rounded(max(choices) if side == 0 else min(choices)), v))
            line = fit(np.asarray(seed))
        for _ in range(2):
            if line is None:
                return np.empty((0, 2), int)
            chosen = []
            for v, candidates in rows:
                pred = line[0]*v+line[1]
                u = min(candidates, key=lambda u: abs(u-pred))
                if abs(u-pred) <= max(12, .06*self.w):
                    chosen.append((rounded(u), v))
            if len(chosen) < 12:
                return np.empty((0, 2), int)
            path = np.asarray(chosen)
            line = fit(path)
        return path

    def evaluate(self, p, side, ref):
        result = dict(status="ok", component_pixels=int(len(p)), bottom=int(p[:, 1].max()) if len(p) else -1,
                      mode="tracking" if ref is not None else "init", pixels=p, line=None)
        if len(p) < 180:
            result["status"] = "too_few_component_pixels"
        elif len(np.unique(p[:, 1]))/max(1, self.h-self.top) < .1:
            result["status"] = "insufficient_row_coverage"
        elif p[:, 1].max()-p[:, 1].min()+1 < max(1, rounded(.05*self.h)):
            result["status"] = "insufficient_component_vertical_span"
        else:
            path = self.path(p, side, ref)
            if len(path) < 12:
                result["status"] = "insufficient_continuous_lane_path"
            else:
                result["pixels"], result["line"] = path, fit(path)
                if not geometry(result["line"], side):
                    result["status"] = "rejected_side_geometry"
        return result

    def side(self, mask, side):
        prev = self.lines[side]
        tracking = prev is not None and self.lost[side] <= 5
        ref = prev if tracking else None
        result = self.evaluate(self.select(mask, side, ref), side, ref)
        if tracking and (result["status"] != "ok" or result["bottom"] < rounded(.78*self.h)):
            recovery = self.evaluate(self.select(mask, side, None), side, None)
            if recovery["status"] == "ok" and (result["status"] != "ok" or recovery["bottom"] >= result["bottom"]+12):
                result, tracking = recovery, False
        if result["status"] != "ok":
            self.lines[side], self.lost[side] = None, 6
            return result
        line = result["line"]
        if tracking and all(abs((line[0]-prev[0])*v+line[1]-prev[1]) > max(4*self.margin(v), .4*self.w)
                            for v in (self.top, self.h-1)):
            result["status"] = "rejected_excessive_jump"
            self.lost[side] += 1
            if self.lost[side] > 5:
                self.lines[side] = None
        else:
            self.lines[side] = line.copy() if prev is None else .9*line+.1*prev
            self.lost[side] = 0
        return result

    def fragmented_side(self, mask, side, camera):
        """同一帧里连接共线虚线片段；不生成掩码、不外推观测范围。"""
        area_scale = self.h*self.w/(1920*1080)
        parts = [p for p in components(cv2.bitwise_and(mask, self.roi(side, None)),
                                      max(8, rounded(50*area_scale)))
                 if len(np.unique(p[:, 1])) >= 3]
        parts = sorted(parts, key=lambda p: -len(p))[:20]
        if len(parts) < 2:
            return None
        centers = np.asarray([p.mean(axis=0) for p in parts])
        tolerance = max(3., .006*self.w)
        seeds = [fit(p) for p in parts]
        for i in range(len(parts)):
            for j in range(i):
                du, dv = centers[i]-centers[j]
                if abs(dv) >= max(6, .02*self.h):
                    a = du/dv
                    seeds.append(np.asarray([a, centers[i, 0]-a*centers[i, 1]]))
        seen, best, best_score = set(), None, None
        for line in seeds:
            if not geometry(line, side):
                continue
            close = np.flatnonzero(np.abs(centers[:, 0]-centers[:, 1]*line[0]-line[1]) <= tolerance)
            members = tuple(int(i) for i in close
                            if np.quantile(np.abs(parts[i][:, 0]-parts[i][:, 1]*line[0]-line[1]), .9) <= 2*tolerance)
            if len(members) < 2 or members in seen:
                continue
            seen.add(members)
            pixels = np.concatenate([parts[i] for i in members])
            if len(pixels) < max(50, rounded(180*area_scale)):
                continue
            if len(np.unique(pixels[:, 1]))/max(1, self.h-self.top) < .1:
                continue
            line = fit(pixels)
            if not geometry(line, side):
                continue
            if np.quantile(np.abs(pixels[:, 0]-pixels[:, 1]*line[0]-line[1]), .9) > 2*tolerance:
                continue
            path = self.path(pixels, side, line)
            if len(path) < 12:
                continue
            line = fit(path)
            if not geometry(line, side):
                continue
            cloud = project(path, camera)
            if len(cloud) < 5 or cloud[-1, 0]-cloud[0, 0] < 2:
                continue
            span = float(cloud[-1, 0]-cloud[0, 0])
            near = float(abs(np.interp(3., cloud[:, 0], cloud[:, 1])))
            score = (span, -near, len(path))
            if best_score is None or score > best_score:
                best_score = score
                best = dict(status="ok", component_pixels=int(len(pixels)), bottom=int(pixels[:, 1].max()),
                            mode="fragmented_current_mask", pixels=path, line=line, cloud=cloud,
                            fragment_count=len(members), observed_span_m=span)
        return best

    @staticmethod
    def observed_triplet_available(cloud):
        """The adapter's existing 5/6/7+ point contract, without new points."""
        n = len(cloud)
        indices = (2, 4, 6) if n >= 7 else (1, 3, 5) if n == 6 else (0, 2, 4) if n == 5 else None
        if indices is None:
            return False
        selected = np.asarray(cloud)[list(indices)]
        return bool(selected.shape == (3, 3) and np.isfinite(selected).all()
                    and selected[0, 0] < selected[1, 0] < selected[2, 0]
                    and selected[2, 0]-selected[0, 0] >= 2.0-1e-6)

    def ranked_components(self, mask, side, tracking):
        """Continue the existing bottom-band/score order after its first choice."""
        items = sorted(components(cv2.bitwise_and(mask, self.roi(side, tracking)), 50),
                       key=lambda p: -p[:, 1].max())
        index = 0
        while index < len(items):
            bottom, band = items[index][:, 1].max(), []
            while index < len(items) and items[index][:, 1].max() >= bottom-max(2, self.h//100):
                p = items[index]
                index += 1
                if tracking is not None:
                    score = np.abs(p[:, 0]-(p[:, 1]*tracking[0]+tracking[1])).mean()
                else:
                    if not geometry(fit(p), side):
                        continue
                    anchor_band = p[p[:, 1] >= rounded(.82*self.h)]
                    if not len(anchor_band):
                        anchor_band = p[p[:, 1] >= np.sort(p[:, 1])[int(.75*(len(p)-1))]]
                    anchor = self.center-anchor_band[:, 0].max() if side == 0 else anchor_band[:, 0].min()-self.center
                    if anchor < 0:
                        continue
                    score = anchor+.2*(self.h-1-p[:, 1].max())-min(200, .01*len(np.unique(p[:, 1])))
                if math.isfinite(score):
                    band.append((score, p))
            for _, p in sorted(band, key=lambda item: item[0]):
                yield p

    def recover_components(self, mask, camera, results, clouds, before_lines, before_lost):
        """Opt-in adapter fallback: original projection of another current component.

        A separate probe preserves the pre-frame ROI/jump reference. Failed
        near components cannot alter it, and failed recovery commits no state.
        """
        probe = LanePublisher()
        probe.h, probe.w, probe.top = self.h, self.w, self.top
        probe.center, probe.scale = self.center, self.scale
        probe.lines = [None if line is None else line.copy() for line in before_lines]
        probe.lost = list(before_lost)
        alternatives = [[], []]
        for side in (0, 1):
            prev = before_lines[side]
            tracking = prev is not None and before_lost[side] <= 5
            references = (prev, None) if tracking else (None,)
            seen = set()
            for reference in references:
                for rank, pixels in enumerate(probe.ranked_components(mask, side, reference)):
                    identity = (reference is not None, pixels.tobytes())
                    if identity in seen:
                        continue
                    seen.add(identity)
                    candidate = probe.evaluate(pixels, side, reference)
                    if candidate["status"] != "ok":
                        continue
                    line = candidate["line"]
                    # Retain the original excessive-jump test against the
                    # saved valid track even when trying the original init ROI.
                    if tracking and all(abs((line[0]-prev[0])*v+line[1]-prev[1]) > max(4*probe.margin(v), .4*probe.w)
                                        for v in (probe.top, probe.h-1)):
                        continue
                    cloud = project(candidate["pixels"], camera)
                    if not self.observed_triplet_available(cloud):
                        continue
                    candidate.update(mode="projected_component_recovery",
                                     recovery_source="adapter_opt_in_current_component",
                                     recovery_reference="tracking" if reference is not None else "init",
                                     component_rank=rank,
                                     observed_span_m=float(cloud[-1, 0]-cloud[0, 0]))
                    state = line.copy() if prev is None else .9*line+.1*prev
                    alternatives[side].append((candidate, cloud, state))
        # Prefer replacing one side. Try two-sided alternatives only when
        # every single replacement fails the original final spacing guard.
        combinations = [((item, None)) for item in alternatives[0]]
        combinations += [((None, item)) for item in alternatives[1]]
        combinations += [(left, right) for left in alternatives[0] for right in alternatives[1]]
        for choices in combinations:
            candidate_results = [choices[i][0] if choices[i] else results[i] for i in (0, 1)]
            candidate_clouds = [choices[i][1] if choices[i] else clouds[i] for i in (0, 1)]
            if all(item["status"] == "ok" for item in candidate_results):
                left, right = (item["line"] for item in candidate_results)
                if any((right[0]-left[0])*v+right[1]-left[1] < .65*self.spacing(v)
                       for v in (rounded(.78*self.h), self.h-1)):
                    continue
            if not any(self.observed_triplet_available(cloud) for cloud in candidate_clouds):
                continue
            for side, choice in enumerate(choices):
                if choice is not None:
                    self.lines[side], self.lost[side] = choice[2], 0
            return candidate_results, candidate_clouds
        return None

    def process(self, mask, camera, recover_projected_components=False):
        recover_projected_components = recover_projected_components is True
        if recover_projected_components:
            before_lines = [None if line is None else line.copy() for line in self.lines]
            before_lost = list(self.lost)
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        self.h, self.w = mask.shape
        self.top, self.center, self.scale = rounded(.5*self.h), .5*(self.w-1), max(.05, self.w/1920)
        binary = (mask > 0).astype(np.uint8)*255
        clean = np.zeros_like(binary)
        for p in components(binary, max(3, rounded(100*max(.01, self.h*self.w/(1920*1080))))):
            clean[p[:, 1], p[:, 0]] = 255
        polygon = np.asarray([(0, self.h-1), (self.w-1, self.h-1), (self.w-1, rounded(.65*self.h)),
                              (rounded(.7*self.w), rounded(.58*self.h)), (rounded(.3*self.w), rounded(.58*self.h)),
                              (0, rounded(.65*self.h))], np.int32)
        roi = cv2.fillPoly(np.zeros_like(clean), [polygon], 255)
        cropped = cv2.bitwise_and(clean, roi)
        results = [self.side(cropped, side) for side in (0, 1)]
        if all(r["status"] == "ok" for r in results):
            left, right = (r["line"] for r in results)
            if any((right[0]-left[0])*v+right[1]-left[1] < .65*self.spacing(v) for v in (rounded(.78*self.h), self.h-1)):
                results[0]["status"] = "rejected_lane_spacing"
                self.lines[0], self.lost[0] = None, 6
        clouds = []
        for side, result in enumerate(results):
            cloud = project(result["pixels"], camera) if result["status"] == "ok" else np.empty((0, 3))
            if len(cloud) < 7:
                recovery = self.fragmented_side(cropped, side, camera)
                if recovery is not None and len(recovery["cloud"]) > len(cloud):
                    cloud = recovery.pop("cloud")
                    results[side] = result = recovery
                    self.lines[side], self.lost[side] = result["line"].copy(), 0
            if result["status"] == "ok" and not len(cloud):
                result["status"] = "projected_output_insufficient"
                self.lines[side], self.lost[side] = None, 6
            clouds.append(cloud)
        if all(r["status"] == "ok" for r in results) and any(r["mode"] == "fragmented_current_mask" for r in results):
            left_line, right_line = (r["line"] for r in results)
            if any((right_line[0]-left_line[0])*v+right_line[1]-left_line[1] < .65*self.spacing(v)
                   for v in (rounded(.78*self.h), self.h-1)):
                results[0]["status"] = "rejected_lane_spacing"
                self.lines[0], self.lost[0] = None, 6
                clouds[0] = np.empty((0, 3))
        left, right = clouds
        center = np.empty((0, 3))
        if len(left) and len(right):
            overlap = left[(left[:, 0] >= right[0, 0]) & (left[:, 0] <= right[-1, 0])].copy()
            if len(overlap):
                overlap[:, 1] = .5*(overlap[:, 1]+np.interp(overlap[:, 0], right[:, 0], right[:, 1]))
                center = overlap
        if recover_projected_components and not any(self.observed_triplet_available(c) for c in (left, right, center)):
            recovered = self.recover_components(cropped, camera, results, clouds, before_lines, before_lost)
            if recovered is not None:
                results, (left, right) = recovered
                center = np.empty((0, 3))
                if len(left) and len(right):
                    overlap = left[(left[:, 0] >= right[0, 0]) & (left[:, 0] <= right[-1, 0])].copy()
                    if len(overlap):
                        overlap[:, 1] = .5*(overlap[:, 1]+np.interp(overlap[:, 0], right[:, 0], right[:, 1]))
                        center = overlap
        debug = [{k: v for k, v in r.items() if k not in ("pixels", "line")} for r in results]
        return [c.astype(np.float32).tolist() for c in (left, right, center)], debug


def project(pixels, camera):
    k = np.asarray(camera["K"], np.float32).reshape(3, 3)
    r = np.asarray(camera["R"], float).reshape(3, 3)
    t = np.asarray(camera["t"], float)
    if not np.isfinite(k).all() or not np.isfinite(r).all() or not np.isfinite(t).all() or t[2] <= 0:
        raise ValueError("Invalid optical camera calibration")
    ray = np.column_stack((pixels, np.ones(len(pixels)))).astype(np.float32) @ np.linalg.inv(k).T
    ray = ray.astype(float) @ r.T
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = -t[2]/ray[:, 2]
        points = ray*scale[:, None]+t
    points = points[(scale >= 0) & np.isfinite(points).all(axis=1) & (points[:, 0] >= 0) & (points[:, 0] <= 10)
                    & (points[:, 1] >= -4) & (points[:, 1] <= 4)]
    points = points[np.argsort(points[:, 0], kind="stable")]
    if len(points):
        points = points[np.r_[True, np.diff(points[:, 0]) > 1e-9]]
    if len(points) < 2 or points[-1, 0]-points[0, 0] < 1:
        return np.empty((0, 3))
    coeff = np.linalg.lstsq(np.column_stack((np.ones(len(points)), points[:, 0])), points[:, 1], rcond=None)[0]
    x = np.arange(math.ceil(points[0, 0]/.5)*.5, math.floor(points[-1, 0]/.5)*.5+.25, .5)
    return np.column_stack((x, coeff[0]+coeff[1]*x, np.zeros(len(x)))) if len(x) >= 3 else np.empty((0, 3))
