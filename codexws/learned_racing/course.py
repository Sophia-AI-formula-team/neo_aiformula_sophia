"""Metric outer corridor for the simulator/referee. The actor never receives this object."""
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d


def area(points):
    other = np.roll(points, -1, axis=0)
    return (points[:, 0] * other[:, 1] - points[:, 1] * other[:, 0]).sum() / 2.


def resample(points, spacing):
    closed = np.vstack((points, points[:1]))
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    target = np.linspace(0, s[-1], round(s[-1] / spacing), endpoint=False)
    return np.column_stack([np.interp(target, s, closed[:, j]) for j in (0, 1)])


def tangent(points):
    delta = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    return delta / np.linalg.norm(delta, axis=1)[:, None]


def build_course(surface_dir: Path):
    surface = surface_dir / "track_surface.json"
    calibration = surface_dir / "TrackMetricCalibration.json"
    meshes = json.loads(surface.read_text(encoding="utf-8-sig"))["meshes"]
    metric = json.loads(calibration.read_text(encoding="utf-8-sig"))
    strip = next(m for m in meshes if m["name"] == "OuterGreenStrip")
    vertices = np.asarray(strip["vertices"])
    triangles = np.asarray(strip["triangles"]).reshape(-1, 3)
    counts = Counter(tuple(sorted(pair)) for tri in triangles
                     for pair in zip(tri, np.roll(tri, -1)))
    neighbours = defaultdict(list)
    for (a, b), count in counts.items():
        if count == 1:
            neighbours[a].append(b)
            neighbours[b].append(a)
    if any(len(v) != 2 for v in neighbours.values()):
        raise ValueError("Expected closed rings around the outer verge")
    unused = set(neighbours)
    rings = []
    while unused:
        first = min(unused)
        previous, current, ring = None, first, []
        while current in unused:
            unused.remove(current)
            ring.append(current)
            a, b = neighbours[current]
            previous, current = current, a if a != previous else b
        if current != first:
            raise ValueError("Mesh has a broken boundary")
        rings.append(vertices[ring])
    if len(rings) != 2:
        raise ValueError("Expected inner and outer edges of the green strip")
    # The smaller edge touches the asphalt. Construct the two-lane outer route
    # from its width; deliberately ignore infield turnoffs, paint and obstacles.
    road_edge = min(rings, key=lambda ring: abs(area(ring)))
    if area(road_edge) < 0:
        road_edge = road_edge[::-1]
    outer = resample(road_edge * metric["scale"], 0.2)
    outer = gaussian_filter1d(outer, 1.5, axis=0, mode="wrap")
    direction = tangent(outer)
    normal = np.column_stack((-direction[:, 1], direction[:, 0]))
    half_width = float(metric["outerLaneCenterWidthM"])
    center = resample(outer + half_width * normal, 0.22)
    center -= center.mean(axis=0)
    # Common start on the middle of the northern straight, used only for resets/timing.
    score = center[:, 1] - 0.03 * np.abs(center[:, 0])
    center = np.roll(center, -int(np.argmax(score)), axis=0)
    return {"source": "Demo_LaneLineTrainer / Simulator / OuterGreenStrip",
            "surface_sha256": hashlib.sha256(surface.read_bytes()).hexdigest(),
            "metric_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
            "half_width_m": half_width, "center_m": np.round(center, 5).tolist()}


PREVIEW = np.array([0., 2., 4., 6., 8.])


def encode_edges(left, right, width):
    return (left+right)/(2*width), (left-right)/(2*width)


class Course:
    def __init__(self, geometry, gates=False, tight=False):
        self.xy = np.asarray(geometry["center_m"], dtype=float)
        self.width = float(geometry["half_width_m"])
        self.length = np.linalg.norm(np.roll(self.xy, -1, axis=0) - self.xy, axis=1).sum()
        self.ds = self.length / len(self.xy)
        self.dir = tangent(self.xy)
        self.normal = np.column_stack((-self.dir[:, 1], self.dir[:, 0]))
        next_dir = np.roll(self.dir, -1, axis=0)
        cross = self.dir[:, 0] * next_dir[:, 1] - self.dir[:, 1] * next_dir[:, 0]
        self.bend = np.arctan2(cross, np.sum(self.dir * next_dir, axis=1)) / self.ds
        self.bend = gaussian_filter1d(self.bend, 2, mode="wrap")
        self.left = np.full(len(self.xy), self.width)
        self.right = np.full(len(self.xy), -self.width)
        if gates:
            # Test layout only. Staggered cone walls narrow the road; they do
            # not specify an apex, target speed or steering command.
            stations = np.arange(len(self.xy))*self.ds
            for group, start in enumerate((24., self.length*0.56)):
                spacing = 7. if tight and group == 1 else 9.
                for j, side in enumerate((1, -1, 1)):
                    distance = (stations - start - spacing*j)
                    extent = 3.9 if spacing == 7. else 5.
                    bump = np.where(abs(distance)<extent, (1+np.cos(np.pi*distance/extent))/2, 0.)
                    if side == 1:
                        self.left -= 4.15*bump
                    else:
                        self.right += 4.15*bump

    def indices(self, s):
        return np.floor(s / self.ds + 0.5).astype(int) % len(self.xy)

    def clearance(self, s, offset, heading, half_length, half_width):
        # Four proxy tyre contact points in the local track frame. Evaluate the
        # boundary at each wheel's station, including on angled cone walls.
        front = np.array([half_length, half_length, -half_length, -half_length])
        side = np.array([half_width, -half_width, half_width, -half_width])
        cs, sn = np.cos(heading)[:, None], np.sin(heading)[:, None]
        scale = (1-self.bend[self.indices(s)]*offset)[:, None]
        foot = self.indices(s[:, None]+(front*cs-side*sn)/np.maximum(scale, 0.30))
        e = offset[:, None]+front*sn+side*cs
        return np.minimum(self.left[foot]-e, e-self.right[foot]).min(axis=1)

    def view(self, s, offset, heading_error):
        # Ideal sensor: lateral locations of BOTH edges at five local cross
        # sections. Return midpoint and half-opening (an invertible change of
        # coordinates). No target trajectory, world pose, clock or lap index.
        index = self.indices(s)
        forward, left = self.dir[index], self.normal[index]
        position = self.xy[index] + offset[:, None] * left
        ahead = self.indices(s[:, None] + PREVIEW)
        body_left = np.cos(heading_error)[:, None]*left - np.sin(heading_error)[:, None]*forward
        lateral = np.sum((self.xy[ahead]-position[:, None])*body_left[:, None], axis=2)
        projection = np.sum(self.normal[ahead]*body_left[:, None], axis=2)
        l, r = lateral + projection*self.left[ahead], lateral + projection*self.right[ahead]
        return encode_edges(l, r, self.width)

    def positions(self, s, offset):
        u = (s / self.ds) % len(self.xy)
        i = np.floor(u).astype(int)
        j = (i + 1) % len(self.xy)
        f = (u-i)[:, None]
        return ((1-f)*(self.xy[i] + offset[:, None]*self.normal[i])
                + f*(self.xy[j] + offset[:, None]*self.normal[j]))
