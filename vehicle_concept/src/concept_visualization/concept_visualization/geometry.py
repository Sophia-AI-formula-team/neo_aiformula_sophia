"""Coordinate conversion used by the markers and frozen-camera overlay."""
import math


def body_point(forward, right):
    return (float(forward), -float(right))


def memory_to_body(pose, forward, right):
    f, r, yaw = pose
    df, dr = forward - f, right - r
    c, s = math.cos(yaw), math.sin(yaw)
    return body_point(c*df+s*dr, -s*df+c*dr)


def pixel(point):
    z, right = point[:2]
    if z < .55 or not math.isfinite(z + right):
        return None
    return (round(330.01302 + 254.391622*right/z),
            round(181.14964 + 254.391622*.655/z))


class ObservedMap:
    def __init__(self):
        self.session = None
        self.cells = {}
        self.paint = {}
        self.repairs = {}
        self.last_t = None

    def update(self, decision):
        session = decision.get('session')
        data = decision.get('map') or {}
        stamp = data.get('t')
        if session != self.session or (stamp is not None and self.last_t is not None and stamp < self.last_t):
            self.cells.clear()
            self.paint.clear()
            self.repairs.clear()
            self.session = session
            self.last_t = None
        if stamp is not None:
            self.last_t = stamp
        for item in data.get('changed', []):
            self.cells[tuple(item[:2])] = item[2:5]
        semantics = decision.get('map_semantics') or {}
        for i, j, kind in semantics.get('changed_markings', []):
            self.paint[i,j] = kind
        for i, j, kind in semantics.get('changed_repairs', []):
            self.repairs[i,j] = kind

    def points(self, pose):
        result = {'ego': [], 'adjacent': [], 'paint': []}
        for i, j in self.cells.keys() | self.repairs.keys():
            ego, adjacent, _ = self.cells.get((i, j), (0, 0, 0))
            if ego or adjacent:
                kind = 'ego' if ego >= adjacent else 'adjacent'
            else:
                kind = 'adjacent' if self.repairs.get((i, j)) == 'adjacent' else 'ego'
            result[kind].append(memory_to_body(pose, (i+.5)*.2, (j+.5)*.2))
        result['paint'] = [memory_to_body(pose,(i+.5)*.2,(j+.5)*.2) for i,j in self.paint]
        return result
