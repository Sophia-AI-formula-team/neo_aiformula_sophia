"""Compact read-only snapshot in the odometry frame at driver startup."""

import base64
from itertools import chain
import json
import zlib

import numpy as np

from concept_road.map_semantics618 import marking_kind
from concept_road.runtime_map import CELL


def grid_rows(mapper, semantics):
    for (z, x), (ego, adjacent, _, _) in mapper.cells.items():
        if ego or adjacent:
            kind = 1 if ego >= adjacent else 2
        else:
            kind = {'ego': 1, 'adjacent': 2}.get(semantics.repairs.get((z, x)), 3)
        yield z, x, kind
    for (z, x), kind in semantics.repairs.items():
        if (z, x) not in mapper.cells:
            yield z, x, 2 if kind == 'adjacent' else 1


def pack_rows(rows, dtype):
    values = np.fromiter(chain.from_iterable(rows), dtype=dtype)
    return base64.b64encode(zlib.compress(values.tobytes(), 1)).decode('ascii')


def encode_map(driver, session, stamp):
    mapper, semantics = driver.mapper, driver.semantics
    labels = {'solid': 1, 'dashed': 2, 'unclassified': 3}
    return json.dumps({
        'schema': 'concept.lap-map.v1', 'session': session, 'stamp': stamp,
        'cell_m': CELL, 'pose': mapper.pose.tolist(), 'distance_m': mapper.distance,
        'road': pack_rows(grid_rows(mapper, semantics), '<i4'),
        'lines': pack_rows(((z, x, labels[marking_kind(v)]) for (z, x), v in semantics.markings.items()), '<i4'),
        'track': pack_rows(((t, z, x) for t, z, x, _ in mapper.path), '<f4'),
        'static': list(mapper.static.values()), 'moving': list(mapper.dynamic.values()),
        'openings': driver.observed_openings,
    }, separators=(',', ':'), allow_nan=False)
