"""Versioned, bounded JSON checkpoints; no ROS or control state is serialized."""
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

SCHEMA_VERSION = 1
MAX_BYTES = 16 * 1024 * 1024


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def validate_state(state, max_samples):
    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('nonfinite_or_invalid_number')

    def pose(value):
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError('invalid_pose')
        for item in value:
            number(item)

    if not isinstance(state, dict):
        raise ValueError('invalid_state')
    samples = state.get('samples')
    if not isinstance(samples, list) or len(samples) > max_samples:
        raise ValueError('invalid_sample_count')
    for row in samples:
        if not isinstance(row, list) or len(row) != 4 or type(row[3]) is not bool:
            raise ValueError('invalid_sample')
        pose(row[:3])
    for key in ('recorded_distance', 'progress'):
        number(state.get(key))
        # Closed-route progress is signed/unwrapped at the start seam. Travel
        # distance (and progress before a reference exists) stays nonnegative.
        if state[key] < 0 and (key == 'recorded_distance' or state.get('closed_at') is None):
            raise ValueError('negative_' + key)
    for key in ('left_start_area', 'excluded_since_sample'):
        if type(state.get(key)) is not bool:
            raise ValueError('invalid_' + key)
    for key in ('odom_frame', 'child_frame'):
        if not isinstance(state.get(key), str) or len(state[key]) > 256:
            raise ValueError('invalid_frame')
    for key in ('last_pose', 'closed_at'):
        if state.get(key) is not None:
            pose(state[key])
    if state.get('odom_stamp') is not None:
        number(state['odom_stamp'])
    if samples and (state.get('last_pose') is None or not state['odom_frame']):
        raise ValueError('missing_anchor')
    if state.get('closed_at') is not None and len(samples) < 30:
        raise ValueError('closed_map_too_short')
    if not samples and (state['recorded_distance'] != 0 or state.get('closed_at') is not None):
        raise ValueError('invalid_empty_map')
    return state


class CheckpointStore:
    """Single writer; atomic replace + fsync keeps previous file on write failure."""
    def __init__(self, filename, fingerprint, max_samples):
        self.path = Path(filename).expanduser().absolute()
        self.fingerprint, self.max_samples = fingerprint, max_samples
        self.lock = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = open(str(self.path) + '.lock', 'a')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            raise ValueError('checkpoint_in_use')
        self.lock = handle

    def load(self):
        if not self.path.exists():
            return None
        with self.path.open('rb') as stream:
            raw = stream.read(MAX_BYTES + 1)
        if len(raw) > MAX_BYTES:
            raise ValueError('checkpoint_too_large')
        envelope = json.loads(raw)
        if not isinstance(envelope, dict):
            raise ValueError('invalid_checkpoint')
        if envelope.get('schema_version') != SCHEMA_VERSION:
            raise ValueError('schema_mismatch')
        if envelope.get('fingerprint') != self.fingerprint:
            raise ValueError('map_or_config_mismatch')
        state = validate_state(envelope.get('state'), self.max_samples)
        if envelope.get('sha256') != digest(state):
            raise ValueError('checksum_mismatch')
        return state

    def save(self, state):
        validate_state(state, self.max_samples)
        raw = json.dumps(dict(schema_version=SCHEMA_VERSION, fingerprint=self.fingerprint,
                              state=state, sha256=digest(state)), allow_nan=False,
                         separators=(',', ':')).encode()
        if len(raw) > MAX_BYTES:
            raise ValueError('checkpoint_too_large')
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(dir=str(self.path.parent), prefix=self.path.name + '.',
                                             suffix='.tmp', delete=False) as stream:
                tmp = stream.name
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(tmp, str(self.path))
            tmp = None
            fd = os.open(str(self.path.parent), os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            if tmp is not None:
                os.unlink(tmp)

    def close(self):
        if self.lock is not None:
            self.lock.close()
            self.lock = None
