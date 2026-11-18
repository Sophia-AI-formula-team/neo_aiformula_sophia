"""Identify identical forward requests before evaluating their motion."""
import numpy as np


def distinct_requests(targets, commands):
    """Return first source slots and a dense index that restores every original slot."""
    target_bits = np.ascontiguousarray(targets, dtype=np.float64).view(np.uint64)
    speed_bits = np.ascontiguousarray(commands, dtype=np.float64).view(np.uint64)
    if target_bits.ndim != 1 or speed_bits.shape != target_bits.shape:
        raise ValueError('Expected one lateral target per speed')
    seen = {}
    representatives, inverse = [], []
    for slot, (x, speed) in enumerate(zip(target_bits, speed_bits)):
        key = int(x), int(speed)
        index = seen.get(key)
        if index is None:
            index = len(representatives)
            seen[key] = index
            representatives.append(slot)
        inverse.append(index)
    return np.asarray(representatives, dtype=np.intp), np.asarray(inverse, dtype=np.intp)
