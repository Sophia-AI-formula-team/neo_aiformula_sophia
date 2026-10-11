"""Private PCA moments dispatch with literal NumPy fallback.

No support selection, reduction order, geometry or admission gate is changed.
Only canonical bounded arrays under ignored underflow use the optional native
API. The returned bytes-backed statistics are temporary, read-only inputs to
eigh; they are not an external mutable-array API.
"""
import numpy as np

try:
    import _lane_mapping_lya_native as _native
except ImportError:
    _native = None

_NATIVE_MOMENTS = (getattr(_native, 'local_moments', None)
    if getattr(_native, 'LOCAL_MOMENTS_API_VERSION', None) == 1 else None)


def _moments(delta, weights, counts, products, native):
    eligible = (native is not None
        and type(delta) is np.ndarray and type(weights) is np.ndarray
        and type(counts) is np.ndarray
        and delta.dtype == np.dtype(np.float64) and delta.dtype.isnative
        and weights.dtype == np.dtype(np.bool_)
        and counts.dtype == np.dtype(np.int64) and counts.dtype.isnative
        and delta.ndim == 3 and delta.shape[2] == 2
        and weights.ndim == 2 and delta.shape[:2] == weights.shape
        and counts.shape == (len(delta),)
        and delta.flags.c_contiguous and weights.flags.c_contiguous
        and counts.flags.c_contiguous
        and 1 <= len(delta) <= 3000 and 1 <= weights.shape[1] <= 2048
        and np.geterr()['under'] == 'ignore')
    if eligible:
        # Ignore only this non-throwing finite/bound check. The original path
        # below still owns all exceptions for rejected input. Overflow/invalid
        # in the native arithmetic is impossible under these finite bounds.
        with np.errstate(all='ignore'):
            eligible = (np.all(np.isfinite(delta)) and np.all(np.abs(delta) <= 2.e7)
                and np.all(weights.view(np.uint8) <= 1) and np.all(counts > 0)
                and np.array_equal(counts, np.count_nonzero(weights, axis=1)))
    if eligible:
        raw_means, raw_covariance = native(delta.tobytes(), weights.tobytes(),
                                           int(len(delta)), int(weights.shape[1]))
        means = np.frombuffer(raw_means, dtype=np.float64).reshape(-1, 2)
        covariance = np.frombuffer(raw_covariance, dtype=np.float64).reshape(-1, 2, 2)
        return means, covariance
    means = np.sum(delta*weights[:, :, None], axis=1)/counts[:, None]
    centered = delta-means[:, None, :]
    covariance = products(centered, centered, weights)/counts[:, None, None]
    return means, covariance
