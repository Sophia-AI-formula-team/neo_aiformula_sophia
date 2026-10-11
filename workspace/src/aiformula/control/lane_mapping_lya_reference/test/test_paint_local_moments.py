"""Literal NumPy oracle for the private extractor PCA moments dispatcher.

Compare actual native calls, exact bytes, warnings and first errors. Geometry
and complete-extractor equivalence are verified separately, not inferred here.
"""
import math
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from lane_mapping_lya_reference import local_moments as HELPER

if os.environ.get('LANE_REQUIRE_NATIVE') == '1':
    import _lane_mapping_lya_native as NATIVE
else:
    NATIVE = pytest.importorskip('_lane_mapping_lya_native')
import unittest
import warnings

import numpy as np



def products(left, right, weights):
    """Literal existing component contraction, not the candidate helper."""
    result = np.empty((len(left), left.shape[2], right.shape[2]), dtype=np.float64)
    for i in range(left.shape[2]):
        for j in range(right.shape[2]):
            result[:, i, j] = np.einsum('nk,nk,nk->n', left[:, :, i],
                                       right[:, :, j], weights, optimize=False)
    return result


def original_np(delta, weights, counts, multiply=products):
    means = np.sum(delta * weights[:, :, None], axis=1) / counts[:, None]
    centered = delta - means[:, None, :]
    covariance = multiply(centered, centered, weights) / counts[:, None, None]
    return means, covariance


class Spy:
    def __init__(self, call):
        self.call, self.calls = call, []

    def __call__(self, *args):
        self.calls.append(args)
        return self.call(*args)


def outcome(call, policy):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        with np.errstate(**policy):
            try:
                value = call()
                result = ('value', tuple((a.dtype.str, a.shape, a.tobytes()) for a in value))
            except Exception as error:
                result = ('error', type(error).__name__, str(error))
    return result, [(type(w.message).__name__, str(w.message)) for w in caught]


class MomentsContract(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(HELPER, 'runner must bind the reviewed helper')
        self.assertIsNotNone(NATIVE, 'native tests cannot be silently skipped')
        self.assertEqual(NATIVE.LOCAL_MOMENTS_API_VERSION, 1)

    def sample(self, n=3, k=17):
        rng = np.random.RandomState(283761 + n + k)
        delta = rng.uniform(-1.35, 1.35, (n, k, 2))
        mask = rng.uniform(size=(n, k)) > .35
        if n and k:
            mask[:, 0] = True
        return delta, mask, np.count_nonzero(mask, axis=1).astype(np.int64)

    def compare(self, args, native_expected, policy=None, use_native=True):
        policy = dict(all='warn', under='ignore') if policy is None else policy
        before = [(a.dtype.str, a.shape, a.tobytes()) for a in args]
        spy = Spy(NATIVE.local_moments)
        multiply = Spy(products)
        expected = outcome(lambda: original_np(*args), policy)
        actual = outcome(lambda: HELPER._moments(*args, multiply,
                                               spy if use_native else None), policy)
        self.assertEqual(actual, expected)
        self.assertEqual(len(spy.calls), int(native_expected))
        self.assertEqual(before, [(a.dtype.str, a.shape, a.tobytes()) for a in args])
        if native_expected:
            self.assertEqual(multiply.calls, [])
            raw = spy.calls[0]
            self.assertEqual(raw, (args[0].tobytes(), args[1].tobytes(),
                                   len(args[0]), args[1].shape[1]))
            self.assertTrue(all(type(x) is bytes for x in raw[:2]))
        return actual

    def test_actual_native_default_policy(self):
        self.compare(self.sample(), True)

    def test_known_mean_covariance_units(self):
        d = np.array([[[1., 2.], [3., 4.], [999., -999.]]])
        w = np.array([[True, True, False]])
        c = np.array([2], dtype=np.int64)
        self.compare((d, w, c), True)
        a, b = HELPER._moments(d, w, c, products, NATIVE.local_moments)
        self.assertEqual(a.tobytes(), np.array([[2., 3.]]).tobytes())
        self.assertEqual(b.tobytes(), np.ones((1, 2, 2)).tobytes())
        scale = 8.
        a2, b2 = HELPER._moments(d*scale, w, c, products, NATIVE.local_moments)
        self.assertEqual(a2.tobytes(), (a*scale).tobytes())
        self.assertEqual(b2.tobytes(), (b*scale*scale).tobytes())

    def test_deterministic_reduction_shapes(self):
        for n, k in ((1,1),(2,3),(7,8),(4,9),(3,127),(2,128),(3,129),(2,257),(1,512)):
            with self.subTest(n=n,k=k):
                self.compare(self.sample(n,k), True)

    def test_signed_zero_and_cancellation(self):
        d = np.array([[[-0.,0.],[0.,-0.],[2**24,2**-40],[-2**24,-2**-40]]])
        w = np.ones((1,4),bool)
        self.compare((d,w,np.array([4],np.int64)), True)

    def test_exact_bound_and_outside_bound(self):
        d,w,c=self.sample(1,2); d[:]=2.e7
        self.compare((d,w,c),True)
        d[0,0,0]=np.nextafter(2.e7,math.inf)
        self.compare((d,w,c),False)
        d=np.array([[[2.e7,-2.e7],[-2.e7,2.e7],[2.e7,2.e7]]])
        w=np.array([[True,False,False]])
        self.compare((d,w,np.array([1],np.int64)),True,dict(all='raise',under='ignore'))

    def test_adapter_row_bound(self):
        self.compare(self.sample(3000,1),True)
        self.compare(self.sample(3001,1),False)

    def test_column_bound(self):
        self.compare(self.sample(1,2048),True)
        self.compare(self.sample(1,2049),False)

    def test_native_kernel_independent_row_bound_4000(self):
        d,w,c=self.sample(4000,1)
        means,cov=NATIVE.local_moments(d.tobytes(),w.tobytes(),4000,1)
        expected=original_np(d,w,c)
        self.assertEqual((means,cov),tuple(a.tobytes() for a in expected))
        self.compare((d,w,c),False)  # Caller bound remains 3000, not kernel 4000.

    def test_no_native_literal_fallback(self):
        self.compare(self.sample(),False,use_native=False)

    def test_readonly_inputs_and_output_bytes(self):
        args=self.sample()
        for a in args: a.flags.writeable=False
        self.compare(args,True)
        out=HELPER._moments(*args,products,NATIVE.local_moments)
        for a in out:
            self.assertFalse(a.flags.writeable)
            with self.assertRaises(ValueError): a.flags.writeable=True

    def test_zero_rows(self):
        self.compare(self.sample(0,7),False)

    def test_zero_columns(self):
        self.compare(self.sample(3,0),False)

    def test_zero_counts_ignore_and_raise(self):
        d,w,c=self.sample(); w[:]=False; c[:]=0
        self.compare((d,w,c),False,dict(all='ignore'))
        self.compare((d,w,c),False,dict(all='raise'))

    def test_wrong_counts_not_recomputed(self):
        d,w,c=self.sample(); c=c+1
        self.compare((d,w,c),False)
        c[0]=-1
        self.compare((d,w,c),False)

    def test_integer_and_float32_inputs_fallback(self):
        d,w,c=self.sample()
        for dtype in (np.float32,np.int64):
            with self.subTest(dtype=str(dtype)):
                self.compare((d.astype(dtype),w,c),False)
        self.compare((d,w,c.astype(np.int32)),False)

    def test_non_native_endian_fallback(self):
        d,w,c=self.sample()
        self.compare((d.astype('>f8'),w,c),False)
        self.compare((d,w,c.astype('>i8')),False)

    def test_noncontiguous_inputs_fallback(self):
        d,w,c=self.sample(6,34)
        self.compare((d[::2,::2],w[::2,::2],np.count_nonzero(w[::2,::2],axis=1)),False)
        self.compare((d,w,np.repeat(c,2)[::2]),False)

    def test_ndarray_subclass_fallback(self):
        class Sub(np.ndarray): pass
        d,w,c=self.sample()
        self.compare((d.view(Sub),w,c),False)

    def test_noncanonical_bool_payload_fallback(self):
        d,w,c=self.sample(1,3)
        w=np.array([[1,2,255]],dtype=np.uint8).view(np.bool_)
        self.compare((d,w,np.array([3],np.int64)),False)

    def test_unselected_nonfinite_not_hidden(self):
        d,w,c=self.sample(1,3); w[0,2]=False; c=np.count_nonzero(w,axis=1)
        for value in (math.nan,math.inf,-math.inf):
            with self.subTest(value=value):
                d[0,2,0]=value
                self.compare((d,w,c),False,dict(all='ignore'))
                self.compare((d,w,c),False,dict(all='raise'))

    def test_all_strict_uses_original_first_error(self):
        d,w,c=self.sample(1,3); d[0,2,0]=math.nan; w[0,2]=False; c[:]=0
        self.compare((d,w,c),False,dict(all='raise'))

    def test_strict_underflow_original_error_and_warning(self):
        d,w,c=self.sample(1,3); d[:]=[[[1e-160,-1e-160],[0.,0.],[-1e-160,1e-160]]]
        for policy in ('raise','warn','ignore'):
            with self.subTest(policy=policy):
                self.compare((d,w,c),policy=='ignore',dict(all='ignore',under=policy))

    def test_strict_nonunderflow_native_eligible(self):
        self.compare(self.sample(),True,dict(all='raise',under='ignore'))

    def test_zero_and_tiny_subnormal_native(self):
        tiny=float.fromhex('0x0.0000000000001p-1022')
        d=np.array([[[tiny,-tiny],[0.,-0.],[-tiny,tiny]]]); w=np.ones((1,3),bool)
        self.compare((d,w,np.array([3],np.int64)),True,dict(all='ignore'))

    def test_shape_errors_keep_original_order(self):
        d,w,c=self.sample()
        for args in ((d,w[:,:-1],c),(d,w,c[:-1]),(d[:,:,:1],w,c)):
            with self.subTest(shapes=[a.shape for a in args]): self.compare(args,False)

    def test_no_unit_normal_or_coordinate_normalization(self):
        d,w,c=self.sample(); d*=1024.
        self.compare((d,w,c),True)

    def test_native_exception_never_quiet_fallback(self):
        def fail(*args): raise RuntimeError('native deliberate failure')
        multiply=Spy(products)
        with self.assertRaisesRegex(RuntimeError,'native deliberate failure'):
            HELPER._moments(*self.sample(),multiply,fail)
        self.assertEqual(multiply.calls,[])

    def test_bad_native_output_never_quiet_fallback(self):
        multiply=Spy(products)
        with self.assertRaises(ValueError):
            HELPER._moments(*self.sample(),multiply,lambda *args:(b'x',b'y'))
        self.assertEqual(multiply.calls,[])

    def test_native_results_unchanged_after_second_call(self):
        args=self.sample(); first=HELPER._moments(*args,products,NATIVE.local_moments)
        saved=tuple(a.tobytes() for a in first)
        HELPER._moments(*self.sample(2,9),products,NATIVE.local_moments)
        self.assertEqual(saved,tuple(a.tobytes() for a in first))


if __name__ == '__main__':
    unittest.main()
