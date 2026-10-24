"""Prepared review3 additions; synthetic contracts, not executed here."""
from dataclasses import replace
import unittest
import cv2
import numpy as np
import observer
import test_observer_quantized_curve as original_tests


class AdaptiveTangentTests(unittest.TestCase):
    def fixture(self):
        value=original_tests.QuantizedCurveTests('test_clean_two_bands_can_support_only_unconfirmed_geometry')
        value.setUp()
        value.main_lane()
        value.side_pair()
        return value

    def describe_synthetic_walk(self, uv, bound=.02):
        # Pure descriptor fixture. Every entry is an explicitly supplied discrete
        # pixel; these synthetic ground coordinates are not a camera/map claim.
        ids=[int(v)*1000+int(u) for u,v in uv]
        observations={key:dict(uv=[int(u),int(v)],ground_xyz=[1+u*.01,1+v*.01,0.],
            pixel_quantization_bound_xy_m=[bound,bound]) for key,(u,v) in zip(ids,uv)}
        return observer._describe_arm(ids,0,1,[],observations,observer.ObserverParameters(),np,cv2)

    def test_original_positive_uses_more_real_support_with_provenance(self):
        value=self.fixture().observe()
        self.assertTrue(any(pair['geometry_supported_under_assumptions'] for pair in value['pair_hypotheses']))
        expanded=[]
        for arm in value['curves']:
            for sample in arm['samples']:
                if not sample['adaptive_tangent_used']:
                    continue
                expanded.append(sample)
                self.assertIn(sample['used_half_window_px'],(128.,256.))
                self.assertEqual(sample['baseline_half_window_px'],64.)
                self.assertLessEqual(sample['tangent_quantization_bound_deg'],5.)
                baseline=sample['baseline_tangent']['tangent_quantization_bound_deg']
                self.assertTrue(baseline is None or baseline>5.)
                a,b=sample['tangent_support_order_index_range']
                self.assertEqual(sample['tangent_support_endpoint_ids'],
                    [arm['ordered_pixel_ids'][a],arm['ordered_pixel_ids'][b]])
                strip=sample['straight_strip_evidence']
                self.assertTrue(strip['contiguous_observed_pixels'])
                self.assertLessEqual(strip['strip_max_residual_px'],strip['strip_quantization_bound_px'])
        self.assertTrue(expanded)
        self.assertFalse(value['junction_recognized'])
        self.assertFalse(value['confirmation_eligible'])

    def test_baseline_cap_keeps_original_failure_and_arbitrary_windows_rejected(self):
        fixture=self.fixture()
        capped=fixture.observe(parameters=replace(observer.ObserverParameters(),adaptive_tangent_max_half_window_px=64.))
        self.assertFalse(any(p['geometry_supported_under_assumptions'] for p in capped['pair_hypotheses']))
        self.assertTrue(any('quantized_parallelism' in p['failed_checks'] for p in capped['pair_hypotheses']))
        self.assertFalse(any(s['adaptive_tangent_used'] for a in capped['curves'] for s in a['samples']))
        for invalid in (96.,512.):
            value=fixture.observe(parameters=replace(observer.ObserverParameters(),adaptive_tangent_max_half_window_px=invalid))
            self.assertEqual(value['status'],'unknown')
            self.assertIn('inconsistent_parameters',value['reason'])

    def test_bend_rejects_expansion_instead_of_skipping_to_larger_window(self):
        uv=[(u,0) for u in range(100)]+[(99,v) for v in range(1,101)]
        arm=self.describe_synthetic_walk(uv)
        corner=arm['samples'][99]
        self.assertFalse(corner['adaptive_tangent_used'])
        self.assertEqual(len(corner['adaptive_attempts']),1)
        self.assertEqual(corner['adaptive_attempts'][0]['reason'],'bend_or_turnaround_not_quantized_straight')
        self.assertFalse(corner['adaptive_attempts'][0]['strip_pass'])

    def test_missing_positive_pixel_cannot_be_crossed_by_expansion(self):
        uv=[(u,0) for u in range(201) if u!=50]
        arm=self.describe_synthetic_walk(uv,bound=.5)
        before_gap=arm['samples'][49]
        self.assertFalse(before_gap['adaptive_tangent_used'])
        self.assertEqual(before_gap['adaptive_attempts'][0]['reason'],'missing_or_repeated_uv_step')
        self.assertEqual(len(before_gap['adaptive_attempts']),1)

    def test_truncated_insufficient_support_is_not_extended_or_completed(self):
        value=self.fixture().observe(parameters=replace(observer.ObserverParameters(),max_lateral_m=2.2))
        self.assertFalse(any(p['geometry_supported_under_assumptions'] for p in value['pair_hypotheses']))
        self.assertTrue(any('ground_domain_or_horizon_truncated' in a['truncation'] for a in value['curves']))
        self.assertTrue(all(not a['gap_filled'] for a in value['curves']))
        self.assertFalse(value['junction_recognized'])

    def test_adaptive_support_capacity_fails_closed(self):
        value=self.fixture().observe(parameters=replace(observer.ObserverParameters(),max_adaptive_tangent_support_checks=1))
        self.assertEqual(value['status'],'unknown')
        self.assertIn('adaptive_tangent_support_budget_exceeded',value['reason'])
        self.assertEqual(value['pair_hypotheses'],[])


if __name__=='__main__':
    unittest.main()
