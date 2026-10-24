"""Portable offline module tests; no fitting, inference or Unity operations."""
import os
from pathlib import Path
import sys
import unittest

bundle = Path(__file__).resolve().parent
repo = bundle.parents[1]
core = repo / 'workspace/src/aiformula/perception/semantic_planning_system/semantic_planner_core'
helpers = bundle / 'overlay/Assets/StreamingAssets/SemanticPlanning'
tests = bundle / 'tests'
if not (core / 'semantic_planner_core/route_fusion.py').is_file():
    raise SystemExit('Run inside the full neo repository; active core package is missing')
os.environ['CALIBRATED_TEST_CORE'] = str(core)
sys.path[:0] = [str(helpers), str(core), str(tests)]
suite = unittest.TestSuite()
loader = unittest.defaultTestLoader
for name in ('test_calibrated_topology','test_continuous_detour_tracker','test_calibrated_transport'):
    suite.addTests(loader.loadTestsFromName(name))
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
