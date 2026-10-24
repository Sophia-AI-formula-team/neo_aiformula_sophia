"""Adapter contracts with actual core reference types and explicit fixture nodes.

These are policy/unit tests, not ROS, simulator motion or fresh-lap acceptance.
No model or actual recorded-route fit is executed by this suite.
"""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

HERE = Path(__file__).resolve().parent
CORE = Path(os.environ.get("CALIBRATED_TEST_CORE", (
    "E:/Mess/Projects/Programming/aiformula/workspace/semantic_stop_20261012/repo/"
    "workspace/src/aiformula/perception/semantic_planning_system/semantic_planner_core")))
sys.path.insert(0, str(CORE))
from semantic_planner_core.route_fusion import FusionResult
from calibrated_transport import (CalibratedTransportAdapter, PreparedReference, fusion_options,
                                  prepare_recorded_reference, result_snapshot)
import test_calibrated_topology as topology_fixture


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.fixture = topology_fixture.CalibrationTests(methodName="runTest")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.osm.parent
        self.config = self.root / "calibration.json"
        self.config.write_text(json.dumps(self.fixture.config), encoding="utf-8")
        self.original = FusionResult(self.fixture.reference, None,
                                     {"available": False, "reason": "alignment_rmse", "rmse_m": 3.4}, {}, {})
        self.calls = []
        def builder(samples, official, *args, **kwargs):
            self.calls.append((samples, official, args, kwargs))
            return self.original
        self.module = NS(build_fused_route=builder, DetourTracker=object())
        self.node = self.make_node()
        self.adapter = CalibratedTransportAdapter(self.module, self.config)
        self.adapter.attach(self.node)

    def make_node(self):
        params = dict(topomap_path=str(self.fixture.osm), sample_spacing_m=.25,
                      smooth_window_m=0., max_detour_gap_m=0., alignment_max_rmse_m=3.,
                      alignment_ambiguity_margin_m=.3, fusion_node_search_m=4.,
                      fusion_feature_window_m=2., fusion_shape_weight=.25,
                      fusion_max_displacement_m=.75, max_recorded_samples=20000,
                      min_lap_distance_m=40., lap_closure_radius_m=1.5, lap_closure_heading_deg=25.)
        official = NS(node_ids=self.fixture.config["outer_node_ids"],
                      points=[self.fixture.nodes[k] for k in self.fixture.config["outer_node_ids"]],
                      chord_length_m=40.)
        return NS(p=params, official=official, reference=None, fusion_status={"available": False},
                  odom=None, samples=[], points={"a": None, "b": None, "c": None},
                  progress=0., pending_distance=0., matched=False, path_count=0,
                  _publish_path=lambda: None)

    def build(self):
        return self.module.build_fused_route([], self.node.official, **fusion_options(self.node.p))

    def test_original_called_once_reference_identity_and_original_failure_preserved(self):
        before = copy.deepcopy(self.original.reference.__dict__)
        result = self.build()
        self.assertEqual(len(self.calls), 1)
        self.assertIs(result.reference, self.original.reference)
        self.assertEqual(before, result.reference.__dict__)
        self.assertTrue(result.status["available"])
        self.assertEqual(result.status["mode"], "calibrated_map")
        self.assertEqual(result.status["metric_source"], "explicit_simulation_calibration")
        self.assertEqual(result.status["reference_mode"], "real_first_lap")
        self.assertEqual(result.status["original_auto_result"]["status"], self.original.status)
        self.assertEqual(self.adapter.original_auto_result, result_snapshot(self.original))
        self.assertEqual(set(result.alignment), {"mode", "direction", "automatic_estimate"})
        self.assertIs(result.alignment["automatic_estimate"], False)

    def test_prepare_supplies_real_calibrated_graph_without_similarity(self):
        result = self.build()
        self.node.reference, self.node.fusion_status = result.reference, result.status
        self.node._prepare_mapped_topology()
        self.assertIsNone(self.node.official_to_odom)
        self.assertIs(self.node.mapped_topology, self.adapter.calibration.graph)
        self.assertTrue(self.node.topology_mapping_status["available"])

    def test_failed_calibration_keeps_original_and_cannot_map(self):
        self.adapter.configuration["osm_sha256"] = "0" * 64
        result = self.build()
        self.assertIs(result, self.original)
        self.node.reference, self.node.fusion_status = result.reference, result.status
        self.node._prepare_mapped_topology()
        self.assertIsNone(self.node.mapped_topology)
        self.assertFalse(self.adapter.status()["available"])

    def test_actual_official_ids_must_match_configuration(self):
        self.node.official.node_ids = ["different"]
        self.assertIs(self.build(), self.original)
        self.assertEqual(self.adapter.status()["reason"], "actual_official_outer_ids_mismatch")

    def test_new_chain_clears_calibration_without_stacking(self):
        result = self.build()
        self.node.reference, self.node.fusion_status = result.reference, result.status
        old = self.node
        wrapper = self.module.build_fused_route
        self.node = self.make_node()
        self.adapter.attach(self.node)
        self.assertIs(self.module.build_fused_route, wrapper)
        self.assertIsNone(self.adapter.active_reference)
        self.assertIsNone(self.adapter.calibration)
        old._prepare_mapped_topology()
        self.assertIsNone(old.mapped_topology)
        self.assertFalse(self.adapter.status()["active_in_current_chain"])

    def test_core_reset_reference_cannot_reuse_graph_or_reseed(self):
        result = self.build()
        self.node.reference, self.node.fusion_status = result.reference, result.status
        self.node.reference = None
        self.node.fusion_status = {"available": False, "reason": "odometry_reset"}
        self.node._prepare_mapped_topology()
        self.assertIsNone(self.node.mapped_topology)
        self.assertIsNone(self.node.reference)
        self.assertFalse(self.adapter.status()["active_in_current_chain"])

    def test_second_install_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "already installed"):
            CalibratedTransportAdapter(self.module, self.config)

    def test_original_builder_exception_never_replaced(self):
        def fail(*args, **kwargs):
            raise ValueError("original fault")
        self.adapter.original_builder = fail
        with self.assertRaisesRegex(ValueError, "original fault"):
            self.build()
        self.assertIsNone(self.adapter.active_reference)

    def test_snapshot_hook_is_outermost_and_runs_original_once(self):
        self.fixture.osm  # No actual sidecar I/O is needed for wrapper ordering.
        observed = []
        inner = self.module.build_fused_route
        def diagnostic(*args, **kwargs):
            result = inner(*args, **kwargs)
            observed.append(result)
            return result
        self.module.build_fused_route = diagnostic
        result = self.build()
        self.assertEqual(self.calls.__len__(), 1)
        self.assertIs(observed[0], result)
        next_node = self.make_node()
        self.adapter.attach(next_node)
        self.assertIs(self.module.build_fused_route, diagnostic)

    def test_warm_start_seeds_reference_not_pose_commands_or_learning_samples(self):
        self.adapter.prepared = PreparedReference(self.original, "a" * 64,
            dict(reference_mode="warm_start", fresh_learning_proof=False, recorded_snapshot_sha256="a" * 64))
        node = self.make_node()
        self.adapter.attach(node, seed_recorded=True)
        self.assertIs(node.reference, self.original.reference)
        self.assertEqual(node.progress, 0.)
        self.assertFalse(node.matched)
        self.assertIsNone(node.odom)
        self.assertEqual(node.samples, [])
        self.assertEqual(node.points, dict(a=None, b=None, c=None))
        self.assertEqual(node.fusion_status["mode"], "calibrated_map")
        self.assertEqual(node.fusion_status["reference_mode"], "warm_start")
        self.assertFalse(node.fusion_status["fresh_learning_proof"])
        self.assertEqual(len(self.calls), 0)

    def document(self):
        code = self.root / "map_assist.py"
        code.write_text("# fixture source\n", encoding="utf-8")
        files = {"map_assist.py": hashlib.sha256(code.read_bytes()).hexdigest(),
                 self.fixture.osm.name: hashlib.sha256(self.fixture.osm.read_bytes()).hexdigest()}
        identity = dict(source_hash=files["map_assist.py"], source_files=files,
                        source_tree_hash=hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest())
        corners = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        samples = [dict(x=a[0]+(b[0]-a[0])*i/10., y=a[1]+(b[1]-a[1])*i/10., yaw=0., excluded=False)
                   for a, b in zip(corners, corners[1:]) for i in range(10)]
        samples.append(dict(x=0., y=0., yaw=0., excluded=False))
        item = dict(schema="semantic_fusion_snapshot/v1", exact_capture=True,
            context=dict(identity, map_parameters=self.node.p, epoch=1, tick_sequence=22,
                         map_state_before=dict(reference=None, recorded_samples=40, recorded_distance_m=40.,
                                               odom=[0., 0., 0.])),
            candidate_samples=samples,
            official=dict(node_ids=self.node.official.node_ids, points=self.node.official.points, chord_length_m=40.),
            positional_options=[], keyword_options=fusion_options(self.node.p), result=result_snapshot(self.original))
        return dict(schema="semantic_route_diagnostics/v1", control_input=False,
                    exact_capture=True, error=None, snapshots=[item]), identity

    def prepare(self, document, identity):
        path = self.root / "snapshot.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        return prepare_recorded_reference(path, self.node, self.root, identity, self.adapter.original_builder)

    def test_recorded_prepare_replays_actual_samples_options_once(self):
        document, identity = self.document()
        prepared = self.prepare(document, identity)
        self.assertIs(prepared.original_result, self.original)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.calls[0][0][2].x, 2.)
        self.assertFalse(self.calls[0][0][2].excluded)
        self.assertEqual(self.calls[0][3], fusion_options(self.node.p))
        self.assertTrue(prepared.provenance["original_result_verified"])
        self.assertFalse(prepared.provenance["fresh_learning_proof"])

    def test_capture_source_options_and_sample_corruption_rejected_before_fit(self):
        mutations = [lambda d: d.update(exact_capture=False),
                     lambda d: d.update(control_input=True),
                     lambda d: d["snapshots"].append(copy.deepcopy(d["snapshots"][0])),
                     lambda d: d["snapshots"][0].update(exact_capture=False),
                     lambda d: d["snapshots"][0]["context"].update(source_tree_hash="bad"),
                     lambda d: d["snapshots"][0]["context"].update(source_hash="bad"),
                     lambda d: d["snapshots"][0]["context"].update(source_files={}),
                     lambda d: d["snapshots"][0]["official"].update(chord_length_m=41.),
                     lambda d: d["snapshots"][0]["keyword_options"].update(spacing_m=.3),
                     lambda d: d["snapshots"][0].update(positional_options=[1]),
                     lambda d: d["snapshots"][0]["candidate_samples"][0].update(excluded=1),
                     lambda d: d["snapshots"][0]["candidate_samples"][0].update(x=float("nan"))]
        for mutation in mutations:
            with self.subTest(mutation=mutations.index(mutation)):
                document, identity = self.document()
                mutation(document)
                with self.assertRaises((ValueError, KeyError)):
                    self.prepare(document, identity)
        self.assertEqual(len(self.calls), 0)

    def test_incomplete_lap_or_bad_closure_cannot_be_warm_seed(self):
        mutations = [lambda d: d["candidate_samples"].__delitem__(slice(2, None)),
                     lambda d: d["context"]["map_state_before"].update(reference={}),
                     lambda d: d["context"]["map_state_before"].update(recorded_samples=39),
                     lambda d: d["context"]["map_state_before"].update(recorded_distance_m=39.9),
                     lambda d: d["candidate_samples"][-1].update(excluded=True),
                     lambda d: d["candidate_samples"][-1].update(x=2.),
                     lambda d: d["candidate_samples"][-1].update(yaw=1.),
                     lambda d: d["context"]["map_state_before"].update(odom=[1., 0., 0.]),
                     lambda d: d["context"]["map_parameters"].update(lap_closure_radius_m=2.)]
        for mutation in mutations:
            with self.subTest(mutation=mutations.index(mutation)):
                document, identity = self.document()
                # JSON copy prevents parameter mutation from altering live p.
                document = copy.deepcopy(document)
                mutation(document["snapshots"][0])
                with self.assertRaises(ValueError):
                    self.prepare(document, identity)
        self.assertEqual(len(self.calls), 0)

    def test_changed_source_bytes_rejected_before_fit(self):
        document, identity = self.document()
        (self.root / "map_assist.py").write_text("different\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "no longer matches"):
            self.prepare(document, identity)
        self.assertEqual(len(self.calls), 0)

    def test_changed_reference_trust_geometry_or_failure_result_rejected_after_recompute(self):
        mutations = [lambda r: r["reference"]["trusted"].__setitem__(0, False),
                     lambda r: r["reference"]["points"][0].__setitem__(0, .001),
                     lambda r: r["reference"].update(length_m=39.999),
                     lambda r: r["status"].update(available=True),
                     lambda r: r.update(alignment={"direction": 1})]
        for mutation in mutations:
            with self.subTest(mutation=mutations.index(mutation)):
                document, identity = self.document()
                mutation(document["snapshots"][0]["result"])
                with self.assertRaisesRegex(ValueError, "mismatch"):
                    self.prepare(document, identity)
        self.assertEqual(len(self.calls), len(mutations))

    def test_default_worker_control_methods_and_diagnostic_body_unchanged(self):
        before = ast.parse((HERE / "semantic_worker.before.py.txt").read_text(encoding="utf-8-sig"))
        after = ast.parse((HERE / "semantic_worker.py").read_text(encoding="utf-8-sig"))
        def methods(tree):
            cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SemanticTransport")
            return {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
        old, new = methods(before), methods(after)
        for name in ("ready", "tick", "_receive_vision", "_corridor_context", "_corridor_guard",
                     "_install_ros_transport", "flush_route_diagnostic", "route_diagnostic_status"):
            if name in old:
                self.assertEqual(ast.dump(old[name]), ast.dump(new[name]), name)
        constructor = new["__init__"]
        default_args = dict(zip([a.arg for a in constructor.args.args][-len(constructor.args.defaults):],
                                constructor.args.defaults))
        for name in ("calibrated_map", "recorded_reference"):
            self.assertIsNone(default_args[name].value)


if __name__ == "__main__":
    unittest.main(verbosity=2)
