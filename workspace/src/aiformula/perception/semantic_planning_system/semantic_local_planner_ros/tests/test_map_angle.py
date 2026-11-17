"""End-to-end fake-ROS check for a graph satisfying the 80..95 rule."""
import math
import unittest

import test_map_assist as fixtures


class AngleMapNodeTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName="runTest")
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.addCleanup(lambda: self.h.node.close_fusion_worker())

    def test_recording_starts_after_visual_and_oa_ready(self):
        node = self.h.node
        node._on_avoidance(fixtures.DataMessage(True))
        self.h.odom()
        self.assertEqual(node.samples, [])
        self.assertEqual(node.reason, 'waiting_valid_mapping_inputs')

        # Visual data alone is not enough while OA owns the points.
        self.h.advance()
        self.h.triplet()
        self.h.odom()
        self.assertEqual(node.samples, [])

        node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(node.p['resume_delay_sec'] + 0.01)
        self.h.triplet()
        self.h.odom()
        self.assertEqual(len(node.samples), 1)
        self.assertFalse(node.samples[0].excluded)
        self.assertFalse(node.excluded_since_sample)

    def test_orthogonal_graph_fuses_and_binds_on_fresh_odom(self):
        node = self.h.node
        self.h.start_session()
        identifiers = ['n0','n1','n2','n3']
        points = [(15.,0.),(30.,10.),(15.,20.),(0.,10.)]
        node.official = self.h.module.OfficialRoute(identifiers+[identifiers[0]],points+[points[0]])
        nodes=dict(zip(identifiers,points));nodes['i0']=(15.,10.)
        adjacency={key:{} for key in nodes}
        def connect(a,b):
            adjacency[a][b]=adjacency[b][a]=math.dist(nodes[a],nodes[b])
        for a,b in zip(identifiers,identifiers[1:]+identifiers[:1]):connect(a,b)
        for a in identifiers:connect(a,'i0')
        node.topology=self.h.module.TopologyGraph(nodes,adjacency,[identifiers+[identifiers[0]]])
        dense=self.h.dense_loop([(0.,0.),(30.,0.),(30.,20.),(0.,20.),(0.,0.)],spacing=.3)[:-1]
        dense=dense[12:]+dense[:12]
        node.samples=[self.h.module.RouteSample(x,y,0.) for x,y in dense]
        node.recorded_distance=sum(math.dist(a,b) for a,b in zip(dense,dense[1:]+dense[:1]))
        node.left_start_area=True
        node.odom=(dense[0][0],dense[0][1],0.)
        node.odom_time=node.odom_stamp=self.h.now
        node.odom_frame='odom';node.child_frame='base_link'
        # Simulate crossing the start-normal plane in the original direction.
        previous = (node.odom[0] - 0.1, node.odom[1], node.odom[2])
        node._try_complete_lap(node.odom,False,previous)
        result=node._fusion_future.result(timeout=20.)
        self.assertTrue(result.status['available'],result.status)
        self.assertEqual(result.alignment['matching_method'],'geometric_seam_v3')
        self.assertTrue(all(80. <= item['angle_deg'] <= 95. for item in result.alignment['branch_angles']))
        self.assertFalse(result.alignment['candidate_gap_enforced'])
        self.h.advance();self.h.odom(*node.odom);node._tick()
        self.assertTrue(node.fusion_status['available'],node.fusion_status)
        self.assertTrue(node.topology_mapping_status['available'],node.topology_mapping_status)
        self.assertEqual(set(node.fused_node_positions),set(identifiers))
        self.assertIn('i0',node.mapped_topology.nodes)


if __name__ == '__main__':
    unittest.main()
