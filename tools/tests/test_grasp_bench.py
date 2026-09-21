"""CPU tests for the hand-agnostic grasp bench binding (RH56E2 acceptance lane B)."""
import json, os, sys, tempfile, unittest
HERE = os.path.dirname(os.path.abspath(__file__)); TOOLS = os.path.dirname(HERE); sys.path.insert(0, TOOLS)
from inspire_grasp_bench import HandBench, DONOR_JOINTS, DONOR_UPPER  # noqa: E402
from inspire_grasp import GraspConfig  # noqa: E402
from inspire_wrist_fixture import write_wrist_fixture  # noqa: E402

E2 = {'source_urdf': 'E2_right_hand_bench.urdf', 'root_link': 'right_base', 'mount_rpy': [0, 0, 1.5707963267948966],
      'axis_joints': {'index': 'right_index_proximal_joint', 'middle': 'right_middle_proximal_joint', 'ring': 'right_ring_proximal_joint', 'little': 'right_pinky_proximal_joint',
                      'thumb_bend': 'right_thumb_proximal_pitch_joint', 'thumb_rotation': 'right_thumb_proximal_yaw_joint'},
      'axis_upper_rad': {'index': 1.4381, 'middle': 1.4381, 'ring': 1.4381, 'little': 1.4381, 'thumb_bend': 0.62, 'thumb_rotation': 1.658}, 'palm_collision': 'mesh_as_delivered'}
DONOR_BENCH = os.path.join(TOOLS, '..', '..', '..', '..', '..', 'campaign-20260913', 'generated', 'ftp_donor', 'FTP_right_hand_bench.urdf')


class BenchTests(unittest.TestCase):
    def test_donor_default_is_identity(self):
        b = HandBench.from_config({}); cfg = GraspConfig()
        self.assertTrue(b.is_donor); self.assertEqual(b.map_targets(cfg.initial_targets()), cfg.initial_targets())
        self.assertEqual(b.facts()['mapping'], 'identity')

    def test_e2_mapping_is_closure_preserving_and_renamed(self):
        b = HandBench.from_config({'hand': E2}); cfg = GraspConfig(thumb_yaw_initial_rad=1.1641, thumb_flexion_initial_rad=0.5864, four_finger_initial_rad=1.4381)
        m = b.map_targets(cfg.initial_targets())
        self.assertAlmostEqual(m['right_thumb_proximal_yaw_joint'], 1.658, places=9); self.assertAlmostEqual(m['right_thumb_proximal_pitch_joint'], 0.62, places=9)
        self.assertAlmostEqual(m['right_pinky_proximal_joint'], 1.4381, places=9); self.assertEqual(set(m), set(E2['axis_joints'].values()))
        self.assertAlmostEqual(b.map_value('right_thumb_1_joint', 0.5), 0.5 * 1.658 / 1.1641, places=9); self.assertIsNone(b.map_value('right_thumb_2_joint', None))

    def test_hand_block_validation(self):
        with self.assertRaises(ValueError): HandBench.from_config({'hand': {**E2, 'palm_collision': 'slabs'}})
        with self.assertRaises(ValueError): HandBench.from_config({'hand': {**E2, 'axis_joints': {'index': 'x'}}})

    @unittest.skipUnless(os.path.exists(DONOR_BENCH), 'donor bench URDF not on this host')
    def test_fixture_default_unchanged_and_mount_declared(self):
        d = tempfile.mkdtemp(); f0 = write_wrist_fixture(DONOR_BENCH, os.path.join(d, 'a.urdf')); f1 = write_wrist_fixture(DONOR_BENCH, os.path.join(d, 'b.urdf'), root_link='right_base_link', mount_rpy=None)
        self.assertEqual(f0['generated_sha256'], f1['generated_sha256']); self.assertIsNone(f0['mount_rpy'])
        with self.assertRaises(ValueError): write_wrist_fixture(DONOR_BENCH, os.path.join(d, 'c.urdf'), root_link='right_base')
        f2 = write_wrist_fixture(DONOR_BENCH, os.path.join(d, 'd.urdf'), mount_rpy=(0., 0., 1.5707963267948966))
        self.assertNotEqual(f2['generated_sha256'], f0['generated_sha256']); self.assertEqual(f2['mount_rpy'], [0.0, 0.0, 1.5707963267948966])
        self.assertIn('rpy="0.0 0.0 1.5707963267948966"', open(os.path.join(d, 'd.urdf')).read())


if __name__ == '__main__':
    unittest.main()


class WriterHandTests(unittest.TestCase):
    def test_donor_default_identity(self):
        from inspire_grasp_bench import WriterHand
        w = WriterHand.from_config({}); cfg = GraspConfig()
        self.assertTrue(w.is_donor); self.assertEqual(w.map_targets(cfg.initial_targets()), cfg.initial_targets()); self.assertEqual(w.facts()['mapping'], 'identity')

    def test_e2_block(self):
        from inspire_grasp_bench import WriterHand
        X2 = ((0., 1., 0., 0.), (1., 0., 0., 0.), (0., 0., -1., 0.), (0., 0., 0., 1.))
        w = WriterHand.from_config({'hand': {'source_urdf': 'g1_29dof_rev_1_0_with_inspire_hand_E2.urdf', 'importer': 'e2', 'palm_body_link': 'right_hand_base_link', 'donor_palm_in_palm_body': X2, 'axis_joints': E2['axis_joints'], 'axis_upper_rad': E2['axis_upper_rad']}})
        self.assertFalse(w.is_donor); m = w.map_targets(GraspConfig(thumb_yaw_initial_rad=1.1641).initial_targets()); self.assertAlmostEqual(m['right_thumb_proximal_yaw_joint'], 1.658, places=9)
        with self.assertRaises(ValueError): WriterHand.from_config({'hand': {'importer': 'x'}})
