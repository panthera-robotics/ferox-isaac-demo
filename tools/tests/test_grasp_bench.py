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


class FoldTests(unittest.TestCase):
    E2B = '/home/ubuntu/panthera/sim-workspace/parallel-20260917/e2-acceptance/inbox/hand-req-Q02-e2prior-asset-r3/asset/E2_right_hand_bench.urdf'

    @unittest.skipUnless(os.path.exists(E2B), 'E2 bench URDF not on this host')
    def test_fold_e2_bench_lands_on_donor_datum(self):
        from inspire_grasp_bench import fold_massless_frames
        from urdf_kinematics import UrdfKinematics
        d = tempfile.mkdtemp(); rec = fold_massless_frames(self.E2B, os.path.join(d, 'f.urdf'))
        self.assertEqual(rec['new_root'], 'right_hand_base_link'); self.assertEqual(len(rec['removed_massless_leaves']), 3); self.assertEqual(rec['folded_root']['removed_joint_rpy'][0], 3.14159)
        f = write_wrist_fixture(os.path.join(d, 'f.urdf'), os.path.join(d, 'w.urdf'), root_link='right_hand_base_link', mount_rpy=(3.141592653589793, 0.0, 1.5707963267948966))
        T = UrdfKinematics(os.path.join(d, 'w.urdf')).transforms({})
        for link, xyz in (('right_index_proximal', (-0.0387, 0.0006, 0.1564)), ('right_pinky_proximal', (0.0259, 0.0006, 0.1536)), ('right_index_force_sensor_3', (-0.035, 0.0126, 0.2417))):
            for a, b in zip(T[link][:3, 3], xyz): self.assertAlmostEqual(float(a), b, places=3)

    E2B_R4 = '/home/ubuntu/panthera/sim-workspace/parallel-20260917/e2-acceptance/inbox/hand-req-Q02-e2prior-asset-r4/asset/E2_right_hand_bench.urdf'

    @unittest.skipUnless(os.path.exists(E2B_R4), 'E2 r4 bench URDF not on this host')
    def test_fold_e2_r4_bench_chains_and_virtual_frames(self):
        """r4: the sensor pads / palm_1,2 / palm_force_sensor / tcp / tcp_* are EMPTY links (fixed chains); the fold must remove them all,
        keep 13 bodies, and give every removed frame a virtual pose in its surviving body; the folded hand with the declared mount lands on the
        same finger datum as r3 and the virtual index sensor_3 frame on the r3 sensor_3 datum."""
        import numpy as np
        from inspire_grasp_bench import fold_massless_frames
        from urdf_kinematics import UrdfKinematics
        d = tempfile.mkdtemp(); rec = fold_massless_frames(self.E2B_R4, os.path.join(d, 'f.urdf'))
        self.assertEqual(rec['new_root'], 'right_hand_base_link'); self.assertEqual(rec['n_links'], 13); self.assertEqual(rec['n_joints'], 12)
        removed = {r['link'] for r in rec['removed_massless_leaves']}
        for n in ('right_index_force_sensor_3', 'right_palm_1', 'right_palm_2', 'right_palm_force_sensor', 'right_tcp', 'right_tcp_pinch', 'right_thumb_force_sensor_4'):
            self.assertIn(n, removed); self.assertIn(n, rec['virtual_frames'])
        self.assertEqual(rec['virtual_frames']['right_index_force_sensor_3']['body'], 'right_index_intermediate')
        self.assertEqual(rec['virtual_frames']['right_tcp_pinch']['hops'], 2); self.assertEqual(rec['virtual_frames']['right_tcp_pinch']['body'], 'right_hand_base_link')
        f = write_wrist_fixture(os.path.join(d, 'f.urdf'), os.path.join(d, 'w.urdf'), root_link='right_hand_base_link', mount_rpy=(3.141592653589793, 0.0, 1.5707963267948966))
        T = UrdfKinematics(os.path.join(d, 'w.urdf')).transforms({})
        for link, xyz in (('right_index_proximal', (-0.0387, 0.0006, 0.1564)), ('right_pinky_proximal', (0.0259, 0.0006, 0.1536))):
            for a, b in zip(T[link][:3, 3], xyz): self.assertAlmostEqual(float(a), b, places=3)
        vf = rec['virtual_frames']['right_index_force_sensor_3']; tip = T['right_index_intermediate'] @ np.asarray(vf['frame_in_body'])
        for a, b in zip(tip[:3, 3], (-0.035, 0.0126, 0.2417)): self.assertAlmostEqual(float(a), b, places=3)
        # the r4 bench carries the declared drive limits on all 12 joints (bench parity with the merged twin)
        import xml.etree.ElementTree as ET
        lim = {j.get('name'): j.find('limit') for j in ET.parse(os.path.join(d, 'f.urdf')).getroot().findall('joint') if j.get('type') == 'revolute'}
        self.assertEqual(len(lim), 12); self.assertTrue(all(float(l.get('effort')) == 10.0 and float(l.get('velocity')) == 1.0 for l in lim.values()))

    @unittest.skipUnless(os.path.exists(DONOR_BENCH), 'donor bench URDF not on this host')
    def test_fold_donor_is_noop(self):
        from inspire_grasp_bench import fold_massless_frames
        d = tempfile.mkdtemp(); rec = fold_massless_frames(DONOR_BENCH, os.path.join(d, 'f.urdf'))
        self.assertEqual(rec['removed_massless_leaves'], []); self.assertIsNone(rec['folded_root']); self.assertEqual(rec['new_root'], 'right_base_link')
