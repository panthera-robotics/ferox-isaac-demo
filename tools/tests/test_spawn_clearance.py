"""CPU tests for the spawn-time hand/scene clearance check (sprint L, hand lane): pure-Python URDF FK cross-checked against
pinocchio numbers, the three REFERENCE_CHECKPOINT_FIXTURE versions (v1/v2 aborted at step 0 on the GPU, v3 ran), the rod30
diagnostic scene (ran, with a ~1 cm margin), and the packager refusing an overlapping fixture before writing a package."""
import json
import tempfile
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import EmbodimentManifest
from isaac.twin.inspire.spawn_clearance import ClearanceError, check_spawn_clearance, hand_points, link_frames, scene_boxes
from tools import replay_commands

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
URDF = Path(__file__).resolve().parents[3] / 'generated/ftp_donor/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
OBJECT = {'center_pelvis_m': [0.4, -0.02, 0.16], 'radius_m': 0.015, 'length_m': 0.12, 'mass_kg': 0.08}
FIXTURE_V1 = {'table': {'center_xy_m': [0.65, 0.0], 'size_m': [0.7, 0.6, 0.03], 'top_z_pelvis_m': 0.10}, 'object': OBJECT}                 # aborted at step 0
FIXTURE_V2 = {'table': {'center_xy_m': [0.65, 0.0], 'size_m': [0.7, 0.6, 0.03], 'top_z_pelvis_m': 0.05}, 'object': dict(OBJECT, center_pelvis_m=[0.4, -0.02, 0.125], length_m=0.15)}   # aborted at step 0
FIXTURE_V3 = {'table': {'center_xy_m': [0.4, -0.02], 'size_m': [0.1, 0.1, 0.03], 'top_z_pelvis_m': 0.10}, 'object': OBJECT}                # ran
ROD30 = {'table': {'center_xy_m': [0.87, 0.0], 'size_m': [0.8, 0.6, 0.03], 'top_z_pelvis_m': 0.19}, 'object': {'center_pelvis_m': [0.57, -0.06, 0.22], 'radius_m': 0.015, 'length_m': 0.06}}   # ran (sprint J/K)


@unittest.skipUnless(URDF.exists(), 'donor URDF not available in this checkout')
class Geometry(unittest.TestCase):
    def test_fk_matches_pinocchio_at_the_zero_pose(self):
        f = link_frames(URDF, {})
        for link, expected in (('right_wrist_yaw_link', (0.1998, -0.1487, 0.0952)), ('right_base_link', (0.2413, -0.1486, 0.0952)), ('left_wrist_yaw_link', (0.1998, 0.1487, 0.0952)),
                               ('right_index_force_sensor_3', (0.4830, -0.1360, 0.1302)), ('right_thumb_force_sensor_4', (0.3666, -0.1037, 0.2113))):
            for i in range(3):
                self.assertAlmostEqual(f[link][1][i], expected[i], 3, link)                      # pinocchio (driver-cpu-venv) at pin.neutral, hand lane cross-check
        # a bent arm moves the hand; mimic children follow their driver
        f2 = link_frames(URDF, {'right_shoulder_pitch_joint': -0.5, 'right_index_1_joint': 1.0})
        self.assertGreater(f2['right_wrist_yaw_link'][1][2], f['right_wrist_yaw_link'][1][2])
        self.assertNotEqual(tuple(f2['right_index_2'][1]), tuple(f['right_index_2'][1]))
        with self.assertRaises(ClearanceError):
            link_frames(URDF, {'right_index_1_joint': float('nan')})

    def test_scene_boxes(self):
        b = {n: (lo, hi) for n, lo, hi in scene_boxes(FIXTURE_V1)}
        self.assertEqual([round(v, 3) for v in b['table'][0]], [0.3, -0.3, 0.07]); self.assertEqual([round(v, 3) for v in b['table'][1]], [1.0, 0.3, 0.1])
        self.assertEqual([round(v, 3) for v in b['object'][0]], [0.385, -0.035, 0.1]); self.assertEqual([round(v, 3) for v in b['object'][1]], [0.415, -0.005, 0.22])

    def test_fixtures_that_exploded_are_refused_and_the_ones_that_ran_are_clear(self):
        r1 = check_spawn_clearance(URDF, FIXTURE_V1, {}, {'urdf_open': {}})
        self.assertEqual(r1['status'], 'OVERLAP')
        self.assertTrue({o['link'] for o in r1['overlaps']} >= {'left_little_1', 'right_little_1', 'left_ring_1', 'right_ring_1'})
        self.assertTrue(all(o['box'] == 'table' for o in r1['overlaps']))
        r2 = check_spawn_clearance(URDF, FIXTURE_V2, {}, {'urdf_open': {}})
        self.assertEqual(r2['status'], 'OVERLAP'); self.assertIn('left_little_force_sensor_3', {o['link'] for o in r2['overlaps']})
        r3 = check_spawn_clearance(URDF, FIXTURE_V3, {}, {'urdf_open': {}})
        self.assertEqual(r3['status'], 'CLEAR'); self.assertGreater(r3['min_margin_m'], 0.04)
        rod = check_spawn_clearance(URDF, ROD30, {}, {'urdf_open': {}, 'closed': {'right_index_1_joint': 1.4, 'right_middle_1_joint': 1.4, 'right_ring_1_joint': 1.4, 'right_little_1_joint': 1.4}})
        self.assertEqual(rod['status'], 'CLEAR'); self.assertGreater(rod['min_margin_m'], 0.005); self.assertLess(rod['min_margin_m'], 0.02)   # ran, but with ~1 cm to spare
        self.assertEqual(rod['closest']['box'], 'table')
        # a raised arm clears v1: the check depends on the spawn pose, not only on the scene
        lifted = check_spawn_clearance(URDF, FIXTURE_V1, {'left_shoulder_pitch_joint': -1.2, 'right_shoulder_pitch_joint': -1.2}, {'urdf_open': {}})
        self.assertEqual(lifted['status'], 'CLEAR')

    def test_hand_points_cover_both_hands(self):
        for side in ('left', 'right'):
            pts = hand_points(URDF, side, {})
            self.assertGreaterEqual(len(pts), 20); self.assertIn(side + '_base_link', pts)
            self.assertTrue(all(k.startswith(side + '_') for k in pts))


@unittest.skipUnless(URDF.exists(), 'donor URDF not available in this checkout')
class Packager(unittest.TestCase):
    def controller(self, m):
        names = m.body_names
        return {'body_home_rad': {n: 0.0 for n in names}, 'body_kp_nm_rad': {n: 50.0 for n in names}, 'body_kd_nm_s_rad': {n: 2.0 for n in names},
                'hand_kp_nm_rad': 1.0, 'hand_kd_nm_s_rad': 0.05, 'provenance': 'unit test fixture gains (not hardware values)', 'manifest_sha256': m.sha256}

    def test_overlapping_fixture_is_refused_before_any_package_is_written(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as td:
            td = Path(td); ctl = td / 'controller.json'; ctl.write_text(json.dumps(self.controller(m)))
            v1 = td / 'v1.json'; v1.write_text(json.dumps(FIXTURE_V1)); v3 = td / 'v3.json'; v3.write_text(json.dumps(FIXTURE_V3))
            base = ['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--source-urdf', str(URDF)]
            rc = replay_commands.main(base + ['--scene', str(v1), '--out', str(td / 'pkg_v1')])
            self.assertEqual(rc, 2); self.assertFalse((td / 'pkg_v1').exists(), 'no package may be written for a refused fixture')
            rc = replay_commands.main(base + ['--scene', str(v3), '--out', str(td / 'pkg_v3')])
            self.assertEqual(rc, 0)
            cfg = json.loads((td / 'pkg_v3' / 'probe-config.json').read_text())
            self.assertEqual(cfg['spawn_clearance']['status'], 'CLEAR'); self.assertGreater(cfg['spawn_clearance']['min_margin_m'], 0.04)
            self.assertEqual(json.loads((td / 'pkg_v3' / 'package.json').read_text())['validation']['spawn_clearance']['status'], 'CLEAR')
            # without the mounted URDF the pose cannot be computed: recorded as NOT_CHECKED, never as CLEAR
            rc = replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--scene', str(v1), '--out', str(td / 'pkg_nourdf')])
            self.assertEqual(rc, 0)
            self.assertEqual(json.loads((td / 'pkg_nourdf' / 'probe-config.json').read_text())['spawn_clearance']['status'], 'NOT_CHECKED')


if __name__ == '__main__':
    unittest.main()
