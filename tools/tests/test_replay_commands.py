"""CPU tests for the replay package builder: validate-only reporting, refusals as data, hashed package output."""
import json
import tempfile
import unittest
from pathlib import Path

from tools import replay_commands
from isaac.twin.inspire.embodiment import EmbodimentManifest

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'


def controller(manifest):
    names = manifest.body_names
    return {'body_home_rad': {n: 0.0 for n in names}, 'body_kp_nm_rad': {n: 50.0 for n in names}, 'body_kd_nm_s_rad': {n: 2.0 for n in names},
            'hand_kp_nm_rad': 1.0, 'hand_kd_nm_s_rad': 0.05, 'provenance': 'unit test fixture gains (not hardware values)', 'manifest_sha256': manifest.sha256}


class ReplayCommandsTests(unittest.TestCase):
    def test_synthetic_sequence_validates_and_packages_with_hashes(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as tmp:
            ctl = Path(tmp) / 'controller.json'; ctl.write_text(json.dumps(controller(m)))
            out = Path(tmp) / 'pkg'
            rc = replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--out', str(out)])
            self.assertEqual(rc, 0)
            pkg = json.loads((out / 'package.json').read_text())
            self.assertEqual(pkg['manifest_sha256'], m.sha256); self.assertEqual(pkg['validation']['status'], 'VALID')
            self.assertEqual(set(pkg['files']), {'manifest.json', 'sequence.json', 'controller.json'})
            seq = json.loads((out / 'sequence.json').read_text())
            self.assertEqual(seq['source']['kind'], 'synthetic_test_sequence'); self.assertEqual(len(seq['rows']), 91)
            cfg = json.loads((out / 'probe-config.json').read_text()); self.assertEqual(cfg['package'], '/workspace/replay-package')
            # packages are never overwritten
            with self.assertRaises(SystemExit):
                replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--out', str(out)])

    def test_invalid_source_is_reported_not_packaged(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as tmp:
            ctl = Path(tmp) / 'controller.json'; ctl.write_text(json.dumps(controller(m)))
            spec = Path(tmp) / 'spec.json'
            spec.write_text(json.dumps({'source': {'source_id': 'x', 'kind': 'real_recording', 'provenance': 'p'}, 'hand_contracts': {'right': replay_commands.SYNTHETIC_CONTRACT},
                                        'rows': [{'t_s': 0.0, 'body_q_rad': None, 'hands': {'right': [0.1, 0.1, 0.1, 0.1, 0.1, 1.7]}}]}))
            rc = replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--source-spec', str(spec), '--out', str(Path(tmp) / 'pkg')])
            self.assertEqual(rc, 2); self.assertFalse((Path(tmp) / 'pkg').exists())

    def test_malformed_gains_refused_before_admission(self):
        m = EmbodimentManifest.load(MANIFEST)
        base = controller(m)
        for mutate in (lambda c: c['body_kp_nm_rad'].__setitem__('right_elbow_joint', float('nan')), lambda c: c['body_kp_nm_rad'].__setitem__('left_knee_joint', -5.0),
                       lambda c: c['body_kd_nm_s_rad'].__setitem__('waist_yaw_joint', float('inf')), lambda c: c['body_kp_nm_rad'].__setitem__('waist_yaw_joint', 0.0),
                       lambda c: c.__setitem__('hand_kp_nm_rad', 0.0), lambda c: c.__setitem__('hand_kd_nm_s_rad', -0.1), lambda c: c.__setitem__('hand_kp_nm_rad', 'strong'),
                       lambda c: c['body_kp_nm_rad'].pop('right_elbow_joint'), lambda c: c['body_kp_nm_rad'].__setitem__('right_elbow_joint', True)):
            c = json.loads(json.dumps(base)); mutate(c)
            problems = replay_commands.validate_controller_gains(m, c)
            self.assertTrue(problems, 'mutation accepted: %r' % problems)
        self.assertEqual(replay_commands.validate_controller_gains(m, base), [])
        c = json.loads(json.dumps(base)); c['body_kp_nm_rad']['waist_yaw_joint'] = 0.0; c['allow_zero_stiffness_joints'] = ['waist_yaw_joint']
        self.assertEqual(replay_commands.validate_controller_gains(m, c), [])   # zero only where the declared mode permits it

    def test_incompatible_asset_profile_is_refused_at_admission(self):
        m = EmbodimentManifest.load(MANIFEST)
        live = {'urdf_sha256': 'other-asset', 'coupling_map_sha256': 'x', 'collision_cooking': 'x', 'wrist_mount_sha256': 'x', 'physics_dt_s': '0.005', 'solver': 'TGS_32_8', 'support': 'FIXED_PELVIS'}
        seq, report = replay_commands.validate(m, replay_commands.synthetic_hand_open_close(controller(m)['body_home_rad']), controller(m), live_dependencies=live)
        self.assertIsNone(seq); self.assertEqual(report['status'], 'REJECTED')
        self.assertTrue(any('incompatible profile' in p for p in report['problems']))
        self.assertEqual(report['qualification_validity']['transforms']['right.wrist_to_hand']['status'], 'INVALID')

    def test_unverified_hand_semantics_refused_on_qualified_route(self):
        m = EmbodimentManifest.load(MANIFEST)
        spec = replay_commands.synthetic_hand_open_close(controller(m)['body_home_rad'])
        seq, report = replay_commands.validate(m, spec, controller(m), require_verified_hand_semantics=True)
        self.assertIsNone(seq); self.assertTrue(any('axis_semantics' in p for p in report['problems']))
        spec['hand_contracts']['right']['axis_semantics'] = {a: {'direction': 'VERIFIED', 'order': 'VERIFIED', 'scale': 'VERIFIED'} for a in spec['hand_contracts']['right']['axis_order']}
        spec['hand_contracts']['right']['axis_semantics']['thumb_rotation']['scale'] = 'UNRESOLVED'
        seq, report = replay_commands.validate(m, spec, controller(m), require_verified_hand_semantics=True)
        self.assertIsNone(seq); self.assertTrue(any('thumb_rotation' in p for p in report['problems']))
        spec['hand_contracts']['right']['axis_semantics']['thumb_rotation']['scale'] = 'VERIFIED'
        seq, report = replay_commands.validate(m, spec, controller(m), require_verified_hand_semantics=True)
        self.assertIsNotNone(seq)
        seq, report = replay_commands.validate(m, replay_commands.synthetic_hand_open_close(controller(m)['body_home_rad']), controller(m))   # exploratory route unchanged
        self.assertIsNotNone(seq)

    def test_controller_for_other_manifest_is_refused(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as tmp:
            bad = controller(m); bad['manifest_sha256'] = '0' * 64
            ctl = Path(tmp) / 'controller.json'; ctl.write_text(json.dumps(bad))
            rc = replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--validate-only'])
            self.assertEqual(rc, 2)


if __name__ == '__main__':
    unittest.main()
