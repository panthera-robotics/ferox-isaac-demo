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
            'hand_kp_nm_rad': 1.0, 'hand_kd_nm_s_rad': 0.05, 'provenance': 'unit test', 'manifest_sha256': manifest.sha256}


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

    def test_controller_for_other_manifest_is_refused(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as tmp:
            bad = controller(m); bad['manifest_sha256'] = '0' * 64
            ctl = Path(tmp) / 'controller.json'; ctl.write_text(json.dumps(bad))
            rc = replay_commands.main(['--manifest', str(MANIFEST), '--controller', str(ctl), '--synthetic', 'hand-open-close', '--validate-only'])
            self.assertEqual(rc, 2)


if __name__ == '__main__':
    unittest.main()
