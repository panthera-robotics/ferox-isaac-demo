"""A successful Kit exit must not manufacture successful robotics evidence."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_isolated_isaac import validate_receipt


class ReceiptTests(unittest.TestCase):
    def test_artifacts_are_required_and_hashed(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / 'probe.json').write_text(json.dumps({'status': 'PASS'}))
            with self.assertRaises(ValueError):
                validate_receipt(p)
            (p / 'probe.json').write_text(json.dumps({'status': 'PASS', 'artifacts': ['metrics.json']}))
            with self.assertRaises(ValueError):
                validate_receipt(p)
            (p / 'metrics.json').write_text('{}')
            result = validate_receipt(p)
            self.assertEqual(len(result['artifact_sha256']['metrics.json']), 64)

    def test_escaped_and_empty_artifacts_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / 'empty').touch()
            (p / 'escape').symlink_to('/etc/os-release')
            for path in ['empty', 'escape', '../outside', '/etc/os-release']:
                (p / 'probe.json').write_text(json.dumps({'status': 'PASS', 'artifacts': [path]}))
                with self.assertRaises(ValueError):
                    validate_receipt(p)


if __name__ == '__main__':
    unittest.main()
