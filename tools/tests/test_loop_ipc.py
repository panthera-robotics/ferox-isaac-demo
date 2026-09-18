"""CPU tests for the probe<->sidecar loop IPC: stale/future/duplicate/foreign/malformed answers are refused before motion."""
import json, tempfile, threading, time, unittest
from pathlib import Path
from isaac.twin.inspire import loop_ipc as ipc
from isaac.twin.inspire.model_action_adapter import PISTON_DIMS

TOK = 'run-abc'


def good_action(h=3):
    return {k: [[0.1] * w for _ in range(h)] for k, w in PISTON_DIMS.items()}


class LoopIPC(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = ipc.layout(Path(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def test_round_trip_and_pending_listing(self):
        ipc.write_observation(self.root, 0, TOK, {'state': {'x': 1}}, image_bytes=b'png')
        self.assertEqual(ipc.pending_observations(self.root, set()), [0])
        ipc.write_action(self.root, 0, TOK, {'status': 'ok', 'action': good_action()})
        act = ipc.wait_for_action(self.root, 0, run_token=TOK, dims=PISTON_DIMS, timeout_s=1.0)
        self.assertEqual(len(act['right_arm']), 3); self.assertEqual(ipc.pending_observations(self.root, set()), [])

    def test_stale_future_duplicate_and_foreign_refused(self):
        ipc.write_observation(self.root, 5, TOK, {})
        ipc.write_action(self.root, 5, TOK, {'status': 'ok', 'action': good_action(), 'obs_id': 4})   # write_action overrides obs_id -> 5; simulate a stale file by hand
        stale = json.loads((self.root / 'act' / '000005.json').read_text()); stale['obs_id'] = 4
        (self.root / 'act' / '000005.json').write_text(json.dumps(stale))
        with self.assertRaisesRegex(ipc.LoopIPCError, 'stale, future or duplicate'):
            ipc.wait_for_action(self.root, 5, run_token=TOK, dims=PISTON_DIMS, timeout_s=0.5)
        foreign = dict(stale, obs_id=5, run_token='other-run'); (self.root / 'act' / '000005.json').write_text(json.dumps(foreign))
        with self.assertRaisesRegex(ipc.LoopIPCError, 'another run'):
            ipc.wait_for_action(self.root, 5, run_token=TOK, dims=PISTON_DIMS, timeout_s=0.5)

    def test_malformed_nan_wrong_width_and_missing_keys_refused(self):
        bad = good_action(); bad['right_arm'][0][2] = float('nan')
        with self.assertRaisesRegex(ipc.LoopIPCError, 'finite'):
            ipc.validate_action_message({'run_token': TOK, 'obs_id': 1, 'status': 'ok', 'action': bad}, expected_obs_id=1, run_token=TOK, dims=PISTON_DIMS)
        bad = good_action(); bad['right_hand'] = [[0.1] * 5]
        with self.assertRaisesRegex(ipc.LoopIPCError, 'wide matrix|horizon'):
            ipc.validate_action_message({'run_token': TOK, 'obs_id': 1, 'status': 'ok', 'action': bad}, expected_obs_id=1, run_token=TOK, dims=PISTON_DIMS)
        bad = good_action(); del bad['base_height']
        with self.assertRaisesRegex(ipc.LoopIPCError, 'keys'):
            ipc.validate_action_message({'run_token': TOK, 'obs_id': 1, 'status': 'ok', 'action': bad}, expected_obs_id=1, run_token=TOK, dims=PISTON_DIMS)
        (self.root / 'act' / '000002.json').write_text('{not json')
        with self.assertRaisesRegex(ipc.LoopIPCError, 'malformed'):
            ipc.wait_for_action(self.root, 2, run_token=TOK, dims=PISTON_DIMS, timeout_s=0.5)

    def test_timeout_crash_and_error_status(self):
        t0 = time.monotonic()
        with self.assertRaisesRegex(ipc.LoopIPCError, 'timeout'):
            ipc.wait_for_action(self.root, 7, run_token=TOK, dims=PISTON_DIMS, timeout_s=0.3)
        self.assertGreaterEqual(time.monotonic() - t0, 0.3)
        with self.assertRaisesRegex(ipc.LoopIPCError, 'not alive'):
            ipc.wait_for_action(self.root, 8, run_token=TOK, dims=PISTON_DIMS, timeout_s=5.0, sidecar_alive=lambda: False)
        ipc.write_action(self.root, 9, TOK, {'status': 'error', 'error': 'inference crashed'})
        with self.assertRaisesRegex(ipc.LoopIPCError, 'inference crashed'):
            ipc.wait_for_action(self.root, 9, run_token=TOK, dims=PISTON_DIMS, timeout_s=0.5)

    def test_answer_arriving_late_is_accepted_once(self):
        def answer():
            time.sleep(0.2); ipc.write_action(self.root, 3, TOK, {'status': 'ok', 'action': good_action()})
        threading.Thread(target=answer).start()
        act = ipc.wait_for_action(self.root, 3, run_token=TOK, dims=PISTON_DIMS, timeout_s=2.0)
        self.assertIn('left_hand', act)


if __name__ == '__main__':
    unittest.main()
