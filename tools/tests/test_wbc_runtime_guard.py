"""CPU tests: each check is a runtime hazard the guard must refuse."""
import io
import json
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wbc_runtime_guard import GuardRefused, RuntimeOwnershipGuard  # noqa: E402
from test_wbc_standing_ab import BODY  # noqa: E402

HANDS = ['right_index_1_joint', 'left_thumb_2_joint']
LIMITS = {n: {'lower': -2.0, 'upper': 2.0, 'effort': 25.0, 'velocity': 30.0} for n in BODY}
LIMITS.update({n: {'lower': 0.0, 'upper': 1.4381, 'effort': 1.0, 'velocity': 10.0} for n in HANDS})


def guard(**over):
    kw = dict(body_names=BODY, hand_names=HANDS, limits=LIMITS, physics_dt=0.005, decimation=4, hand_margin_rad=0.02)
    kw.update(over)
    return RuntimeOwnershipGuard(**kw)


def run_steps(g, n, start=0):
    for s in range(start, start + n):
        g.begin_step(s, (s + 1) * 0.005)
        g.observe_state(BODY + HANDS, [0.0] * 31, [0.0] * 31)


class GuardTests(unittest.TestCase):
    def test_construction_refusals(self):
        with self.assertRaisesRegex(GuardRefused, 'finger joints in the body set'):
            guard(body_names=BODY[:28] + ['left_index_1_joint'], limits=dict(LIMITS, left_index_1_joint=LIMITS[HANDS[0]]))
        with self.assertRaisesRegex(GuardRefused, 'hand margin'):
            guard(hand_margin_rad=0.0)
        with self.assertRaisesRegex(GuardRefused, 'exactly 29'):
            guard(body_names=BODY[:28])

    def test_nominal_flow_and_journal(self):
        buf = io.StringIO()
        g = guard(journal=buf)
        g.claim_body('named_policy_single_writer'); g.claim_hands('probe_margin_hold')
        g.declare_support('RIG_WRENCH')
        g.begin_step(0, 0.005)
        g.assert_support_row({'kind': 'RIG_WRENCH', 'force_n': [0., 0., 5.], 'torque_nm': [0., 0., 0.]})
        g.admit_body_command('named_policy_single_writer', BODY, [0.0] * 29)
        g.admit_hand_command('probe_margin_hold', HANDS, [0.02, 0.02])
        g.observe_state(BODY + HANDS, [0.0] * 31, [0.0] * 31)
        g.begin_step(1, 0.010)
        g.release_support()
        g.assert_support_row({'kind': 'NONE', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]})
        g.observe_state(BODY + HANDS, [0.0] * 31, [0.0] * 31)
        entries = [json.loads(l) for l in buf.getvalue().splitlines()]
        self.assertEqual([e['kind'] for e in entries], ['body_owner', 'hand_owner', 'support', 'support_release'])
        self.assertEqual(g.release_sequence, 1)
        self.assertEqual(g.summary()['state'], 'unsupported')

    def test_handover_only_while_supported_or_idle(self):
        g = guard(); g.claim_body('a')
        g.claim_body('b')                                   # idle: hand-over is legal and journaled
        self.assertEqual([e['kind'] for e in g.entries][-2:], ['body_owner_handover', 'body_owner'])
        g.declare_support('FIXED_PELVIS'); g.claim_body('c')  # supported settle: legal
        g.begin_step(0, 0.005); g.release_support()
        with self.assertRaisesRegex(GuardRefused, 'switching while unsupported'):
            g.claim_body('d')
        self.assertEqual(g.body_owner, 'c')

    def test_wrong_owner_wrong_order_finger_names_and_nan(self):
        g = guard(); g.claim_body('a'); g.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, "from 'b' but owner is 'a'"):
            g.admit_body_command('b', BODY, [0.0] * 29)
        with self.assertRaisesRegex(GuardRefused, 'declared order'):
            g.admit_body_command('a', list(reversed(BODY)), [0.0] * 29)
        with self.assertRaisesRegex(GuardRefused, 'declared order'):
            g.admit_body_command('a', BODY[:28] + ['right_index_1_joint'], [0.0] * 29)
        with self.assertRaisesRegex(GuardRefused, 'finite'):
            g.admit_body_command('a', BODY, [float('nan')] + [0.0] * 28)

    def test_target_rate_bound_and_limits(self):
        g = guard(); g.claim_body('a'); g.begin_step(0, 0.005)
        g.admit_body_command('a', BODY, [0.0] * 29)
        with self.assertRaisesRegex(GuardRefused, 'target step'):
            g.admit_body_command('a', BODY, [0.6] + [0.0] * 28)
        g2 = guard(max_target_step_rad=5.0); g2.claim_body('a'); g2.begin_step(0, 0.005)
        g2.admit_body_command('a', BODY, [2.3] + [0.0] * 28)      # beyond the 2.0 limit: journaled, not refused
        self.assertEqual(g2.entries[-1]['kind'], 'target_beyond_limit')
        self.assertEqual(g2.summary()['steps_with_targets_beyond_limit'], 1)
        with self.assertRaisesRegex(GuardRefused, 'beyond'):
            g2.admit_body_command('a', BODY, [2.7] + [0.0] * 28)  # gross excursion (> 0.5 rad past the limit)

    def test_stale_or_out_of_order_state(self):
        g = guard(); g.claim_body('a')
        g.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, 'no state was observed'):
            g.begin_step(1, 0.010)
        g = guard(); run_steps(g, 2)
        with self.assertRaisesRegex(GuardRefused, 'non-contiguous'):
            g.begin_step(5, 0.030)
        g = guard(); run_steps(g, 1)
        with self.assertRaisesRegex(GuardRefused, 'admitted dt'):
            g.begin_step(1, 0.050)
        g = guard(); g.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, 'non-finite'):
            g.observe_state(BODY + HANDS, [float('inf')] * 31, [0.0] * 31)

    def test_hand_margin_and_hand_owner(self):
        g = guard(); g.claim_hands('h'); g.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, 'below the declared margin'):
            g.admit_hand_command('h', HANDS, [0.0, 0.02])
        with self.assertRaisesRegex(GuardRefused, "owner is 'h'"):
            g.admit_hand_command('policy', HANDS, [0.02, 0.02])
        with self.assertRaisesRegex(GuardRefused, 'exactly the independent hand joints'):
            g.admit_hand_command('h', HANDS + ['right_index_2_joint'], [0.02] * 3)

    def test_support_release_and_reattachment(self):
        g = guard(); g.claim_body('a')
        with self.assertRaisesRegex(GuardRefused, 'requires an active supported settle'):
            g.release_support()
        g.declare_support('RIG_WRENCH'); g.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, "differs from the declared"):
            g.assert_support_row({'kind': 'FIXED_PELVIS', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]})
        g.release_support()
        with self.assertRaisesRegex(GuardRefused, 'hidden support'):
            g.declare_support('RIG_WRENCH')
        with self.assertRaisesRegex(GuardRefused, 'reattached'):
            g.assert_support_row({'kind': 'RIG_WRENCH', 'force_n': [0., 0., 1.], 'torque_nm': [0., 0., 0.]})
        self.assertEqual(g.state, 'fault_damp')
        self.assertEqual(g.body_owner, 'damp')

    def test_never_supported_declaration(self):
        g = guard(); g.claim_body('probe_default_pose_warmup'); g.claim_hands('h')
        g.declare_never_supported('named_policy_single_writer')
        self.assertEqual((g.state, g.support, g.release_sequence, g.body_owner), ('unsupported', 'NONE', -1, 'named_policy_single_writer'))
        g.begin_step(0, 0.005)
        g.assert_support_row({'kind': 'NONE', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]})
        with self.assertRaisesRegex(GuardRefused, 'switching while unsupported'):
            g.claim_body('other')
        g2 = guard(); g2.begin_step(0, 0.005)
        with self.assertRaisesRegex(GuardRefused, 'precede the first step'):
            g2.declare_never_supported('a')

    def test_fault_is_terminal(self):
        g = guard(); g.claim_body('a'); g.begin_step(0, 0.005); g.fault('body_dq guard')
        with self.assertRaisesRegex(GuardRefused, 'faulted'):
            g.admit_body_command('a', BODY, [0.0] * 29)
        with self.assertRaisesRegex(GuardRefused, 'faulted'):
            g.begin_step(1, 0.010)
        self.assertEqual(g.summary()['fault_reason'], 'body_dq guard')


if __name__ == '__main__':
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    unittest.main(verbosity=1)
