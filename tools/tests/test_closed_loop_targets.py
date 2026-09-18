"""CPU tests for the closed-loop target advancement (sprint L C1).

Regression matrix: interpolation ON/OFF x hand owner MODEL/SCRIPTED; first row; middle of the prefix; chunk boundary;
tail hold; abort mid-row; hand-only change; unequal L/R commands; missing/foreign/NaN values; duplicate/stale rows;
no silent overwrite of a model-owned hand target. No simulator, no model.
"""
import math
import unittest
from isaac.twin.inspire.closed_loop_targets import ClosedLoopTargets, TargetError

ARMS = ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint']
HANDS = ['left_index_1_joint', 'right_index_1_joint', 'right_thumb_1_joint']


def decoder(side, vals):
    # dataset-order [index, ..., thumb] -> named joints (a stand-in for HandCommandAdapter.to_joint_targets)
    out = {'%s_index_1_joint' % side: float(vals[0])}
    if side == 'right':
        out['right_thumb_1_joint'] = float(vals[1])
    return out


def make(interpolate=True, owner='MODEL', max_step=None, ticks=4):
    return ClosedLoopTargets(arm_names=ARMS, hand_names=HANDS, initial_arm={a: 0.0 for a in ARMS}, initial_hand={h: 0.0 for h in HANDS},
                             model_ticks=ticks, max_step_rad=max_step, interpolate=interpolate, hand_owner=owner,
                             hand_decoder=decoder if owner == 'MODEL' else None)


def row(left=0.1, right=-0.2, hands=None):
    return {'body_q_rad': {ARMS[0]: left, ARMS[1]: right, 'waist_yaw_joint': 0.3}, 'hands': hands if hands is not None else {'left': [0.6], 'right': [0.6, 0.25]}}


class HandOwnership(unittest.TestCase):
    def test_model_hands_applied_with_interpolation_on_and_off(self):
        for interp in (True, False):
            t = make(interpolate=interp)
            rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row())
            self.assertEqual(rec['source'], 'model_chunk'); self.assertEqual(rec['hand_owner'], 'MODEL')
            self.assertAlmostEqual(rec['hand_targets_rad']['right_index_1_joint'], 0.6)
            self.assertAlmostEqual(t.hand['right_index_1_joint'], 0.6, msg='interpolate=%s' % interp)
            self.assertAlmostEqual(t.hand['right_thumb_1_joint'], 0.25)
            applied = t.advance(tick=0, physics_s=0.005)
            self.assertAlmostEqual(applied['hand_targets_rad']['left_index_1_joint'], 0.6)

    def test_scripted_owner_ignores_model_hands_and_records_raw(self):
        t = make(owner='SCRIPTED')
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row())
        self.assertIsNone(rec['hand_targets_rad']); self.assertEqual(rec['model_hand_raw'], {'left': [0.6], 'right': [0.6, 0.25]})
        self.assertEqual(t.hand['right_index_1_joint'], 0.0)
        t.set_scripted_hand({'right_index_1_joint': 0.9}); self.assertEqual(t.hand['right_index_1_joint'], 0.9)
        t.begin_row(tick=4, physics_s=0.02, iteration=1, chunk_pos=1, **row())
        self.assertEqual(t.hand['right_index_1_joint'], 0.9, 'model row must not touch scripted hands')

    def test_scripted_write_refused_when_model_owns_hands(self):
        t = make(owner='MODEL')
        with self.assertRaises(TargetError):
            t.set_scripted_hand({'right_index_1_joint': 0.9})

    def test_hand_only_change_and_unequal_left_right(self):
        t = make()
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands={'left': [0.1], 'right': [0.7, 0.0]}))
        self.assertAlmostEqual(t.hand['left_index_1_joint'], 0.1); self.assertAlmostEqual(t.hand['right_index_1_joint'], 0.7)
        t.begin_row(tick=4, physics_s=0.02, iteration=1, chunk_pos=1, **row(hands={'left': [0.1], 'right': [0.8, 0.0]}))   # arms unchanged, hand only
        self.assertAlmostEqual(t.hand['right_index_1_joint'], 0.8); self.assertAlmostEqual(t.arm[ARMS[0]], 0.0)
        t.advance(tick=7, physics_s=0.04); self.assertAlmostEqual(t.arm[ARMS[0]], 0.1)


class ArmAdvancement(unittest.TestCase):
    def test_interpolation_over_model_ticks_then_stepped_when_off(self):
        t = make(interpolate=True)
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(left=0.4, right=-0.4))
        vals = [t.advance(tick=k, physics_s=k * 0.005)['arm_targets_rad'][ARMS[0]] for k in range(4)]
        self.assertEqual([round(v, 6) for v in vals], [0.1, 0.2, 0.3, 0.4])
        s = make(interpolate=False)
        s.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(left=0.4, right=-0.4))
        self.assertEqual(s.advance(tick=0, physics_s=0.0)['arm_targets_rad'][ARMS[0]], 0.4)

    def test_only_declared_axes_interpolate(self):
        t = ClosedLoopTargets(arm_names=ARMS, hand_names=HANDS, initial_arm={a: 0.0 for a in ARMS}, initial_hand={h: 0.0 for h in HANDS},
                              model_ticks=4, interpolate=True, hand_owner='SCRIPTED', interpolation_axes=[ARMS[0]])
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(left=0.4, right=-0.4))
        a = t.advance(tick=0, physics_s=0.0)['arm_targets_rad']
        self.assertAlmostEqual(a[ARMS[0]], 0.1); self.assertAlmostEqual(a[ARMS[1]], -0.4)
        with self.assertRaises(TargetError):
            ClosedLoopTargets(arm_names=ARMS, hand_names=HANDS, initial_arm={a: 0.0 for a in ARMS}, initial_hand={h: 0.0 for h in HANDS},
                              model_ticks=4, hand_owner='SCRIPTED', interpolation_axes=['right_index_1_joint'])

    def test_rate_limit_is_an_intervention_and_ramps_from_applied(self):
        t = make(max_step=0.01)
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(left=0.4, right=-0.4))
        self.assertEqual(len(t.interventions), 2); self.assertAlmostEqual(rec['arm_targets_rate_limited_rad'][ARMS[0]], 0.01)
        for k in range(4):
            t.advance(tick=k, physics_s=k * 0.005)
        self.assertAlmostEqual(t.arm[ARMS[0]], 0.01)
        t.begin_row(tick=4, physics_s=0.02, iteration=1, chunk_pos=1, **row(left=0.4, right=-0.4))   # middle of the prefix
        self.assertAlmostEqual(t.ramp_to[ARMS[0]], 0.02); self.assertEqual(len(t.interventions), 4)

    def test_chunk_boundary_and_tail_hold(self):
        t = make()
        for pos in range(3):
            t.begin_row(tick=pos * 4, physics_s=pos * 0.02, iteration=1, chunk_pos=pos, **row(left=0.1 * (pos + 1), right=0.0))
            for k in range(4):
                t.advance(tick=pos * 4 + k, physics_s=0.0)
        self.assertAlmostEqual(t.arm[ARMS[0]], 0.3)
        t.begin_row(tick=12, physics_s=0.06, iteration=2, chunk_pos=0, obs_id=1, **row(left=0.5, right=0.0))   # new chunk after a fresh observation
        self.assertAlmostEqual(t.ramp_from[ARMS[0]], 0.3)
        for k in range(4):
            t.advance(tick=12 + k, physics_s=0.0)
        h = t.hold(tick=16, physics_s=0.08); self.assertAlmostEqual(h['arm_targets_rad'][ARMS[0]], 0.5)
        h = t.hold(tick=40, physics_s=0.2); self.assertAlmostEqual(h['arm_targets_rad'][ARMS[0]], 0.5); self.assertEqual(t.rows, 4)

    def test_abort_mid_row_leaves_state_consistent(self):
        t = make(interpolate=True)
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(left=0.4, right=0.0))
        t.advance(tick=0, physics_s=0.0); t.advance(tick=1, physics_s=0.005)   # abort here
        self.assertAlmostEqual(t.arm[ARMS[0]], 0.2); self.assertEqual(t.applied_records, 2); self.assertEqual(t.chunk_records, 1)


class Refusals(unittest.TestCase):
    def test_missing_foreign_nan(self):
        t = make()
        with self.assertRaisesRegex(TargetError, 'misses'):
            t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, body_q_rad={ARMS[0]: 0.1}, hands={'left': [0.1], 'right': [0.1, 0.1]})
        with self.assertRaisesRegex(TargetError, 'non-finite'):
            t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, body_q_rad={ARMS[0]: float('nan'), ARMS[1]: 0.0}, hands={'left': [0.1], 'right': [0.1, 0.1]})
        with self.assertRaisesRegex(TargetError, 'non-finite hand'):
            t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, body_q_rad={ARMS[0]: 0.0, ARMS[1]: 0.0}, hands={'left': [math.inf], 'right': [0.1, 0.1]})
        with self.assertRaisesRegex(TargetError, 'no hand values'):
            t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, body_q_rad={ARMS[0]: 0.0, ARMS[1]: 0.0}, hands=None)
        bad = ClosedLoopTargets(arm_names=ARMS, hand_names=HANDS, initial_arm={a: 0.0 for a in ARMS}, initial_hand={h: 0.0 for h in HANDS},
                                model_ticks=4, hand_owner='MODEL', hand_decoder=lambda s, v: {'foreign_joint': 0.1})
        with self.assertRaisesRegex(TargetError, 'not a hand target'):
            bad.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, body_q_rad={ARMS[0]: 0.0, ARMS[1]: 0.0}, hands={'left': [0.1]})
        self.assertEqual(t.rows, 0, 'refused rows are not counted')

    def test_duplicate_row_is_idempotent_and_logged_once_each(self):
        t = make()
        r1 = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row())
        r2 = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row())   # a stale duplicate delivered again
        self.assertEqual(r1['body_targets_rad'], r2['body_targets_rad']); self.assertEqual(t.chunk_records, 2)
        self.assertEqual(t.ramp_from[ARMS[0]], 0.0, 'a duplicate before any tick advanced must not move the ramp origin')


if __name__ == '__main__':
    unittest.main()
