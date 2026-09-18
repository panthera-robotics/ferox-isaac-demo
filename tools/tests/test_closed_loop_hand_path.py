"""CPU tests of the model-hand command path through the REAL conversion adapter (sprint L H3, hand lane): the probe's
radian contract and decoder are reproduced verbatim from tools/probes/command_replay.py (closed loop, C1 1ac1295) and driven
through ClosedLoopTargets exactly as the probe drives it. No simulator, no model.

Covered: one model hand command reaches its named actuator with arm interpolation ON and OFF, at the first tick of its row;
left and right stay distinct (no cross-talk); coupled joints are never commanded (mimic children never appear, a decoder that
names one is refused); hybrid mode keeps the hand ownership scripted; the logs have the row/tick cadence and carry the
requested, mapped and applied values across the first observation, a chunk boundary and the tail hold. Two remaining gaps
are stated as expected failures with the one-line fixes (coordinator-owned runtime).
"""
import unittest
from pathlib import Path

from isaac.twin.inspire.closed_loop_targets import ClosedLoopTargets, TargetError
from isaac.twin.inspire.embodiment import HAND_ACTUATORS, EmbodimentManifest, HandCommandAdapter
from isaac.twin.inspire.model_action_adapter import PISTON_HAND_ORDER

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
ARMS = ['right_shoulder_pitch_joint', 'right_elbow_joint']
MODEL_TICKS = 4


class Probe:
    """The probe's hand-side objects, built exactly as tools/probes/command_replay.py builds them."""

    def __init__(self):
        self.manifest = EmbodimentManifest.load(MANIFEST)
        self.hand_names = list(self.manifest.hand_joint_names('left')) + list(self.manifest.hand_joint_names('right'))
        limits = {a: self.manifest.hand_actuator('right', a)['closed_rad'] for a in HAND_ACTUATORS}
        self.radian_contract = {'axis_order': list(PISTON_HAND_ORDER), 'open_value': 0.0, 'closed_value': 1.0,
                                'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': limits[a]} for a in HAND_ACTUATORS}, 'saturation_policy': 'clip_declared'}
        self.adapters = {sd: HandCommandAdapter(self.manifest, sd, self.radian_contract) for sd in ('left', 'right')}
        self.decoder = (lambda sd_, vals_: self.adapters[sd_].to_joint_targets(vals_))              # verbatim probe lambda (targets, info)
        self.closure_contract = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}
        self.hand_script = HandCommandAdapter(self.manifest, 'right', self.closure_contract)
        self.mimic = [n for sd in ('left', 'right') for n in self.manifest.data['hands'][sd]['coupled_joints']]

    def targets(self, *, interpolate=True, owner='MODEL', initial_hand=None):
        return ClosedLoopTargets(arm_names=ARMS, hand_names=self.hand_names, initial_arm={a: 0.0 for a in ARMS},
                                 initial_hand=initial_hand or {h: 0.0 for h in self.hand_names}, model_ticks=MODEL_TICKS, max_step_rad=None,
                                 interpolate=interpolate, hand_owner=owner, hand_decoder=(None if owner == 'SCRIPTED' else self.decoder))


def hands(right=(0, 0, 0, 0, 0, 0), left=(0, 0, 0, 0, 0, 0)):
    return {'left': [float(v) for v in left], 'right': [float(v) for v in right]}


def row(hands_, arm=0.1):
    return {'body_q_rad': {ARMS[0]: arm, ARMS[1]: -arm, 'waist_yaw_joint': 0.0}, 'hands': hands_}


class HandCommandPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = Probe()

    def test_one_model_hand_command_reaches_its_named_actuator_with_interpolation_on_and_off(self):
        for interp in (True, False):
            t = self.p.targets(interpolate=interp)
            rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(0, 0, 0, 1.3, 0, 0))))   # dataset order: index is axis 3
            applied = t.advance(tick=0, physics_s=0.0)
            self.assertAlmostEqual(applied['hand_targets_rad']['right_index_1_joint'], 1.3, 9, 'interpolate=%s' % interp)   # radian identity, reaches at the row's first tick
            self.assertAlmostEqual(rec['hand_targets_rad']['right_index_1_joint'], 1.3, 9)
            for n in self.p.hand_names:
                if n != 'right_index_1_joint':
                    self.assertEqual(applied['hand_targets_rad'][n], 0.0, n)                                                # nothing else moved
            self.assertAlmostEqual(applied['arm_targets_rad'][ARMS[0]], 0.1 / MODEL_TICKS if interp else 0.1, 9)             # the arm ramps or steps; the hand never does

    def test_left_and_right_remain_distinct(self):
        t = self.p.targets()
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(0.2, 0.3, 0.4, 0.5, 0.1, 0.6), left=(0.9, 0.8, 0.7, 0.6, 0.2, 0.3))))
        h = t.advance(tick=0, physics_s=0.0)['hand_targets_rad']
        self.assertAlmostEqual(h['right_little_1_joint'], 0.2); self.assertAlmostEqual(h['right_thumb_2_joint'], 0.1); self.assertAlmostEqual(h['right_thumb_1_joint'], 0.6)
        self.assertAlmostEqual(h['left_little_1_joint'], 0.9); self.assertAlmostEqual(h['left_index_1_joint'], 0.6); self.assertAlmostEqual(h['left_thumb_1_joint'], 0.3)
        # a right-only change leaves every left joint where it was
        t.begin_row(tick=4, physics_s=0.02, iteration=1, chunk_pos=1, **row(hands(right=(1.0, 1.0, 1.0, 1.0, 0.5, 1.0), left=(0.9, 0.8, 0.7, 0.6, 0.2, 0.3))))
        h2 = t.advance(tick=4, physics_s=0.02)['hand_targets_rad']
        for n in self.p.manifest.hand_joint_names('left'):
            self.assertEqual(h2[n], h[n], n)
        self.assertAlmostEqual(h2['right_thumb_2_joint'], 0.5); self.assertAlmostEqual(h2['right_index_1_joint'], 1.0)

    def test_coupled_joints_are_never_commanded(self):
        t = self.p.targets()
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(1.3, 1.3, 1.3, 1.3, 0.5, 1.0), left=(1.0, 1.0, 1.0, 1.0, 0.4, 0.9))))
        commanded = set(rec['hand_targets_rad']) | set(t.advance(tick=0, physics_s=0.0)['hand_targets_rad'])
        self.assertEqual(commanded, set(self.p.hand_names))                                        # the twelve independent actuators, nothing else
        self.assertFalse(commanded & set(self.p.mimic), 'a coupled joint was commanded')
        for n in self.p.mimic:
            self.assertTrue(n.endswith(('_2_joint', 'thumb_3_joint', 'thumb_4_joint')), n)
        bad = ClosedLoopTargets(arm_names=ARMS, hand_names=self.p.hand_names, initial_arm={a: 0.0 for a in ARMS}, initial_hand={h: 0.0 for h in self.p.hand_names},
                                model_ticks=MODEL_TICKS, hand_owner='MODEL', hand_decoder=lambda sd, v: {'%s_index_2_joint' % sd: 0.5})
        with self.assertRaisesRegex(TargetError, 'not a hand target'):
            bad.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands()))

    def test_declared_clip_is_applied_by_the_adapter_never_a_limit_violation(self):
        t = self.p.targets()
        t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(0, 0, 0, 1.7, 0, -0.1))))   # above the donor closed limit; thumb_yaw -0.1 (pinned dataset value)
        h = t.advance(tick=0, physics_s=0.0)['hand_targets_rad']
        self.assertAlmostEqual(h['right_index_1_joint'], 1.4381, 6)                                         # clip_declared: to the declared endpoint
        self.assertAlmostEqual(h['right_thumb_1_joint'], 0.0, 9)                                            # -0.1 -> 0 on the donor (declared clip)

    def test_hybrid_mode_keeps_hand_ownership_scripted(self):
        t = self.p.targets(owner='SCRIPTED')
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(1.3, 1.3, 1.3, 1.3, 0.0, -0.1))))
        self.assertIsNone(rec['hand_targets_rad']); self.assertEqual(rec['hand_owner'], 'SCRIPTED')
        self.assertEqual(rec['model_hand_raw']['right'], [1.3, 1.3, 1.3, 1.3, 0.0, -0.1])                   # raw model values recorded, not applied
        self.assertTrue(all(v == 0.0 for v in t.advance(tick=0, physics_s=0.0)['hand_targets_rad'].values()))
        closed, _ = self.p.hand_script.to_joint_targets([0.8] * 6)                                          # the scripted closure adapter (probe hybrid route)
        t.set_scripted_hand(closed)
        self.assertAlmostEqual(t.advance(tick=1, physics_s=0.005)['hand_targets_rad']['right_index_1_joint'], 0.8 * 1.4381, 6)
        with self.assertRaises(TargetError):
            self.p.targets(owner='MODEL').set_scripted_hand(closed)

    def test_log_cadence_and_values_first_observation_chunk_boundary_tail(self):
        handover = {h: 0.0 for h in self.p.hand_names}; handover['right_index_1_joint'] = 0.05                   # the targets held at the hand-over
        t = self.p.targets(initial_hand=handover)
        chunk_records, applied_records = [], []
        chunks = [[row(hands(right=(0, 0, 0, 0.3 * (k + 1), 0, 0)), arm=0.1 * (k + 1)) for k in range(3)],          # observation 1
                  [row(hands(right=(0, 0, 0, 1.3, 0, 0)), arm=0.5)] * 3]                                              # observation 2 (new chunk)
        tick = 0
        for it, chunk in enumerate(chunks, start=1):
            for pos, r in enumerate(chunk):
                chunk_records.append(t.begin_row(tick=tick, physics_s=tick * 0.005, iteration=it, chunk_pos=pos, obs_id=it - 1, **r))
                for _ in range(MODEL_TICKS):
                    applied_records.append(t.advance(tick=tick, physics_s=tick * 0.005)); tick += 1
        for _ in range(5):
            applied_records.append(t.hold(tick=tick, physics_s=tick * 0.005)); tick += 1
        self.assertEqual(len(chunk_records), 6); self.assertEqual(t.chunk_records, 6)                          # exactly one per model row
        self.assertEqual(len(applied_records), 6 * MODEL_TICKS + 5); self.assertEqual(t.applied_records, len(applied_records))   # exactly one per tick
        self.assertEqual([r['sequence'] for r in applied_records], list(range(len(applied_records))))
        self.assertEqual([r['sequence'] for r in chunk_records], [0, 4, 8, 12, 16, 20])
        # first observation: the first row's hand target replaces the hand-over value at the row's first tick
        self.assertAlmostEqual(applied_records[0]['hand_targets_rad']['right_index_1_joint'], 0.3, 9)
        self.assertEqual(chunk_records[0]['obs_id'], 0); self.assertEqual(chunk_records[3]['obs_id'], 1)
        # chunk boundary: the new chunk's first row applies its hand target at the boundary tick; the arm ramps from the last applied value
        self.assertAlmostEqual(applied_records[12]['hand_targets_rad']['right_index_1_joint'], 1.3, 9)
        self.assertAlmostEqual(applied_records[11]['hand_targets_rad']['right_index_1_joint'], 0.9, 9)
        self.assertAlmostEqual(chunk_records[3]['arm_targets_rate_limited_rad'][ARMS[0]], 0.5, 9); self.assertAlmostEqual(applied_records[11]['arm_targets_rad'][ARMS[0]], 0.3, 9)
        # tail hold: the last applied targets persist unchanged
        for r in applied_records[-5:]:
            self.assertAlmostEqual(r['hand_targets_rad']['right_index_1_joint'], 1.3, 9); self.assertAlmostEqual(r['arm_targets_rad'][ARMS[0]], 0.5, 9)
            self.assertEqual(r['row_index'], 6)
        # values carried: requested (body_targets_rad), mapped (arm_targets_rate_limited_rad, hand_targets_rad), applied (per tick)
        for r in chunk_records:
            self.assertEqual(r['source'], 'model_chunk'); self.assertEqual(set(r['body_targets_rad']), set(ARMS))
            self.assertEqual(set(r['arm_targets_rate_limited_rad']), set(ARMS)); self.assertEqual(set(r['hand_targets_rad']), set(self.p.hand_names))
        for r in applied_records:
            self.assertEqual(r['source'], 'applied'); self.assertEqual(set(r['arm_targets_rad']), set(ARMS)); self.assertEqual(set(r['hand_targets_rad']), set(self.p.hand_names))

    def test_gap_requested_raw_hand_values_are_logged_for_the_model_owner(self):
        """H3 asks the logs to carry requested, mapped and applied values. With the hand owner MODEL the model_chunk record
        carries the mapped joint targets but drops the requested raw vector (model_hand_raw is None unless SCRIPTED).
        Fix (closed_loop_targets.begin_row): record 'model_hand_raw': hands for both owners."""
        t = self.p.targets()
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(0, 0, 0, 1.3, 0, -0.1))))
        self.assertEqual(rec['model_hand_raw']['right'], [0.0, 0.0, 0.0, 1.3, 0.0, -0.1])

    def test_gap_declared_clip_events_are_logged(self):
        """The adapter reports clipped axes (to_joint_targets()[1]['clipped_axes']); the probe's decoder lambda keeps only [0],
        so a declared clip (e.g. thumb_rotation -0.1 -> 0, or a finger above 1.4381) leaves no trace in the command log.
        Fix: let the decoder return (targets, info) and record 'hand_clipped_axes' per side in the model_chunk record."""
        t = self.p.targets()
        rec = t.begin_row(tick=0, physics_s=0.0, iteration=1, chunk_pos=0, **row(hands(right=(0, 0, 0, 1.7, 0, -0.1))))
        self.assertEqual(rec['hand_clipped_axes']['right'], ['index', 'thumb_rotation'])


if __name__ == '__main__':
    unittest.main()
