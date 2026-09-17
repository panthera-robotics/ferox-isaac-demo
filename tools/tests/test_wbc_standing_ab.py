"""CPU tests for tools/wbc_standing_ab.py: config, rig wrench, evaluator, real-trace ingestion.

Synthetic rows here are MOCK_HARNESS_ONLY fixtures for the evaluator; none is runtime evidence.
"""
import json
import math
import os
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))

from assembled_balance import FEET, GATES, GROUND  # noqa: E402
from wbc_standing_ab import (ARMS, ARM_JOINTS, EXPERIMENTAL_OWNER, Lcg, StandingABConfig, arm_reference_at, config_sha256,  # noqa: E402
                             evaluate_standing, perturbed_initial, rig_stability_margins, rig_wrench)

BODY = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint',
        'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint',
        'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint',
        'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint', 'right_shoulder_pitch_joint',
        'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
        'right_wrist_pitch_joint', 'right_wrist_yaw_joint']
LIMITS = {n: {'lower': -2.0, 'upper': 2.0, 'effort': 25.0, 'velocity': 30.0} for n in BODY}
MASS = 33.3411
CAMPAIGN_STATE = Path('/home/ubuntu/panthera/sim-workspace/campaign-20260913/evidence/overnight-balance-zero-02/state.jsonl')


def cfg(**over):
    base = dict(arm='bare', seed=0, supported_settle_steps=100, unsupported_steps=2000, policy_warmup_steps=20,
                handover={'ramp_start_step': 5, 'ramp_end_step': 20, 'residual_scale': 0.15, 'history_priming': True})
    base.update(over)
    return StandingABConfig.from_dict(base)


def contact(foot, impulse_z):
    return {'actor0': GROUND, 'actor1': '/World/G1/' + foot, 'collider0': GROUND, 'collider1': '/World/G1/' + foot + '/c',
            'sequence': 0, 'physics_s': 0.0, 'phase': 'x', 'position_world_m': [0., 0., 0.], 'normal_world': [0., 0., 1.],
            'impulse_ns': [0., 0., impulse_z], 'separation_m': -0.0002, 'source': 'PhysX_simulated_contact_proxy'}


def rows_for(c, *, topple_after=None, hidden_support=False, weight_fraction=1.0, near_cap_from=None, nonfoot_at=None):
    rows, events = [], []
    total = c.supported_settle_steps + c.unsupported_steps
    release = c.supported_settle_steps - 1
    per_foot = 0.5 * weight_fraction * MASS * 9.81 * GATES['physics_dt_s']
    for seq in range(total):
        supported = seq <= release
        warmup = seq < c.policy_warmup_steps
        if seq == release + 1:
            events.append({'sequence': release, 'name': 'support_release'})
        pitch = 0.0
        if topple_after is not None and not supported and seq - release > topple_after:
            pitch = 0.4 * ((seq - release - topple_after) / 200.0)
        z = 0.79 - 0.4 * math.sin(pitch)
        row = {'sequence': seq, 'phase': 'supported_settle' if supported else 'unsupported', 'physics_s': (seq + 1) * 0.005,
               'runtime_names': BODY, 'q_rad': [0.0] * 29, 'dq_rad_s': [0.0] * 29, 'measured_generalized_effort_nm': [0.0] * 29,
               'pelvis_linear_velocity_m_s': [0., 0., 0.], 'pelvis_angular_velocity_rad_s': [0., 0., 0.],
               'link_poses_world_xyzw': {'pelvis': [0., 0., z, 0., math.sin(pitch / 2), 0., math.cos(pitch / 2)],
                                         'torso_link': [0., 0., z + .3, 0., 0., 0., 1.],
                                         FEET[0]: [0., .1, .03, 0., 0., 0., 1.], FEET[1]: [0., -.1, .03, 0., 0., 0., 1.]},
               'command_velocity': [0., 0., 0.], 'body_command_owner': 'probe_default_pose_warmup' if warmup else 'named_policy_single_writer', 'body_command_names': BODY,
               'body_command_rad': [0.0] * 29, 'hand_command_names': [], 'hand_command_rad': [],
               'policy_inference_this_step': (not warmup) and seq % 4 == 0, 'policy_observation': [] if warmup else [0.0] * 480, 'policy_action': [] if warmup else [0.0] * 29,
               'contacts': [contact(FEET[0], per_foot), contact(FEET[1], per_foot)],
               'drive_estimate_near_cap_names': (['left_knee_joint'] if near_cap_from is not None and seq >= near_cap_from else []),
               'support': ({'kind': 'RIG_WRENCH', 'force_n': [0., 0., 10.], 'torque_nm': [0., 0., 0.]} if supported or hidden_support
                           else {'kind': 'NONE', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]})}
        if nonfoot_at is not None and seq == nonfoot_at:
            row['contacts'].append(contact('left_knee_link', 0.5))
        rows.append(row)
    return rows, events


INTEGRITY = {'source_mass_com_inertia_preserved': True, 'no_static_triangle_shapes': True, 'initial_pose_matches_source_fk': True,
             'policy_named_mapping_and_live_gains_caps_verified': True, 'runtime_joint_count_matches_arm': True,
             'contact_instrumentation_valid': True, 'both_palm_shape_counts_match': True, 'left_thumb_shape_count_matches': True}


class ConfigTests(unittest.TestCase):
    def test_arms_and_defaults(self):
        c = cfg()
        self.assertEqual(ARMS[c.arm]['joint_count'], 29)
        self.assertEqual(ARMS['donor']['joint_count'], 53)
        self.assertEqual(c.hand_margin_rad, 0.02)
        self.assertEqual(len(config_sha256(c)), 64)
        self.assertNotEqual(config_sha256(c), config_sha256(cfg(seed=1)))

    def test_refusals(self):
        with self.assertRaisesRegex(ValueError, 'policy_warmup_steps'):
            cfg(policy_warmup_steps=100, handover={'ramp_start_step': 5, 'ramp_end_step': 20, 'residual_scale': 0.15, 'history_priming': True})
        with self.assertRaisesRegex(ValueError, 'ramp must complete'):
            cfg(handover={'ramp_start_step': 5, 'ramp_end_step': 30, 'residual_scale': 0.15, 'history_priming': True})
        d = StandingABConfig.from_dict({'arm': 'bare'})
        self.assertEqual((d.policy_warmup_steps, d.handover['ramp_end_step']), (250, 250))
        with self.assertRaisesRegex(ValueError, 'arm must be'):
            cfg(arm='thumbchain2')
        with self.assertRaisesRegex(ValueError, 'exact-open zero margin'):
            cfg(hand_margin_rad=0.0)
        with self.assertRaisesRegex(ValueError, 'unsupported_steps'):
            cfg(unsupported_steps=1500)
        with self.assertRaisesRegex(ValueError, 'immutable private mounts'):
            cfg(policy_path='/tmp/policy')
        with self.assertRaisesRegex(ValueError, 'perturbation magnitudes'):
            cfg(perturbation={'joint_rad': 0.5, 'pelvis_xy_m': 0.0, 'pelvis_yaw_rad': 0.0})
        with self.assertRaisesRegex(ValueError, 'Unknown standing'):
            StandingABConfig.from_dict({'arm': 'bare', 'extra': 1})

    def test_seeded_perturbation_is_deterministic_and_clipped(self):
        default = {n: 0.0 for n in BODY}
        limits = dict(LIMITS, left_knee_joint={'lower': -0.001, 'upper': 0.001, 'effort': 25., 'velocity': 30.})
        c = cfg(seed=3, perturbation={'joint_rad': 0.01, 'pelvis_xy_m': 0.01, 'pelvis_yaw_rad': 0.05})
        q1, p1 = perturbed_initial(default, limits, c)
        q2, p2 = perturbed_initial(default, limits, c)
        self.assertEqual((q1, p1), (q2, p2))
        self.assertTrue(all(abs(v) <= 0.01 for v in q1.values()))
        self.assertLessEqual(abs(q1['left_knee_joint']), 0.001)
        q0, p0 = perturbed_initial(default, limits, cfg(seed=0))
        self.assertTrue(all(v == 0.0 for v in q0.values()))   # seed 0 with zero magnitudes: the nominal start
        self.assertNotEqual(q1, perturbed_initial(default, limits, cfg(seed=4, perturbation=c.perturbation))[0])
        self.assertNotEqual(Lcg(1).uniform(0, 1), Lcg(2).uniform(0, 1))


class RigTests(unittest.TestCase):
    def test_wrench_direction_and_caps(self):
        c = cfg()
        target = [0., 0., 0.8, 0., 0., 0., 1.]
        f, t = rig_wrench(c, [0., 0., 0.79, 0., 0., 0., 1.], [0., 0., 0.], [0., 0., 0.], target)
        self.assertGreater(f[2], 0.)
        self.assertAlmostEqual(f[2], c.rig['kp_n_m'] * 0.01)
        self.assertEqual(t, [0., 0., 0.])
        # gravity feed-forward carries the weight at zero error
        f, t = rig_wrench(c, target, [0., 0., 0.], [0., 0., 0.], target, body_mass_kg=34.7577)
        self.assertAlmostEqual(f[2], 34.7577 * 9.81)
        f, t = rig_wrench(c, [0., 0., 0.0, 0., 0., 0., 1.], [0., 0., 0.], [0., 0., 0.], target)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in f)), c.rig['max_force_n'])
        # pitched forward: restoring torque about -y (small-angle vector part)
        s = math.sin(0.05); w = math.cos(0.05)
        f, t = rig_wrench(c, [0., 0., 0.8, 0., s, 0., w], [0., 0., 0.], [0., 0., 0.], target)
        self.assertLess(t[1], 0.)


class RigStabilityTests(unittest.TestCase):
    PELVIS_M, PELVIS_I = 3.813, 0.0079184   # imported donor/bare pelvis link (identical body)

    def test_rev2_gains_are_unstable_and_rev3_gains_are_stable(self):
        rev2 = cfg(rig={'kp_n_m': 20000.0, 'kd_n_s_m': 2000.0, 'kr_nm_rad': 2000.0, 'kdr_nm_s_rad': 200.0, 'max_force_n': 1200.0, 'max_torque_nm': 400.0, 'gravity_feedforward': True})
        m = rig_stability_margins(rev2, self.PELVIS_M, self.PELVIS_I)
        self.assertFalse(m['stable'])
        self.assertGreater(m['kdr_dt_over_I'], 100)
        rev3 = cfg(rig={'kp_n_m': 10000.0, 'kd_n_s_m': 500.0, 'kr_nm_rad': 100.0, 'kdr_nm_s_rad': 0.5, 'max_force_n': 1200.0, 'max_torque_nm': 400.0, 'gravity_feedforward': True})
        m = rig_stability_margins(rev3, self.PELVIS_M, self.PELVIS_I)
        self.assertTrue(m['stable'])
        self.assertLess(max(m['kd_dt_over_m'], m['kdr_dt_over_I'], m['kp_dt2_over_m'], m['kr_dt2_over_I']), 0.7)


class HandoverTests(unittest.TestCase):
    def test_rig_scale_ramp(self):
        from wbc_standing_ab import rig_scale_at
        c = cfg()
        self.assertEqual(rig_scale_at(c, 0), 1.0)
        self.assertEqual(rig_scale_at(c, 4), 1.0)
        self.assertAlmostEqual(rig_scale_at(c, 12), 1.0 + (7 / 15) * (0.15 - 1.0))
        self.assertEqual(rig_scale_at(c, 20), 0.15)
        self.assertEqual(rig_scale_at(c, 99), 0.15)

    def test_feet_must_be_loaded_at_handover(self):
        c = cfg()
        rows, events = rows_for(c)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertTrue(m['checks']['feet_loaded_at_policy_handover'])
        self.assertGreater(m['peaks']['handover_foot_load_n'], 0.8 * MASS * 9.81)
        rows, events = rows_for(c)
        rows[c.policy_warmup_steps - 1]['contacts'] = []          # hanging at hand-over
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['feet_loaded_at_policy_handover'])
        self.assertEqual(m['status'], 'FAIL')


class EvaluatorTests(unittest.TestCase):
    def test_synthetic_stand_passes_and_is_labelled_by_config(self):
        c = cfg()
        rows, events = rows_for(c)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertEqual(m['status'], 'PASS', m['first_failed_gate'])
        self.assertEqual(m['unsupported_steps'], 2000)
        self.assertEqual(m['recorded_release_sequence'], 99)
        self.assertGreaterEqual(m['peaks']['mean_foot_ground_load_n'], 0.9 * MASS * 9.81)
        self.assertEqual(m['execution_label'], 'UNSUPPORTED_STANDING_AB')

    def test_topple_reports_first_cause_and_first_failed_gate(self):
        c = cfg()
        rows, events = rows_for(c, topple_after=300)
        m = evaluate_standing(rows[:1000], events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS,
                              integrity_checks=INTEGRITY, abort={'sequence': 999, 'reason': 'fall_or_source_envelope_abort'})
        self.assertEqual(m['status'], 'FAIL')
        self.assertFalse(m['checks']['roll_pitch_within_gate'])
        self.assertFalse(m['checks']['full_unsupported_duration'])
        names = [e['name'] for e in m['first_causal_events']]
        self.assertIn('base_tilt_exceeds_0.1rad', names)
        self.assertLess(names.index('base_tilt_exceeds_0.1rad'), names.index('abort'))

    def test_hidden_support_after_release_fails(self):
        c = cfg()
        rows, events = rows_for(c, hidden_support=True)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['no_support_after_release'])
        self.assertEqual(m['first_causal_events'][0]['name'], 'hidden_support_after_release')

    def test_feet_must_carry_the_weight(self):
        c = cfg()
        rows, events = rows_for(c, weight_fraction=0.5)   # half the weight on the feet: something else holds it
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['feet_carry_body_weight_after_release'])

    def test_missing_release_and_wrong_joint_count(self):
        c = cfg()
        rows, events = rows_for(c)
        m = evaluate_standing(rows, [], c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['single_recorded_release'])
        self.assertEqual(m['first_causal_events'][0]['name'], 'evidence_error')
        m = evaluate_standing(rows, events, c, joint_count=53, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['finite_complete_measured_state'])

    def test_saturation_and_nonfoot_contact_events(self):
        c = cfg()
        rows, events = rows_for(c, near_cap_from=500, nonfoot_at=700)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['no_sustained_drive_saturation'])
        self.assertFalse(m['checks']['no_loaded_nonfoot_ground_contact'])
        names = [e['name'] for e in m['first_causal_events']]
        self.assertEqual(names[:2], ['drive_near_effort_cap', 'nonfoot_ground_contact'])

    def test_wrong_owner_or_positional_body_order_is_refused(self):
        c = cfg()
        rows, events = rows_for(c)
        for r in rows:
            r['body_command_owner'] = 'j_fixed_pelvis_replay_v4'
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['single_body_owner_named_policy'])
        rows, events = rows_for(c)
        for r in rows:
            r['body_command_names'] = BODY[:28] + ['left_hand_index_0_joint']
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['single_body_owner_named_policy'])

    def test_ownership_journal_consistency_gate(self):
        c = cfg()
        rows, events = rows_for(c)
        good = [{'kind': 'body_owner', 'sequence': None, 'owner': 'probe_default_pose_warmup'}, {'kind': 'hand_owner', 'sequence': None}, {'kind': 'support', 'sequence': None},
                {'kind': 'body_owner_handover', 'sequence': 20}, {'kind': 'body_owner', 'sequence': 20, 'owner': 'named_policy_single_writer'},
                {'kind': 'support_release', 'sequence': 100}]
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS,
                              integrity_checks=INTEGRITY, guard_entries=good)
        self.assertTrue(m['checks']['ownership_journal_consistent'])
        self.assertEqual(m['status'], 'PASS')
        bad = good + [{'kind': 'refused', 'sequence': 500, 'reason': 'x'}]
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS,
                              integrity_checks=INTEGRITY, guard_entries=bad)
        self.assertFalse(m['checks']['ownership_journal_consistent'])
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS,
                              integrity_checks=INTEGRITY, guard_entries=good[:3] + [{'kind': 'support_release', 'sequence': 7}])
        self.assertFalse(m['checks']['ownership_journal_consistent'])

    def test_nonqualifying_diagnostic_can_never_pass(self):
        with self.assertRaisesRegex(ValueError, 'donor arm only'):
            cfg(arm='bare', diagnostics={'disable_hand_collisions': True, 'lock_hand_joints': False})
        c = cfg(arm='donor', diagnostics={'disable_hand_collisions': True, 'lock_hand_joints': False})
        self.assertTrue(c.nonqualifying)
        rows, events = rows_for(c)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertEqual(m['status'], 'FAIL')
        self.assertFalse(m['checks']['no_nonqualifying_diagnostic'])
        self.assertTrue(m['nonqualifying_diagnostic'])
        self.assertTrue(all(v for k, v in m['checks'].items() if k != 'no_nonqualifying_diagnostic'))

    def test_command_schedule_validation_and_walk_measurements(self):
        with self.assertRaisesRegex(ValueError, 'refused, not clipped'):
            cfg(unsupported_steps=6000, command_schedule=[[5.0, 1.5, 0.0, 0.0]])
        with self.assertRaisesRegex(ValueError, 'start >= 5 s'):
            cfg(unsupported_steps=6000, command_schedule=[[1.0, 0.3, 0.0, 0.0]])
        with self.assertRaisesRegex(ValueError, '30 s unsupported'):
            cfg(command_schedule=[[5.0, 0.3, 0.0, 0.0]])
        c = cfg(unsupported_steps=6000, command_schedule=[[5.0, 0.3, 0.0, 0.0], [11.0, 0.0, 0.0, 0.0]])
        rows, events = rows_for(c)
        rel = c.supported_settle_steps - 1
        for r in rows:
            if r['phase'] == 'unsupported':
                tu = (r['sequence'] - rel) * 0.005
                r['command_velocity'] = [0.3, 0., 0.] if 5.0 <= tu < 11.0 else [0., 0., 0.]
                x = 0.3 * max(0.0, min(tu, 11.0) - 5.0)
                r['link_poses_world_xyzw']['pelvis'][0] = x
                for f in FEET:
                    r['link_poses_world_xyzw'][f][0] = x
                r['pelvis_linear_velocity_m_s'] = [0.3 if 5.0 <= tu < 11.0 else 0.0, 0., 0.]
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertEqual(m['status'], 'PASS', m['first_failed_gate'])
        self.assertAlmostEqual(m['peaks']['walk']['forward_travel_m'], 1.8, places=2)
        self.assertTrue(m['checks']['walk_travel_reached'])
        self.assertTrue(m['checks']['stopped_after_stop_command'])
        # a row whose command differs from the schedule is a foreign writer
        rows[-1]['command_velocity'] = [0.5, 0., 0.]
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['single_body_owner_named_policy'])

    def test_arm_override_is_experimental_owner_from_support_and_tracks(self):
        with self.assertRaisesRegex(ValueError, 'separate experiments'):
            cfg(unsupported_steps=6000, command_schedule=[[5.0, 0.3, 0.0, 0.0]], arm_override={'enabled': True, 'start_s': 5.0, 'mode': 'null', 'trajectory': []})
        with self.assertRaisesRegex(ValueError, 'rate exceeds'):
            cfg(arm_override={'enabled': True, 'start_s': 5.0, 'mode': 'trajectory', 'trajectory': [[0.0] + [0.0] * 14, [0.5] + [0.8] * 14]})
        c = cfg(arm_override={'enabled': True, 'start_s': 5.0, 'mode': 'trajectory', 'trajectory': [[0.0] + [0.0] * 14, [2.0] + [0.2] * 14]})
        default_arm = [0.0] * 14
        self.assertIsNone(arm_reference_at(c, default_arm, 4.0))
        self.assertAlmostEqual(arm_reference_at(c, default_arm, 6.0)[0], 0.1)
        self.assertAlmostEqual(arm_reference_at(c, default_arm, 9.0)[0], 0.2)
        rows, events = rows_for(c)
        rel = c.supported_settle_steps - 1
        ai = [BODY.index(n) for n in ARM_JOINTS]
        for r in rows:
            if r['phase'] != 'supported_settle' or r['sequence'] >= c.policy_warmup_steps:
                r['body_command_owner'] = EXPERIMENTAL_OWNER
            tu = (r['sequence'] - rel) * 0.005
            ref = arm_reference_at(c, default_arm, tu) if r['phase'] == 'unsupported' else None
            r['arm_reference_rad'] = ref
            if ref is not None:
                q = list(r['q_rad'])
                for i, v in zip(ai, ref):
                    q[i] = v - 0.01     # 10 mrad tracking error
                r['q_rad'] = q
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertEqual(m['status'], 'PASS', m['first_failed_gate'])
        self.assertIn('EXPERIMENTAL_COMBINED_CONTROLLER', m['verdict_scope'])
        self.assertAlmostEqual(m['peaks']['arm_override']['tracking_peak_abs_rad'], 0.01, places=6)
        # a row claiming the plain policy owner during an override run is a foreign writer
        rows[-1]['body_command_owner'] = 'named_policy_single_writer'
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS, integrity_checks=INTEGRITY)
        self.assertFalse(m['checks']['single_body_owner_named_policy'])

    def test_integrity_failure_blocks_pass(self):
        c = cfg()
        rows, events = rows_for(c)
        m = evaluate_standing(rows, events, c, joint_count=29, limits=LIMITS, mimics={}, source_mass_kg=MASS,
                              integrity_checks=dict(INTEGRITY, source_mass_com_inertia_preserved=False))
        self.assertEqual(m['status'], 'FAIL')
        self.assertEqual(m['first_failed_gate'], 'initialization_source_mass_com_inertia_preserved')

    @unittest.skipUnless(CAMPAIGN_STATE.is_file(), 'campaign balance-02 evidence not present on this host')
    def test_ingests_the_real_campaign_balance_02_trace(self):
        """Real PhysX rows (donor, 53 joints, no support phase) flow through the evaluator; verdict FAIL is expected."""
        rows = [json.loads(line) for line in CAMPAIGN_STATE.read_text().splitlines() if line.strip()]
        c = cfg(arm='donor', policy_warmup_steps=1, handover={'ramp_start_step': 0, 'ramp_end_step': 1, 'residual_scale': 0.0, 'history_priming': False})
        limits = json.loads((CAMPAIGN_STATE.parent / 'assembled_asset.json').read_text())['joint_limits']
        mimics = json.loads((CAMPAIGN_STATE.parent / 'assembled_asset.json').read_text())['mimic_map']
        for r in rows:
            r['phase'] = 'unsupported'
            r['support'] = {'kind': 'NONE', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]}
        events = [{'sequence': -1, 'name': 'support_release'}]
        m = evaluate_standing(rows, events, c, joint_count=53, limits=limits, mimics=mimics, source_mass_kg=34.7577,
                              integrity_checks=INTEGRITY, abort={'sequence': 40, 'reason': 'fall_or_source_envelope_abort'})
        self.assertEqual(m['status'], 'FAIL')
        self.assertEqual(m['unsupported_steps'], 41)
        self.assertTrue(m['first_causal_events'], 'real trace must yield at least one causal event')
        self.assertIn(m['first_causal_events'][0]['name'], ('overspeed', 'foot_drift_exceeds_gate', 'drive_near_effort_cap', 'base_tilt_exceeds_0.1rad', 'joint_limit_violation'))


if __name__ == '__main__':
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    unittest.main(verbosity=1)


class GuardedStateView(unittest.TestCase):
    def test_mimic_joints_are_projected_out_by_name(self):
        from wbc_standing_ab import guarded_state_view
        names = BODY + ['right_index_1_joint', 'right_index_2_joint', 'left_thumb_1_joint']
        q = list(range(len(names))); dq = [float(-i) for i in range(len(names))]
        n, qq, dd = guarded_state_view(names, q, dq, BODY + ['left_thumb_1_joint', 'right_index_1_joint'])
        self.assertEqual(n[-2:], ['left_thumb_1_joint', 'right_index_1_joint'])
        self.assertEqual(qq[-2:], [len(names) - 1, len(names) - 3]); self.assertEqual(dd[-1], -(len(names) - 3))
        with self.assertRaises(ValueError):
            guarded_state_view(names, q[:-1], dq, BODY)
        with self.assertRaises(ValueError):
            guarded_state_view(names, q, dq, BODY + ['missing_joint'])
