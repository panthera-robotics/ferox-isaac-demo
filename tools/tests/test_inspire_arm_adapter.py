"""CPU/std-library-only tests for named simulation body ownership and freshness."""

import unittest

from isaac.twin.inspire.arm_adapter import (
    JointBound, NamedBodyArbiter, SimulationAdmissionError, UPPER_NAMES,
)


LEG_NAMES = tuple('%s_%s_joint' % (side, joint) for side in ('left', 'right')
                  for joint in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee',
                                'ankle_pitch', 'ankle_roll'))
NAMES = LEG_NAMES + UPPER_NAMES
# Deliberately interleave non-body indices, simulating an importer with hand joints.
INDICES = {name: i * 2 for i, name in enumerate(reversed(NAMES))}
BOUNDS = {name: JointBound(-3.0, 3.0, 5.0, 200.0, 10.0, 100.0) for name in NAMES}


def arbiter(**kw):
    config = dict(body_indices=INDICES, bounds=BOUNDS, simulator_id='isaacsim_test',
                  run_id='episode_001', mode='hybrid', controller_id='balance_surrogate',
                  simulation_authorized=True, implicit_drives_disabled=True)
    config.update(kw)
    return NamedBodyArbiter(**config)


def body(q=0.1, kp=20.0, tau=0.0):
    return {name: dict(q=q, dq=0.0, kp=kp, kd=1.0, tau=tau) for name in NAMES}


def sample(a, sequence=1, sim_t=1.0, source_t=10.0, now=10.0):
    a.observe_physics(sequence=sequence, sim_time_s=sim_t, source_monotonic_s=source_t,
                      position={name: 0.0 for name in NAMES},
                      velocity={name: 0.0 for name in NAMES}, now_monotonic_s=now)


def task(**kw):
    value = dict(schema_version=1, kind='upper_body_reference', target='isaacsim',
                 simulator_id='isaacsim_test', run_id='episode_001', sequence=1,
                 sim_time_s=1.0, source_monotonic_time_s=10.0, valid_for_s=0.10,
                 blend_weight=0.5,
                 joints={name: dict(q=0.2, dq=0.0, kp=40.0, kd=1.0, tau=0.0)
                         for name in UPPER_NAMES},
                 semantics='arm_sdk_surrogate_reference_not_final_actuation',
                 hand_authority='none', leg_authority='none')
    value.update(kw)
    return value


def output(a, reference=None, controller='balance_surrogate', sequence=1, now=10.0):
    return a.compose(body() if reference is None else reference, controller_id=controller,
                     physics_sequence=sequence, now_monotonic_s=now)


class AdmissionTests(unittest.TestCase):
    def test_requires_separate_authorization_and_exactly_one_mode(self):
        for changes in ({'simulation_authorized': False}, {'simulation_authorized': 1},
                        {'target': 'hardware'}, {'implicit_drives_disabled': False},
                        {'mode': 'hybrid+fullbody'}, {'controller_id': ''}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                arbiter(**changes)

    def test_asset_names_and_indices_are_not_contiguous_slice_assumptions(self):
        a = arbiter()
        sample(a)
        result = output(a)
        self.assertEqual(len(result.effort_nm), 29)
        self.assertEqual(result.articulation_indices, tuple(range(0, 58, 2)))
        self.assertEqual(set(result.joint_names), set(NAMES))
        self.assertEqual(result.final_writer, 'simulation_body_arbiter')
        self.assertEqual(result.effort_nm, (2.0,) * 29)

    def test_wrong_asset_name_duplicate_index_and_missing_bound_refuse(self):
        missing = dict(INDICES)
        missing.pop('right_elbow_joint')
        duplicate = dict(INDICES)
        duplicate[NAMES[0]] = duplicate[NAMES[1]]
        for changes in ({'body_indices': missing}, {'body_indices': duplicate},
                        {'bounds': {k: v for k, v in BOUNDS.items() if k != NAMES[0]}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                arbiter(**changes)

    def test_bound_definitions_refuse_nonfinite_or_nonpositive_limits(self):
        for args in ((0, 0, 1, 1, 1, 1), (-1, 1, 0, 1, 1, 1),
                     (-1, 1, 1, 1, 1, float('nan'))):
            with self.subTest(args=args), self.assertRaises(ValueError):
                JointBound(*args)


class OwnershipTests(unittest.TestCase):
    def test_hybrid_blends_efforts_once_and_leaves_legs_with_balance(self):
        a = arbiter()
        sample(a)
        a.accept_upper_reference(task(), now_monotonic_s=10.0)
        result = output(a)
        for name, effort, owner in zip(result.joint_names, result.effort_nm,
                                       result.reference_owners):
            # 0.5 * (20 * 0.1) + 0.5 * (40 * 0.2) = 5.0, not 30 * 0.15 = 4.5.
            self.assertEqual(effort, 5.0 if name in UPPER_NAMES else 2.0)
            self.assertIn('hybrid' if name in UPPER_NAMES else 'balance_surrogate', owner)

    def test_blend_endpoints_and_feedforward(self):
        for weight, expected in ((0.0, 3.0), (1.0, 8.0)):
            with self.subTest(weight=weight):
                a = arbiter()
                sample(a)
                a.accept_upper_reference(task(blend_weight=weight), now_monotonic_s=10.0)
                result = output(a, body(tau=1.0))
                values = dict(zip(result.joint_names, result.effort_nm))
                self.assertEqual(values['waist_pitch_joint'], expected)
                self.assertEqual(values['left_knee_joint'], 3.0)

    def test_fullbody_is_exclusive_and_only_its_named_controller_can_write(self):
        a = arbiter(mode='fullbody', controller_id='wbc_surrogate')
        sample(a)
        result = output(a, controller='wbc_surrogate')
        self.assertEqual(set(result.reference_owners), {'wbc_surrogate'})
        with self.assertRaisesRegex(SimulationAdmissionError, 'conflicts'):
            a.accept_upper_reference(task(), now_monotonic_s=10.0)
        b = arbiter()
        sample(b)
        with self.assertRaisesRegex(SimulationAdmissionError, 'controller'):
            output(b, controller='another_writer')

    def test_second_writer_cannot_apply_pd_twice_to_same_physics_sample(self):
        a = arbiter()
        sample(a)
        output(a)
        with self.assertRaisesRegex(SimulationAdmissionError, 'second body write'):
            output(a)

    def test_effort_overflow_faults_instead_of_silently_changing_approved_target(self):
        a = arbiter()
        sample(a)
        with self.assertRaisesRegex(SimulationAdmissionError, 'final_effort'):
            output(a, body(q=2.0, kp=200.0))

    def test_handover_is_a_fresh_run_and_never_clears_original_latch(self):
        a = arbiter()
        sample(a)
        with self.assertRaises(SimulationAdmissionError):
            output(a, controller='unknown')
        with self.assertRaises(ValueError):
            a.reset_for_run(run_id=a.run_id, mode='fullbody', controller_id='wbc',
                            simulation_authorized=True)
        b = a.reset_for_run(run_id='episode_002', mode='fullbody', controller_id='wbc',
                            simulation_authorized=True)
        with self.assertRaises(SimulationAdmissionError):
            output(a)
        self.assertIsNone(b.fault_reason)
        sample(b)
        self.assertEqual(output(b, controller='wbc').mode, 'fullbody')


class FreshnessTests(unittest.TestCase):
    def test_no_physics_stale_physics_and_future_physics_refuse(self):
        a = arbiter()
        with self.assertRaisesRegex(SimulationAdmissionError, 'no admitted physics'):
            output(a)
        for source_t in (9.0, 11.0):
            with self.subTest(source_t=source_t), self.assertRaises(SimulationAdmissionError):
                sample(arbiter(), source_t=source_t)

    def test_task_replay_wrong_run_wrong_target_or_extra_authority_latches(self):
        for changes in ({'run_id': 'episode_000'}, {'target': 'hardware'},
                        {'hand_authority': 'right'}, {'leg_authority': 'all'},
                        {'schema_version': True}, {'sequence': True},
                        {'blend_weight': float('nan')}, {'blend_weight': 1.1},
                        {'valid_for_s': 1.0}, {'unexpected': 1}):
            with self.subTest(changes=changes):
                a = arbiter()
                sample(a)
                with self.assertRaises(SimulationAdmissionError):
                    a.accept_upper_reference(task(**changes), now_monotonic_s=10.0)
                with self.assertRaises(SimulationAdmissionError):
                    a.accept_upper_reference(task(), now_monotonic_s=10.0)
        a = arbiter()
        sample(a)
        a.accept_upper_reference(task(), now_monotonic_s=10.0)
        with self.assertRaisesRegex(SimulationAdmissionError, 'replay'):
            a.accept_upper_reference(task(), now_monotonic_s=10.0)

    def test_fresh_wall_time_does_not_let_old_or_future_simulation_samples_through(self):
        for sim_t in (0.8, 1.1):
            with self.subTest(sim_t=sim_t):
                a = arbiter()
                sample(a)
                with self.assertRaisesRegex(SimulationAdmissionError, 'physics age'):
                    a.accept_upper_reference(task(sim_time_s=sim_t), now_monotonic_s=10.0)

    def test_source_wall_age_is_independent_of_simulation_time(self):
        for source_t in (9.8, 10.1):
            with self.subTest(source_t=source_t):
                a = arbiter()
                sample(a)
                with self.assertRaisesRegex(SimulationAdmissionError, 'task age'):
                    a.accept_upper_reference(task(source_monotonic_time_s=source_t),
                                              now_monotonic_s=10.0)

    def test_physics_pause_reset_duplicate_and_clock_rewind_are_latched(self):
        for seq, sim_t, source_t, now in ((1, 1.02, 10.02, 10.02),
                                        (2, 1.0, 10.02, 10.02),
                                        (2, 0.1, 10.02, 10.02),
                                        (2, 1.02, 10.0, 10.02),
                                        (2, 1.02, 9.99, 9.99)):
            with self.subTest(seq=seq, sim_t=sim_t, source_t=source_t):
                a = arbiter()
                sample(a)
                with self.assertRaises(SimulationAdmissionError):
                    sample(a, seq, sim_t, source_t, now)
                self.assertEqual(a.fault_action, 'stop_physics_and_reset_episode')

    def test_pause_watchdog_cannot_keep_stale_blend_latched(self):
        a = arbiter()
        sample(a)
        a.accept_upper_reference(task(blend_weight=1.0), now_monotonic_s=10.0)
        with self.assertRaises(SimulationAdmissionError):
            a.check_freshness(10.11)
        self.assertIsNone(a._upper)
        with self.assertRaises(SimulationAdmissionError):
            output(a, now=10.11)

    def test_fast_physics_does_not_reuse_old_task_while_wall_clock_is_fresh(self):
        a = arbiter()
        sample(a)
        a.accept_upper_reference(task(), now_monotonic_s=10.0)
        sample(a, sequence=2, sim_t=1.2, source_t=10.02, now=10.02)
        with self.assertRaisesRegex(SimulationAdmissionError, 'task physics age'):
            output(a, sequence=2, now=10.02)

    def test_next_fresh_reference_and_physics_sample_are_accepted(self):
        a = arbiter()
        sample(a)
        a.accept_upper_reference(task(), now_monotonic_s=10.0)
        output(a)
        sample(a, sequence=2, sim_t=1.02, source_t=10.02, now=10.02)
        a.accept_upper_reference(task(sequence=2, sim_time_s=1.02,
                                     source_monotonic_time_s=10.02), now_monotonic_s=10.02)
        self.assertEqual(output(a, sequence=2, now=10.02).physics_sequence, 2)

    def test_malformed_named_references_and_nonfinite_channels_fail_closed(self):
        for name, channel, value in ((UPPER_NAMES[0], 'kp', 0.0),
                                     (UPPER_NAMES[1], 'q', float('inf')),
                                     (UPPER_NAMES[-1], 'tau', 101.0)):
            with self.subTest(name=name, channel=channel):
                a = arbiter()
                sample(a)
                msg = task()
                msg['joints'][name][channel] = value
                with self.assertRaises(SimulationAdmissionError):
                    a.accept_upper_reference(msg, now_monotonic_s=10.0)
        a = arbiter()
        sample(a)
        msg = task()
        msg['joints']['right_hand'] = msg['joints'].pop(UPPER_NAMES[-1])
        with self.assertRaisesRegex(SimulationAdmissionError, 'name set'):
            a.accept_upper_reference(msg, now_monotonic_s=10.0)


if __name__ == '__main__':
    unittest.main()


class ImplicitDriveArbiterTests(unittest.TestCase):
    """Versioned implicit realization: gain-weighted blended target, zero additive effort."""

    def drive_arbiter(self, **kw):
        from isaac.twin.inspire.implicit_arm_adapter import NamedBodyDriveArbiter
        config = dict(body_indices=INDICES, bounds=BOUNDS, simulator_id='isaacsim_test',
                      run_id='episode_001', mode='hybrid', controller_id='balance_surrogate',
                      simulation_authorized=True, explicit_efforts_disabled=True,
                      source_caps_verified=True)
        config.update(kw)
        return NamedBodyDriveArbiter(**config)

    def test_requires_verified_zero_explicit_effort_and_source_caps(self):
        for bad in (dict(explicit_efforts_disabled=False), dict(source_caps_verified=False),
                    dict(maximum_feedforward_bias_rad=0.5), dict(maximum_feedforward_bias_rad=float('nan'))):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.drive_arbiter(**bad)
        with self.assertRaises(ValueError):
            arbiter(actuation_backend='implicit_biased_drive_v1')  # implicit needs declared drives
        with self.assertRaises(ValueError):
            arbiter(actuation_backend='unknown_backend')

    def test_blended_drive_matches_explicit_effort_field_before_the_cap(self):
        a = self.drive_arbiter()
        sample(a)
        a.accept_upper_reference(task(), now_monotonic_s=10.0)
        drive = a.compose_drives(body(tau=1.0), controller_id='balance_surrogate',
                                 physics_sequence=1, now_monotonic_s=10.0)
        b = arbiter()
        sample(b)
        b.accept_upper_reference(task(), now_monotonic_s=10.0)
        explicit = dict(zip(*[(r := output(b, body(tau=1.0))).joint_names, r.effort_nm]))
        self.assertEqual(drive.actuation_semantics.split(';')[0], 'implicit_biased_drive_v1')
        self.assertEqual(drive.final_writer, 'simulation_implicit_body_arbiter')
        for name, target, dq, kp, kd, cap, bias, estimate in zip(
                drive.joint_names, drive.target_position_rad, drive.target_velocity_rad_s,
                drive.stiffness_nm_rad, drive.damping_nm_s_rad, drive.source_effort_caps_nm,
                drive.feedforward_target_bias_rad, drive.current_pd_estimate_nm):
            # kp*(target-q)+kd*(dq_target-dq) at q=dq=0 reproduces the explicit blended field exactly.
            self.assertAlmostEqual(kp * target + kd * dq, explicit[name], places=12)
            self.assertAlmostEqual(estimate, explicit[name], places=12)
            self.assertEqual(cap, BOUNDS[name].max_effort)
            if name in UPPER_NAMES:
                self.assertEqual(kp, 30.0)      # 0.5*20+0.5*40
                self.assertAlmostEqual(bias, 0.5 / 30.0)
            else:
                self.assertEqual(kp, 20.0)
                self.assertAlmostEqual(bias, 1.0 / 20.0)

    def test_explicit_compose_is_refused_and_second_write_same_sample_is_refused(self):
        a = self.drive_arbiter()
        sample(a)
        with self.assertRaises(SimulationAdmissionError):
            output(a)
        b = self.drive_arbiter()
        sample(b)
        b.compose_drives(body(), controller_id='balance_surrogate', physics_sequence=1, now_monotonic_s=10.0)
        with self.assertRaises(SimulationAdmissionError):
            b.compose_drives(body(), controller_id='balance_surrogate', physics_sequence=1, now_monotonic_s=10.0)

    def test_bias_and_target_bounds_fault_instead_of_clipping(self):
        a = self.drive_arbiter(maximum_feedforward_bias_rad=0.01)
        sample(a)
        with self.assertRaises(SimulationAdmissionError):   # tau/kp = 1/20 > 0.01
            a.compose_drives(body(tau=1.0), controller_id='balance_surrogate', physics_sequence=1, now_monotonic_s=10.0)
        b = self.drive_arbiter()
        sample(b)
        with self.assertRaises(SimulationAdmissionError):   # q=2.99 + bias 0.05 exceeds q_max 3.0
            b.compose_drives(body(q=2.99, tau=1.0), controller_id='balance_surrogate', physics_sequence=1, now_monotonic_s=10.0)
        c = self.drive_arbiter()
        sample(c)
        with self.assertRaises(SimulationAdmissionError):   # non-positive gain cannot realize a drive
            c.compose_drives(body(kp=0.0), controller_id='balance_surrogate', physics_sequence=1, now_monotonic_s=10.0)

    def test_handover_keeps_implicit_backend_and_needs_new_run(self):
        a = self.drive_arbiter()
        with self.assertRaises(ValueError):
            a.reset_for_run(run_id='episode_001', mode='hybrid', controller_id='balance_surrogate', simulation_authorized=True)
        b = a.reset_for_run(run_id='episode_002', mode='fullbody', controller_id='wbc', simulation_authorized=True)
        self.assertEqual(b.actuation_backend, 'implicit_biased_drive_v1')
        self.assertEqual(b.run_id, 'episode_002')
