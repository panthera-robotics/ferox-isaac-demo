"""CPU guards for the physical writer probe; importing never boots Isaac."""
import importlib.util
import math
from pathlib import Path
import unittest
import tempfile

spec = importlib.util.spec_from_file_location('assembled_writer_probe', Path(__file__).parents[1]/'probes/assembled_writer.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class PhysicalWriterGuards(unittest.TestCase):
    def test_immutable_template_remains_unchanged_while_runtime_is_writable(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'input';source.mkdir();(source/'poses').mkdir()
            profile=source/'profile.json';profile.write_text('{"run_id":"template"}')
            pose=source/'poses/home.yaml';pose.write_text('fixture: true')
            for p in [profile,pose]:p.chmod(0o444)
            for p in [source,source/'poses']:p.chmod(0o555)
            try:
                destination=Path(directory)/'runtime';probe.copy_writable_template(source,destination)
                self.assertEqual((destination/'profile.json').read_bytes(),profile.read_bytes())
                self.assertEqual((destination/'profile.json').stat().st_mode&0o777,0o644)
                self.assertEqual((destination/'poses').stat().st_mode&0o777,0o755)
                (destination/'profile.json').write_text('{"run_id":"admitted"}')
                (destination/'recording').mkdir()
                self.assertEqual(profile.read_text(),'{"run_id":"template"}')
                self.assertEqual(profile.stat().st_mode&0o777,0o444)
            finally:
                for p in [source,source/'poses']:p.chmod(0o755)

    def fixture(self):
        names = ['joint_%02d' % i for i in range(53)]
        limits = {n: {'lower': -1., 'upper': 1., 'velocity': 10.} for n in names}
        mimic = {names[-1]: {'parent': names[-2], 'multiplier': 1., 'offset': 0.}}
        return names, [0.]*53, [0.]*53, [0.]*53, limits, mimic

    def test_measured_mimic_divergence_stops_episode(self):
        args = self.fixture()
        args[1][-1] = .04
        with self.assertRaisesRegex(ValueError, 'mimic'):
            probe.observed_checks(*args)

    def test_fast_body_motion_stops_before_any_controller_decision(self):
        args = self.fixture()
        args[2][0] = 5.1
        with self.assertRaisesRegex(ValueError, 'velocity'):
            probe.observed_checks(*args)

    def test_source_velocity_can_be_stricter_than_global_guard(self):
        args = self.fixture()
        args[4][args[0][0]]['velocity'] = 1.
        args[2][0] = 1.1
        with self.assertRaisesRegex(ValueError, 'velocity'):
            probe.observed_checks(*args)

    def test_nan_measured_effort_is_an_abort_not_zero_effort(self):
        args = self.fixture()
        args[3][0] = math.nan
        with self.assertRaisesRegex(ValueError, 'finite'):
            probe.observed_checks(*args)

    def test_joint_limit_overshoot_is_retained_as_failure(self):
        args = self.fixture()
        args[1][0] = 1.031
        with self.assertRaisesRegex(ValueError, 'limit'):
            probe.observed_checks(*args)

    def test_quaternion_is_not_silently_normalized(self):
        with self.assertRaisesRegex(ValueError, 'unit'):
            probe.pose_matrix([0., 0., 0., 0., 0., 0., 2.])

    def test_named_frame_comparison_preserves_known_millimetre_difference(self):
        import numpy as np
        donor, planner = np.eye(4), np.eye(4)
        planner[0,3] = .000784590957
        planner[2,3] = .000030826663
        error = probe.frame_error(donor, planner)
        self.assertAlmostEqual(error['translation_m'], .000785196315, places=12)
        self.assertEqual(error['rotation_rad'], 0.)

    def test_reflection_is_not_a_valid_frame(self):
        import numpy as np
        reflection = np.eye(4); reflection[0,0] = -1.
        with self.assertRaisesRegex(ValueError, 'proper'):
            probe.frame_error(np.eye(4), reflection)


if __name__ == '__main__':
    unittest.main()


class ActuationBackendConfigTests(unittest.TestCase):
    def config(self, **extra):
        names = ['j%d' % i for i in range(29)]
        value = {'schema_version': 1, 'hardware_authorized': False,
            'private_driver_path': '/workspace/driver', 'private_dependency_path': '/workspace/deps',
            'private_profile_path': '/workspace/fixture/profile.json', 'planner_frames_path': '/workspace/fixture/frames.json',
            'planner_frames_sha256': 'a' * 64,
            'body_home_rad': {n: 0. for n in names}, 'kp_nm_rad': {n: 60. for n in names},
            'kd_nm_s_rad': {n: 1.5 for n in names}, 'tau_ff_nm': {n: 0. for n in names},
            'gain_provenance': 'declared', 'feedforward_provenance': 'declared',
            'workflow_mode': 'complete', 'letter_height_m': .02, 'maximum_steps': 1000, 'maximum_wall_s': 300.}
        value.update(extra)
        return value

    def test_backend_defaults_to_explicit_and_only_versioned_backends_are_admitted(self):
        probe.validate_config(self.config())
        probe.validate_config(self.config(actuation_backend='explicit_pd'))
        probe.validate_config(self.config(actuation_backend='implicit_biased_drive_v1'))
        for bad in ('implicit', 'implicit_biased_drive_v2', '', None, 1):
            with self.assertRaises(ValueError):
                probe.validate_config(self.config(actuation_backend=bad))
        with self.assertRaises(ValueError):
            probe.validate_config(self.config(unknown_key=1))

    def test_contact_writing_block_is_explicit_bounded_and_mounted(self):
        block = {'held_marker_grasp_path': '/workspace/writer-fixture/held_marker_grasp.json', 'preload_support_s': .8,
                 'grasp_finger_kp_nm_rad': 1., 'grasp_finger_kd_nm_s_rad': .05, 'provenance': 'v9b measured grasp'}
        probe.validate_config(self.config(contact_writing=block))
        for bad in ({**block, 'preload_support_s': 2.}, {**block, 'grasp_finger_kp_nm_rad': 20.}, {**block, 'held_marker_grasp_path': '/tmp/x.json'},
                    {**block, 'provenance': ''}, {k: v for k, v in block.items() if k != 'provenance'}, {**block, 'extra': 1}):
            with self.assertRaises(ValueError):
                probe.validate_config(self.config(contact_writing=bad))

    def test_wall_age_tolerance_is_declared_and_bounded(self):
        probe.validate_config(self.config(maximum_wall_age_s=1.0))
        for bad in (.05, 6., float('nan'), '1'):
            with self.assertRaises(ValueError):
                probe.validate_config(self.config(maximum_wall_age_s=bad))

    def test_idle_hand_hold_gains_are_declared_and_bounded_by_source_effort(self):
        probe.validate_config(self.config(hand_hold_kp_nm_rad=5., hand_hold_kd_nm_s_rad=.5))
        for key, bad in (('hand_hold_kp_nm_rad', 11.), ('hand_hold_kp_nm_rad', 0.), ('hand_hold_kd_nm_s_rad', 2.), ('hand_hold_kd_nm_s_rad', float('nan'))):
            with self.assertRaises(ValueError):
                probe.validate_config(self.config(**{key: bad}))


class Float32PoseReadbackTests(unittest.TestCase):
    def test_float32_unit_quaternion_readback_is_a_valid_frame(self):
        import numpy as np
        q = np.array([.3, -.5, .2, .7853], dtype=np.float64); q /= np.linalg.norm(q)
        q32 = q.astype(np.float32).astype(float)          # PhysX float32 readback
        frame = probe.pose_matrix([.1, .2, .3, *q32])
        self.assertLess(abs(np.linalg.norm(q32) - 1.), 1e-6)
        result = probe.frame_error(frame, frame)
        self.assertAlmostEqual(result['translation_m'], 0.)
        with self.assertRaisesRegex(ValueError, 'unit'):
            probe.pose_matrix([.1, .2, .3, *(q * 1.01)])
