"""CPU guards for the physical writer probe; importing never boots Isaac."""
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('assembled_writer_probe', Path(__file__).parents[1]/'probes/assembled_writer.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class PhysicalWriterGuards(unittest.TestCase):
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
