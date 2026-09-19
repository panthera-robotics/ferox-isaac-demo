"""CPU tests for the optional closed_loop.extra_cameras probe-config key (Sprint O model lane proposal): parsing, validation,
byte-identical default, and the link->camera transform conventions. No Isaac import."""
import math
import unittest

from isaac.twin.inspire.extra_cameras import ExtraCameraError, R_LINK_CAM, camera_transform, focal_length, optical_axis_and_up, parse_extra_cameras, rpy_matrix

LEFT = {'name': 'left_wrist', 'link': 'left_wrist_yaw_link', 'xyz': [-0.02, 0.06, 0.0], 'rpy': [-math.pi / 2, 0.0, -0.2617993877991494], 'hfov_deg': 69.0, 'width': 640, 'height': 480}
RIGHT = {'name': 'right_wrist', 'link': 'right_wrist_yaw_link', 'xyz': [-0.02, -0.06, 0.0], 'rpy': [math.pi / 2, 0.0, 0.2617993877991494], 'hfov_deg': 69.0, 'width': 640, 'height': 480}


def close(a, b, tol=1e-3):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class ExtraCamerasConfig(unittest.TestCase):
    def test_absent_or_empty_key_is_the_byte_identical_default(self):
        self.assertEqual(parse_extra_cameras(None), [])
        self.assertEqual(parse_extra_cameras({}), [])
        self.assertEqual(parse_extra_cameras({'schema': 'closed_loop_v1'}), [])
        self.assertEqual(parse_extra_cameras({'extra_cameras': []}), [])
        self.assertEqual(parse_extra_cameras({'extra_cameras': None}), [])

    def test_declared_wrist_cameras_parse_with_defaults_filled(self):
        specs = parse_extra_cameras({'extra_cameras': [LEFT, RIGHT]})
        self.assertEqual([s['name'] for s in specs], ['left_wrist', 'right_wrist'])
        self.assertEqual(specs[1]['link'], 'right_wrist_yaw_link'); self.assertEqual((specs[1]['width'], specs[1]['height']), (640, 480))
        minimal = parse_extra_cameras({'extra_cameras': [{'name': 'cam', 'link': 'torso_link', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}]})[0]
        self.assertEqual((minimal['hfov_deg'], minimal['width'], minimal['height']), (69.0, 640, 480))

    def test_transform_matches_the_probe_camera_convention(self):
        # camera_adapter.json geometry_check: right wrist optical axis (0.9659, 0.2588, 0), image up (0.2588, -0.9659, 0); left mirrored
        axis, up = optical_axis_and_up(RIGHT)
        self.assertTrue(close(axis, [0.9659, 0.2588, 0.0])); self.assertTrue(close(up, [0.2588, -0.9659, 0.0]))
        axis, up = optical_axis_and_up(LEFT)
        self.assertTrue(close(axis, [0.9659, -0.2588, 0.0])); self.assertTrue(close(up, [0.2588, 0.9659, 0.0]))
        T = camera_transform(RIGHT)
        self.assertEqual([T[i][3] for i in range(3)], RIGHT['xyz']); self.assertEqual(T[3], [0.0, 0.0, 0.0, 1.0])
        # identity rpy: optical axis = link +X, up = link +Z (URDF camera link convention through R_LINK_CAM)
        axis, up = optical_axis_and_up({'xyz': [0, 0, 0], 'rpy': [0, 0, 0]})
        self.assertTrue(close(axis, [1, 0, 0])); self.assertTrue(close(up, [0, 0, 1]))
        self.assertEqual(R_LINK_CAM, ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
        R = rpy_matrix(0.1, -0.2, 0.3)
        self.assertTrue(all(abs(sum(R[i][k] * R[j][k] for k in range(3)) - (1.0 if i == j else 0.0)) < 1e-12 for i in range(3) for j in range(3)))

    def test_focal_length_formula_matches_the_policy_camera(self):
        aperture = 20.955
        self.assertAlmostEqual(focal_length(aperture, 69.0), aperture / (2.0 * math.tan(math.radians(69.0) / 2.0)), places=12)

    def test_refusals(self):
        bad = [
            ({'extra_cameras': {'name': 'x'}}, 'must be a list'),
            ({'extra_cameras': [LEFT, dict(LEFT)]}, 'unique identifier'),
            ({'extra_cameras': [dict(LEFT, name='policy')]}, 'unique identifier'),
            ({'extra_cameras': [dict(LEFT, name='left wrist')]}, 'unique identifier'),
            ({'extra_cameras': [dict(LEFT, link='/World/G1/left_wrist_yaw_link')]}, 'bare robot link'),
            ({'extra_cameras': [dict(LEFT, xyz=[0, 0])]}, 'three numbers'),
            ({'extra_cameras': [dict(LEFT, rpy=[0, float('nan'), 0])]}, 'finite'),
            ({'extra_cameras': [dict(LEFT, hfov_deg=0.5)]}, 'hfov_deg'),
            ({'extra_cameras': [dict(LEFT, width=32)]}, 'width'),
            ({'extra_cameras': [dict(LEFT, height=480.0)]}, 'width'),
            ({'extra_cameras': [dict(LEFT, extra='x')]}, 'unknown keys'),
            ({'extra_cameras': [{'name': 'cam', 'link': 'torso_link'}]}, 'xyz'),
            ({'extra_cameras': [dict(LEFT, name='c%d' % i) for i in range(9)]}, 'at most'),
        ]
        for cfg, msg in bad:
            with self.assertRaisesRegex(ExtraCameraError, msg):
                parse_extra_cameras(cfg)


if __name__ == '__main__':
    unittest.main()
