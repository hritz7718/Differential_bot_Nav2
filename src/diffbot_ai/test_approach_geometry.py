import math
import unittest

from approach_geometry import approach_pose, deproject_rectified


class GeometryTest(unittest.TestCase):
    def test_optical_center(self):
        self.assertEqual(deproject_rectified(320, 240, 2, 600, 600, 320, 240), (0, 0, 2))

    def test_off_center(self):
        self.assertEqual(deproject_rectified(380, 210, 2, 600, 600, 320, 240), (0.2, -0.1, 2))

    def test_invalid_depth(self):
        for depth in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                deproject_rectified(320, 240, depth, 600, 600, 320, 240)

    def test_standoff(self):
        x, y, yaw = approach_pose((0, 0), (2, 0), 0.5)
        self.assertEqual((x, y, yaw), (1.5, 0.0, 0.0))

    def test_diagonal(self):
        x, y, yaw = approach_pose((0, 0), (1, 1), 0.5)
        self.assertAlmostEqual(math.hypot(1 - x, 1 - y), 0.5)
        self.assertAlmostEqual(yaw, math.pi / 4)

    def test_already_close(self):
        self.assertIsNone(approach_pose((0, 0), (0.4, 0), 0.5))


if __name__ == '__main__':
    unittest.main()
