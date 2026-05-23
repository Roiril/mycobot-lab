"""Unit tests for forward kinematics.

Run: python -m pytest tests/
or:  python -m unittest tests.test_kinematics
"""
import math
import unittest
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.kinematics import joint_positions, end_effector, DH, JOINT_LIMITS, link_frames


class TestKinematics(unittest.TestCase):
    def test_dh_length(self):
        self.assertEqual(len(DH), 6)
        self.assertEqual(len(JOINT_LIMITS), 6)

    def test_home_pose_geometry(self):
        # HOME [0,0,-90,0,0,0]: J0 at origin, J1 at (0,0,173.9), J3 ~(0,0,308.9)
        pts = joint_positions([0, 0, -90, 0, 0, 0])
        self.assertEqual(len(pts), 7)
        self.assertAlmostEqual(pts[0][0], 0, delta=0.1)
        self.assertAlmostEqual(pts[0][1], 0, delta=0.1)
        self.assertAlmostEqual(pts[0][2], 0, delta=0.1)
        self.assertAlmostEqual(pts[1][2], 173.9, delta=0.1)
        # J3 at top of upper arm with J3 folded -90°
        self.assertAlmostEqual(pts[3][2], 308.9, delta=0.1)

    def test_link_frames_returns_7(self):
        T = link_frames([0, 0, 0, 0, 0, 0])
        self.assertEqual(len(T), 7)
        # T[0] is identity
        for i in range(4):
            for j in range(4):
                self.assertAlmostEqual(T[0][i][j], 1.0 if i == j else 0.0)

    def test_invalid_angle_count(self):
        with self.assertRaises(ValueError):
            joint_positions([0, 0, 0])

    def test_end_effector_returns_3(self):
        tip = end_effector([0, 0, -90, 0, 0, 0])
        self.assertEqual(len(tip), 3)
        self.assertTrue(all(math.isfinite(v) for v in tip))


if __name__ == "__main__":
    unittest.main()
