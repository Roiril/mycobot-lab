"""Unit tests for safety checks. No hardware needed."""
import unittest
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.safety import check_angles, clamp_angles
from arm.kinematics import JOINT_LIMITS
from arm.constants import HOME_ANGLES


class TestSafety(unittest.TestCase):
    def test_home_passes(self):
        ok, msg, bad = check_angles(HOME_ANGLES)
        self.assertTrue(ok, msg)
        self.assertEqual(bad, [])

    def test_upright_passes(self):
        ok, _, _ = check_angles([0, 0, 0, 0, 0, 0])
        self.assertTrue(ok)

    def test_joint_limit_rejected(self):
        ok, msg, bad = check_angles([200, 0, -90, 0, 0, 0])
        self.assertFalse(ok)
        self.assertIn(1, bad)
        self.assertIn("J1", msg)

    def test_all_six_joints_limits(self):
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            ang = list(HOME_ANGLES)
            ang[i] = hi + 10
            ok, _, bad = check_angles(ang)
            self.assertFalse(ok, f"J{i+1}={hi+10} should fail")
            self.assertIn(i+1, bad)

    def test_floor_strike_blocked(self):
        # J2 forward + J3 lifted → wrist into floor
        ok, msg, bad = check_angles([0, -120, -10, 0, 0, 0])
        self.assertFalse(ok)
        self.assertTrue("床" in msg)

    def test_self_collision_detected(self):
        # extreme fold-back
        ok, msg, bad = check_angles([0, -135, -150, 0, 90, 0])
        self.assertFalse(ok)
        self.assertTrue("自己干渉" in msg)

    def test_clamp_angles(self):
        clamped = clamp_angles([200, -200, 0, 0, 0, 0])
        self.assertEqual(clamped[0], JOINT_LIMITS[0][1])
        self.assertEqual(clamped[1], JOINT_LIMITS[1][0])

    def test_wrong_length(self):
        ok, msg, _ = check_angles([0, 0, 0])
        self.assertFalse(ok)

    def test_bad_joints_indices_are_1based(self):
        # All joint-limit fails should return 1-based indices
        ok, _, bad = check_angles([200, 0, 0, 0, 0, 0])
        self.assertEqual(bad, [1])


class TestPlanner(unittest.TestCase):
    def test_plan_validate(self):
        from arm.planner import plan_and_validate
        wps, ok, msg, bad = plan_and_validate([0, 0, -90, 0, 0, 0], [30, 0, -90, 0, 0, 0])
        self.assertTrue(ok, msg)
        self.assertTrue(len(wps) > 0)
        # last waypoint is target
        for i, v in enumerate([30, 0, -90, 0, 0, 0]):
            self.assertAlmostEqual(wps[-1][i], v, delta=0.01)

    def test_plan_rejects_unsafe_target(self):
        from arm.planner import plan_and_validate
        _, ok, msg, bad = plan_and_validate([0, 0, -90, 0, 0, 0], [200, 0, -90, 0, 0, 0])
        self.assertFalse(ok)
        self.assertIn(1, bad)


if __name__ == "__main__":
    unittest.main()
