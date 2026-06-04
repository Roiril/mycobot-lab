"""SO-101 safety checks: joint limits, floor clearance, self-collision.

Joint-limit and floor logic are exact and tested precisely. Self-collision uses
provisional clearances; the test only guards that it is not overzealous (does
not reject normal poses wholesale) — real-collision accuracy needs hardware.

Run: python -m unittest tests.test_so101_safety
"""
import unittest
import sys, pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile, safety
from robots.so101.kinematics import end_effector

# Representative in-limit, above-floor poses that must always validate.
GOOD_POSES = [
    [0, 0, 0, 0, 0],         # HOME, tip ~ (391,0,226)
    [0, -90, 0, 0, 0],       # up
    [0, -45, 60, -15, 0],    # mid reach
    [0, 80, -90, 0, 0],      # folded-forward but open
]


class TestLimits(unittest.TestCase):
    def test_good_poses_pass(self):
        for a in GOOD_POSES:
            ok, msg, bad = safety.check_angles(a)
            self.assertTrue(ok, msg=f"{a} unexpectedly rejected: {msg}")

    def test_over_limit_rejected(self):
        a = [200.0, 0, 0, 0, 0]  # shoulder_pan past +110
        ok, msg, bad = safety.check_angles(a)
        self.assertFalse(ok)
        self.assertEqual(bad, [1])

    def test_under_limit_rejected(self):
        a = [0, 0, 0, 0, -200.0]  # wrist_roll past -157
        ok, msg, bad = safety.check_angles(a)
        self.assertFalse(ok)
        self.assertEqual(bad, [5])

    def test_wrong_length_rejected(self):
        ok, msg, bad = safety.check_angles([0, 0, 0, 0, 0, 0])  # 6-DoF habit
        self.assertFalse(ok)
        self.assertIn("length", msg)

    def test_clamp(self):
        clamped = safety.clamp_angles([200, -200, 0, 0, 0])
        for a, (lo, hi) in zip(clamped, profile.JOINT_LIMITS):
            self.assertGreaterEqual(a, lo)
            self.assertLessEqual(a, hi)


class TestFloor(unittest.TestCase):
    def test_below_floor_rejected(self):
        a = [0, 90, 0, 0, 0]  # reaches down, tip ~ (179,0,-206)
        self.assertLess(end_effector(a)[2], profile.SAFETY["floor_z_mm"])
        ok, msg, bad = safety.check_angles(a)
        self.assertFalse(ok)
        self.assertIn("床", msg)

    def test_floor_only_skips_collision(self):
        # floor-only must pass a limit+floor-valid pose regardless of collision.
        for a in GOOD_POSES:
            ok, _, _ = safety.check_angles_floor_only(a)
            self.assertTrue(ok)


class TestSelfCollisionNotOverzealous(unittest.TestCase):
    def test_collision_reject_rate_is_bounded(self):
        # Of poses that already pass joint-limits + floor, only a small minority
        # should be rejected by self-collision. A high rate would mean the
        # clearance is too aggressive and would cripple normal motion.
        rng = np.random.default_rng(7)
        floor_ok = 0
        collision_rejects = 0
        for _ in range(2000):
            a = [rng.uniform(lo, hi) for (lo, hi) in profile.JOINT_LIMITS]
            if not safety.check_angles_floor_only(a)[0]:
                continue
            floor_ok += 1
            if not safety.check_angles(a)[0]:
                collision_rejects += 1
        rate = collision_rejects / max(1, floor_ok)
        self.assertGreater(floor_ok, 1000, msg="too few floor-valid poses sampled")
        self.assertLess(rate, 0.20, msg=f"self-collision rejects {rate:.1%} of valid poses (too aggressive)")


if __name__ == "__main__":
    unittest.main()
