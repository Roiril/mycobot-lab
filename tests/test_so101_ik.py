"""SO-101 position IK: FK round-trip verification.

Gold-standard check: pick random in-limit joint angles, compute the TCP via FK,
then ask IK to recover joints for that position and confirm FK of the solution
lands back on the target. This validates IK + FK together, offline, no hardware.

Run: python -m unittest tests.test_so101_ik
"""
import math
import unittest
import sys, pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile
from robots.so101.kinematics import end_effector
from robots.so101.ik import solve_position, solve


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class TestPositionIK(unittest.TestCase):
    def test_fk_roundtrip(self):
        rng = np.random.default_rng(42)
        fails = []
        n = 40
        for _ in range(n):
            # sample within 70% of each joint limit to avoid singular extremes
            ang = [rng.uniform(0.7 * lo, 0.7 * hi) for (lo, hi) in profile.JOINT_LIMITS]
            target = end_effector(ang)
            sol = solve_position(target, seed=profile.HOME_ANGLES)
            if sol is None:
                fails.append((ang, target, "no solution"))
                continue
            reached = end_effector(sol)
            err = _dist(target, reached)
            if err > 3.0:
                fails.append((ang, target, f"{err:.2f}mm"))
        self.assertEqual(fails, [], msg=f"{len(fails)}/{n} round-trips failed: {fails[:3]}")

    def test_solution_respects_limits(self):
        target = end_effector([20, -40, 50, -20, 30])
        sol = solve_position(target, seed=profile.HOME_ANGLES)
        self.assertIsNotNone(sol)
        for a, (lo, hi) in zip(sol, profile.JOINT_LIMITS):
            self.assertGreaterEqual(a, lo - 1e-6)
            self.assertLessEqual(a, hi + 1e-6)

    def test_unreachable_returns_none(self):
        self.assertIsNone(solve_position([1000.0, 0.0, 300.0]))
        self.assertIsNone(solve_position([0.0, 0.0, 2000.0]))

    def test_seed_defaults_to_home(self):
        target = end_effector([0, -30, 40, -10, 0])
        sol = solve_position(target)  # no seed -> HOME_ANGLES
        self.assertIsNotNone(sol)
        self.assertEqual(len(sol), profile.NUM_JOINTS)

    def test_position_only_ignores_orientation(self):
        # A 6-vec target with absurd orientation must still solve for position.
        target = list(end_effector([10, -20, 30, -10, 45])) + [180.0, 0.0, -90.0]
        sol = solve(target, profile.HOME_ANGLES, position_only=True)
        self.assertIsNotNone(sol)
        self.assertLess(_dist(end_effector(sol), target), 3.0)


if __name__ == "__main__":
    unittest.main()
