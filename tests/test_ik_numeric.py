"""Tests for numeric IK."""
import unittest, math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.ik_numeric import solve, _fk_pose
from arm.constants import HOME_ANGLES


class TestNumericIK(unittest.TestCase):
    HOME = HOME_ANGLES

    def test_identity_returns_seed(self):
        fk = _fk_pose(self.HOME)
        sol = solve(list(fk), self.HOME, position_only=True)
        self.assertIsNotNone(sol)
        for a, b in zip(sol, self.HOME):
            self.assertAlmostEqual(a, b, delta=1.0)

    def test_small_z_lift_position_only(self):
        fk = _fk_pose(self.HOME)
        target = [fk[0], fk[1], fk[2] + 30, fk[3], fk[4], fk[5]]
        sol = solve(target, self.HOME, position_only=True)
        self.assertIsNotNone(sol)
        tip = _fk_pose(sol)
        err = math.hypot(tip[0]-target[0], tip[1]-target[1], tip[2]-target[2])
        self.assertLess(err, 3.0)

    def test_unreachable_returns_none(self):
        sol = solve([1000, 0, 500, 0, 0, 0], self.HOME, position_only=True)
        self.assertIsNone(sol)

    def test_far_reach(self):
        sol = solve([280, 100, 250, 0, 0, 0], self.HOME, position_only=True)
        self.assertIsNotNone(sol)
        tip = _fk_pose(sol)
        err = math.hypot(tip[0]-280, tip[1]-100, tip[2]-250)
        self.assertLess(err, 3.0)


if __name__ == "__main__":
    unittest.main()
