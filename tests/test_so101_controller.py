"""SO-101 motion controller tests (against the in-memory mock driver)."""
import math
import unittest
import sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile
from robots.so101.driver import MockSo101Driver
from robots.so101.controller import So101Controller, plan_joint_path
from robots.so101.kinematics import end_effector


def _mk():
    d = MockSo101Driver()
    d.connect()
    # no-op sleep: tests shouldn't pay real speed pacing (~5s across the suite)
    return So101Controller(d, sleep_fn=lambda s: None)


class TestPlanner(unittest.TestCase):
    def test_step_bound(self):
        wps = plan_joint_path([0, 0, 0, 0, 0], [40, 0, 0, 0, 0], max_step_deg=4.0)
        self.assertEqual(wps[-1], [40, 0, 0, 0, 0])
        prev = [0, 0, 0, 0, 0]
        for wp in wps:
            self.assertLessEqual(max(abs(a - b) for a, b in zip(wp, prev)), 4.0 + 1e-9)
            prev = wp

    def test_no_motion_when_already_there(self):
        wps = plan_joint_path([5, 5, 5, 5, 5], [5, 5, 5, 5, 5])
        self.assertEqual(wps, [[5, 5, 5, 5, 5]])


class TestController(unittest.TestCase):
    def test_move_to_angles_reaches(self):
        c = _mk()
        ok, msg = c.move_to_angles([20, -30, 40, -10, 25])
        self.assertTrue(ok, msg)
        for a, b in zip(c.current_angles(), [20, -30, 40, -10, 25]):
            self.assertAlmostEqual(a, b, places=6)

    def test_rejects_out_of_limit_target(self):
        c = _mk()
        ok, msg = c.move_to_angles([200, 0, 0, 0, 0])
        self.assertFalse(ok)
        self.assertIn("unsafe", msg)
        # driver must not have moved toward the bad target
        self.assertEqual(c.current_angles(), list(profile.HOME_ANGLES))

    def test_rejects_below_floor_target(self):
        c = _mk()
        ok, msg = c.move_to_angles([0, 90, 0, 0, 0])  # tip below floor
        self.assertFalse(ok)

    def test_every_step_is_safe_and_recorded(self):
        c = _mk()
        seen = []
        ok, msg = c.move_to_angles([30, -40, 50, -20, 0], on_step=seen.append)
        self.assertTrue(ok, msg)
        self.assertGreater(len(seen), 1)
        self.assertEqual(seen[-1], c.current_angles())

    def test_move_to_position_reaches(self):
        c = _mk()
        # a reachable target from a known pose
        target = end_effector([15, -35, 45, -15, 0])
        ok, msg = c.move_to_position(target)
        self.assertTrue(ok, msg)
        reached = end_effector(c.current_angles())
        self.assertLess(math.dist(reached, target), 3.0)

    def test_home(self):
        c = _mk()
        c.move_to_angles([30, -30, 30, -30, 30])
        ok, _ = c.home()
        self.assertTrue(ok)
        self.assertEqual(c.current_angles(), list(profile.HOME_ANGLES))

    def test_unreachable_position(self):
        c = _mk()
        ok, msg = c.move_to_position([2000, 0, 300])
        self.assertFalse(ok)
        self.assertIn("no IK", msg)


if __name__ == "__main__":
    unittest.main()
