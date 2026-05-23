"""Tests for cartesian path generation, IK chain, and current monitor."""
import unittest, time, threading
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.path_cartesian import linear, lift_translate_lower, _euler_to_quat, _quat_to_euler, _slerp
from arm.ik_path import plan_ik_path, _angle_diff
from arm.current_monitor import CurrentMonitor


class TestCartesianPaths(unittest.TestCase):
    def test_linear_step_count(self):
        wps = linear((0,0,0,0,0,0), (300,0,0,0,0,0), step_mm=30)
        # 300mm / 30mm = 10 steps
        self.assertEqual(len(wps), 10)
        self.assertAlmostEqual(wps[-1][0], 300, delta=0.01)

    def test_lift_skips_for_short_horiz(self):
        # short horizontal → falls back to linear (no extra waypoints)
        wps = lift_translate_lower((0,0,300,0,0,0), (50,30,300,0,0,0))
        # horizontal = ~58mm < 80 threshold → linear
        # linear with 30mm step over ~58mm = 2 steps
        self.assertLessEqual(len(wps), 4)

    def test_lift_uses_three_segments_for_long_horiz(self):
        wps = lift_translate_lower((100,0,200,0,0,0), (-100,0,200,0,0,0))
        # horizontal 200mm > 80 → expect 3 segments worth of waypoints
        self.assertGreater(len(wps), 6)


class TestIKPath(unittest.TestCase):
    START = (100, 0, 300, 0, 0, 0)
    SEED  = [0, 0, -90, 0, 0, 0]

    def test_failing_ik_returns_error(self):
        wps = linear(self.START, (200,0,300,0,0,0))
        joints, ok, msg, bad = plan_ik_path(self.SEED, self.START, wps, ik=lambda t,s: None)
        self.assertFalse(ok)
        self.assertIn("IK 解なし", msg)

    def test_continuous_ik_passes(self):
        def fake_ik(target, seed):
            return [s + 1.0 for s in seed]
        wps = linear(self.START, (200,0,300,0,0,0), step_mm=30)
        joints, ok, msg, bad = plan_ik_path(self.SEED, self.START, wps, ik=fake_ik)
        self.assertTrue(ok, msg)
        self.assertEqual(len(joints), len(wps))

    def test_global_waypoint_cap(self):
        # IK that produces a valid solution → many cart wps → expect cap
        def fake_ik(target, seed): return list(seed)
        wps = linear((0,0,300,0,0,0), (3000, 0, 300, 0, 0, 0), step_mm=10)  # 300 wps
        joints, ok, msg, bad = plan_ik_path(self.SEED, (0,0,300,0,0,0), wps, ik=fake_ik, max_wps=200)
        self.assertFalse(ok)
        self.assertIn("waypoint 超過", msg)

    def test_wrist_flip_detected(self):
        # IK fake that flips J6 by ~340° between waypoints
        flipped = False
        def fake_ik(target, seed):
            nonlocal flipped
            r = list(seed)
            if not flipped:
                flipped = True
                return r
            r[5] = seed[5] + 200  # giant jump → wrist flip
            return r
        wps = linear(self.START, (150,0,300,0,0,0), step_mm=30)
        joints, ok, msg, bad = plan_ik_path(self.SEED, self.START, wps, ik=fake_ik)
        # depending on exact path may report wrist-flip or subdivide-exhausted
        self.assertFalse(ok)


class TestAngleWrap(unittest.TestCase):
    def test_wrap_small(self):
        self.assertAlmostEqual(_angle_diff(10, 5), 5)

    def test_wrap_across_180(self):
        # 170 vs -170 should be 20° not 340°
        self.assertAlmostEqual(abs(_angle_diff(170, -170)), 20)
        self.assertAlmostEqual(abs(_angle_diff(-179, 179)), 2)


class TestSlerp(unittest.TestCase):
    def test_roundtrip_nonsingular(self):
        # Skip pitch=±90 cases (gimbal lock — Euler decomposition non-unique).
        for euler in [(0,0,0), (45, 30, 60), (10, -45, -90), (180, 0, 0)]:
            q = _euler_to_quat(*euler)
            back = _quat_to_euler(*q)
            for a, b in zip(euler, back):
                self.assertAlmostEqual(((a-b+180)%360)-180, 0, delta=1.0,
                                       msg=f"euler {euler} → quat → {back}")

    def test_slerp_endpoints(self):
        qa = _euler_to_quat(0, 0, 0)
        qb = _euler_to_quat(90, 0, 0)
        # at t=0 should equal qa, t=1 should equal qb (up to sign)
        s0 = _slerp(qa, qb, 0)
        s1 = _slerp(qa, qb, 1)
        # dot product ~ 1 (or -1 — quat antipodal)
        d0 = abs(s0[0]*qa[0] + s0[1]*qa[1] + s0[2]*qa[2] + s0[3]*qa[3])
        d1 = abs(s1[0]*qb[0] + s1[1]*qb[1] + s1[2]*qb[2] + s1[3]*qb[3])
        self.assertAlmostEqual(d0, 1.0, delta=1e-3)
        self.assertAlmostEqual(d1, 1.0, delta=1e-3)


class TestCurrentMonitor(unittest.TestCase):
    def test_trigger_on_sustained_overcurrent(self):
        triggered = threading.Event()
        def reader(): return [2000, 100, 100, 100, 100, 100]
        m = CurrentMonitor(reader, on_overcurrent=lambda cs: triggered.set(),
                            threshold_mA=1500, poll_hz=50, sustained=3)
        m.start()
        self.assertTrue(triggered.wait(timeout=1.0))
        m.stop(); m.join(timeout=1.0)
        self.assertTrue(m.triggered)

    def test_no_trigger_below_threshold(self):
        triggered = threading.Event()
        def reader(): return [500, 500, 500, 500, 500, 500]
        m = CurrentMonitor(reader, on_overcurrent=lambda cs: triggered.set(),
                            threshold_mA=1500, poll_hz=50, sustained=3)
        m.start()
        self.assertFalse(triggered.wait(timeout=0.4))
        m.stop(); m.join(timeout=1.0)
        self.assertFalse(m.triggered)


if __name__ == "__main__":
    unittest.main()
