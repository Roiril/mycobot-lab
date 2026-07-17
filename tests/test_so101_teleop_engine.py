"""SO-101 teleop engine: pure smoothing + stepping logic.

Runs on plain Python 3.10 (no lerobot, no hardware) — the engine's lerobot
imports are lazy and only the pure paths are exercised here via a FakeBus.

Run: python -m unittest tests.test_so101_teleop_engine
"""
import sys
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.teleop_engine import (
    OneEuroFilter, OneEuroConfig, TeleopEngine, TeleopConfig, JOINTS,
)


class FakeBus:
    """In-memory stand-in for FeetechMotorsBus (records writes, echoes goals)."""

    def __init__(self, present=None):
        self.present = dict(present) if present else {n: 0 for n in JOINTS}
        self.writes = []          # (data_name, motor, value)
        self.sync_writes = []     # list of goal dicts
        self.torque = {n: 0 for n in JOINTS}

    def sync_read(self, data_name, normalize=False, num_retry=1):
        return dict(self.present)

    def sync_write(self, data_name, values, normalize=False):
        self.sync_writes.append(dict(values))
        for k, v in values.items():      # follower "reaches" the goal instantly
            self.present[k] = v

    def read(self, data_name, motor, normalize=False):
        if data_name == "Present_Position":
            return self.present[motor]
        return 0

    def write(self, data_name, motor, value, normalize=False):
        self.writes.append((data_name, motor, value))
        if data_name == "Torque_Enable":
            self.torque[motor] = value


def make_engine(ranges=None, present=None, **cfg_kw):
    cfg = TeleopConfig(torque_stagger_s=0.0, **cfg_kw)
    ranges = ranges or {n: (0, 4095) for n in JOINTS}
    lead, foll = FakeBus(), FakeBus(present)
    return TeleopEngine(lead, foll, ranges, config=cfg), lead, foll


# ---------------------------------------------------------------------------
class TestOneEuroFilter(unittest.TestCase):
    def test_first_sample_passthrough(self):
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        self.assertEqual(f.filter(1234.0, 0.0), 1234.0)

    def test_converges_to_constant(self):
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        out = 0.0
        for i in range(200):
            out = f.filter(2000.0, i / 60)
        self.assertAlmostEqual(out, 2000.0, delta=1.0)

    def test_rejects_jitter(self):
        """Alternating +-8 tick noise on a constant is attenuated."""
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        base, dev_in, dev_out = 1500.0, 0.0, 0.0
        for i in range(400):
            noisy = base + (8.0 if i % 2 else -8.0)
            out = f.filter(noisy, i / 60)
            if i > 100:                    # after settling
                dev_in += abs(noisy - base)
                dev_out += abs(out - base)
        self.assertLess(dev_out, dev_in * 0.3)   # >70% jitter removed

    def test_low_lag_on_fast_ramp(self):
        """A 1000 tick/s ramp (~90 deg/s) tracks within a few ticks at steady state."""
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        dt, vel = 1 / 60, 1000.0
        out = 0.0
        for i in range(120):
            x = vel * i * dt
            out = f.filter(x, i * dt)
        x_final = vel * 119 * dt
        # lag ~ vel/(2*pi*cutoff); cutoff ~ 1 + 0.01*1000 = 11Hz -> ~14 ticks
        self.assertLess(abs(out - x_final), 25.0)


# ---------------------------------------------------------------------------
class TestComputeGoals(unittest.TestCase):
    def test_step_clamp_far_target(self):
        eng, _, _ = make_engine(max_step_ticks=170)
        eng.last_goal = {n: 0 for n in JOINTS}
        goals = eng.compute_goals({n: 2000 for n in JOINTS}, now=0.0)
        for n in JOINTS:
            self.assertEqual(goals[n], 170)          # clamped to +max_step
            self.assertEqual(eng.last_goal[n], 170)

    def test_deadband_skips_write(self):
        eng, _, _ = make_engine(deadband_ticks=2)
        eng.last_goal = {n: 1000 for n in JOINTS}
        goals = eng.compute_goals({n: 1001 for n in JOINTS}, now=0.0)
        self.assertEqual(goals, {})                   # 1-tick move skipped
        for n in JOINTS:
            self.assertEqual(eng.last_goal[n], 1000)  # unchanged

    def test_range_clamp(self):
        ranges = {n: (500, 1500) for n in JOINTS}
        eng, _, _ = make_engine(ranges=ranges, max_step_ticks=170)
        eng.last_goal = {n: 1400 for n in JOINTS}
        goals = eng.compute_goals({n: 3000 for n in JOINTS}, now=0.0)
        for n in JOINTS:
            self.assertEqual(goals[n], 1500)          # range cap, not 1400+170
            self.assertEqual(eng.last_goal[n], 1500)

    def test_converges_over_cycles(self):
        eng, _, _ = make_engine(max_step_ticks=170, deadband_ticks=2)
        eng.last_goal = {n: 0 for n in JOINTS}
        target = 1000
        for i in range(50):
            eng.compute_goals({n: target for n in JOINTS}, now=i / 60)
        for n in JOINTS:
            self.assertAlmostEqual(eng.last_goal[n], target, delta=2)

    def test_last_goal_not_reset_by_present(self):
        """Present-position drift must not move last_goal (only compute does)."""
        eng, _, _ = make_engine()
        eng.last_goal = {n: 800 for n in JOINTS}
        eng.follower_bus.present = {n: 200 for n in JOINTS}   # follower lagging
        goals = eng.compute_goals({n: 800 for n in JOINTS}, now=0.0)
        self.assertEqual(goals, {})                   # already at goal
        for n in JOINTS:
            self.assertEqual(eng.last_goal[n], 800)   # not rewound to 200


# ---------------------------------------------------------------------------
class TestEngineIO(unittest.TestCase):
    def test_start_teleop_seeds_and_enables(self):
        present = {n: 512 for n in JOINTS}
        eng, lead, foll = make_engine(present=present)
        eng.start_teleop()
        self.assertTrue(eng.active)
        self.assertEqual(eng.last_goal, present)                  # seeded from follower
        for n in JOINTS:                                          # caps written
            self.assertIn(("Torque_Limit", n, ec_limit(n)), foll.writes)
            self.assertEqual(foll.torque[n], 1)                   # staged torque on

    def test_caps_written_before_torque(self):
        eng, lead, foll = make_engine()
        eng.start_teleop()
        first_torque = next(i for i, w in enumerate(foll.writes)
                            if w[0] == "Torque_Enable")
        last_cap = max(i for i, w in enumerate(foll.writes)
                       if w[0] in ("Torque_Limit", "Acceleration", "Goal_Velocity"))
        self.assertLess(last_cap, first_torque)       # all caps precede any enable

    def test_step_writes_only_changed_joints(self):
        eng, lead, foll = make_engine(deadband_ticks=2)
        eng.start_teleop()
        foll.sync_writes.clear()
        lead.present = {n: 2000 for n in JOINTS}       # leader jumps
        eng.step(now=1.0)
        self.assertEqual(len(foll.sync_writes), 1)
        self.assertTrue(foll.sync_writes[0])           # non-empty goal dict

    def test_periodic_follower_read(self):
        eng, lead, foll = make_engine(follower_read_every=2)
        eng.start_teleop()
        lead.present = {n: 100 for n in JOINTS}
        eng.step(now=1.0)                              # cycle 1: no follower read
        res = eng.step(now=2.0)                        # cycle 2: follower read
        self.assertIn("follower", res)
        self.assertIsInstance(eng.follower_lag_ticks, int)

    def test_measured_hz(self):
        eng, lead, foll = make_engine()
        eng.start_teleop()
        for i in range(10):
            eng.step(now=i / 60.0)                     # 60 Hz spacing
        self.assertAlmostEqual(eng.measured_hz, 60.0, delta=5.0)


def ec_limit(joint):
    from robots.so101.teleop_engine import TORQUE_LIMITS
    return TORQUE_LIMITS[joint]


if __name__ == "__main__":
    unittest.main()
