"""SO-101 driver: translation helpers + MockSo101Driver behavior.

The real LerobotSo101Driver needs lerobot + hardware, so it is not exercised
here; the pure ordered-list <-> lerobot-dict translation (shared by both
drivers) and the in-memory mock are.

Run: python -m unittest tests.test_so101_driver
"""
import unittest
import sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile
from robots.so101.driver import (
    angles_to_action, observation_to_angles, MockSo101Driver,
)


class TestTranslation(unittest.TestCase):
    def test_angles_to_action_keys(self):
        act = angles_to_action([1, 2, 3, 4, 5], gripper=50)
        self.assertEqual(act["shoulder_pan.pos"], 1.0)
        self.assertEqual(act["wrist_roll.pos"], 5.0)
        self.assertEqual(act["gripper.pos"], 50.0)
        self.assertEqual(len(act), profile.NUM_JOINTS + 1)

    def test_angles_to_action_no_gripper(self):
        act = angles_to_action([0, 0, 0, 0, 0])
        self.assertNotIn("gripper.pos", act)
        self.assertEqual(len(act), profile.NUM_JOINTS)

    def test_wrong_length_raises(self):
        with self.assertRaises(ValueError):
            angles_to_action([0, 0, 0])

    def test_roundtrip(self):
        angles = [10, -20, 30, -40, 55]
        obs = angles_to_action(angles, gripper=10)
        self.assertEqual(observation_to_angles(obs), [float(a) for a in angles])


class TestMockDriver(unittest.TestCase):
    def test_connect_write_read(self):
        d = MockSo101Driver()
        d.connect()
        d.write_angles([10, -20, 30, -10, 40], gripper=60)
        self.assertEqual(d.read_angles(), [10, -20, 30, -10, 40])
        self.assertEqual(d.read_gripper(), 60.0)

    def test_clamps_to_limits(self):
        d = MockSo101Driver()
        d.connect()
        d.write_angles([999, -999, 0, 0, 0])
        a = d.read_angles()
        self.assertEqual(a[0], profile.JOINT_LIMITS[0][1])  # clamped to upper
        self.assertEqual(a[1], profile.JOINT_LIMITS[1][0])  # clamped to lower

    def test_gripper_clamped_0_100(self):
        d = MockSo101Driver()
        d.connect()
        d.write_angles([0, 0, 0, 0, 0], gripper=250)
        self.assertEqual(d.read_gripper(), 100.0)

    def test_torque_gate(self):
        d = MockSo101Driver()
        d.connect()
        d.set_torque(False)
        with self.assertRaises(RuntimeError):
            d.write_angles([0, 0, 0, 0, 0])

    def test_write_before_connect_raises(self):
        d = MockSo101Driver()
        with self.assertRaises(RuntimeError):
            d.write_angles([0, 0, 0, 0, 0])

    def test_release_disables_torque(self):
        d = MockSo101Driver()
        d.connect()
        d.release()
        with self.assertRaises(RuntimeError):
            d.write_angles([0, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
