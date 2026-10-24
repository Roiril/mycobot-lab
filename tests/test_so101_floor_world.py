"""FloorArm (every link touches the floor) and the web simulator's Sim wrapper.

Skipped automatically if mujoco is not installed (optional sim dependency).

Run: python -m unittest tests.test_so101_floor_world
"""
import importlib.util
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

try:
    import mujoco  # noqa: F401
    _HAVE_MUJOCO = True
except ImportError:
    _HAVE_MUJOCO = False


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed")
class FloorArmTest(unittest.TestCase):
    def setUp(self):
        from robots.so101.sim.floor_world import FloorArm, FloorParams
        self.FloorArm, self.FloorParams = FloorArm, FloorParams

    def test_floor_is_base_bottom(self):
        a = self.FloorArm()
        self.assertAlmostEqual(a.floor_z, -0.0024, places=4)

    def test_limp_arm_comes_to_rest_on_the_floor(self):
        a = self.FloorArm(start_deg=[0, 0, 0, 0, 0])      # stretched out, torque off
        a.advance(3.0)
        depth = a.floor_z - a.contact_points_world()[:, 2].min()
        self.assertGreater(len(a.contacts), 0)
        self.assertLess(depth, 0.002)                     # held up, < 2 mm into the floor
        self.assertLess(max(abs(v) for v in a.d.qvel[:5]), 0.05)   # and at rest

    def test_without_floor_the_limp_arm_falls_through(self):
        a = self.FloorArm(start_deg=[0, 0, 0, 0, 0], floor=self.FloorParams(enabled=False))
        a.advance(1.5)
        self.assertEqual(a.contacts, [])
        self.assertLess(a.contact_points_world()[:, 2].min(), a.floor_z - 0.01)

    def test_raised_arm_does_not_touch(self):
        a = self.FloorArm(start_deg=[0, -35, 45, -15, 0])
        for s in a.servos:
            s.write("Torque_Enable", 1, s.setpoint)
        a.advance(1.0)
        self.assertEqual(a.contacts, [])


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed")
class SimServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("so101_sim_server", ROOT / "scripts" / "so101_sim_server.py")
        cls.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)

    def _run(self, sim, seconds):
        for _ in range(int(seconds / 0.001)):
            sim.arm.step(0.001)
            if sim.arm.t >= sim._next_sample:
                sim._sample()
                sim._next_sample = sim.arm.t + self.mod.HIST_DT

    def test_goal_is_reached_and_traced(self):
        sim = self.mod.Sim()
        sim.set_goals({"elbow_flex": 60.0})
        self._run(sim, 1.5)
        st = sim.state(0)
        el = next(j for j in st["joints"] if j["name"] == "elbow_flex")
        self.assertAlmostEqual(el["goal"], 60.0, delta=0.1)
        self.assertAlmostEqual(el["q"], 60.0, delta=5.0)     # P-only loop sags under gravity
        self.assertGreater(len(st["hist"]), 250)
        self.assertEqual(len(st["hist"][0]), 1 + 6 * 5)
        self.assertEqual(len(st["bodies"]), 7)

    def test_goal_is_clamped_to_joint_range(self):
        sim = self.mod.Sim()
        sim.set_goals({"shoulder_pan": 500.0})
        st = sim.state(0)
        pan = st["joints"][0]
        self.assertAlmostEqual(pan["goal"], pan["range"][1], delta=0.1)

    def test_torque_limit_reaches_every_servo(self):
        sim = self.mod.Sim()
        sim.set_regs({"Torque_Limit": 300, "Bogus": 5})
        self.assertTrue(all(s.regs.torque_limit == 300 for s in sim.arm.servos))
        self.assertNotIn("Bogus", sim.regs)

    def test_torque_off_lets_the_arm_drop(self):
        sim = self.mod.Sim()
        q0 = math.degrees(sim.arm.d.qpos[1])
        sim.set_torque(False)
        self._run(sim, 0.5)
        self.assertGreater(abs(math.degrees(sim.arm.d.qpos[1]) - q0), 5.0)

    def test_mesh_binary_layout(self):
        sim = self.mod.Sim()
        b = sim.mesh_bin(0)
        import struct
        vn, fn = struct.unpack("<II", b[:8])
        self.assertEqual(len(b), 8 + vn * 12 + fn * 12)


if __name__ == "__main__":
    unittest.main()
