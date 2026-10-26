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

    def test_payload_adds_gravity_and_inertia_without_resetting_state(self):
        import numpy as np
        a = self.FloorArm(start_deg=[0, -35, 45, -15, 0])
        m, d = a.m, a.d
        before_q = d.qpos.copy()
        before_bias = d.qfrc_bias.copy()
        before_mass = m.body_mass.copy()
        before_inertia = np.zeros((m.nv, m.nv))
        # MuJoCo 3.5 takes MjData here (older versions took the packed qM).
        try:
            a.mj.mj_fullM(m, d, before_inertia)
        except TypeError:
            a.mj.mj_fullM(m, before_inertia, d.qM)
        a.set_payload(0.3)
        after_inertia = np.zeros_like(before_inertia)
        try:
            a.mj.mj_fullM(m, d, after_inertia)
        except TypeError:
            a.mj.mj_fullM(m, after_inertia, d.qM)
        self.assertAlmostEqual(float(m.body_mass.sum() - before_mass.sum()), 0.3)
        self.assertGreater(after_inertia[1, 1], before_inertia[1, 1])
        self.assertGreater(abs(d.qfrc_bias[1]), abs(before_bias[1]))
        np.testing.assert_allclose(d.qpos, before_q)
        self.assertEqual(a.world.payload_kg, 0.0)  # no extra external gravity
        a.set_payload(0.0)
        np.testing.assert_allclose(m.body_mass, before_mass)
        np.testing.assert_allclose(d.qfrc_bias, before_bias, atol=1e-10)

    def test_payload_updates_do_not_accumulate_mass(self):
        a = self.FloorArm()
        bare = float(a.m.body_mass.sum())
        for load in (0.2, 0.4, 0.2):
            a.set_payload(load)
            self.assertAlmostEqual(float(a.m.body_mass.sum()), bare + load)


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

    def test_per_joint_controls_and_thermal_parameters_are_independent(self):
        sim = self.mod.Sim()
        sim.set_regs({"Torque_Limit": 200}, "elbow_flex")
        sim.set_motor("elbow_flex", {"r_th": 10.0, "c_th": 50.0})
        st = sim.state(0)
        self.assertEqual(st["joints"][2]["regs"]["Torque_Limit"], 200)
        self.assertEqual(st["joints"][1]["regs"]["Torque_Limit"], 1000)
        self.assertEqual(st["joints"][2]["physics"]["thermal_tau_s"], 500)
        self.assertEqual(st["joints"][1]["physics"]["thermal_tau_s"], 87)
        for params in ({"r_th": float("nan")}, {"c_th": 0}, {"R": 1}):
            with self.assertRaises(ValueError):
                sim.set_motor("elbow_flex", params)
        with self.assertRaises(ValueError):
            sim.set_regs({}, "not-a-servo")

    def test_environment_changes_preserve_heat_time_and_payload_state(self):
        sim = self.mod.Sim()
        sim.arm.servos[2].temp_c = 55.0
        sim.set_world(0.3, None, 20.0)
        self.assertEqual(sim.arm.servos[2].temp_c, 55.0)
        self.assertEqual(sim.arm.servos[2].mp.ambient_c, 20.0)
        self.assertEqual(sim.arm.payload_kg, 0.3)
        self.assertEqual(sim.arm.t, 0.0)
        generation = sim.generation
        sim.reset()
        self.assertGreater(sim.generation, generation)
        self.assertEqual(sim.arm.servos[2].temp_c, 20.0)
        self.assertEqual(sim.arm.payload_kg, 0.3)

    def test_temperature_history_uses_physics_time_and_difference_cursor(self):
        sim = self.mod.Sim()
        self._run(sim, 2.1)
        st = sim.state(0)
        self.assertEqual(len(st["thermal_hist"]), 3)
        self.assertEqual(len(st["thermal_hist"][0]), 7)
        self.assertEqual(sim.state(st["seq"], st["thermal_seq"])["thermal_hist"], [])
        before_t = sim.arm.t
        sim.set_run(True, 0.5)
        self.assertEqual(sim.arm.t, before_t)
        self.assertEqual(sim.state(0)["thermal_seq"], st["thermal_seq"])

    def test_analysis_is_conditional_and_does_not_advance_physics(self):
        sim = self.mod.Sim()
        analysis = sim.analysis("elbow_flex")
        self.assertEqual(analysis["forecast"]["assumption"], "constant_loss")
        self.assertEqual(analysis["forecast"]["points"][-1][0], 600)
        self.assertEqual(len(analysis["envelope"]), 41)
        self.assertEqual(sim.arm.t, 0.0)
        sim.set_motor("elbow_flex", {"c_th": 100})
        self.assertIsNot(sim.analysis("elbow_flex"), analysis)
        with self.assertRaises(ValueError):
            sim.analysis("missing")


if __name__ == "__main__":
    unittest.main()
