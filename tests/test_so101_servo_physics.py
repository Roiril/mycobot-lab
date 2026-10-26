"""Servo drive/thermal estimates without physical hardware or MuJoCo."""
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.sim.servo_sim import MotorParams, SimFollowerBus, SimServo, TICKS_PER_RAD


class ServoPhysicsTest(unittest.TestCase):
    def servo(self, **kwargs):
        return SimServo("elbow_flex", 3, MotorParams(**kwargs), 2048.0)

    def driven(self, **kwargs):
        s = self.servo(**kwargs)
        s.regs.torque_enable = 1
        s.regs.goal_position = 4095
        s.setpoint = 4095.0
        s.regs.d = 0
        s.mp.overcurrent_a = 100.0
        s.regs.protection_time = 100000
        return s

    def test_double_current_quadruples_copper_loss(self):
        s = self.servo()
        s.current = 1.0
        loss = s.heat_w
        s.current = 2.0
        self.assertAlmostEqual(s.heat_w, 4.0 * loss)
        self.assertAlmostEqual(s.heat_w, 4.0 * s.copper_resistance)
        self.assertNotAlmostEqual(s.copper_resistance, s.resistance)

    def test_cooling_is_exponential_and_reaches_ambient(self):
        s = self.servo()
        s.temp_c = 65.0
        tau = s.mp.r_th * s.mp.c_th
        s._heat(tau)
        self.assertAlmostEqual(s.temp_c, 27.0 + 38.0 / math.e)
        s._heat(100.0 * tau)
        self.assertAlmostEqual(s.temp_c, s.mp.ambient_c)
        self.assertEqual(s.energy_j, 0.0)

    def test_heating_and_energy_are_independent_of_time_step(self):
        coarse, fine = self.servo(), self.servo()
        for s in (coarse, fine):
            s.current = 2.0
            s.regs.torque_enable = 1
            s.regs.max_temperature = 1000
        coarse._heat(30.0)
        for _ in range(3000):
            fine._heat(0.01)
        self.assertAlmostEqual(coarse.temp_c, fine.temp_c, places=10)
        self.assertAlmostEqual(coarse.energy_j, fine.energy_j, places=9)
        self.assertAlmostEqual(coarse.energized_s, fine.energized_s, places=9)

    def test_heat_balance_accounts_for_heating_and_cooling(self):
        s = self.servo()
        s.temp_c = 50.0
        s.current = 2.0
        b = s.heat_balance()
        self.assertAlmostEqual(b["resistance_ohm"], s.resistance)
        self.assertAlmostEqual(b["copper_resistance_ohm"], s.copper_resistance)
        self.assertAlmostEqual(b["temp_rate_c_s"], (b["heat_w"] - b["cooling_w"]) / s.mp.c_th)
        self.assertEqual(b["thermal_tau_s"], 87.0)
        self.assertAlmostEqual(b["torque_nm"], 2.0 * s.mp.Kt)

    def test_hot_motor_has_more_resistance_and_less_stall_torque(self):
        s = self.driven()
        cool_r = s.resistance
        cool_tau = s.capability(12.0, 0.0)["stall_torque_nm"]
        cool_heat_r = s.copper_resistance
        s.temp_c = 65.0
        self.assertGreater(s.resistance, cool_r)
        self.assertGreater(s.copper_resistance, cool_heat_r)
        self.assertLess(s.capability(12.0, 0.0)["stall_torque_nm"], cool_tau)

    def test_torque_speed_curve_matches_dc_motor(self):
        s = self.driven()
        tau0 = s.update(0.0, 0.0, 12.0, 0.001)
        s.temp_c = s.mp.ambient_c
        resistance = s.resistance
        omega = 2.0
        tau1 = s.update(0.0, omega * TICKS_PER_RAD, 12.0, 0.001)
        self.assertAlmostEqual(tau0 - tau1, s.mp.Kt * s.mp.Ke * omega / resistance)
        s.temp_c = s.mp.ambient_c
        limits = s.capability(12.0, omega)
        self.assertAlmostEqual(limits["torque_positive_nm"], tau1)
        self.assertLess(limits["torque_negative_nm"], 0.0)

    def test_no_load_speed_includes_friction_and_profile_limit(self):
        s = self.driven()
        s.regs.goal_velocity = 500
        cap = s.capability(12.0, 0.0)
        w = math.radians(cap["no_load_speed_deg_s"])
        drive = s.mp.Kt * (12.0 - s.mp.Ke * w) / s.resistance
        self.assertAlmostEqual(drive, s.mp.coulomb_nm + s.mp.viscous_nms * w)
        self.assertAlmostEqual(cap["speed_limit_deg_s"], math.degrees(500 / TICKS_PER_RAD))
        self.assertGreater(cap["accel_limit_deg_s2"], 0.0)
        self.assertLessEqual(cap["continuous_torque_nm"], cap["stall_torque_nm"])

    def test_disabled_and_faulted_bridge_do_not_short_brake(self):
        for mode in ("off", "overheat", "overcurrent"):
            with self.subTest(mode=mode):
                s = self.driven()
                s.temp_c = 75.0
                if mode == "off":
                    s.regs.torque_enable = 0
                else:
                    setattr(s, mode, True)
                torque = s.update(0.0, 3000.0, 12.0, 0.01)
                self.assertEqual((torque, s.current, s.duty), (0.0, 0.0, 0.0))
                self.assertLess(s.temp_c, 75.0)
                self.assertEqual(s.energy_j, 0.0)
                cap = s.capability(12.0, 2.0)
                self.assertEqual(cap["drive_limit"], 0.0)
                self.assertEqual(cap["torque_positive_nm"], 0.0)
                self.assertEqual(cap["torque_negative_nm"], 0.0)

    def test_overload_limit_applies_in_trip_period(self):
        s = self.driven()
        s.regs.protection_time = 0
        tau = s.update(0.0, 0.0, 12.0, 0.001)
        self.assertTrue(s.overload)
        self.assertEqual(s.duty, 200.0)
        self.assertAlmostEqual(tau, s.mp.Kt * 2.4 / (s.mp.R * (1.0 + s.mp.copper_alpha * 2.0)))
        self.assertEqual(s.capability(12.0, 0.0)["drive_limit"], 0.2)

    def test_overcurrent_trip_opens_bridge_immediately(self):
        s = self.driven()
        s.mp.overcurrent_a = 0.1
        s.regs.over_current_protection_time = 0
        self.assertEqual(s.update(0.0, 1000.0, 12.0, 0.001), 0.0)
        self.assertTrue(s.overcurrent)
        self.assertEqual(s.current, 0.0)
        self.assertEqual(s.heat_w, 0.0)

    def test_overheat_trip_opens_bridge_and_cools_until_hysteresis(self):
        s = self.driven()
        s.mp.r_th = 10.0
        s.temp_c = 69.99999
        self.assertEqual(s.update(0.0, 0.0, 12.0, 0.01), 0.0)
        self.assertTrue(s.overheat)
        self.assertEqual(s.current, 0.0)
        self.assertGreater(s.energy_j, 0.0)
        s.update(0.0, 1000.0, 12.0, 100.0)
        self.assertFalse(s.overheat)
        self.assertEqual(s.current, 0.0)

    def test_forecast_is_constant_loss_side_effect_free(self):
        s = self.servo()
        s.current = 5.0
        before = dict(vars(s))
        forecast = s.thermal_forecast(10000.0, 1000.0)
        self.assertEqual(forecast["assumption"], "constant_loss")
        self.assertEqual(forecast["points"][0], [0.0, s.temp_c, s.temp_c])
        self.assertAlmostEqual(forecast["points"][-1][1], forecast["equilibrium_c"])
        self.assertAlmostEqual(forecast["points"][-1][2], s.mp.ambient_c)
        trip = forecast["trip_in_s"]
        self.assertGreater(trip, 0.0)
        point = s.thermal_forecast(trip, trip)["points"][-1]
        self.assertAlmostEqual(point[1], s.regs.max_temperature)
        self.assertEqual(vars(s), before)

    def test_forecast_no_trip_and_exact_endpoint(self):
        s = self.servo()
        f = s.thermal_forecast(25.0, 10.0)
        self.assertEqual([p[0] for p in f["points"]], [0.0, 10.0, 20.0, 25.0])
        self.assertIsNone(f["trip_in_s"])
        s.temp_c = s.regs.max_temperature
        self.assertEqual(s.thermal_forecast()["trip_in_s"], 0.0)
        for seconds, step in ((-1, 10), (10, 0), (float("inf"), 10)):
            with self.assertRaises(ValueError):
                s.thermal_forecast(seconds, step)

    def test_motor_configuration_is_per_servo(self):
        mp = MotorParams()
        a = SimServo("a", 1, mp, 2048)
        b = SimServo("b", 2, mp, 2048)
        a.mp.c_th = 200
        a.mp.copper_r = 2
        self.assertEqual(b.mp.c_th, 29)
        self.assertEqual(mp.copper_r, 1)
        self.assertIsNot(a.mp, b.mp)

    def test_stalled_host_does_not_skip_physics_time(self):
        class Arm:
            t = 0.0
            def advance(self, seconds):
                self.t += seconds
        now = [0.0]
        arm = Arm()
        bus = SimFollowerBus(arm, clock=lambda: now[0])
        now[0] = 10.0
        bus._sync_time()
        self.assertAlmostEqual(arm.t, 0.25)
        bus._sync_time()
        self.assertAlmostEqual(arm.t, 0.25)
        now[0] += 0.1
        bus._sync_time()
        self.assertAlmostEqual(arm.t, 0.35)


if __name__ == "__main__":
    unittest.main()
