"""Servo settings: session writes and the audit's operator-facing findings.

Run: python -m unittest discover tests -p "test_so101_servo_config.py"
"""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import servo_config  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def factory_row(**over):
    row = {"Firmware_Major": 3, "Firmware_Minor": 10, "Max_Voltage_Limit": 140,
           "Min_Voltage_Limit": 40, "Max_Temperature_Limit": 70, "Operating_Mode": 0,
           "Unloading_Condition": 44, "Overload_Torque": 80, "Protective_Torque": 20,
           "Protection_Time": 200, "P_Coefficient": 16, "I_Coefficient": 0,
           "Return_Delay_Time": 0, "Min_Position_Limit": 800, "Max_Position_Limit": 3100,
           "Phase": 12}
    row.update(over)
    return row


def table(**per_servo):
    return {n: factory_row(**per_servo.get(n, {})) for n in JOINTS}


class TestFindings(unittest.TestCase):
    def levels(self, t, **kw):
        return {(f.servo, f.register): f.level for f in servo_config.findings(t, **kw)}

    def test_factory_settings_are_clean(self):
        self.assertEqual(servo_config.findings(table()), [])

    def test_the_2026_07_17_voltage_limit_is_an_error(self):
        lv = self.levels(table(elbow_flex={"Max_Voltage_Limit": 120}), supply_v=12.2)
        self.assertEqual(lv[("elbow_flex", "Max_Voltage_Limit")], "error")

    def test_a_servo_integral_term_is_flagged(self):
        lv = self.levels(table(elbow_flex={"I_Coefficient": 8}))
        self.assertEqual(lv[("elbow_flex", "I_Coefficient")], "warn")

    def test_a_raised_overheat_limit_is_flagged(self):
        lv = self.levels(table(elbow_flex={"Max_Temperature_Limit": 80}))
        self.assertEqual(lv[("elbow_flex", "Max_Temperature_Limit")], "warn")

    def test_mixed_firmware_is_flagged(self):
        lv = self.levels(table(gripper={"Firmware_Minor": 9}))
        self.assertEqual(lv[("*", "Firmware")], "warn")

    def test_wrong_operating_mode_is_an_error(self):
        lv = self.levels(table(wrist_roll={"Operating_Mode": 1}))
        self.assertEqual(lv[("wrist_roll", "Operating_Mode")], "error")

    def test_unreadable_values_are_not_findings(self):
        t = table(elbow_flex={"Max_Voltage_Limit": 0, "Unloading_Condition": 0,
                              "Overload_Torque": 0})
        self.assertNotIn(("elbow_flex", "Max_Voltage_Limit"), self.levels(t))

    def test_a_silent_servo_is_an_error(self):
        t = table()
        t["wrist_flex"] = {k: None for k in t["wrist_flex"]}
        self.assertEqual(self.levels(t)[("wrist_flex", "ID")], "error")


class TestSessionWrites(unittest.TestCase):
    def test_the_lock_closes_before_any_eeprom_area_write(self):
        w = servo_config.session_writes(servo_config.SessionSettings(), ["elbow_flex"])
        self.assertEqual(w[0], ("Lock", "elbow_flex", 1))
        regs = [r for r, _, _ in w]
        self.assertLess(regs.index("Lock"), regs.index("P_Coefficient"))

    def test_gripper_keeps_its_burnout_guard(self):
        w = dict(((r, n), v) for r, n, v in
                 servo_config.session_writes(servo_config.SessionSettings(), JOINTS))
        self.assertEqual(w[("Torque_Limit", "gripper")], 500)
        self.assertEqual(w[("Torque_Limit", "elbow_flex")], 1000)
        self.assertEqual(w[("I_Coefficient", "elbow_flex")], 0)

    def test_speed_presets_convert_to_steps_per_second(self):
        self.assertEqual(servo_config.dps_to_goal_velocity(45.0), 512)
        self.assertLess(servo_config.dps_to_goal_velocity(30.0),
                        servo_config.dps_to_goal_velocity(70.0))

    def test_apply_reports_the_p_actually_in_effect(self):
        regs = {n: {"P_Coefficient": 32} for n in JOINTS}

        def write(reg, n, v):
            if reg == "P_Coefficient" and n == "elbow_flex":
                return False, 0          # refused: keeps its old 32
            regs[n][reg] = v
            return True, 0

        def read(reg, n):
            return regs[n].get(reg), 0

        p = servo_config.apply_session(write, read, servo_config.SessionSettings(), JOINTS)
        self.assertEqual(p["elbow_flex"], 32)
        self.assertEqual(p["shoulder_lift"], 16)


class TestLockGuard(unittest.TestCase):
    def test_pdi_are_not_written_when_the_lock_cannot_be_closed(self):
        writes = []

        def write(reg, n, v):
            writes.append((reg, n))
            return (False, 0) if (reg == "Lock" and n == "elbow_flex") else (True, 0)

        servo_config.apply_session(write, lambda reg, n: (16, 0),
                                   servo_config.SessionSettings(), JOINTS)
        self.assertNotIn(("P_Coefficient", "elbow_flex"), writes)
        self.assertIn(("Torque_Limit", "elbow_flex"), writes)      # RAM registers still go
        self.assertIn(("P_Coefficient", "wrist_flex"), writes)


if __name__ == "__main__":
    unittest.main()
