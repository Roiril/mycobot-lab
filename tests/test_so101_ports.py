"""SO-101 port identification: the pure decision layer.

Runs on plain Python 3.10 with no hardware and no lerobot — everything here is
the logic that decides which physical arm a COM port belongs to, which is the
part that must never guess: a swapped assignment torque-enables the leader.

Run: python -m unittest tests.test_so101_ports
"""
import json
import pathlib
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.ports import (
    LEADER, FOLLOWER, PortInfo, Probe, Roles, PortError,
    classify_arm_voltage, port_keys, decide_roles,
    roles_from_cache, load_cache, save_cache,
)


def port(device, sn=None, loc=None, desc="USB-Enhanced-SERIAL CH343"):
    return PortInfo(device=device, description=desc, serial_number=sn,
                    location=loc, vid=0x1A86, pid=0x55D3)


def probe(device, volt=None, ok=None, error=""):
    ok = (volt is not None) if ok is None else ok
    return Probe(device=device, volt=volt, motors=["shoulder_pan"] if ok else [],
                 ok=ok, error=error)


# ---------------------------------------------------------------------------
class TestClassifyArmVoltage(unittest.TestCase):
    def test_twelve_volt_arm_is_the_follower(self):
        for v in (12.4, 12.0, 11.2, 9.6):
            self.assertEqual(classify_arm_voltage(v), FOLLOWER, f"{v}V")

    def test_low_voltage_arm_is_the_leader(self):
        for v in (8.4, 7.4, 5.0, 4.9):
            self.assertEqual(classify_arm_voltage(v), LEADER, f"{v}V")

    def test_gap_between_the_windows_is_refused(self):
        # a browning-out follower must not be mistaken for a leader
        self.assertEqual(classify_arm_voltage(9.2), "unknown")

    def test_missing_or_stale_reading_is_unknown(self):
        self.assertEqual(classify_arm_voltage(None), "unknown")
        self.assertEqual(classify_arm_voltage(0.0), "unknown")

    def test_probe_role_needs_a_successful_read(self):
        self.assertEqual(probe("COM3", volt=12.2, ok=False).role, "unknown")


# ---------------------------------------------------------------------------
class TestPortKeys(unittest.TestCase):
    def test_distinct_serials_are_used(self):
        keys = port_keys([port("COM13", sn="AAA"), port("COM14", sn="BBB")])
        self.assertEqual(keys, {"COM13": "sn:AAA", "COM14": "sn:BBB"})

    def test_shared_serial_falls_through_to_usb_location(self):
        # CH343 boards commonly ship with the same serial: it identifies nothing
        keys = port_keys([port("COM13", sn="SAME", loc="1-4"),
                          port("COM14", sn="SAME", loc="1-5")])
        self.assertEqual(keys, {"COM13": "loc:1-4", "COM14": "loc:1-5"})

    def test_no_serial_no_location_falls_back_to_the_com_number(self):
        self.assertEqual(port_keys([port("COM9")]), {"COM9": "dev:COM9"})


# ---------------------------------------------------------------------------
class TestDecideRoles(unittest.TestCase):
    def test_one_of_each(self):
        roles = decide_roles([probe("COM13", 12.2), probe("COM14", 7.4)])
        self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))
        self.assertTrue(roles.confirmed)

    def test_assignment_does_not_depend_on_enumeration_order(self):
        roles = decide_roles([probe("COM14", 7.4), probe("COM13", 12.2)])
        self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))

    def test_silent_port_is_the_other_arm_when_only_two_exist(self):
        roles = decide_roles([probe("COM13", 12.2),
                              probe("COM14", error="応答なし")])
        self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))

    def test_silent_port_is_the_follower_when_the_leader_answered(self):
        roles = decide_roles([probe("COM14", 7.4), probe("COM13", error="x")])
        self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))

    def test_two_twelve_volt_arms_are_refused(self):
        with self.assertRaises(PortError):
            decide_roles([probe("COM13", 12.2), probe("COM14", 12.1)])

    def test_two_low_voltage_arms_are_refused(self):
        # e.g. the follower's 12V supply is off — guessing would drive the leader
        with self.assertRaises(PortError):
            decide_roles([probe("COM13", 7.4), probe("COM14", 7.3)])

    def test_nothing_responding_is_refused(self):
        with self.assertRaises(PortError):
            decide_roles([probe("COM13", error="a"), probe("COM14", error="b")])

    def test_no_ports_is_refused(self):
        with self.assertRaises(PortError):
            decide_roles([])

    def test_elimination_does_not_apply_to_three_ports(self):
        with self.assertRaises(PortError):
            decide_roles([probe("COM13", 12.2), probe("COM14", error="x"),
                          probe("COM15", error="y")])

    def test_message_names_every_port_and_its_reading(self):
        with self.assertRaises(PortError) as cm:
            decide_roles([probe("COM13", 12.2), probe("COM14", 12.1)])
        msg = str(cm.exception)
        self.assertIn("COM13", msg)
        self.assertIn("12.1V", msg)


# ---------------------------------------------------------------------------
class TestRolesCache(unittest.TestCase):
    def setUp(self):
        self.ports = [port("COM13", loc="1-4"), port("COM14", loc="1-5")]
        self.cache = {"leader": {"key": "loc:1-5"}, "follower": {"key": "loc:1-4"}}

    def test_remembered_identities_are_reapplied(self):
        roles = roles_from_cache(self.cache, self.ports)
        self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))

    def test_remembered_mapping_is_never_treated_as_confirmed(self):
        # it is keyed by USB socket, so swapped cables would make it a lie
        self.assertFalse(roles_from_cache(self.cache, self.ports).confirmed)

    def test_survives_com_renumbering(self):
        renumbered = [port("COM7", loc="1-4"), port("COM8", loc="1-5")]
        roles = roles_from_cache(self.cache, renumbered)
        self.assertEqual((roles.leader, roles.follower), ("COM8", "COM7"))

    def test_absent_arm_yields_nothing(self):
        self.assertIsNone(roles_from_cache(self.cache, [port("COM13", loc="1-4")]))

    def test_empty_or_degenerate_cache_yields_nothing(self):
        self.assertIsNone(roles_from_cache({}, self.ports))
        self.assertIsNone(roles_from_cache(
            {"leader": {"key": "loc:1-4"}, "follower": {"key": "loc:1-4"}},
            self.ports))

    def test_confirmed_mapping_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            fp = pathlib.Path(d) / "nested" / "so101_ports.json"
            save_cache(Roles(leader="COM14", follower="COM13", source="probe"),
                       self.ports, fp)
            roles = roles_from_cache(load_cache(fp), self.ports)
            self.assertEqual((roles.leader, roles.follower), ("COM14", "COM13"))
            self.assertEqual(json.loads(fp.read_text(encoding="utf-8"))["version"], 1)

    def test_unconfirmed_mapping_is_not_written(self):
        with tempfile.TemporaryDirectory() as d:
            fp = pathlib.Path(d) / "so101_ports.json"
            save_cache(Roles(leader="COM14", follower="COM13", source="cache"),
                       self.ports, fp)
            self.assertFalse(fp.exists())
            save_cache(Roles(leader="COM14", follower="COM13", source="manual"),
                       self.ports, fp)
            self.assertFalse(fp.exists())

    def test_unwritable_path_does_not_raise(self):
        save_cache(Roles(leader="COM14", follower="COM13", source="probe"),
                   self.ports, pathlib.Path("\0invalid") / "x.json")


if __name__ == "__main__":
    unittest.main()
