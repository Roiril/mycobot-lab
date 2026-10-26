"""HardwareLink tests use a moving mock or packet-level fakes, never a port."""
import json
import pathlib
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import twin_link as twin
from robots.so101.ports import PortInfo


def goals(snapshot):
    return {n: j["q"] for n, j in snapshot["joints"].items()}


class HardwareLinkTests(unittest.TestCase):
    def make_link(self):
        link = twin.HardwareLink("mock")
        self.addCleanup(link.close)
        link.command("connect")
        return link

    def wait_for(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("condition did not become true")

    def test_off_and_construction_do_not_start_worker(self):
        with patch.object(twin.HardwareLink, "_open_backend", side_effect=AssertionError("I/O")):
            for mode in ("off", "mock", "real"):
                link = twin.HardwareLink(mode)
                self.assertIsNone(link._thread)
                self.assertFalse(link.snapshot()["connected"])
                self.assertFalse(link.command("stop")["connected"])
                self.assertIsNone(link._thread)
                link.close()
            link = twin.HardwareLink()
            with self.assertRaises(ValueError):
                link.command("connect")

    def test_observation_writes_nothing_and_snapshot_is_cache(self):
        link = self.make_link()
        backend = link._backend
        seq = link.snapshot()["sample_seq"]
        self.wait_for(lambda: link.snapshot()["sample_seq"] > seq)
        self.assertEqual(backend.writes, [])
        with link._lock, patch.object(backend, "read_telemetry", side_effect=AssertionError("I/O")):
            snap = link.snapshot()
            snap["joints"]["gripper"]["q"] = 999
            self.assertNotEqual(link.snapshot()["joints"]["gripper"]["q"], 999)
        link.command("disconnect")
        self.assertEqual(backend.writes, [])

    def test_degrees_match_lerobot_and_gripper_is_not_percent(self):
        c = {"range_min": 1605, "range_max": 3086, "drive_mode": 1}
        self.assertAlmostEqual(twin._to_deg(1605, c), -1481 / 2 * 360 / 4095)
        self.assertEqual(twin._to_raw(0, c), 2345)
        self.assertEqual(twin._to_raw(20, c), int(20 * 4095 / 360 + 2345.5))
        with self.assertRaises(ValueError):
            twin._to_raw(100, c)
        self.assertEqual(twin._to_deg(2345.5, c), 0)

    def test_invalid_input_never_writes(self):
        link = self.make_link()
        initial = goals(link.snapshot())
        invalid = [None, {}, {**initial, "extra": 0}, {**initial, "gripper": True},
                   {**initial, "gripper": float("nan")}, {**initial, "gripper": 101},
                   {**initial, "elbow_flex": float("inf")}, {**initial, "wrist_roll": "0"}]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                link.command("follow", {"client_id": "one", "goals": value})
        with self.assertRaises(ValueError):
            link.command("follow", {"client_id": "", "goals": initial})
        with self.assertRaises(ValueError):
            link.set_target({})
        self.assertEqual(link._backend.writes, [])

    def test_follow_movement_lease_and_physical_stop(self):
        link = self.make_link()
        initial = goals(link.snapshot())
        reply = link.command("follow", {"client_id": "one", "goals": initial})
        self.assertIn("lease", reply)
        self.assertNotIn("lease", link.snapshot())
        with self.assertRaises(ValueError):
            link.command("heartbeat", {"client_id": "two", "lease": reply["lease"]})
        with self.assertRaises(ValueError):
            link.command("heartbeat", {"client_id": "one", "lease": "bad"})
        target = {**initial, "shoulder_pan": initial["shoulder_pan"] + 12}
        start = time.monotonic()
        link.set_target(target)
        self.wait_for(lambda: link.snapshot()["joints"]["shoulder_pan"]["q"] > initial["shoulder_pan"] + 1)
        moved = link.snapshot()["joints"]["shoulder_pan"]["q"] - initial["shoulder_pan"]
        self.assertLessEqual(moved, 15 * (time.monotonic() - start) + 0.2)
        link.command("heartbeat", {"client_id": "one", "lease": reply["lease"]})
        stopped = link.command("stop")
        self.assertFalse(stopped["following"])
        self.wait_for(lambda: link.snapshot()["stop_confirmed"])
        stopped_q = goals(link.snapshot())
        link.set_target(initial)
        time.sleep(0.12)
        self.assertEqual(goals(link.snapshot()), stopped_q)
        with self.assertRaises(ValueError):
            link.command("heartbeat", {"client_id": "one", "lease": reply["lease"]})
        new = link.command("follow", {"client_id": "two", "goals": stopped_q})
        self.assertNotEqual(new["lease"], reply["lease"])

    def test_watchdog_is_worker_owned_and_no_automatic_resume(self):
        with patch.object(twin, "LEASE_S", 0.15):
            link = self.make_link()
            original = goals(link.snapshot())
            reply = link.command("follow", {"client_id": "one", "goals": original})
            link.set_target({**original, "shoulder_pan": 10})
            self.wait_for(lambda: link.snapshot()["phase"] == "stopped")
            self.assertIn("expired", link.snapshot()["reason"])
            self.assertIsNone(link.snapshot()["owner"])
            with self.assertRaises(ValueError):
                link.command("heartbeat", {"client_id": "one", "lease": reply["lease"]})
            self.assertFalse(link.snapshot()["following"])

    def test_follow_seed_precedes_torque_enable(self):
        link = self.make_link()
        link._backend.torque = {n: False for n in twin.JOINTS}
        link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
        writes = link._backend.writes
        self.assertEqual(writes[0][0], "commands")
        self.assertEqual(writes[1][0], "Torque_Enable")
        self.assertTrue(all(j["torque"] for j in link.snapshot()["joints"].values()))

    def test_alignment_and_safety_failure_are_read_only(self):
        link = self.make_link()
        target = goals(link.snapshot())
        with self.assertRaises(ValueError):
            link.command("follow", {"client_id": "one", "goals": {**target, "shoulder_pan": 10}})
        with patch.object(twin.safety, "check_angles", return_value=(False, "unsafe", [])):
            with self.assertRaisesRegex(ValueError, "unsafe"):
                link.command("follow", {"client_id": "one", "goals": target})
        self.assertEqual(link._backend.writes, [])

    def test_missing_calibration_allows_observation_but_not_follow(self):
        link = twin.HardwareLink("real")
        self.addCleanup(link.close)
        backend = twin._MockBackend()
        with patch.object(link, "_open_backend", return_value=backend), \
                patch.object(twin, "_load_calibration", side_effect=ValueError("missing calibration")):
            link.command("connect")
            self.assertFalse(link.snapshot()["calibration"]["valid"])
            self.assertTrue(all(j["q"] is None for j in link.snapshot()["joints"].values()))
            with self.assertRaisesRegex(ValueError, "calibration"):
                link.command("follow", {"client_id": "one", "goals": dict.fromkeys(twin.JOINTS, 0)})
            self.assertEqual(backend.writes, [])

    def test_mismatching_calibration_and_mode_refuse_follow(self):
        link = self.make_link()
        target = goals(link.snapshot())
        read = link._backend.read_reg
        def wrong(reg, name):
            return (1, 0) if reg == "Operating_Mode" else read(reg, name)
        with patch.object(link._backend, "read_reg", side_effect=wrong):
            with self.assertRaisesRegex(ValueError, "calibration"):
                link.command("follow", {"client_id": "one", "goals": target})
        self.assertFalse(link.snapshot()["calibration"]["valid"])
        self.assertEqual(link._backend.writes, [])

    def test_fault_stops_and_does_not_execute_cooling_move(self):
        link = self.make_link()
        link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
        original_read = link._backend.read_telemetry
        def hot():
            values = original_read()
            values["elbow_flex"].temp = 69
            return values
        with patch.object(link._backend, "read_telemetry", side_effect=hot):
            self.wait_for(lambda: link.snapshot()["phase"] == "fault")
        self.assertFalse(link.snapshot()["following"])
        with self.assertRaises(ValueError):
            link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
        self.assertIsNone(link.snapshot()["owner"])

    def test_communication_loss_never_claims_stop_confirmation(self):
        link = self.make_link()
        link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
        with patch.object(link._backend, "read_telemetry", side_effect=RuntimeError("serial lost")):
            self.wait_for(lambda: link.snapshot()["phase"] == "fault")
            self.assertFalse(link.snapshot()["stop_confirmed"])
            self.assertIn("unconfirmed", link.snapshot()["reason"])

    def test_stale_queued_follow_is_cancelled_by_stop(self):
        link = self.make_link()
        generation = link._generation
        link.command("stop")
        with link._lock, self.assertRaisesRegex(ValueError, "cancelled"):
            link._handle(generation, "follow", {"client_id": "one", "goals": goals(link.snapshot())})
        self.assertEqual(link._backend.writes, [])

    def test_stop_priority_discards_queued_follow_and_old_lease(self):
        link = self.make_link()
        target = goals(link.snapshot())
        done_follow, done_stop = threading.Event(), threading.Event()
        follow_box, stop_box = {}, {}
        with link._lock:
            generation = link._generation
            link._queue.put((10, 100, generation, "follow", {"client_id": "one", "goals": target},
                             done_follow, follow_box))
            link._invalidate()
            link._queue.put((0, 101, link._generation, "stop", {}, done_stop, stop_box))
        self.assertTrue(done_stop.wait(1))
        self.assertTrue(done_follow.wait(1))
        self.assertIn("cancelled", follow_box["error"])
        self.assertFalse(link.snapshot()["following"])
        self.assertEqual(link._backend.writes, [])

    def test_single_owner_cannot_be_replaced_by_second_follow(self):
        link = self.make_link()
        target = goals(link.snapshot())
        reply = link.command("follow", {"client_id": "one", "goals": target})
        with self.assertRaises(ValueError):
            link.command("follow", {"client_id": "two", "goals": target})
        self.assertEqual(link.snapshot()["owner"], "one")
        self.assertTrue(link.command("heartbeat", {"client_id": "one", "lease": reply["lease"]})["ok"])

    def test_calibration_validation_has_no_full_range_fallback(self):
        valid = twin._MockBackend().calibration
        with patch.object(pathlib.Path, "read_text", return_value=json.dumps(valid)):
            self.assertEqual(twin._load_calibration("dummy.json"), valid)
        for field, value in (("id", 4), ("range_min", -1), ("range_max", 0),
                             ("homing_offset", 3000), ("drive_mode", 2)):
            bad = json.loads(json.dumps(valid))
            bad["shoulder_pan"][field] = value
            with self.subTest(field=field), \
                    patch.object(pathlib.Path, "read_text", return_value=json.dumps(bad)), \
                    self.assertRaises(ValueError):
                twin._load_calibration("dummy.json")

    def test_real_observation_sends_only_read_instructions(self):
        # Exercise the actual RawFeetechIO packet path, rather than merely
        # trusting a backend's write counter. No serial device is opened.
        class Serial:
            def __init__(self, *args, **kwargs):
                self.buf = bytearray()
                self.instructions = []
            @property
            def in_waiting(self):
                return len(self.buf)
            def reset_input_buffer(self):
                self.buf.clear()
            def write(self, packet):
                self.instructions.append(packet[4])
                inst = packet[4]
                if inst == 0x82:
                    data = twin.sts3215.encode_telemetry(
                        twin.sts3215.ServoState(pos=2048, volt=12, temp=27))
                    for sid in packet[7:-1]:
                        self.reply(sid, data)
                elif inst == 0x02:
                    sid, reg, length = packet[2], packet[5], packet[6]
                    values = {5: sid, 9: 0, 11: 4095, 30: 1, 31: 0, 33: 0, 18: 4, 40: 1,
                              twin.sts3215.REG["P_Coefficient"][0]: 16}
                    value = values[reg]
                    self.reply(sid, bytes([value & 255, value >> 8][:length]))
                else:
                    raise AssertionError("register write during observation")
                return len(packet)
            def reply(self, sid, data):
                body = bytes([sid, len(data) + 2, 0]) + data
                self.buf.extend(b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
            def read(self, count):
                out = bytes(self.buf[:count])
                del self.buf[:count]
                return out
            def close(self):
                pass
        fake_serial = Serial()
        fake_module = type("SerialModule", (), {"Serial": lambda *args, **kw: fake_serial})()
        link = twin.HardwareLink("real")
        self.addCleanup(link.close)
        with patch.dict(sys.modules, {"serial": fake_module}), \
                patch.object(twin, "list_so101_ports", return_value=[
                    PortInfo("COM81", vid=0x1A86, pid=0x55D3)]), \
                patch.object(twin, "_load_calibration", return_value=twin._MockBackend().calibration):
            snap = link.command("connect")
            self.assertTrue(snap["connected"])
            self.assertTrue(snap["calibration"]["valid"])
            link.command("disconnect")
        self.assertTrue(fake_serial.instructions)
        self.assertEqual(set(fake_serial.instructions), {0x02, 0x82})

    def test_changed_goals_only_and_path_checked_before_write(self):
        link = self.make_link()
        target = goals(link.snapshot())
        link.command("follow", {"client_id": "one", "goals": target})
        count = len(link._backend.writes)
        time.sleep(0.12)
        self.assertEqual(len(link._backend.writes), count)
        with patch.object(twin.safety, "check_angles", return_value=(False, "path blocked", [])) as check:
            link.set_target({**target, "shoulder_pan": 10})
            self.wait_for(lambda: link.snapshot()["phase"] == "fault")
            self.assertTrue(check.called)
        # Only a stop-at-current command may be added, never the blocked target.
        self.assertTrue(all(c.goal == link._states[n].pos
                            for kind, batch in link._backend.writes[count:] if kind == "commands"
                            for n, c in batch.items()))

    def test_full_requested_path_is_rejected_before_first_motion(self):
        link = self.make_link()
        target = goals(link.snapshot())
        link.command("follow", {"client_id": "one", "goals": target})
        check = twin.safety.check_angles
        def reject_far(q):
            return (False, "far target blocked", []) if q[0] >= 8 else check(q)
        with patch.object(twin.safety, "check_angles", side_effect=reject_far):
            link.set_target({**target, "shoulder_pan": 10})
            self.wait_for(lambda: link.snapshot()["phase"] == "fault")
        self.assertAlmostEqual(link.snapshot()["joints"]["shoulder_pan"]["q"],
                               target["shoulder_pan"], places=6)

    def test_observed_protection_status_is_visible_without_writes(self):
        link = self.make_link()
        read = link._backend.read_reg
        def with_status(reg, name):
            value, status = read(reg, name)
            return value, 0x20 if reg == "Torque_Enable" else status
        seq = link.snapshot()["sample_seq"]
        with patch.object(link._backend, "read_reg", side_effect=with_status):
            self.wait_for(lambda: link.snapshot()["sample_seq"] > seq)
            self.assertEqual(link.snapshot()["joints"]["elbow_flex"]["status"], 0x20)
        self.assertEqual(link._backend.writes, [])

    def test_real_port_selection_is_strict_and_exclusive_failure_is_not_ignored(self):
        ports = [PortInfo("COM81", vid=0x1A86, pid=0x55D3),
                 PortInfo("COM82", vid=0x1A86, pid=0x55D3)]
        link = twin.HardwareLink("real")
        backends = [twin._MockBackend(), twin._MockBackend()]
        with patch.object(twin, "list_so101_ports", return_value=ports), \
                patch.object(twin, "_RealBackend", side_effect=backends):
            with self.assertRaisesRegex(ValueError, "ambiguous"):
                link._open_backend()
        with patch.object(twin, "list_so101_ports", return_value=ports), \
                patch.object(twin, "_RealBackend", side_effect=PermissionError("COM occupied")):
            with self.assertRaisesRegex(PermissionError, "occupied"):
                link._open_backend()

    def test_all_io_is_worker_owned(self):
        link = self.make_link()
        ids = []
        backend = link._backend
        read, write = backend.read_telemetry, backend.write_commands
        def capture_read():
            ids.append(threading.get_ident())
            return read()
        def capture_write(value):
            ids.append(threading.get_ident())
            return write(value)
        with patch.object(backend, "read_telemetry", side_effect=capture_read), \
                patch.object(backend, "write_commands", side_effect=capture_write):
            link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
            link.command("stop")
        self.assertTrue(ids)
        self.assertEqual(set(ids), {link._thread.ident})

    def test_governor_uses_measured_gain_without_overwriting_it(self):
        link = self.make_link()
        read = link._backend.read_reg
        def measured_gain(reg, name):
            return (32, 0) if reg == "P_Coefficient" else read(reg, name)
        with patch.object(link._backend, "read_reg", side_effect=measured_gain):
            link.command("follow", {"client_id": "one", "goals": goals(link.snapshot())})
            self.assertEqual(set(link._governor.p_coeff.values()), {32})
        self.assertFalse(any(reg == "P_Coefficient" for reg, _ in link._backend.writes))

    def test_protected_follower_can_be_identified_for_observation(self):
        port = PortInfo("COM81", vid=0x1A86, pid=0x55D3)
        backend = twin._MockBackend()
        read = backend.read_telemetry
        def protected():
            states = read()
            for state in states.values():
                state.status = 0x20
            return states
        link = twin.HardwareLink("real")
        with patch.object(twin, "list_so101_ports", return_value=[port]), \
                patch.object(twin, "_RealBackend", return_value=backend), \
                patch.object(backend, "read_telemetry", side_effect=protected):
            self.assertIs(link._open_backend(), backend)
        self.assertEqual(backend.writes, [])


if __name__ == "__main__":
    unittest.main()
