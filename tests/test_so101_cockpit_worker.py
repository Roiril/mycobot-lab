"""SO-101 cockpit worker: port resolution + auto-follow arming, on fake buses.

The gate itself is unit-tested in test_so101_teleop_engine; what is exercised
here is the wiring around it — that an armed cockpit engages follow by itself
when the arms agree, and that every operator stop disarms it so nothing
re-engages behind their back.

Runs on plain Python 3.10: the cockpit's lerobot use is behind make_bus, which
is replaced here, and the Quest launch import is already failure-tolerant.

Run: python -m unittest discover tests -p "test_so101_cockpit_worker.py"
"""
import sys
import time
import pathlib
import threading
import importlib.util
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.teleop_engine import JOINTS, TeleopConfig, deg_to_ticks
from robots.so101.ports import Roles


def _load_cockpit():
    """Import scripts/so101_cockpit_server.py (not importable as a package)."""
    spec = importlib.util.spec_from_file_location(
        "so101_cockpit_server", ROOT / "scripts" / "so101_cockpit_server.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


COCKPIT = _load_cockpit()

NEUTRAL = 2048
VOLT_RAW = {"leader": 74, "follower": 122}      # 0.1V units


class FakeBus:
    """Stand-in for FeetechMotorsBus: records writes, answers telemetry."""

    def __init__(self, port, present=None, volt_raw=122):
        self.port = port
        self.present = dict(present) if present else {n: NEUTRAL for n in JOINTS}
        self.volt_raw = volt_raw
        self.torque = {n: 0 for n in JOINTS}
        self.connected = False
        self.goals = []
        # motors that error-packet every telemetry read (protection trip)
        self.sick = set()

    # -- lifecycle
    def connect(self, handshake=True):
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.connected = False

    # -- io
    def sync_read(self, data_name, normalize=False, num_retry=1):
        return dict(self.present)

    def sync_write(self, data_name, values, normalize=False):
        self.goals.append(dict(values))
        self.present.update(values)

    def read(self, data_name, motor, normalize=False):
        if motor in self.sick and data_name != "Present_Position":
            raise RuntimeError(
                f"Failed to read '{data_name}' on id_={motor}. "
                "[RxPacketError] OverEle error!")
        if data_name == "Present_Position":
            return self.present[motor]
        if data_name == "Present_Voltage":
            return self.volt_raw
        if data_name == "Torque_Enable":
            return self.torque[motor]
        return 0

    def write(self, data_name, motor, value, normalize=False):
        if data_name == "Torque_Enable":
            self.torque[motor] = value
        elif data_name == "Goal_Position":
            self.present[motor] = value


class WorkerHarness:
    """Runs COCKPIT.worker on fake buses, with the module globals restored after."""

    def __init__(self, leader_present=None, follower_present=None,
                 roles=None, **worker_kw):
        self.leader = FakeBus("COM_L", leader_present, VOLT_RAW["leader"])
        self.follower = FakeBus("COM_F", follower_present, VOLT_RAW["follower"])
        self.roles = roles or Roles(leader="COM_L", follower="COM_F",
                                    source="probe", detail="test")
        self.stop = threading.Event()
        self.worker_kw = worker_kw
        self._saved = {}

    def __enter__(self):
        buses = {"COM_L": self.leader, "COM_F": self.follower}
        self._saved = {"make_bus": COCKPIT.make_bus,
                       "wait": COCKPIT.so101_ports.wait_for_ports,
                       "state": dict(COCKPIT.STATE)}
        COCKPIT.make_bus = lambda port: buses[port]
        COCKPIT.so101_ports.wait_for_ports = lambda *a, **kw: self.roles
        COCKPIT.STATE["log"] = []
        cfg = TeleopConfig(target_hz=200.0, torque_stagger_s=0.0)
        self.thread = threading.Thread(
            target=COCKPIT.worker,
            args=("auto", "auto", cfg),
            kwargs=dict(stop=self.stop, **self.worker_kw), daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join(timeout=3.0)
        COCKPIT.make_bus = self._saved["make_bus"]
        COCKPIT.so101_ports.wait_for_ports = self._saved["wait"]
        COCKPIT.STATE.update(self._saved["state"])
        return False

    # -- helpers
    def wait_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with COCKPIT.LOCK:
                snapshot = dict(COCKPIT.STATE)
            if predicate(snapshot):
                return snapshot
            time.sleep(0.02)
        return None


def far_pose(deg=45.0):
    p = {n: NEUTRAL for n in JOINTS}
    p["elbow_flex"] = NEUTRAL + deg_to_ticks(deg)
    return p


# ---------------------------------------------------------------------------
class TestPortWiring(unittest.TestCase):
    def test_resolved_roles_are_published_for_the_ui(self):
        with WorkerHarness() as h:
            st = h.wait_until(lambda s: s["ports"].get("leader"))
            self.assertIsNotNone(st, "ports were never published")
            self.assertEqual(st["ports"]["leader"], "COM_L")
            self.assertEqual(st["ports"]["follower"], "COM_F")
            self.assertTrue(st["ports"]["confirmed"])

    def test_each_arm_gets_the_bus_for_its_own_port(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            self.assertTrue(h.leader.connected and h.follower.connected)
            # only the follower is ever driven
            self.assertEqual(h.leader.goals, [])


class TestSickMotorDoesNotFreezeTheCockpit(unittest.TestCase):
    """A servo in a protection trip error-packets every telemetry read.

    That used to escape the cycle before the STATE write, so the page froze on
    the last good snapshot and showed a stale 51C / 345mA as if it were live
    while the arm had actually cooled (2026-08-07). The bad motor must now cost
    only its own fields.
    """

    def test_state_keeps_updating(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            h.follower.sick.add("elbow_flex")
            # move the leader: a live cockpit must reflect it
            moved = NEUTRAL + 300
            h.leader.present["shoulder_pan"] = moved
            st = h.wait_until(
                lambda s: s["leader"].get("shoulder_pan", {}).get("pos") == moved,
                timeout=5.0)
            self.assertIsNotNone(st, "STATE stopped updating while a motor was sick")

    def test_the_sick_motor_is_flagged_stale(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            h.follower.sick.add("elbow_flex")
            st = h.wait_until(
                lambda s: s["follower"].get("elbow_flex", {}).get("stale"),
                timeout=5.0)
            self.assertIsNotNone(st, "a frozen reading was never marked stale")
            # its healthy neighbours stay live
            self.assertFalse(st["follower"]["shoulder_pan"].get("stale"))

    def test_the_repeated_error_does_not_flush_the_log(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            h.follower.sick.add("elbow_flex")
            time.sleep(1.5)          # ~50 failing reads at the idle rate
            with COCKPIT.LOCK:
                log = list(COCKPIT.STATE["log"])
            self.assertTrue(any("接続 OK" in e["msg"] for e in log),
                            "the connect line was flushed out by error spam")


class TestAutoFollowArming(unittest.TestCase):
    def test_engages_once_the_arms_agree(self):
        with WorkerHarness(auto_follow=True) as h:
            st = h.wait_until(lambda s: s["teleop"])
            self.assertIsNotNone(st, "auto-follow never engaged on matching poses")
            self.assertFalse(st["auto_follow"]["armed"], "should disarm on engage")
            self.assertTrue(all(h.follower.torque[n] for n in JOINTS))

    def test_waits_while_the_arms_disagree(self):
        with WorkerHarness(auto_follow=True, leader_present=far_pose()) as h:
            st = h.wait_until(lambda s: s["auto_follow"].get("gap_deg", 0) > 0)
            self.assertIsNotNone(st)
            self.assertFalse(st["teleop"], "engaged across a 45deg gap")
            self.assertAlmostEqual(st["auto_follow"]["gap_deg"], 45.0, delta=0.2)
            self.assertEqual(st["auto_follow"]["gap_joint"], "elbow_flex")
            self.assertTrue(st["auto_follow"]["armed"], "should stay armed, waiting")
            # ... and the follower is not being commanded meanwhile
            self.assertEqual(h.follower.goals, [])

    def test_off_by_default(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            time.sleep(0.2)
            with COCKPIT.LOCK:
                self.assertFalse(COCKPIT.STATE["teleop"])
                self.assertFalse(COCKPIT.STATE["auto_follow"]["armed"])

    def test_can_be_armed_from_the_ui(self):
        with WorkerHarness(leader_present=far_pose()) as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            COCKPIT.CMDQ.put({"op": "auto_follow", "on": True})
            st = h.wait_until(lambda s: s["auto_follow"]["armed"])
            self.assertIsNotNone(st, "arming from the UI had no effect")
            self.assertFalse(st["teleop"])

    def test_abort_disarms_so_it_cannot_re_engage(self):
        with WorkerHarness(auto_follow=True, leader_present=far_pose()) as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["auto_follow"]["armed"]))
            COCKPIT.CMDQ.put({"op": "abort"})
            st = h.wait_until(lambda s: not s["auto_follow"]["armed"])
            self.assertIsNotNone(st, "ABORT left auto-follow armed")
            # bring the arms into agreement: it must NOT start following
            h.leader.present = {n: NEUTRAL for n in JOINTS}
            time.sleep(0.3)
            with COCKPIT.LOCK:
                self.assertFalse(COCKPIT.STATE["teleop"])

    def test_relax_disarms(self):
        with WorkerHarness(auto_follow=True, leader_present=far_pose()) as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["auto_follow"]["armed"]))
            COCKPIT.CMDQ.put({"op": "relax"})
            self.assertIsNotNone(h.wait_until(lambda s: not s["auto_follow"]["armed"]),
                                 "relax left auto-follow armed")

    def test_manual_follow_off_disarms(self):
        with WorkerHarness(auto_follow=True) as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["teleop"]))
            COCKPIT.CMDQ.put({"op": "teleop", "on": False})
            st = h.wait_until(lambda s: not s["teleop"])
            self.assertIsNotNone(st)
            self.assertFalse(st["auto_follow"]["armed"])

    def test_unconfirmed_ports_never_auto_start(self):
        # a remembered mapping can be wrong if the cables were swapped
        remembered = Roles(leader="COM_L", follower="COM_F", source="cache",
                           detail="記憶から復元")
        with WorkerHarness(auto_follow=True, roles=remembered) as h:
            st = h.wait_until(lambda s: s["connected"])
            self.assertIsNotNone(st)
            time.sleep(0.3)
            with COCKPIT.LOCK:
                self.assertFalse(COCKPIT.STATE["teleop"],
                                 "started motion from an unconfirmed port mapping")


if __name__ == "__main__":
    unittest.main()
