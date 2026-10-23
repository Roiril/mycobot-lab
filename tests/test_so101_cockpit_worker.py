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
    """Stand-in for a lerobot FeetechMotorsBus (no raw serial port, so the
    engine reaches it through servo_io.LegacyBusIO): records writes by
    register, answers telemetry, and can have servos go silent."""

    def __init__(self, port, present=None, volt_raw=122):
        self.port = port
        self.present = dict(present) if present else {n: NEUTRAL for n in JOINTS}
        self.volt_raw = volt_raw
        self.torque = {n: 0 for n in JOINTS}
        self.regs = {n: {} for n in JOINTS}
        self.connected = False
        self.goals = []
        # servos that no longer answer at all (cable out / dead servo)
        self.sick = set()

    # -- lifecycle
    def connect(self, handshake=True):
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.connected = False

    # -- io
    def sync_read(self, data_name, normalize=False, num_retry=1):
        if self.sick:
            raise ConnectionError("[TxRxResult] There is no status packet!")
        if data_name == "Present_Position":
            return dict(self.present)
        return {n: self.read(data_name, n) for n in JOINTS}

    def sync_write(self, data_name, values, normalize=False):
        if data_name == "Goal_Position":
            self.goals.append(dict(values))
            for n, v in values.items():
                if self.torque.get(n):
                    self.present[n] = v
        elif data_name == "Torque_Enable":
            self.torque.update(values)
        else:
            for n, v in values.items():
                self.regs[n][data_name] = v

    def read(self, data_name, motor, normalize=False):
        if motor in self.sick:
            raise ConnectionError("[TxRxResult] There is no status packet!")
        if data_name == "Present_Position":
            return self.present[motor]
        if data_name == "Present_Voltage":
            return self.volt_raw
        if data_name == "Present_Temperature":
            return 30
        if data_name == "Torque_Enable":
            return self.torque[motor]
        if data_name == "P_Coefficient":
            return self.regs[motor].get(data_name, 16)
        return self.regs[motor].get(data_name, 0)

    def write(self, data_name, motor, value, normalize=False):
        if motor in self.sick:
            raise ConnectionError("[TxRxResult] There is no status packet!")
        if data_name == "Torque_Enable":
            self.torque[motor] = value
        elif data_name == "Goal_Position":
            self.present[motor] = value
        else:
            self.regs[motor][data_name] = value


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
    """A servo that stops answering must cost only its own fields.

    On 2026-08-07 one bad servo escaped the cycle before the STATE write, so the
    page froze on the last good snapshot and showed a stale 51C / 345 (raw, i.e.
    2.2 A) as if it were live while the arm had actually cooled. A servo in
    protection now answers normally (servo_io reports its status bits as data);
    a silent one is marked stale; a whole silent arm is a disconnect.
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

    def test_a_whole_arm_going_silent_is_reported_as_disconnected(self):
        # isolating one sick motor must not hide an unplugged cable: every joint
        # silent means the arm is gone, and saying "接続OK" then is a lie
        with WorkerHarness(auto_follow=True) as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["teleop"]))
            h.follower.sick.update(JOINTS)
            st = h.wait_until(lambda s: not s["connected"], timeout=6.0)
            self.assertIsNotNone(st, "a fully silent arm still reported connected")
            self.assertFalse(st["teleop"], "kept following an unreachable arm")
            self.assertIn("follower", st["error"])

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
            # ... and the follower is not driven anywhere meanwhile: the only
            # goals it gets are its own positions (kept so torque-on never jumps)
            self.assertTrue(all(g[n] == NEUTRAL for g in h.follower.goals for n in g))
            self.assertFalse(any(h.follower.torque.values()))

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


class TestCommandQueue(unittest.TestCase):
    def test_abort_jumps_ahead_of_queued_jogs(self):
        # dragging a slider floods /jog; an ABORT pressed meanwhile must not wait
        q = COCKPIT.queue.Queue()
        for i in range(20):
            q.put({"op": "jog", "joint": "elbow_flex", "pos": 2000 + i})
        q.put({"op": "abort"})
        order = [c["op"] for c in COCKPIT._drain(q)]
        self.assertEqual(order[0], "abort")

    def test_only_the_latest_jog_per_joint_is_kept(self):
        q = COCKPIT.queue.Queue()
        for i in range(5):
            q.put({"op": "jog", "joint": "elbow_flex", "pos": 2000 + i})
        q.put({"op": "jog", "joint": "wrist_flex", "pos": 1500})
        jogs = [c for c in COCKPIT._drain(q) if c["op"] == "jog"]
        self.assertEqual(len(jogs), 2)
        self.assertEqual(next(c["pos"] for c in jogs if c["joint"] == "elbow_flex"), 2004)


class TestStopSupersedesMotion(unittest.TestCase):
    def test_follow_on_clicked_just_before_abort_does_not_run_after_it(self):
        q = COCKPIT.queue.Queue()
        q.put({"op": "teleop", "on": True})
        q.put({"op": "retreat"})
        q.put({"op": "speed", "preset": "slow"})
        q.put({"op": "abort"})
        q.put({"op": "jog", "joint": "elbow_flex", "pos": 2100})     # after the stop: kept
        ops = [c["op"] for c in COCKPIT._drain(q)]
        self.assertEqual(ops[0], "abort")
        self.assertNotIn("teleop", ops)
        self.assertNotIn("retreat", ops)
        self.assertIn("speed", ops)
        self.assertIn("jog", ops)


class TestRepeatedFaultsFreezeTheArm(unittest.TestCase):
    def test_a_cycle_that_keeps_failing_ends_limp_or_frozen(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            COCKPIT.CMDQ.put({"op": "torque", "arm": "follower", "joint": "all", "on": True})
            self.assertIsNotNone(h.wait_until(
                lambda s: all(h.follower.torque[n] for n in JOINTS)))
            original = h.follower.sync_write

            def broken(data_name, values, normalize=False):
                if data_name == "Goal_Position":
                    raise ValueError("simulated bug in the write path")
                return original(data_name, values, normalize=normalize)
            h.follower.sync_write = broken
            COCKPIT.CMDQ.put({"op": "jog", "joint": "elbow_flex", "pos": NEUTRAL + 200})
            st = h.wait_until(lambda s: any("エラーが続く" in e["msg"] or "脱力しました" in e["msg"]
                                            for e in s["log"]), timeout=6.0)
            self.assertIsNotNone(st, "a repeating failure never stopped the arm")
            self.assertTrue(any("詳細:" in e["msg"] for e in st["log"]), "no traceback logged")


class TestGovernedState(unittest.TestCase):
    def test_follower_entries_carry_the_governor_view(self):
        with WorkerHarness() as h:
            st = h.wait_until(lambda s: s["follower"].get("elbow_flex", {}).get("state"))
            self.assertIsNotNone(st)
            e = st["follower"]["elbow_flex"]
            for key in ("state", "reason", "duty", "current", "current_max", "goal", "temp"):
                self.assertIn(key, e)
            self.assertIn("thermal", st["arm"])
            self.assertEqual(st["source"], "hw")

    def test_speed_preset_command(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            COCKPIT.CMDQ.put({"op": "speed", "preset": "slow"})
            self.assertIsNotNone(h.wait_until(lambda s: s["arm"].get("speed") == "slow"))

    def test_the_retired_torque_profile_says_so(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            COCKPIT.CMDQ.put({"op": "torque_profile", "profile": "safe"})
            st = h.wait_until(lambda s: any("廃止" in e["msg"] for e in s["log"]))
            self.assertIsNotNone(st, "a retired command was silently ignored")

    def test_session_settings_reach_the_follower(self):
        with WorkerHarness() as h:
            self.assertIsNotNone(h.wait_until(lambda s: s["connected"]))
            time.sleep(0.2)
            self.assertEqual(h.follower.regs["elbow_flex"].get("P_Coefficient"), 16)
            self.assertEqual(h.follower.regs["elbow_flex"].get("I_Coefficient"), 0)
            self.assertEqual(h.follower.regs["elbow_flex"].get("Lock"), 1)


try:
    import mujoco  # noqa: F401
    _HAVE_MUJOCO = True
except ImportError:  # pragma: no cover
    _HAVE_MUJOCO = False


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed (optional sim dep)")
class TestSimMode(unittest.TestCase):
    def test_the_simulated_cockpit_connects_and_follows(self):
        stop = threading.Event()
        saved = dict(COCKPIT.STATE)
        COCKPIT.STATE["log"] = []
        th = threading.Thread(target=COCKPIT.worker,
                              args=("auto", "auto", TeleopConfig(torque_stagger_s=0.0)),
                              kwargs={"stop": stop, "sim": True}, daemon=True)
        th.start()
        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                with COCKPIT.LOCK:
                    if COCKPIT.STATE["connected"] and COCKPIT.STATE["follower"]:
                        break
                time.sleep(0.05)
            with COCKPIT.LOCK:
                self.assertEqual(COCKPIT.STATE["source"], "sim")
                self.assertTrue(COCKPIT.STATE["ports"]["confirmed"])
            COCKPIT.CMDQ.put({"op": "teleop", "on": True})
            deadline = time.monotonic() + 5.0
            following = False
            while time.monotonic() < deadline and not following:
                with COCKPIT.LOCK:
                    following = COCKPIT.STATE["teleop"]
                time.sleep(0.05)
            self.assertTrue(following)
        finally:
            stop.set()
            th.join(timeout=5.0)
            COCKPIT.STATE.update(saved)


if __name__ == "__main__":
    unittest.main()
