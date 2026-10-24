"""SO-101 follower engine: references, write path, torque, modes.

Runs on plain Python 3.10 (no lerobot, no hardware): the buses are fakes that
speak the ServoIO interface, plus one integration test on the simulated arm
(skipped without mujoco).

Run: python -m unittest discover tests -p "test_so101_teleop_engine.py"
"""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import servo_config, sts3215  # noqa: E402
from robots.so101.servo_io import Command  # noqa: E402
from robots.so101.sts3215 import ServoState  # noqa: E402
from robots.so101.teleop_engine import (  # noqa: E402
    AUTO_FOLLOW_TOL_DEG, JOINTS, RELAX_ORDER, AutoFollowGate, OneEuroConfig, OneEuroFilter,
    TeleopConfig, TeleopEngine, deg_to_ticks, max_pose_gap, pose_gaps,
)

try:
    import mujoco  # noqa: F401
    _HAVE_MUJOCO = True
except ImportError:  # pragma: no cover
    _HAVE_MUJOCO = False


class FakeIO:
    """ServoIO stand-in: servos reach their goal at once (unless pinned)."""

    def __init__(self, present=None, volt=12.2):
        self.names = list(JOINTS)
        self.pos = dict(present) if present else {n: 2048 for n in JOINTS}
        self.volt = volt
        self.temp = {n: 30 for n in JOINTS}
        self.load = {n: 0 for n in JOINTS}
        self.faults = {n: 0 for n in JOINTS}
        self.torque = {n: 0 for n in JOINTS}
        self.regs = {n: {"P_Coefficient": 32, "I_Coefficient": 0, "Max_Voltage_Limit": 140,
                         "Max_Temperature_Limit": 70, "Operating_Mode": 0,
                         "Unloading_Condition": 44} for n in JOINTS}
        self.pinned = set()               # joints that do not move (obstacle)
        self.commands = []                # list of {joint: Command}
        self.sync = []                    # (reg, values)
        self.sick = set()                 # joints whose acknowledged writes fail
        self.refuse_p = set()
        self.silent = set()               # joints that do not answer reads
        self.drop_next_torque_off = set() # first Torque_Enable=0 to these is lost
        self.dropped_reads = 0            # telemetry reads to drop entirely (one each)

    def read_telemetry(self, names=None):
        out = {}
        drop_all = self.dropped_reads > 0
        if drop_all:
            self.dropped_reads -= 1
        for n in (names or self.names):
            if n in self.silent or drop_all:
                out[n] = ServoState(ok=False, pos=0, age=1)
                continue
            s = ServoState(ok=True, pos=self.pos[n], volt=self.volt, temp=self.temp[n],
                           load=self.load[n], status=self.faults[n])
            s.faults = sts3215.status_names(self.faults[n])
            out[n] = s
        return out

    def read_positions(self, names=None):
        return {n: self.pos[n] for n in (names or self.names)}

    def sync_write(self, reg, values):
        self.sync.append((reg, dict(values)))
        if reg == "Torque_Enable":
            for n, v in values.items():
                if v == 0 and n in self.drop_next_torque_off:
                    self.drop_next_torque_off.discard(n)      # lost on the wire
                    continue
                self.torque[n] = v

    def write_commands(self, cmds):
        self.commands.append(dict(cmds))
        for n, c in cmds.items():
            if n not in self.pinned and self.torque.get(n):
                self.pos[n] = c.goal

    def read_reg(self, reg, name):
        if name in self.silent:
            return None, 0
        if reg == "Present_Position":
            return self.pos[name], 0
        if reg == "Torque_Enable":
            return self.torque[name], 0
        return self.regs[name].get(reg, 0), 0

    def write_reg(self, reg, name, value):
        if name in self.sick:
            return False, 0x20
        if reg == "P_Coefficient" and name in self.refuse_p:
            return False, 0
        self.regs[name][reg] = int(value)
        return True, 0


class FakeLeader(FakeIO):
    pass


def make_engine(present=None, leader=None, **cfg_kw):
    cfg = TeleopConfig(torque_stagger_s=0.0, **cfg_kw)
    ranges = {n: (1000, 3100) for n in JOINTS}
    ranges["wrist_roll"] = (0, 4095)
    lead = FakeLeader(leader or present)
    foll = FakeIO(present)
    eng = TeleopEngine(lead, foll, ranges, config=cfg, sleep=lambda s: None)
    return eng, lead, foll


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
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        base, dev_in, dev_out = 1500.0, 0.0, 0.0
        for i in range(400):
            noisy = base + (8.0 if i % 2 else -8.0)
            out = f.filter(noisy, i / 60)
            if i > 100:
                dev_in += abs(noisy - base)
                dev_out += abs(out - base)
        self.assertLess(dev_out, dev_in * 0.3)

    def test_low_lag_on_fast_ramp(self):
        f = OneEuroFilter(OneEuroConfig(), nominal_dt=1 / 60)
        dt, vel = 1 / 60, 1000.0
        out = 0.0
        for i in range(120):
            out = f.filter(vel * i * dt, i * dt)
        self.assertLess(abs(out - vel * 119 * dt), 25.0)


# ---------------------------------------------------------------------------
class TestReferences(unittest.TestCase):
    def test_jump_guard_is_relative_to_the_previous_reference(self):
        eng, _, _ = make_engine(max_step_ticks=170)
        eng.last_ref = {n: 1500 for n in JOINTS}
        refs = eng.compute_refs({n: 2800 for n in JOINTS}, now=0.0)
        self.assertTrue(all(refs[n] == 1670 for n in JOINTS))

    def test_a_lagging_follower_does_not_rewind_the_reference(self):
        eng, _, foll = make_engine()
        eng.last_ref = {n: 1800 for n in JOINTS}
        foll.pos = {n: 1200 for n in JOINTS}            # follower far behind
        refs = eng.compute_refs({n: 1800 for n in JOINTS}, now=0.0)
        self.assertTrue(all(refs[n] == 1800 for n in JOINTS))

    def test_a_missing_leader_joint_keeps_its_last_reference(self):
        eng, _, _ = make_engine()
        eng.last_ref = {n: 1500 for n in JOINTS}
        refs = eng.compute_refs({**{n: 1500 for n in JOINTS}, "wrist_flex": None}, now=0.0)
        self.assertEqual(refs["wrist_flex"], 1500)


class TestSessionAndChecks(unittest.TestCase):
    def test_connect_writes_pdi_with_the_lock_closed_and_reads_p_back(self):
        eng, _, foll = make_engine()
        p = eng.configure_follower()
        for n in JOINTS:
            self.assertEqual(foll.regs[n]["Lock"], 1)
            self.assertEqual(foll.regs[n]["P_Coefficient"], 16)
            self.assertEqual(foll.regs[n]["I_Coefficient"], 0)
        self.assertEqual(p["elbow_flex"], 16)
        self.assertEqual(eng.governor.p_coeff["elbow_flex"], 16)

    def test_a_refused_p_is_reported_and_used_as_is(self):
        eng, _, foll = make_engine()
        foll.refuse_p.add("elbow_flex")               # stays at 32
        p = eng.configure_follower()
        self.assertEqual(p["elbow_flex"], 32)
        self.assertEqual(eng.governor.p_coeff["elbow_flex"], 32)

    def test_a_failed_instrument_check_demotes_the_governor_to_labels_only(self):
        eng, _, foll = make_engine()
        foll.volt = 3.0                                # nonsense reading
        eng.configure_follower()
        self.assertFalse(eng.instrument_ok)
        self.assertEqual(eng.governor.mode, "shadow")

    def test_audit_findings_are_kept(self):
        eng, _, foll = make_engine()
        foll.regs["elbow_flex"]["Max_Voltage_Limit"] = 120     # the 07-17 case
        eng.configure_follower()
        self.assertTrue(any(f.servo == "elbow_flex" and f.register == "Max_Voltage_Limit"
                            for f in eng.findings))


class TestTorqueAndFollow(unittest.TestCase):
    def test_start_holds_here_before_any_torque_enable(self):
        eng, _, foll = make_engine(present={n: 1800 for n in JOINTS})
        ok, _ = eng.start_teleop(now=0.0)
        self.assertTrue(ok and eng.active)
        first_torque = next(i for i, (r, _) in enumerate(foll.sync) if r == "Torque_Enable")
        self.assertTrue(foll.commands, "no hold command before torque")
        self.assertTrue(all(c.goal == 1800 for c in foll.commands[0].values()))
        self.assertEqual(first_torque, 0)              # holds are commands, then torque
        self.assertTrue(all(foll.torque[n] == 1 for n in JOINTS))

    def test_following_moves_the_follower_toward_the_leader(self):
        eng, lead, foll = make_engine(present={n: 2000 for n in JOINTS})
        eng.start_teleop(now=0.0)
        lead.pos["shoulder_pan"] = 2100
        for i in range(60):
            eng.cycle(now=0.02 + i / 60)
        self.assertGreater(foll.pos["shoulder_pan"], 2080)

    def test_a_still_arm_sends_nothing(self):
        # a new position command clears the servo's overload flag: streaming the
        # same goal would keep resetting the firmware's own protection
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        for i in range(10):
            eng.cycle(now=2.0 + i / 60)            # past the 1.5 s soft start
        n_before = len(foll.commands)
        for i in range(30):
            eng.cycle(now=3.0 + i / 60)
        self.assertEqual(len(foll.commands), n_before)

    def test_soft_start_ramps_the_speed(self):
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        lead.pos["elbow_flex"] = 2400
        eng.cycle(now=0.05)
        early = [c["elbow_flex"].velocity for c in foll.commands if "elbow_flex" in c][-1]
        for i in range(120):
            lead.pos["elbow_flex"] = 2400 + (i % 2) * 40
            eng.cycle(now=0.1 + i / 60)
        late = [c["elbow_flex"].velocity for c in foll.commands if "elbow_flex" in c][-1]
        self.assertLess(early, late)
        self.assertEqual(late, eng.goal_velocity())

    def test_speed_presets(self):
        eng, _, _ = make_engine()
        eng.set_speed("slow")
        slow = eng.goal_velocity()
        eng.set_speed("fast")
        self.assertGreater(eng.goal_velocity(), slow)
        with self.assertRaises(ValueError):
            eng.set_speed("ludicrous")

    def test_jog_moves_one_joint_while_idle(self):
        eng, _, foll = make_engine()
        eng.jog("wrist_flex", 2300)
        eng.cycle(now=0.0)
        self.assertEqual(foll.pos["wrist_flex"], 2300)
        self.assertEqual(foll.torque["wrist_flex"], 1)

    def test_stop_holds_where_the_follower_is(self):
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        lead.pos["shoulder_pan"] = 2300
        for i in range(30):
            eng.cycle(now=0.02 + i / 60)
        eng.stop_teleop()
        here = foll.pos["shoulder_pan"]
        lead.pos["shoulder_pan"] = 1500                # leader keeps moving
        for i in range(30):
            eng.cycle(now=2.0 + i / 60)
        self.assertEqual(foll.pos["shoulder_pan"], here)


class TestRelax(unittest.TestCase):
    def test_relax_order_is_tip_to_base(self):
        self.assertEqual(RELAX_ORDER, list(reversed(JOINTS)))
        self.assertEqual(RELAX_ORDER[0], "gripper")
        self.assertEqual(RELAX_ORDER[-1], "shoulder_pan")

    def test_relax_cuts_every_joint_even_with_a_tripped_servo(self):
        # acknowledged writes to a servo in protection fail; sync writes cannot
        eng, _, foll = make_engine()
        eng.start_teleop(now=0.0)
        foll.sick.add("elbow_flex")
        order = eng.relax_follower(delay=0.0)
        self.assertEqual(order, list(reversed(JOINTS)))
        offs = [list(v)[0] for r, v in foll.sync if r == "Torque_Enable" and 0 in v.values()]
        self.assertEqual(offs, list(reversed(JOINTS)))
        self.assertTrue(all(foll.torque[n] == 0 for n in JOINTS))
        self.assertFalse(eng.active)

    def test_relax_staggers_between_joints_only(self):
        eng, _, _ = make_engine()
        sleeps = []
        eng.relax_follower(delay=0.05, sleep=lambda s: sleeps.append(s))
        self.assertEqual(sleeps, [0.05] * (len(JOINTS) - 1))


class TestGovernorInTheLoop(unittest.TestCase):
    def test_a_pinned_joint_pushing_hard_is_eased_and_held(self):
        eng, lead, foll = make_engine(present={n: 2000 for n in JOINTS})
        eng.start_teleop(now=0.0)
        foll.pinned.add("elbow_flex")
        lead.pos["elbow_flex"] = 2200
        t = 0.02
        for _ in range(int(2.5 * 60)):
            foll.load["elbow_flex"] = 700 if eng.governor._j["elbow_flex"].state == "track" else 250
            eng.cycle(now=t)
            t += 1 / 60
        js = eng.last_out.joints["elbow_flex"]
        self.assertEqual(js.state, "hold")
        self.assertLess(js.goal, 2200)

    def test_a_thermal_stop_ends_following(self):
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        foll.temp["shoulder_lift"] = 67
        for t in (0.1, 0.11, 0.12):                # a jump counts after 3 agreeing reads
            eng.cycle(now=t)
        self.assertFalse(eng.active)
        ok, why = eng.start_teleop(now=0.2)
        self.assertFalse(ok)
        self.assertIn("温度", why)

    def test_a_retreat_moves_to_the_cooling_pose_then_holds(self):
        eng, lead, foll = make_engine(present={n: 2050 for n in JOINTS})
        eng.start_teleop(now=0.0)
        foll.temp["elbow_flex"] = 69
        t = 0.02
        for _ in range(60 * 20):
            eng.cycle(now=t)
            t += 1 / 60
            if not eng.retreating and t > 1.0:
                break
        target = eng.governor.retreat_pose_ticks(foll.read_telemetry())
        self.assertAlmostEqual(foll.pos["elbow_flex"], target["elbow_flex"], delta=3)
        self.assertFalse(eng.retreating)

    def test_metrics_are_published(self):
        eng, _, _ = make_engine()
        eng.cycle(now=0.0)
        m = eng.metrics()
        for key in ("measured_hz", "cycle_ms_p99", "speed", "governor_mode"):
            self.assertIn(key, m)


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed (optional sim dep)")
class TestOnTheSimulatedArm(unittest.TestCase):
    def test_follows_a_moving_leader_without_tripping(self):
        from robots.so101.sim.servo_sim import ScriptedLeaderBus, SimArm, SimFollowerBus
        ranges = {"shoulder_pan": (874, 3159), "shoulder_lift": (815, 3145),
                  "elbow_flex": (859, 3065), "wrist_flex": (911, 3193),
                  "wrist_roll": (0, 4095), "gripper": (1605, 3086)}
        arm = SimArm(ranges=ranges, start_deg=[0, -30, 40, 20, 0])
        bus = SimFollowerBus(arm, realtime=False)
        import math
        path = lambda t: {"shoulder_pan": 25 * math.sin(t), "shoulder_lift": -30 + 10 * math.sin(0.7 * t),
                          "elbow_flex": 40, "wrist_flex": 20}
        lead = ScriptedLeaderBus(ranges, path, clock=lambda: arm.t)
        eng = TeleopEngine(lead, bus, ranges, config=TeleopConfig(torque_stagger_s=0.0),
                           sleep=lambda s: None)
        eng.configure_follower()
        eng.start_teleop(now=0.0)
        t = 0.0
        worst = 0
        while t < 6.0:
            res = eng.cycle(now=t)
            arm.advance(1 / 60)
            t += 1 / 60
            if t > 2.0:
                worst = max(worst, abs(res["follower"]["shoulder_pan"].pos
                                       - lead.ticks()["shoulder_pan"]))
        self.assertTrue(all(not any(s.trips.values()) for s in arm.servos))
        self.assertLess(worst, deg_to_ticks(8.0))      # lag of a 25 deg sweep at 45 deg/s


# ---------------------------------------------------------------------------
# regressions from the 2026-10-24 code audit
class TestAuditRegressions(unittest.TestCase):
    def test_relax_with_a_pending_retreat_stays_limp(self):
        eng, lead, foll = make_engine(present={n: 2050 for n in JOINTS})
        eng.start_teleop(now=0.0)
        foll.temp["elbow_flex"] = 69                  # retreat requested
        eng.relax_follower(delay=0.0)
        for i in range(30):
            eng.cycle(now=0.1 + i / 60)
        self.assertTrue(all(foll.torque[n] == 0 for n in JOINTS), foll.torque)
        self.assertFalse(eng.retreating)

    def test_abort_ends_a_retreat_and_it_does_not_restart_by_itself(self):
        eng, lead, foll = make_engine(present={n: 2050 for n in JOINTS})
        eng.start_teleop(now=0.0)
        foll.temp["elbow_flex"] = 69
        for t in (0.05, 0.06, 0.07):
            eng.cycle(now=t)
        self.assertTrue(eng.retreating)
        eng.stop_teleop()                              # ABORT
        for i in range(30):
            eng.cycle(now=0.1 + i / 60)
        self.assertFalse(eng.retreating)

    def test_a_joint_that_cannot_be_read_is_never_energized_or_sent_zero(self):
        eng, lead, foll = make_engine(present={n: 2000 for n in JOINTS})
        foll.silent.add("elbow_flex")
        eng.enable_follower_torque_staged()
        self.assertEqual(foll.torque["elbow_flex"], 0)
        for i in range(10):
            eng.cycle(now=i / 60)
        sent = [c["elbow_flex"].goal for c in foll.commands if "elbow_flex" in c]
        self.assertEqual(sent, [])
        ok, why = eng.start_teleop(now=1.0)
        self.assertFalse(ok)
        self.assertIn("elbow_flex", why)

    def test_switching_one_joint_on_mid_follow_is_refused(self):
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        eng.set_follower_torque("wrist_flex", False)
        self.assertFalse(eng.set_follower_torque("wrist_flex", True))
        self.assertEqual(foll.torque["wrist_flex"], 0)

    def test_supply_sag_slows_acceleration_too(self):
        eng, lead, foll = make_engine()
        eng.start_teleop(now=0.0)
        foll.volt = 10.4
        t = 2.0
        for _ in range(40):
            lead.pos["shoulder_pan"] += 3
            eng.cycle(now=t)
            t += 1 / 60
        c = [c["shoulder_pan"] for c in foll.commands if "shoulder_pan" in c][-1]
        self.assertLess(c.acceleration, eng.cfg.session.acceleration)
        self.assertLess(c.velocity, eng.goal_velocity())

    def test_one_dropped_read_does_not_fail_the_instrument_check(self):
        eng, _, foll = make_engine()
        foll.dropped_reads = 1
        self.assertTrue(eng.instrument_check())
        self.assertEqual(eng.governor.mode, "active")

    def test_relax_is_read_back_and_resent(self):
        eng, _, foll = make_engine()
        eng.start_teleop(now=0.0)
        foll.drop_next_torque_off.add("shoulder_lift")
        eng.relax_follower(delay=0.0)
        self.assertEqual(foll.torque["shoulder_lift"], 0)

    def test_leaving_the_cooling_pose_rearms_the_automatic_retreat(self):
        eng, lead, foll = make_engine(present={n: 2050 for n in JOINTS})
        eng.start_teleop(now=0.0)
        foll.temp["elbow_flex"] = 69
        t = 0.02
        for _ in range(60 * 20):
            eng.cycle(now=t)
            t += 1 / 60
            if not eng.retreating and t > 1.0:
                break
        self.assertIsNotNone(eng._cool_target)
        eng.jog("elbow_flex", foll.pos["elbow_flex"] + deg_to_ticks(30))
        # the jog goes out, the move is seen and re-arms, the next step asks again
        for k in range(3):
            eng.cycle(now=t + 0.02 * k)
        self.assertTrue(eng.retreating, "the hot arm moved out of the pose and nothing re-armed")


# ---------------------------------------------------------------------------
def pose(**over):
    p = {n: 2048 for n in JOINTS}
    p.update(over)
    return p


class TestPoseGap(unittest.TestCase):
    def test_worst_joint_is_reported(self):
        joint, gap = max_pose_gap(pose(elbow_flex=2148, gripper=2078), pose())
        self.assertEqual((joint, gap), ("elbow_flex", 100))

    def test_gap_is_unsigned(self):
        self.assertEqual(pose_gaps(pose(wrist_roll=1948), pose())["wrist_roll"], 100)

    def test_identical_poses_have_no_gap(self):
        self.assertEqual(max_pose_gap(pose(), pose()), ("shoulder_pan", 0))

    def test_missing_joints_are_skipped_not_faked(self):
        self.assertEqual(max_pose_gap({}, {}), (None, 0))
        self.assertEqual(pose_gaps(pose(elbow_flex=None), pose()).get("elbow_flex"), None)


class TestAutoFollowGate(unittest.TestCase):
    def make(self, **kw):
        kw.setdefault("tol_ticks", deg_to_ticks(20.0))
        kw.setdefault("hold_cycles", 3)
        return AutoFollowGate(**kw)

    def feed(self, gate, leader, follower, times):
        return [gate.update(leader, follower) for _ in range(times)]

    def test_engages_after_the_hold_period_once_the_arms_agree(self):
        self.assertEqual(self.feed(self.make(), pose(), pose(), 3), [False, False, True])

    def test_a_gap_beyond_tolerance_never_engages(self):
        gate = self.make()
        far = pose(shoulder_lift=2048 + deg_to_ticks(45))
        self.assertNotIn(True, self.feed(gate, far, pose(), 20))
        self.assertIn("shoulder_lift", gate.status)

    def test_a_leader_swinging_through_the_pose_does_not_engage(self):
        gate = self.make()
        moving = deg_to_ticks(10)
        engaged = [gate.update(pose(wrist_flex=2048 + i * moving), pose()) for i in range(3)]
        self.assertNotIn(True, engaged)
        self.assertIn("動いています", gate.status)

    def test_the_hold_restarts_when_the_arms_drift_apart(self):
        gate = self.make()
        self.feed(gate, pose(), pose(), 2)
        gate.update(pose(elbow_flex=2048 + deg_to_ticks(40)), pose())
        self.assertEqual(gate.held, 0)
        self.assertEqual(self.feed(gate, pose(), pose(), 2), [False, False])

    def test_engaging_is_a_one_shot_per_reset(self):
        gate = self.make()
        self.feed(gate, pose(), pose(), 3)
        gate.reset()
        self.assertEqual(gate.held, 0)
        self.assertEqual(self.feed(gate, pose(), pose(), 3), [False, False, True])

    def test_tolerance_boundary_is_inclusive(self):
        gate = self.make(tol_ticks=100, hold_cycles=1)
        self.assertTrue(gate.update(pose(gripper=2148), pose()))
        gate.reset()
        self.assertFalse(gate.update(pose(gripper=2149), pose()))

    def test_status_reports_the_gap_in_degrees(self):
        gate = self.make()
        gate.update(pose(elbow_flex=2048 + deg_to_ticks(45)), pose())
        self.assertIn("45.0°", gate.status)

    def test_defaults_are_a_usable_window(self):
        gate = AutoFollowGate()
        self.assertEqual(gate.tol_ticks, deg_to_ticks(AUTO_FOLLOW_TOL_DEG))
        self.assertLess(gate.still_ticks, gate.tol_ticks)


class TestGateNeedsEveryJoint(unittest.TestCase):
    def test_an_unread_joint_blocks_auto_follow(self):
        gate = AutoFollowGate(tol_ticks=deg_to_ticks(20.0), hold_cycles=1)
        leader = pose()
        leader.pop("wrist_flex")
        self.assertFalse(gate.update(leader, pose()))
        self.assertIn("wrist_flex", gate.status)


if __name__ == "__main__":
    unittest.main()
