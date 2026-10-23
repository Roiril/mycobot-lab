"""FollowerGovernor: state transitions on synthetic telemetry (pure, no hardware).

Each test drives the governor with ServoState sequences shaped like the
situations it has to tell apart: a joint pressing on an obstacle, a joint
holding a weight, a firmware protection trip, a hot servo, a sagging supply.

Run: python -m unittest discover tests -p "test_so101_governor.py"
"""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import sts3215  # noqa: E402
from robots.so101.governor import FollowerGovernor, GovernorParams  # noqa: E402
from robots.so101.sts3215 import ServoState  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
RANGES = {n: (1000, 3000) for n in JOINTS}
RANGES["wrist_roll"] = (0, 4095)
DT = 1 / 60


def deg(d):
    return int(round(sts3215.deg_to_ticks(d)))


class Rig:
    """Feeds the governor and keeps the 'arm' wherever the test puts it."""

    def __init__(self, **gov_kw):
        self.g = FollowerGovernor(JOINTS, RANGES, gov_kw.pop("params", None), **gov_kw)
        self.pos = {n: 2000 for n in JOINTS}
        self.load = {n: 0 for n in JOINTS}
        self.cur = {n: 0.0 for n in JOINTS}
        self.vel = {n: 0 for n in JOINTS}
        self.temp = {n: 30 for n in JOINTS}
        self.volt = 12.2
        self.faults = {n: 0 for n in JOINTS}
        self.ok = {n: True for n in JOINTS}
        self.age = {n: 0 for n in JOINTS}
        self.torque = {n: True for n in JOINTS}
        self.t = 0.0
        self.g.reset(self.states())

    def states(self):
        out = {}
        for n in JOINTS:
            s = ServoState(ok=self.ok[n], pos=self.pos[n], vel=self.vel[n], load=self.load[n],
                           volt=self.volt, temp=self.temp[n], status=self.faults[n],
                           current=self.cur[n], age=self.age[n])
            s.faults = sts3215.status_names(self.faults[n])
            out[n] = s
        return out

    def step(self, refs=None, n=1):
        out = None
        for _ in range(n):
            r = {j: self.pos[j] for j in JOINTS}
            if refs:
                r.update(refs)
            out = self.g.step(r, self.states(), self.torque, self.t)
            self.t += DT
        return out


class TestClamp(unittest.TestCase):
    def test_goal_stays_off_the_end_stops(self):
        g = FollowerGovernor(JOINTS, RANGES)
        self.assertEqual(g.clamp_goal("elbow_flex", 990), 1000 + round(deg(3.0)))
        self.assertEqual(g.clamp_goal("elbow_flex", 3500), 3000 - round(deg(3.0)))
        self.assertEqual(g.clamp_goal("elbow_flex", 2000), 2000)

    def test_full_turn_joint_is_not_clamped(self):
        g = FollowerGovernor(JOINTS, RANGES)
        self.assertEqual(g.clamp_goal("wrist_roll", 4095), 4095)

    def test_tracking_passes_the_reference_through(self):
        rig = Rig()
        out = rig.step({"shoulder_pan": 2100})
        self.assertEqual(out.goals["shoulder_pan"], 2100)
        self.assertEqual(out.joints["shoulder_pan"].state, "track")


class TestObstacle(unittest.TestCase):
    """The elbow is pressed on something: stuck short, pushing hard."""

    def press(self, rig, joint="elbow_flex", gap_deg=12.0, duty=650):
        ref = rig.pos[joint] + deg(gap_deg)
        rig.load[joint] = duty
        rig.cur[joint] = 1.3
        return ref

    def test_pushing_on_an_obstacle_ends_in_hold_with_less_push(self):
        rig = Rig()
        ref = self.press(rig)
        out = rig.step({"elbow_flex": ref}, n=int(0.6 / DT))     # block timer fires
        self.assertEqual(out.joints["elbow_flex"].state, "probe")
        eased = out.goals["elbow_flex"]
        self.assertLess(eased - rig.pos["elbow_flex"], ref - rig.pos["elbow_flex"])
        # the joint does not move while eased: an obstacle
        out = rig.step({"elbow_flex": ref}, n=int(0.4 / DT))
        self.assertEqual(out.joints["elbow_flex"].state, "hold")
        self.assertEqual(out.goals["elbow_flex"], eased)
        self.assertIn("当たって", out.message)

    def test_hold_releases_when_the_leader_comes_back(self):
        rig = Rig()
        ref = self.press(rig)
        rig.step({"elbow_flex": ref}, n=int(1.0 / DT))
        rig.load["elbow_flex"] = 100
        out = rig.step({"elbow_flex": rig.pos["elbow_flex"] + deg(1.0)})
        self.assertEqual(out.joints["elbow_flex"].state, "track")

    def test_hold_releases_when_the_leader_moves_the_other_way(self):
        rig = Rig()
        ref = self.press(rig)
        rig.step({"elbow_flex": ref}, n=int(1.0 / DT))
        out = rig.step({"elbow_flex": rig.pos["elbow_flex"] - deg(10.0)})
        self.assertEqual(out.joints["elbow_flex"].state, "track")

    def test_gravity_droop_is_not_an_obstacle(self):
        # shoulder held out: error = droop, load = what gravity needs -> no probe
        rig = Rig()
        rig.pos["shoulder_lift"] = 2000
        rig.load["shoulder_lift"] = 280
        out = rig.step({"shoulder_lift": 2000 + deg(6.0)}, n=int(2.0 / DT))
        self.assertEqual(out.joints["shoulder_lift"].state, "track")

    def test_a_moving_joint_is_never_held(self):
        rig = Rig()
        ref = self.press(rig)
        rig.vel["elbow_flex"] = deg(20.0)
        out = rig.step({"elbow_flex": ref}, n=int(2.0 / DT))
        self.assertEqual(out.joints["elbow_flex"].state, "track")

    def test_joints_pushing_together_ease_together(self):
        # a contact couples the joints: easing one alone lets the other shove it
        rig = Rig()
        ref_e = self.press(rig, "elbow_flex", duty=650)
        ref_s = self.press(rig, "shoulder_lift", duty=520)
        rig.step({"elbow_flex": ref_e, "shoulder_lift": ref_s}, n=int(0.3 / DT))
        out = rig.step({"elbow_flex": ref_e, "shoulder_lift": ref_s}, n=int(0.3 / DT))
        self.assertEqual(out.joints["elbow_flex"].state, "probe")
        self.assertEqual(out.joints["shoulder_lift"].state, "probe")


class TestWeight(unittest.TestCase):
    def test_a_sag_while_eased_is_a_weight_and_the_goal_is_restored(self):
        rig = Rig()
        j = "shoulder_lift"
        ref = rig.pos[j] + deg(14.0)
        rig.load[j] = 650
        rig.cur[j] = 1.8
        out = rig.step({j: ref}, n=int(0.3 / DT))
        while out.joints[j].state != "probe":
            out = rig.step({j: ref})
        before = out.goals[j]
        rig.pos[j] -= deg(4.0)              # the arm sags under the weight
        out = rig.step({j: ref})
        self.assertEqual(out.joints[j].state, "load")
        self.assertGreater(out.goals[j], before)        # old goal back: no drop
        self.assertIn("重い", out.message)

    def test_a_slow_sag_found_during_hold_is_also_a_weight(self):
        rig = Rig()
        j = "elbow_flex"
        ref = rig.pos[j] + deg(12.0)
        rig.load[j] = 600
        out = rig.step({j: ref}, n=int(1.2 / DT))
        self.assertEqual(out.joints[j].state, "hold")
        rig.pos[j] -= deg(3.5)
        out = rig.step({j: ref})
        self.assertEqual(out.joints[j].state, "load")

    def test_a_known_weight_is_not_re_probed_until_the_pose_changes(self):
        rig = Rig()
        j = "shoulder_lift"
        ref = rig.pos[j] + deg(14.0)
        rig.load[j] = 650
        out = rig.step({j: ref}, n=int(0.6 / DT))
        rig.pos[j] -= deg(4.0)
        rig.step({j: ref})
        out = rig.step({j: ref}, n=int(20.0 / DT))
        self.assertEqual(out.joints[j].state, "load")
        out = rig.step({j: ref + deg(8.0)})                 # new pose
        self.assertEqual(out.joints[j].state, "track")


class TestProtect(unittest.TestCase):
    def test_a_protection_bit_freezes_the_joint(self):
        rig = Rig()
        rig.faults["elbow_flex"] = 0x20
        out = rig.step({"elbow_flex": rig.pos["elbow_flex"] + deg(10)})
        self.assertEqual(out.joints["elbow_flex"].state, "protect")
        self.assertEqual(out.goals["elbow_flex"], rig.pos["elbow_flex"])
        self.assertIn("過負荷", out.message)

    def test_it_resumes_one_second_after_the_bit_clears(self):
        rig = Rig()
        rig.faults["elbow_flex"] = 0x08
        rig.step()
        rig.faults["elbow_flex"] = 0
        out = rig.step(n=int(0.5 / DT))
        self.assertEqual(out.joints["elbow_flex"].state, "protect")
        out = rig.step(n=int(0.7 / DT))
        self.assertEqual(out.joints["elbow_flex"].state, "track")

    def test_three_trips_in_five_minutes_open_the_breaker(self):
        rig = Rig()
        for _ in range(3):
            rig.faults["elbow_flex"] = 0x20
            rig.step()
            rig.faults["elbow_flex"] = 0
            out = rig.step(n=int(1.5 / DT))
        self.assertFalse(out.follow_allowed)
        self.assertIn("保護", out.stop_reason)
        rig.g.clear_breaker()
        self.assertTrue(rig.step().follow_allowed)


class TestThermal(unittest.TestCase):
    def test_levels_escalate_and_stop_following(self):
        rig = Rig()
        rig.temp["elbow_flex"] = 56
        self.assertEqual(rig.step().thermal, "warn")
        rig.temp["elbow_flex"] = 63
        self.assertEqual(rig.step().thermal, "alert")
        rig.temp["elbow_flex"] = 66
        out = rig.step()
        self.assertEqual(out.thermal, "stop")
        self.assertFalse(out.follow_allowed)
        self.assertFalse(out.retreat)
        rig.temp["elbow_flex"] = 68
        out = rig.step()
        self.assertTrue(out.retreat)

    def test_no_flicker_on_a_one_degree_wobble(self):
        rig = Rig()
        rig.temp["elbow_flex"] = 66
        rig.step()
        rig.temp["elbow_flex"] = 65
        self.assertEqual(rig.step().thermal, "stop")

    def test_resume_needs_a_cool_arm(self):
        rig = Rig()
        rig.temp["elbow_flex"] = 66
        rig.step()
        rig.temp["elbow_flex"] = 63
        rig.step()
        self.assertFalse(rig.g.clear_thermal_stop())
        rig.temp["elbow_flex"] = 50
        rig.step()
        self.assertTrue(rig.g.clear_thermal_stop())

    def test_the_cooling_pose_stands_the_arm_up(self):
        rig = Rig()
        target = rig.g.retreat_pose_ticks(rig.states())
        self.assertEqual(target["shoulder_pan"], rig.pos["shoulder_pan"])   # pan kept
        mid = (RANGES["elbow_flex"][0] + RANGES["elbow_flex"][1]) / 2
        self.assertAlmostEqual(target["elbow_flex"], mid - 80 * 4095 / 360, delta=2)


class TestSupply(unittest.TestCase):
    def test_sag_slows_the_arm_and_recovers(self):
        rig = Rig()
        rig.volt = 10.5
        out = rig.step(n=int(0.4 / DT))
        self.assertLess(out.speed_scale, 1.0)
        self.assertIn("電圧", out.message)
        rig.volt = 12.1
        out = rig.step(n=int(1.0 / DT))
        self.assertLess(out.speed_scale, 1.0)            # not yet: needs 2 s
        out = rig.step(n=int(1.5 / DT))
        self.assertEqual(out.speed_scale, 1.0)


class TestGripper(unittest.TestCase):
    def test_closing_on_an_object_becomes_a_constant_squeeze(self):
        rig = Rig()
        ref = rig.pos["gripper"] + deg(20)
        rig.load["gripper"] = 450
        out = rig.step({"gripper": ref}, n=int(0.5 / DT))
        self.assertEqual(out.joints["gripper"].state, "grip")
        squeeze = out.goals["gripper"] - rig.pos["gripper"]
        self.assertGreater(squeeze, 0)
        self.assertLess(squeeze, ref - rig.pos["gripper"])
        out = rig.step({"gripper": rig.pos["gripper"] - deg(5)})     # opening
        self.assertEqual(out.joints["gripper"].state, "track")


class TestOffStaleShadow(unittest.TestCase):
    def test_a_torque_off_joint_is_told_to_stay_where_it_is(self):
        rig = Rig()
        rig.torque["wrist_flex"] = False
        rig.pos["wrist_flex"] = 2345
        out = rig.step({"wrist_flex": 2800})
        self.assertEqual(out.goals["wrist_flex"], 2345)
        self.assertEqual(out.joints["wrist_flex"].state, "off")

    def test_a_silent_joint_keeps_its_goal_and_goes_stale(self):
        rig = Rig()
        rig.step({"wrist_flex": 2100})
        rig.ok["wrist_flex"] = False
        rig.age["wrist_flex"] = 20
        out = rig.step({"wrist_flex": 2500})
        self.assertEqual(out.goals["wrist_flex"], 2100)
        self.assertEqual(out.joints["wrist_flex"].state, "stale")

    def test_shadow_mode_labels_but_commands_the_plain_reference(self):
        rig = Rig(mode="shadow")
        ref = rig.pos["elbow_flex"] + deg(12)
        rig.load["elbow_flex"] = 650
        out = rig.step({"elbow_flex": ref}, n=int(1.0 / DT))
        self.assertIn(out.joints["elbow_flex"].state, ("probe", "hold"))
        self.assertEqual(out.goals["elbow_flex"], ref)
        self.assertTrue(out.follow_allowed)


class TestGravityAwareness(unittest.TestCase):
    def test_gravity_duty_comes_from_the_model(self):
        g = FollowerGovernor(JOINTS, RANGES)
        st = {n: ServoState(ok=True, pos=2000) for n in JOINTS}
        u = g.gravity_duty(st)
        # all joints at mid range: upper arm up, forearm horizontal -> elbow ~0.45 N*m
        self.assertAlmostEqual(u["elbow_flex"], 0.453 / 2.94 * 1000, delta=10)
        self.assertEqual(u["gripper"], 0.0)


if __name__ == "__main__":
    unittest.main()
