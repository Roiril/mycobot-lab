"""Follower governor: decide what each SO-101 servo is told, from what it reports.

Pure logic (no I/O). One `step()` per control cycle takes the reference per
joint (where the operator wants it), the servos' telemetry (sts3215.ServoState)
and the torque state, and returns the goal per joint plus a status the cockpit
can show. Design and the reasoning behind every threshold:
.agent/plans/2026-10-24_so101-motion-core.md (section 3.2).

The physics in one paragraph
----------------------------
An STS3215 is a P(D) position servo: its output is proportional to how far it
is from its goal (duty ~= P/4 permille per tick, not yet measured on this arm).
So a joint that cannot reach its goal - pressed on the table, on its own end
stop, or holding more than it can - keeps pushing, and the push only grows with
the gap. Gravity alone needs at most 29 % (shoulder) / 15 % (elbow); the
firmware's overload protection needs > 80 % for 2 s and the 2 A over-current
trip ~74 % at stall. Sitting there is what tripped and overheated the elbow.
The cure is to stop asking: when a joint is stuck and pushing harder than its
own weight explains, its goal is frozen where it is. A P servo told to stay
where it already is pushes with (almost) nothing, sags by its gravity droop and
holds. That needs no knowledge of the gain k nor of any sign, which is why it is
the v1 mechanism.

States per joint
----------------
OFF      torque off: goal = present (enabling never jumps)
STALE    no telemetry for a while: keep the last goal
TRACK    goal = reference, clamped a few degrees inside the calibrated range
PROBE    stuck short of its goal and pushing harder than its own weight explains:
         the error the servo sees is scaled down (output ~ error, so this scales
         its push without knowing the gain) and we watch for 0.3 s
HOLD     ...and it did not move: an obstacle. The eased goal is kept until the
         reference comes back
LOAD     ...and it sagged: a weight. The old goal is restored at once (the arm
         does not drop) and the joint keeps tracking, labelled "heavy"
GRIP     gripper closed on something: the same proportional ease to a squeeze
PROTECT  a firmware protection bit is set: goal frozen where the joint is. The
         engine sends only changed commands, so the freeze goes out once - and a
         new position command is what clears overload / over-current

Arm-level
---------
- circuit breaker: 3 PROTECT entries within 5 min on any joint -> stop following
- temperature: warn 55 C, alert 62 C, stop following 66 C, retreat to the
  gravity-neutral pose 68 C (the firmware cuts torque at 70 C and drops the arm)
- supply sag: < 10.8 V for 0.3 s -> speed scale 0.6 until > 11.3 V for 2 s
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

from . import gravity, profile, sts3215
from .sts3215 import ServoState

GRIPPER = profile.GRIPPER_NAME
FULL_TURN = ("wrist_roll",)          # no end stop to keep away from

# Gravity-neutral pose for cooling (the arm stands up; holding torque ~0.02 N*m
# in total). pan and roll are kept from the current pose.
COOL_LIFT_DEG = -10.0
COOL_ELBOW_DEG = -80.0
COOL_WRIST_DEG = 0.0


@dataclass
class GovernorParams:
    soft_margin_deg: float = 3.0         # never command the calibrated end stops
    gripper_margin_deg: float = 1.5
    still_dps: float = 3.0
    block_err_deg: float = 2.5
    block_droop_margin_deg: float = 2.0
    block_duty_over_gravity: float = 250.0   # permille the joint pushes beyond its own weight
    block_duty_min: float = 400.0
    block_time_s: float = 0.5
    effort_duty: float = 700.0           # above this (or the current below) the block
    effort_current_a: float = 1.6        # timer runs twice as fast (80 %/2 s, 2 A/2 s trips)
    yield_over_gravity: float = 150.0    # eased push: own weight + this much contact
    yield_min_duty: float = 200.0
    probe_time_s: float = 0.3
    probe_sag_deg: float = 1.5           # moved this far away from the goal -> a weight
    load_release_deg: float = 5.0        # a known weight is re-checked only after a new pose
    hold_release_deg: float = 2.0
    protect_clear_s: float = 1.0
    breaker_count: int = 3
    breaker_window_s: float = 300.0
    temp_warn_c: float = 55.0
    temp_alert_c: float = 62.0
    temp_stop_c: float = 66.0
    temp_retreat_c: float = 68.0
    temp_resume_c: float = 62.0
    temp_hysteresis_c: float = 2.0
    volt_low_v: float = 10.8
    volt_ok_v: float = 11.3
    volt_low_s: float = 0.3
    volt_ok_s: float = 2.0
    volt_scale: float = 0.6
    stale_cycles: int = 15
    grip_detect_duty: float = 300.0
    grip_detect_s: float = 0.3
    grip_duty: float = 200.0             # squeeze; under the gripper's Overload_Torque = 25 %
    duty_window: int = 3


# States whose goal must take effect at once: the servo's internal profile
# would otherwise walk its setpoint back toward an eased goal at the normal
# speed limit (seconds, at the soft-start speed) while it keeps pushing.
FAST_STATES = ("probe", "hold", "load", "protect", "grip")


@dataclass
class JointStatus:
    state: str = "off"
    reason: str = ""
    goal: int = 0
    fast: bool = False                   # bypass the servo's speed / acceleration limits
    ref: Optional[int] = None
    duty: float = 0.0                    # averaged |Present_Load|, permille
    current: float = 0.0                 # averaged |Present_Current|, A
    gravity: float = 0.0                 # permille needed to hold the pose
    temp: int = 0
    faults: Tuple[str, ...] = ()
    ok: bool = False


@dataclass
class GovernorOutput:
    goals: Dict[str, int]
    joints: Dict[str, JointStatus]
    follow_allowed: bool
    stop_reason: str
    retreat: bool                        # ask the engine to move to the cooling pose
    speed_scale: float
    thermal: str                         # ok | warn | alert | stop | retreat
    supply_v: float
    message: str
    message_level: str                   # info | warn | danger


@dataclass
class _Joint:
    state: str = "off"
    reason: str = ""
    goal: Optional[int] = None
    hold_goal: Optional[int] = None
    prev_goal: Optional[int] = None
    probe_pos: int = 0
    probe_expect: float = 0.0            # how far a weight would sag after the ease
    push_sign: int = 0
    ready: bool = False                  # block timer fired this cycle
    pushing: bool = False                # also pressing (eases off with the others)
    duty: Deque[float] = field(default_factory=lambda: deque(maxlen=3))
    current: Deque[float] = field(default_factory=lambda: deque(maxlen=3))
    t_block: float = 0.0
    t_grip: float = 0.0
    t_clear: float = 0.0
    t_probe: float = 0.0
    t_load: float = 0.0
    last_faults: Tuple[str, ...] = ()


class FollowerGovernor:
    def __init__(self, joints: List[str], ranges: Dict[str, Tuple[int, int]],
                 params: Optional[GovernorParams] = None,
                 p_coeff: Optional[Dict[str, int]] = None,
                 mode: str = "active"):
        self.joints = list(joints)
        self.ranges = dict(ranges)
        self.p = params or GovernorParams()
        self.p_coeff = {n: 16 for n in self.joints}
        if p_coeff:
            self.p_coeff.update({k: int(v) for k, v in p_coeff.items()})
        self.mode = mode                 # "active" | "shadow"
        self._j: Dict[str, _Joint] = {}
        for n in self.joints:
            self._j[n] = _Joint(duty=deque(maxlen=self.p.duty_window),
                                current=deque(maxlen=self.p.duty_window))
        self._protect_log: Deque[float] = deque()
        self.breaker = False
        self.breaker_reason = ""
        self.thermal = "ok"
        self._thermal_stop = False
        self._retreat_requested = False
        self._retreat_done = False
        self._volt_low = False
        self._t_volt = 0.0
        self._tmax = 0.0
        self._last_now: Optional[float] = None
        self.events: List[Tuple[float, str, str]] = []   # (t, joint, what) for logs

    # -- helpers ---------------------------------------------------------------
    def k_duty(self, n: str) -> float:
        return sts3215.k_duty_per_tick(self.p_coeff.get(n, 16))

    def clamp_goal(self, n: str, ticks: float) -> int:
        """Keep the final goal inside the calibrated range, minus a margin."""
        if n in FULL_TURN:
            return int(round(ticks))
        lo, hi = self.ranges.get(n, (0, 4095))
        margin = sts3215.deg_to_ticks(self.p.gripper_margin_deg if n == GRIPPER
                                      else self.p.soft_margin_deg)
        lo2, hi2 = lo + margin, hi - margin
        if lo2 > hi2:                      # degenerate range: stay in the middle
            lo2 = hi2 = (lo + hi) / 2.0
        return int(round(min(hi2, max(lo2, ticks))))

    def gravity_duty(self, states: Dict[str, ServoState]) -> Dict[str, float]:
        """Permille of stall each arm joint needs to hold the measured pose."""
        out = {n: 0.0 for n in self.joints}
        arm = [n for n in profile.JOINT_NAMES]
        if not all(n in states for n in arm):
            return out
        pose = gravity.pose_from_ticks({n: states[n].pos for n in arm}, self.ranges)
        jaw = 0.0
        taus = gravity.holding_torques(pose, jaw_deg=jaw)
        for n, tau in zip(arm, taus):
            if n in out:
                out[n] = abs(tau) / sts3215.STALL_TORQUE_NM * 1000.0
        return out

    def reset(self, states: Dict[str, ServoState]) -> None:
        """Forget holds; every joint starts from where it is (connect / engage)."""
        for n in self.joints:
            j = self._j[n]
            pos = states[n].pos if n in states else None
            j.state = "track" if pos is not None else "stale"
            j.reason = ""
            j.goal = pos
            j.hold_goal = None
            j.prev_goal = None
            j.push_sign = 0
            j.t_block = j.t_grip = j.t_clear = j.t_probe = j.t_load = 0.0
            j.duty.clear()
            j.current.clear()

    def clear_breaker(self) -> None:
        self.breaker = False
        self.breaker_reason = ""
        self._protect_log.clear()

    def clear_thermal_stop(self) -> bool:
        """Allow following again once the hottest joint is back under resume."""
        if self._tmax >= self.p.temp_resume_c:
            return False
        self._thermal_stop = False
        self._retreat_requested = False
        return True

    # -- the cycle -------------------------------------------------------------
    def ack_retreat(self) -> None:
        """The engine finished the cooling move; do not ask again until the arm
        has cooled back to the alert level and heats up once more."""
        self._retreat_requested = False
        self._retreat_done = True

    def step(self, refs: Dict[str, Optional[int]], states: Dict[str, ServoState],
             torque_on: Dict[str, bool], now: float) -> GovernorOutput:
        p = self.p
        dt = 0.0 if self._last_now is None else max(0.0, min(0.2, now - self._last_now))
        self._last_now = now
        u_g = self.gravity_duty(states)
        goals: Dict[str, int] = {}
        report: Dict[str, JointStatus] = {}

        # ---- arm-level: temperature, supply ---------------------------------
        temps = [states[n].temp for n in self.joints if n in states and states[n].ok]
        tmax = max(temps) if temps else 0
        self._tmax = tmax
        self._update_thermal(tmax)
        volts = [states[n].volt for n in self.joints
                 if n in states and states[n].ok and states[n].volt > 1.0]
        vmin = min(volts) if volts else 0.0
        self._update_voltage(vmin, dt)

        decided: Dict[str, Tuple[int, str, str]] = {}
        ctx: Dict[str, tuple] = {}
        for n in self.joints:
            j = self._j[n]
            st = states.get(n)
            ref = refs.get(n)
            js = JointStatus(ref=ref)
            if st is None:
                st = ServoState()
            js.ok = st.ok
            js.temp = st.temp
            js.faults = st.faults
            if st.ok:
                j.duty.append(abs(st.load))
                j.current.append(abs(st.current))
            duty = sum(j.duty) / len(j.duty) if j.duty else 0.0
            cur = sum(j.current) / len(j.current) if j.current else 0.0
            js.duty, js.current, js.gravity = duty, cur, u_g.get(n, 0.0)
            j.ready = j.pushing = False
            decided[n] = self._joint_step(n, j, st, ref, torque_on.get(n, False),
                                          duty, cur, u_g.get(n, 0.0), dt, now)
            ctx[n] = (st, ref, duty)
            report[n] = js

        # ---- ease off together ---------------------------------------------
        # When the arm presses on something, several joints push at once and the
        # contact couples them: easing one alone lets the others shove it back,
        # which would read as a sagging weight. So when any joint is ready to
        # ease, every joint that is also pushing eases in the same cycle.
        if any(self._j[n].ready for n in self.joints):
            for n in self.joints:
                j = self._j[n]
                if n != GRIPPER and (j.ready or j.pushing) and j.state == "track":
                    st, ref, duty = ctx[n]
                    last_goal = j.goal if j.goal is not None else st.pos
                    decided[n] = self._enter_probe(n, j, st, ref, last_goal, duty,
                                                   u_g.get(n, 0.0))

        for n in self.joints:
            j = self._j[n]
            st, ref, _duty = ctx[n]
            js = report[n]
            goal, state, reason = decided[n]
            if self.mode == "shadow":
                # labels only: command exactly what the legacy path would
                if not torque_on.get(n, False) or ref is None:
                    goal = st.pos if st.ok else (j.goal if j.goal is not None else st.pos)
                else:
                    goal = self.clamp_goal(n, ref)
            j.goal = goal
            goals[n] = int(goal)
            js.goal, js.state, js.reason = int(goal), state, reason
            js.fast = state in FAST_STATES and self.mode == "active"

        follow_allowed = not self.breaker and not self._thermal_stop
        stop_reason = self.breaker_reason if self.breaker else (
            "温度が高いため追従を止めています" if self._thermal_stop else "")
        if self.mode == "shadow":
            follow_allowed, stop_reason = True, ""
        msg, level = self._message(report, vmin)
        return GovernorOutput(
            goals=goals, joints=report, follow_allowed=follow_allowed,
            stop_reason=stop_reason,
            retreat=self._retreat_requested and self.mode == "active",
            speed_scale=p.volt_scale if self._volt_low else 1.0,
            thermal=self.thermal, supply_v=round(vmin, 2), message=msg,
            message_level=level)

    # -- per joint --------------------------------------------------------------
    def _joint_step(self, n: str, j: _Joint, st: ServoState, ref: Optional[int],
                    torque: bool, duty: float, cur: float, ug: float, dt: float,
                    now: float) -> Tuple[int, str, str]:
        p = self.p
        last_goal = j.goal if j.goal is not None else st.pos

        if not st.ok and st.age >= p.stale_cycles:
            j.state, j.reason = "stale", "読み取れません"
            return last_goal, j.state, j.reason
        if not torque:
            j.state, j.reason = "off", "脱力中"
            j.hold_goal = None
            j.t_block = j.t_grip = 0.0
            return st.pos, j.state, j.reason
        if not st.ok:
            # a missed read or two: keep doing what we were doing
            return last_goal, j.state, j.reason

        target = st.pos if ref is None else ref
        err = target - st.pos
        still = abs(st.vel) < sts3215.deg_to_ticks(p.still_dps)

        # ---- firmware protection -------------------------------------------
        protect_bits = tuple(f for f in st.faults if f in ("overload", "overcurrent", "overheat"))
        if protect_bits:
            if j.state != "protect":
                j.hold_goal = st.pos
                self._note_protect(n, now, protect_bits)
            j.state = "protect"
            j.reason = f"保護作動中（{sts3215.describe_faults(protect_bits)}）"
            j.t_clear = 0.0
            return int(j.hold_goal), j.state, j.reason
        if j.state == "protect":
            j.t_clear += dt
            if j.t_clear < p.protect_clear_s:
                return int(j.hold_goal), j.state, "保護が解けるのを待っています"
            # restart from where the joint is, not from where it was pushing
            j.state, j.reason, j.hold_goal = "track", "", None
            j.t_block = 0.0

        # ---- gripper: squeeze instead of push -------------------------------
        if n == GRIPPER:
            return self._gripper_step(j, st, ref, target, err, still, duty, last_goal, dt)

        # ---- PROBE: we eased off - does the joint move? ----------------------
        # A P servo pushing on an obstacle and a P servo holding a weight look the
        # same while still (error proportional to load either way). Easing off
        # tells them apart: an obstacle keeps the joint where it is, a weight
        # makes it sag at once. A weight gets its old goal back (no drop).
        if j.state == "probe":
            j.t_probe += dt
            sag = (st.pos - j.probe_pos) * j.push_sign
            # a weight keeps sagging toward the full eased-off amount; a joint
            # nudged by the contact moves a little and stops
            threshold = max(sts3215.deg_to_ticks(p.probe_sag_deg), 0.3 * j.probe_expect)
            if sag < -threshold:
                j.state = "load"
                j.reason = "重い物を支えています（連続で持てる重さを超えています）"
                j.t_load = 0.0
                self.events.append((now, n, "load"))
                return int(j.prev_goal), j.state, j.reason
            if j.t_probe < p.probe_time_s:
                return int(j.hold_goal), j.state, j.reason
            j.state = "hold"
            j.reason = "何かに当たっています。押すのをやめて止めています"
            self.events.append((now, n, "hold"))

        # ---- HOLD: stay put until the reference comes back -------------------
        if j.state == "hold":
            hg = j.hold_goal
            # a slow sag shows up after the probe window: still a weight
            sag = (st.pos - j.probe_pos) * j.push_sign
            if sag < -max(sts3215.deg_to_ticks(p.probe_sag_deg), 0.3 * j.probe_expect):
                j.state = "load"
                j.reason = "重い物を支えています（連続で持てる重さを超えています）"
                j.t_load = 0.0
                self.events.append((now, n, "load"))
                return int(j.prev_goal), j.state, j.reason
            near = abs(target - st.pos) <= sts3215.deg_to_ticks(p.hold_release_deg)
            away = j.push_sign != 0 and (target - st.pos) * j.push_sign < 0
            if near or away:
                j.state, j.reason, j.hold_goal, j.push_sign = "track", "", None, 0
                j.t_block = 0.0
            else:
                return int(hg), j.state, j.reason

        # ---- LOAD: track normally, just labelled ----------------------------
        # No re-probing while the situation is the same (each probe dips the arm
        # a couple of degrees): only a new pose or a lighter load ends it.
        if j.state == "load":
            j.t_load += dt
            moved = abs(target - (j.prev_goal if j.prev_goal is not None else target)) \
                > sts3215.deg_to_ticks(p.load_release_deg)
            lighter = duty < max(ug + p.yield_over_gravity, 0.75 * p.block_duty_min)
            if moved or lighter:
                j.state, j.reason = "track", ""
                j.t_block = 0.0
            else:
                return self.clamp_goal(n, target), j.state, j.reason

        # ---- TRACK, watching for a joint stuck short of its goal --------------
        k = self.k_duty(n)
        droop = ug / k if k > 0 else 0.0
        need_err = max(sts3215.deg_to_ticks(p.block_err_deg),
                       droop + sts3215.deg_to_ticks(p.block_droop_margin_deg))
        blocked = (still and abs(err) > need_err
                   and duty > max(ug + p.block_duty_over_gravity, p.block_duty_min))
        if blocked:
            # a hard push gets there twice as fast: the 80 % / 2 s overload and
            # the 2 A / 2 s over-current trips are what we must beat
            j.t_block += dt * (2.0 if (duty > p.effort_duty or cur > p.effort_current_a) else 1.0)
        else:
            j.t_block = 0.0
        # the looser "also pushing" test decides who eases off together (step())
        j.pushing = (still and abs(err) > 0.5 * need_err
                     and duty > max(ug + p.yield_over_gravity, 0.75 * p.block_duty_min))
        j.ready = j.t_block >= p.block_time_s

        j.state = "track"
        j.reason = "負荷大" if duty > max(ug + 150.0, 350.0) else ""
        return self.clamp_goal(n, target), j.state, j.reason

    def _enter_probe(self, n: str, j: _Joint, st: ServoState, ref: Optional[int],
                     last_goal: int, duty: float, ug: float) -> Tuple[int, str, str]:
        """Ease off proportionally: duty ~ k * error, so scaling the error the
        servo sees by r scales its output by r - whatever k really is."""
        p = self.p
        target = st.pos if ref is None else ref
        e_s = last_goal - st.pos
        budget = max(ug + p.yield_over_gravity, p.yield_min_duty)
        r = min(1.0, budget / duty) if duty > 0 else 1.0
        j.prev_goal = last_goal
        j.hold_goal = int(round(st.pos + e_s * r))
        j.probe_pos = st.pos
        j.probe_expect = abs(e_s) * (1.0 - r)
        j.push_sign = 1 if (target - st.pos) > 0 else -1
        j.t_probe = 0.0
        j.t_block = 0.0
        j.ready = False
        j.state = "probe"
        j.reason = "当たっているか確かめています"
        return int(j.hold_goal), j.state, j.reason

    def _gripper_step(self, j: _Joint, st: ServoState, ref: Optional[int], target: int,
                      err: int, still: bool, duty: float, last_goal: int,
                      dt: float) -> Tuple[int, str, str]:
        p = self.p
        n = GRIPPER
        if j.state == "grip":
            # let go as soon as the operator opens (moves against the squeeze)
            if (target - st.pos) * j.push_sign < 0:
                j.state, j.reason, j.hold_goal, j.push_sign = "track", "", None, 0
            else:
                return int(j.hold_goal), "grip", "把持中"
        closing_on_something = (still and abs(err) > sts3215.deg_to_ticks(2.0)
                                and duty > p.grip_detect_duty)
        j.t_grip = j.t_grip + dt if closing_on_something else 0.0
        if j.t_grip >= p.grip_detect_s:
            # keep squeezing with grip_duty: scale the error the servo sees
            e_s = last_goal - st.pos
            r = min(1.0, p.grip_duty / duty) if duty > 0 else 1.0
            j.state, j.reason = "grip", "把持中"
            j.hold_goal = self.clamp_goal(n, st.pos + e_s * r)
            j.push_sign = 1 if err > 0 else -1
            j.t_grip = 0.0
            return int(j.hold_goal), j.state, j.reason
        j.state, j.reason = "track", ""
        return self.clamp_goal(n, target), j.state, j.reason

    # -- arm level ---------------------------------------------------------------
    def _note_protect(self, n: str, now: float, bits) -> None:
        self.events.append((now, n, "protect:" + ",".join(bits)))
        self._protect_log.append(now)
        while self._protect_log and now - self._protect_log[0] > self.p.breaker_window_s:
            self._protect_log.popleft()
        if len(self._protect_log) >= self.p.breaker_count and not self.breaker:
            self.breaker = True
            self.breaker_reason = (f"保護が {int(self.p.breaker_window_s // 60)} 分間に "
                                   f"{len(self._protect_log)} 回作動したので追従を止めました。"
                                   "押し付け・荷物・高温を取り除いてから追従を入れ直してください")

    _THERMAL_ORDER = ("ok", "warn", "alert", "stop", "retreat")

    def _update_thermal(self, tmax: float) -> None:
        p = self.p
        bounds = {"warn": p.temp_warn_c, "alert": p.temp_alert_c,
                  "stop": p.temp_stop_c, "retreat": p.temp_retreat_c}
        level = "ok"
        for name in ("warn", "alert", "stop", "retreat"):
            if tmax >= bounds[name]:
                level = name
        # rise at once, fall only 2 C below the level's threshold (the sensor
        # reads whole degrees and would otherwise flicker between two levels)
        cur = self.thermal
        order = self._THERMAL_ORDER
        if order.index(level) < order.index(cur):
            while cur != "ok" and tmax < bounds[cur] - p.temp_hysteresis_c \
                    and order.index(cur) > order.index(level):
                cur = order[order.index(cur) - 1]
            level = cur
        self.thermal = level
        if level in ("stop", "retreat"):
            self._thermal_stop = True
        if level == "retreat" and not self._retreat_done:
            self._retreat_requested = True
        if order.index(level) <= order.index("alert"):
            self._retreat_done = False
        if self._thermal_stop and tmax < p.temp_resume_c and level not in ("stop", "retreat"):
            # cooled down: the stop stays until the operator re-engages, but a
            # pending retreat that never ran is no longer needed
            self._retreat_requested = False

    def _update_voltage(self, vmin: float, dt: float) -> None:
        p = self.p
        if vmin <= 1.0:
            return
        if not self._volt_low:
            self._t_volt = self._t_volt + dt if vmin < p.volt_low_v else 0.0
            if self._t_volt >= p.volt_low_s:
                self._volt_low, self._t_volt = True, 0.0
        else:
            self._t_volt = self._t_volt + dt if vmin > p.volt_ok_v else 0.0
            if self._t_volt >= p.volt_ok_s:
                self._volt_low, self._t_volt = False, 0.0

    def retreat_pose_ticks(self, states: Dict[str, ServoState]) -> Dict[str, int]:
        """Cooling pose in ticks: stand the arm up, keep pan / roll / gripper."""
        out = {}
        for n in self.joints:
            if n in states:
                out[n] = states[n].pos
        def deg_to(n: str, deg: float) -> int:
            lo, hi = self.ranges.get(n, (0, 4095))
            return self.clamp_goal(n, (lo + hi) / 2.0 + deg * (sts3215.TICKS_PER_REV - 1) / 360.0)
        out["shoulder_lift"] = deg_to("shoulder_lift", COOL_LIFT_DEG)
        out["elbow_flex"] = deg_to("elbow_flex", COOL_ELBOW_DEG)
        out["wrist_flex"] = deg_to("wrist_flex", COOL_WRIST_DEG)
        return out

    def _message(self, report: Dict[str, JointStatus], vmin: float) -> Tuple[str, str]:
        ja = {"shoulder_pan": "土台", "shoulder_lift": "肩", "elbow_flex": "肘",
              "wrist_flex": "手首", "wrist_roll": "手首の回転", "gripper": "グリッパ"}
        if self.breaker:
            return self.breaker_reason, "danger"
        for n, js in report.items():
            if js.state == "protect":
                return f"{ja.get(n, n)}: {js.reason}", "danger"
        if self.thermal == "retreat":
            return "温度が限界に近いので、腕を立てて冷ましています", "danger"
        if self.thermal == "stop":
            return "温度が高いので追従を止めました。腕を支えて冷ましてください", "danger"
        for n, js in report.items():
            if js.state == "hold":
                return f"{ja.get(n, n)}: {js.reason}。リーダーを戻すと追従に戻ります", "warn"
        for n, js in report.items():
            if js.state == "load":
                return f"{ja.get(n, n)}: {js.reason}。長く続けると熱くなります", "warn"
        if self.thermal == "alert":
            return "サーボが熱くなっています。腕を下ろすか、冷却姿勢ボタンを押してください", "warn"
        if self._volt_low:
            return f"電源の電圧が下がっています（{vmin:.1f}V）。速度を落としています", "warn"
        if self.thermal == "warn":
            return "サーボが温まってきました。伸ばした姿勢で長く止めないでください", "info"
        for n, js in report.items():
            if js.state == "stale":
                return f"{ja.get(n, n)} の値が読めていません", "warn"
        return "", "info"
