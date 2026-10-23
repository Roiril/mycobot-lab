"""SO-101 follower engine: leader->follower teleop, holding, jogging, cooling.

Shared by the cockpit (scripts/so101_cockpit_server.py) and the CLI
(scripts/so101_teleop.py) so both run one motion pipeline.

One control cycle (`cycle()`)
-----------------------------
1. read the leader's positions (one sync read)
2. read the follower's whole telemetry window (one sync read: position, speed,
   load, voltage, temperature, status, current of all six servos)
3. build the reference per joint: the One-Euro-filtered leader while following,
   the hold / jog target otherwise, a slow straight line while retreating
4. FollowerGovernor decides the goal per joint from reference + telemetry
   (governor.py: end-stop margin, stuck / overworked -> hold, protection,
   breaker, temperature, supply sag)
5. write what changed - goal, speed, acceleration, torque limit - in ONE sync
   write of registers 41..49

Why writes only on change: a new position command clears the servo's overload
and over-current flags. Streaming the same goal every cycle would keep
resetting the firmware's own protections, so a still arm sends nothing.

All follower and leader traffic goes through servo_io (status bits are data,
never exceptions; a silent servo costs only itself). lerobot's bus is used to
open the port and for calibration only.

Pure vs I/O
-----------
OneEuroFilter, AutoFollowGate, compute_refs and the governor are pure and run
on plain Python 3.10. lerobot is imported lazily (make_bus) so this module stays
importable without it.
"""
from __future__ import annotations

import json
import math
import pathlib
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from . import profile, servo_config, sts3215
from .governor import FollowerGovernor, GovernorOutput, GovernorParams
from .servo_io import Command, wrap_bus
from .sts3215 import ServoState

# --- Units --------------------------------------------------------------------
TICKS_PER_REV = profile.HARDWARE["encoder_ticks"]          # 4096
TICKS_PER_DEG = TICKS_PER_REV / 360.0                       # ~11.377

# Joints mirrored, in Feetech motor order 1..6 (5-DoF arm + gripper).
JOINTS: List[str] = list(profile.JOINT_NAMES) + [profile.GRIPPER_NAME]

MAX_STEP_TICKS = 170    # per-cycle reference jump guard (~15 deg): glitch filter only

# --- Cooling relax (torque-off for heat shedding) -----------------------------
# Tip -> base (gripper first, shoulder_pan last): the gravity-loaded roots hold
# longest, so the arm settles instead of slamming down. Done with sync writes,
# which have no reply and so cannot be stopped by a servo in protection.
RELAX_ORDER: List[str] = list(reversed(JOINTS))
RELAX_STAGGER_S = 0.05

# --- Auto-follow gate (arm follow mode without a human clicking) --------------
AUTO_FOLLOW_TOL_DEG = 20.0     # per-joint agreement required before engaging
AUTO_FOLLOW_STILL_DEG = 3.0    # leader must be roughly parked, not swinging through
AUTO_FOLLOW_HOLD_CYCLES = 5    # consecutive good cycles (rejects a momentary crossing)

RETREAT_DPS = 12.0             # speed of the automatic move to the cooling pose
# Profile bypass for a joint the governor is easing / holding: an eased goal
# must reach the servo's setpoint now, not at the cruise speed.
FAST_VELOCITY = 3000           # step/s (~260 deg/s, the 12 V no-load speed)
FAST_ACCELERATION = 254        # x100 step/s^2, the register maximum
SOFT_START_S = 1.5             # follow engage: speed ramps from 25 % to 100 %
SOFT_START_FLOOR = 0.25


def deg_to_ticks(deg: float) -> int:
    return int(round(deg * TICKS_PER_DEG))


def pose_gaps(leader_ticks: Dict[str, int], follower_ticks: Dict[str, int],
              joints: Optional[List[str]] = None) -> Dict[str, int]:
    """Per-joint |leader - follower| in ticks (pure).

    Both arms report in the same homed tick domain (calibration writes
    Homing_Offset into each servo's EEPROM), which is what lets the engine feed
    leader ticks straight into follower goals - so the difference is meaningful.
    """
    names = joints or JOINTS
    return {n: abs(int(leader_ticks[n]) - int(follower_ticks[n]))
            for n in names if n in leader_ticks and n in follower_ticks
            and leader_ticks[n] is not None and follower_ticks[n] is not None}


def max_pose_gap(leader_ticks: Dict[str, int], follower_ticks: Dict[str, int],
                 joints: Optional[List[str]] = None) -> Tuple[Optional[str], int]:
    """Worst-disagreeing joint and its gap in ticks (pure). (None, 0) if empty."""
    gaps = pose_gaps(leader_ticks, follower_ticks, joints)
    if not gaps:
        return None, 0
    joint = max(gaps, key=lambda n: gaps[n])
    return joint, gaps[joint]


class AutoFollowGate:
    """Decides when it is safe to engage follow mode unattended (pure).

    `update` is called once per idle cycle with both arms' present positions and
    returns True exactly when the arms have agreed, with the leader held still,
    for `hold_cycles` consecutive calls. `status` carries a Japanese reason for
    the UI so the operator can see what is still blocking it.
    """

    def __init__(self, tol_ticks: Optional[int] = None,
                 still_ticks: Optional[int] = None,
                 hold_cycles: int = AUTO_FOLLOW_HOLD_CYCLES,
                 joints: Optional[List[str]] = None):
        self.tol_ticks = (deg_to_ticks(AUTO_FOLLOW_TOL_DEG)
                          if tol_ticks is None else int(tol_ticks))
        self.still_ticks = (deg_to_ticks(AUTO_FOLLOW_STILL_DEG)
                            if still_ticks is None else int(still_ticks))
        self.hold_cycles = max(1, int(hold_cycles))
        self.joints = joints or list(JOINTS)
        self.reset()

    def reset(self) -> None:
        self._prev_leader: Optional[Dict[str, int]] = None
        self.held = 0
        self.gap_ticks = 0
        self.gap_joint: Optional[str] = None
        self.status = "待機中"

    def update(self, leader_ticks: Dict[str, int],
               follower_ticks: Dict[str, int]) -> bool:
        joint, gap = max_pose_gap(leader_ticks, follower_ticks, self.joints)
        self.gap_joint, self.gap_ticks = joint, gap
        if joint is None:
            self.held = 0
            self.status = "姿勢を取得できません"
            return False

        moved = 0
        if self._prev_leader is not None:
            _, moved = max_pose_gap(leader_ticks, self._prev_leader, self.joints)
        self._prev_leader = dict(leader_ticks)

        gap_deg = gap / TICKS_PER_DEG
        if gap > self.tol_ticks:
            self.held = 0
            self.status = (f"リーダーをフォロワーの姿勢に合わせてください"
                           f"（最大ずれ {gap_deg:.1f}° / {joint}）")
            return False
        if moved > self.still_ticks:
            self.held = 0
            self.status = "リーダーが動いています（静止すると開始）"
            return False

        self.held += 1
        if self.held < self.hold_cycles:
            self.status = f"姿勢一致 — 開始待ち {self.held}/{self.hold_cycles}"
            return False
        self.status = f"姿勢一致（最大ずれ {gap_deg:.1f}°）"
        return True


CAL_DIR = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"


def make_bus(port: str):
    """Build a 6-motor Feetech bus for `port` (lazy lerobot import).

    Only used to open the serial port (and by the calibration tools): once
    connected, the engine talks through servo_io.
    """
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    motors = {n: Motor(i + 1, "sts3215", MotorNormMode.DEGREES)
              for i, n in enumerate(JOINTS)}
    return FeetechMotorsBus(port, motors)


def load_ranges(log: Optional[Callable[[str], None]] = None) -> Dict[str, Tuple[int, int]]:
    """Follower calibration ranges (offset-applied tick domain).

    Falls back to the full 0..4095 range if the calibration file is missing.
    """
    fp = CAL_DIR / "robots/so_follower/so101_follower.json"
    try:
        cal = json.loads(fp.read_text())
        return {n: (int(c["range_min"]), int(c["range_max"])) for n, c in cal.items()}
    except Exception as e:  # pragma: no cover - depends on host calibration
        if log:
            log(f"calibration load failed ({e}) - using full range")
        return {n: (0, 4095) for n in JOINTS}


# ==============================================================================
# One-Euro filter (pure)
# ==============================================================================
def _smoothing_alpha(cutoff: float, dt: float) -> float:
    """Exponential-smoothing factor for a 1st-order low-pass at `cutoff` (Hz)."""
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


@dataclass
class OneEuroConfig:
    """One-Euro parameters, in RAW TICK units (11.377 ticks/deg).

    Derivation (target loop 60 Hz; manual leader up to ~90 deg/s ~= 1000 tick/s):

    A 1st-order low-pass at cutoff fc lags a constant-velocity input by
    ~1/(2*pi*fc) seconds. One-Euro raises the cutoff with speed:
        cutoff = min_cutoff + beta * |filtered_speed|   (speed in tick/s)

    - min_cutoff = 1.0 Hz: the resting cutoff. At rest the leader speed ~0 so
      the cutoff stays here, giving strong jitter rejection (a couple of ticks
      of Feetech encoder noise vanish) with negligible lag (velocity ~0).
    - beta = 0.01: at a brisk 1000 tick/s (~90 deg/s) move the cutoff rises to
      1.0 + 0.01*1000 = 11 Hz, i.e. ~14 ms (~1.6 tick, ~0.14 deg) of lag —
      imperceptible while tracking, yet the resting smoothing is preserved.
    - d_cutoff = 1.0 Hz: cutoff of the speed estimator itself (Casiez default);
      keeps the speed signal from chattering and re-injecting jitter.
    """
    min_cutoff: float = 1.0
    beta: float = 0.01
    d_cutoff: float = 1.0


class OneEuroFilter:
    """Scalar One-Euro filter (Casiez, Roussel, Vogel 2012). Units-agnostic."""

    def __init__(self, cfg: OneEuroConfig, nominal_dt: float):
        self.cfg = cfg
        self._nominal_dt = nominal_dt      # dt fallback for the first / bad sample
        self.reset()

    def reset(self) -> None:
        self._x_prev: Optional[float] = None
        self._dx_prev: float = 0.0
        self._t_prev: Optional[float] = None

    def filter(self, x: float, t: float) -> float:
        if self._x_prev is None:
            self._x_prev = x
            self._t_prev = t
            self._dx_prev = 0.0
            return x

        dt = t - self._t_prev
        if not (dt > 0.0):
            dt = self._nominal_dt
        self._t_prev = t

        # filtered derivative (speed)
        dx = (x - self._x_prev) / dt
        a_d = _smoothing_alpha(self.cfg.d_cutoff, dt)
        edx = a_d * dx + (1.0 - a_d) * self._dx_prev
        self._dx_prev = edx

        # speed-adaptive cutoff, then low-pass the value
        cutoff = self.cfg.min_cutoff + self.cfg.beta * abs(edx)
        a = _smoothing_alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        return x_hat


# ==============================================================================
# Engine
# ==============================================================================
@dataclass
class TeleopConfig:
    target_hz: float = 60.0
    max_step_ticks: int = MAX_STEP_TICKS
    goal_deadband_ticks: int = 1       # a goal that moved less is not re-sent
    torque_stagger_s: float = 0.05     # delay between per-joint torque enables
    speed: str = "normal"              # servo_config.SPEED_PRESETS key
    governor_mode: str = "active"      # "active" | "shadow" (labels only)
    one_euro: OneEuroConfig = field(default_factory=OneEuroConfig)
    governor: GovernorParams = field(default_factory=GovernorParams)
    session: servo_config.SessionSettings = field(default_factory=servo_config.SessionSettings)

    @property
    def period(self) -> float:
        return 1.0 / self.target_hz


class TeleopEngine:
    """Owns the follower: references, governor, torque, and the write path.

    Bus objects are injected (any lerobot-like bus, a raw FeetechMotorsBus, or
    a ServoIO such as the simulated follower) and can be re-bound after a
    reconnect. The engine never opens or closes a port itself.
    """

    def __init__(self, leader_bus, follower_bus,
                 ranges: Dict[str, Tuple[int, int]],
                 config: Optional[TeleopConfig] = None,
                 joints: Optional[List[str]] = None,
                 log: Optional[Callable[[str], None]] = None,
                 sleep: Callable[[float], None] = time.sleep):
        self.cfg = config or TeleopConfig()
        self.joints = joints or list(JOINTS)
        self.ranges = ranges
        self._log = log or (lambda _m: None)
        self._sleep = sleep
        self.leader = wrap_bus(leader_bus, self.joints) if leader_bus is not None else None
        self.follower = wrap_bus(follower_bus, self.joints)
        self.governor = FollowerGovernor(self.joints, ranges, self.cfg.governor,
                                         mode=self.cfg.governor_mode)
        self.filters: Dict[str, OneEuroFilter] = {
            n: OneEuroFilter(self.cfg.one_euro, self.cfg.period) for n in self.joints}
        self.active = False                # following the leader
        self.retreating = False
        self.torque_on: Dict[str, bool] = {n: False for n in self.joints}
        self.hold_target: Dict[str, Optional[int]] = {n: None for n in self.joints}
        self.last_ref: Dict[str, Optional[int]] = {n: None for n in self.joints}
        self.speed = self.cfg.speed if self.cfg.speed in servo_config.SPEED_PRESETS else "normal"
        self._engaged_at: Optional[float] = None
        self._retreat: Optional[dict] = None
        self._sent: Dict[str, Command] = {}
        self.states: Dict[str, ServoState] = {}
        self.leader_pos: Dict[str, Optional[int]] = {}
        self.last_out: Optional[GovernorOutput] = None
        self.findings: List[servo_config.Finding] = []
        self.p_effective: Dict[str, int] = {}
        self.instrument_ok: Optional[bool] = None
        self.instrument_detail = ""
        # metrics
        self._cycle_ms: Deque[float] = deque(maxlen=300)
        self._last_cycle_t: Optional[float] = None
        self._hz = 0.0

    # -- bus lifecycle ---------------------------------------------------------
    def rebind(self, leader_bus, follower_bus) -> None:
        """Point the engine at freshly reconnected buses (following must be off)."""
        self.leader = wrap_bus(leader_bus, self.joints) if leader_bus is not None else None
        self.follower = wrap_bus(follower_bus, self.joints)
        self._sent.clear()

    def sync_torque_state(self) -> Dict[str, bool]:
        """Learn which follower joints are already energized (e.g. the cockpit
        restarted while the arm was holding) and hold them where they are."""
        st = self.follower.read_telemetry()
        for n in self.joints:
            v, _err = self.follower.read_reg("Torque_Enable", n)
            on = bool(v) if v is not None else False
            self.torque_on[n] = on
            if on and n in st and st[n].ok:
                self.hold_target[n] = st[n].pos
        self._sent.clear()
        return dict(self.torque_on)

    def configure_follower(self) -> Dict[str, int]:
        """Session settings + audit + instrument check. Call with torque off.

        P/D/I, acceleration, speed and torque limits are written with the EEPROM
        lock closed, so they last for this power cycle only. The P actually in
        effect is read back and handed to the governor (its gain is P/4).
        """
        io = self.follower
        table = servo_config.audit(io.read_reg, self.joints)
        st = io.read_telemetry()
        supply = max([s.volt for s in st.values() if s.ok] or [12.2])
        self.findings = servo_config.findings(table, supply_v=supply)
        for f in self.findings:
            if f.level != "info":
                self._log(f"⚠ 設定: {f.servo} {f.message}")
        self.p_effective = servo_config.apply_session(
            io.write_reg, io.read_reg, self.cfg.session, self.joints,
            goal_velocity=self.goal_velocity(), log=self._log)
        self.governor.p_coeff.update(self.p_effective)
        bad = {n: p for n, p in self.p_effective.items() if p != self.cfg.session.p}
        if bad:
            self._log(f"⚠ P を {self.cfg.session.p} にできなかった関節: {bad}")
        self.instrument_check()
        return self.p_effective

    def instrument_check(self) -> bool:
        """Does the one-shot telemetry read agree with single-register reads?

        If not, the governor must not act on it: it falls back to shadow mode
        (labels only) and the cockpit says so.
        """
        io = self.follower
        st = io.read_telemetry()
        problems = []
        for n in self.joints:
            s = st.get(n)
            if s is None or not s.ok:
                problems.append(f"{n}: 一括読み出しに応答なし")
                continue
            single, _err = io.read_reg("Present_Position", n)
            if single is not None and abs(int(single) - s.pos) > 8:
                problems.append(f"{n}: 位置が一致しません（{s.pos} / {single}）")
            if not (6.0 <= s.volt <= 14.5):
                problems.append(f"{n}: 電圧 {s.volt:.1f}V が範囲外")
            if not (0 <= s.temp <= 90):
                problems.append(f"{n}: 温度 {s.temp}℃ が範囲外")
        self.instrument_ok = not problems
        self.instrument_detail = "; ".join(problems)
        if problems and self.cfg.governor_mode == "active":
            self.governor.mode = "shadow"
            self._log("⚠ 計器チェック不合格のため、力の自動調整は表示だけにします: "
                      + self.instrument_detail)
        elif not problems:
            self.governor.mode = self.cfg.governor_mode
        return self.instrument_ok

    # -- speed -------------------------------------------------------------------
    def goal_velocity(self) -> int:
        dps = servo_config.SPEED_PRESETS[self.speed]["dps"]
        return servo_config.dps_to_goal_velocity(dps)

    def set_speed(self, preset: str) -> None:
        if preset not in servo_config.SPEED_PRESETS:
            raise ValueError(f"unknown speed preset: {preset!r}")
        self.speed = preset

    def _velocity_scale(self, now: float) -> float:
        scale = 1.0
        if self._engaged_at is not None:
            r = (now - self._engaged_at) / SOFT_START_S
            if r < 1.0:
                scale = SOFT_START_FLOOR + (1.0 - SOFT_START_FLOOR) * max(0.0, r)
        if self.last_out is not None:
            scale *= self.last_out.speed_scale
        return scale

    # -- torque ------------------------------------------------------------------
    def _hold_here(self, joints: List[str]) -> None:
        """Make each joint's goal its present position (so enabling never jumps)."""
        st = self.follower.read_telemetry()
        cmds = {}
        for n in joints:
            s = st.get(n)
            if s is not None and s.ok:
                cmds[n] = self._command(n, s.pos, 1.0)
                self.hold_target[n] = s.pos
        self.follower.write_commands(cmds)
        self._sent.update(cmds)

    def set_follower_torque(self, joint: str, on: bool) -> None:
        if on and not self.torque_on.get(joint):
            self._hold_here([joint])
        self.follower.sync_write("Torque_Enable", {joint: 1 if on else 0})
        self.torque_on[joint] = bool(on)

    def enable_follower_torque_staged(self) -> None:
        """Torque-enable one joint at a time (inrush control)."""
        self._hold_here(self.joints)
        for n in self.joints:
            self.follower.sync_write("Torque_Enable", {n: 1})
            self.torque_on[n] = True
            if self.cfg.torque_stagger_s:
                self._sleep(self.cfg.torque_stagger_s)

    def relax_order(self) -> List[str]:
        return [n for n in RELAX_ORDER if n in self.joints]

    def relax_follower(self, delay: float = RELAX_STAGGER_S,
                       sleep: Optional[Callable[[float], None]] = None) -> List[str]:
        """Cooling relax: cut all follower torque, tip -> base.

        Sync writes only - a joint in protection cannot make this stop half way.
        The follower goes limp: the caller must have warned the operator.
        """
        sleep = sleep or self._sleep
        self.active = False
        self.retreating = False
        self._retreat = None
        order = self.relax_order()
        for i, n in enumerate(order):
            self.follower.sync_write("Torque_Enable", {n: 0})
            self.torque_on[n] = False
            if delay and i < len(order) - 1:
                sleep(delay)
        return order

    # -- modes -------------------------------------------------------------------
    def start_teleop(self, now: Optional[float] = None) -> Tuple[bool, str]:
        """Begin following. Refused while the breaker or a thermal stop holds."""
        now = time.perf_counter() if now is None else now
        self.governor.clear_breaker()               # the operator asked explicitly
        if not self.governor.clear_thermal_stop():
            return False, "温度が高いので追従を始められません。冷めるまで待ってください"
        st = self.follower.read_telemetry()
        self.states = st
        self.governor.reset(st)
        for f in self.filters.values():
            f.reset()
        self.last_ref = {n: (st[n].pos if n in st and st[n].ok else None) for n in self.joints}
        self.retreating = False
        self._retreat = None
        if not all(self.torque_on.values()):
            self.enable_follower_torque_staged()
        self._engaged_at = now
        self.active = True
        return True, ""

    def stop_teleop(self) -> None:
        """Stop following and hold where the follower is now."""
        self.active = False
        self.retreating = False
        self._retreat = None
        self._engaged_at = None
        for n in self.joints:
            s = self.states.get(n)
            self.hold_target[n] = s.pos if (s is not None and s.ok) else self.hold_target[n]

    def jog(self, joint: str, ticks: int) -> None:
        """Move one joint while not following (the governor still applies)."""
        if self.active:
            return
        if not self.torque_on.get(joint):
            self.set_follower_torque(joint, True)
        self.hold_target[joint] = int(ticks)

    def start_retreat(self) -> None:
        """Slow straight-line move to the gravity-neutral cooling pose."""
        st = self.follower.read_telemetry()
        target = self.governor.retreat_pose_ticks(st)
        start = {n: st[n].pos for n in self.joints if n in st}
        span = max((abs(target[n] - start[n]) for n in start), default=0)
        dur = max(0.5, span / (RETREAT_DPS * TICKS_PER_DEG))
        self._retreat = {"start": start, "target": target, "t0": None, "dur": dur}
        self.active = False
        self.retreating = True
        if not all(self.torque_on.values()):
            self.enable_follower_torque_staged()
        self._log(f"冷却姿勢へ移動します（約 {dur:.0f} 秒）")

    # -- references ----------------------------------------------------------------
    def compute_refs(self, leader_ticks: Dict[str, Optional[int]], now: float
                     ) -> Dict[str, Optional[int]]:
        """PURE: filter the leader, then limit the per-cycle jump (glitch guard).

        The previous reference - not the follower's present position - is the
        base for the jump limit, so a follower that lags (or is held) is never
        handed a full jump to catch up in one cycle.
        """
        refs: Dict[str, Optional[int]] = {}
        for n in self.joints:
            raw = leader_ticks.get(n)
            if raw is None:
                refs[n] = self.last_ref.get(n)
                continue
            filt = self.filters[n].filter(float(raw), now)
            last = self.last_ref.get(n)
            if last is None:
                ref = filt
            else:
                step = max(-self.cfg.max_step_ticks, min(self.cfg.max_step_ticks, filt - last))
                ref = last + step
            refs[n] = int(round(ref))
            self.last_ref[n] = refs[n]
        return refs

    def _retreat_refs(self, now: float) -> Tuple[Dict[str, Optional[int]], bool]:
        r = self._retreat
        if r["t0"] is None:
            r["t0"] = now
        s = min(1.0, (now - r["t0"]) / r["dur"])
        s = s * s * (3.0 - 2.0 * s)                  # smooth start and stop
        refs = {n: int(round(r["start"][n] + (r["target"][n] - r["start"][n]) * s))
                for n in r["start"]}
        return refs, s >= 1.0

    # -- the cycle -------------------------------------------------------------------
    def _command(self, n: str, goal: int, vscale: float, fast: bool = False) -> Command:
        ss = self.cfg.session
        tl = ss.torque_limit_gripper if n == profile.GRIPPER_NAME else ss.torque_limit_arm
        if fast:
            return Command(goal=int(goal), velocity=FAST_VELOCITY,
                           acceleration=FAST_ACCELERATION, torque_limit=tl)
        vel = max(20, int(round(self.goal_velocity() * vscale)))
        if self.retreating:
            vel = max(20, int(round(RETREAT_DPS * TICKS_PER_DEG * 1.5)))
        return Command(goal=int(goal), velocity=vel, acceleration=ss.acceleration,
                       torque_limit=tl)

    def _changed(self, n: str, c: Command) -> bool:
        prev = self._sent.get(n)
        if prev is None:
            return True
        return (abs(prev.goal - c.goal) >= self.cfg.goal_deadband_ticks
                or prev.velocity != c.velocity or prev.acceleration != c.acceleration
                or prev.torque_limit != c.torque_limit)

    def cycle(self, now: Optional[float] = None) -> dict:
        """One control cycle. Returns what the cockpit displays."""
        now = time.perf_counter() if now is None else now
        t0 = time.perf_counter()
        if self.leader is not None:
            self.leader_pos = self.leader.read_positions(self.joints)
        st = self.follower.read_telemetry(self.joints)
        self.states = st

        if self.active:
            refs = self.compute_refs(self.leader_pos, now)
        elif self.retreating and self._retreat is not None:
            refs, done = self._retreat_refs(now)
            if done:
                self.retreating = False
                self._retreat = None
                for n, v in refs.items():
                    self.hold_target[n] = v
                self.governor.ack_retreat()
                self._log("冷却姿勢に着きました。冷めるまでこの姿勢で保持します")
        else:
            refs = {}
            for n in self.joints:
                if self.hold_target.get(n) is None and n in st and st[n].ok:
                    self.hold_target[n] = st[n].pos
                refs[n] = self.hold_target.get(n)

        out = self.governor.step(refs, st, self.torque_on, now)
        self.last_out = out

        if self.active and not out.follow_allowed:
            self._log(f"追従を止めました: {out.stop_reason}")
            self.stop_teleop()
            for n, g in out.goals.items():
                self.hold_target[n] = g
        if out.retreat and not self.retreating:
            self.start_retreat()
        if not self.active and not self.retreating:
            # a frozen joint stays where the governor froze it once we stop following
            for n, js in out.joints.items():
                if js.state in ("hold", "protect"):
                    self.hold_target[n] = js.goal

        vscale = self._velocity_scale(now)
        cmds = {}
        for n, goal in out.goals.items():
            js = out.joints.get(n)
            c = self._command(n, goal, vscale, fast=bool(js is not None and js.fast))
            if self._changed(n, c):
                cmds[n] = c
        if cmds:
            self.follower.write_commands(cmds)
            self._sent.update(cmds)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._cycle_ms.append(dt_ms)
        if self._last_cycle_t is not None and now > self._last_cycle_t:
            inst = 1.0 / (now - self._last_cycle_t)
            self._hz = 0.9 * self._hz + 0.1 * inst if self._hz else inst
        self._last_cycle_t = now
        return {"leader": self.leader_pos, "follower": st, "governor": out,
                "written": sorted(cmds)}

    # -- introspection -----------------------------------------------------------------
    @property
    def measured_hz(self) -> float:
        return round(self._hz, 1)

    def metrics(self) -> dict:
        ms = sorted(self._cycle_ms)
        p50 = ms[len(ms) // 2] if ms else 0.0
        p99 = ms[min(len(ms) - 1, int(len(ms) * 0.99))] if ms else 0.0
        io_stats = getattr(self.follower, "stats", {}) or {}
        return {
            "measured_hz": self.measured_hz,
            "target_hz": self.cfg.target_hz,
            "cycle_ms_p50": round(p50, 2),
            "cycle_ms_p99": round(p99, 2),
            "io_timeouts": io_stats.get("timeouts", 0),
            "io_missing": dict(io_stats.get("missing", {})),
            "speed": self.speed,
            "governor_mode": self.governor.mode,
            "instrument_ok": self.instrument_ok,
            "instrument_detail": self.instrument_detail,
            "p_effective": dict(self.p_effective),
            "one_euro": {"min_cutoff": self.cfg.one_euro.min_cutoff,
                         "beta": self.cfg.one_euro.beta,
                         "d_cutoff": self.cfg.one_euro.d_cutoff},
        }
