"""SO-101 leader->follower teleop engine (shared by CLI + cockpit).

Single source of truth for the smoothing + safety pipeline so the CLI
(scripts/so101_teleop.py) and the cockpit server (scripts/so101_cockpit_server.py)
run identical motion logic instead of two drifting copies.

Design goals
------------
- **Smooth, low-latency follow**: a per-joint One-Euro filter (Casiez et al.,
  CHI 2012) on the raw leader ticks removes encoder jitter at rest while adding
  almost no lag during a fast hand move.
- **last-goal stepping**: the follower's Present_Position is NOT read every
  cycle. The per-cycle step clamp is measured against `last_goal` (what we last
  commanded), which halves the serial I/O on the hot path. The follower present
  is sampled only every `follower_read_every` cycles, for a divergence/lag
  telemetry warning — it is deliberately NOT fed back into `last_goal`, because
  rewinding the goal to a lagging follower would license a full MAX_STEP jump
  next cycle (jerk).
- **brownout-safe torque**: per-joint Torque_Limit / Acceleration / Goal_Velocity
  are written BEFORE any torque enable, and torque is enabled one joint at a
  time (staged inrush) — the 12V supply sags and drops the Feetech bus if every
  servo torque-enables at once.

Pure vs I/O
-----------
The smoothing math (`OneEuroFilter`) and the decision logic
(`TeleopEngine.compute_goals`) are pure and import nothing from lerobot, so they
run and unit-test on plain Python 3.10 with no hardware. Everything that touches
a serial bus is isolated in the I/O methods. lerobot / scservo imports (only in
the CLI/cockpit callers) stay lazy.

Bus contract (duck-typed, satisfied by lerobot.motors.feetech.FeetechMotorsBus)
-------------------------------------------------------------------------------
- ``sync_read(data_name, *, normalize=False, num_retry=1) -> dict[str, int]``
- ``sync_write(data_name, values: dict[str, int], *, normalize=False) -> None``
- ``read(data_name, motor, *, normalize=False) -> int``
- ``write(data_name, motor, value, *, normalize=False) -> None``
"""
from __future__ import annotations

import json
import math
import time
import pathlib
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from . import profile

# --- Units --------------------------------------------------------------------
# STS3215: 12-bit absolute encoder, 4096 ticks / rev -> 11.377 ticks/deg.
TICKS_PER_REV = profile.HARDWARE["encoder_ticks"]          # 4096
TICKS_PER_DEG = TICKS_PER_REV / 360.0                       # ~11.377

# Joints mirrored, in Feetech motor order 1..6 (5-DoF arm + gripper).
JOINTS: List[str] = list(profile.JOINT_NAMES) + [profile.GRIPPER_NAME]

# --- Safety / drive constants (SINGLE definition; previously duplicated in
#     so101_teleop.py and so101_cockpit_server.py) ---------------------------
# Per-joint torque caps (/1000). shoulder_lift/elbow_flex fight gravity and
# stall+overheat+latch if capped too low; the rest stay low to limit total draw
# on the 12V supply.
TORQUE_LIMITS: Dict[str, int] = {
    "shoulder_pan": 400,
    "shoulder_lift": 700,
    "elbow_flex": 700,
    "wrist_flex": 400,
    "wrist_roll": 400,
    "gripper": 400,
}
ACCELERATION = 30       # gentle ramp, avoids current spikes
GOAL_VELOCITY = 800     # raw units; fast enough for live tracking
MAX_STEP_TICKS = 170    # per-cycle follower move clamp (~15 deg)

CAL_DIR = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"


def make_bus(port: str):
    """Build a 6-motor Feetech bus for `port` (lazy lerobot import).

    Kept here so the CLI and cockpit construct the bus identically. lerobot is
    imported inside the call so this module stays importable on plain Py3.10.
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
    deadband_ticks: int = 2            # skip writes when the clamped step is tiny
    follower_read_every: int = 30      # sample follower present every N cycles (~0.5s @60Hz)
    torque_stagger_s: float = 0.05     # delay between per-joint torque enables
    one_euro: OneEuroConfig = field(default_factory=OneEuroConfig)

    @property
    def period(self) -> float:
        return 1.0 / self.target_hz


class TeleopEngine:
    """Owns the smoothing/last-goal state and the follower safety sequence.

    Bus objects are injected (and can be re-bound after a reconnect). The engine
    never opens or closes a port itself.
    """

    def __init__(self, leader_bus, follower_bus,
                 ranges: Dict[str, Tuple[int, int]],
                 config: Optional[TeleopConfig] = None,
                 joints: Optional[List[str]] = None,
                 log: Optional[Callable[[str], None]] = None):
        self.leader_bus = leader_bus
        self.follower_bus = follower_bus
        self.ranges = ranges
        self.cfg = config or TeleopConfig()
        self.joints = joints or list(JOINTS)
        self._log = log or (lambda _m: None)

        self.filters: Dict[str, OneEuroFilter] = {
            n: OneEuroFilter(self.cfg.one_euro, self.cfg.period) for n in self.joints
        }
        self.last_goal: Dict[str, int] = {n: 0 for n in self.joints}
        self.active = False

        # metrics
        self._cycle = 0
        self._last_step_t: Optional[float] = None
        self._measured_hz = 0.0
        self.follower_lag_ticks = 0

    # -- bus lifecycle ---------------------------------------------------------
    def rebind(self, leader_bus, follower_bus) -> None:
        """Point the engine at freshly reconnected buses (teleop must be off)."""
        self.leader_bus = leader_bus
        self.follower_bus = follower_bus

    def apply_follower_caps(self) -> None:
        """Write per-joint Torque_Limit / Acceleration / Goal_Velocity.

        Call while follower torque is DISABLED (before any staged enable) so the
        caps are in effect before the servos draw current — the brownout guard.
        """
        for n in self.joints:
            self.follower_bus.write("Torque_Limit", n, TORQUE_LIMITS[n], normalize=False)
            self.follower_bus.write("Acceleration", n, ACCELERATION, normalize=False)
            self.follower_bus.write("Goal_Velocity", n, GOAL_VELOCITY, normalize=False)

    # -- torque helpers --------------------------------------------------------
    def set_follower_torque(self, joint: str, on: bool) -> None:
        self.follower_bus.write("Torque_Enable", joint, 1 if on else 0, normalize=False)

    def enable_follower_torque_staged(self) -> None:
        """Torque-enable the follower one joint at a time (inrush control)."""
        for n in self.joints:
            self.set_follower_torque(n, True)
            time.sleep(self.cfg.torque_stagger_s)

    def freeze_follower(self) -> None:
        """Hold the follower where it is: command present->goal on live joints."""
        for n in self.joints:
            try:
                p = self.follower_bus.read("Present_Position", n, normalize=False)
                self.follower_bus.write("Goal_Position", n, int(p), normalize=False)
            except Exception:
                pass

    # -- teleop lifecycle ------------------------------------------------------
    def read_follower_present(self) -> Dict[str, int]:
        pos = self.follower_bus.sync_read("Present_Position", normalize=False, num_retry=1)
        return {n: int(pos[n]) for n in self.joints}

    def read_leader(self) -> Dict[str, int]:
        pos = self.leader_bus.sync_read("Present_Position", normalize=False, num_retry=1)
        return {n: int(pos[n]) for n in self.joints}

    def reset_filters(self) -> None:
        for f in self.filters.values():
            f.reset()

    def start_teleop(self) -> None:
        """Begin follow mode with the full safety sequence.

        follower present -> last_goal ; reset filters ; caps (torque off) ;
        staged torque enable ; go active. Order preserves the brownout guard
        (caps written before any torque draw).
        """
        present = self.read_follower_present()
        self.last_goal = dict(present)
        self.reset_filters()
        self.apply_follower_caps()
        self.enable_follower_torque_staged()
        self._cycle = 0
        self._last_step_t = None
        self.follower_lag_ticks = 0
        self.active = True

    def stop_teleop(self) -> None:
        self.active = False
        self.freeze_follower()

    # -- hot path --------------------------------------------------------------
    def compute_goals(self, leader_ticks: Dict[str, int],
                      now: Optional[float] = None) -> Dict[str, int]:
        """PURE decision step: filter -> range clamp -> last_goal +-step clamp ->
        deadband. Returns only the joints to WRITE and advances `last_goal` for
        them. No I/O.
        """
        if now is None:
            now = time.perf_counter()
        goals: Dict[str, int] = {}
        for n in self.joints:
            raw = float(leader_ticks[n])
            filt = self.filters[n].filter(raw, now)
            lo, hi = self.ranges.get(n, (0, 4095))
            tgt = min(hi, max(lo, filt))
            last = self.last_goal[n]
            step = max(-self.cfg.max_step_ticks,
                       min(self.cfg.max_step_ticks, tgt - last))
            newgoal = int(round(last + step))
            if abs(newgoal - last) >= self.cfg.deadband_ticks:
                goals[n] = newgoal
                self.last_goal[n] = newgoal
        return goals

    def write_goals(self, goals: Dict[str, int]) -> None:
        if goals:
            self.follower_bus.sync_write("Goal_Position", goals, normalize=False)

    def _update_hz(self, now: float) -> None:
        if self._last_step_t is not None:
            dt = now - self._last_step_t
            if dt > 0:
                inst = 1.0 / dt
                # EMA so the reported Hz is stable
                self._measured_hz = (0.9 * self._measured_hz + 0.1 * inst
                                     if self._measured_hz else inst)
        self._last_step_t = now

    def step(self, leader_ticks: Optional[Dict[str, int]] = None,
             now: Optional[float] = None) -> dict:
        """One teleop cycle: (read leader) -> compute -> write follower.

        `leader_ticks` may be supplied by the caller (cockpit already reads the
        leader for telemetry) to avoid a duplicate sync_read; if omitted the
        engine reads it. Every `follower_read_every` cycles it also samples the
        follower present position for a divergence warning (not fed into
        last_goal). Returns a dict with 'leader', 'goals', and optionally
        'follower' (present) for the caller's telemetry.
        """
        if now is None:
            now = time.perf_counter()
        if leader_ticks is None:
            leader_ticks = self.read_leader()

        goals = self.compute_goals(leader_ticks, now)
        self.write_goals(goals)

        out: dict = {"leader": leader_ticks, "goals": goals}

        self._cycle += 1
        if (self.cfg.follower_read_every > 0
                and self._cycle % self.cfg.follower_read_every == 0):
            try:
                present = self.read_follower_present()
                out["follower"] = present
                self.follower_lag_ticks = max(
                    abs(present[n] - self.last_goal[n]) for n in self.joints)
            except Exception:
                pass

        self._update_hz(now)
        return out

    # -- introspection ---------------------------------------------------------
    @property
    def measured_hz(self) -> float:
        return round(self._measured_hz, 1)

    def metrics(self) -> dict:
        oe = self.cfg.one_euro
        return {
            "measured_hz": self.measured_hz,
            "target_hz": self.cfg.target_hz,
            "max_step_ticks": self.cfg.max_step_ticks,
            "deadband_ticks": self.cfg.deadband_ticks,
            "follower_lag_ticks": self.follower_lag_ticks,
            "one_euro": {
                "min_cutoff": oe.min_cutoff,
                "beta": oe.beta,
                "d_cutoff": oe.d_cutoff,
            },
        }
