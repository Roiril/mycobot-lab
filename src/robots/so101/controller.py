"""SO-101 motion control layer: safety-validated joint + Cartesian moves.

Sits on top of a So101DriverBase (real lerobot, mock, or MuJoCo) and combines
kinematics + IK + safety + a joint-space planner into the verbs the rest of the
stack needs. This is the SO-101-specific "brain" that the future SO101Hub
(HubBase) will wrap — keeping it driver-agnostic means it is fully testable
offline against MockSo101Driver and watchable against the MuJoCo driver.

Design parallels src/arm/hub.py's motion logic (plan -> validate each waypoint
-> execute) but for 5 DoF and with no hardware coupling.
"""
from __future__ import annotations
import math
import time
from typing import Callable, List, Optional, Sequence, Tuple

from . import profile
from .driver import So101DriverBase
from .ik import solve_position
from . import safety

# Default per-step joint increment when interpolating a path (deg).
PATH_STEP_DEG = 4.0

# Joint speed pacing (deg/s). Without pacing the waypoints are written
# back-to-back and the real servos chase at their own full speed.
# Values live in profile.SPEED_DPS (single source for server/UI too).
DEFAULT_SPEED_DPS = profile.SPEED_DPS["default"]
MIN_SPEED_DPS = profile.SPEED_DPS["min"]
MAX_SPEED_DPS = profile.SPEED_DPS["max"]

StepCb = Optional[Callable[[List[float]], None]]


def plan_joint_path(start: Sequence[float], goal: Sequence[float],
                    max_step_deg: float = PATH_STEP_DEG) -> List[List[float]]:
    """Linear joint-space interpolation start->goal. Returns waypoints
    (excluding start, including goal). Step count sized so no joint moves more
    than max_step_deg between waypoints."""
    deltas = [g - s for s, g in zip(start, goal)]
    max_delta = max((abs(d) for d in deltas), default=0.0)
    n = max(1, math.ceil(max_delta / max_step_deg))
    return [[s + d * (k / n) for s, d in zip(start, deltas)] for k in range(1, n + 1)]


AbortCb = Optional[Callable[[], bool]]


class So101Controller:
    def __init__(self, driver: So101DriverBase, step_deg: float = PATH_STEP_DEG,
                 sleep_fn: Callable[[float], None] = time.sleep):
        self.driver = driver
        self.step_deg = step_deg
        self.sleep_fn = sleep_fn  # injectable so tests/mocks don't pay real pacing

    def current_angles(self) -> List[float]:
        return self.driver.read_angles()

    def move_to_angles(self, target: Sequence[float], gripper: Optional[float] = None,
                       on_step: StepCb = None,
                       speed_dps: Optional[float] = None,
                       should_abort: AbortCb = None) -> Tuple[bool, str]:
        """Validate target + every interpolated waypoint with safety.check_angles,
        then drive the path paced at `speed_dps` (deg/s of the fastest joint).
        Aborts (without moving further) on the first unsafe waypoint, or as soon
        as `should_abort()` turns True (emergency stop: holds at the last
        commanded waypoint and reports)."""
        if len(target) != profile.NUM_JOINTS:
            return False, f"target length != {profile.NUM_JOINTS}"
        if not all(math.isfinite(float(a)) for a in target):
            return False, "target contains non-finite values"
        if gripper is not None and not math.isfinite(float(gripper)):
            return False, "gripper is non-finite"
        ok, msg, _ = safety.check_angles(target)
        if not ok:
            return False, f"target unsafe: {msg}"
        start = self.current_angles()
        waypoints = plan_joint_path(start, target, self.step_deg)
        for wp in waypoints:
            ok, msg, _ = safety.check_angles(wp)
            if not ok:
                return False, f"path unsafe: {msg}"
        sp = DEFAULT_SPEED_DPS if speed_dps is None else float(speed_dps)
        if not math.isfinite(sp):  # NaN slips through min/max clamps
            sp = DEFAULT_SPEED_DPS
        sp = max(MIN_SPEED_DPS, min(MAX_SPEED_DPS, sp))

        # Smooth path is linear per-joint (monotonic), so on a driver with its
        # own velocity-profiled motion (real servos) we send the FINAL target
        # ONCE at a matching Goal_Velocity and let the firmware glide — no
        # per-waypoint zip-then-stop stutter. Safety already validated every
        # waypoint above. Other drivers (mock/sim) keep streamed pacing so the
        # offline UI animates.
        if getattr(self.driver, "streams_smoothly", False):
            self.driver.set_speed_dps(sp)
            self.driver.write_angles(target, gripper)
            if on_step is not None:
                on_step(list(target))
            max_delta = max((abs(t - s) for t, s in zip(target, start)), default=0.0)
            deadline = time.time() + (max_delta / sp) + 2.0
            while time.time() < deadline:
                if should_abort is not None and should_abort():
                    self.driver.write_angles(self.current_angles(), gripper)  # stop here
                    return False, "aborted"
                cur = self.current_angles()
                if max((abs(c - t) for c, t in zip(cur, target)), default=0.0) <= 2.0:
                    break
                self.sleep_fn(0.05)
            return True, "ok"

        dt = self.step_deg / sp  # seconds per waypoint (fastest joint moves step_deg)
        prev = start
        for wp in waypoints:
            if should_abort is not None and should_abort():
                return False, "aborted"
            self.driver.write_angles(wp, gripper)
            if on_step is not None:
                on_step(wp)
            step = max((abs(a - b) for a, b in zip(wp, prev)), default=self.step_deg)
            prev = wp
            self.sleep_fn(min(dt, step / sp) if step > 0 else 0.0)
        return True, "ok"

    def move_to_position(self, xyz: Sequence[float], gripper: Optional[float] = None,
                         on_step: StepCb = None,
                         speed_dps: Optional[float] = None,
                         should_abort: AbortCb = None) -> Tuple[bool, str]:
        """Position-only IK from the current pose, then a validated joint move."""
        if not all(math.isfinite(float(c)) for c in xyz):
            return False, "xyz contains non-finite values"
        seed = self.current_angles()
        sol = solve_position(xyz, seed=seed)
        if sol is None:
            return False, f"no IK solution for {[round(c) for c in xyz[:3]]}"
        return self.move_to_angles(sol, gripper, on_step, speed_dps=speed_dps,
                                   should_abort=should_abort)

    def home(self, on_step: StepCb = None,
             speed_dps: Optional[float] = None,
             should_abort: AbortCb = None) -> Tuple[bool, str]:
        return self.move_to_angles(list(profile.HOME_ANGLES), on_step=on_step,
                                   speed_dps=speed_dps, should_abort=should_abort)

    def set_gripper(self, value_0_100: float) -> None:
        cur = self.current_angles()
        self.driver.write_angles(cur, gripper=value_0_100)
