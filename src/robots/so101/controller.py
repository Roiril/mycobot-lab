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
from typing import Callable, List, Optional, Sequence, Tuple

from . import profile
from .driver import So101DriverBase
from .ik import solve_position
from . import safety

# Default per-step joint increment when interpolating a path (deg).
PATH_STEP_DEG = 4.0

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


class So101Controller:
    def __init__(self, driver: So101DriverBase, step_deg: float = PATH_STEP_DEG):
        self.driver = driver
        self.step_deg = step_deg

    def current_angles(self) -> List[float]:
        return self.driver.read_angles()

    def move_to_angles(self, target: Sequence[float], gripper: Optional[float] = None,
                       on_step: StepCb = None) -> Tuple[bool, str]:
        """Validate target + every interpolated waypoint with safety.check_angles,
        then drive the path. Aborts (without moving further) on the first unsafe
        waypoint. Returns (ok, message)."""
        if len(target) != profile.NUM_JOINTS:
            return False, f"target length != {profile.NUM_JOINTS}"
        ok, msg, _ = safety.check_angles(target)
        if not ok:
            return False, f"target unsafe: {msg}"
        start = self.current_angles()
        waypoints = plan_joint_path(start, target, self.step_deg)
        for wp in waypoints:
            ok, msg, _ = safety.check_angles(wp)
            if not ok:
                return False, f"path unsafe: {msg}"
        for wp in waypoints:
            self.driver.write_angles(wp, gripper)
            if on_step is not None:
                on_step(wp)
        return True, "ok"

    def move_to_position(self, xyz: Sequence[float], gripper: Optional[float] = None,
                         on_step: StepCb = None) -> Tuple[bool, str]:
        """Position-only IK from the current pose, then a validated joint move."""
        seed = self.current_angles()
        sol = solve_position(xyz, seed=seed)
        if sol is None:
            return False, f"no IK solution for {[round(c) for c in xyz[:3]]}"
        return self.move_to_angles(sol, gripper, on_step)

    def home(self, on_step: StepCb = None) -> Tuple[bool, str]:
        return self.move_to_angles(list(profile.HOME_ANGLES), on_step=on_step)

    def set_gripper(self, value_0_100: float) -> None:
        cur = self.current_angles()
        self.driver.write_angles(cur, gripper=value_0_100)
