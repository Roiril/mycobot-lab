"""Joint-space path planning + safety validation.

Pure functions — no hardware dependency. Given a start pose and target pose,
generate a sequence of intermediate joint configurations small enough that the
firmware planner produces continuous IK solutions, and validate every step
against safety.check_angles.
"""
from __future__ import annotations
import math
from typing import Sequence, Tuple, List

from .constants import PATH_STEP_DEG
from .safety import check_angles, clamp_angles


def plan_joint_path(start_angles: Sequence[float],
                    target_angles: Sequence[float],
                    step_deg: float = PATH_STEP_DEG) -> List[List[float]]:
    """Linear interp in joint space. Returns waypoints excluding start, including target."""
    deltas = [t - s for s, t in zip(start_angles, target_angles)]
    max_delta = max((abs(d) for d in deltas), default=0.0)
    n = max(1, int(math.ceil(max_delta / step_deg)))
    return [[s + d * (k / n) for s, d in zip(start_angles, deltas)] for k in range(1, n + 1)]


def plan_and_validate(start_angles: Sequence[float],
                      target_angles: Sequence[float]
                      ) -> Tuple[List[List[float]], bool, str, List[int]]:
    """Plan + validate every waypoint. Returns (waypoints, ok, msg, bad_joints).

    Note: does NOT silently clamp; over-limit targets are reported as their original value.
    """
    ok, msg, bad = check_angles(target_angles)
    if not ok:
        return [], False, f"目標 NG: {msg}", bad
    waypoints = plan_joint_path(start_angles, target_angles)
    for i, wp in enumerate(waypoints):
        ok, msg, bad = check_angles(wp)
        if not ok:
            return [], False, f"経路 waypoint {i+1}/{len(waypoints)} NG: {msg}", bad
    return waypoints, True, "ok", []
