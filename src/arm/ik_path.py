"""IK along a cartesian path with continuity enforcement.

Pipeline per cartesian waypoint:
  1. Try IK with previous joint solution as seed
  2. If solution jumps > IK_MAX_JOINT_JUMP_DEG (wrap-aware), subdivide the cartesian segment
  3. If IK returns None, try alternate seeds (perturbed J3, J5)
  4. Validate each result with joint-limit buffer
"""
from __future__ import annotations
import logging
from typing import Callable, List, Sequence, Tuple

from .constants import (
    IK_MAX_JOINT_JUMP_DEG, IK_MAX_SUBDIVIDE, IK_MAX_JOINT_WAYPOINTS,
    IK_SEED_RETRIES, JOINT_LIMIT_BUFFER_DEG,
)
from .kinematics import JOINT_LIMITS
from .safety import check_angles

log = logging.getLogger("mycobot.ik_path")

Pose = Tuple[float, float, float, float, float, float]
IKFn = Callable[[Sequence[float], Sequence[float]], List[float] | None]


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed difference between angles (deg), wrapped to [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def _max_joint_jump(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(_angle_diff(x, y)) for x, y in zip(a, b))


def _interp_pose(a: Pose, b: Pose, t: float) -> Pose:
    """Position lerp + orientation slerp via path_cartesian helpers."""
    from .path_cartesian import _interp_pose as ip
    return ip(a, b, t)


def _within_buffered_limits(angles: Sequence[float]) -> Tuple[bool, str, List[int]]:
    """Same as check_angles but uses buffered joint limits (so IK results near limits are rejected
    before they can become physical-limit violations under readback noise)."""
    bad = []
    for i, (a, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS), 1):
        if not (lo + JOINT_LIMIT_BUFFER_DEG) <= a <= (hi - JOINT_LIMIT_BUFFER_DEG):
            bad.append(i)
    if bad:
        i = bad[0]
        lo, hi = JOINT_LIMITS[i-1]
        return False, f"J{i}={angles[i-1]:.0f}° は安全余裕 [{lo+JOINT_LIMIT_BUFFER_DEG:.0f}, {hi-JOINT_LIMIT_BUFFER_DEG:.0f}] 外", bad
    return check_angles(angles)


def _try_ik_with_seeds(target, seed, ik: IKFn) -> List[float] | None:
    """Try IK with seed first, then alternate seeds on failure."""
    sol = ik(target, seed)
    if sol is not None:
        return sol
    # alternate seeds: flip J3 sign (elbow up/down branch), perturb J5 (wrist), perturb J6
    perturbations = []
    for i in range(min(IK_SEED_RETRIES, 3)):
        alt = list(seed)
        if i == 0:
            alt[2] = -alt[2]  # elbow flip
        elif i == 1:
            alt[4] = alt[4] + 30  # wrist perturb
        elif i == 2:
            alt[5] = alt[5] + 60  # tool perturb
        perturbations.append(alt)
    for alt in perturbations:
        sol = ik(target, alt)
        if sol is not None:
            return sol
    return None


def plan_ik_path(start_joints: Sequence[float],
                 start_pose: Pose,
                 cart_waypoints: List[Pose],
                 ik: IKFn,
                 max_jump: float = IK_MAX_JOINT_JUMP_DEG,
                 max_subdiv: int = IK_MAX_SUBDIVIDE,
                 max_wps: int = IK_MAX_JOINT_WAYPOINTS,
                 ) -> Tuple[List[List[float]], bool, str, List[int]]:
    """Resolve a cartesian path to a joint waypoint list.

    `start_pose` is the actual current end-effector pose (from FK or get_coords).
    `cart_waypoints` are the path waypoints AFTER start_pose (excluding start).
    """
    seed = list(start_joints)
    prev_pose: Pose = tuple(start_pose)
    out: List[List[float]] = []

    for wp_idx, target in enumerate(cart_waypoints):
        sub_ok, sub_joints, sub_msg, sub_bad = _resolve_segment(
            seed_pose=prev_pose, target=target, seed_joints=seed,
            ik=ik, max_jump=max_jump, depth=0, max_depth=max_subdiv, wp_idx=wp_idx,
        )
        if not sub_ok:
            return out, False, f"cartesian wp {wp_idx+1}/{len(cart_waypoints)}: {sub_msg}", sub_bad
        out.extend(sub_joints)
        if len(out) > max_wps:
            return out, False, f"経路 waypoint 超過 ({len(out)} > {max_wps})。step を粗くするか経路を短く", []
        seed = sub_joints[-1]
        prev_pose = target

    return out, True, "ok", []


def _resolve_segment(seed_pose: Pose, target: Pose, seed_joints: List[float],
                     ik: IKFn, max_jump: float, depth: int, max_depth: int, wp_idx: int,
                     ) -> Tuple[bool, List[List[float]], str, List[int]]:
    """Try IK for seed_pose → target. Subdivide on joint jump."""
    sol = _try_ik_with_seeds(target, seed_joints, ik)
    if sol is None:
        return False, [], f"IK 解なし (depth={depth}, wp={wp_idx+1})", []
    ok, msg, bad = _within_buffered_limits(sol)
    if not ok:
        return False, [], f"安全 NG: {msg}", bad
    jump = _max_joint_jump(seed_joints, sol)
    if jump <= max_jump:
        if depth > 0:
            log.info("ik_path 細分化成功 wp=%d depth=%d jump=%.1f°", wp_idx+1, depth, jump)
        return True, [sol], "ok", []
    # Detect wrist-flip (large delta on J4/J5/J6 that subdivide cannot fix)
    flip_joints = [i+1 for i, (a, b) in enumerate(zip(seed_joints, sol))
                   if i >= 3 and abs(_angle_diff(a, b)) > 170]
    if flip_joints:
        return False, [], f"IK wrist-flip 検出 (J{flip_joints}, 細分化不可)。経由姿勢を分割して指定して", flip_joints
    if depth >= max_depth:
        # report which joint jumped most
        max_i = max(range(6), key=lambda i: abs(_angle_diff(seed_joints[i], sol[i]))) + 1
        return False, [], f"IK 不連続 (J{max_i}が{jump:.0f}°, depth={max_depth}打切り)", [max_i]
    mid = _interp_pose(seed_pose, target, 0.5)
    ok_a, ja, msg_a, bad_a = _resolve_segment(seed_pose, mid, seed_joints, ik, max_jump, depth+1, max_depth, wp_idx)
    if not ok_a:
        return False, [], msg_a, bad_a
    ok_b, jb, msg_b, bad_b = _resolve_segment(mid, target, ja[-1], ik, max_jump, depth+1, max_depth, wp_idx)
    if not ok_b:
        return False, [], msg_b, bad_b
    return True, ja + jb, "ok", []
