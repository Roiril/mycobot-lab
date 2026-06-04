"""Numerical inverse kinematics for SO-101 (5-DoF), damped least squares.

Standalone (Phase 0): consumes so101.kinematics + so101.profile, does not touch
the myCobot stack. Mirrors the DLS approach of src/arm/ik_numeric.py but
parameterized for N = profile.NUM_JOINTS joints.

IMPORTANT: SO-101 has only 5 arm DoF, so it CANNOT achieve an arbitrary 6-DoF
pose (position + full orientation). Position-only IK (3 constraints) is the
robust, well-posed mode and is the primary supported path — matching the
integration plan. Orientation can be requested as a soft objective but may not
be reachable; treat `solve_position` as the workhorse.
"""
from __future__ import annotations
import math
import time as _time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import profile
from .kinematics import tool_frame

DEG = math.pi / 180.0
N = profile.NUM_JOINTS


def _matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    sy = max(-1.0, min(1.0, -R[2, 0]))
    pitch = math.asin(sy)
    if abs(sy) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return (roll / DEG, pitch / DEG, yaw / DEG)


def _fk_pose(angles: Sequence[float]) -> np.ndarray:
    """TCP pose as 6-vector [x,y,z (mm), rx,ry,rz (deg)]."""
    T = tool_frame(angles)
    R = np.array([[T[i][j] for j in range(3)] for i in range(3)])
    rx, ry, rz = _matrix_to_rpy(R)
    return np.array([T[0][3], T[1][3], T[2][3], rx, ry, rz])


def _rpy_to_matrix(rx_deg, ry_deg, rz_deg) -> np.ndarray:
    rx, ry, rz = rx_deg * DEG, ry_deg * DEG, rz_deg * DEG
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy, sx * cy, cx * cy],
    ])


def _angular_error(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    R_err = R_target @ R_current.T
    cos = max(-1.0, min(1.0, (np.trace(R_err) - 1) * 0.5))
    theta = math.acos(cos)
    if abs(theta) < 1e-9:
        return np.zeros(3)
    sin = math.sin(theta)
    if sin < 1e-9:
        return np.zeros(3)
    return (theta / (2 * sin)) * np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ])


def _pose_error(target: np.ndarray, q: Sequence[float]) -> np.ndarray:
    T = tool_frame(q)
    R_cur = np.array([[T[i][j] for j in range(3)] for i in range(3)])
    R_tgt = _rpy_to_matrix(target[3], target[4], target[5])
    pos_err = np.array([target[0] - T[0][3], target[1] - T[1][3], target[2] - T[2][3]])
    return np.concatenate([pos_err, _angular_error(R_tgt, R_cur)])


def _jacobian(q: Sequence[float], eps: float = 1e-4) -> np.ndarray:
    """6xN numerical Jacobian (central differences). Orientation rows in rad."""
    J = np.zeros((6, N))
    for i in range(N):
        a = list(q); a[i] += eps
        plus = _fk_pose(a)
        a = list(q); a[i] -= eps
        minus = _fk_pose(a)
        d = (plus - minus) / (2 * eps)
        d[3:] *= DEG
        J[:, i] = d
    return J


def _clamp(q: Sequence[float]) -> List[float]:
    return [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(q, profile.JOINT_LIMITS)]


def solve(target: Sequence[float], seed: Sequence[float],
          max_iter: int = 80, pos_tol_mm: float = 2.0, rot_tol_deg: float = 6.0,
          rot_weight: float = 30.0, max_step_deg: float = 6.0,
          position_only: bool = True) -> Optional[List[float]]:
    """DLS IK with Levenberg-Marquardt damping.

    target: [x,y,z] or [x,y,z,rx,ry,rz]. position_only ignores orientation.
    Returns joint angles (deg, length N) or None.
    """
    if len(target) < 3:
        return None
    tgt = np.array([float(target[i]) if i < len(target) else 0.0 for i in range(6)])
    q = np.array(list(seed), dtype=float)
    rot_tol = rot_tol_deg * DEG
    rw = 0.0 if position_only else rot_weight
    W = np.diag([1.0, 1.0, 1.0, rw, rw, rw])
    I = np.eye(6)
    damping = 1.0
    last_total = float("inf")
    no_improve = 0
    best_q: Optional[np.ndarray] = None
    best_pos_err = float("inf")

    for _ in range(max_iter):
        err = _pose_error(tgt, q)
        pos_norm = float(np.linalg.norm(err[:3]))
        rot_norm = float(np.linalg.norm(err[3:]))
        if pos_norm < best_pos_err:
            best_pos_err = pos_norm; best_q = q.copy()
        if pos_norm < pos_tol_mm and (position_only or rot_norm < rot_tol):
            return _clamp(q)
        total = pos_norm + rw * rot_norm
        if total < last_total:
            damping = max(0.1, damping * 0.7); no_improve = 0
        else:
            damping = min(500.0, damping * 2.0); no_improve += 1
            if no_improve > 20:
                break
        last_total = min(last_total, total)
        Werr = W @ err
        WJ = W @ _jacobian(q)
        try:
            dq = WJ.T @ np.linalg.solve(WJ @ WJ.T + damping * damping * I, Werr)
        except np.linalg.LinAlgError:
            break
        dq_deg = dq / DEG
        nrm = float(np.linalg.norm(dq_deg))
        if nrm > max_step_deg:
            dq_deg *= max_step_deg / nrm
        q += dq_deg
        q = np.array(_clamp(q))

    if best_q is not None and best_pos_err < pos_tol_mm:
        return _clamp(best_q)
    return None


# Shoulder-lift / elbow-flex presets spanning elbow-up, elbow-down and reach
# extension — these two joints dominate the reach plane for a 5-DoF arm.
_LIFT_ELBOW_PRESETS = [
    (0.0, 0.0), (-45.0, 60.0), (45.0, -60.0),
    (-80.0, 90.0), (80.0, -90.0), (-30.0, 30.0), (30.0, -30.0),
]


def _seed_grid(seed: Sequence[float], target_xyz: Sequence[float]) -> List[List[float]]:
    """Seeds for position IK. Highest leverage: point shoulder_pan at the target
    azimuth, then sweep shoulder_lift/elbow_flex presets. The base-frame +x sign
    is not assumed, so both az and az±180 are tried.
    """
    s = list(seed)
    az = math.degrees(math.atan2(target_xyz[1], target_xyz[0]))
    pan_opts = {round(az, 1), round(az + 180.0, 1), round(az - 180.0, 1), round(s[0], 1)}
    raw: List[List[float]] = [s]
    for pan in pan_opts:
        for lift, elbow in _LIFT_ELBOW_PRESETS:
            raw.append([pan, lift, elbow, -(lift + elbow) * 0.5, 0.0])
    return [_clamp(a) for a in raw]


# Fast-fail sphere, derived from FK workspace sampling (200k random in-limit
# poses): max TCP distance from the base origin ≈ 546mm (the base column offsets
# the shoulder above the origin, so reach exceeds the summed link lengths).
# 560mm leaves margin so reachable targets are never wrongly rejected.
_MAX_REACH_MM = 560.0


def solve_position(target_xyz: Sequence[float],
                   seed: Optional[Sequence[float]] = None,
                   time_budget_s: float = 0.5) -> Optional[List[float]]:
    """Position-only IK with multi-seed retry. The primary SO-101 IK entry point.
    Returns joint angles (deg) or None if unreachable / no convergence.
    """
    if seed is None:
        seed = list(profile.HOME_ANGLES)
    if math.sqrt(sum(float(c) ** 2 for c in target_xyz[:3])) > _MAX_REACH_MM:
        return None
    t0 = _time.monotonic()
    tgt = list(target_xyz[:3])
    for alt in _seed_grid(seed, tgt):
        if _time.monotonic() - t0 > time_budget_s:
            break
        sol = solve(tgt, alt, position_only=True)
        if sol is not None:
            return sol
    # Deterministic random restarts as a last resort (still within budget).
    rng = np.random.default_rng(0)
    while _time.monotonic() - t0 <= time_budget_s:
        alt = _clamp([rng.uniform(lo, hi) for (lo, hi) in profile.JOINT_LIMITS])
        sol = solve(tgt, alt, position_only=True)
        if sol is not None:
            return sol
    return None
