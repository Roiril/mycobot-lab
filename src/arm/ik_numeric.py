"""Numerical inverse kinematics (damped least squares Jacobian).

Same DH as kinematics.py. Used as offline-mode IK and as a fallback when
firmware solve_inv_kinematics returns None.

API: solve(target_xyzrxryrz, seed_angles) -> angles | None

Target orientation (rx, ry, rz) follows the same RPY convention as path_cartesian
(extrinsic XYZ). If you only need position-only IK, pass the seed orientation as
rx,ry,rz of the target (so orientation error stays small).
"""
from __future__ import annotations
import math
from typing import Optional, Sequence, List, Tuple

import numpy as np

from .kinematics import link_frames, end_effector, JOINT_LIMITS, DH, TOOL_LENGTH

DEG = math.pi / 180.0


def _fk_pose(angles: Sequence[float]) -> np.ndarray:
    """Return 6-vector [x,y,z, rx,ry,rz] (mm, deg)."""
    T = link_frames(angles)[-1]
    # tool tip = T6 origin + TOOL_LENGTH along z6
    px = T[0][3] + TOOL_LENGTH * T[0][2]
    py = T[1][3] + TOOL_LENGTH * T[1][2]
    pz = T[2][3] + TOOL_LENGTH * T[2][2]
    R = np.array([[T[0][0], T[0][1], T[0][2]],
                  [T[1][0], T[1][1], T[1][2]],
                  [T[2][0], T[2][1], T[2][2]]])
    rx, ry, rz = _matrix_to_rpy(R)
    return np.array([px, py, pz, rx, ry, rz])


def _matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Rotation matrix → (roll, pitch, yaw) deg (extrinsic XYZ = intrinsic ZYX-RPY)."""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw  = math.atan2(R[1, 0], R[0, 0])
    else:
        # gimbal lock
        roll = 0.0
        yaw  = math.atan2(-R[0, 1], R[1, 1])
    return (roll / DEG, pitch / DEG, yaw / DEG)


def _angular_error(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    """Returns angular-velocity error (3-vec, rad) to rotate current → target."""
    R_err = R_target @ R_current.T
    # log-map of rotation
    cos = (np.trace(R_err) - 1) * 0.5
    cos = max(-1.0, min(1.0, cos))
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


def _rpy_to_matrix(rx_deg, ry_deg, rz_deg) -> np.ndarray:
    rx, ry, rz = rx_deg * DEG, ry_deg * DEG, rz_deg * DEG
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rz * Ry * Rx
    return np.array([
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [   -sy,                sx * cy,                 cx * cy],
    ])


def _pose_error(target: np.ndarray, current_angles: Sequence[float]) -> np.ndarray:
    """Return 6-vec error [dx,dy,dz, ωx,ωy,ωz] (mm, rad)."""
    T = link_frames(current_angles)[-1]
    px = T[0][3] + TOOL_LENGTH * T[0][2]
    py = T[1][3] + TOOL_LENGTH * T[1][2]
    pz = T[2][3] + TOOL_LENGTH * T[2][2]
    R_cur = np.array([[T[0][0], T[0][1], T[0][2]],
                      [T[1][0], T[1][1], T[1][2]],
                      [T[2][0], T[2][1], T[2][2]]])
    R_tgt = _rpy_to_matrix(target[3], target[4], target[5])
    pos_err = np.array([target[0] - px, target[1] - py, target[2] - pz])
    ang_err = _angular_error(R_tgt, R_cur)
    return np.concatenate([pos_err, ang_err])


def _jacobian(angles: Sequence[float], eps: float = 1e-4) -> np.ndarray:
    """6x6 numerical Jacobian via central differences."""
    J = np.zeros((6, 6))
    base = _fk_pose(angles)
    for i in range(6):
        a = list(angles); a[i] += eps
        plus = _fk_pose(a)
        a = list(angles); a[i] -= eps
        minus = _fk_pose(a)
        d = (plus - minus) / (2 * eps)
        # convert orientation deg → rad for the angular columns
        d[3:] *= DEG
        J[:, i] = d
    return J


def _clamp_to_limits(angles: Sequence[float]) -> List[float]:
    return [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(angles, JOINT_LIMITS)]


def _rotate_orientation_around_tool_z(rpy_deg: Tuple[float,float,float], roll_off_deg: float) -> Tuple[float,float,float]:
    """Rotate the orientation by `roll_off_deg` around its OWN tool z-axis (extrinsic XYZ Euler).
    Returns new (rx, ry, rz) in deg.
    """
    rx, ry, rz = (a * DEG for a in rpy_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz * Ry * Rx (extrinsic XYZ matches path_cartesian)
    R = np.array([
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [   -sy,           sx*cy,           cx*cy],
    ])
    # Tool z-axis (third column of R)
    tz = R[:, 2]
    a = roll_off_deg * DEG
    c, s = math.cos(a), math.sin(a)
    ux, uy, uz = float(tz[0]), float(tz[1]), float(tz[2])
    Rrot = np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)],
    ])
    Rnew = Rrot @ R
    # Extract back to RPY (extrinsic XYZ)
    sy_ = -Rnew[2, 0]; sy_ = max(-1.0, min(1.0, sy_))
    pitch = math.asin(sy_)
    if abs(sy_) < 0.99999:
        roll = math.atan2(Rnew[2, 1], Rnew[2, 2])
        yaw  = math.atan2(Rnew[1, 0], Rnew[0, 0])
    else:
        roll = 0.0
        yaw  = math.atan2(-Rnew[0, 1], Rnew[1, 1])
    return (roll / DEG, pitch / DEG, yaw / DEG)


def _seed_perturbations(seed: Sequence[float]) -> List[List[float]]:
    """Alternate seeds for retry. CLAMPED to joint limits so first iteration starts feasible."""
    s = list(seed)
    raw = [
        s,                                                            # original
        [s[0], s[1], -s[2], s[3], s[4], s[5]],                        # elbow flip
        [s[0], s[1], s[2], s[3], s[4] + 90, s[5]],                    # J5 +90
        [s[0], s[1], s[2], s[3], s[4] - 90, s[5]],                    # J5 -90
        [s[0], s[1], s[2], s[3], s[4], s[5] + 90],                    # J6 roll +90
        [s[0], s[1], s[2], s[3], s[4], s[5] - 90],                    # J6 roll -90
        [s[0], s[1], s[2], s[3] + 30, s[4] - 30, s[5]],               # wrist combo
    ]
    return [_clamp_to_limits(a) for a in raw]


def solve_with_retries(target_pos: Sequence[float],
                       target_orientation: Optional[Tuple[float, float, float]],
                       seed: Sequence[float],
                       roll_relaxation_deg: Tuple[float, ...] = (0, 15, -15, 30, -30, 45, -45)
                       ) -> tuple[Optional[List[float]], str]:
    """Try full-6DoF IK with multi-seed retry + orientation relaxation fallback.

    target_orientation: (rx,ry,rz) or None for position-only.
    Returns (angles, mode) where mode ∈ {"full", "relaxed_roll", "position_only", "failed"}.
    """
    # Full pose attempts with original + perturbed seeds
    if target_orientation is not None:
        full_target = list(target_pos[:3]) + list(target_orientation)
        for alt_seed in _seed_perturbations(seed):
            sol = solve(full_target, alt_seed, position_only=False)
            if sol is not None:
                return sol, "full"
        # Roll relaxation: rotate the TARGET'S tool z-axis (not world Z) by ±N°.
        # Convert RPY → matrix, multiply by Rz_tool, extract new RPY.
        for roll_off in roll_relaxation_deg:
            if roll_off == 0: continue
            relaxed_rpy = _rotate_orientation_around_tool_z(target_orientation, roll_off)
            relaxed = [target_pos[0], target_pos[1], target_pos[2],
                       relaxed_rpy[0], relaxed_rpy[1], relaxed_rpy[2]]
            for alt_seed in _seed_perturbations(seed):
                sol = solve(relaxed, alt_seed, position_only=False)
                if sol is not None:
                    return sol, "relaxed_roll"
    # Last resort: position-only with seed perturbations
    pos_target = list(target_pos[:3]) + [0.0, 0.0, 0.0]
    for alt_seed in _seed_perturbations(seed):
        sol = solve(pos_target, alt_seed, position_only=True)
        if sol is not None:
            return sol, "position_only"
    return None, "failed"


def solve(target: Sequence[float], seed: Sequence[float],
          max_iter: int = 200, pos_tol_mm: float = 2.0, rot_tol_deg: float = 5.0,
          rot_weight: float = 50.0, max_step_deg: float = 3.0,
          position_only: bool = False) -> Optional[List[float]]:
    """Damped least squares IK with adaptive damping (Levenberg-Marquardt style).

    position_only=True ignores target orientation (uses 0 weight) — best for UI drag,
    where the user only cares about tool tip position.

    Returns best-effort joints when pos_tol is met (rot may still be loose due to singularities).
    """
    if len(target) < 3:
        return None
    tgt = np.array([
        float(target[0]), float(target[1]), float(target[2]),
        float(target[3]) if len(target) > 3 else 0.0,
        float(target[4]) if len(target) > 4 else 0.0,
        float(target[5]) if len(target) > 5 else 0.0,
    ])
    q = np.array(list(seed), dtype=float)
    rot_tol = rot_tol_deg * DEG
    rw = 0.0 if position_only else rot_weight
    W = np.diag([1, 1, 1, rw, rw, rw])
    I = np.eye(6)
    damping = 1.0
    last_total = float('inf')
    no_improve = 0
    best_q: Optional[np.ndarray] = None
    best_pos_err = float('inf')

    for it in range(max_iter):
        err = _pose_error(tgt, q)
        pos_norm = np.linalg.norm(err[:3])
        rot_norm = np.linalg.norm(err[3:])
        if pos_norm < best_pos_err:
            best_pos_err = pos_norm; best_q = q.copy()
        # success: position met AND (orientation met OR position_only)
        if pos_norm < pos_tol_mm and (position_only or rot_norm < rot_tol):
            return [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(q, JOINT_LIMITS)]
        total = pos_norm + rw * rot_norm
        if total < last_total:
            damping = max(0.1, damping * 0.7)
            no_improve = 0
        else:
            damping = min(500.0, damping * 2.0)
            no_improve += 1
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
        nrm = np.linalg.norm(dq_deg)
        if nrm > max_step_deg:
            dq_deg *= max_step_deg / nrm
        q += dq_deg
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            if q[i] < lo: q[i] = lo
            elif q[i] > hi: q[i] = hi

    # Best-effort fallback: if position alone is within tolerance, accept
    if best_q is not None and best_pos_err < pos_tol_mm:
        return [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(best_q, JOINT_LIMITS)]
    return None
