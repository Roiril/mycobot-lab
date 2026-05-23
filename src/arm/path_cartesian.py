"""Cartesian path generation (waypoints in tool-space).

Position is interpolated linearly. Orientation is interpolated via quaternion
slerp — linear interp of Euler angles is incorrect (gimbal lock, no shortest path).
"""
from __future__ import annotations
import math
from typing import List, Sequence, Tuple

from .constants import (
    CART_STEP_MM, CART_LIFT_Z_MM, CART_LIFT_HORIZ_THRESHOLD_MM,
    CART_AUTO_LINEAR_THRESHOLD_MM,
)

Pose = Tuple[float, float, float, float, float, float]
D = math.pi / 180.0


# ---------- quaternion ↔ Euler (RPY: roll about X, pitch about Y, yaw about Z, extrinsic XYZ) ----------
def _euler_to_quat(rx: float, ry: float, rz: float):
    """Convention matches pymycobot's get_coords (RPY in deg, extrinsic XYZ)."""
    cx, sx = math.cos(rx * D / 2), math.sin(rx * D / 2)
    cy, sy = math.cos(ry * D / 2), math.sin(ry * D / 2)
    cz, sz = math.cos(rz * D / 2), math.sin(rz * D / 2)
    # Rz * Ry * Rx (extrinsic XYZ)
    qw = cx * cy * cz + sx * sy * sz
    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    return (qw, qx, qy, qz)


def _quat_to_euler(qw, qx, qy, qz):
    """Inverse of _euler_to_quat. Returns (rx, ry, rz) in deg."""
    # roll (x)
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    rx = math.atan2(sinr, cosr)
    # pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    ry = math.asin(sinp)
    # yaw (z)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    rz = math.atan2(siny, cosy)
    return (rx / D, ry / D, rz / D)


def _slerp(qa, qb, t: float):
    qw1, qx1, qy1, qz1 = qa
    qw2, qx2, qy2, qz2 = qb
    dot = qw1*qw2 + qx1*qx2 + qy1*qy2 + qz1*qz2
    if dot < 0:  # take shortest arc
        qw2, qx2, qy2, qz2 = -qw2, -qx2, -qy2, -qz2
        dot = -dot
    if dot > 0.9995:
        # Linear interp + renormalize (near-parallel)
        w = qw1 + t*(qw2-qw1); x = qx1 + t*(qx1-qx1); y = qy1 + t*(qy2-qy1); z = qz1 + t*(qz2-qz1)
    else:
        theta = math.acos(dot)
        s1 = math.sin((1-t)*theta) / math.sin(theta)
        s2 = math.sin(t*theta) / math.sin(theta)
        w = s1*qw1 + s2*qw2
        x = s1*qx1 + s2*qx2
        y = s1*qy1 + s2*qy2
        z = s1*qz1 + s2*qz2
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)


def _interp_pose(a: Pose, b: Pose, t: float) -> Pose:
    """Position lerp + orientation slerp."""
    px = a[0] + (b[0] - a[0]) * t
    py = a[1] + (b[1] - a[1]) * t
    pz = a[2] + (b[2] - a[2]) * t
    qa = _euler_to_quat(a[3], a[4], a[5])
    qb = _euler_to_quat(b[3], b[4], b[5])
    rx, ry, rz = _quat_to_euler(*_slerp(qa, qb, t))
    return (px, py, pz, rx, ry, rz)


# ---------- public path builders ----------
def linear(start: Sequence[float], target: Sequence[float], step_mm: float = CART_STEP_MM) -> List[Pose]:
    """Straight line in cartesian space. Orientation via slerp."""
    sx, sy, sz = start[:3]; tx, ty, tz = target[:3]
    dist = math.hypot(tx-sx, ty-sy, tz-sz)
    n = max(1, int(math.ceil(dist / step_mm)))
    return [_interp_pose(tuple(start), tuple(target), i/n) for i in range(1, n+1)]


def lift_translate_lower(start: Sequence[float], target: Sequence[float],
                         lift_z: float = CART_LIFT_Z_MM,
                         horiz_threshold: float = CART_LIFT_HORIZ_THRESHOLD_MM,
                         step_mm: float = CART_STEP_MM) -> List[Pose]:
    """3-segment table-safe motion. Falls through to linear if horizontal distance is small."""
    sx, sy, sz = start[:3]; tx, ty, tz = target[:3]
    horiz = math.hypot(tx-sx, ty-sy)
    if horiz <= horiz_threshold or horiz <= CART_AUTO_LINEAR_THRESHOLD_MM:
        return linear(start, target, step_mm)
    safe = max(sz, tz, lift_z)
    if safe <= sz + 10:
        return linear(start, target, step_mm)
    up_pose   = (sx, sy, safe, *start[3:])
    over_pose = (tx, ty, safe, *target[3:])
    pts: List[Pose] = []
    pts += linear(start, up_pose, step_mm)
    pts += linear(up_pose, over_pose, step_mm)
    pts += linear(over_pose, target, step_mm)
    return pts


def auto(start: Sequence[float], target: Sequence[float], step_mm: float = CART_STEP_MM) -> List[Pose]:
    """Pick lift for big horizontal moves, linear for small adjustments."""
    sx, sy, sz = start[:3]; tx, ty, tz = target[:3]
    if math.hypot(tx-sx, ty-sy) > CART_LIFT_HORIZ_THRESHOLD_MM:
        return lift_translate_lower(start, target, step_mm=step_mm)
    return linear(start, target, step_mm)
