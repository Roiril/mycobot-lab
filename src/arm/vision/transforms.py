"""SE(3) transform helpers for vision/camera frames.

Conventions:
  - All matrices are 4x4 row-major lists (compatible with kinematics.link_frames output).
  - Translation in mm, rotation in degrees (RPY = extrinsic XYZ = Rz*Ry*Rx).
  - Right-handed, base frame = robot base origin.

We deliberately keep numpy out of this module so it can be used in
pure-Python contexts (matches the no-numpy convention of kinematics.py).
"""
from __future__ import annotations
import math
from typing import Sequence, List, Tuple

from ..kinematics import link_frames

D = math.pi / 180.0


def identity4() -> List[List[float]]:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def compose(*Ts):
    """T_final = T1 @ T2 @ ... @ Tn (left-to-right composition)."""
    out = identity4()
    for T in Ts:
        out = mat_mul(out, T)
    return out


def invert(T):
    """Invert a rigid-body 4x4 transform (R^T, -R^T * t)."""
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    t = [T[i][3] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]  # transpose
    tinv = [-(Rt[i][0]*t[0] + Rt[i][1]*t[1] + Rt[i][2]*t[2]) for i in range(3)]
    out = identity4()
    for i in range(3):
        for j in range(3):
            out[i][j] = Rt[i][j]
        out[i][3] = tinv[i]
    return out


def xyz_rpy_to_T(x, y, z, rx_deg, ry_deg, rz_deg):
    """Build a 4x4 from translation (mm) + RPY (deg). Same convention as pose_resolver:
    extrinsic XYZ Euler = Rz * Ry * Rx applied in that order to body frame.
    """
    rx, ry, rz = rx_deg * D, ry_deg * D, rz_deg * D
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy],
    ]
    T = identity4()
    for i in range(3):
        for j in range(3):
            T[i][j] = R[i][j]
    T[0][3] = x; T[1][3] = y; T[2][3] = z
    return T


def T_to_xyz_rpy(T) -> Tuple[float, float, float, float, float, float]:
    """Decompose 4x4 → (x,y,z,rx,ry,rz) deg."""
    x, y, z = T[0][3], T[1][3], T[2][3]
    sy = -T[2][0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 0.99999:
        roll = math.atan2(T[2][1], T[2][2])
        yaw = math.atan2(T[1][0], T[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-T[0][1], T[1][1])
    return (x, y, z, roll / D, pitch / D, yaw / D)


def T_base_ee(angles_deg: Sequence[float]):
    """End-effector (J6 frame) transform in base coords. Uses the same DH chain as kinematics."""
    return link_frames(angles_deg)[-1]


def T_base_cam_wrist(angles_deg: Sequence[float], T_ee_cam) -> List[List[float]]:
    """Compose base→camera via current end-effector pose and hand-eye calibration.

    T_base_cam = T_base_ee @ T_ee_cam
    """
    Tbe = T_base_ee(angles_deg)
    return mat_mul(Tbe, T_ee_cam)


def apply(T, p3) -> Tuple[float, float, float]:
    """Apply 4x4 to a 3-vector (treating as point)."""
    x = T[0][0]*p3[0] + T[0][1]*p3[1] + T[0][2]*p3[2] + T[0][3]
    y = T[1][0]*p3[0] + T[1][1]*p3[1] + T[1][2]*p3[2] + T[1][3]
    z = T[2][0]*p3[0] + T[2][1]*p3[1] + T[2][2]*p3[2] + T[2][3]
    return (x, y, z)


def apply_rot(T, v3) -> Tuple[float, float, float]:
    """Apply rotation part of T to a 3-vector (treating as direction)."""
    x = T[0][0]*v3[0] + T[0][1]*v3[1] + T[0][2]*v3[2]
    y = T[1][0]*v3[0] + T[1][1]*v3[1] + T[1][2]*v3[2]
    z = T[2][0]*v3[0] + T[2][1]*v3[1] + T[2][2]*v3[2]
    return (x, y, z)
