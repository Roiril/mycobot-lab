"""Forward kinematics for myCobot 320 M5 (URDF-derived).

Derived from elephantrobotics/mycobot_ros2 URDF (mycobot_320_m5_2022).
Verified against firmware get_coords() across 6 poses: residual <= 1.1 mm.

Convention: each link i has a fixed parent→child transform (xyz, rpy) followed
by a revolute joint about the child frame's z-axis. The 7 frames returned by
link_frames() are: [base, after_j1, after_j2, ..., after_j5, flange].
The flange is the tool mount face — its +z points out of the flange face.

Historical note: prior code used a Modified-DH parameterization that had
sign errors in alpha_5/alpha_6 and treated the tool-flange offset as a
separate TOOL_LENGTH along the wrong axis. The result was tip-position
errors of 60-90 mm at typical poses. See git log f10bf81..HEAD.
"""
from __future__ import annotations
import math
from typing import List, Sequence

from .constants import TOOL_LENGTH

PI = math.pi

# URDF parent→child transforms (xyz_mm, rpy_rad). Joint angle rotates about child z.
URDF_LINKS = [
    ((  0.0,   0.0, 173.9),  ( 0.0,   0.0,    0.0)),    # base   → j1
    ((  0.0, -88.78,  0.0),  ( 0.0,  -PI/2,   PI/2)),   # j1     → j2
    ((135.0,  0.0, -88.78),  ( 0.0,   0.0,    0.0)),    # j2     → j3
    ((120.0,  0.0,  88.78),  ( 0.0,   0.0,    PI/2)),   # j3     → j4
    ((  0.0, -95.0,  0.0),   ( PI/2,  0.0,    0.0)),    # j4     → j5
    ((  0.0,  65.5,  0.0),   (-PI/2,  0.0,    0.0)),    # j5     → flange (j6 output)
]

# Hardware joint limits (myCobot 320, degrees).
# J3 firmware enforces ±145 (not the ±150 advertised in some spec sheets) —
# verified empirically: send_angles rejects -150 with "should be -145 ~ 145".
JOINT_LIMITS = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-145.0, 145.0),
    (-145.0, 145.0),
    (-165.0, 165.0),
    (-180.0, 180.0),
]

# Kept for backward compatibility with anything that imports DH; values are now
# representative-only and not used by FK. Do not rely on them for IK.
DH = [
    (0.0, 0.0, 173.9, 0.0),
    (-90.0, 0.0, 0.0, -90.0),
    (0.0, -135.0, 0.0, 0.0),
    (0.0, -120.0, 88.78, -90.0),
    (90.0, 0.0, 95.0, 90.0),
    (-90.0, 0.0, 65.5, 0.0),
]


def _identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy_to_mat(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rz * Ry * Rx (URDF convention)
    return [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,                sx*cy,           cx*cy],
    ]


def _origin_tf(xyz, rpy):
    R = _rpy_to_mat(*rpy)
    return [
        [R[0][0], R[0][1], R[0][2], xyz[0]],
        [R[1][0], R[1][1], R[1][2], xyz[1]],
        [R[2][0], R[2][1], R[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotz(theta):
    c, s = math.cos(theta), math.sin(theta)
    return [
        [c, -s, 0.0, 0.0],
        [s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def link_frames(angles_deg: Sequence[float]) -> List[List[List[float]]]:
    """Return [T0..T6]: 7 cumulative base-relative transforms.
    T6 is the tool flange frame (joint6_output); +z is the tool-out direction.
    """
    if len(angles_deg) != 6:
        raise ValueError("angles must be length 6")
    T = [_identity()]
    for i in range(6):
        xyz, rpy = URDF_LINKS[i]
        step = _mat_mul(_origin_tf(xyz, rpy), _rotz(math.radians(angles_deg[i])))
        T.append(_mat_mul(T[-1], step))
    return T


def joint_positions(angles_deg: Sequence[float]) -> List[tuple[float, float, float]]:
    """Origins of frames 0..6 in base coords (mm)."""
    return [(T[0][3], T[1][3], T[2][3]) for T in link_frames(angles_deg)]


def end_effector(angles_deg: Sequence[float]) -> tuple[float, float, float]:
    """Tool tip position. With no gripper (TOOL_LENGTH=0) this is the flange.
    With a gripper attached, set TOOL_LENGTH = extension beyond flange (mm)
    along the flange +z direction.
    """
    T = link_frames(angles_deg)[-1]
    return (T[0][3] + TOOL_LENGTH * T[0][2],
            T[1][3] + TOOL_LENGTH * T[1][2],
            T[2][3] + TOOL_LENGTH * T[2][2])
