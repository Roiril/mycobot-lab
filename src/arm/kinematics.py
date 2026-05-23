"""Forward kinematics for myCobot 320-M5.

Modified DH convention (Craig). For each joint i:
  T_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i + theta_offset_i) * Tz(d_i)

The tool offset (TOOL_LENGTH along J6 z-axis) is approximate — the true tool
direction has a frame-orientation discrepancy that the safety code compensates
for via FLOOR_Z's FK_TOOL_SLOP margin.
"""
from __future__ import annotations
import math
from typing import List, Sequence

from .constants import TOOL_LENGTH

# (alpha_{i-1} [deg], a_{i-1} [mm], d_i [mm], theta_offset_i [deg])
DH = [
    (   0.0,    0.0, 173.9,    0.0),   # J1
    (  90.0,    0.0,   0.0,  -90.0),   # J2
    (   0.0, -135.0,   0.0,    0.0),   # J3
    (   0.0, -120.0,  88.78,  90.0),   # J4
    ( -90.0,    0.0,  95.0,  -90.0),   # J5
    (  90.0,    0.0,   0.0,    0.0),   # J6 (tool d=0; offset handled via TOOL_LENGTH)
]

# Hardware joint limits (myCobot 320 spec, degrees).
JOINT_LIMITS = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-165.0, 165.0),
    (-180.0, 180.0),
]


def _identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _link_tf(alpha_deg, a, d, theta_deg):
    al = math.radians(alpha_deg)
    th = math.radians(theta_deg)
    ca, sa = math.cos(al), math.sin(al)
    ct, st = math.cos(th), math.sin(th)
    return [
        [ct,        -st,       0.0,    a       ],
        [st * ca,   ct * ca,  -sa,    -sa * d  ],
        [st * sa,   ct * sa,   ca,     ca * d  ],
        [0.0,       0.0,       0.0,    1.0     ],
    ]


def link_frames(angles_deg: Sequence[float]) -> List[List[List[float]]]:
    """Return [T0..T6], 7 cumulative base-relative transforms."""
    if len(angles_deg) != 6:
        raise ValueError("angles must be length 6")
    T = [_identity()]
    for i in range(6):
        alpha_prev, a_prev, d, theta_off = DH[i]
        theta = angles_deg[i] + theta_off
        T.append(_mat_mul(T[-1], _link_tf(alpha_prev, a_prev, d, theta)))
    return T


def joint_positions(angles_deg: Sequence[float]) -> List[tuple[float, float, float]]:
    """Origins of frames 0..6 in base coords (mm)."""
    return [(T[0][3], T[1][3], T[2][3]) for T in link_frames(angles_deg)]


def end_effector(angles_deg: Sequence[float]) -> tuple[float, float, float]:
    """Tool tip position — J6 origin extended by TOOL_LENGTH along J6 z-axis."""
    T = link_frames(angles_deg)[-1]
    return (T[0][3] + TOOL_LENGTH * T[0][2],
            T[1][3] + TOOL_LENGTH * T[1][2],
            T[2][3] + TOOL_LENGTH * T[2][2])
