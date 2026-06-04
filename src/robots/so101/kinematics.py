"""Forward kinematics for SO-101 (5-DoF arm), URDF-derived.

Standalone for now (Phase 0 of the SO-101 integration plan): consumes the data
in profile.py and does not touch the myCobot stack. The math mirrors
src/arm/kinematics.py exactly (URDF convention: Rz*Ry*Rx, joint about child z),
so this can later be hoisted into a shared, parameterized core FK.

Difference vs myCobot: the tool offset is a full fixed SE(3) transform
(profile.TOOL_TRANSFORM = the URDF `gripper_frame_joint`), not a scalar +z
extension. The arm has 5 revolute DoF; the gripper jaw is not in this chain.
"""
from __future__ import annotations
import math
from typing import List, Sequence

from . import profile


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
    """Return [T0..T5]: 6 cumulative base-relative transforms.
    T0 = base, T5 = wrist_roll output frame (where the gripper mounts).
    """
    if len(angles_deg) != profile.NUM_JOINTS:
        raise ValueError(f"angles must be length {profile.NUM_JOINTS}")
    T = [_identity()]
    for i in range(profile.NUM_JOINTS):
        xyz, rpy = profile.URDF_LINKS[i]
        step = _mat_mul(_origin_tf(xyz, rpy), _rotz(math.radians(angles_deg[i])))
        T.append(_mat_mul(T[-1], step))
    return T


def joint_positions(angles_deg: Sequence[float]) -> List[tuple[float, float, float]]:
    """Origins of frames 0..5 in base coords (mm)."""
    return [(T[0][3], T[1][3], T[2][3]) for T in link_frames(angles_deg)]


def tool_frame(angles_deg: Sequence[float]) -> List[List[float]]:
    """Full 4x4 pose of the gripper TCP frame in base coords.
    = wrist_roll output frame composed with the fixed TOOL_TRANSFORM.
    """
    xyz, rpy = profile.TOOL_TRANSFORM
    return _mat_mul(link_frames(angles_deg)[-1], _origin_tf(xyz, rpy))


def end_effector(angles_deg: Sequence[float]) -> tuple[float, float, float]:
    """Gripper TCP position (mm) in base coords."""
    T = tool_frame(angles_deg)
    return (T[0][3], T[1][3], T[2][3])
