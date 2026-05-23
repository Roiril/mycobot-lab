"""Resolve a Pose discriminated union to a concrete (rx, ry, rz) Euler triple.

Pose kinds:
  {kind:"preserve"}                                — keep current orientation (return None)
  {kind:"any"}                                      — IK pick anything (return None, position-only)
  {kind:"explicit", euler_xyz:[r,p,y]}             — exact RPY (deg)
  {kind:"explicit", quat:[w,x,y,z]}                — exact quaternion
  {kind:"align_tool", approach:"+z"|... or [vx,vy,vz], roll_deg?:n}
                                                   — tool z-axis = -approach; optional roll
  {kind:"extend_toward", target:[x,y,z], roll_deg?:n}
                                                   — tool z-axis points from current J6 to target

Returns (rx, ry, rz) in deg (RPY extrinsic XYZ matching path_cartesian convention),
or None when caller should use position-only IK / preserve.
"""
from __future__ import annotations
import math
from typing import Optional, Sequence, Tuple

from .kinematics import link_frames

D = math.pi / 180.0

AXIS_PRESETS = {
    "+x": (1.0, 0.0, 0.0),  "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),  "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),  "-z": (0.0, 0.0, -1.0),
}


def _normalize(v):
    n = math.sqrt(sum(c*c for c in v))
    return (v[0]/n, v[1]/n, v[2]/n) if n > 1e-9 else (0.0, 0.0, 1.0)


def _resolve_axis(spec) -> Tuple[float, float, float]:
    if isinstance(spec, str):
        if spec in AXIS_PRESETS: return AXIS_PRESETS[spec]
        raise ValueError(f"unknown axis preset: {spec!r}")
    if isinstance(spec, (list, tuple)) and len(spec) == 3:
        return _normalize(tuple(float(c) for c in spec))
    raise ValueError(f"axis must be preset string or 3-vector, got {spec!r}")


def _matrix_to_rpy(R) -> Tuple[float, float, float]:
    """Rotation matrix → (rx, ry, rz) in deg (extrinsic XYZ = ZYX-RPY)."""
    sy = -R[2][0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 0.99999:
        roll = math.atan2(R[2][1], R[2][2])
        yaw  = math.atan2(R[1][0], R[0][0])
    else:
        roll = 0.0
        yaw  = math.atan2(-R[0][1], R[1][1])
    return (roll / D, pitch / D, yaw / D)


def _matrix_from_tool_z(tool_z, prefer_x_world=None):
    """Build a 3x3 rotation matrix with the given tool z-axis. The X axis is chosen
    perpendicular to z, biased toward `prefer_x_world` (a hint, default world X).
    Returns 3x3 row-major list. Roll around tool_z is at zero in the returned matrix
    (caller can multiply by Rz(roll) later)."""
    tz = _normalize(tool_z)
    # Choose initial x: world X projected onto plane perpendicular to tz
    hint = prefer_x_world if prefer_x_world is not None else (1.0, 0.0, 0.0)
    # If hint is parallel to tz, use world Y instead
    d = abs(hint[0]*tz[0] + hint[1]*tz[1] + hint[2]*tz[2])
    if d > 0.99:
        hint = (0.0, 1.0, 0.0)
    # x = hint - (hint·tz)*tz
    dot = hint[0]*tz[0] + hint[1]*tz[1] + hint[2]*tz[2]
    tx = (hint[0] - dot*tz[0], hint[1] - dot*tz[1], hint[2] - dot*tz[2])
    tx = _normalize(tx)
    # y = tz × tx
    ty = (tz[1]*tx[2] - tz[2]*tx[1],
          tz[2]*tx[0] - tz[0]*tx[2],
          tz[0]*tx[1] - tz[1]*tx[0])
    # Column-major then row-major output: R = [tx | ty | tz]
    return [
        [tx[0], ty[0], tz[0]],
        [tx[1], ty[1], tz[1]],
        [tx[2], ty[2], tz[2]],
    ]


def _rotate_around_axis(R, axis, deg):
    """Rotate matrix R around the given axis (in world frame) by deg degrees."""
    a = deg * D
    c, s = math.cos(a), math.sin(a)
    ux, uy, uz = _normalize(axis)
    # Rodrigues
    Rrot = [
        [c + ux*ux*(1-c),     ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s,  c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s,  uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)],
    ]
    # Rrot * R
    out = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(Rrot[i][k] * R[k][j] for k in range(3))
    return out


def _quat_to_matrix(qw, qx, qy, qz):
    return [
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ]


def _euler_xyz_to_matrix(rx_deg, ry_deg, rz_deg):
    """Same convention as path_cartesian: Rz * Ry * Rx (extrinsic XYZ)."""
    rx, ry, rz = rx_deg * D, ry_deg * D, rz_deg * D
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy],
    ]


def _current_j6_origin(angles):
    """Return (x,y,z) of J6 frame origin (the wrist before tool offset)."""
    T6 = link_frames(angles)[6]
    return (T6[0][3], T6[1][3], T6[2][3])


def resolve_pose(pose_spec: Optional[dict],
                 position_xyz: Sequence[float],
                 current_angles: Sequence[float]) -> Optional[Tuple[float, float, float]]:
    """Resolve a Pose union to (rx, ry, rz) in degrees, or None to signal
    position-only / preserve.

    Returns None when:
      - pose_spec is None
      - kind == "preserve"  (caller should use position-only or seed orientation)
      - kind == "any"
    """
    if pose_spec is None:
        return None
    kind = pose_spec.get("kind", "preserve")

    if kind in ("preserve", "any"):
        return None

    if kind == "explicit":
        if "euler_xyz" in pose_spec:
            r = pose_spec["euler_xyz"]
            if len(r) != 3: raise ValueError("euler_xyz must be length 3")
            return (float(r[0]), float(r[1]), float(r[2]))
        if "quat" in pose_spec:
            q = pose_spec["quat"]
            if len(q) != 4: raise ValueError("quat must be length 4 [w,x,y,z]")
            R = _quat_to_matrix(*[float(c) for c in q])
            return _matrix_to_rpy(R)
        raise ValueError("explicit pose requires euler_xyz or quat")

    if kind == "align_tool":
        if "approach" not in pose_spec:
            raise ValueError("align_tool requires 'approach'")
        approach = _resolve_axis(pose_spec["approach"])
        # Tool z-axis = -approach (tool approaches from this direction)
        tool_z = (-approach[0], -approach[1], -approach[2])
        R = _matrix_from_tool_z(tool_z)
        roll_deg = float(pose_spec.get("roll_deg", 0.0))
        if abs(roll_deg) > 1e-6:
            R = _rotate_around_axis(R, tool_z, roll_deg)
        return _matrix_to_rpy(R)

    if kind == "extend_toward":
        if "target" not in pose_spec:
            raise ValueError("extend_toward requires 'target' [x,y,z]")
        target = pose_spec["target"]
        if len(target) != 3: raise ValueError("target must be length 3")
        # Tool z-axis = direction from current J6 origin → target
        j6 = _current_j6_origin(current_angles)
        delta = (float(target[0]) - j6[0],
                 float(target[1]) - j6[1],
                 float(target[2]) - j6[2])
        norm = math.sqrt(sum(c*c for c in delta))
        if norm < 5.0:  # too close to compute direction → preserve
            return None
        tool_z = _normalize(delta)
        R = _matrix_from_tool_z(tool_z)
        roll_deg = float(pose_spec.get("roll_deg", 0.0))
        if abs(roll_deg) > 1e-6:
            R = _rotate_around_axis(R, tool_z, roll_deg)
        return _matrix_to_rpy(R)

    raise ValueError(f"unknown pose kind: {kind!r}")
