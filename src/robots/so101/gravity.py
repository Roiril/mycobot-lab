"""Static gravity torque of the SO-101 arm, per joint (pure, stdlib only).

Why this exists
---------------
Each STS3215 is a position servo: it does not know how much of its effort goes
into holding the arm up and how much into pushing against something. The host
does, if it knows the arm's mass distribution. With a gravity model the follower
controller can tell three situations apart that look identical on a load gauge:

- the arm is simply extended (load == what gravity needs),
- the arm carries something (load > gravity, but the joint reaches its goal),
- the arm is pushing on the table, itself, or an end stop (load > gravity AND
  the joint is stuck short of its goal).

Only the last one is worth yielding for, and telling it apart needs this number.

Source of the numbers
---------------------
Masses and centres of mass are read from the vendored MuJoCo model
(sim/so101_new_calib.xml, TheRobotStudio/SO-ARM100, onshape-to-robot export)
at import time instead of being copied by hand. The joint frames come from
profile.URDF_LINKS via kinematics.link_frames; the MJCF body frames coincide
with those link frames (same CAD export), which tests/test_so101_gravity.py
checks against MuJoCo's own qfrc_bias.

Each body's mass includes its STS3215 (~55 g). The CAD masses ignore cables and
screws, so treat the result as a floor, not an exact value; the follower
controller adds margin and learns a per-joint bias online.

Sign convention
---------------
`holding_torques` returns the torque the joint actuator must apply to hold the
pose against gravity (MuJoCo's qfrc_bias at zero velocity), in N*m, in the
URDF joint-angle convention (the same angles the rest of src/robots/so101 uses).
"""
from __future__ import annotations

import functools
import math
import pathlib
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

from . import kinematics, profile

G = 9.81  # m/s^2, matches the MJCF default gravity
_MJCF = pathlib.Path(__file__).with_name("sim") / "so101_new_calib.xml"

# MJCF body that each kinematic frame T1..T5 corresponds to (link_frames order).
_BODY_OF_FRAME = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper"]
_JAW_BODY = "moving_jaw_so101_v1"

Vec = Tuple[float, float, float]


def _quat_to_mat(w: float, x: float, y: float, z: float) -> List[List[float]]:
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _floats(s: Optional[str], n: int, default: float = 0.0) -> List[float]:
    if not s:
        return [default] * n
    vals = [float(v) for v in s.split()]
    if len(vals) != n:
        raise ValueError(f"expected {n} numbers, got {s!r}")
    return vals


@functools.lru_cache(maxsize=1)
def link_inertials() -> Dict[str, dict]:
    """{body: {"mass": kg, "com": (x, y, z) m in body frame, "pos"/"quat" of
    the body in its parent}} parsed from the vendored MJCF."""
    root = ET.parse(_MJCF).getroot()
    out: Dict[str, dict] = {}
    for body in root.iter("body"):
        name = body.get("name")
        inertial = body.find("inertial")
        if name is None or inertial is None:
            continue
        out[name] = {
            "mass": float(inertial.get("mass")),
            "com": tuple(_floats(inertial.get("pos"), 3)),
            "pos": tuple(_floats(body.get("pos"), 3)),
            "quat": tuple(_floats(body.get("quat"), 4, 0.0)) if body.get("quat") else (1.0, 0.0, 0.0, 0.0),
        }
    missing = [b for b in _BODY_OF_FRAME + [_JAW_BODY] if b not in out]
    if missing:
        raise RuntimeError(f"MJCF is missing bodies {missing}: {_MJCF}")
    return out


def _apply(T, p: Vec) -> Vec:
    return (T[0][0] * p[0] + T[0][1] * p[1] + T[0][2] * p[2] + T[0][3],
            T[1][0] * p[0] + T[1][1] * p[1] + T[1][2] * p[2] + T[1][3],
            T[2][0] * p[0] + T[2][1] * p[1] + T[2][2] * p[2] + T[2][3])


def _mm_to_m_frame(T):
    """kinematics works in mm; rescale the translation to metres."""
    return [[T[0][0], T[0][1], T[0][2], T[0][3] / 1000.0],
            [T[1][0], T[1][1], T[1][2], T[1][3] / 1000.0],
            [T[2][0], T[2][1], T[2][2], T[2][3] / 1000.0],
            [0.0, 0.0, 0.0, 1.0]]


def _jaw_frame(T_gripper, jaw_rad: float):
    """World frame of the moving jaw: fixed MJCF offset, then rotation about z."""
    j = link_inertials()[_JAW_BODY]
    R = _quat_to_mat(*j["quat"])
    c, s = math.cos(jaw_rad), math.sin(jaw_rad)
    Rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    Rl = [[sum(R[i][k] * Rz[k][jj] for k in range(3)) for jj in range(3)] for i in range(3)]
    local = [[Rl[0][0], Rl[0][1], Rl[0][2], j["pos"][0]],
             [Rl[1][0], Rl[1][1], Rl[1][2], j["pos"][1]],
             [Rl[2][0], Rl[2][1], Rl[2][2], j["pos"][2]],
             [0.0, 0.0, 0.0, 1.0]]
    return [[sum(T_gripper[i][k] * local[k][jj] for k in range(4)) for jj in range(4)] for i in range(4)]


def holding_torques(angles_deg: Sequence[float], *, jaw_deg: float = 0.0,
                    payload_kg: float = 0.0) -> List[float]:
    """Torque (N*m) each of the 5 arm joints must apply to hold `angles_deg`
    still against gravity. `payload_kg` is a point mass at the gripper TCP
    (profile.TOOL_TRANSFORM) for what-if analysis.
    """
    if len(angles_deg) != profile.NUM_JOINTS:
        raise ValueError(f"angles must be length {profile.NUM_JOINTS}")
    frames = [_mm_to_m_frame(T) for T in kinematics.link_frames(angles_deg)]
    inert = link_inertials()

    # point masses (world position, mass) and the first joint index they hang on
    masses: List[Tuple[Vec, float, int]] = []
    for i, body in enumerate(_BODY_OF_FRAME):
        b = inert[body]
        masses.append((_apply(frames[i + 1], b["com"]), b["mass"], i))
    jaw_T = _jaw_frame(frames[5], math.radians(jaw_deg))
    jb = inert[_JAW_BODY]
    masses.append((_apply(jaw_T, jb["com"]), jb["mass"], 4))
    if payload_kg > 0.0:
        tcp = _mm_to_m_frame(kinematics.tool_frame(angles_deg))
        masses.append(((tcp[0][3], tcp[1][3], tcp[2][3]), float(payload_kg), 4))

    taus: List[float] = []
    for j in range(profile.NUM_JOINTS):
        Tj = frames[j + 1]
        p = (Tj[0][3], Tj[1][3], Tj[2][3])
        z = (Tj[0][2], Tj[1][2], Tj[2][2])
        tau = 0.0
        for c, m, first in masses:
            if first < j:
                continue
            r = (c[0] - p[0], c[1] - p[1], c[2] - p[2])
            # generalized gravity force on joint j = z . (r x F), F = (0, 0, -m g)
            # -> (r x F) = (-r_y * m g, r_x * m g, 0)
            gen = z[0] * (-r[1] * m * G) + z[1] * (r[0] * m * G)
            tau -= gen          # actuator must cancel it
        taus.append(tau)
    return taus


def ticks_to_deg(ticks: int, rng: Tuple[int, int], resolution: int = 4096) -> float:
    """Homed servo ticks -> URDF joint angle (deg), lerobot DEGREES semantics:
    deg = (ticks - mid) * 360 / (resolution - 1), mid = calibrated range centre."""
    mid = (rng[0] + rng[1]) / 2.0
    return (float(ticks) - mid) * 360.0 / (resolution - 1)


def pose_from_ticks(ticks: Dict[str, int], ranges: Dict[str, Tuple[int, int]]) -> List[float]:
    """Arm joint angles (deg) from a {joint: homed ticks} dict, canonical order."""
    out = []
    for n in profile.JOINT_NAMES:
        a = ticks_to_deg(ticks[n], ranges.get(n, (0, 4095)))
        out.append(((a + 180.0) % 360.0) - 180.0)   # wrist_roll may be multi-turn
    return out
