"""Joint-space gesture primitives for human-arm communication.

Each gesture returns a list of pose steps (dicts) suitable for /move_sequence.
Hand-tuned on real hardware; designed to be:
  - safety-passing in the default state space (no self-collision)
  - readable as motion (clear physical meaning to a human observer)
  - quick (~3-6s total)

Gestures take a direction or a remembered-object reference and produce angle
sequences. The actual safety check / execution happens server-side.
"""
from __future__ import annotations
import math
from typing import Optional

from .constants import CAMERA_UPRIGHT_J6_DEG


def _dir_to_j1(direction) -> float:
    """Map direction → J1 degrees, with safety margin (±165° not ±168°)."""
    if isinstance(direction, (int, float)):
        return max(-165.0, min(165.0, float(direction)))
    dirmap = {"back": 0.0, "right": 90.0, "front": 165.0, "left": -90.0}
    return dirmap.get(str(direction).lower(), 0.0)


def face(direction, *, with_camera_upright: bool = True) -> list:
    """Turn the arm to face a direction (camera horizontal). One step."""
    j1 = _dir_to_j1(direction)
    j6 = CAMERA_UPRIGHT_J6_DEG if with_camera_upright else 0.0
    return [{"label": f"face_{direction}", "angles": [j1, 0, -90, 0, 0, j6], "speed": 30}]


def bow(direction, *, depth_deg: float = 25.0, hold_s: float = 0.5) -> list:
    """Bow in the given direction. Three steps: face → lean → return."""
    j1 = _dir_to_j1(direction)
    j6 = CAMERA_UPRIGHT_J6_DEG
    depth = max(10.0, min(40.0, depth_deg))
    return [
        {"label": "bow_face",   "angles": [j1, 0,         -90, 0, 0,             j6], "speed": 30, "pause_s": 0.2},
        {"label": "bow_down",   "angles": [j1, -depth,    -(90-depth*0.6), 0, depth*0.6, j6], "speed": 30, "pause_s": hold_s},
        {"label": "bow_return", "angles": [j1, 0,         -90, 0, 0,             j6], "speed": 30, "pause_s": 0.1},
    ]


def point_at(target_xyz, *, j5_extend_deg: float = 0.0) -> list:
    """Point the wrist toward a target XYZ in base coords.

    Doesn't extend (target may be out of reach). Computes J1 from target X,Y and
    J5 from target height vs camera height to tilt the wrist up/down.

    j5_extend_deg: extra J5 rotation to "emphasize" the gesture.
    """
    tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
    # J1 from XY angle. Camera at J1=0 faces -Y, so:
    #   world_angle (from +X CCW) = J1 + 270
    # → J1 = atan2(ty, tx) - 270 (modulo)
    world_angle_deg = math.degrees(math.atan2(ty, tx))
    j1 = world_angle_deg - 270.0
    while j1 > 180: j1 -= 360
    while j1 < -180: j1 += 360
    j1 = max(-165.0, min(165.0, j1))

    # J5 from target height. Camera at observe pose is at z~310mm. Tilt up if
    # target higher, down if lower. Distance for angle calc uses horizontal reach.
    horiz = math.sqrt(tx*tx + ty*ty)
    dz = tz - 310.0
    # tilt_deg positive = wrist up. Tilt = atan2(dz, horiz) approximately maps to
    # J5 angle (which is the wrist pitch).
    tilt_rad = math.atan2(dz, max(100.0, horiz))
    j5 = math.degrees(tilt_rad) + j5_extend_deg
    j5 = max(-90.0, min(90.0, j5))

    # Use a pointing pose: arm at HOME-style fold with J5 tilted, J6 upright
    return [{
        "label": f"point_at_({tx:.0f},{ty:.0f},{tz:.0f})",
        "angles": [j1, -20.0, -80.0, 0.0, j5, CAMERA_UPRIGHT_J6_DEG],
        "speed": 25, "pause_s": 0.8,
    }]


def go_home() -> list:
    return [{"label": "home", "angles": [0, 0, -90, 0, 0, 0], "speed": 30}]


def build(spec: dict) -> list:
    """Build a step sequence from a high-level spec.

    spec = {"kind": "face"|"bow"|"point_at"|"home", ...params}
    Returns list of move steps (for /move_sequence).
    """
    kind = spec.get("kind")
    if kind == "face":
        return face(spec.get("direction", "back"), with_camera_upright=spec.get("upright", True))
    if kind == "bow":
        return bow(spec.get("direction", "front"),
                   depth_deg=spec.get("depth_deg", 25.0),
                   hold_s=spec.get("hold_s", 0.5))
    if kind == "point_at":
        return point_at(spec["target_xyz"], j5_extend_deg=spec.get("j5_extend_deg", 0.0))
    if kind == "home":
        return go_home()
    raise ValueError(f"unknown gesture kind: {kind}")
