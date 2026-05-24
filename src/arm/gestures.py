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


def point_at_extending(target_xyz, *, label: Optional[str] = None) -> list:
    """Whole-arm pointing: shoulder lifts, elbow extends, J1 + J5 fine-tuned
    so the tool z-axis points at the target, J6 keeps camera upright.

    Algorithm: fix J2=-35, J3=-45, J4=0 (an obviously extended arm shape).
    Search (J1, J5) over a coarse grid to minimize the angle between the
    achieved tool z-axis and the line (flange → target). Returns the best pose.

    Always safety-clean (the search space is bounded and J2/J3/J4 fixed are safe).
    """
    from .kinematics import link_frames
    tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
    J2, J3, J4 = -35.0, -45.0, 0.0
    # Initial J1 guess: target XY → arm forward. At J1=0 arm extends in -Y, so
    # for world angle α, J1 ≈ α + 90.
    alpha = math.degrees(math.atan2(ty, tx))
    j1_init = max(-165.0, min(165.0, alpha + 90.0))

    def align_error(j1, j5):
        T = link_frames([j1, J2, J3, J4, j5, CAMERA_UPRIGHT_J6_DEG])[-1]
        tz_v = (T[0][2], T[1][2], T[2][2])
        fl = (T[0][3], T[1][3], T[2][3])
        dx, dy, dz_ = tx - fl[0], ty - fl[1], tz - fl[2]
        n = math.sqrt(dx*dx + dy*dy + dz_*dz_)
        if n < 1.0: return 999.0
        dot = (tz_v[0]*dx + tz_v[1]*dy + tz_v[2]*dz_) / n
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))

    # Coarse grid around initial guess, then refine
    best = (align_error(j1_init, 0.0), j1_init, 0.0)
    for j1 in [j1_init + d for d in (-40,-25,-15,-8,-4,0,4,8,15,25,40)]:
        if not -165.0 <= j1 <= 165.0: continue
        for j5 in range(-60, 61, 6):
            e = align_error(j1, j5)
            if e < best[0]: best = (e, j1, float(j5))
    # Finer 2nd pass
    _, j1c, j5c = best
    for j1 in [j1c + d*0.5 for d in range(-8, 9)]:
        if not -165.0 <= j1 <= 165.0: continue
        for j5 in [j5c + d*0.5 for d in range(-8, 9)]:
            if not -60.0 <= j5 <= 60.0: continue
            e = align_error(j1, j5)
            if e < best[0]: best = (e, j1, j5)
    err, j1, j5 = best
    # If best achievable alignment is too poor, refuse rather than silently
    # point off-target. 30° is the practical cutoff — beyond that, the arm
    # is visibly "not pointing at" the thing.
    if err > 30.0:
        raise ValueError(f"point_at: ターゲット({tx:.0f},{ty:.0f},{tz:.0f}) に対し "
                         f"最良 alignment {err:.0f}° (>30°) — 指差し不可能な位置")
    pretty = (f"{label}に手を伸ばして指差し (誤差{err:.0f}°)"
              if label else f"指差し ({tx:.0f},{ty:.0f},{tz:.0f}) 誤差{err:.0f}°")
    return [{
        "label": pretty,
        "angles": [j1, J2, J3, J4, j5, CAMERA_UPRIGHT_J6_DEG],
        "speed": 25, "pause_s": 0.8,
    }]


def _point_at_compact(target_xyz, *, j5_extend_deg: float = 0.0, label: Optional[str] = None) -> list:
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
    pretty = f"{label}を指差し" if label else f"指差し ({tx:.0f},{ty:.0f},{tz:.0f})"
    return [{
        "label": pretty,
        "angles": [j1, -20.0, -80.0, 0.0, j5, CAMERA_UPRIGHT_J6_DEG],
        "speed": 25, "pause_s": 0.8,
    }]


def go_home() -> list:
    return [{"label": "home", "angles": [0, 0, -90, 0, 0, 0], "speed": 30}]


def nod(direction, *, times: int = 2) -> list:
    """Yes-nod: small repeated bow. Faster + smaller than bow()."""
    j1 = _dir_to_j1(direction)
    j6 = CAMERA_UPRIGHT_J6_DEG
    out = [{"label": "nod_face", "angles": [j1, 0, -90, 0, 0, j6], "speed": 35, "pause_s": 0.1}]
    for k in range(max(1, min(5, times))):
        out.append({"label": f"nod_down_{k}",   "angles": [j1, -12, -85, 0, 8, j6], "speed": 35, "pause_s": 0.15})
        out.append({"label": f"nod_return_{k}", "angles": [j1, 0,   -90, 0, 0, j6], "speed": 35, "pause_s": 0.1})
    return out


def wave(direction, *, times: int = 3) -> list:
    """Greeting wave: arm raised, J6 swings left-right. Quick + friendly."""
    j1 = _dir_to_j1(direction)
    j6_left  = CAMERA_UPRIGHT_J6_DEG - 30
    j6_right = CAMERA_UPRIGHT_J6_DEG + 30
    out = [
        {"label": "wave_raise", "angles": [j1, -30, -45, 0, -45, CAMERA_UPRIGHT_J6_DEG], "speed": 35, "pause_s": 0.15},
    ]
    for k in range(max(1, min(5, times))):
        out.append({"label": f"wave_R_{k}", "angles": [j1, -30, -45, 0, -45, j6_right], "speed": 40, "pause_s": 0.1})
        out.append({"label": f"wave_L_{k}", "angles": [j1, -30, -45, 0, -45, j6_left ], "speed": 40, "pause_s": 0.1})
    out.append({"label": "wave_center", "angles": [j1, -30, -45, 0, -45, CAMERA_UPRIGHT_J6_DEG], "speed": 35, "pause_s": 0.1})
    return out


def build(spec: dict) -> list:
    """Build a step sequence from a high-level spec.

    spec = {"kind": "face"|"bow"|"nod"|"wave"|"point_at"|"home", ...params}
    Returns list of move steps (for /move_sequence).
    """
    kind = spec.get("kind")
    if kind == "face":
        return face(spec.get("direction", "back"), with_camera_upright=spec.get("upright", True))
    if kind == "bow":
        return bow(spec.get("direction", "front"),
                   depth_deg=spec.get("depth_deg", 25.0),
                   hold_s=spec.get("hold_s", 0.5))
    if kind == "nod":
        return nod(spec.get("direction", "front"), times=spec.get("times", 2))
    if kind == "wave":
        return wave(spec.get("direction", "front"), times=spec.get("times", 3))
    if kind == "point_at":
        # Always use the IK-quality extending pose; compact fallback removed.
        # Caller (server) routes point_at to point_at_extending directly; build()
        # is only reached if caller bypassed that — re-route here for safety.
        return point_at_extending(spec["target_xyz"], label=spec.get("label"))
    if kind == "home":
        return go_home()
    raise ValueError(f"unknown gesture kind: {kind}")
