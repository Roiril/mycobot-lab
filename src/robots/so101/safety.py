"""Safety checks for SO-101 (5-DoF) motion planning.

Standalone (Phase 0): mirrors src/arm/safety.py but parameterized for 5 joints
and consuming so101.profile + so101.kinematics. Returns (ok, msg, bad_joints).

Check status:
  - Joint limits  : EXACT (limits come from the URDF).
  - Floor clearance: structurally exact; FLOOR_Z is a config the user sets once
    the arm is mounted (profile.SAFETY["floor_z_mm"]).
  - Self-collision : PROVISIONAL. The segment-distance math is correct, but the
    clearance thresholds are estimates for the STS3215 arm and are deliberately
    conservative (small) so normal poses are not false-rejected. Re-tune on
    hardware before relying on it to catch real collisions.
"""
from __future__ import annotations
from typing import List, Sequence, Tuple

from . import profile
from .kinematics import joint_positions, end_effector

_FLOOR_Z = profile.SAFETY["floor_z_mm"]
_LINK_RADIUS = profile.SAFETY["link_radius_mm"]
_SELF_CLEARANCE = profile.SAFETY["self_clearance_mm"]
_TOOL_CLEARANCE = profile.SAFETY["tool_clearance_mm"]
_DEGENERATE = profile.SAFETY["degenerate_link_mm"]

# Tool segment is the last link index (5): segments 0..4 are the arm links
# between frames, segment 5 is wrist_roll-frame -> TCP.
_TOOL_LINK = profile.NUM_JOINTS  # == 5


def _seg_dist_sq(a, b, c, d):
    """Squared min distance between segments AB and CD (Ericson 5.1.9)."""
    EPS = 1e-9
    d1 = [b[k] - a[k] for k in range(3)]
    d2 = [d[k] - c[k] for k in range(3)]
    r = [a[k] - c[k] for k in range(3)]
    a_ = sum(x * x for x in d1)
    e_ = sum(x * x for x in d2)
    f = sum(d2[k] * r[k] for k in range(3))
    if a_ <= EPS and e_ <= EPS:
        return sum((a[k] - c[k]) ** 2 for k in range(3))
    if a_ <= EPS:
        s = 0.0; t = max(0.0, min(1.0, f / e_))
    else:
        c_ = sum(d1[k] * r[k] for k in range(3))
        if e_ <= EPS:
            t = 0.0; s = max(0.0, min(1.0, -c_ / a_))
        else:
            b_ = sum(d1[k] * d2[k] for k in range(3))
            denom = a_ * e_ - b_ * b_
            s = max(0.0, min(1.0, (b_ * f - c_ * e_) / denom)) if denom else 0.0
            t = (b_ * s + f) / e_
            if t < 0:
                t = 0.0; s = max(0.0, min(1.0, -c_ / a_))
            elif t > 1:
                t = 1.0; s = max(0.0, min(1.0, (b_ - c_) / a_))
    p = [a[k] + d1[k] * s for k in range(3)]
    q = [c[k] + d2[k] * t for k in range(3)]
    return sum((p[k] - q[k]) ** 2 for k in range(3))


def _seglen(s):
    return sum((s[1][k] - s[0][k]) ** 2 for k in range(3)) ** 0.5


def _check_limits(angles: Sequence[float]) -> Tuple[bool, str, List[int]]:
    if len(angles) != profile.NUM_JOINTS:
        return False, f"angles length != {profile.NUM_JOINTS}", []
    bad = [i for i, (a, (lo, hi)) in enumerate(zip(angles, profile.JOINT_LIMITS), 1)
           if not lo <= a <= hi]
    if bad:
        i = bad[0]
        lo, hi = profile.JOINT_LIMITS[i - 1]
        name = profile.JOINT_NAMES[i - 1]
        return False, f"J{i}({name})={angles[i-1]:.0f} は限界 [{lo:.0f}, {hi:.0f}] 外", bad
    return True, "ok", []


def _check_floor(pts, tip) -> Tuple[bool, str, List[int]]:
    for i, p in enumerate(pts[1:], 1):  # skip base origin
        if (p[2] - _LINK_RADIUS) < _FLOOR_Z:
            margin = p[2] - _LINK_RADIUS - _FLOOR_Z
            return False, f"J{i} が床に近接 (z={p[2]:.0f}, 余裕={margin:.0f}mm)", [i]
    if (tip[2] - _LINK_RADIUS) < _FLOOR_Z:
        margin = tip[2] - _LINK_RADIUS - _FLOOR_Z
        return False, f"ツール先端が床に近接 (z={tip[2]:.0f}, 余裕={margin:.0f}mm)", [_TOOL_LINK]
    return True, "ok", []


def check_angles_floor_only(angles: Sequence[float]) -> Tuple[bool, str, List[int]]:
    """Joint limits + floor only (no self-collision). For VR teleop where the
    user is watching and the only un-recoverable hazard is the floor/table."""
    ok, msg, bad = _check_limits(angles)
    if not ok:
        return ok, msg, bad
    return _check_floor(joint_positions(angles), end_effector(angles))


def check_angles(angles: Sequence[float]) -> Tuple[bool, str, List[int]]:
    """Full validation: joint limits, floor clearance, self-collision.
    Returns (ok, message, bad_joint_indices_1based)."""
    ok, msg, bad = _check_limits(angles)
    if not ok:
        return ok, msg, bad
    pts = joint_positions(angles)
    tip = end_effector(angles)
    ok, msg, bad = _check_floor(pts, tip)
    if not ok:
        return ok, msg, bad

    # links 0..4 between consecutive frames, link 5 = wrist frame -> TCP
    links = [(pts[i], pts[i + 1]) for i in range(profile.NUM_JOINTS)] + [(pts[_TOOL_LINK], tip)]
    n = len(links)
    for i in range(n):
        for j in range(i + 2, n):  # non-adjacent only
            if _seglen(links[i]) < _DEGENERATE or _seglen(links[j]) < _DEGENERATE:
                continue
            clearance = _TOOL_CLEARANCE if j == _TOOL_LINK else _SELF_CLEARANCE
            if _seg_dist_sq(links[i][0], links[i][1], links[j][0], links[j][1]) < clearance * clearance:
                tag = "ツール" if j == _TOOL_LINK else f"link{j}"
                jhi = min(j + 1, profile.NUM_JOINTS)
                return False, f"自己干渉: link{i} と {tag} が近接 (< {clearance:.0f}mm)", [jhi]
    return True, "ok", []


def clamp_angles(angles: Sequence[float]) -> List[float]:
    """Hard-clamp to joint limits (does NOT run check_angles)."""
    return [max(lo, min(hi, a)) for a, (lo, hi) in zip(angles, profile.JOINT_LIMITS)]
