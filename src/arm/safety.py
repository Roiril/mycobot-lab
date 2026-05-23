"""Safety checks for myCobot 320 motion planning.

All checks operate on joint angles (deg). Floor and self-collision are evaluated
via forward kinematics. Returns (ok, msg, badJoints) so the UI can highlight
which joint(s) caused the rejection.
"""
from __future__ import annotations
from typing import Sequence, Tuple, List
from .kinematics import JOINT_LIMITS, joint_positions, end_effector
from .constants import FLOOR_Z, LINK_RADIUS, SELF_CLEARANCE, DEGENERATE_LINK_MM


def _seg_dist_sq(a, b, c, d):
    """Squared min distance between segments AB and CD (Ericson §5.1.9)."""
    EPS = 1e-9
    d1 = [b[k] - a[k] for k in range(3)]
    d2 = [d[k] - c[k] for k in range(3)]
    r  = [a[k] - c[k] for k in range(3)]
    a_  = sum(x*x for x in d1)
    e_  = sum(x*x for x in d2)
    f   = sum(d2[k]*r[k] for k in range(3))
    if a_ <= EPS and e_ <= EPS:
        return sum((a[k]-c[k])**2 for k in range(3))
    if a_ <= EPS:
        s = 0.0; t = max(0.0, min(1.0, f/e_))
    else:
        c_ = sum(d1[k]*r[k] for k in range(3))
        if e_ <= EPS:
            t = 0.0; s = max(0.0, min(1.0, -c_/a_))
        else:
            b_ = sum(d1[k]*d2[k] for k in range(3))
            denom = a_*e_ - b_*b_
            s = max(0.0, min(1.0, (b_*f - c_*e_)/denom)) if denom else 0.0
            t = (b_*s + f) / e_
            if t < 0:
                t = 0.0; s = max(0.0, min(1.0, -c_/a_))
            elif t > 1:
                t = 1.0; s = max(0.0, min(1.0, (b_ - c_)/a_))
    p = [a[k] + d1[k]*s for k in range(3)]
    q = [c[k] + d2[k]*t for k in range(3)]
    return sum((p[k]-q[k])**2 for k in range(3))


def check_angles(angles: Sequence[float]) -> Tuple[bool, str, List[int]]:
    """Validate joint angles. Returns (ok, message, bad_joint_indices_1based)."""
    if len(angles) != 6:
        return False, "angles length != 6", []
    bad: List[int] = []
    # joint limits
    for i, (a, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS), 1):
        if not lo <= a <= hi:
            bad.append(i)
    if bad:
        i = bad[0]
        lo, hi = JOINT_LIMITS[i-1]
        return False, f"J{i}={angles[i-1]:.0f}° は限界 [{lo:.0f}, {hi:.0f}] 外", bad
    # FK
    pts = joint_positions(angles)
    tip = end_effector(angles)
    # floor: skip J0 (=base origin)
    for i, p in enumerate(pts[1:], 1):
        if (p[2] - LINK_RADIUS) < FLOOR_Z:
            margin = p[2] - LINK_RADIUS - FLOOR_Z
            return False, f"J{i} が床に近接 (z={p[2]:.0f}, 余裕={margin:.0f}mm)", [i]
    if (tip[2] - LINK_RADIUS) < FLOOR_Z:
        margin = tip[2] - LINK_RADIUS - FLOOR_Z
        return False, f"ツール先端が床に近接 (z={tip[2]:.0f}, 余裕={margin:.0f}mm)", [6]
    # self-collision: include tool as link 6
    links = [(pts[i], pts[i+1]) for i in range(6)] + [(pts[6], tip)]
    def seglen(s): return ((s[1][0]-s[0][0])**2 + (s[1][1]-s[0][1])**2 + (s[1][2]-s[0][2])**2)**0.5
    pairs = [
        (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 3), (1, 4), (1, 5), (1, 6),
        (2, 4), (2, 5), (2, 6),
        (3, 5), (3, 6),
    ]
    for i, j in pairs:
        if seglen(links[i]) < DEGENERATE_LINK_MM or seglen(links[j]) < DEGENERATE_LINK_MM:
            continue
        d2 = _seg_dist_sq(links[i][0], links[i][1], links[j][0], links[j][1])
        if d2 < SELF_CLEARANCE * SELF_CLEARANCE:
            tag = "ツール" if j == 6 else f"link{j}"
            # blame the higher-indexed joints (further from base, more controllable)
            jhi = j + 1 if j < 6 else 6
            return False, f"自己干渉: link{i} と {tag} が近接 ({d2**0.5:.0f}mm < {SELF_CLEARANCE:.0f})", [jhi]
    return True, "ok", []


def clamp_angles(angles: Sequence[float]) -> list[float]:
    """Hard-clamp to joint limits. Returns clamped list (does NOT run check_angles)."""
    return [max(lo, min(hi, a)) for a, (lo, hi) in zip(angles, JOINT_LIMITS)]
