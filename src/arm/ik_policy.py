"""IK solution selection policy.

For a given target XYZ, position-only 6-DoF IK typically admits multiple valid
joint configurations (elbow up/down, wrist flips, shoulder L/R). The default
DLS solver picks whichever happens to converge from the live seed — adjacent
grid points can end up in totally different configurations (flips, twists),
breaking visual continuity and making teleop unpredictable.

This module enforces a coherent posture across the workspace:

  J1 = atan2(y, x)              — base rotates to face the target
  J4 = 0                        — no forearm roll
  J5 = -90                      — flange +z = world +z (camera up)
  J6 = 90                       — camera image upright
  J2, J3 = whatever DLS finds   — shoulder/elbow span the position

Result: arm sweeps the reachable workspace like the user-drawn "petal" pattern.

Public API:
  enumerate_solutions(target_xyz)      → list of (angles, posture_score)
  pick_with_continuity(candidates, prev_angles) → angles for this point
"""
from __future__ import annotations
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .ik_numeric import solve as _dls_solve
from .kinematics import JOINT_LIMITS
from .safety import check_angles

DEG = math.pi / 180.0

# ── posture preferences ────────────────────────────────────────────────────
J4_NEUTRAL = 0.0
J5_NEUTRAL = -90.0   # flange +z = world +z (camera up) — derived empirically
J6_NEUTRAL = 90.0    # camera image upright (matches CAMERA_UPRIGHT_J6_DEG)

# Scoring weights. Higher = stronger preference. Tuned for the "petal" pattern.
W_J1_DIRECTION = 0.5    # |J1 − atan2(y,x)| (deg)
W_J4_ZERO      = 4.0    # J4² — forearm roll is the worst flip offender
W_J5_NEUTRAL   = 2.0    # (J5 − J5_NEUTRAL)²
W_J6_NEUTRAL   = 2.0    # (J6 − J6_NEUTRAL)²
W_ELBOW_UP     = 10.0   # (target_z − elbow_z) mm → DOMINATES other preferences; arm reaches with elbow as high as the geometry allows

# Continuity penalty between adjacent grid points (per joint, deg).
W_CONTINUITY   = 0.3    # added to score when prev_angles is given


def _wrap_to_180(deg: float) -> float:
    """Map angle to (-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0


def posture_score(angles: Sequence[float], target_xyz: Sequence[float]) -> float:
    """Lower is better. Pure cost of how far this joint config is from the
    petal-policy neutral. Does NOT include continuity (caller adds that)."""
    from .kinematics import joint_positions
    j1, _, _, j4, j5, j6 = angles
    j1_pref = math.degrees(math.atan2(target_xyz[1], target_xyz[0]))
    s  = W_J1_DIRECTION * abs(_wrap_to_180(j1 - j1_pref))
    s += W_J4_ZERO      * (j4 - J4_NEUTRAL) ** 2 / 100.0     # normalize: ²/100
    s += W_J5_NEUTRAL   * (j5 - J5_NEUTRAL) ** 2 / 100.0
    s += W_J6_NEUTRAL   * (j6 - J6_NEUTRAL) ** 2 / 100.0
    # Elbow-up preference: reward higher z of the J3 frame origin (the elbow).
    # Tip is at target_z; we reward elbow_z RELATIVE to tip_z so the bias
    # doesn't fight overall workspace altitude.
    elbow_z = joint_positions(angles)[3][2]
    s += W_ELBOW_UP * (target_xyz[2] - elbow_z)
    return s


def continuity_penalty(angles: Sequence[float], prev_angles: Sequence[float]) -> float:
    """Sum of squared joint-angle deltas (deg) between this and previous point.
    Wraps each delta to (-180, 180] to handle ±180 equivalence."""
    s = 0.0
    for a, b in zip(angles, prev_angles):
        d = _wrap_to_180(a - b)
        s += d * d / 100.0
    return s * W_CONTINUITY


def _clamp_to_limits(angles: Sequence[float]) -> List[float]:
    return [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(angles, JOINT_LIMITS)]


def _policy_seeds(target_xyz: Sequence[float]) -> List[List[float]]:
    """Seeds biased toward the petal policy. Order matters: most-preferred first."""
    j1_pref = math.degrees(math.atan2(target_xyz[1], target_xyz[0]))
    j1_alt  = _wrap_to_180(j1_pref + 180.0)  # back-reach variant
    base = [
        # primary: face target, neutral wrist
        [j1_pref, 0,   -90, 0, J5_NEUTRAL, J6_NEUTRAL],
        # elbow flexed variants (helps reach near/far)
        [j1_pref, -30, -60, 0, J5_NEUTRAL, J6_NEUTRAL],
        [j1_pref, 30,  -120, 0, J5_NEUTRAL, J6_NEUTRAL],
        # higher J3 — for low targets
        [j1_pref, -30, -30, 0, J5_NEUTRAL, J6_NEUTRAL],
        # back-reach (J1 flipped) — fallback for far targets behind base
        [j1_alt,  0,   -90, 0, J5_NEUTRAL, J6_NEUTRAL],
        # neutral wrist relaxed: let J5 find its own value
        [j1_pref, 0,   -90, 0, 0,           J6_NEUTRAL],
        # HOME — always include as a final fallback
        [0,       0,   -90, 0, 0,           0],
    ]
    return [_clamp_to_limits(s) for s in base]


def _dedupe_solutions(sols: List[List[float]], tol_deg: float = 5.0) -> List[List[float]]:
    """Drop solutions that are within `tol_deg` per-joint of an earlier one."""
    keep: List[List[float]] = []
    for s in sols:
        dup = False
        for k in keep:
            if all(abs(_wrap_to_180(a - b)) < tol_deg for a, b in zip(s, k)):
                dup = True; break
        if not dup:
            keep.append(s)
    return keep


def enumerate_solutions(target_xyz: Sequence[float],
                        max_iter: int = 60,
                        pos_tol_mm: float = 2.0,
                        ) -> List[Tuple[List[float], float]]:
    """Run DLS IK from each policy seed; return list of (angles, posture_score).

    Filters out unsafe configs (safety.check_angles). Empty list = unreachable.
    """
    seeds = _policy_seeds(target_xyz)
    target = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]),
              0.0, 0.0, 0.0]  # rxyz ignored in position_only
    raw: List[List[float]] = []
    for seed in seeds:
        sol = _dls_solve(target, seed,
                         max_iter=max_iter, pos_tol_mm=pos_tol_mm,
                         position_only=True)
        if sol is None:
            continue
        ok, _, _ = check_angles(sol)
        if not ok:
            continue
        raw.append([float(a) for a in sol])
    unique = _dedupe_solutions(raw)
    return [(s, posture_score(s, target_xyz)) for s in unique]


def pick_best(target_xyz: Sequence[float],
              candidates: List[Tuple[List[float], float]],
              prev_angles: Optional[Sequence[float]] = None,
              ) -> Optional[List[float]]:
    """Choose the best candidate considering posture score + optional continuity
    to the previously-baked neighbor. Returns None if candidates is empty."""
    if not candidates:
        return None
    best = None
    best_score = float("inf")
    for angles, ps in candidates:
        total = ps
        if prev_angles is not None:
            total += continuity_penalty(angles, prev_angles)
        if total < best_score:
            best_score = total; best = angles
    return best


def solve_with_policy(target_xyz: Sequence[float],
                      prev_angles: Optional[Sequence[float]] = None,
                      ) -> Optional[List[float]]:
    """Convenience: enumerate + pick. Used by grid generator."""
    cands = enumerate_solutions(target_xyz)
    return pick_best(target_xyz, cands, prev_angles)
