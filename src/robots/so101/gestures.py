"""Parameterized gestures for SO-101 (5-DoF).

Each gesture returns a list of joint-space waypoints (deg, canonical order),
clamped to joint limits. Feed them to So101Controller.move_to_angles in
sequence; every waypoint is validated by safety.check_angles there (and in
tests). Amplitudes are kept conservative so waypoints stay limit- and
floor-safe around the neutral poses.

Joint index map: 0 shoulder_pan, 1 shoulder_lift, 2 elbow_flex,
3 wrist_flex, 4 wrist_roll.
"""
from __future__ import annotations
from typing import List, Optional, Sequence

from . import profile, poses

PAN, LIFT, ELBOW, WFLEX, WROLL = 0, 1, 2, 3, 4


def _clamp(a: Sequence[float]) -> List[float]:
    return [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(a, profile.JOINT_LIMITS)]


def _osc(base: Sequence[float], joint: int, amp: float, times: int) -> List[List[float]]:
    """Oscillate one joint +amp/-amp around base for `times` cycles, returning
    to base. Waypoints are clamped."""
    seq: List[List[float]] = []
    for _ in range(max(1, times)):
        for off in (amp, -amp, 0.0):
            w = list(base)
            w[joint] = base[joint] + off
            seq.append(_clamp(w))
    return seq


def face(azimuth_deg: float, base: Optional[Sequence[float]] = None) -> List[float]:
    """Turn the base (shoulder_pan) toward an azimuth, keeping the rest of the
    arm at `base` (default READY). Returns a single target pose."""
    b = list(base) if base is not None else list(poses.READY)
    b[PAN] = azimuth_deg
    return _clamp(b)


def point(azimuth_deg: float) -> List[float]:
    """Point toward an azimuth: turn the base and extend the arm forward/low."""
    b = list(poses.FORWARD_LOW)
    b[PAN] = azimuth_deg
    return _clamp(b)


def nod(base: Optional[Sequence[float]] = None, times: int = 2, amp: float = 15.0) -> List[List[float]]:
    """Nod 'yes' by tilting the wrist (wrist_flex) up/down."""
    b = list(base) if base is not None else list(poses.READY)
    return _osc(b, WFLEX, amp, times)


def shake(base: Optional[Sequence[float]] = None, times: int = 2, amp: float = 25.0) -> List[List[float]]:
    """Shake 'no' by rotating the base (shoulder_pan) side to side."""
    b = list(base) if base is not None else list(poses.READY)
    return _osc(b, PAN, amp, times)


def wave(times: int = 3, amp: float = 30.0) -> List[List[float]]:
    """Greeting wave: raise the arm, then oscillate the wrist roll."""
    raised = list(poses.UP)
    seq: List[List[float]] = [_clamp(raised)]
    seq += _osc(raised, WROLL, amp, times)
    return seq


def bow(base: Optional[Sequence[float]] = None, amp: float = 25.0) -> List[List[float]]:
    """Bow by leaning forward at the shoulder, then returning."""
    b = list(base) if base is not None else list(poses.READY)
    leaned = list(b)
    leaned[LIFT] = b[LIFT] + amp
    return [_clamp(b), _clamp(leaned), _clamp(b)]
