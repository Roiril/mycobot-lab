"""GPU-batched IK for the reach-grid generator.

Ports the CPU URDF FK + DLS-IK + policy-scoring pipeline to PyTorch so 10k+
grid points can be solved in parallel on the RTX 4090. Falls back gracefully
if CUDA is unavailable (still works on CPU, just slower).

Public API:
  solve_grid_baked(targets: list[(x,y,z)]) → list[Optional[list[6 floats]]]

Mirrors ik_policy.solve_with_policy semantics but batched.
"""
from __future__ import annotations
import math
from typing import List, Optional, Sequence

import numpy as np
import torch

from .kinematics import URDF_LINKS, JOINT_LIMITS
from .constants import TOOL_LENGTH
from .ik_policy import (
    J4_NEUTRAL, J5_NEUTRAL, J6_NEUTRAL,
    W_J1_DIRECTION, W_J4_ZERO, W_J5_NEUTRAL, W_J6_NEUTRAL, W_ELBOW_UP,
    POLICY_SEED_TEMPLATES,
)
from .safety import check_angles

DEG = math.pi / 180.0
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DTYPE = torch.float64  # FP32 fails DLS convergence for 2mm tol; FP64 is robust and still GPU-fast enough


# ── precompute URDF parent→child transforms as constant tensors ────────────
def _build_origin_tfs() -> torch.Tensor:
    """Return [6, 4, 4] tensor: the fixed parent→child URDF transforms."""
    Ts = []
    for xyz, rpy in URDF_LINKS:
        cx, sx = math.cos(rpy[0]), math.sin(rpy[0])
        cy, sy = math.cos(rpy[1]), math.sin(rpy[1])
        cz, sz = math.cos(rpy[2]), math.sin(rpy[2])
        T = torch.tensor([
            [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz, xyz[0]],
            [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz, xyz[1]],
            [-sy,                sx*cy,            cx*cy, xyz[2]],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=DTYPE, device=DEVICE)
        Ts.append(T)
    return torch.stack(Ts)  # [6, 4, 4]


_ORIGIN_TFS = None
_LIMITS_LO = None
_LIMITS_HI = None


def _lazy_init():
    global _ORIGIN_TFS, _LIMITS_LO, _LIMITS_HI
    if _ORIGIN_TFS is None:
        _ORIGIN_TFS = _build_origin_tfs()
        lo = torch.tensor([l for l, _ in JOINT_LIMITS], dtype=DTYPE, device=DEVICE)
        hi = torch.tensor([h for _, h in JOINT_LIMITS], dtype=DTYPE, device=DEVICE)
        _LIMITS_LO, _LIMITS_HI = lo, hi


def _rotz_batch(angles_deg: torch.Tensor) -> torch.Tensor:
    """angles_deg [B, 6] → [B, 6, 4, 4] rotation-about-z matrices per joint."""
    B = angles_deg.shape[0]
    th = angles_deg * DEG
    c, s = torch.cos(th), torch.sin(th)  # [B, 6]
    R = torch.zeros(B, 6, 4, 4, dtype=DTYPE, device=DEVICE)
    R[:, :, 0, 0] = c;  R[:, :, 0, 1] = -s
    R[:, :, 1, 0] = s;  R[:, :, 1, 1] = c
    R[:, :, 2, 2] = 1
    R[:, :, 3, 3] = 1
    return R


def fk_batch(angles_deg: torch.Tensor) -> torch.Tensor:
    """Batched URDF FK.

    angles_deg: [B, 6]
    returns: [B, 7, 4, 4] cumulative frames T0..T6 (T6 is the flange).
    """
    _lazy_init()
    B = angles_deg.shape[0]
    rotz = _rotz_batch(angles_deg)              # [B, 6, 4, 4]
    # step[i] = ORIGIN_TFS[i] @ rotz[:, i]      # [B, 4, 4]
    steps = torch.einsum("ijk,bikl->bijl", _ORIGIN_TFS, rotz)  # [B, 6, 4, 4]
    # Cumulative product along joint axis. Python loop ×6 is fine (tiny).
    frames = [torch.eye(4, dtype=DTYPE, device=DEVICE).expand(B, 4, 4).contiguous()]
    for i in range(6):
        frames.append(torch.bmm(frames[-1], steps[:, i]))
    return torch.stack(frames, dim=1)            # [B, 7, 4, 4]


def tip_batch(angles_deg: torch.Tensor) -> torch.Tensor:
    """Tool tip position [B, 3]. tip = flange.origin + TOOL_LENGTH * flange.z_axis."""
    T = fk_batch(angles_deg)[:, 6]               # [B, 4, 4]
    origin = T[:, :3, 3]                          # [B, 3]
    z_axis = T[:, :3, 2]                          # [B, 3]
    return origin + TOOL_LENGTH * z_axis


def jacobian_batch(angles_deg: torch.Tensor, eps: float = 1e-3) -> torch.Tensor:
    """Numerical Jacobian of tip position wrt joint angles via central differences.
    angles: [B, 6] → [B, 3, 6]
    """
    B = angles_deg.shape[0]
    J = torch.empty(B, 3, 6, dtype=DTYPE, device=DEVICE)
    for i in range(6):
        ap = angles_deg.clone(); ap[:, i] += eps
        am = angles_deg.clone(); am[:, i] -= eps
        J[:, :, i] = (tip_batch(ap) - tip_batch(am)) / (2 * eps)
    return J


def dls_step(angles_deg: torch.Tensor,
             targets: torch.Tensor,
             damping: torch.Tensor,
             max_step_deg: float = 5.0) -> torch.Tensor:
    """One Levenberg-Marquardt iteration. Returns updated angles [B, 6]."""
    tip = tip_batch(angles_deg)                  # [B, 3]
    err = targets - tip                           # [B, 3]
    J = jacobian_batch(angles_deg)                # [B, 3, 6]
    # dq = J^T (J J^T + λ² I)⁻¹ err
    JJt = torch.bmm(J, J.transpose(1, 2))         # [B, 3, 3]
    I3 = torch.eye(3, dtype=DTYPE, device=DEVICE).expand(JJt.shape[0], 3, 3)
    A = JJt + (damping ** 2).view(-1, 1, 1) * I3
    sol = torch.linalg.solve(A, err.unsqueeze(-1))  # [B, 3, 1]
    dq = torch.bmm(J.transpose(1, 2), sol).squeeze(-1)  # [B, 6]
    # Cap per-iteration step
    nrm = torch.linalg.norm(dq, dim=1, keepdim=True)
    scale = torch.where(nrm > max_step_deg, max_step_deg / (nrm + 1e-9), torch.ones_like(nrm))
    dq = dq * scale
    new_angles = angles_deg + dq
    # Clamp to joint limits
    new_angles = torch.maximum(new_angles, _LIMITS_LO)
    new_angles = torch.minimum(new_angles, _LIMITS_HI)
    return new_angles


def solve_batch(targets: torch.Tensor,
                seeds: torch.Tensor,
                max_iter: int = 80,
                pos_tol_mm: float = 2.0,
                ) -> tuple[torch.Tensor, torch.Tensor]:
    """Position-only DLS IK, batched.

    targets: [B, 3] in mm
    seeds:   [B, 6] in deg
    Returns: (final_angles [B, 6], success_mask [B] bool)
    """
    _lazy_init()
    q = seeds.clone()
    damping = torch.full((q.shape[0],), 1.0, dtype=DTYPE, device=DEVICE)
    last_err = torch.full((q.shape[0],), float("inf"), dtype=DTYPE, device=DEVICE)
    best_q = q.clone()
    best_err = torch.full((q.shape[0],), float("inf"), dtype=DTYPE, device=DEVICE)

    for _ in range(max_iter):
        tip = tip_batch(q)
        err_norm = torch.linalg.norm(targets - tip, dim=1)
        improved = err_norm < best_err
        best_err = torch.where(improved, err_norm, best_err)
        best_q = torch.where(improved.unsqueeze(1), q, best_q)
        # Adaptive damping per element
        better = err_norm < last_err
        damping = torch.where(better, torch.clamp(damping * 0.7, min=0.1),
                              torch.clamp(damping * 2.0, max=500.0))
        last_err = torch.minimum(last_err, err_norm)
        q = dls_step(q, targets, damping)
    final_err = torch.linalg.norm(targets - tip_batch(q), dim=1)
    # Use whichever is better between best_q and final q
    use_best = best_err < final_err
    final_q = torch.where(use_best.unsqueeze(1), best_q, q)
    final_err_norm = torch.minimum(best_err, final_err)
    success = final_err_norm < pos_tol_mm
    return final_q, success


# ── policy seeds — built from ik_policy.POLICY_SEED_TEMPLATES (single source) ──
def _policy_seeds_batch(targets: torch.Tensor) -> torch.Tensor:
    """For each target [B, 3], return [B, S, 6] policy-biased seeds.
    Uses the SAME templates as the CPU solver so both stay in lock-step."""
    _lazy_init()
    B = targets.shape[0]
    j1_pref = torch.atan2(targets[:, 1], targets[:, 0]) / DEG          # [B]
    j1_alt  = ((j1_pref + 180.0 + 180.0) % 360.0) - 180.0
    j1_zero = torch.zeros(B, dtype=DTYPE, device=DEVICE)
    j1_by_kind = {'pref': j1_pref, 'alt': j1_alt, 'zero': j1_zero}
    def _const(v): return torch.full((B,), float(v), dtype=DTYPE, device=DEVICE)
    rows = []
    for kind, j2, j3, j4, j5, j6 in POLICY_SEED_TEMPLATES:
        j1 = j1_by_kind[kind]
        rows.append(torch.stack([j1, _const(j2), _const(j3), _const(j4), _const(j5), _const(j6)], dim=1))
    seeds = torch.stack(rows, dim=1)  # [B, S, 6]
    seeds = torch.maximum(seeds, _LIMITS_LO)
    seeds = torch.minimum(seeds, _LIMITS_HI)
    return seeds


def _posture_score_batch(angles: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
    """Batched posture cost matching ik_policy.posture_score (lower is better)."""
    j1, _, _, j4, j5, j6 = angles.unbind(dim=1)
    j1_pref = torch.atan2(targets[:, 1], targets[:, 0]) / DEG
    d1 = ((j1 - j1_pref + 180.0) % 360.0) - 180.0
    s  = W_J1_DIRECTION * torch.abs(d1)
    s += W_J4_ZERO     * (j4 - J4_NEUTRAL) ** 2 / 100.0
    s += W_J5_NEUTRAL  * (j5 - J5_NEUTRAL) ** 2 / 100.0
    s += W_J6_NEUTRAL  * (j6 - J6_NEUTRAL) ** 2 / 100.0
    # Elbow-up: penalize (target_z - elbow_z). Higher elbow → lower score.
    frames = fk_batch(angles)                 # [B, 7, 4, 4]
    elbow_z = frames[:, 3, 2, 3]              # J3 frame origin z
    s += W_ELBOW_UP * (targets[:, 2] - elbow_z)
    return s


def solve_grid(targets_np: np.ndarray,
               max_iter: int = 80,
               pos_tol_mm: float = 2.0,
               ) -> List[Optional[List[float]]]:
    """Solve the grid in one parallel GPU pass.

    For each of N targets, runs S=7 policy seeds, picks the lowest-posture
    successful solution. Returns list of [6 floats] or None per target.

    Safety check still runs on CPU (small per-pose cost; cheap relative to IK).
    """
    _lazy_init()
    N = len(targets_np)
    targets = torch.as_tensor(targets_np, dtype=DTYPE, device=DEVICE)  # [N, 3]
    seeds = _policy_seeds_batch(targets)                # [N, S, 6]
    S = seeds.shape[1]
    # Flatten: treat each (target, seed) pair as an independent IK problem
    flat_targets = targets.unsqueeze(1).expand(-1, S, -1).reshape(N * S, 3)
    flat_seeds   = seeds.reshape(N * S, 6)
    final_q, success = solve_batch(flat_targets, flat_seeds, max_iter=max_iter,
                                   pos_tol_mm=pos_tol_mm)
    # Score each. inf = IK didn't converge for this (target, seed).
    scores = _posture_score_batch(final_q, flat_targets)
    scores = torch.where(success, scores, torch.full_like(scores, float("inf")))
    final_q  = final_q.reshape(N, S, 6)                  # [N, S, 6]
    scores   = scores.reshape(N, S)                       # [N, S]
    # Rank seeds by score ascending; iterate through ranked list per target
    # picking the first solution that passes safety (which can't be vectorized
    # cheaply — but N safety checks at best-case is still fast).
    sorted_idx = torch.argsort(scores, dim=1)             # [N, S], best score first
    sorted_q   = torch.gather(final_q, 1, sorted_idx.unsqueeze(-1).expand(-1, -1, 6))  # [N, S, 6]
    sorted_sc  = torch.gather(scores, 1, sorted_idx)      # [N, S]
    qs_np  = sorted_q.cpu().numpy()
    sc_np  = sorted_sc.cpu().numpy()
    out: List[Optional[List[float]]] = []
    for i in range(N):
        picked = None
        for s in range(S):
            if not np.isfinite(sc_np[i, s]):
                break  # all remaining are inf too (sorted)
            q = [float(x) for x in qs_np[i, s]]
            ok, _, _ = check_angles(q)
            if ok:
                picked = q
                break
        out.append(picked)
    return out
