"""IK+safety-tested reachable grid for the UI click-selection target.

Generates a 3D cylindrical grid of points and tests each with policy-aware
multi-seed IK + safety check. Each accepted point ships with its baked joint
angles, chosen via the petal-policy:

  - J1 takes the target direction (atan2(y,x))
  - J4 ≈ 0 (no forearm roll)
  - J5 = -90 (flange +z = world up; camera up)
  - J6 = 90 (camera image upright)
  - J2, J3 span the remaining position

Continuity pass: after initial greedy assignment (using previous θ as seed),
iterates over the grid and re-selects each point's solution against its
6-neighborhood (±θ, ±r, ±z), minimizing posture+neighbor-delta score.

Output schema v3: each point = {x, y, z, angles[6]}.

Usage:
  python scripts/reachable_grid.py [--angles 24] [--z-step 25] [--r-step 25]
                                   [--smooth-passes 2]
"""
from __future__ import annotations
import argparse, json, math, sys, pathlib, time
from typing import Dict, List, Optional, Tuple

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.ik_policy import (
    enumerate_solutions, pick_best, posture_score, continuity_penalty,
)
from arm.safety import check_angles


def load_profile():
    p = ROOT / "data" / "reachable_rz.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())


def _generate_initial(args, profile):
    """Phase 1+greedy-continuity: sweep z→r→θ, use prev-θ angles as seed bias."""
    z_min = min(p["z"] for p in profile)
    z_max = max(p["z"] for p in profile)
    prof_by_z = {round(p["z"]): p for p in profile}
    sorted_zs = sorted(prof_by_z.keys())

    def r_range_at(z):
        if z < z_min or z > z_max:
            return None
        nearest_z = min(sorted_zs, key=lambda zz: abs(zz - z))
        if abs(nearest_z - z) > args.z_step:
            return None
        p = prof_by_z[nearest_z]
        return (p["r_min"], p["r_max"])

    # Output: each cell keyed by (z_idx, r_idx, theta_idx). Stored as dict so
    # smoothing pass can address neighbors O(1).
    cells: Dict[Tuple[int, int, int], dict] = {}
    n_tried = n_ik_fail = n_safe_fail = 0
    t0 = time.monotonic()

    z_levels = []
    z = z_min
    while z <= z_max:
        z_levels.append(z); z += args.z_step

    for zi, z in enumerate(z_levels):
        rr = r_range_at(z)
        if rr is None:
            continue
        r_min, r_max = rr
        if r_max - r_min < args.r_step:
            continue
        r_values = []
        r = r_min
        while r <= r_max:
            r_values.append(r); r += args.r_step
        for ri, r in enumerate(r_values):
            prev_angles = None  # reset at start of each ring
            for ai in range(args.angles):
                theta = 2 * math.pi * ai / args.angles
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                n_tried += 1
                cands = enumerate_solutions([x, y, z])
                if not cands:
                    n_ik_fail += 1
                    continue
                ang = pick_best([x, y, z], cands, prev_angles=prev_angles)
                if ang is None:
                    n_ik_fail += 1
                    continue
                ok, _, _ = check_angles(ang)
                if not ok:
                    n_safe_fail += 1
                    continue
                cells[(zi, ri, ai)] = {
                    "x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                    "angles": [round(a, 2) for a in ang],
                    "candidates": [(s, sc) for s, sc in cands],  # for smoothing pass
                }
                prev_angles = ang
        elapsed = time.monotonic() - t0
        print(f"  [init] z={z:.0f} tried={n_tried} ok={len(cells)} "
              f"ik_fail={n_ik_fail} safe_fail={n_safe_fail} ({elapsed:.0f}s)",
              flush=True)

    stats = {
        "n_tried": n_tried, "n_ok": len(cells),
        "n_ik_fail": n_ik_fail, "n_safe_fail": n_safe_fail,
        "elapsed_s": round(time.monotonic() - t0, 1),
    }
    return cells, stats, len(z_levels)


def _neighbors(key: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
    """6-connectivity in (z_idx, r_idx, theta_idx) cylindrical grid."""
    zi, ri, ai = key
    return [
        (zi, ri, ai - 1), (zi, ri, ai + 1),
        (zi, ri - 1, ai), (zi, ri + 1, ai),
        (zi - 1, ri, ai), (zi + 1, ri, ai),
    ]


def _smooth_pass(cells: Dict[Tuple[int, int, int], dict], n_angles: int) -> int:
    """One pass of neighbor-aware re-selection. Returns number of cells changed.

    For each cell with multiple candidates, score each candidate against
    posture + mean continuity cost to existing neighbors. Switch if better.
    """
    n_changed = 0
    for key, cell in cells.items():
        cands = cell.get("candidates")
        if not cands or len(cands) < 2:
            continue
        # gather existing neighbor angles (wrap theta)
        zi, ri, ai = key
        neighbor_keys = [
            (zi, ri, (ai - 1) % n_angles), (zi, ri, (ai + 1) % n_angles),
            (zi, ri - 1, ai), (zi, ri + 1, ai),
            (zi - 1, ri, ai), (zi + 1, ri, ai),
        ]
        neighbor_qs = [cells[nk]["angles"] for nk in neighbor_keys if nk in cells]
        if not neighbor_qs:
            continue
        target = [cell["x"], cell["y"], cell["z"]]
        best = None; best_score = float("inf")
        for ang, ps in cands:
            cont = sum(continuity_penalty(ang, nq) for nq in neighbor_qs) / len(neighbor_qs)
            total = ps + cont
            if total < best_score:
                best_score = total; best = ang
        if best is not None:
            current = cell["angles"]
            # rounded equality check
            if any(abs(a - b) > 1e-3 for a, b in zip(current, [round(v, 2) for v in best])):
                cell["angles"] = [round(a, 2) for a in best]
                n_changed += 1
    return n_changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", type=int, default=24)
    ap.add_argument("--z-step", type=float, default=30.0)
    ap.add_argument("--r-step", type=float, default=30.0)
    ap.add_argument("--smooth-passes", type=int, default=2,
                    help="continuity-smoothing iterations after initial pass")
    ap.add_argument("--out", type=pathlib.Path,
                    default=ROOT / "data" / "reachable_grid.json")
    args = ap.parse_args()

    prof_data = load_profile()
    if not prof_data:
        print("ERROR: data/reachable_rz.json not found. Run reachable_rz.py first.")
        return 1

    cells, stats, _ = _generate_initial(args, prof_data["profile_rz"])

    for p in range(args.smooth_passes):
        t0 = time.monotonic()
        n_changed = _smooth_pass(cells, args.angles)
        print(f"  [smooth pass {p+1}] changed={n_changed} ({time.monotonic()-t0:.1f}s)",
              flush=True)
        if n_changed == 0:
            break

    # Strip the candidate cache (only used during smoothing) before saving
    points = []
    for cell in cells.values():
        points.append({"x": cell["x"], "y": cell["y"], "z": cell["z"],
                       "angles": cell["angles"]})

    out = {
        "schema": "reachable_grid_v3",  # v3: petal-policy + continuity smoothing
        "params": {
            "angles": args.angles, "z_step": args.z_step, "r_step": args.r_step,
            "smooth_passes": args.smooth_passes,
        },
        "stats": stats,
        "points": points,
    }
    args.out.write_text(json.dumps(out))
    print(f"\nSaved {len(points)} reachable points → {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
