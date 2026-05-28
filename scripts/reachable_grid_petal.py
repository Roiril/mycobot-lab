"""Petal-symmetric reach grid generator.

Key insight: J1 is a pure rotation about the base Z axis, and everything
downstream is independent of J1's value (it just rotates with the wrist).
So for any target at (r·cosθ, r·sinθ, z), the joint solution is:

  [J1_solved_at_theta0 + θ_deg, J2, J3, J4, J5, J6]

where J2..J6 come from solving IK ONCE at the (r, 0, z) target.

This guarantees a true petal pattern — every theta-rotated copy of a target
has identical J2..J6, so the arm posture is rigorously coherent across the
fan. Also means we only run N_r × N_z IK problems (e.g. 23 × 33 ≈ 760)
instead of N_r × N_z × N_θ (e.g. 27000) — 30-40× faster than the cylindrical
generator while producing higher-quality output.

Output: schema reachable_grid_v4 (adds field "petal_symmetric": true).
"""
from __future__ import annotations
import argparse, json, math, sys, pathlib, time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.ik_gpu import solve_grid, DEVICE  # noqa: E402


def load_profile():
    p = ROOT / "data" / "reachable_rz.json"
    if not p.exists():
        return None
    return json.loads(p.read_text())


def build_rz_cells(args, profile):
    """Generate (r, z) cells along the workspace envelope."""
    z_min = min(p["z"] for p in profile)
    z_max = max(p["z"] for p in profile)
    prof_by_z = {round(p["z"]): p for p in profile}
    sorted_zs = sorted(prof_by_z.keys())

    def r_range_at(z):
        if z < z_min or z > z_max:
            return None
        nz = min(sorted_zs, key=lambda zz: abs(zz - z))
        if abs(nz - z) > args.z_step:
            return None
        p = prof_by_z[nz]
        return (p["r_min"], p["r_max"])

    cells = []  # list of (r, z)
    z = z_min
    while z <= z_max:
        rr = r_range_at(z)
        if rr is None:
            z += args.z_step; continue
        r_min, r_max = rr
        if r_max - r_min < args.r_step:
            z += args.z_step; continue
        r = r_min
        while r <= r_max:
            cells.append((r, z))
            r += args.r_step
        z += args.z_step
    return cells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", type=int, default=36, help="J1 fan sweep count")
    ap.add_argument("--z-step", type=float, default=15.0)
    ap.add_argument("--r-step", type=float, default=15.0)
    ap.add_argument("--out", type=pathlib.Path,
                    default=ROOT / "data" / "reachable_grid.json")
    args = ap.parse_args()

    prof_data = load_profile()
    if not prof_data:
        print("ERROR: data/reachable_rz.json not found.")
        return 1

    print(f"Device: {DEVICE}", flush=True)
    cells = build_rz_cells(args, prof_data["profile_rz"])
    N_rz = len(cells)
    print(f"(r, z) cells to solve: {N_rz}  | angle fan: {args.angles}  "
          f"=> {N_rz * args.angles} output points", flush=True)

    # Solve IK once per (r, z) at theta=0 (target on +x axis)
    targets_at_zero = np.array([[r, 0.0, z] for r, z in cells], dtype=np.float64)
    t0 = time.monotonic()
    base_solutions = solve_grid(targets_at_zero)
    solve_s = time.monotonic() - t0
    n_solved = sum(1 for s in base_solutions if s is not None)
    print(f"  IK solved {n_solved}/{N_rz} (r,z) cells in {solve_s:.1f}s "
          f"({solve_s/N_rz*1000:.1f} ms/cell)", flush=True)

    # Fan: for each θ, replicate each (r, z) solution with J1 += θ
    # Joint limits on J1 may clamp some fans; we drop fans where J1 falls
    # outside the limit (the symmetry breaks at the J1 boundary).
    from arm.kinematics import JOINT_LIMITS
    j1_lo, j1_hi = JOINT_LIMITS[0]

    points = []
    n_dropped_j1 = 0
    for (r, z), sol in zip(cells, base_solutions):
        if sol is None:
            continue
        j1_base, j2, j3, j4, j5, j6 = sol
        for ai in range(args.angles):
            theta_deg = ai * (360.0 / args.angles)
            # Map [0, 360) → (-180, 180] for J1
            j1 = j1_base + theta_deg
            j1 = ((j1 + 180.0) % 360.0) - 180.0
            if not (j1_lo <= j1 <= j1_hi):
                n_dropped_j1 += 1
                continue
            theta_rad = math.radians(theta_deg)
            x = r * math.cos(theta_rad)
            y = r * math.sin(theta_rad)
            points.append({
                "x": round(x, 1), "y": round(y, 1), "z": round(z, 1),
                "angles": [round(j1, 2), round(j2, 2), round(j3, 2),
                           round(j4, 2), round(j5, 2), round(j6, 2)],
            })

    out = {
        "schema": "reachable_grid_v4",
        "params": {
            "angles": args.angles, "z_step": args.z_step, "r_step": args.r_step,
            "engine": "gpu_petal_symmetric",
        },
        "stats": {
            "n_rz_cells": N_rz, "n_rz_solved": n_solved,
            "n_output_points": len(points),
            "n_dropped_j1_limit": n_dropped_j1,
            "elapsed_s": round(time.monotonic() - t0, 1),
            "device": str(DEVICE),
            "petal_symmetric": True,
        },
        "points": points,
    }
    args.out.write_text(json.dumps(out))
    print(f"\nSaved {len(points)} petal-symmetric points → {args.out}", flush=True)
    if n_dropped_j1:
        print(f"  ({n_dropped_j1} fan copies dropped due to J1 limit)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
