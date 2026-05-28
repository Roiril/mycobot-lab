"""GPU-batched reach grid generator.

Same output format as scripts/reachable_grid.py (schema reachable_grid_v3) but
uses src/arm/ik_gpu (PyTorch on RTX 4090) instead of the per-point CPU DLS.
Typical speedup is ~200x — a 30000-point dense grid finishes in under a minute
where the CPU version would take hours.

Usage:
  python scripts/reachable_grid_gpu.py [--angles 36] [--z-step 15] [--r-step 15]
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


def build_target_grid(args, profile):
    """Generate cylindrical (x,y,z) targets using the FK profile envelope."""
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

    targets = []  # list of (x, y, z)
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
            for ai in range(args.angles):
                theta = 2 * math.pi * ai / args.angles
                targets.append((r * math.cos(theta), r * math.sin(theta), z))
            r += args.r_step
        z += args.z_step
    return targets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", type=int, default=36, help="J1 sweep count per ring")
    ap.add_argument("--z-step", type=float, default=15.0)
    ap.add_argument("--r-step", type=float, default=15.0)
    ap.add_argument("--batch", type=int, default=20000,
                    help="GPU batch size (memory: ~80 MB per 10k points × 7 seeds @ FP64)")
    ap.add_argument("--out", type=pathlib.Path,
                    default=ROOT / "data" / "reachable_grid.json")
    args = ap.parse_args()

    prof_data = load_profile()
    if not prof_data:
        print("ERROR: data/reachable_rz.json not found. Run reachable_rz.py first.")
        return 1

    print(f"Device: {DEVICE}", flush=True)
    targets = build_target_grid(args, prof_data["profile_rz"])
    N = len(targets)
    print(f"Generated {N} candidate targets (angles={args.angles} "
          f"z_step={args.z_step} r_step={args.r_step})", flush=True)

    # Batched GPU solve
    points = []
    n_ok = 0
    t0 = time.monotonic()
    targets_arr = np.array(targets, dtype=np.float64)
    for start in range(0, N, args.batch):
        chunk = targets_arr[start:start + args.batch]
        t_chunk = time.monotonic()
        results = solve_grid(chunk)
        for (x, y, z), q in zip(chunk, results):
            if q is None:
                continue
            points.append({"x": round(float(x), 1),
                           "y": round(float(y), 1),
                           "z": round(float(z), 1),
                           "angles": [round(a, 2) for a in q]})
            n_ok += 1
        elapsed = time.monotonic() - t0
        chunk_s = time.monotonic() - t_chunk
        print(f"  batch {start:6d}..{start + len(chunk):6d}  "
              f"chunk={chunk_s:.1f}s  total_ok={n_ok}  ({elapsed:.0f}s)", flush=True)

    out = {
        "schema": "reachable_grid_v3",
        "params": {"angles": args.angles, "z_step": args.z_step,
                   "r_step": args.r_step, "engine": "gpu"},
        "stats": {
            "n_tried": N,
            "n_ok": len(points),
            "elapsed_s": round(time.monotonic() - t0, 1),
            "device": str(DEVICE),
        },
        "points": points,
    }
    args.out.write_text(json.dumps(out))
    print(f"\nSaved {len(points)} / {N} reachable points → {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
