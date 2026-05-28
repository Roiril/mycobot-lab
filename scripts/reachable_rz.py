"""FK-based reachable region in the (r, z) plane.

Sweep J2/J3/J4 (J5=0, J1=0, J6=0), apply safety.check_angles, collect tip
positions, project to (r=sqrt(x^2+y^2), z). Output PNG mask.

This is the "pre-IK" envelope: lets the UI show roughly where the tip can go
WITHOUT solving IK. Rotationally symmetric around base z-axis (modulo J1 limits).

Usage:
  python scripts/reachable_rz.py [--step 5] [--out data/reachable_rz.png]
"""
from __future__ import annotations
import argparse, json, math, sys, pathlib, time
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.kinematics import end_effector, JOINT_LIMITS
from arm.safety import check_angles


def sweep(step_deg: float, j5_step: float = 0.0):
    j2lo, j2hi = JOINT_LIMITS[1]
    j3lo, j3hi = JOINT_LIMITS[2]
    j4lo, j4hi = JOINT_LIMITS[3]
    j5lo, j5hi = JOINT_LIMITS[4]
    if j5_step <= 0:
        j5_values = [0.0]
    else:
        j5_values = []
        v = j5lo
        while v <= j5hi:
            j5_values.append(v); v += j5_step
    points = []
    n_tot = 0
    n_ok = 0
    j2 = j2lo
    while j2 <= j2hi:
        j3 = j3lo
        while j3 <= j3hi:
            j4 = j4lo
            while j4 <= j4hi:
                for j5 in j5_values:
                    n_tot += 1
                    ang = [0.0, j2, j3, j4, j5, 0.0]
                    ok, _, _ = check_angles(ang)
                    if ok:
                        x, y, z = end_effector(ang)
                        r = math.hypot(x, y)
                        points.append((r, z))
                        n_ok += 1
                j4 += step_deg
            j3 += step_deg
        j2 += step_deg
    return points, n_tot, n_ok


def rasterize(points, px_per_mm=0.4, r_max=600, z_lo=-300, z_hi=700):
    W = int((r_max + 50) * px_per_mm)
    H = int((z_hi - z_lo) * px_per_mm)
    img = Image.new("RGBA", (W, H), (30, 30, 30, 255))
    px = img.load()
    radius = 2
    for (r, z) in points:
        if r < 0 or r > r_max: continue
        if z < z_lo or z > z_hi: continue
        u = int(r * px_per_mm)
        v = H - 1 - int((z - z_lo) * px_per_mm)
        for du in range(-radius, radius + 1):
            for dv in range(-radius, radius + 1):
                uu, vv = u + du, v + dv
                if 0 <= uu < W and 0 <= vv < H:
                    px[uu, vv] = (80, 200, 120, 255)
    # axes overlay: floor (z=0), base column (r=0), 100mm grid
    grid = (90, 90, 90, 255)
    for r_grid in range(0, r_max + 1, 100):
        u = int(r_grid * px_per_mm)
        if 0 <= u < W:
            for v in range(H):
                if px[u, v] == (30, 30, 30, 255):
                    px[u, v] = grid
    for z_grid in range(z_lo - (z_lo % 100), z_hi + 1, 100):
        v = H - 1 - int((z_grid - z_lo) * px_per_mm)
        if 0 <= v < H:
            for u in range(W):
                if px[u, v] == (30, 30, 30, 255):
                    px[u, v] = grid
    # floor line (z=0) in stronger color
    v_floor = H - 1 - int((0 - z_lo) * px_per_mm)
    if 0 <= v_floor < H:
        for u in range(W):
            px[u, v_floor] = (180, 60, 60, 255)
    return img


def profile_from_points(points, z_bin=10):
    """For each Z bin, find r_min and r_max. Returns sorted list of dicts."""
    bins = {}
    for r, z in points:
        zk = round(z / z_bin) * z_bin
        if zk not in bins:
            bins[zk] = [r, r]
        else:
            if r < bins[zk][0]: bins[zk][0] = r
            if r > bins[zk][1]: bins[zk][1] = r
    return [{"z": zk, "r_min": v[0], "r_max": v[1]} for zk, v in sorted(bins.items())]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=10.0, help="J2/J3/J4 sample step deg")
    ap.add_argument("--j5-step", type=float, default=30.0,
                    help="J5 sample step deg (0 = fix at 0°)")
    ap.add_argument("--out-png", default=str(ROOT / "data" / "reachable_rz.png"))
    ap.add_argument("--out-json", default=str(ROOT / "data" / "reachable_rz.json"))
    args = ap.parse_args()
    print(f"sweeping J2/J3/J4 step={args.step}°, J5 step={args.j5_step}° ...")
    t0 = time.time()
    pts, n_tot, n_ok = sweep(args.step, args.j5_step)
    dt = time.time() - t0
    print(f"  {n_ok}/{n_tot} valid ({100*n_ok/n_tot:.1f}%) in {dt:.1f}s")
    if pts:
        rs = [p[0] for p in pts]; zs = [p[1] for p in pts]
        print(f"  r: {min(rs):.0f}..{max(rs):.0f}mm   z: {min(zs):.0f}..{max(zs):.0f}mm")
    img = rasterize(pts)
    out_png = pathlib.Path(args.out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png)
    print(f"→ {out_png}")

    profile = profile_from_points(pts)
    summary = {
        "params": {"step_deg": args.step, "j5_step_deg": args.j5_step,
                   "j1_fixed_deg": 0.0, "j6_fixed_deg": 0.0,
                   "note": "rotational symmetry around base z-axis assumed"},
        "stats": {"n_total": n_tot, "n_valid": n_ok,
                  "elapsed_s": round(dt, 2),
                  "r_min_mm": min((p[0] for p in pts), default=None),
                  "r_max_mm": max((p[0] for p in pts), default=None),
                  "z_min_mm": min((p[1] for p in pts), default=None),
                  "z_max_mm": max((p[1] for p in pts), default=None)},
        "profile_rz": profile,
    }
    out_json = pathlib.Path(args.out_json)
    out_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"→ {out_json}  ({len(profile)} z-bins)")


if __name__ == "__main__":
    main()
