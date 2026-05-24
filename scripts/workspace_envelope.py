"""Dense IK-only envelope probe.

No motion — safe and fast. Builds a per-Z radial-reach envelope used by
fast-fail rejection in solve_with_retries.

Output: data/workspace_envelope.json
  {
    "z_slices_mm": [...],
    "r_max_at_z_mm": [...],   # outer reach per Z
    "r_min_at_z_mm": [...],   # inner dead zone per Z (often base column)
    "r_max_overall_mm": ...,
    "z_min_reached_mm": ..., "z_max_reached_mm": ...
  }

Usage:
  python scripts/workspace_envelope.py        # ~1 min, 8 θ × 12 r × 12 z = 1152 IK calls
"""
import json, math, time, urllib.request, pathlib

BASE = "http://localhost:8000"
ROOT = pathlib.Path(__file__).resolve().parent.parent

def post(p, b):
    r = urllib.request.Request(f"{BASE}{p}", data=json.dumps(b).encode(),
                               headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(r, timeout=5) as resp:
            return json.loads(resp.read())
    except Exception:
        return {"ok": False}

def reachable(x, y, z):
    r = post("/solve_ik", {"x": x, "y": y, "z": z})
    return bool(r.get("ok") and r.get("angles"))

def main():
    # Grid: 8 angles × 12 radii × 12 z slices = 1152 calls. ~3 min at 300ms/call.
    # Coarser than full sweep but enough for smooth envelope.
    rs = list(range(40, 500, 40))            # 40..480 step 40 = 12
    thetas = list(range(0, 360, 45))         # 8
    zs = list(range(40, 520, 40))            # 40..480 step 40 = 12

    print(f"probing {len(rs)*len(thetas)*len(zs)} (r,θ,z) IK targets...")
    t0 = time.time()
    # For each (z, theta): find max r reachable + min r reachable
    raw = {}  # (z, theta) -> sorted list of r
    done = 0
    total = len(rs) * len(thetas) * len(zs)
    for z in zs:
        for theta in thetas:
            for r in rs:
                x = round(r * math.cos(math.radians(theta)), 1)
                y = round(r * math.sin(math.radians(theta)), 1)
                ok = reachable(x, y, z)
                if ok:
                    raw.setdefault((z, theta), []).append(r)
                done += 1
                if done % 100 == 0:
                    rate = done / (time.time() - t0)
                    print(f"  {done}/{total}  rate={rate:.1f}/s  eta={(total-done)/rate:.0f}s")

    # Aggregate per-Z: pessimistic envelope (r_max = MIN across θ; r_min = MAX across θ)
    # This gives a conservative cylinder we know is reachable from ANY angle.
    # Also collect optimistic (max across θ).
    per_z = {}
    for z in zs:
        opt_max, pes_max = 0, 10000
        opt_min, pes_min = 10000, 0
        any_reachable = False
        thetas_at_z = []
        for theta in thetas:
            rs_at = raw.get((z, theta))
            if not rs_at: continue
            thetas_at_z.append(theta)
            any_reachable = True
            opt_max = max(opt_max, max(rs_at))
            pes_max = min(pes_max, max(rs_at))
            opt_min = min(opt_min, min(rs_at))
            pes_min = max(pes_min, min(rs_at))
        if any_reachable:
            per_z[z] = {
                "r_max_opt": opt_max, "r_max_pes": pes_max,
                "r_min_opt": opt_min, "r_min_pes": pes_min,
                "n_thetas_reachable": len(thetas_at_z),
                "thetas_reachable": thetas_at_z,
            }

    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_s": round(time.time() - t0, 1),
        "grid": {"r": rs, "theta": thetas, "z": zs},
        "z_reachable_min_mm": min(per_z),
        "z_reachable_max_mm": max(per_z),
        "r_max_anywhere_mm": max(s["r_max_opt"] for s in per_z.values()),
        "per_z": per_z,
    }
    out = ROOT / "data" / "workspace_envelope.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\n=== done ({summary['elapsed_s']}s) ===")
    print(f"Z reach: {summary['z_reachable_min_mm']}..{summary['z_reachable_max_mm']}mm")
    print(f"R max:   {summary['r_max_anywhere_mm']}mm")
    print(f"per-Z slices: {len(per_z)}")
    print(f"→ {out}")

if __name__ == "__main__":
    main()
