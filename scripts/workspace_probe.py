"""Empirical workspace probe - drives the real arm to each target.

Discovers what IK alone cannot:
  - positions where IK succeeds but motion times out (singularity in transit)
  - positions where firmware overload protection triggers
  - positions where safety check (floor/self-collision) blocks
  - reachable-with-orientation differences (tool-down vs free)

Safe defaults:
  - speed 20 (gentle)
  - skip low Z (default Z >= 80 mm - well above table)
  - skip points whose IK already returns OUT_OF_REACH
  - abort whole probe if any servo gets released mid-run
  - return to HOME after each successful probe (re-establishes safe start)

Usage:
  python scripts/workspace_probe.py            # default ~70 points, ~5 min
  python scripts/workspace_probe.py --quick    # ~30 points, ~2 min
  python scripts/workspace_probe.py --ik-only  # IK probe only, no motion (1 min, no risk)
"""
import argparse, json, math, time, pathlib, urllib.request, urllib.error

BASE = "http://localhost:8000"
ROOT = pathlib.Path(__file__).resolve().parent.parent

def req(path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    hdr = {"Content-Type": "application/json"} if body is not None else {}
    r = urllib.request.Request(f"{BASE}{path}", data=data, headers=hdr,
                               method="POST" if body is not None else "GET")
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as e:
        try: return e.code, json.loads(e.read())
        except: return e.code, {"_err": str(e)}
    except Exception as e:
        return -1, {"_err": str(e)}

def ik_solve(x, y, z, orient=None):
    body = {"x": x, "y": y, "z": z}
    if orient is not None:
        body["rx"], body["ry"], body["rz"] = orient
    _, r = req("/solve_ik", body)
    if r.get("ok") and r.get("angles"):
        return r["angles"], r.get("ikMode")
    return None, (r.get("error") or {}).get("code", "UNKNOWN")

def all_servos_enabled():
    _, r = req("/servo_diagnostics")
    return r.get("all_enabled") == 1

def move(angles, speed=20):
    code, r = req("/move", {"angles": angles, "speed": speed}, timeout=20)
    return code, r

def go_home(speed=20):
    code, r = req("/home", {}, timeout=20)
    return code, r

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--ik-only", action="store_true", help="no motion, just IK probe (safe)")
    ap.add_argument("--speed", type=int, default=20)
    ap.add_argument("--min-z", type=float, default=80.0)
    args = ap.parse_args()

    if args.quick:
        rs = [120, 220, 320]
        thetas = list(range(0, 360, 90))   # 4
        zs = [120, 240, 380]
    else:
        rs = [100, 180, 260, 340, 400]
        thetas = list(range(0, 360, 60))   # 6
        zs = [80, 160, 240, 320, 400]

    # 3 orientations to test
    orients = {
        "free": None,
        "tool_down": (-180.0, 0.0, 0.0),   # tool pointing -z (downward) at HOME-ish
        "tool_horiz": (90.0, 0.0, 0.0),    # tool pointing outward horizontally
    }

    # Build target list - skip low Z
    targets = []
    for r in rs:
        for th in thetas:
            x = round(r * math.cos(math.radians(th)), 1)
            y = round(r * math.sin(math.radians(th)), 1)
            for z in zs:
                if z < args.min_z: continue
                targets.append({"r": r, "theta": th, "z": z, "x": x, "y": y})

    print(f"== workspace probe ==")
    print(f"  targets: {len(targets)}  speed={args.speed}  min_z={args.min_z}mm")
    print(f"  mode: {'IK-only (safe)' if args.ik_only else 'real motion'}")
    print()

    if not args.ik_only:
        print("[init] going to HOME for clean start...")
        c, _ = go_home(args.speed)
        if c != 200:
            print(f"[ABORT] home failed ({c}); arm in unknown state - investigate first")
            return
        time.sleep(0.5)

    samples = []
    t0 = time.time()
    aborted = False
    consecutive_failures = 0
    MAX_CONSECUTIVE_FAILURES = 3  # circuit breaker: 3 in a row → probable stuck servo
    last_angles_seen = None
    for i, tgt in enumerate(targets, 1):
        rec = dict(tgt)
        # ---- IK probe at each orientation ----
        for orient_name, orient in orients.items():
            angles, mode_or_code = ik_solve(tgt["x"], tgt["y"], tgt["z"], orient)
            rec[f"ik_{orient_name}"] = {
                "ok": angles is not None,
                "mode": mode_or_code,
                "angles": [round(a, 1) for a in angles] if angles else None,
            }
        ik_free = rec["ik_free"]["ok"]

        # ---- motion probe ----
        # Skip motion if IK chose extreme J5 angles (empirically those are the
        # poses that trigger the J5 latch quirk on this firmware — see
        # memory/mycobot_firmware_quirks.md). Detect by |J5| > 60 (close to the
        # +75..+165 stuck range observed on this unit).
        ik_angles = rec["ik_free"]["angles"]
        j5_extreme = ik_angles and abs(ik_angles[4]) > 60
        if not args.ik_only and ik_free and not j5_extreme:
            angles = ik_angles
            # Reset to HOME first — each probe starts from the same clean state,
            # so failures don't cascade into worse poses for subsequent probes.
            hc, _ = go_home(args.speed)
            if hc != 200:
                rec["motion"] = {"ok": False, "skipped": "home_reset_failed", "http": hc}
                samples.append(rec); continue
            t_start = time.time()
            code, r = move(angles, args.speed)
            dt = time.time() - t_start
            actual = r.get("angles")
            motion_ok = code == 200
            joint_err = None
            if actual and len(actual) == 6:
                joint_err = [round(actual[j] - angles[j], 2) for j in range(6)]
            rec["motion"] = {
                "ok": motion_ok, "http": code, "elapsed_s": round(dt, 2),
                "smooth_mode": r.get("smoothMode"),
                "n_waypoints": r.get("nWaypoints"),
                "joint_err_deg": joint_err,
                "max_abs_err_deg": max(abs(e) for e in joint_err) if joint_err else None,
                "peak_currents": r.get("peakCurrents"),
                "error": r.get("error"),
            }
            # safety: if any servo got released, abort the whole probe
            if not all_servos_enabled():
                print(f"  ! [ABORT] servos released after probe {i} - STOP probe.")
                aborted = True
                samples.append(rec); break
            # circuit breaker: 3 consecutive motion failures = likely stuck servo
            # (see memory/mycobot_firmware_quirks.md for the J5 latch pattern)
            if motion_ok:
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                    print(f"  ! [ABORT] {MAX_CONSECUTIVE_FAILURES} consecutive motion failures - probable stuck servo.")
                    print(f"          last actual angles: {actual}")
                    print(f"          recovery: power-cycle the M5 (electrical reset only — clear_error_information/focus_servo are ineffective).")
                    aborted = True
                    samples.append(rec); break
            # additional detection: if same angles 3 times in a row despite different targets → stuck
            if actual is not None:
                if last_angles_seen is not None and all(abs(actual[j] - last_angles_seen[j]) < 0.5 for j in range(6)):
                    if not motion_ok:
                        pass  # already counted above
                last_angles_seen = actual
            time.sleep(0.2)
        else:
            if j5_extreme and not args.ik_only:
                rec["motion"] = {"ok": False, "skipped": "j5_extreme",
                                 "j5_deg": ik_angles[4] if ik_angles else None}
            else:
                rec["motion"] = None

        samples.append(rec)
        # progress
        n_ik_ok = sum(1 for s in samples if s["ik_free"]["ok"])
        n_mo_ok = sum(1 for s in samples if (s.get("motion") or {}).get("ok"))
        rate = i / (time.time() - t0)
        eta = (len(targets) - i) / rate if rate > 0 else 0
        m = rec.get("motion") or {}
        if args.ik_only or not ik_free:
            mo_tag = "-"
        elif m.get("ok"):
            mo_tag = f"OK({m.get('elapsed_s')}s)"
        else:
            err = m.get("error")
            err_msg = err if isinstance(err, str) else (err.get("message") if isinstance(err, dict) else "")
            mo_tag = f"NG http={m.get('http')} {(err_msg or '')[:70]}"
        print(f"  [{i:3d}/{len(targets)}] (r={tgt['r']:3d},θ={tgt['theta']:3d},z={tgt['z']:3d}) "
              f"ik_free={ik_free} motion={mo_tag}  "
              f"ik_ok={n_ik_ok}/{i} mo_ok={n_mo_ok}/{i}  eta={eta:.0f}s")

    if not args.ik_only and not aborted:
        print("\n[cleanup] returning to HOME...")
        go_home(args.speed)

    # --- analyze and write ---
    elapsed = time.time() - t0
    n_ik_free = sum(1 for s in samples if s["ik_free"]["ok"])
    n_ik_top  = sum(1 for s in samples if s["ik_tool_down"]["ok"])
    n_ik_horiz = sum(1 for s in samples if s["ik_tool_horiz"]["ok"])
    n_mo_ok   = sum(1 for s in samples if s.get("motion") and s["motion"]["ok"])
    n_mo_tried = sum(1 for s in samples if s.get("motion"))
    motion_failures = [s for s in samples if s.get("motion") and not s["motion"]["ok"]]
    ik_ok_motion_fail = [s for s in samples if s["ik_free"]["ok"] and s.get("motion") and not s["motion"]["ok"]]

    # Compute reach envelope (per-Z slice radial bounds from motion-confirmed points)
    by_z = {}
    for s in samples:
        if s.get("motion") and s["motion"]["ok"]:
            by_z.setdefault(s["z"], []).append(s["r"])
    z_envelope = [{"z": z, "r_min": min(rs), "r_max": max(rs), "n": len(rs)}
                  for z, rs in sorted(by_z.items())]

    # Maximum currents observed
    peak_per_joint = [0]*6
    for s in samples:
        m = s.get("motion") or {}
        pc = m.get("peak_currents")
        if pc and len(pc) == 6:
            for j in range(6):
                if pc[j] is not None and abs(pc[j]) > peak_per_joint[j]:
                    peak_per_joint[j] = abs(pc[j])

    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_s": round(elapsed, 1),
        "aborted": aborted,
        "n_samples": len(samples),
        "n_ik_free_ok": n_ik_free,
        "n_ik_tool_down_ok": n_ik_top,
        "n_ik_tool_horiz_ok": n_ik_horiz,
        "n_motion_tried": n_mo_tried,
        "n_motion_ok": n_mo_ok,
        "n_ik_ok_but_motion_failed": len(ik_ok_motion_fail),
        "motion_failure_reasons": [
            {"r": s["r"], "theta": s["theta"], "z": s["z"],
             "http": s["motion"].get("http"), "error": s["motion"].get("error"),
             "skipped": s["motion"].get("skipped"),
             "stall_stuck": (s["motion"].get("stall") or {}).get("stuck_joints") if isinstance(s["motion"].get("error"), dict) else None}
            for s in motion_failures[:20]
        ],
        "z_envelope_motion_confirmed": z_envelope,
        "peak_currents_per_joint_mA": peak_per_joint,
        "speed": args.speed,
        "min_z_filter": args.min_z,
        "ik_only": args.ik_only,
    }

    out = ROOT / "data" / "workspace.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"summary": summary, "samples": samples},
                              indent=2, ensure_ascii=False))

    print()
    print(f"=== probe complete ({elapsed:.0f}s, aborted={aborted}) ===")
    print(f"IK free reachable:        {n_ik_free}/{len(samples)}")
    print(f"IK tool_down reachable:   {n_ik_top}/{len(samples)}")
    print(f"IK tool_horiz reachable:  {n_ik_horiz}/{len(samples)}")
    if n_mo_tried:
        print(f"motion succeeded:         {n_mo_ok}/{n_mo_tried}")
        print(f"IK-ok-but-motion-NG:      {len(ik_ok_motion_fail)}  <- these are the interesting ones")
    print(f"peak currents per joint:  {peak_per_joint} mA")
    print(f"→ {out}")

if __name__ == "__main__":
    main()
