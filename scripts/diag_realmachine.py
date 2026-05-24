"""End-to-end behavior diagnostic on the real arm.

Runs via HTTP API against an already-running server. Records findings to
data/diag_report.json. Designed to be safe — uses speed<=25 and only poses
that should clear self-collision/floor checks.
"""
import json, time, math, urllib.request, urllib.parse, pathlib

BASE = "http://localhost:8000"
ROOT = pathlib.Path(__file__).resolve().parent.parent

def req(path, body=None):
    url = f"{BASE}{path}"
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Content-Type": "application/json"} if body else {}
    r = urllib.request.Request(url, data=data, headers=headers, method="POST" if body is not None else "GET")
    try:
        with urllib.request.urlopen(r, timeout=60) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        try: body = json.loads(e.read())
        except: body = str(e)
        return {"_http_error": e.code, "body": body}

def reqq(path, params):
    return req(f"{path}?{urllib.parse.urlencode(params)}")

def vec_err(a, b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

def move(angles, speed=20):
    return req("/move", {"angles": angles, "speed": speed})

def home():
    return req("/home", {})

report = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "tests": {}}

# -------------------------------------------------------------------------
# 1. FK vs firmware sweep across 10 random-ish poses
# -------------------------------------------------------------------------
print("== 1. FK vs firmware sweep ==")
fk_errors = []
test_poses = [
    [  0,   0,  -90,    0,    0,    0],
    [ 30,   0,  -90,    0,    0,    0],
    [-30,   0,  -90,    0,    0,    0],
    [  0, -30,  -60,    0,   30,    0],
    [ 45, -45,  -45,  -45,   60,   45],
    [-45, -30,  -90,   30,   45,  -45],
    [  0, -60,  -30,    0,   90,    0],
    [ 60, -20, -100,   20,   30,   90],
    [  0,   0,  -90,    0,   45,    0],
    [  0,   0,  -90,    0,  -45,    0],
]
for p in test_poses:
    move(p, speed=25); time.sleep(0.4)
    r = req("/debug/fk_compare")
    d = r["delta_fk_minus_fw_xyz_mm"]
    err = vec_err([0,0,0], d) if d else None
    fk_errors.append({"target": p, "actual": [round(x,2) for x in r["angles"]],
                      "delta": [round(x,2) for x in d] if d else None,
                      "err_mm": round(err,3) if err else None})
    print(f"  ang={[round(a,1) for a in r['angles']]}  |delta|={err:.2f}mm")
report["tests"]["fk_vs_firmware"] = {
    "max_err_mm": max(s["err_mm"] for s in fk_errors),
    "mean_err_mm": round(sum(s["err_mm"] for s in fk_errors)/len(fk_errors), 3),
    "samples": fk_errors,
}

# -------------------------------------------------------------------------
# 2. Waypoint readback accuracy (command vs final readback)
# -------------------------------------------------------------------------
print("== 2. waypoint readback accuracy ==")
wp_errs = []
for p in test_poses[:6]:
    move(p, speed=20); time.sleep(0.6)
    actual = req("/angles")["angles"]
    diff = [round(actual[i] - p[i], 3) for i in range(6)]
    max_abs = max(abs(d) for d in diff)
    wp_errs.append({"target": p, "actual": [round(x,2) for x in actual], "diff": diff, "max_abs_deg": round(max_abs,3)})
    print(f"  target={p}  max|diff|={max_abs:.2f}°  diff={diff}")
report["tests"]["waypoint_readback"] = {
    "max_abs_deg": max(s["max_abs_deg"] for s in wp_errs),
    "mean_max_abs_deg": round(sum(s["max_abs_deg"] for s in wp_errs)/len(wp_errs), 3),
    "samples": wp_errs,
}

# -------------------------------------------------------------------------
# 3. Per-joint current under slow motion (no load)
# -------------------------------------------------------------------------
print("== 3. per-joint peak currents (no load) ==")
home()
time.sleep(0.5)
peak = [0]*6
sweep_poses = [
    [ 60,   0,  -90,    0,    0,    0], [-60,   0,  -90,    0,    0,    0],
    [  0, -60,  -90,    0,    0,    0], [  0,  60,  -90,    0,    0,    0],
    [  0,   0, -150,    0,    0,    0], [  0,   0,  -30,    0,    0,    0],
    [  0,   0,  -90,   90,    0,    0], [  0,   0,  -90,  -90,    0,    0],
    [  0,   0,  -90,    0,   90,    0], [  0,   0,  -90,    0,  -90,    0],
    [  0,   0,  -90,    0,    0,  120], [  0,   0,  -90,    0,    0, -120],
]
samples = []
for p in sweep_poses:
    r = move(p, speed=25)
    if "peakCurrents" in r:
        peak = [max(peak[i], r["peakCurrents"][i]) for i in range(6)]
        samples.append({"target": p, "peak": r["peakCurrents"]})
        print(f"  target={p}  peak={r['peakCurrents']}")
report["tests"]["current_sweep"] = {
    "per_joint_peak_mA": peak,
    "current_threshold_active_mA": req("/currents")["threshold_mA"],
    "recommended_threshold_mA": [round(p*1.8) if p > 0 else 200 for p in peak],
    "samples": samples,
}

# -------------------------------------------------------------------------
# 4. IK round-trip (firmware + numeric)
# -------------------------------------------------------------------------
print("== 4. IK round-trip ==")
home(); time.sleep(0.3)
# pick reachable targets via current FK
ik_results = []
for tgt_ang in [[0,-30,-60,0,30,0], [30,-30,-60,-20,30,0], [-30,-30,-60,20,30,0]]:
    move(tgt_ang, speed=25); time.sleep(0.3)
    s = req("/debug/fk_compare")
    tip = s["fk_tip"]; fw = s["firmware_coords"]
    # Now ask server to solve IK back to that 6D pose
    if fw is None or len(fw) < 6:
        continue
    ik = req("/solve_ik", {"x": fw[0], "y": fw[1], "z": fw[2], "rx": fw[3], "ry": fw[4], "rz": fw[5]})
    sol = ik.get("angles")
    mode = ik.get("mode", "?")
    if sol:
        # Compare solution to original
        diff = [round(sol[i] - tgt_ang[i], 2) for i in range(6)]
        max_abs = max(abs(d) for d in diff)
    else:
        diff = None; max_abs = None
    ik_results.append({"target_angles": tgt_ang, "fw_pose": fw,
                       "ik_solution": [round(x,2) for x in sol] if sol else None,
                       "mode": mode, "diff_deg": diff, "max_abs_deg": max_abs})
    print(f"  tgt={tgt_ang}  ik={[round(x,2) for x in sol] if sol else None}  mode={mode}  max|diff|={max_abs}")
report["tests"]["ik_roundtrip"] = ik_results

# -------------------------------------------------------------------------
# 5. Cartesian linearity
# -------------------------------------------------------------------------
print("== 5. cartesian linearity ==")
home(); time.sleep(0.3)
# Move via /move_cartesian if available; sample tip positions between start/end
try:
    s0 = req("/debug/fk_compare"); start = s0["firmware_coords"][:3]
    target = [start[0] + 60, start[1] - 30, start[2] - 40]
    cart = req("/move_cartesian", {"target": target, "speed": 15, "mode": "linear"})
    time.sleep(0.5)
    s1 = req("/debug/fk_compare"); end = s1["firmware_coords"][:3]
    err = vec_err(end, target)
    report["tests"]["cartesian"] = {
        "start": start, "requested": target, "end_firmware": end,
        "endpoint_err_mm": round(err, 2),
        "response_summary": {k: cart[k] for k in cart if k not in ("waypoints","waypointPoses")},
    }
    print(f"  start={[round(x,1) for x in start]} → target={target}")
    print(f"  end  ={[round(x,1) for x in end]}  err={err:.2f}mm")
except Exception as e:
    report["tests"]["cartesian"] = {"error": str(e)}
    print(f"  cartesian test failed: {e}")

# -------------------------------------------------------------------------
# 6. Abort mid-motion
# -------------------------------------------------------------------------
print("== 6. abort mid-motion ==")
home(); time.sleep(0.3)
import threading
abort_test = {}
def kick_abort():
    time.sleep(0.4)
    try:
        abort_test["abort_resp"] = req("/abort", {})
    except Exception as e:
        abort_test["abort_err"] = str(e)
t = threading.Thread(target=kick_abort); t.start()
try:
    t0 = time.time()
    r = move([60, -60, -90, 60, 60, 90], speed=15)
    abort_test["move_resp"] = {k: r[k] for k in ("elapsed","aborted","error") if k in r}
    abort_test["actual_elapsed_s"] = round(time.time() - t0, 2)
except Exception as e:
    abort_test["move_err"] = str(e)
t.join()
final = req("/angles")["angles"]
abort_test["final_angles"] = [round(x,2) for x in final]
report["tests"]["abort"] = abort_test
print(f"  result: {abort_test}")

# back to HOME
home()

# -------------------------------------------------------------------------
out = ROOT / "data" / "diag_report.json"
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
print(f"\n→ written: {out}")
