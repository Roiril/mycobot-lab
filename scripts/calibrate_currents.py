"""Per-joint current calibration.

Drives the arm through a representative set of poses (no payload) while polling
servo currents at 20 Hz. Reports per-joint peak and suggests a threshold of
peak * 2.0 (with a 200 mA floor for noise margin).

Requires the server to be running. Talks to it via HTTP.

Writes:
  data/current_calibration.json  — raw samples + recommendation
  .calibrated                    — marker file (server uses 1500 mA instead of safe 800)

After running, restart the server so it picks up the calibration marker. To
adopt the recommended threshold, edit CURRENT_THRESHOLD_MA in
src/arm/constants.py manually (kept explicit so a change of payload requires
a conscious update).
"""
from __future__ import annotations
import json, time, urllib.request, urllib.error, threading, pathlib, sys

BASE = "http://localhost:8000"
ROOT = pathlib.Path(__file__).resolve().parent.parent

def req(path, body=None, timeout=60):
    data = json.dumps(body).encode() if body is not None else None
    hdr = {"Content-Type": "application/json"} if body is not None else {}
    r = urllib.request.Request(f"{BASE}{path}", data=data, headers=hdr,
                               method="POST" if body is not None else "GET")
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        return {"_http_error": e.code, "body": e.read().decode()[:300]}

# Poses chosen to load every joint: each pose loads at least one wrist or
# shoulder joint into a configuration where gravity does the most work.
POSES = [
    [  0,   0,  -90,    0,    0,    0],   # HOME
    [ 60,   0,  -90,    0,    0,    0],   # J1 swing
    [-60,   0,  -90,    0,    0,    0],
    [  0, -60,  -90,    0,    0,    0],   # J2 lift
    [  0,  60,  -90,    0,    0,    0],
    [  0, -30, -150,    0,    0,    0],   # J3 fold
    [  0, -30,  -30,    0,    0,    0],   # J3 extend
    [  0, -30,  -90,   90,    0,    0],   # J4
    [  0, -30,  -90,  -90,    0,    0],
    [  0, -30,  -90,    0,   90,    0],   # J5 (loaded by tool weight)
    [  0, -30,  -90,    0,  -90,    0],
    [  0, -30,  -90,    0,    0,  120],   # J6 spin
    [  0,   0,  -90,    0,    0,    0],   # back to HOME
]

def poll_loop(stop, peaks):
    """Poll /currents at 20 Hz, update peaks in place."""
    while not stop.is_set():
        r = req("/currents")
        cs = r.get("currents")
        if isinstance(cs, list) and len(cs) == 6:
            for i, c in enumerate(cs):
                if c is not None and abs(c) > peaks[i]:
                    peaks[i] = abs(c)
        time.sleep(0.05)

def main():
    print("== current calibration ==")
    print("ensure: arm has no payload, environment is clear, e-stop reachable.")
    print()

    # Verify server reachable and arm powered
    s = req("/power")
    if not s.get("ok"):
        print("[ABORT] servos not powered (E-stop? Transponder not active?)")
        sys.exit(2)
    print(f"power ok. baseline currents: {req('/currents').get('currents')}")

    peaks = [0]*6
    stop = threading.Event()
    t = threading.Thread(target=poll_loop, args=(stop, peaks), daemon=True)
    t.start()

    print("\nsweeping poses...")
    for i, p in enumerate(POSES, 1):
        print(f"  [{i}/{len(POSES)}] {p}")
        r = req("/move", {"angles": p, "speed": 25})
        if "_http_error" in r:
            print(f"    skipped: HTTP {r['_http_error']} {r['body']}")
            continue
        time.sleep(0.2)

    stop.set(); t.join()

    rec = []
    for i, p in enumerate(peaks):
        suggested = max(200, round(p * 2.0))
        rec.append(suggested)
    print()
    print(f"per-joint peak (mA):       {peaks}")
    print(f"suggested threshold (mA):  {rec}  (peak*2, min 200)")
    print(f"max suggested:             {max(rec)} mA")

    out = ROOT / "data" / "current_calibration.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "peaks_mA": peaks,
        "suggested_per_joint_mA": rec,
        "suggested_uniform_mA": max(rec),
        "notes": "Set CURRENT_THRESHOLD_MA in src/arm/constants.py to the uniform value (or implement per-joint thresholds).",
    }, indent=2, ensure_ascii=False))
    print(f"→ wrote {out}")

    marker = ROOT / ".calibrated"
    if not marker.exists():
        marker.write_text(time.strftime("%Y-%m-%d %H:%M:%S\n"))
        print(f"→ created {marker} (server will now use full CURRENT_THRESHOLD_MA instead of safe-mode 800)")

if __name__ == "__main__":
    main()
