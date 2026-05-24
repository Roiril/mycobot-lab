"""J5 latch behavior diagnostic.

Empirically characterize the J5 freezing pattern observed on this unit:
- J5 has been observed to latch (servo "locked" but enable=1) at +74.97°
- Same value reproduces across power cycles
- Other joints unaffected

Tests:
  A) Step J5 up from 0 in 5° increments, see where readback freezes
  B) Approach the latch point from above (start at +90 if possible) — does it
     unstick going DOWN?
  C) Test other quadrants (-J5 direction)
"""
import json, time, urllib.request, urllib.error

BASE = "http://localhost:8000"

def req(p, b=None, t=10):
    data = json.dumps(b).encode() if b is not None else None
    hdr = {"Content-Type": "application/json"} if b is not None else {}
    r = urllib.request.Request(f"{BASE}{p}", data=data, headers=hdr,
                               method="POST" if b is not None else "GET")
    try:
        with urllib.request.urlopen(r, timeout=t) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as e:
        try: return e.code, json.loads(e.read())
        except: return e.code, {"_err": str(e)}

def angles(): return req("/angles")[1]["angles"]
def move_to(a, speed=15, rescue=False):
    body = {"angles": a, "speed": speed}
    if rescue: body["rescue"] = True
    return req("/move", body, t=20)

def step_test(label, j5_targets, base=(0,0,-90,0,0,0)):
    print(f"\n=== {label} ===")
    for j5 in j5_targets:
        tgt = list(base); tgt[4] = j5
        code, r = move_to(tgt, speed=15)
        actual = (r.get("angles") if code == 200 else r.get("lastActual")) or angles()
        actual_j5 = actual[4] if actual else None
        ok = code == 200
        stall = r.get("stall")
        if stall:
            stuck = stall.get("stuck_joints")
            stuck_j5 = 5 in (stuck or [])
            print(f"  J5 target={j5:7.2f}  actual={actual_j5:7.2f}  delta={actual_j5-j5:+.2f}°  "
                  f"http={code}  STALL stuck={stuck} (j5_stuck={stuck_j5})")
            # if stuck, return to lower J5 first before continuing
            recover_target = list(base); recover_target[4] = max(0, j5 - 30)
            print(f"    recovering to J5={recover_target[4]}...")
            rc, rr = move_to(recover_target, speed=15)
            time.sleep(0.5)
        else:
            tag = "OK" if ok else f"NG http={code}"
            print(f"  J5 target={j5:7.2f}  actual={actual_j5:7.2f}  delta={actual_j5-j5:+.2f}°  {tag}")
        time.sleep(0.3)

def main():
    # Make sure we start from HOME-ish (J5=0)
    print("[init] verifying HOME...")
    a = angles()
    if abs(a[4]) > 5:
        print(f"  J5 is {a[4]:.1f}°, returning to HOME first...")
        move_to([0,0,-90,0,0,0], speed=15, rescue=abs(a[4])>30)
        time.sleep(2)
    print(f"  current angles: {[round(x,1) for x in angles()]}")

    # === Test A: ramp up from 0 in 10° steps ===
    step_test("Test A: J5 ramp +0 → +90 in 10° steps",
              [10, 20, 30, 40, 50, 60, 70, 75, 80, 90])

    # bring back to safe
    move_to([0,0,-90,0,30,0], speed=15)
    time.sleep(2)

    # === Test B: fine-grain near suspected latch zone ===
    step_test("Test B: fine 70→80 in 2° steps",
              [70, 72, 74, 76, 78, 80])

    move_to([0,0,-90,0,30,0], speed=15)
    time.sleep(2)

    # === Test C: negative J5 direction (does it also latch?) ===
    step_test("Test C: J5 ramp 0 → -90 in 10° steps",
              [-10, -20, -30, -40, -50, -60, -70, -75, -80, -90])

    move_to([0,0,-90,0,0,0], speed=15)
    print("\n[done] returned to HOME")

if __name__ == "__main__":
    main()
