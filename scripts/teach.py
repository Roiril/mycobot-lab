"""Lead-through teaching helper for pose registration.

Drag-teach workflow (per-invocation, torque state persists across serial close):

    python scripts/teach.py status          # connection + current angles
    python scripts/teach.py release          # torque OFF — arm goes LIMP (hold it!)
    python scripts/teach.py read [NAME]      # read current angles (poses.py-ready line)
    python scripts/teach.py hold             # re-power: lock servos at current pose

Safe ordering (avoids the firmware overload latch — see mycobot_firmware_quirks):
    release  →  (you pose the arm by hand)  →  read  →  hold

⚠️ `release` makes the arm limp; gravity pulls J2/J3 down. Support the arm by
   hand BEFORE running release, and keep holding until after `hold`.
"""
import sys, pathlib, time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from arm import Arm


def _read_angles(a, tries: int = 5):
    """get_angles can return [] / junk on the first poll — retry a few times."""
    for _ in range(tries):
        ang = a.angles()
        if isinstance(ang, (list, tuple)) and len(ang) == 6:
            return [round(float(v), 1) for v in ang]
        time.sleep(0.2)
    return None


def main():
    cmd = (sys.argv[1] if len(sys.argv) > 1 else "status").lower()
    name = sys.argv[2] if len(sys.argv) > 2 else "POSE"

    a = Arm()
    print("port    :", a.port)

    if cmd == "status":
        print("powered :", a.mc.is_power_on())
        print("angles  :", _read_angles(a))
        print("coords  :", a.coords())

    elif cmd == "release":
        a.release()
        print("released: torque OFF — ARM IS LIMP. Keep holding it.")

    elif cmd == "read":
        ang = _read_angles(a)
        if ang is None:
            print("read    : FAILED (no valid angles) — retry")
        else:
            print("angles  :", ang)
            print(f"poses   : {name} = {ang}")

    elif cmd == "hold":
        a.power_on()
        print("held    : powered ON, locked at", _read_angles(a))

    else:
        print(f"unknown command: {cmd!r}")
        print(__doc__)

    a.close()


if __name__ == "__main__":
    main()
