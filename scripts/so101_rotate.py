"""Rotate ONE SO-101 servo by a relative angle — bench helper for fixing horn
orientation before/while assembling. Single servo on the bus, raw-tick control
(no calibration needed).

STS3215: 4096 ticks = 360 deg. Reads Present_Position, commands
Goal_Position = present + deg, holds (torque stays ON) unless --release.

Usage:
  python scripts/so101_rotate.py --port COM13 --id 1 --deg 45     # read only if --deg 0
  python scripts/so101_rotate.py --port COM13 --id 1 --deg -45
  python scripts/so101_rotate.py --port COM13 --id 1 --release    # just disable torque
"""
from __future__ import annotations
import argparse, time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

TICKS_PER_REV = 4096


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--deg", type=float, default=0.0, help="relative degrees (+/-); 0 = read only")
    ap.add_argument("--release", action="store_true", help="disable torque and exit")
    ap.add_argument("--speed", type=int, default=300, help="goal speed (lower = gentler)")
    args = ap.parse_args()

    bus = FeetechMotorsBus(args.port, {"m": Motor(args.id, "sts3215", MotorNormMode.DEGREES)})
    bus.connect()
    try:
        present = bus.read("Present_Position", "m", normalize=False)
        print(f"present_position = {present} ticks ({present * 360.0 / TICKS_PER_REV:.1f} deg)")

        if args.release:
            bus.disable_torque("m")
            print("torque DISABLED (脱力)。手で自由に回せます。")
            return
        if args.deg == 0.0:
            print("(--deg 0: 読み取りのみ。回すなら --deg 45 等)")
            return

        delta = round(args.deg * TICKS_PER_REV / 360.0)
        goal = present + delta
        try:
            bus.write("Goal_Velocity", "m", args.speed, normalize=False)
        except Exception:
            pass  # not all firmwares expose it; ignore
        bus.enable_torque("m")
        bus.write("Goal_Position", "m", goal, normalize=False)
        print(f"rotating {args.deg:+.1f} deg ({delta:+d} ticks): {present} -> {goal} ...")
        time.sleep(1.2)
        now = bus.read("Present_Position", "m", normalize=False)
        print(f"now = {now} ticks ({now * 360.0 / TICKS_PER_REV:.1f} deg). torque は ON のまま保持中。")
        print("  逆向きにしたい → --deg で反対符号 / 脱力 → --release")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
