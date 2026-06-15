"""Two-stage SO-101 calibration — the non-interactive core of `lerobot-calibrate`,
driven by external cues instead of ENTER prompts.

`lerobot-calibrate` is interactive: it input()s for "pose to middle" (homing) and
streams positions until ENTER (range recording). Both moments must sync with the
human moving the arm. This splits the SAME work (bus.set_half_turn_homings +
range capture + robot._save_calibration to the standard lerobot path) into two
cue-able stages:

  --stage home    : disable torque, set POSITION mode, set half-turn homing
                    (call this once the arm is hand-posed to a neutral middle).
  --stage ranges  : record min/max of every joint (except wrist_roll = full turn)
                    over a fixed time window while you sweep each joint by hand,
                    then build + WRITE + SAVE the calibration file.

Result is saved where SO101Follower(calibrate=False) loads it, so the real driver
in scripts/server.py (--so101-driver real) picks it up automatically.

Usage:
  python scripts/so101_calibrate.py --port COM13 --id so101_follower_01 --stage home
  python scripts/so101_calibrate.py --port COM13 --id so101_follower_01 --stage ranges --seconds 45
"""
from __future__ import annotations
import sys, time, argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode

FULL_TURN = "wrist_roll"  # treated as 0..4095 (no manual sweep needed)


def out(msg: str) -> None:
    sys.stdout.write(msg + "\n"); sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", default="so101_follower_01")
    ap.add_argument("--stage", choices=["home", "ranges"], required=True)
    ap.add_argument("--seconds", type=float, default=45.0, help="range recording window (ranges stage)")
    args = ap.parse_args()

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    bus = robot.bus
    bus.connect()
    try:
        if args.stage == "home":
            # Sign-safe half-turn homing. lerobot's set_half_turn_homings() reads
            # Present_Position as SIGNED for some joints (e.g. elbow read as -2051),
            # so its offset = 2048 - signed can exceed the ±2047 register limit and
            # raises "Magnitude ... exceeds 2047". We convert to UNSIGNED first and
            # wrap the offset into [-2047, 2047] so every joint centers cleanly.
            bus.disable_torque()
            bus.reset_calibration()
            for m in bus.motors:
                bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
            # Lock the held neutral with torque (goal=present first to avoid any
            # jerk) so the limp arm cannot sag during the homing read.
            p0 = bus.sync_read("Present_Position", normalize=False, num_retry=3)
            for m in bus.motors:
                bus.write("Goal_Position", m, p0[m], normalize=False)
            bus.enable_torque()
            time.sleep(0.5)
            pres = bus.sync_read("Present_Position", normalize=False, num_retry=3)  # stable, held
            bus.disable_torque()  # required for EEPROM (Homing_Offset) write
            offsets = {}
            for m in bus.motors:
                u = pres[m] % 4096  # unsigned 0..4095
                off = ((2048 - u + 2048) % 4096) - 2048  # wrap into [-2048, 2047]
                off = max(-2047, min(2047, off))
                offsets[m] = off
                bus.write("Homing_Offset", m, off)
            chk = bus.sync_read("Present_Position", normalize=False)
            out("HOME_OK offsets=" + str(offsets))
            out("centered_present=" + str({m: int(chk[m]) for m in bus.motors}) + " (≈2048 が理想)")
            return

        # --- ranges ---
        bus.disable_torque()  # arm limp so you can sweep by hand
        rec = [m for m in bus.motors if m != FULL_TURN]
        start = bus.sync_read("Present_Position", rec, normalize=False, num_retry=3)
        mins = dict(start); maxes = dict(start)
        out(f"RECORDING {args.seconds:.0f}s 開始 — 今すぐ各関節を端から端までゆっくり振って！(wrist_roll は不要)")
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < args.seconds:
            p = bus.sync_read("Present_Position", rec, normalize=False, num_retry=3)
            for m in rec:
                if p[m] < mins[m]: mins[m] = p[m]
                if p[m] > maxes[m]: maxes[m] = p[m]
            now = time.time()
            if now - last >= 5.0:
                last = now
                rem = args.seconds - (now - t0)
                spans = {m: int(maxes[m] - mins[m]) for m in rec}
                out(f"  残り{rem:4.0f}s  可動幅(ticks)={spans}")
            time.sleep(0.03)

        homing = {m: int(bus.read("Homing_Offset", m, normalize=False)) for m in bus.motors}
        range_min = {m: int(mins[m]) for m in rec}
        range_max = {m: int(maxes[m]) for m in rec}
        range_min[FULL_TURN] = 0
        range_max[FULL_TURN] = 4095

        cal = {}
        for m, mt in bus.motors.items():
            cal[m] = MotorCalibration(
                id=mt.id, drive_mode=0,
                homing_offset=homing[m],
                range_min=range_min[m], range_max=range_max[m],
            )
        robot.calibration = cal
        bus.write_calibration(cal)
        robot._save_calibration()
        out("RANGES_OK 各関節 range:")
        for m in bus.motors:
            out(f"  {m:14} min={cal[m].range_min:4d} max={cal[m].range_max:4d} home={cal[m].homing_offset}")
        out("SAVED " + str(robot.calibration_fpath))
    finally:
        try:
            bus.disable_torque()
        except Exception:
            pass
        try:
            bus.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
