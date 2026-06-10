"""Assign ONE SO-101 servo's ID + baudrate — the non-interactive core of
`lerobot-setup-motors`, callable one motor at a time.

The official `lerobot-setup-motors` is interactive (it input()s between every
motor so you can swap the single connected servo). This script does exactly the
same per-motor work (`bus.setup_motor(name)`: find the lone servo on the bus,
disable torque, write the target ID, set baud to 1 Mbps) but for ONE named
motor, so the swapping stays manual while the register write is scripted.

Rule: exactly ONE servo connected to the bus when you run this.

Usage:
  python scripts/so101_set_id.py --port COM13 gripper
  python scripts/so101_set_id.py --port COM13 wrist_roll
  ... (gripper, wrist_roll, wrist_flex, elbow_flex, shoulder_lift, shoulder_pan)
"""
from __future__ import annotations
import sys, argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ORDER = ["gripper", "wrist_roll", "wrist_flex", "elbow_flex", "shoulder_lift", "shoulder_pan"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("motor", choices=ORDER, help="which joint this single connected servo becomes")
    ap.add_argument("--port", required=True, help="board COM port (e.g. COM13)")
    args = ap.parse_args()

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id="so101_follower"))
    bus = robot.bus
    target_id = bus.motors[args.motor].id
    print(f"[{args.motor}] -> assigning ID {target_id} at 1 Mbps on {args.port} ...")
    try:
        bus.setup_motor(args.motor)
    except Exception as e:
        print(f"FAILED: {e}")
        print("  - servo が1個だけ繋がっているか / 電源 ON か / COM が正しいか確認")
        sys.exit(1)
    finally:
        try:
            if bus.is_connected:
                bus.disconnect()
        except Exception:
            pass
    print(f"OK: '{args.motor}' の ID を {target_id} に設定。このサーボに「{args.motor} (ID{target_id})」と印を付けて。")
    nxt = ORDER.index(args.motor) + 1
    if nxt < len(ORDER):
        print(f"次: このサーボを外して、別のサーボ1個を繋いだら「{ORDER[nxt]} やって」と言って。")
    else:
        print("これで6個すべて完了。次はキャリブレーション (lerobot-calibrate)。")


if __name__ == "__main__":
    main()
