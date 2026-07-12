"""SO-101 leader->follower teleoperation (run with .venv-so101 / Python 3.12).

Wraps lerobot's SO101Leader / SO101Follower in a loop equivalent to
lerobot-teleoperate, with brownout mitigations for the current 12V 2A supply:
full-torque inrush on several servos at once sags the rail and drops the whole
Feetech bus ("There is no status packet!"), so we cap Torque_Limit and
Acceleration on every follower servo and clamp per-cycle steps with
max_relative_target. Transient bus errors are retried instead of crashing.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_teleop.py
Stop: Ctrl+C (follower torque is released on exit)
"""
from __future__ import annotations
import argparse
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

TORQUE_LIMIT = 500      # /1000 — enough to track, caps inrush current
ACCELERATION = 30       # gentle ramp, avoids current spikes
GOAL_VELOCITY = 800     # raw units; fast enough for live tracking


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader-port", default="COM14")
    ap.add_argument("--follower-port", default="COM13")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--max-step", type=float, default=15.0,
                    help="max joint move per cycle, in normalized units (deg)")
    args = ap.parse_args()

    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id="so101_leader"))
    follower = SO101Follower(SO101FollowerConfig(
        port=args.follower_port, id="so101_follower",
        max_relative_target=args.max_step))
    leader.connect(calibrate=False)
    follower.connect(calibrate=False)

    for n in follower.bus.motors:
        follower.bus.write("Torque_Limit", n, TORQUE_LIMIT, normalize=False)
        follower.bus.write("Acceleration", n, ACCELERATION, normalize=False)
        follower.bus.write("Goal_Velocity", n, GOAL_VELOCITY, normalize=False)

    print(f"[teleop] leader={args.leader_port} follower={args.follower_port} "
          f"fps={args.fps} max_step={args.max_step}deg torque_limit={TORQUE_LIMIT}")
    period = 1.0 / args.fps
    errors = 0
    try:
        while True:
            t0 = time.perf_counter()
            try:
                action = leader.get_action()
                follower.send_action(action)
                errors = 0
            except ConnectionError as e:
                # transient bus dropout (supply sag) — back off and retry
                errors += 1
                print(f"[teleop] bus error ({errors}): {e}")
                if errors >= 10:
                    raise
                time.sleep(0.5)
                continue
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[teleop] stopping")
    finally:
        try:
            follower.bus.disable_torque()
        except Exception:
            pass
        try:
            follower.disconnect()
        except Exception:
            pass
        try:
            leader.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
