"""SO-101 leader->follower teleoperation CLI (run with .venv-so101 / Python 3.12).

Thin wrapper over src/robots/so101/teleop_engine.py — the same engine the
cockpit server uses, so both share one motion + safety pipeline (One-Euro
smoothing, last-goal stepping, brownout-safe staged torque). This drives the
raw FeetechMotorsBus directly (no lerobot SO101Leader/Follower wrappers) so the
follower path is byte-identical to the cockpit.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_teleop.py
Stop: Ctrl+C (follower torque is released on exit)
"""
from __future__ import annotations
import sys
import time
import pathlib
import argparse

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.teleop_engine import (
    TeleopEngine, TeleopConfig, OneEuroConfig,
    make_bus, load_ranges, TICKS_PER_DEG,
)


def _connect(leader_port: str, follower_port: str):
    leader = make_bus(leader_port)
    follower = make_bus(follower_port)
    leader.connect(handshake=False)
    follower.connect(handshake=False)
    return leader, follower


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader-port", default="COM14")
    ap.add_argument("--follower-port", default="COM13")
    ap.add_argument("--fps", type=float, default=60.0, help="target loop rate (Hz)")
    ap.add_argument("--max-step", type=float, default=15.0,
                    help="max follower move per cycle, in degrees")
    ap.add_argument("--min-cutoff", type=float, default=1.0,
                    help="One-Euro min cutoff (Hz); lower = smoother at rest")
    ap.add_argument("--beta", type=float, default=0.01,
                    help="One-Euro beta; higher = less lag when moving fast")
    ap.add_argument("--torque-profile", choices=["full", "safe"], default="full",
                    help="follower torque caps: full (max, normal) | safe (demoted)")
    args = ap.parse_args()

    cfg = TeleopConfig(
        target_hz=args.fps,
        max_step_ticks=int(round(args.max_step * TICKS_PER_DEG)),
        torque_profile=args.torque_profile,
        one_euro=OneEuroConfig(min_cutoff=args.min_cutoff, beta=args.beta),
    )
    ranges = load_ranges(log=lambda m: print(f"[teleop] {m}"))

    leader, follower = _connect(args.leader_port, args.follower_port)
    engine = TeleopEngine(leader, follower, ranges, config=cfg,
                          log=lambda m: print(f"[teleop] {m}"))
    engine.start_teleop()

    print(f"[teleop] leader={args.leader_port} follower={args.follower_port} "
          f"target={args.fps}Hz max_step={args.max_step}deg "
          f"({cfg.max_step_ticks}tk) min_cutoff={args.min_cutoff} beta={args.beta} "
          f"torque_profile={args.torque_profile}")

    period = cfg.period
    errors = 0
    try:
        while True:
            t0 = time.perf_counter()
            try:
                engine.step(now=t0)
                errors = 0
            except ConnectionError as e:
                # transient bus dropout (supply sag) — back off, reconnect, and
                # restart teleop so filters + last_goal reseed cleanly.
                errors += 1
                print(f"[teleop] bus error ({errors}): {e}")
                if errors >= 10:
                    raise
                engine.active = False
                for b in (leader, follower):
                    try:
                        b.disconnect(disable_torque=False)
                    except Exception:
                        pass
                time.sleep(0.5)
                try:
                    leader, follower = _connect(args.leader_port, args.follower_port)
                    engine.rebind(leader, follower)
                    engine.start_teleop()
                    print("[teleop] reconnected")
                except Exception as e2:
                    print(f"[teleop] reconnect failed: {e2}")
                    time.sleep(1.0)
                continue
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print(f"\n[teleop] stopping (measured {engine.measured_hz} Hz)")
    finally:
        try:
            follower.disable_torque()
        except Exception:
            pass
        for b in (follower, leader):
            try:
                b.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
