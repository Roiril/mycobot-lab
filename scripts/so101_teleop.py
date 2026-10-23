"""SO-101 leader->follower teleoperation CLI (run with .venv-so101 / Python 3.12).

Thin wrapper over src/robots/so101/teleop_engine.py - the same engine the
cockpit server uses (servo_io telemetry, governor, write-on-change), so both run
one motion pipeline. Prints the governor's operator message whenever it changes.

Ports are identified by measurement (src/robots/so101/ports.py) rather than by
number, and the run waits for the leader to be parked on the follower's pose
before engaging, so following starts from agreement instead of a snap.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_teleop.py
Stop: Ctrl+C (the follower relaxes tip -> base on exit: support the arm)
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import servo_config  # noqa: E402
from robots.so101 import ports as so101_ports  # noqa: E402
from robots.so101.teleop_engine import (  # noqa: E402
    AUTO_FOLLOW_TOL_DEG, TICKS_PER_DEG, AutoFollowGate, OneEuroConfig, TeleopConfig,
    TeleopEngine, deg_to_ticks, load_ranges, make_bus,
)


def _connect(leader_port: str, follower_port: str):
    leader = make_bus(leader_port)
    follower = make_bus(follower_port)
    leader.connect(handshake=False)
    follower.connect(handshake=False)
    return leader, follower


def _wait_for_pose_match(engine: TeleopEngine, gate: AutoFollowGate) -> None:
    """Hold until the leader is parked on the follower's pose (prints the worst
    joint so the operator knows which way to move the leader)."""
    last = ""
    while True:
        lead = {n: v for n, v in engine.leader.read_positions().items() if v is not None}
        foll = {n: s.pos for n, s in engine.follower.read_telemetry().items() if s.ok}
        if gate.update(lead, foll):
            print(f"[teleop] {gate.status} — 追従開始")
            return
        if gate.status != last:
            print(f"[teleop] {gate.status}")
            last = gate.status
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader-port", default="auto",
                    help="COM port of the leader arm, or 'auto' to identify it")
    ap.add_argument("--follower-port", default="auto",
                    help="COM port of the follower arm, or 'auto' to identify it")
    ap.add_argument("--fps", type=float, default=60.0, help="target loop rate (Hz)")
    ap.add_argument("--max-step", type=float, default=15.0,
                    help="max reference jump per cycle, in degrees (glitch guard)")
    ap.add_argument("--min-cutoff", type=float, default=1.0,
                    help="One-Euro min cutoff (Hz); lower = smoother at rest")
    ap.add_argument("--beta", type=float, default=0.01,
                    help="One-Euro beta; higher = less lag when moving fast")
    ap.add_argument("--speed", choices=list(servo_config.SPEED_PRESETS), default="normal")
    ap.add_argument("--governor", choices=["active", "shadow"], default="active",
                    help="shadow: only report what the governor would do")
    ap.add_argument("--tol-deg", type=float, default=AUTO_FOLLOW_TOL_DEG,
                    help="per-joint agreement required before follow starts")
    ap.add_argument("--no-pose-gate", action="store_true",
                    help="start following immediately, however far apart the arms are")
    args = ap.parse_args()

    cfg = TeleopConfig(
        target_hz=args.fps,
        max_step_ticks=int(round(args.max_step * TICKS_PER_DEG)),
        speed=args.speed, governor_mode=args.governor,
        one_euro=OneEuroConfig(min_cutoff=args.min_cutoff, beta=args.beta),
    )
    say = lambda m: print(f"[teleop] {m}")            # noqa: E731
    ranges = load_ranges(log=say)

    # Identify the arms by measurement — the two boards share a USB id, so a
    # fixed COM pair can silently swap and drive the leader.
    roles = so101_ports.wait_for_ports(args.leader_port, args.follower_port, log=say)
    leader, follower = _connect(roles.leader, roles.follower)
    engine = TeleopEngine(leader, follower, ranges, config=cfg, log=say)
    engine.sync_torque_state()
    p = engine.configure_follower()
    print(f"[teleop] P={p}  計器チェック={'OK' if engine.instrument_ok else 'NG ' + engine.instrument_detail}")
    if not args.no_pose_gate:
        _wait_for_pose_match(engine, AutoFollowGate(tol_ticks=deg_to_ticks(args.tol_deg)))
    ok, why = engine.start_teleop()
    if not ok:
        print(f"[teleop] 追従を開始できません: {why}")
        return 1

    print(f"[teleop] leader={roles.leader} follower={roles.follower} target={args.fps}Hz "
          f"speed={args.speed} governor={engine.governor.mode}")

    period = cfg.period
    errors = 0
    last_msg = ""
    try:
        while True:
            t0 = time.perf_counter()
            try:
                res = engine.cycle(now=t0)
                errors = 0
                msg = res["governor"].message
                if msg != last_msg:
                    if msg:
                        print(f"[teleop] {msg}")
                    last_msg = msg
                if not engine.active and not engine.retreating:
                    print("[teleop] 追従が止まりました（上のメッセージを確認）。Ctrl+C で終了")
                    while True:
                        time.sleep(period)
                        engine.cycle()
            except ConnectionError as e:
                # transient bus dropout — back off, reconnect, restart following
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
                    leader, follower = _connect(roles.leader, roles.follower)
                    engine.rebind(leader, follower)
                    engine.sync_torque_state()
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
        print(f"\n[teleop] stopping (measured {engine.measured_hz} Hz) — 脱力します")
    finally:
        try:
            engine.relax_follower()
        except Exception:
            pass
        for b in (follower, leader):
            try:
                b.disconnect(disable_torque=False)
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
