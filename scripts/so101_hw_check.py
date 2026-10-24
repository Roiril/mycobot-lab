"""SO-101 hardware check: run this first whenever the arms are (re)connected.

    .venv-so101\\Scripts\\python.exe scripts\\so101_hw_check.py               # stages 0-2, no torque
    .venv-so101\\Scripts\\python.exe scripts\\so101_hw_check.py --characterize # + stage 3 (moves ±1.5 deg)
    py -3.10 scripts\\so101_hw_check.py --sim --characterize                  # same on the simulated arm

Stage 0  ports: which COM is the follower / leader (measured, ports.py)
Stage 1  instrument check, torque untouched. Does the telemetry the governor
         acts on agree with the servos? (calibrate the meter before reading it)
         - cycle time of one telemetry read + one leader read, 300 times
         - a sync read with a non-existent ID in the middle: do the servos after
           it still answer? (firmware behaviour the fault tolerance relies on)
         - one-shot positions vs single-register reads, voltage / temperature /
           current plausible, no status bits at rest
Stage 2  servo settings audit, read only: P/D/I, protection thresholds, voltage
         and temperature limits, firmware, position limits - on both arms
Stage 3  --characterize, torque ON, the operator stands by the arm: for each
         gravity-loaded joint, step the goal +-1.5 deg around the present pose
         and measure the position-loop gain k (duty permille per tick), the
         sign of the load against the gravity model, and the model's scale.
         Writes data/so101_servo_params.json (the input for gravity
         compensation, v2 of the governor).

Nothing here writes EEPROM. Stage 3 leaves the follower energized, holding
where it started.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import statistics
import sys
import time
from typing import Dict, List, Optional

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import gravity, servo_config, sts3215  # noqa: E402
from robots.so101.servo_io import Command, RawFeetechIO, build_sync_read, wrap_bus  # noqa: E402
from robots.so101.teleop_engine import JOINTS, load_ranges  # noqa: E402

PARAMS_PATH = ROOT / "data" / "so101_servo_params.json"
GRAVITY_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def say(msg: str) -> None:
    print(msg, flush=True)


class Report:
    def __init__(self):
        self.ok = True
        self.lines: List[str] = []
        self.data: dict = {}

    def check(self, cond: bool, good: str, bad: str) -> bool:
        line = ("  OK   " + good) if cond else ("  NG   " + bad)
        self.lines.append(line)
        say(line)
        if not cond:
            self.ok = False
        return cond

    def note(self, msg: str) -> None:
        line = "  ..   " + msg
        self.lines.append(line)
        say(line)


# ---------------------------------------------------------------------------
def stage1(foll, lead, rep: Report, cycles: int = 300) -> None:
    say("\n[1] 計器の確認（トルクには触りません）")
    times = []
    missing = 0
    for _ in range(cycles):
        t0 = time.perf_counter()
        st = foll.read_telemetry()
        if lead is not None:
            lead.read_positions()
        times.append((time.perf_counter() - t0) * 1000.0)
        missing += sum(1 for s in st.values() if not s.ok)
    times.sort()
    p50, p99 = times[len(times) // 2], times[int(len(times) * 0.99)]
    rep.data["cycle_ms"] = {"p50": round(p50, 2), "p99": round(p99, 2)}
    rep.check(p99 < 12.0, f"読み出し 1 周期 p50 {p50:.1f} ms / p99 {p99:.1f} ms（60 Hz に収まる）",
              f"読み出し 1 周期 p99 {p99:.1f} ms が長すぎます（60 Hz の周期は 16.7 ms）")
    rep.check(missing == 0, f"{cycles} 周期で欠けた読み出し 0",
              f"{cycles} 周期で {missing} 回の欠け（配線・電源を確認）")

    # a missing ID in the middle of a sync read
    if isinstance(foll, RawFeetechIO):
        ids = [foll.ids[n] for n in JOINTS]
        probe_ids = ids[:2] + [0x20] + ids[2:]
        got = foll._transact(build_sync_read(sts3215.TELEMETRY_ADDR, sts3215.TELEMETRY_LEN,
                                             probe_ids), probe_ids, sts3215.TELEMETRY_LEN)
        after = [i for i in ids[2:] if i in got]
        rep.data["missing_id_tolerance"] = len(after) == len(ids) - 2
        rep.check(len(after) == len(ids) - 2,
                  "途中に存在しない ID があっても、後ろのサーボは応答する",
                  f"存在しない ID の後ろで応答したのは {len(after)}/{len(ids) - 2} 台"
                  "（1 台の故障で後ろが読めなくなる。周期は下がるが制御は続く）")
    else:
        rep.note("生のシリアルではないので、欠けた ID の試験は省略")

    st = foll.read_telemetry()
    for n in JOINTS:
        s = st[n]
        if not rep.check(s.ok, f"{n}: 応答あり", f"{n}: 応答なし"):
            continue
        single, _ = foll.read_reg("Present_Position", n)
        rep.check(single is not None and abs(single - s.pos) <= 8,
                  f"{n}: 位置 {s.pos}（単独読み出し {single}）",
                  f"{n}: 一括読み出しの位置 {s.pos} と単独読み出し {single} が食い違う")
        rep.check(9.0 <= s.volt <= 14.0, f"{n}: 電圧 {s.volt:.1f}V",
                  f"{n}: 電圧 {s.volt:.1f}V（12V が来ていない可能性）")
        rep.check(5 <= s.temp <= 60, f"{n}: 温度 {s.temp}℃", f"{n}: 温度 {s.temp}℃ が範囲外")
        rep.check(not s.faults, f"{n}: 異常ビットなし",
                  f"{n}: 異常ビット {sts3215.describe_faults(s.faults)}")
        if abs(s.vel) < 20:
            rep.check(abs(s.current) < 0.6, f"{n}: 静止中の電流 {abs(s.current):.2f}A",
                      f"{n}: 静止中なのに {abs(s.current):.2f}A 流れている（押し付け・単位を確認）")


def stage2(foll, lead, rep: Report) -> None:
    say("\n[2] サーボ設定の確認（読むだけ）")
    if getattr(lead, "path", None) is not None:      # the scripted sim leader has no registers
        lead = None
    keys = ["Firmware_Major", "Firmware_Minor", "P_Coefficient", "D_Coefficient",
            "I_Coefficient", "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
            "Overload_Torque", "Protective_Torque", "Protection_Time",
            "Over_Current_Protection_Time", "Max_Temperature_Limit", "Max_Voltage_Limit",
            "Min_Voltage_Limit", "Unloading_Condition", "Min_Position_Limit",
            "Max_Position_Limit", "Acceleration", "Phase", "Lock"]
    for arm, io in (("follower", foll), ("leader", lead)):
        if io is None:
            continue
        table = servo_config.audit(io.read_reg, JOINTS)
        rep.data.setdefault("audit", {})[arm] = table
        say(f"  {arm}")
        say("    " + " ".join(f"{k[:10]:>10s}" for k in ["servo"] + keys))
        for n in JOINTS:
            row = table[n]
            say("    " + " ".join(f"{str(v):>10s}" for v in [n[:10]] + [row.get(k) for k in keys]))
        supply = 12.2 if arm == "follower" else 7.4
        for f in servo_config.findings(table, supply_v=supply, role=arm):
            line = f"{arm}/{f.servo}: {f.message}"
            if f.level == "info":
                rep.note(line)
            else:
                rep.check(False, "", line)


# ---------------------------------------------------------------------------
def _settle(foll, n: str, seconds: float = 1.2) -> Dict[str, float]:
    """Average telemetry of joint n after it has settled."""
    time.sleep(seconds * 0.6)
    pos, load, cur = [], [], []
    t_end = time.perf_counter() + seconds * 0.4
    while time.perf_counter() < t_end:
        s = foll.read_telemetry([n])[n]
        if s.ok:
            pos.append(s.pos)
            load.append(s.load)
            cur.append(abs(s.current))
        time.sleep(0.02)
    return {"pos": statistics.mean(pos), "load": statistics.mean(load),
            "current": max(cur) if cur else 0.0}


def stage3(foll, ranges, rep: Report, joints: List[str], step_deg: float = 1.5) -> dict:
    say("\n[3] 特性の測定（トルクを入れて ±{:.1f}° 動かします。腕のそばにいてください）".format(step_deg))
    st = foll.read_telemetry()
    if not all(st[n].ok for n in JOINTS):
        rep.check(False, "", "全サーボが応答していないので測定しません")
        return {}
    settings = servo_config.SessionSettings()
    for reg, n, v in servo_config.session_writes(settings, JOINTS, goal_velocity=200):
        foll.write_reg(reg, n, v)
    start = {n: st[n].pos for n in JOINTS}
    foll.write_commands({n: Command(start[n], 200, 20, 1000 if n != "gripper" else 500)
                         for n in JOINTS})
    for n in JOINTS:
        foll.sync_write("Torque_Enable", {n: 1})
        time.sleep(0.05)
    time.sleep(1.0)
    pose = gravity.pose_from_ticks({n: start[n] for n in JOINTS[:5]}, ranges)
    taus = dict(zip(JOINTS[:5], gravity.holding_torques(pose)))
    out = {"date": time.strftime("%Y-%m-%dT%H:%M:%S"), "p": settings.p,
           "pose_deg": [round(a, 1) for a in pose], "joints": {}}
    step = int(round(sts3215.deg_to_ticks(step_deg)))
    for n in joints:
        res = {}
        samples = []
        for off in (0, +step, -step, 0):
            goal = start[n] + off
            foll.write_commands({n: Command(goal, 200, 20, 1000)})
            m = _settle(foll, n)
            s_now = foll.read_telemetry([n])[n]
            if s_now.faults or m["current"] > 1.2:
                rep.check(False, "", f"{n}: 測定中に {m['current']:.2f}A / "
                          f"{sts3215.describe_faults(s_now.faults)} — 中止します")
                foll.write_commands({n: Command(start[n], 200, 20, 1000)})
                break
            samples.append((goal - m["pos"], m["load"]))
        foll.write_commands({n: Command(start[n], 200, 20, 1000)})
        if len(samples) < 3:
            continue
        errs = [e for e, _ in samples]
        loads = [l for _, l in samples]
        de = max(errs) - min(errs)
        k = (loads[errs.index(max(errs))] - loads[errs.index(min(errs))]) / de if de else 0.0
        tau = taus.get(n, 0.0)
        pred = tau / sts3215.STALL_TORQUE_NM * 1000.0
        hold_load = samples[0][1]
        res = {"k_duty_per_tick": round(k, 2), "k_expected": sts3215.k_duty_per_tick(settings.p),
               "hold_error_ticks": round(samples[0][0], 1), "hold_load": round(hold_load),
               "gravity_pred_load": round(pred), "gravity_sign_agrees": (hold_load * pred) > 0,
               "gravity_scale": round(abs(hold_load) / abs(pred), 2) if abs(pred) > 20 else None}
        out["joints"][n] = res
        rep.note(f"{n}: k = {k:.2f}‰/tick（P/4 の想定 {res['k_expected']:.2f}）, "
                 f"保持の誤差 {res['hold_error_ticks']} tick, 負荷 {hold_load:+.0f}‰ "
                 f"（重力モデル {pred:+.0f}‰, 比 {res['gravity_scale']}）")
        if abs(pred) > 60:
            rep.check(res["gravity_sign_agrees"], f"{n}: 負荷の向きが重力モデルと一致",
                      f"{n}: 負荷の向きが重力モデルと逆（校正の向きか角度の写像を確認）")
    say("  フォロワーはトルク ON のまま開始姿勢を保持しています")
    return out


def move_to_pose(bus, ranges, pose_text: str, rep: Report, timeout_s: float = 25.0) -> bool:
    """Energize and move to a pose through the real engine (governor on, slow)."""
    from robots.so101.teleop_engine import TeleopConfig, TeleopEngine
    deg = [float(v) for v in pose_text.split(",")]
    say(f"\n[2.5] 姿勢 {deg} へゆっくり移動します（おだやか 30°/s・ガバナ有効）")
    eng = TeleopEngine(None, bus, ranges, config=TeleopConfig(speed="slow", torque_stagger_s=0.05),
                       log=lambda m: say("  .. " + m))
    eng.sync_torque_state()
    eng.configure_follower()
    eng.enable_follower_torque_staged()
    names = JOINTS[:5]
    target = {}
    for n, a in zip(names, deg):
        lo, hi = ranges[n]
        target[n] = eng.governor.clamp_goal(n, (lo + hi) / 2 + a * 4095 / 360)
        eng.hold_target[n] = target[n]
    t0 = time.perf_counter()
    last = ""
    while time.perf_counter() - t0 < timeout_s:
        res = eng.cycle()
        st = res["follower"]
        msg = res["governor"].message
        if msg and msg != last:
            say("  .. " + msg)
            last = msg
        err = max(abs(st[n].pos - target[n]) for n in names if st[n].ok)
        if err < sts3215.deg_to_ticks(2.0) and all(abs(st[n].vel) < 20 for n in names):
            break
        time.sleep(1 / 30)
    st = eng.follower.read_telemetry()
    errs = {n: round((st[n].pos - target[n]) * 360 / 4095, 1) for n in names}
    cur = {n: round(abs(st[n].current), 2) for n in names}
    states = {n: res["governor"].joints[n].state for n in names}
    rep.note(f"到達誤差（°、重力のたわみ込み）{errs}")
    rep.note(f"電流 A {cur} / 状態 {states}")
    return max(abs(v) for v in errs.values()) < 8.0


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--follower-port", default="auto")
    ap.add_argument("--leader-port", default="auto")
    ap.add_argument("--characterize", action="store_true", help="stage 3: energize and step joints")
    ap.add_argument("--joints", default=",".join(GRAVITY_JOINTS))
    ap.add_argument("--sim", action="store_true", help="run against the simulated arm")
    ap.add_argument("--pose", default="",
                    help="before stage 3, move slowly (governed) to these 5 arm angles in deg, "
                         "e.g. 0,0,0,0,0 = upper arm up, forearm level (gravity-loaded)")
    ap.add_argument("--out", default=str(ROOT / "data" / "so101_hw_check.json"))
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")
    rep = Report()

    say("[0] ポートの判別")
    buses = []
    if args.sim:
        from robots.so101.sim.servo_sim import ScriptedLeaderBus, SimArm, SimFollowerBus
        ranges = {"shoulder_pan": (874, 3159), "shoulder_lift": (815, 3145),
                  "elbow_flex": (859, 3065), "wrist_flex": (911, 3193),
                  "wrist_roll": (0, 4095), "gripper": (1605, 3086)}
        arm = SimArm(ranges=ranges, start_deg=[0, 20, -60, 0, 0])
        foll = SimFollowerBus(arm, realtime=True)
        foll.sync_write("Torque_Enable", {n: 1 for n in JOINTS})
        lead = ScriptedLeaderBus(ranges, lambda t: {})
        rep.note("シミュレーション（実機ではありません）")
        source = "sim"
    else:
        from robots.so101 import ports as so101_ports
        from robots.so101.teleop_engine import make_bus
        ranges = load_ranges(say)
        if args.leader_port == "none":
            # follower alone: identify it by its 12 V supply, refuse anything else
            probe = so101_ports.probe_port(args.follower_port)
            rep.check(probe.ok and probe.volt >= so101_ports.FOLLOWER_MIN_VOLT,
                      f"follower={args.follower_port}（{probe.volt:.1f}V, {len(probe.motors)} 台）",
                      f"{args.follower_port} はフォロワーと確認できません（{probe.volt}V {probe.error}）")
            if not (probe.ok and probe.volt >= so101_ports.FOLLOWER_MIN_VOLT):
                return 1
            fb = make_bus(args.follower_port)
            fb.connect(handshake=False)
            buses = [fb]
            foll, lead = wrap_bus(fb, JOINTS), None
        else:
            roles = so101_ports.wait_for_ports(args.leader_port, args.follower_port, log=say,
                                               timeout=30.0)
            rep.check(roles.confirmed, f"follower={roles.follower} leader={roles.leader}（実測）",
                      f"follower={roles.follower} leader={roles.leader}（{roles.source}: 実測ではない）")
            fb = make_bus(roles.follower)
            fb.connect(handshake=False)
            lb = make_bus(roles.leader)
            lb.connect(handshake=False)
            buses = [fb, lb]
            foll, lead = wrap_bus(fb, JOINTS), wrap_bus(lb, JOINTS)
        source = "hw"
    try:
        stage1(foll, lead, rep)
        stage2(foll, lead, rep)
        params = None
        if args.pose:
            move_to_pose(buses[0] if buses else foll, ranges, args.pose, rep)
        if args.characterize:
            joints = [j for j in args.joints.split(",") if j in JOINTS]
            params = stage3(foll, ranges, rep, joints)
            if params and source == "hw":
                params["source"] = "hw"
                PARAMS_PATH.write_text(json.dumps(params, ensure_ascii=False, indent=2),
                                       encoding="utf-8")
                say(f"  → {PARAMS_PATH}")
            elif params:
                say("  （シミュレーションの測定値はファイルに書きません）")
                rep.data["characterize_sim"] = params
    finally:
        for b in buses:
            try:
                b.disconnect(disable_torque=False)
            except Exception:
                pass

    rep.data["source"] = source
    rep.data["ok"] = rep.ok
    rep.data["date"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    pathlib.Path(args.out).write_text(json.dumps(rep.data, ensure_ascii=False, indent=2,
                                                 default=str), encoding="utf-8")
    say(f"\n結果: {'すべて OK' if rep.ok else 'NG あり（上の NG 行を確認）'} → {args.out}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
