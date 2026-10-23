"""SO-101 leader/follower cockpit server (run with .venv-so101 / Python 3.12).

Browser GUI for the leader->follower rig: live state of every motor on both
arms, follow mode (manual or automatic once the arms agree), per-joint jog and
torque for the follower, speed presets, the cooling pose, relax and ABORT.

What runs underneath (src/robots/so101/):
- teleop_engine.TeleopEngine: one control cycle = leader positions + the
  follower's whole telemetry window + governor decision + one sync write of
  what changed
- governor.FollowerGovernor: keeps goals off the end stops, eases off a joint
  that is pressing on something, keeps holding a weight, freezes a joint whose
  firmware protection tripped, opens a breaker on repeated trips, stops and
  cools the arm when it gets hot, slows down on supply sag
- servo_io: fault-tolerant protocol I/O (status bits are data, a silent servo
  costs only itself); the follower and leader ports are used through it only

A single worker thread owns both buses (serial is not thread-safe); HTTP
handlers only enqueue commands and read a shared snapshot. Bus dropouts are
logged and reconnected, not fatal. Both COM ports are identified by measurement
(src/robots/so101/ports.py): the two arms carry the same USB chip.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_cockpit_server.py
      ... --auto-follow      # follow engages by itself once the arms agree
      ... --sim              # simulated arm (MuJoCo + STS3215 model) on :8023
Open: http://localhost:8013/  (simulation: http://localhost:8023/)
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import queue
import sys
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import servo_config  # noqa: E402
from robots.so101 import ports as so101_ports  # noqa: E402
from robots.so101.teleop_engine import (  # noqa: E402
    AUTO_FOLLOW_TOL_DEG, JOINTS, TICKS_PER_DEG, AutoFollowGate, TeleopConfig,
    TeleopEngine, deg_to_ticks, load_ranges, make_bus,
)

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "so101_cockpit.html"

# ---- ✋ Quest hand-teleop one-click launch (fully independent of the serial worker) ----
# The operator lives on this cockpit page and shouldn't have to open the home
# launcher (:8010) just to start the HMD. hand_launch.py is the single source of
# truth for the adb/CDP launch flow (shared with home_server.py). It is imported
# lazily-safe: a failure here must not stop the cockpit from serving SO-101.
#
# ⚠ env: this server runs on .venv-so101 (Python 3.12). hand_launch + adb_util are
# stdlib-only; websocket-client (needed ONLY by the browser/WebXR fallback) is
# absent here, so cdp_navigate raises a clean RuntimeError that the browser-flow
# path reports as an error. The native-VR launch path (monkey) is dependency-free
# and is the normal case, so a headset with the native app installed launches fine.
#
# ⚠ isolation: the adb work runs on its own worker thread and NEVER touches CMDQ or
# the Feetech buses — it only shells out to adb and probes the hand server (:8001).
sys.path.insert(0, str(HERE / "quest"))
try:
    import hand_launch  # noqa: E402
except Exception:  # pragma: no cover - keep the cockpit usable even if import fails
    hand_launch = None

HAND_SERVER_PROBE_URL = "http://127.0.0.1:8001/"
HAND_STATUS_URL = "http://127.0.0.1:8001/hand/status"
# Overall wall-clock cap for the /quest/launch-hand handler (adb/CDP can hang).
QUEST_LAUNCH_DEADLINE_S = 18.0
QUEST_LAUNCH_JOIN_S = 20.0


def probe_once(url: str, timeout: float = 0.8) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return 200 <= r.status < 500
    except Exception:
        return False


def hand_status_proxy(timeout: float = 2.0) -> dict:
    """Server-side proxy to the hand server's /hand/status (:8001). The browser
    can't hit :8001 directly (cross-origin), so the cockpit relays it. Adds
    server_up; on any failure returns a not-up snapshot instead of raising."""
    try:
        with urllib.request.urlopen(HAND_STATUS_URL, timeout=timeout) as r:
            data = json.loads(r.read().decode("utf-8"))
        if isinstance(data, dict):
            data["server_up"] = True
            return data
        return {"server_up": True, "present": False, "connected": False,
                "mcu_alive": False, "error": "unexpected status payload"}
    except Exception as e:
        return {"server_up": False, "present": False, "connected": False,
                "mcu_alive": False, "error": str(e)}


# ja labels for per-device launch actions (mirrors home_server.py). Native = the
# app enters VR on launch (no in-headset tap); browser = the WebXR page still
# needs a "VR 開始" tap once worn.
_ACTION_JA = {
    "launched_native": "VR アプリ起動（被るだけで操作可）",
    "already_native": "VR アプリ起動済み（被るだけで操作可）",
    "navigated": "既存タブを /hand に遷移",
    "already": "すでに /hand 表示中",
    "launched": "アプリ起動（新規タブ）",
    "error": "失敗",
}
_NATIVE_ACTIONS = ("launched_native", "already_native")


def _quest_launch_message(result: dict) -> str:
    """Short human summary for the launch button (mirrors home_server.py)."""
    if not result.get("ok"):
        return result.get("error") or "起動に失敗しました。"
    devices = result.get("devices", [])
    parts = []
    for d in devices:
        tag = _ACTION_JA.get(d.get("action"), d.get("action", "?"))
        detail = f"（{d['detail']}）" if d.get("action") == "error" and d.get("detail") else ""
        parts.append(f"{d.get('serial', '?')[:8]}…: {tag}{detail}")
    actions = [d.get("action") for d in devices]
    if actions and all(a in _NATIVE_ACTIONS for a in actions):
        head = "HMD で VR アプリを起動しました。被るだけで操作できます。"
    elif any(a in _NATIVE_ACTIONS for a in actions):
        head = ("HMD で VR アプリを起動しました（被るだけで操作可）。"
                "ブラウザ版で開いた台はヘッドセット内で「VR 開始」をタップしてください。")
    else:
        head = "HMD でページを開きました。ヘッドセット内で「VR 開始」をタップしてください。"
    return head + ("  " + " / ".join(parts) if parts else "")


# How often (cycles) the leader's slow telemetry (voltage / temperature) is read.
LEADER_TELEMETRY_EVERY = 30
# A follower whose every joint has been silent this many cycles is unplugged.
ARM_DOWN_CYCLES = 30
# Operator messages stay up at least this long, so a passing state is readable.
MESSAGE_HOLD_S = 3.0
CURRENT_PEAK_WINDOW_S = 2.0

LOCK = threading.Lock()
CMDQ: "queue.Queue" = queue.Queue()
STATE = {
    "source": "hw", "connected": False, "teleop": False, "retreating": False,
    "error": "", "log": [], "leader": {}, "follower": {}, "arm": {}, "hz": 0.0,
    "engine": {}, "findings": [], "ranges": {},
    "speed_presets": {k: v["label"] for k, v in servo_config.SPEED_PRESETS.items()},
    "ports": {"leader": "", "follower": "", "source": "", "confirmed": False,
              "detail": "検出中…"},
    "auto_follow": {"enabled": False, "armed": False, "status": "無効",
                    "gap_deg": 0.0, "gap_joint": "", "tol_deg": AUTO_FOLLOW_TOL_DEG},
}


def log(msg: str):
    with LOCK:
        entries = STATE["log"]
        # collapse repeats so one noisy servo cannot flush the useful lines
        if entries and entries[-1].get("raw") == msg:
            entries[-1]["n"] = entries[-1].get("n", 1) + 1
            entries[-1]["msg"] = f"{msg}（×{entries[-1]['n']}）"
            entries[-1]["t"] = time.strftime("%H:%M:%S")
            return
        entries.append({"t": time.strftime("%H:%M:%S"), "msg": msg, "raw": msg})
        del entries[:-60]


# ---------------------------------------------------------------------------
# simulation backend
# ---------------------------------------------------------------------------
SIM_RANGES = {
    "shoulder_pan": (874, 3159), "shoulder_lift": (815, 3145), "elbow_flex": (859, 3065),
    "wrist_flex": (911, 3193), "wrist_roll": (0, 4095), "gripper": (1605, 3086),
}


def sim_demo_path(t: float) -> dict:
    """A 30 s loop for the simulated leader: sweep, press into the table, lift,
    reach out and hold. Shows tracking, HOLD and recovery in the cockpit."""
    t = t % 30.0

    def blend(a, b, t0, t1):
        s = 0.0 if t <= t0 else 1.0 if t >= t1 else (t - t0) / (t1 - t0)
        s = s * s * (3 - 2 * s)
        return {k: a[k] + (b[k] - a[k]) * s for k in a}

    above = {"shoulder_pan": 0.0, "shoulder_lift": -20.0, "elbow_flex": 30.0, "wrist_flex": 60.0}
    pressed = {"shoulder_pan": 0.0, "shoulder_lift": 50.0, "elbow_flex": 10.0, "wrist_flex": 40.0}
    reach = {"shoulder_pan": 0.0, "shoulder_lift": 30.0, "elbow_flex": -40.0, "wrist_flex": 10.0}
    if t < 6.0:
        p = dict(above)
        p["shoulder_pan"] = 25.0 * math.sin(t * math.pi / 3.0)
    elif t < 14.0:
        p = blend(above, pressed, 6.0, 9.0)
    elif t < 18.0:
        p = blend(pressed, above, 14.0, 17.0)
    else:
        p = blend(above, reach, 18.0, 21.0)
        p["shoulder_pan"] = 15.0 * math.sin((t - 18.0) * math.pi / 6.0)
    p["gripper"] = 0.0
    return p


class SimBackend:
    def __init__(self, ranges):
        from robots.so101.sim.servo_sim import ScriptedLeaderBus, SimArm, SimFollowerBus, World
        self.arm = SimArm(ranges=ranges, world=World(table_z_m=0.0),
                          start_deg=[0.0, -20.0, 30.0, 60.0, 0.0])
        self.follower = SimFollowerBus(self.arm, realtime=True)
        self.leader = ScriptedLeaderBus(ranges, sim_demo_path)
        # the arm starts energized, holding its pose: a limp simulated arm would
        # fall through the table (only the gripper collides with it)
        self.follower.sync_write("Torque_Enable", {n: 1 for n in self.follower.names})


# ---------------------------------------------------------------------------
# worker
# ---------------------------------------------------------------------------
def _drain(q: "queue.Queue") -> list:
    """All pending commands, safety first, latest jog per joint only."""
    cmds = []
    while True:
        try:
            cmds.append(q.get_nowait())
        except queue.Empty:
            break
    urgent = [c for c in cmds if c.get("op") in ("abort", "relax")]
    jogs = {}
    rest = []
    for c in cmds:
        op = c.get("op")
        if op == "jog":
            jogs[c.get("joint")] = c
        elif op not in ("abort", "relax"):
            rest.append(c)
    return urgent + rest + list(jogs.values())


def worker(leader_arg: str, follower_arg: str, cfg: TeleopConfig,
           auto_follow: bool = False, tol_deg: float = AUTO_FOLLOW_TOL_DEG,
           stop: threading.Event = None, sim: bool = False):
    stopping = (lambda: False) if stop is None else stop.is_set
    ranges = SIM_RANGES if sim else load_ranges(log)
    backend = SimBackend(ranges) if sim else None
    engine = None
    leader_bus = follower_bus = None
    roles = None
    gate = AutoFollowGate(tol_ticks=deg_to_ticks(tol_deg))
    auto = {"enabled": bool(auto_follow), "armed": bool(auto_follow)}
    leader_view = {n: {"pos": None, "torque": 0, "volt": 0.0, "temp": 0, "stale": True}
                   for n in JOINTS}
    peaks = {n: deque() for n in JOINTS}          # (t, |I|) for the 2 s current peak
    held_msg = {"text": "", "level": "info", "until": 0.0}
    silent_cycles = 0
    cycle = 0

    with LOCK:
        STATE["source"] = "sim" if sim else "hw"
        STATE["ranges"] = ranges

    def publish_auto(status: str = None):
        with LOCK:
            STATE["auto_follow"] = {
                "enabled": auto["enabled"], "armed": auto["armed"],
                "status": status if status is not None else gate.status,
                "gap_deg": round(gate.gap_ticks / TICKS_PER_DEG, 1),
                "gap_joint": gate.gap_joint or "", "tol_deg": tol_deg,
            }

    def disarm_auto(why: str):
        if auto["armed"]:
            auto["armed"] = False
            gate.reset()
            log(f"自動追従を解除（{why}）")
        publish_auto("解除中 — 「追従モード」か「自動追従」で再開")

    publish_auto("待機中" if auto["enabled"] else "無効")

    def ensure_roles():
        nonlocal roles
        if roles is not None:
            return
        if sim:
            roles = so101_ports.Roles(leader="sim", follower="sim", source="sim",
                                      detail="シミュレーション（実機ではありません）")
        else:
            roles = so101_ports.wait_for_ports(leader_arg, follower_arg, log=log, poll=2.0,
                                               should_stop=stopping)
        confirmed = roles.confirmed or sim       # the simulated arms cannot be swapped
        with LOCK:
            STATE["ports"] = {"leader": roles.leader, "follower": roles.follower,
                              "source": roles.source, "confirmed": confirmed,
                              "detail": roles.detail}
        if auto["armed"] and not confirmed:
            # a remembered mapping can be wrong if the cables were swapped, and
            # a swapped mapping drives the LEADER: require a human
            auto["armed"] = False
            log("⚠ ポートを実測で確定できていないため自動追従は開始しません"
                "（役割を確認して手動で ON にしてください）")
            publish_auto("ポート未確定 — 手動で確認してください")

    def connect_all():
        nonlocal leader_bus, follower_bus, engine
        ensure_roles()
        if sim:
            leader_bus, follower_bus = backend.leader, backend.follower
        else:
            leader_bus = make_bus(roles.leader)
            leader_bus.connect(handshake=False)
            follower_bus = make_bus(roles.follower)
            follower_bus.connect(handshake=False)
        if engine is None:
            engine = TeleopEngine(leader_bus, follower_bus, ranges, config=cfg, log=log)
        else:
            engine.rebind(leader_bus, follower_bus)
        # an unplugged follower must fail fast, not after a 180-register audit
        if not any(s.ok for s in engine.follower.read_telemetry().values()):
            raise ConnectionError("follower が応答しません（ケーブル・12V を確認）")
        engine.sync_torque_state()
        engine.configure_follower()
        gate.reset()
        with LOCK:
            STATE["findings"] = [f.__dict__ for f in engine.findings]

    def close_buses():
        if sim:
            return
        for b in (leader_bus, follower_bus):
            try:
                if b is not None:
                    b.disconnect(disable_torque=False)
            except Exception:
                pass

    def handle(cmd: dict):
        op = cmd.get("op")
        if op == "abort":
            engine.stop_teleop()
            log("⛔ ABORT — 追従停止・その場で保持")
            disarm_auto("ABORT")
        elif op == "relax":
            log("🧊 脱力（冷却）— アームを支えてください")
            engine.relax_follower()
            log("脱力完了。復帰は追従ON か各関節トルクスイッチで")
            disarm_auto("脱力")
        elif op == "teleop":
            if bool(cmd.get("on")):
                ok, why = engine.start_teleop()
                if ok:
                    log("追従モード ON")
                    auto["armed"] = False
                    publish_auto("追従中")
                else:
                    log(f"追従を開始できません: {why}")
            else:
                engine.stop_teleop()
                log("追従モード OFF（姿勢保持）")
                disarm_auto("手動 OFF")
        elif op == "auto_follow":
            if bool(cmd.get("on")):
                auto["armed"] = True
                gate.reset()
                log(f"自動追従を待機 — リーダーをフォロワーの姿勢に合わせると開始"
                    f"（各関節 {tol_deg:g}° 以内）")
                publish_auto()
            else:
                disarm_auto("手動解除")
        elif op == "jog":
            n = cmd.get("joint")
            if n in JOINTS and not engine.active:
                lo, hi = ranges[n]
                engine.jog(n, max(lo, min(hi, int(cmd.get("pos", 0)))))
        elif op == "torque":
            n = cmd.get("joint")
            arm = cmd.get("arm", "follower")
            on = bool(cmd.get("on"))
            names = JOINTS if n == "all" else ([n] if n in JOINTS else [])
            if arm == "follower":
                if n == "all" and not on:
                    engine.relax_follower()
                elif n == "all" and on:
                    engine.enable_follower_torque_staged()
                else:
                    for j in names:
                        engine.set_follower_torque(j, on)
            elif engine.leader is not None:
                engine.leader.sync_write("Torque_Enable", {j: 1 if on else 0 for j in names})
                for j in names:
                    leader_view[j]["torque"] = 1 if on else 0
            log(f"{arm} {n} torque {'ON' if on else 'OFF'}")
        elif op == "speed":
            try:
                engine.set_speed(str(cmd.get("preset")))
                log(f"速度: {servo_config.SPEED_PRESETS[engine.speed]['label']}")
            except ValueError as e:
                log(f"不正な速度: {e}")
        elif op == "retreat":
            engine.start_retreat()
            disarm_auto("冷却姿勢")
        elif op == "torque_profile":
            log("トルクプロファイルは廃止しました（負荷と温度による自動調整に置き換え）")

    def publish(res: dict, now: float):
        out = res["governor"]
        fol = {}
        for n in JOINTS:
            s = res["follower"].get(n)
            js = out.joints.get(n)
            if s is None or js is None:
                continue
            dq = peaks[n]
            dq.append((now, abs(s.current)))
            while dq and now - dq[0][0] > CURRENT_PEAK_WINDOW_S:
                dq.popleft()
            fol[n] = {
                "pos": s.pos, "goal": js.goal, "torque": 1 if engine.torque_on.get(n) else 0,
                "state": js.state, "reason": js.reason, "duty": round(js.duty),
                "current": round(abs(s.current), 2),
                "current_max": round(max(v for _, v in dq), 2) if dq else 0.0,
                "temp": s.temp, "volt": round(s.volt, 1), "faults": list(s.faults),
                "stale": (not s.ok) and s.age > 3, "gravity": round(js.gravity),
            }
        lpos = res.get("leader") or {}
        for n in JOINTS:
            if lpos.get(n) is not None:
                leader_view[n]["pos"] = lpos[n]
                leader_view[n]["stale"] = False
            elif n in lpos:
                leader_view[n]["stale"] = True
        # hold an operator message for a few seconds so a passing state is readable
        if out.message:
            if out.message != held_msg["text"]:
                held_msg.update(text=out.message, level=out.message_level,
                                until=now + MESSAGE_HOLD_S)
            else:
                held_msg["until"] = now + MESSAGE_HOLD_S
        elif now > held_msg["until"]:
            held_msg.update(text="", level="info")
        currents = [abs(s.current) for s in res["follower"].values() if s.ok]
        with LOCK:
            STATE["connected"] = True
            STATE["error"] = ""
            STATE["teleop"] = engine.active
            STATE["retreating"] = engine.retreating
            STATE["follower"] = fol
            STATE["leader"] = json.loads(json.dumps(leader_view))
            STATE["arm"] = {
                "supply_v": out.supply_v, "current_sum": round(sum(currents), 2),
                "thermal": out.thermal, "follow_allowed": out.follow_allowed,
                "stop_reason": out.stop_reason, "speed": engine.speed,
                "speed_scale": out.speed_scale, "message": held_msg["text"],
                "message_level": held_msg["level"], "breaker": engine.governor.breaker,
            }
            STATE["hz"] = engine.measured_hz
            STATE["engine"] = engine.metrics()

    # ---- connect (retrying: a latched voltage error clears on a power cycle) ----
    first_fail = True
    fails = 0
    while not stopping():
        try:
            connect_all()
            with LOCK:
                STATE["connected"] = True
                STATE["error"] = ""
            log("両アーム接続 OK" + ("（シミュレーション）" if sim else ""))
            if auto["armed"]:
                log(f"自動追従: 待機中 — リーダーをフォロワーの姿勢に合わせると"
                    f"（各関節 {tol_deg:g}° 以内）自動で追従を開始します")
            break
        except Exception as e:
            if stopping():
                return
            fails += 1
            if fails >= 2:
                roles = None          # COM numbers move on replug: re-identify
            msg = str(e)
            if "Input voltage error" in msg:
                msg += ("（電圧リミット外れ: 電源ではなくサーボ設定側の可能性大。"
                        "scripts/so101_check_voltage_limits.py で Max/Min_Voltage と"
                        "実電圧を突き合わせてください）")
            with LOCK:
                STATE["error"] = f"connect failed: {msg}"
            if first_fail:
                log(f"接続失敗: {msg}")
                log("5秒ごとに再接続を試みます")
                first_fail = False
            close_buses()
            time.sleep(5.0)

    while not stopping():
        cycle_start = time.perf_counter()
        try:
            for cmd in _drain(CMDQ):
                handle(cmd)

            res = engine.cycle()
            now = time.perf_counter()

            # armed auto-follow: engage once the operator has parked the leader
            # on the follower's pose, so follow starts from agreement
            if auto["armed"] and not engine.active and not engine.retreating:
                lead = {n: v for n, v in (res.get("leader") or {}).items() if v is not None}
                foll = {n: s.pos for n, s in res["follower"].items() if s.ok}
                if gate.update(lead, foll):
                    ok, why = engine.start_teleop()
                    if ok:
                        auto["armed"] = False
                        log(f"▶ 自動追従 ON — {gate.status}")
                        publish_auto("追従中")
                    else:
                        log(f"自動追従を開始できません: {why}")
                        disarm_auto(why)
                else:
                    publish_auto()

            if cycle % LEADER_TELEMETRY_EVERY == 0 and engine.leader is not None:
                try:
                    lt = engine.leader.read_telemetry()
                    for n, s in lt.items():
                        if s.ok:
                            leader_view[n].update(volt=round(s.volt, 1), temp=s.temp)
                except Exception:
                    pass
            cycle += 1

            # an unplugged follower: every joint silent for a while
            if all((not s.ok) for s in res["follower"].values()):
                silent_cycles += 1
            else:
                silent_cycles = 0
            if silent_cycles >= ARM_DOWN_CYCLES:
                raise ConnectionError("follower が全軸無応答（ケーブルを確認）")

            publish(res, now)
        except Exception as e:
            # bus dropout / unplugged cable / serial error: stop, report, reconnect
            is_bus = isinstance(e, (ConnectionError, OSError)) or "Serial" in type(e).__name__
            log(("バス断: " if is_bus else "エラー: ") + str(e))
            if engine is not None and (engine.active or engine.retreating):
                engine.stop_teleop()
            with LOCK:
                STATE["connected"] = False if is_bus else STATE["connected"]
                STATE["error"] = str(e)
                if engine is not None:
                    STATE["teleop"] = engine.active
                    STATE["retreating"] = engine.retreating
            disarm_auto("バス断" if is_bus else "エラー")
            if is_bus:
                silent_cycles = 0
                close_buses()
                time.sleep(1.0)
                try:
                    connect_all()
                    log("再接続 OK（追従モードは安全のため OFF）")
                except Exception as e2:
                    log(f"再接続失敗: {e2}")
                    roles = None
                    time.sleep(2.0)
            else:
                time.sleep(0.5)

        period = cfg.period if (engine and (engine.active or engine.retreating)) else 1.0 / 30.0
        time.sleep(max(0.0, cycle_start + period - time.perf_counter()))


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body: bytes, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, UI_HTML.read_bytes(), "text/html; charset=utf-8")
        elif path == "/state":
            with LOCK:
                body = json.dumps(STATE, ensure_ascii=False)
            self._send(200, body.encode("utf-8"))
        elif path == "/hand-status":
            # server-side proxy to the hand server (:8001) so the browser avoids
            # a cross-origin call. Independent of the SO-101 serial worker.
            self._send(200, json.dumps(hand_status_proxy(), ensure_ascii=False).encode("utf-8"))
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            body = {}
        if path == "/quest/launch-hand":
            self._quest_launch_hand()
            return
        ops = {"/teleop": "teleop", "/jog": "jog", "/torque": "torque",
               "/torque_profile": "torque_profile", "/abort": "abort",
               "/relax": "relax", "/auto_follow": "auto_follow", "/speed": "speed",
               "/retreat": "retreat"}
        op = ops.get(path)
        if op is None:
            self._send(404, b'{"error":"not found"}')
            return
        body["op"] = op
        CMDQ.put(body)
        self._send(200, b'{"ok":true}')

    def _quest_launch_hand(self):
        """Open the ✋ hand-teleop (native VR app, else WebXR page) on every
        connected Quest — one click from the cockpit. The adb/CDP work runs on a
        worker thread joined with a hard cap so the handler never blocks forever,
        and it never touches CMDQ / the Feetech buses."""
        result = {"hand_server_up": probe_once(HAND_SERVER_PROBE_URL)}

        if hand_launch is None:
            result.update(ok=False,
                          error="hand_launch モジュールを読み込めませんでした（scripts/quest）。",
                          devices=[])
            result["message"] = result["error"]
            self._send(200, json.dumps(result, ensure_ascii=False).encode())
            return

        if not result["hand_server_up"]:
            result.update(ok=False,
                          error="hand server が起動していません（:8001）。"
                                "teleop_all.ps1 か server.py --real-hand を先に起動してください。",
                          devices=[])
            result["message"] = result["error"]
            self._send(200, json.dumps(result, ensure_ascii=False).encode())
            return

        box = {}

        def work():
            try:
                deadline = time.monotonic() + QUEST_LAUNCH_DEADLINE_S
                box["r"] = hand_launch.launch_hand_all(srv_port=8001, cdp_base=9223,
                                                       deadline=deadline)
            except Exception as e:  # pragma: no cover
                box["r"] = {"ok": False, "error": f"内部エラー: {e}", "devices": []}

        th = threading.Thread(target=work, daemon=True)
        th.start()
        th.join(QUEST_LAUNCH_JOIN_S)
        if th.is_alive():
            result.update(ok=False,
                          error=f"adb/CDP がタイムアウトしました（{int(QUEST_LAUNCH_JOIN_S)}s）。"
                                "USB 接続を確認してください。",
                          devices=[])
        else:
            result.update(box.get("r", {"ok": False, "error": "結果が取得できませんでした。",
                                        "devices": []}))
        result["message"] = _quest_launch_message(result)
        self._send(200, json.dumps(result, ensure_ascii=False).encode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader-port", default="auto",
                    help="COM port of the leader arm, or 'auto' to identify it")
    ap.add_argument("--follower-port", default="auto",
                    help="COM port of the follower arm, or 'auto' to identify it")
    ap.add_argument("--http-port", type=int, default=None,
                    help="default 8013 (8023 with --sim)")
    ap.add_argument("--fps", type=float, default=60.0, help="control rate while moving (Hz)")
    ap.add_argument("--speed", choices=list(servo_config.SPEED_PRESETS), default="normal")
    ap.add_argument("--governor", choices=["active", "shadow"], default="active",
                    help="shadow: show what the governor would do, command the plain reference")
    ap.add_argument("--auto-follow", action="store_true",
                    help="arm follow mode at boot: engages by itself once the "
                         "leader is brought onto the follower's pose")
    ap.add_argument("--auto-follow-tol-deg", type=float, default=AUTO_FOLLOW_TOL_DEG,
                    help="per-joint agreement required before auto-engaging")
    ap.add_argument("--sim", action="store_true",
                    help="simulated follower (MuJoCo + STS3215 model) and a scripted leader")
    args = ap.parse_args()

    http_port = args.http_port or (8023 if args.sim else 8013)
    cfg = TeleopConfig(target_hz=args.fps, speed=args.speed, governor_mode=args.governor)
    threading.Thread(target=worker,
                     args=(args.leader_port, args.follower_port, cfg,
                           args.auto_follow, args.auto_follow_tol_deg),
                     kwargs={"sim": args.sim}, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", http_port), Handler)
    print(f"[SO-101 COCKPIT{' SIM' if args.sim else ''}] leader={args.leader_port} "
          f"follower={args.follower_port} target={args.fps}Hz speed={args.speed} "
          f"governor={args.governor} auto_follow={'ON' if args.auto_follow else 'OFF'}"
          f" -> http://127.0.0.1:{http_port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
