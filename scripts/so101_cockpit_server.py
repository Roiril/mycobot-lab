"""SO-101 leader/follower cockpit server (run with .venv-so101 / Python 3.12).

Browser GUI for the leader->follower rig: live state of every motor on both
arms (position, voltage, temperature, current, torque), a teleop-mode toggle
(follower mirrors leader through the shared teleop engine — One-Euro smoothing,
last-goal stepping, brownout-safe staged torque), per-joint jog sliders and
torque switches for the follower, and ABORT.

Separate page (not a ui.html workspace tab) for the same reason as the calib
GUI: it must own BOTH COM ports exclusively and runs on the lerobot venv,
which conflicts with scripts/server.py's real driver on the follower port.

Single worker thread owns both Feetech buses (serial is not thread-safe);
HTTP handlers only enqueue commands and read a shared snapshot. Transient bus
dropouts (12V supply sag) are logged and reconnected, not fatal. The motion +
safety pipeline lives in src/robots/so101/teleop_engine.py, shared with the CLI
(scripts/so101_teleop.py) so there is one definition of the caps, smoothing and
stepping logic.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_cockpit_server.py
Open: http://localhost:8013/
"""
from __future__ import annotations
import sys
import json
import time
import queue
import argparse
import pathlib
import threading
import urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.teleop_engine import (
    TeleopEngine, TeleopConfig, JOINTS, make_bus, load_ranges,
    classify_voltage, classify_temperature,
    VOLT_WARN_V, VOLT_DEMOTE_V, TEMP_WARN_C, TEMP_DEMOTE_C,
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

# How often (in worker cycles) to read one slow telemetry field (V/temp/current).
# Kept off the teleop hot path so smoothing latency is unaffected.
TELEMETRY_EVERY = 6

LOCK = threading.Lock()
CMDQ: "queue.Queue" = queue.Queue()
STATE = {
    "connected": False, "teleop": False, "error": "", "log": [],
    "leader": {}, "follower": {}, "hz": 0.0, "engine": {},
    "torque_profile": "full", "demote_reason": "",
}


def log(msg: str):
    with LOCK:
        STATE["log"].append({"t": time.strftime("%H:%M:%S"), "msg": msg})
        del STATE["log"][:-40]


def worker(leader_port: str, follower_port: str, cfg: TeleopConfig):
    ranges = load_ranges(log)
    telemetry = {a: {n: {"pos": 0, "volt": 0.0, "temp": 0, "cur": 0, "torque": 0}
                     for n in JOINTS} for a in ("leader", "follower")}
    engine = None
    leader = follower = None
    slow_idx = 0    # round-robin (arm, joint) for slow telemetry reads
    cycle = 0
    # trip-wire edge state: last classified level so WARN/DEMOTE logs fire once
    # on transition instead of every round-robin pass through the same joint.
    trip = {"volt": "ok", "temp": {n: "ok" for n in JOINTS}}

    def connect_all():
        nonlocal leader, follower, engine
        leader = make_bus(leader_port)
        leader.connect(handshake=False)
        follower = make_bus(follower_port)
        follower.connect(handshake=False)
        if engine is None:
            engine = TeleopEngine(leader, follower, ranges, config=cfg, log=log)
        else:
            engine.rebind(leader, follower)
        engine.apply_follower_caps()

    def set_follower_torque(joint: str, on: bool):
        engine.set_follower_torque(joint, on)
        telemetry["follower"][joint]["torque"] = 1 if on else 0

    # 初回接続はリトライループ（電圧エラーラッチ等は電源挿し直しで直るため、
    # プロセス再起動なしで復帰できるようにする）
    first_fail = True
    while True:
        try:
            connect_all()
            with LOCK:
                STATE["connected"] = True
                STATE["error"] = ""
            log("両アーム接続 OK")
            break
        except Exception as e:
            msg = str(e)
            if "Input voltage error" in msg:
                # サーボ EEPROM の Max_Voltage(addr14)/Min_Voltage(addr15) を実電圧が
                # 外れると出る。電源再投入では消えない（設定側の問題）。2026-07-17 に
                # id3/4/5 が Max=12.0V のまま実測 12.2V で恒久エラーになった実績あり。
                msg += ("（電圧リミット外れ: 電源ではなくサーボ設定側の可能性大。"
                        "scripts/so101_check_voltage_limits.py で Max/Min_Voltage と"
                        "実電圧を突き合わせてください）")
            with LOCK:
                STATE["error"] = f"connect failed: {msg}"
            if first_fail:
                log(f"接続失敗: {msg}")
                log("5秒ごとに再接続を試みます")
                first_fail = False
            for b in (leader, follower):
                try:
                    if b is not None:
                        b.disconnect(disable_torque=False)
                except Exception:
                    pass
            time.sleep(5.0)

    while True:
        cycle_start = time.perf_counter()
        try:
            # ---- commands -------------------------------------------------
            try:
                cmd = CMDQ.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd:
                op = cmd.get("op")
                if op == "teleop":
                    if bool(cmd["on"]):
                        engine.start_teleop()      # staged torque enable inside
                        for n in JOINTS:
                            telemetry["follower"][n]["torque"] = 1
                        log("追従モード ON")
                    else:
                        engine.stop_teleop()       # freeze (torque stays on)
                        log("追従モード OFF（姿勢保持）")
                elif op == "jog" and not engine.active:
                    n = cmd["joint"]
                    lo, hi = ranges[n]
                    tgt = max(lo, min(hi, int(cmd["pos"])))
                    set_follower_torque(n, True)
                    follower.write("Goal_Position", n, tgt, normalize=False)
                    engine.last_goal[n] = tgt
                elif op == "torque":
                    n = cmd["joint"]
                    arm = cmd.get("arm", "follower")
                    names = JOINTS if n == "all" else [n]
                    for j in names:
                        if arm == "follower":
                            set_follower_torque(j, bool(cmd["on"]))
                        else:
                            leader.write("Torque_Enable", j, 1 if cmd["on"] else 0,
                                         normalize=False)
                            telemetry["leader"][j]["torque"] = 1 if cmd["on"] else 0
                    log(f"{arm} {n} torque {'ON' if cmd['on'] else 'OFF'}")
                elif op == "torque_profile":
                    prof = str(cmd.get("profile", "full"))
                    try:
                        # applies live (even mid-teleop). "full" is a manual
                        # recovery: clears any watchdog demotion + per-joint holds.
                        engine.set_torque_profile(prof, reason="手動切替")
                        if prof == "full":
                            trip["volt"] = "ok"
                            trip["temp"] = {n: "ok" for n in JOINTS}
                        log(f"トルクプロファイル → {prof}"
                            f"{'（手動復帰）' if prof == 'full' else '（手動降格）'}")
                    except ValueError as e:
                        log(f"不正なトルクプロファイル: {e}")
                elif op == "abort":
                    engine.stop_teleop()
                    log("⛔ ABORT — 追従停止・姿勢凍結")
                elif op == "relax":
                    # 冷却モード: 追従OFF + フォロワー全関節トルクOFF（先端→根元）。
                    # freeze はしない（トルクを切るので姿勢保持は無意味）。
                    log("🧊 脱力（冷却モード）— アームを支えてください")
                    engine.relax_follower()
                    for n in JOINTS:
                        telemetry["follower"][n]["torque"] = 0
                    log("脱力完了。復帰は追従ON か各関節トルクスイッチで")

            # ---- fast telemetry: leader positions (teleop input + display) -
            lead = engine.read_leader()
            for n in JOINTS:
                telemetry["leader"][n]["pos"] = lead[n]

            # ---- teleop step (follower present NOT read every cycle) -------
            if engine.active:
                res = engine.step(leader_ticks=lead)
                for n in JOINTS:                       # display commanded goal
                    telemetry["follower"][n]["pos"] = engine.last_goal[n]
                if "follower" in res:                  # periodic present sample
                    for n in JOINTS:
                        telemetry["follower"][n]["pos"] = res["follower"][n]
            else:
                # idle: keep follower display live with an actual present read
                fpos = engine.read_follower_present()
                for n in JOINTS:
                    telemetry["follower"][n]["pos"] = fpos[n]

            # ---- slow telemetry: V / temp / current, one motor per N cycles-
            if cycle % TELEMETRY_EVERY == 0:
                arm = ("leader", "follower")[slow_idx % 2]
                n = JOINTS[(slow_idx // 2) % len(JOINTS)]
                bus = leader if arm == "leader" else follower
                t = telemetry[arm][n]
                t["volt"] = bus.read("Present_Voltage", n, normalize=False) / 10
                t["temp"] = bus.read("Present_Temperature", n, normalize=False)
                t["cur"] = bus.read("Present_Current", n, normalize=False)
                if arm == "leader":
                    t["torque"] = bus.read("Torque_Enable", n, normalize=False)
                elif arm == "follower":
                    # trip-wire: watch the 12V follower for supply sag / overheat.
                    # Log on level transition (edge) so a persistent low reading
                    # doesn't spam. Demotion is one-way here — recovery is manual.
                    v = t["volt"]
                    if v and v > 1.0:                      # ignore stale/zero reads
                        vlvl = classify_voltage(v)
                        if vlvl != trip["volt"]:
                            if vlvl == "warn":
                                log(f"⚠ フォロワー電圧低下 {v:.1f}V (< {VOLT_WARN_V}V)")
                            elif vlvl == "demote":
                                log(f"⚠ 電源sag {v:.1f}V (< {VOLT_DEMOTE_V}V)"
                                    f" → safe プロファイルへ自動降格")
                                if engine.torque_profile != "safe":
                                    engine.set_torque_profile(
                                        "safe", reason=f"電源sag {v:.1f}V")
                            trip["volt"] = vlvl
                    tp = t["temp"]
                    tlvl = classify_temperature(tp)
                    if tlvl != trip["temp"][n]:
                        if tlvl == "warn":
                            log(f"⚠ {n} 温度上昇 {tp}℃ (> {TEMP_WARN_C}℃)")
                        elif tlvl == "demote":
                            log(f"⚠ {n} 高温 {tp}℃ (> {TEMP_DEMOTE_C}℃)"
                                f" → この関節を safe 値へ降格")
                            engine.demote_joint(n, reason=f"{n} 高温 {tp}℃")
                        trip["temp"][n] = tlvl
                slow_idx += 1
            cycle += 1

            with LOCK:
                STATE["teleop"] = engine.active
                STATE["connected"] = True
                STATE["error"] = ""
                STATE["leader"] = json.loads(json.dumps(telemetry["leader"]))
                STATE["follower"] = json.loads(json.dumps(telemetry["follower"]))
                STATE["ranges"] = ranges
                STATE["hz"] = engine.measured_hz
                STATE["engine"] = engine.metrics()
                STATE["torque_profile"] = engine.torque_profile
                STATE["demote_reason"] = engine.demote_reason
        except ConnectionError as e:
            # supply sag / bus dropout: log, reconnect, keep serving
            log(f"バス断: {e}")
            with LOCK:
                STATE["connected"] = False
                STATE["error"] = str(e)
            if engine is not None:
                engine.active = False
                # a bus dropout is itself a brownout symptom: demote to safe so
                # the reconnected follower comes back with reduced draw. state
                # only (apply=False) — connect_all re-applies caps on reconnect.
                if engine.torque_profile != "safe":
                    engine.set_torque_profile(
                        "safe", reason="バス断（brownout の徴候）", apply=False)
                    trip["volt"] = "demote"
                    log("バス断のため safe プロファイルへ降格（復帰は手動）")
            for b in (leader, follower):
                try:
                    if b is not None:
                        b.disconnect(disable_torque=False)
                except Exception:
                    pass
            time.sleep(1.0)
            try:
                connect_all()
                log("再接続 OK（追従モードは安全のため OFF）")
            except Exception as e2:
                log(f"再接続失敗: {e2}")
                time.sleep(2.0)
        except Exception as e:
            with LOCK:
                STATE["error"] = str(e)
            log(f"エラー: {e}")
            time.sleep(0.5)

        # 締切基準でペースを取る（固定 sleep だと実 I/O 時間の分だけ周期が漂う）。
        # teleop 中は target_hz、idle 時は 30ms 相当に落としてバスを空ける。
        period = cfg.period if (engine and engine.active) else 0.03
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
                self._send(200, json.dumps(STATE).encode("utf-8"))
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
               "/relax": "relax"}
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
    ap.add_argument("--leader-port", default="COM14")
    ap.add_argument("--follower-port", default="COM13")
    ap.add_argument("--http-port", type=int, default=8013)
    ap.add_argument("--fps", type=float, default=60.0, help="teleop target rate (Hz)")
    args = ap.parse_args()

    cfg = TeleopConfig(target_hz=args.fps)
    threading.Thread(target=worker,
                     args=(args.leader_port, args.follower_port, cfg),
                     daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"[SO-101 COCKPIT] leader={args.leader_port} follower={args.follower_port} "
          f"target={args.fps}Hz -> http://127.0.0.1:{args.http_port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
