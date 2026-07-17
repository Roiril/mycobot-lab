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
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101.teleop_engine import (
    TeleopEngine, TeleopConfig, JOINTS, make_bus, load_ranges,
)

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "so101_cockpit.html"

# How often (in worker cycles) to read one slow telemetry field (V/temp/current).
# Kept off the teleop hot path so smoothing latency is unaffected.
TELEMETRY_EVERY = 6

LOCK = threading.Lock()
CMDQ: "queue.Queue" = queue.Queue()
STATE = {
    "connected": False, "teleop": False, "error": "", "log": [],
    "leader": {}, "follower": {}, "hz": 0.0, "engine": {},
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
                elif op == "abort":
                    engine.stop_teleop()
                    log("⛔ ABORT — 追従停止・姿勢凍結")

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
        except ConnectionError as e:
            # supply sag / bus dropout: log, reconnect, keep serving
            log(f"バス断: {e}")
            with LOCK:
                STATE["connected"] = False
                STATE["error"] = str(e)
            if engine is not None:
                engine.active = False
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
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            body = {}
        ops = {"/teleop": "teleop", "/jog": "jog", "/torque": "torque", "/abort": "abort"}
        op = ops.get(path)
        if op is None:
            self._send(404, b'{"error":"not found"}')
            return
        body["op"] = op
        CMDQ.put(body)
        self._send(200, b'{"ok":true}')


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
