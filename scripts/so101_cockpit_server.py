"""SO-101 leader/follower cockpit server (run with .venv-so101 / Python 3.12).

Browser GUI for the leader->follower rig: live state of every motor on both
arms (position, voltage, temperature, current, torque), a teleop-mode toggle
(follower mirrors leader with the brownout-safe caps from so101_teleop.py),
per-joint jog sliders and torque switches for the follower, and ABORT.

Separate page (not a ui.html workspace tab) for the same reason as the calib
GUI: it must own BOTH COM ports exclusively and runs on the lerobot venv,
which conflicts with scripts/server.py's real driver on the follower port.

Single worker thread owns both Feetech buses (serial is not thread-safe);
HTTP handlers only enqueue commands and read a shared snapshot. Transient bus
dropouts (12V 2A supply sag) are logged and reconnected, not fatal.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_cockpit_server.py
Open: http://localhost:8013/
"""
from __future__ import annotations
import json
import time
import queue
import argparse
import pathlib
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "so101_cockpit.html"
CAL_DIR = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]

# Brownout mitigations for the 12V 2A supply (see so101_teleop.py):
# gravity-loaded joints need headroom or they stall/overheat; the rest stay
# low to cap total draw.
TORQUE_LIMITS = {
    "shoulder_pan": 400, "shoulder_lift": 700, "elbow_flex": 700,
    "wrist_flex": 400, "wrist_roll": 400, "gripper": 400,
}
ACCELERATION = 30
GOAL_VELOCITY = 800
MAX_STEP_TICKS = 170          # per-cycle clamp (~15 deg) while teleoperating

LOCK = threading.Lock()
CMDQ: "queue.Queue" = queue.Queue()
STATE = {
    "connected": False, "teleop": False, "error": "", "log": [],
    "leader": {}, "follower": {},
}


def log(msg: str):
    with LOCK:
        STATE["log"].append({"t": time.strftime("%H:%M:%S"), "msg": msg})
        del STATE["log"][:-40]


def load_ranges() -> dict:
    """Follower calibration ranges (offset-applied domain, shared by both
    arms now that the leader homing is aligned to the follower)."""
    fp = CAL_DIR / "robots/so_follower/so101_follower.json"
    try:
        cal = json.loads(fp.read_text())
        return {n: (c["range_min"], c["range_max"]) for n, c in cal.items()}
    except Exception as e:
        log(f"calibration load failed ({e}) - using full range")
        return {n: (0, 4095) for n in JOINTS}


def make_bus(port: str) -> FeetechMotorsBus:
    motors = {n: Motor(i + 1, "sts3215", MotorNormMode.DEGREES)
              for i, n in enumerate(JOINTS)}
    return FeetechMotorsBus(port, motors)


def worker(leader_port: str, follower_port: str):
    ranges = load_ranges()
    arms = {}       # name -> bus
    telemetry = {a: {n: {"pos": 0, "volt": 0.0, "temp": 0, "cur": 0, "torque": 0}
                     for n in JOINTS} for a in ("leader", "follower")}
    teleop = False
    slow_idx = 0    # round-robin (arm, joint, field) for slow telemetry reads

    def connect_all():
        for name, port in (("leader", leader_port), ("follower", follower_port)):
            b = make_bus(port)
            b.connect(handshake=False)
            arms[name] = b
        for n in JOINTS:
            arms["follower"].write("Torque_Limit", n, TORQUE_LIMITS[n], normalize=False)
            arms["follower"].write("Acceleration", n, ACCELERATION, normalize=False)
            arms["follower"].write("Goal_Velocity", n, GOAL_VELOCITY, normalize=False)

    def set_follower_torque(joint: str, on: bool):
        arms["follower"].write("Torque_Enable", joint, 1 if on else 0)
        telemetry["follower"][joint]["torque"] = 1 if on else 0

    def freeze_follower():
        for n in JOINTS:
            if telemetry["follower"][n]["torque"]:
                p = arms["follower"].read("Present_Position", n, normalize=False)
                arms["follower"].write("Goal_Position", n, p, normalize=False)

    try:
        connect_all()
        with LOCK:
            STATE["connected"] = True
        log("両アーム接続 OK")
    except Exception as e:
        with LOCK:
            STATE["error"] = f"connect failed: {e}"
        log(f"接続失敗: {e}")
        return

    while True:
        try:
            # ---- commands -------------------------------------------------
            try:
                cmd = CMDQ.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd:
                op = cmd.get("op")
                if op == "teleop":
                    teleop = bool(cmd["on"])
                    if teleop:
                        # enable torque one joint at a time (inrush control)
                        for n in JOINTS:
                            set_follower_torque(n, True)
                            time.sleep(0.05)
                        log("追従モード ON")
                    else:
                        freeze_follower()
                        log("追従モード OFF（姿勢保持）")
                elif op == "jog" and not teleop:
                    n = cmd["joint"]
                    tgt = max(ranges[n][0], min(ranges[n][1], int(cmd["pos"])))
                    set_follower_torque(n, True)
                    arms["follower"].write("Goal_Position", n, tgt, normalize=False)
                elif op == "torque":
                    n = cmd["joint"]
                    arm = cmd.get("arm", "follower")
                    names = JOINTS if n == "all" else [n]
                    for j in names:
                        if arm == "follower":
                            set_follower_torque(j, bool(cmd["on"]))
                        else:
                            arms["leader"].write("Torque_Enable", j, 1 if cmd["on"] else 0)
                            telemetry["leader"][j]["torque"] = 1 if cmd["on"] else 0
                    log(f"{arm} {n} torque {'ON' if cmd['on'] else 'OFF'}")
                elif op == "abort":
                    teleop = False
                    freeze_follower()
                    log("⛔ ABORT — 追従停止・姿勢凍結")

            # ---- fast telemetry: positions -------------------------------
            for arm in ("leader", "follower"):
                pos = arms[arm].sync_read("Present_Position", normalize=False, num_retry=1)
                for n in JOINTS:
                    telemetry[arm][n]["pos"] = int(pos[n])

            # ---- teleop step ----------------------------------------------
            if teleop:
                for n in JOINTS:
                    lp = telemetry["leader"][n]["pos"]
                    fp = telemetry["follower"][n]["pos"]
                    tgt = max(ranges[n][0], min(ranges[n][1], lp))
                    step = max(-MAX_STEP_TICKS, min(MAX_STEP_TICKS, tgt - fp))
                    arms["follower"].write("Goal_Position", n, fp + step, normalize=False)

            # ---- slow telemetry: V / temp / current, one motor per cycle --
            arm = ("leader", "follower")[slow_idx % 2]
            n = JOINTS[(slow_idx // 2) % len(JOINTS)]
            t = telemetry[arm][n]
            t["volt"] = arms[arm].read("Present_Voltage", n, normalize=False) / 10
            t["temp"] = arms[arm].read("Present_Temperature", n, normalize=False)
            t["cur"] = arms[arm].read("Present_Current", n, normalize=False)
            if arm == "leader":
                t["torque"] = arms[arm].read("Torque_Enable", n, normalize=False)
            slow_idx += 1

            with LOCK:
                STATE["teleop"] = teleop
                STATE["connected"] = True
                STATE["error"] = ""
                STATE["leader"] = json.loads(json.dumps(telemetry["leader"]))
                STATE["follower"] = json.loads(json.dumps(telemetry["follower"]))
                STATE["ranges"] = ranges
        except ConnectionError as e:
            # supply sag / bus dropout: log, reconnect, keep serving
            log(f"バス断: {e}")
            with LOCK:
                STATE["connected"] = False
                STATE["error"] = str(e)
            teleop = False
            for b in arms.values():
                try:
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
        time.sleep(0.03)


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
    args = ap.parse_args()

    threading.Thread(target=worker, args=(args.leader_port, args.follower_port),
                     daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"[SO-101 COCKPIT] leader={args.leader_port} follower={args.follower_port} "
          f"-> http://127.0.0.1:{args.http_port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
