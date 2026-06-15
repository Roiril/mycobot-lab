"""SO-101 calibration GUI server (run with .venv-so101 / Python 3.12 — needs lerobot).

A browser tool for calibrating the SO-101 with LIVE progress, so you can watch
each joint's position and recorded range fill in (and instantly see which joints
you still need to sweep). Replaces the chat-driven CLI calibration.

Flow in the UI:
  1. Pose the arm to a straight neutral, hold, click "中立をセット" -> sign-safe
     torque-hold homing centers every joint to ~2048.
  2. Click "記録開始", sweep every joint end-to-end (bars turn green as their span
     passes the threshold), click "記録停止".
  3. Click "保存" -> writes the lerobot calibration file so the real driver in
     scripts/server.py (--so101-driver real) picks it up.

Single dedicated worker thread owns the Feetech bus (serial is not thread-safe);
HTTP handlers only enqueue commands and read a shared snapshot.

Run:  .venv-so101\\Scripts\\python.exe scripts\\so101_calib_server.py --port COM13
Open: http://localhost:8012/
"""
from __future__ import annotations
import sys, json, time, queue, argparse, threading, pathlib
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "so101_calib.html"

FULL_TURN = "wrist_roll"          # auto 0..4095, no manual sweep
SWEEP_OK_TICKS = 500              # span over this = "swept enough" (wrist_roll exempt)

LOCK = threading.Lock()
CMDQ: "queue.Queue" = queue.Queue()
STATE = {
    "connected": False, "error": "", "homed": False, "recording": False,
    "saved": False, "saved_path": "", "joints": [],
}


def worker(port: str, robot_id: str):
    robot = SO101Follower(SO101FollowerConfig(port=port, id=robot_id))
    bus = robot.bus
    names = list(bus.motors)
    ids = {n: bus.motors[n].id for n in names}
    offsets = {n: 0 for n in names}
    mins = {n: None for n in names}
    maxes = {n: None for n in names}

    try:
        bus.connect()
        bus.disable_torque()
        # NOTE: do NOT reset_calibration() here — it wipes Homing_Offset in the
        # motors' EEPROM. Recorded ranges must live in the offset-applied domain
        # (centered ~2048, no 0/4095 boundary crossing), so existing offsets
        # must stay. Offsets are (re)written only by the homing step.
        for n in names:
            bus.write("Operating_Mode", n, OperatingMode.POSITION.value)
        with LOCK:
            STATE["connected"] = True
    except Exception as e:
        with LOCK:
            STATE["error"] = f"connect failed: {e}"
        return

    def publish(pos):
        rec = STATE["recording"]
        joints = []
        for n in names:
            mn, mx = mins[n], maxes[n]
            span = (mx - mn) if (mn is not None and mx is not None) else 0
            joints.append({
                "name": n, "id": ids[n], "pos": int(pos.get(n, 0)),
                "offset": int(offsets[n]),
                "min": None if mn is None else int(mn),
                "max": None if mx is None else int(mx),
                "span": int(span),
                "full_turn": n == FULL_TURN,
                "swept": n == FULL_TURN or span >= SWEEP_OK_TICKS,
            })
        with LOCK:
            STATE["joints"] = joints

    def do_home():
        bus.disable_torque()
        bus.reset_calibration()
        for n in names:
            bus.write("Operating_Mode", n, OperatingMode.POSITION.value)
        p0 = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        for n in names:
            bus.write("Goal_Position", n, p0[n], normalize=False)
        bus.enable_torque()
        time.sleep(0.5)
        pres = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        bus.disable_torque()
        for n in names:
            u = pres[n] % 4096
            # Feetech STS3215: Present = raw - Homing_Offset (verified on this
            # hardware by readback after write). So to make the held pose read
            # 2048: off = u - 2048. (2048-u, the intuitive sign, mirrors the pose.)
            off = ((u - 2048 + 2048) % 4096) - 2048
            offsets[n] = max(-2047, min(2047, off))
            bus.write("Homing_Offset", n, offsets[n])
        for n in names:
            mins[n] = None
            maxes[n] = None
        with LOCK:
            STATE["homed"] = True
            STATE["saved"] = False

    def do_save():
        cal = {}
        for n in names:
            if n == FULL_TURN:
                rmin, rmax = 0, 4095
            else:
                rmin = mins[n] if mins[n] is not None else 1024
                rmax = maxes[n] if maxes[n] is not None else 3072
            cal[n] = MotorCalibration(id=ids[n], drive_mode=0,
                                      homing_offset=int(offsets[n]),
                                      range_min=int(rmin), range_max=int(rmax))
        robot.calibration = cal
        bus.write_calibration(cal)
        robot._save_calibration()
        with LOCK:
            STATE["saved"] = True
            STATE["saved_path"] = str(robot.calibration_fpath)

    while True:
        try:
            try:
                cmd = CMDQ.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd == "home":
                do_home()
            elif cmd == "start_rec":
                cur = bus.sync_read("Present_Position", normalize=False, num_retry=3)
                for n in names:
                    if n == FULL_TURN:
                        continue
                    mins[n] = cur[n]; maxes[n] = cur[n]
                with LOCK:
                    STATE["recording"] = True
            elif cmd == "stop_rec":
                with LOCK:
                    STATE["recording"] = False
            elif cmd == "save":
                do_save()

            pos = bus.sync_read("Present_Position", normalize=False, num_retry=2)
            if STATE["recording"]:
                for n in names:
                    if n == FULL_TURN:
                        continue
                    if mins[n] is None or pos[n] < mins[n]:
                        mins[n] = pos[n]
                    if maxes[n] is None or pos[n] > maxes[n]:
                        maxes[n] = pos[n]
            publish(pos)
            with LOCK:
                STATE["error"] = ""
        except Exception as e:
            with LOCK:
                STATE["error"] = str(e)
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
        elif path == "/cal/state":
            with LOCK:
                self._send(200, json.dumps(STATE).encode("utf-8"))
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):
        path = urlparse(self.path).path
        cmd = {"/cal/home": "home", "/cal/start": "start_rec",
               "/cal/stop": "stop_rec", "/cal/save": "save"}.get(path)
        if cmd is None:
            self._send(404, b'{"error":"not found"}')
            return
        CMDQ.put(cmd)
        self._send(200, b'{"ok":true}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="board COM port (e.g. COM13)")
    # MUST match the id the integrated real driver uses (LerobotSo101Driver
    # defaults robot_id="so101_follower"), or the saved calibration won't load.
    ap.add_argument("--id", default="so101_follower")
    ap.add_argument("--http-port", type=int, default=8012)
    args = ap.parse_args()

    threading.Thread(target=worker, args=(args.port, args.id), daemon=True).start()
    srv = HTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"[SO-101 CALIB] bus={args.port} -> http://127.0.0.1:{args.http_port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
