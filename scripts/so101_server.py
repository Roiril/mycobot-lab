"""Standalone OFFLINE control server for the SO-101 (5-DoF) arm.

This is intentionally SEPARATE from scripts/server.py (the 6-DoF myCobot
server, which is module-globally bound to src/arm/*). It serves a minimal
browser UI to jog the SO-101, run position IK, and watch the real arm geometry
rendered by MuJoCo — entirely without hardware. It is the concrete form of the
plan's "VirtualSO101Hub -> offline UI" deliverable.

Drivers (--driver):
  sim   MuJoCo virtual arm with real meshes + live PNG rendering (default)
  mock  in-memory, no rendering (fast smoke test)
  real  lerobot SO101Follower over a COM port (lazy import; needs calibration)

The motion brain is robots.so101.controller.So101Controller (safety-validated
joint/Cartesian moves) — the same code path the future unified server will use.

Run:
  python scripts/so101_server.py                 # sim, http://localhost:8011/
  python scripts/so101_server.py --driver mock
  python scripts/so101_server.py --driver real --robot-port COM7
"""
from __future__ import annotations
import sys, io, json, argparse, pathlib, threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile  # noqa: E402
from robots.so101.controller import So101Controller  # noqa: E402
from robots.so101.kinematics import end_effector  # noqa: E402

UI_HTML = HERE / "so101.html"

# Process-wide state. The server is single-threaded (one user, dev tool), so a
# single lock around the driver + MuJoCo GL context is sufficient and avoids
# per-thread GL affinity problems.
CTRL: So101Controller | None = None
DRIVER = None
RENDERS = False           # True when the driver can render() PNGs (sim)
LOCK = threading.Lock()


def make_driver(kind: str, robot_port: str | None):
    """Return (driver, renders)."""
    if kind == "sim":
        from robots.so101.sim.mujoco_sim import MujocoSo101Driver
        return MujocoSo101Driver(), True
    if kind == "mock":
        from robots.so101.driver import MockSo101Driver
        return MockSo101Driver(), False
    if kind == "real":
        if not robot_port:
            sys.exit("--driver real requires --robot-port COMx")
        from robots.so101.driver import LerobotSo101Driver
        return LerobotSo101Driver(port=robot_port), False
    sys.exit(f"unknown --driver {kind!r}")


def state_dict() -> dict:
    angles = CTRL.current_angles()
    tip = end_effector(angles)
    grip = DRIVER.read_gripper()
    return {
        "joint_names": profile.JOINT_NAMES,
        "gripper_name": profile.GRIPPER_NAME,
        "num_joints": profile.NUM_JOINTS,
        "limits": profile.JOINT_LIMITS,
        "gripper_range": [0, 100],
        "home": profile.HOME_ANGLES,
        "angles": [round(a, 2) for a in angles],
        "gripper": None if grip is None else round(grip, 1),
        "tip_mm": [round(c, 1) for c in tip],
        "renders": RENDERS,
    }


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # quiet
        pass

    def _send(self, code, body: bytes, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj).encode("utf-8"))

    def _read_json(self) -> dict:
        n = int(self.headers.get("Content-Length", 0) or 0)
        if not n:
            return {}
        return json.loads(self.rfile.read(n).decode("utf-8") or "{}")

    # --- GET ---
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, UI_HTML.read_bytes(), "text/html; charset=utf-8")
        elif path == "/state":
            with LOCK:
                self._json(state_dict())
        elif path == "/frame.png":
            self._frame()
        else:
            self._json({"error": "not found"}, 404)

    def _frame(self):
        if not RENDERS:
            self._json({"error": "driver has no rendering"}, 404)
            return
        try:
            with LOCK:
                arr = DRIVER.render(width=560, height=420)
            from PIL import Image
            buf = io.BytesIO()
            Image.fromarray(arr).save(buf, format="PNG")
            self._send(200, buf.getvalue(), "image/png")
        except Exception as e:
            self._json({"error": f"render failed: {e}"}, 500)

    # --- POST ---
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            body = self._read_json()
        except Exception as e:
            self._json({"ok": False, "msg": f"bad json: {e}"}, 400)
            return

        with LOCK:
            if path == "/jog":
                angles = body.get("angles")
                grip = body.get("gripper")
                if not isinstance(angles, list) or len(angles) != profile.NUM_JOINTS:
                    self._json({"ok": False, "msg": f"angles must be length {profile.NUM_JOINTS}"}, 400)
                    return
                ok, msg = CTRL.move_to_angles(angles, gripper=grip)
                self._result(ok, msg)
            elif path == "/ik":
                xyz = body.get("xyz")
                grip = body.get("gripper")
                if not isinstance(xyz, list) or len(xyz) != 3:
                    self._json({"ok": False, "msg": "xyz must be length 3"}, 400)
                    return
                ok, msg = CTRL.move_to_position(xyz, gripper=grip)
                self._result(ok, msg)
            elif path == "/home":
                ok, msg = CTRL.home()
                self._result(ok, msg)
            elif path == "/release":
                DRIVER.release()
                self._result(True, "released")
            else:
                self._json({"error": "not found"}, 404)

    def _result(self, ok, msg):
        out = state_dict()
        out["ok"] = ok
        out["msg"] = msg
        self._json(out)


def main():
    global CTRL, DRIVER, RENDERS
    ap = argparse.ArgumentParser(description="Offline SO-101 control server")
    ap.add_argument("--driver", choices=["sim", "mock", "real"], default="sim")
    ap.add_argument("--robot-port", default=None, help="COM port for --driver real")
    ap.add_argument("--port", type=int, default=8011)
    ap.add_argument("--bind", default="127.0.0.1")
    args = ap.parse_args()

    DRIVER, RENDERS = make_driver(args.driver, args.robot_port)
    DRIVER.connect()
    if hasattr(DRIVER, "set_torque"):
        DRIVER.set_torque(True)
    CTRL = So101Controller(DRIVER)

    # Park at HOME so the first render/state is well-defined.
    try:
        CTRL.move_to_angles(list(profile.HOME_ANGLES), gripper=50.0)
    except Exception as e:
        print(f"[warn] initial home failed: {e}")

    srv = HTTPServer((args.bind, args.port), Handler)
    print(f"[SO-101] driver={args.driver} renders={RENDERS}  ->  http://{args.bind}:{args.port}/")
    print("  Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        try:
            DRIVER.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
