"""SO-101 virtual arm for the browser: STS3215 servo physics on a floor.

The physics is `FloorArm` (src/robots/so101/sim): the same STS3215 model the
cockpit's --sim mode uses (trapezoidal profile, P/D position loop, DC motor
behind the 1/345 gear, overload / over-current / overheat protections, heat),
integrated by MuJoCo at 1 kHz with no window. This server only runs it in real
time (or slowed down), takes goal / register writes, and streams the state.

Run:   .venv-so101\\Scripts\\python.exe scripts\\so101_sim_server.py
Open:  http://localhost:8025/

Nothing here talks to hardware.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import struct
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from robots.so101 import poses as so101_poses  # noqa: E402
from robots.so101 import sts3215  # noqa: E402
from robots.so101.sim.floor_world import FloorArm, FloorParams  # noqa: E402
from robots.so101.sim.servo_sim import JOINTS, TICKS_PER_RAD, World  # noqa: E402

UI_HTML = HERE / "so101_sim.html"
STATIC = HERE / "static"

START_DEG = [0.0, -35.0, 45.0, -15.0, 0.0]          # READY: clear of the floor
HIST_DT = 0.005                                      # trace sample period (sim s)
HIST_LEN = 4000                                      # 20 s of trace
THERMAL_LEN = 3601                                  # one hour, 1 Hz, simulation time
MOTOR_LIMITS = {"r_th": (0.5, 15.0), "c_th": (5.0, 100.0)}
EDITABLE_REGS = ("Goal_Velocity", "Acceleration", "P_Coefficient", "D_Coefficient",
                 "Torque_Limit")
REG_LIMITS = {"Goal_Velocity": (0, 3400), "Acceleration": (0, 254),
              "P_Coefficient": (1, 254), "D_Coefficient": (0, 254), "Torque_Limit": (0, 1000)}


def _rad_to_ticks(servo, rad: float) -> int:
    return int(round(servo.mid + rad * TICKS_PER_RAD))


def _ticks_to_deg(servo, ticks: float) -> float:
    return math.degrees((ticks - servo.mid) / TICKS_PER_RAD)


class Sim:
    def __init__(self):
        self.lock = threading.RLock()
        self.rtf = 1.0
        self.paused = False
        self.payload_kg = 0.0
        self.floor_on = True
        self.ambient_c = 27.0
        self.motor_params = {name: {"r_th": 3.0, "c_th": 29.0} for name in JOINTS}
        self.regs = {"Goal_Velocity": 0, "Acceleration": 254, "P_Coefficient": 16,
                     "D_Coefficient": 32, "Torque_Limit": 1000}
        self.hist = deque(maxlen=HIST_LEN)
        self.thermal_hist = deque(maxlen=THERMAL_LEN)
        self.thermal_seq = 0
        # Remain distinct across server restarts as well as cold resets.
        # Microseconds still fit exactly into a JavaScript integer.
        self.generation = time.time_ns() // 1000
        self._analysis_cache = {}
        self.seq = 0
        self._build(START_DEG, torque=True)

    # -- lifecycle ------------------------------------------------------------
    def _build(self, start_deg, torque: bool) -> None:
        self.arm = FloorArm(world=World(payload_kg=self.payload_kg), start_deg=list(start_deg),
                            floor=FloorParams(enabled=self.floor_on))
        m = self.arm.m
        self.ghost = mujoco.MjData(m)
        self.jnt_range = [tuple(math.degrees(v) for v in m.jnt_range[j]) for j in range(m.njnt)]
        for s in self.arm.servos:
            s.mp.ambient_c = self.ambient_c
            for key, value in self.motor_params[s.name].items():
                setattr(s.mp, key, value)
            s.temp_c = self.ambient_c
            s.peak_temp = self.ambient_c
            for reg, v in self.regs.items():
                s.write(reg, v, s.setpoint)
            s.write("Torque_Enable", 1 if torque else 0, s.setpoint)
        self.hist.clear()
        self.thermal_hist.clear()
        self._next_thermal = 0.0
        self.generation += 1
        self._analysis_cache.clear()
        self._next_sample = 0.0
        self._wall0 = time.perf_counter()
        self._sim0 = 0.0
        self._rtf_meas = deque(maxlen=60)

    def reset(self, pose_deg=None) -> None:
        with self.lock:
            q = list(pose_deg or START_DEG)
            q = (q + [0.0] * 5)[:5]
            self._build(q, torque=True)

    # -- commands ---------------------------------------------------------------
    def set_goals(self, goals: dict) -> None:
        with self.lock:
            for name, deg in goals.items():
                if name not in JOINTS:
                    continue
                j = JOINTS.index(name)
                lo, hi = self.jnt_range[j]
                deg = max(lo, min(hi, float(deg)))
                s = self.arm.servos[j]
                s.write("Goal_Position", _rad_to_ticks(s, math.radians(deg)), self.arm.ticks(j))

    def set_torque(self, on: bool) -> None:
        with self.lock:
            for j, s in enumerate(self.arm.servos):
                pos = self.arm.ticks(j)
                if on:
                    # hold where the shaft is now, like the real servo
                    s.regs.goal_position = int(round(pos))
                s.write("Torque_Enable", 1 if on else 0, pos)
            self._analysis_cache.clear()

    def set_regs(self, regs: dict, joint=None) -> None:
        with self.lock:
            if joint is not None and joint not in JOINTS:
                raise ValueError("unknown joint")
            for reg, v in regs.items():
                if reg not in EDITABLE_REGS:
                    continue
                lo, hi = REG_LIMITS[reg]
                v = max(lo, min(hi, int(v)))
                if joint is None:
                    self.regs[reg] = v
                for j, s in enumerate(self.arm.servos):
                    if joint is None or s.name == joint:
                        s.write(reg, v, self.arm.ticks(j))
            self._analysis_cache.clear()

    def set_motor(self, joint: str, params: dict) -> None:
        if joint not in JOINTS:
            raise ValueError("unknown joint")
        values = {}
        for key, value in params.items():
            if key not in MOTOR_LIMITS:
                raise ValueError("unknown motor parameter")
            value = float(value)
            lo, hi = MOTOR_LIMITS[key]
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{key} must be between {lo} and {hi}")
            values[key] = value
        with self.lock:
            self.motor_params[joint].update(values)
            s = self.arm.servos[JOINTS.index(joint)]
            for key, value in values.items():
                setattr(s.mp, key, value)
            self._analysis_cache.clear()

    def set_world(self, payload_kg=None, floor=None, ambient_c=None) -> None:
        if ambient_c is not None:
            ambient_c = float(ambient_c)
            if not math.isfinite(ambient_c) or not -10 <= ambient_c <= 50:
                raise ValueError("ambient_c must be between -10 and 50")
        if payload_kg is not None:
            payload_kg = float(payload_kg)
            if not math.isfinite(payload_kg):
                raise ValueError("payload must be finite")
        with self.lock:
            if payload_kg is not None:
                self.payload_kg = max(0.0, min(1.0, payload_kg))
                self.arm.set_payload(self.payload_kg)
            if floor is not None:
                self.floor_on = bool(floor)
                self.arm.floor.enabled = self.floor_on
            if ambient_c is not None:
                self.ambient_c = ambient_c
                for s in self.arm.servos:
                    s.mp.ambient_c = ambient_c
            self._analysis_cache.clear()

    def set_run(self, paused=None, rtf=None) -> None:
        with self.lock:
            if paused is not None:
                self.paused = bool(paused)
            if rtf is not None:
                self.rtf = max(0.02, min(2.0, float(rtf)))
            self._wall0 = time.perf_counter()
            self._sim0 = self.arm.t

    # -- physics thread -----------------------------------------------------------
    def run_forever(self) -> None:
        while True:
            with self.lock:
                if not self.paused:
                    target = self._sim0 + (time.perf_counter() - self._wall0) * self.rtf
                    lag = target - self.arm.t
                    if lag > 0.1:                     # host stalled: drop, don't race
                        self._sim0 += lag - 0.1
                        lag = 0.1
                    n = int(lag / 0.001)
                    for _ in range(n):
                        self.arm.step(0.001)
                        if self.arm.t >= self._next_sample:
                            self._sample()
                            self._next_sample = self.arm.t + HIST_DT
            time.sleep(0.002)

    def _sample(self) -> None:
        a = self.arm
        row = [round(a.t, 4)]
        for j, s in enumerate(a.servos):
            row += [round(_ticks_to_deg(s, s.regs.goal_position), 2),
                    round(_ticks_to_deg(s, s.setpoint), 2),
                    round(math.degrees(a.d.qpos[j]), 2),
                    round(s.duty / 10.0, 1),            # % of full drive
                    round(s.current, 3)]
        self.seq += 1
        self.hist.append((self.seq, row))
        if a.t >= self._next_thermal:
            self.thermal_seq += 1
            self.thermal_hist.append((self.thermal_seq, [round(a.t, 4)] +
                                      [round(s.temp_c, 3) for s in a.servos]))
            self._next_thermal = a.t + 1.0

    # -- views -------------------------------------------------------------------
    def _bodies(self, data) -> list:
        out = []
        for b in range(1, self.arm.m.nbody):
            p, q = data.xpos[b], data.xquat[b]
            out.append([round(float(v), 6) for v in (*p, *q)])
        return out

    def state(self, since: int, since_thermal: int = 0) -> dict:
        with self.lock:
            a, m = self.arm, self.arm.m
            g = self.ghost
            for j, s in enumerate(a.servos):
                g.qpos[j] = (s.regs.goal_position - s.mid) / TICKS_PER_RAD
            mujoco.mj_kinematics(m, g)
            joints = []
            for j, s in enumerate(a.servos):
                r = s.regs
                ol_need = r.protection_time * 0.01
                joints.append({
                    "name": JOINTS[j], "id": s.id,
                    "goal": round(_ticks_to_deg(s, r.goal_position), 2),
                    "sp": round(_ticks_to_deg(s, s.setpoint), 2),
                    "q": round(math.degrees(a.d.qpos[j]), 2),
                    "vel": round(math.degrees(a.d.qvel[j]), 1),
                    "duty": round(s.duty / 10.0, 1),
                    "current": round(s.current, 3),
                    "temp": round(s.temp_c, 2),
                    "faults": list(sts3215.status_names(s.status)),
                    "ol_frac": round(min(1.0, s._ol_timer / ol_need) if ol_need > 0 else 0.0, 3),
                    "oc_frac": round(min(1.0, s._oc_timer / (r.over_current_protection_time * 0.01))
                                     if r.over_current_protection_time else 0.0, 3),
                    "trips": dict(s.trips),
                    "torque": bool(r.torque_enable),
                    "range": [round(v, 1) for v in self.jnt_range[j]],
                    "overload_pct": r.overload_torque,
                    "protective_pct": r.protective_torque,
                    "max_temp": r.max_temperature,
                    "regs": {key: s.read(key) for key in EDITABLE_REGS},
                    "physics": {**s.heat_balance(),
                                **s.capability(a.supply_v, float(a.d.qvel[j]))},
                })
            tcp = a.tcp_world()
            rows = [row for sq, row in self.hist if sq > since]
            return {
                "t": round(a.t, 4), "paused": self.paused, "rtf": self.rtf,
                "supply_v": round(a.supply_v, 2),
                "total_current": round(sum(abs(s.current) for s in a.servos), 3),
                "floor": {"on": a.floor.enabled, "z": a.floor_z, "force": round(a.floor_force, 2),
                          "contacts": [[round(v, 5) for v in c] for c in a.contacts]},
                "payload_kg": self.payload_kg,
                "ambient_c": self.ambient_c,
                "tcp": [round(float(v), 5) for v in tcp],
                "bodies": self._bodies(a.d), "ghost": self._bodies(g),
                "joints": joints, "regs": dict(self.regs),
                "seq": self.seq, "hist": rows[-HIST_LEN:],
                "generation": self.generation,
                "thermal_seq": self.thermal_seq,
                "thermal_hist": [row for sq, row in self.thermal_hist if sq > since_thermal],
            }

    def analysis(self, joint: str) -> dict:
        if joint not in JOINTS:
            raise ValueError("unknown joint")
        with self.lock:
            s = self.arm.servos[JOINTS.index(joint)]
            stamp = int(self.arm.t)
            cached = self._analysis_cache.get(joint)
            if cached is not None and cached[0] == stamp:
                return cached[1]
            cap = s.capability(self.arm.supply_v, 0.0)
            span = max(1.0, cap["no_load_speed_deg_s"])
            envelope = []
            for i in range(41):
                speed = (i / 20.0 - 1.0) * span
                point = s.capability(self.arm.supply_v, math.radians(speed))
                envelope.append([speed, point["torque_positive_nm"], point["torque_negative_nm"]])
            result = {"joint": joint, "forecast": s.thermal_forecast(), "envelope": envelope}
            self._analysis_cache[joint] = (stamp, result)
            return result

    def model(self) -> dict:
        """Visual geometry: one entry per visual geom, meshes shared by id."""
        m = self.arm.m
        geoms = []
        for gi in range(m.ngeom):
            if m.geom_group[gi] != 2 or m.geom_type[gi] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = int(m.geom_dataid[gi])
            geoms.append({
                "body": int(m.geom_bodyid[gi]) - 1,
                "mesh": mid,
                "mesh_name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid),
                "pos": [float(v) for v in m.geom_pos[gi]],
                "quat": [float(v) for v in m.geom_quat[gi]],
            })
        bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(1, m.nbody)]
        body_joint = []
        for b in range(1, m.nbody):
            jn = m.body_jntnum[b]
            body_joint.append(JOINTS.index(mujoco.mj_id2name(
                m, mujoco.mjtObj.mjOBJ_JOINT, int(m.body_jntadr[b]))) if jn else -1)
        return {"geoms": geoms, "bodies": bodies, "body_joint": body_joint,
                "joints": JOINTS, "floor_z": self.arm.floor_z,
                "poses": {k: v for k, v in so101_poses.POSES.items()},
                "pose_bodies": {k: self.pose_bodies(v) for k, v in so101_poses.POSES.items()},
                "ref_bodies": self.pose_bodies(START_DEG),
                "editable_regs": list(EDITABLE_REGS), "reg_limits": REG_LIMITS,
                "motor_limits": MOTOR_LIMITS,
                "servo": {"stall_nm": sts3215.STALL_TORQUE_NM, "stall_a": sts3215.STALL_CURRENT_A,
                          "no_load_rpm": sts3215.NO_LOAD_RPM, "gear": sts3215.GEAR_RATIO,
                          "overcurrent_a": sts3215.OVERCURRENT_TRIP_A}}

    def pose_bodies(self, arm_deg, gripper_deg: float = 0.0) -> list:
        """Body poses of the arm at the given joint angles (kinematics only)."""
        m = self.arm.m
        d = mujoco.MjData(m)
        for j, a in enumerate(list(arm_deg)[:5]):
            d.qpos[j] = math.radians(a)
        d.qpos[5] = math.radians(gripper_deg)
        mujoco.mj_kinematics(m, d)
        return self._bodies(d)

    def mesh_bin(self, mid: int) -> bytes:
        m = self.arm.m
        if not 0 <= mid < m.nmesh:
            raise KeyError(mid)
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        v = np.ascontiguousarray(m.mesh_vert[va:va + vn], dtype="<f4")
        f = np.ascontiguousarray(m.mesh_face[fa:fa + fn], dtype="<u4")
        return struct.pack("<II", vn, fn) + v.tobytes() + f.tobytes()


SIM: Sim = None  # set in main()
MESH_CACHE: dict = {}
CTYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8"}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # quiet: the UI polls at 30 Hz
        pass

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def do_GET(self):
        u = urlparse(self.path)
        path = u.path
        if path in ("/", "/index.html"):
            return self._send(200, UI_HTML.read_bytes(), CTYPES[".html"])
        if path == "/api/state":
            since = int(parse_qs(u.query).get("since", ["0"])[0] or 0)
            since_thermal = int(parse_qs(u.query).get("since_thermal", ["0"])[0] or 0)
            return self._json(SIM.state(since, since_thermal))
        if path == "/api/analysis":
            try:
                return self._json(SIM.analysis(parse_qs(u.query).get("joint", [""])[0]))
            except ValueError as e:
                return self._json({"error": str(e)}, 400)
        if path == "/docs/servo-model":
            return self._send(200, (HERE.parent / "docs" / "SO101_SERVO_MODEL.md").read_bytes(),
                              "text/plain; charset=utf-8")
        if path == "/api/model":
            return self._json(SIM.model())
        if path.startswith("/api/mesh/"):
            try:
                mid = int(path.rsplit("/", 1)[1])
                if mid not in MESH_CACHE:
                    MESH_CACHE[mid] = SIM.mesh_bin(mid)
                return self._send(200, MESH_CACHE[mid], "application/octet-stream")
            except (ValueError, KeyError):
                return self._json({"error": "no such mesh"}, 404)
        if path.startswith("/static/"):
            f = (STATIC / path[len("/static/"):]).resolve()
            if STATIC.resolve() in f.parents and f.is_file():
                return self._send(200, f.read_bytes(),
                                  CTYPES.get(f.suffix, "application/octet-stream"))
        return self._json({"error": "not found"}, 404)

    def do_POST(self):
        if urlparse(self.path).path != "/api/cmd":
            return self._json({"error": "not found"}, 404)
        try:
            n = int(self.headers.get("Content-Length") or 0)
            cmd = json.loads(self.rfile.read(n) or b"{}")
            op = cmd.get("op")
            if op == "goal":
                SIM.set_goals(cmd.get("goals") or {})
            elif op == "torque":
                SIM.set_torque(bool(cmd.get("on")))
            elif op == "regs":
                SIM.set_regs(cmd.get("regs") or {}, cmd.get("joint"))
            elif op == "motor":
                SIM.set_motor(cmd.get("joint"), cmd.get("params") or {})
            elif op == "world":
                SIM.set_world(cmd.get("payload_kg"), cmd.get("floor"), cmd.get("ambient_c"))
            elif op == "run":
                SIM.set_run(cmd.get("paused"), cmd.get("rtf"))
            elif op == "reset":
                SIM.reset(cmd.get("pose"))
            else:
                return self._json({"ok": False, "error": f"unknown op {op!r}"}, 400)
            return self._json({"ok": True})
        except Exception as e:  # report, keep serving
            return self._json({"ok": False, "error": str(e)}, 400)


def main():
    global SIM
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=8025)
    ap.add_argument("--bind", default="127.0.0.1")
    args = ap.parse_args()
    SIM = Sim()
    threading.Thread(target=SIM.run_forever, daemon=True).start()
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"SO-101 simulator: http://localhost:{args.port}/", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
