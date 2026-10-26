"""Explicit, leased simulator-to-follower link; construction never opens a port.

Only the worker owns the bus. Observation uses raw read packets, including on
connect/disconnect: lerobot's setup, calibration and teleop cycles are not run.
The mock is a separate, labelled moving model, never a claim about hardware.

Polling is best-effort 25 Hz: each sample reads one telemetry block and six
torque registers. Serial transactions have short bounded read/write deadlines;
missing replies can make a cycle longer than 40 ms. The state lock serializes
I/O with stop invalidation, so snapshot() reads only cached data but can wait
for the active transaction batch. The watchdog uses wall time, never sim time;
it is software protection, not a replacement for the physical emergency stop.
"""
from __future__ import annotations

import copy
import json
import math
import pathlib
import queue
import secrets
import threading
import time
from types import SimpleNamespace

from . import profile, safety, sts3215
from .governor import FollowerGovernor
from .ports import FOLLOWER, classify_arm_voltage, list_so101_ports
from .servo_io import Command, RawFeetechIO

JOINTS = list(profile.JOINT_NAMES) + [profile.GRIPPER_NAME]
TICKS_PER_DEG = 4095.0 / 360.0  # installed lerobot MotorNormMode.DEGREES
PERIOD_S = 0.04
LEASE_S = 1.0
MAX_DPS = 15.0
FRESH_S = 0.3
CALIBRATION_PATH = (pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"
                    / "robots/so_follower/so101_follower.json")


def _goals(value):
    if not isinstance(value, dict) or set(value) != set(JOINTS):
        raise ValueError("goals must contain all six joint names in degrees")
    out = {}
    limits = dict(zip(JOINTS, profile.JOINT_LIMITS + [profile.GRIPPER_LIMIT_DEG]))
    for n, v in value.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f"{n}: angle must be a finite number in degrees")
        v = float(v)
        if not math.isfinite(v) or not limits[n][0] <= v <= limits[n][1]:
            raise ValueError(f"{n}: angle outside simulator degree limits")
        out[n] = v
    return out


def _load_calibration(path=None):
    data = json.loads(pathlib.Path(path or CALIBRATION_PATH).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != set(JOINTS):
        raise ValueError("calibration must contain exactly six motors")
    for i, n in enumerate(JOINTS, 1):
        c = data[n]
        fields = ("id", "drive_mode", "homing_offset", "range_min", "range_max")
        if not isinstance(c, dict) or any(type(c.get(k)) is not int for k in fields):
            raise ValueError(f"{n}: invalid calibration fields")
        if (c["id"] != i or c["drive_mode"] not in (0, 1)
                or not -2047 <= c["homing_offset"] <= 2047
                or not 0 <= c["range_min"] < c["range_max"] <= 4095):
            raise ValueError(f"{n}: invalid calibration range or motor identity")
    return data


def _to_deg(raw, c):
    # Homing_Offset is already applied by firmware. DEGREES in lerobot does
    # not apply drive_mode or clamp measured values, even for the gripper.
    return (raw - (c["range_min"] + c["range_max"]) / 2) / TICKS_PER_DEG


def _to_raw(deg, c):
    raw = int(deg * TICKS_PER_DEG + (c["range_min"] + c["range_max"]) / 2)
    if not c["range_min"] <= raw <= c["range_max"]:
        raise ValueError("target outside calibrated raw range")
    return raw


class _RealBackend:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, baudrate=1_000_000, timeout=0.002,
                                 write_timeout=0.05, exclusive=True)
        bus = SimpleNamespace(port_handler=SimpleNamespace(ser=self.ser))
        self.io = RawFeetechIO(bus, {n: i for i, n in enumerate(JOINTS, 1)})
        self.port = port

    def close(self):
        self.ser.close()


class _MockBackend:
    """Tick-domain mock with finite speed and observable stopping."""
    def __init__(self):
        self.io = self
        self.port = "mock"
        self.calibration = {n: {"id": i, "drive_mode": 0, "homing_offset": 0,
                                "range_min": 0, "range_max": 4095}
                            for i, n in enumerate(JOINTS, 1)}
        start = dict(zip(JOINTS, [0, -35, 45, -15, 0, 0]))
        self.pos = {n: float(_to_raw(start[n], self.calibration[n])) for n in JOINTS}
        self.goal = {n: int(v) for n, v in self.pos.items()}
        self.torque = {n: True for n in JOINTS}
        self.last = time.monotonic()
        self.writes = []

    def read_telemetry(self):
        now = time.monotonic()
        dt, self.last = min(0.1, now - self.last), now
        out = {}
        for n in JOINTS:
            gap = self.goal[n] - self.pos[n]
            step = min(abs(gap), MAX_DPS * TICKS_PER_DEG * dt)
            vel = math.copysign(step / dt, gap) if dt > 0 and self.torque[n] else 0
            if self.torque[n]:
                self.pos[n] += math.copysign(step, gap)
            out[n] = sts3215.ServoState(ok=True, pos=round(self.pos[n]),
                                        vel=round(vel), moving=int(abs(vel) > 1),
                                        volt=12.0, temp=27, current=0.1, t=now)
        return out

    def read_reg(self, reg, name):
        c = self.calibration[name]
        values = {"ID": c["id"], "Homing_Offset": c["homing_offset"],
                  "Min_Position_Limit": c["range_min"], "Max_Position_Limit": c["range_max"],
                  "Operating_Mode": 0, "Angular_Resolution": 1, "Phase": 4,
                  "P_Coefficient": 16,
                  "Torque_Enable": int(self.torque[name])}
        return values[reg], 0

    def sync_write(self, reg, values):
        self.writes.append((reg, dict(values)))
        if reg == "Torque_Enable":
            self.torque.update({n: bool(v) for n, v in values.items()})

    def write_commands(self, values):
        self.writes.append(("commands", copy.deepcopy(values)))
        self.goal.update({n: c.goal for n, c in values.items()})

    def close(self):
        pass


class HardwareLink:
    def __init__(self, mode="off", port="auto"):
        if mode not in ("off", "mock", "real"):
            raise ValueError("mode must be off, mock or real")
        if not isinstance(port, str) or not port.strip():
            raise ValueError("port must be auto or a serial port name")
        self.mode, self.port = mode, port
        self._lock = threading.RLock()
        self._queue = queue.PriorityQueue()
        self._thread = None
        self._closed = False
        self._sequence = 0
        self._generation = 0
        self._backend = None
        self._states = {}
        self._torque = {}
        self._cal = None
        self._governor = None
        self._target = None
        self._validated_target = None
        self._last_goal = {}
        self._owner = self._lease = None
        self._deadline = 0.0
        self._sample_time = None
        self._last_cycle = time.monotonic()
        self._stop_goal = None
        self._view = {"mode": mode, "phase": "off" if mode == "off" else "disconnected",
                      "connected": False, "following": False, "reason": "",
                      "joints": {}, "age_s": None, "sample_seq": 0,
                      "commands_sent": 0, "targets_received": 0, "owner": None,
                      "max_error_deg": None, "calibration": {"valid": False, "reason": "not checked"},
                      "stop_confirmed": False}

    def snapshot(self):
        with self._lock:
            out = copy.deepcopy(self._view)
            out["age_s"] = (None if self._sample_time is None
                            else max(0.0, time.monotonic() - self._sample_time))
            return out

    def set_target(self, goals):
        target = _goals(goals)
        with self._lock:
            if self._closed:
                raise ValueError("hardware link is closed")
            self._view["targets_received"] += 1
            # Stop discards pending simulator goals. A new explicit follow
            # supplies a complete fresh target and obtains a different lease.
            if self._view["following"]:
                self._target = target

    def command(self, op, payload=None):
        if op not in ("connect", "disconnect", "follow", "stop", "heartbeat"):
            raise ValueError("unknown hardware operation")
        payload = {} if payload is None else payload
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        if op in ("follow", "heartbeat"):
            client = payload.get("client_id")
            if not isinstance(client, str) or not client.strip() or len(client) > 128:
                raise ValueError("client_id must be a nonempty string up to 128 characters")
        if op == "follow":
            payload = {**payload, "goals": _goals(payload.get("goals"))}
        if op == "heartbeat" and not isinstance(payload.get("lease"), str):
            raise ValueError("heartbeat requires a lease")
        done = threading.Event()
        box = {}
        with self._lock:
            if self._closed:
                raise ValueError("hardware link is closed")
            if self.mode == "off":
                if op in ("stop", "disconnect"):
                    return {"ok": True, **self.snapshot()}
                raise ValueError("hardware mode is off")
            if self._thread is None:
                if op != "connect":
                    if op in ("disconnect", "stop"):
                        return {"ok": True, **self.snapshot()}
                    raise ValueError("connect before following")
                self._thread = threading.Thread(target=self._run, name="so101-twin-link", daemon=True)
                self._thread.start()
            if op in ("stop", "disconnect"):
                self._invalidate()
            self._sequence += 1
            self._queue.put((0 if op in ("stop", "disconnect") else 10, self._sequence,
                             self._generation, op, copy.deepcopy(payload), done, box))
        # Raw transactions have bounded deadlines. Do not return a timeout
        # while a queued follow might subsequently start moving.
        done.wait()
        if "error" in box:
            raise ValueError(str(box["error"]))
        return box["result"]

    def close(self):
        with self._lock:
            if self._closed:
                return
        try:
            self.command("disconnect")
        finally:
            with self._lock:
                self._closed = True
            if self._thread is not None:
                self._thread.join(timeout=2.0)

    def _invalidate(self):
        self._generation += 1
        self._owner = self._lease = self._target = None
        self._validated_target = None
        self._deadline = 0.0
        self._view.update(following=False, owner=None)

    def _open_backend(self):
        if self.mode == "mock":
            return _MockBackend()
        ports = [p for p in list_so101_ports() if (p.vid, p.pid) == (0x1A86, 0x55D3)]
        if self.port.lower() != "auto":
            ports = [p for p in ports if p.device.casefold() == self.port.casefold()]
        if not ports:
            raise ValueError("no matching CH343 follower port")
        found = []
        try:
            for p in ports:
                # Serial open is exclusive. Never close another process's port
                # or fall back to a cached leader/follower assignment.
                b = _RealBackend(p.device)
                try:
                    st = b.io.read_telemetry()
                    if (set(st) == set(JOINTS) and all(s.ok
                            and classify_arm_voltage(s.volt) == FOLLOWER for s in st.values())):
                        found.append(b)
                    else:
                        b.close()
                except Exception:
                    b.close()
                    raise
            if len(found) != 1:
                raise ValueError("follower identification is ambiguous or incomplete")
            return found.pop()
        finally:
            for b in found:
                b.close()

    def _check_calibration(self):
        self._cal = None
        try:
            cal = (copy.deepcopy(self._backend.calibration) if self.mode == "mock"
                   else _load_calibration())
            p_coeff = {}
            for n in JOINTS:
                c = cal[n]
                expected = {"ID": c["id"], "Homing_Offset": c["homing_offset"],
                            "Min_Position_Limit": c["range_min"],
                            "Max_Position_Limit": c["range_max"], "Operating_Mode": 0,
                            "Angular_Resolution": 1}
                for reg, want in expected.items():
                    got, status = self._backend.io.read_reg(reg, n)
                    if status or got != want:
                        raise ValueError(f"{n}: calibration/register mismatch ({reg})")
                phase, status = self._backend.io.read_reg("Phase", n)
                if status or phase is None or not phase & 4:
                    raise ValueError(f"{n}: velocity unit is not confirmed as ticks/s")
                p, status = self._backend.io.read_reg("P_Coefficient", n)
                if status or type(p) is not int or not 1 <= p <= 254:
                    raise ValueError(f"{n}: position gain is not confirmed")
                p_coeff[n] = p
            self._cal = cal
            ranges = {n: (c["range_min"], c["range_max"]) for n, c in cal.items()}
            self._governor = FollowerGovernor(JOINTS, ranges, p_coeff=p_coeff)
            self._view["calibration"] = {"valid": True, "reason": "mock" if self.mode == "mock" else "verified"}
        except Exception as e:
            self._view["calibration"] = {"valid": False, "reason": str(e)}

    def _read(self):
        st = self._backend.io.read_telemetry()
        if set(st) != set(JOINTS) or not all(s.ok for s in st.values()):
            raise RuntimeError("incomplete follower telemetry")
        torque = {}
        for n in JOINTS:
            value, status = self._backend.io.read_reg("Torque_Enable", n)
            if value not in (0, 1):
                raise RuntimeError(f"{n}: torque state unavailable")
            st[n].status |= status
            torque[n] = bool(value)
        self._states, self._torque = st, torque
        now = time.monotonic()
        self._sample_time = now
        self._view["sample_seq"] += 1
        self._view["joints"] = {
            n: {"q": _to_deg(s.pos, self._cal[n]) if self._cal else None,
                "raw": s.pos, "current": s.current, "temp": s.temp,
                "voltage": s.volt, "torque": torque[n], "status": s.status | s.err}
            for n, s in st.items()}
        if self._target is not None and self._cal:
            self._view["max_error_deg"] = max(abs(self._target[n] - _to_deg(st[n].pos, self._cal[n]))
                                               for n in JOINTS)
        if self._stop_goal is not None:
            self._view["stop_confirmed"] = all(
                abs(s.vel) <= TICKS_PER_DEG and not s.moving
                and abs(s.pos - self._stop_goal[n]) <= 1.5 * TICKS_PER_DEG
                and not s.status and not s.err and torque[n] for n, s in st.items())

    def _healthy(self):
        if self._sample_time is None or time.monotonic() - self._sample_time > FRESH_S:
            raise ValueError("fresh telemetry is required")
        for n, s in self._states.items():
            if (not all(math.isfinite(v) for v in (s.pos, s.vel, s.volt, s.temp, s.current))
                    or s.status or s.err or not 10.8 <= s.volt <= 13.0
                    or not 0 <= s.temp < 62 or abs(s.current) >= 1.9):
                raise ValueError(f"{n}: unsafe voltage, temperature, current or status")
            if self._cal and not self._cal[n]["range_min"] <= s.pos <= self._cal[n]["range_max"]:
                raise ValueError(f"{n}: position outside calibration")

    def _write(self, goals):
        cmds = {n: Command(goal=g, velocity=max(1, int(MAX_DPS * TICKS_PER_DEG)), acceleration=10)
                for n, g in goals.items() if self._last_goal.get(n) != g}
        if cmds:
            self._backend.io.write_commands(cmds)
            self._view["commands_sent"] += 1
            self._last_goal.update({n: c.goal for n, c in cmds.items()})

    def _halt(self, reason, fault=False):
        was_moving = self._view["following"] or bool(self._last_goal)
        self._invalidate()
        self._view.update(phase="fault" if fault else "stopped", reason=reason, stop_confirmed=False)
        self._stop_goal = None
        if was_moving and self._backend is not None:
            try:
                self._read()
                hold = {n: s.pos for n, s in self._states.items()}
                self._write(hold)
                self._stop_goal = hold
                # A successful write is not a physical stop acknowledgement.
                self._read()
            except Exception as e:
                self._view.update(phase="fault", reason=f"{reason}; stop unconfirmed: {e}", stop_confirmed=False)

    def _handle(self, generation, op, p):
        if op == "connect":
            if self._backend is None:
                self._backend = self._open_backend()
                self._last_goal = {}
                self._stop_goal = None
                self._view.update(connected=True, phase="observing", reason="", stop_confirmed=False)
                self._check_calibration()
                self._read()
            elif self._view["phase"] == "fault":
                raise ValueError("disconnect and reconnect to clear the fault")
        elif op == "disconnect":
            if self._backend is not None:
                self._halt("disconnected")
                self._backend.close()
                self._backend = None
            self._view.update(connected=False, phase="disconnected")
            self._last_goal = {}
        elif op == "stop":
            self._halt("operator stop")
        elif op == "heartbeat":
            if (not self._view["following"] or p["client_id"] != self._owner
                    or not secrets.compare_digest(p["lease"], self._lease or "")
                    or time.monotonic() >= self._deadline):
                raise ValueError("invalid or expired follow lease")
            self._deadline = time.monotonic() + LEASE_S
        elif op == "follow":
            if generation != self._generation:
                raise ValueError("follow cancelled by stop or disconnect")
            if self._backend is None or self._view["phase"] not in ("observing", "stopped"):
                raise ValueError("follow requires an explicit connection without a fault")
            if self._view["following"] or self._owner:
                raise ValueError("a client already owns following")
            self._check_calibration()
            if self._cal is None:
                raise ValueError("verified calibration is required for following")
            self._read()
            self._healthy()
            target = p["goals"]
            for n in JOINTS:
                _to_raw(target[n], self._cal[n])
                if abs(target[n] - _to_deg(self._states[n].pos, self._cal[n])) > 3.0:
                    raise ValueError(f"{n}: target must be within 3 degrees of hardware")
            ok, message, _ = safety.check_angles([_to_deg(self._states[n].pos, self._cal[n])
                                                 for n in profile.JOINT_NAMES])
            if not ok:
                raise ValueError(message)
            self._validate_path({n: _to_deg(self._states[n].pos, self._cal[n]) for n in JOINTS}, target)
            if any(abs(s.vel) > TICKS_PER_DEG or s.moving for s in self._states.values()):
                raise ValueError("the follower must be stationary before following")
            # Seed a hold at the measured position BEFORE enabling any torque.
            self._last_goal = {}
            self._write({n: s.pos for n, s in self._states.items()})
            off = {n: 1 for n in JOINTS if not self._torque[n]}
            if off:
                self._backend.io.sync_write("Torque_Enable", off)
                self._view["commands_sent"] += 1
                self._read()
                if not all(self._torque.values()):
                    raise RuntimeError("torque enable was not confirmed")
            self._governor.reset(self._states)
            self._owner, self._lease = p["client_id"], secrets.token_urlsafe(24)
            self._target = dict(target)
            self._validated_target = dict(target)
            self._deadline = time.monotonic() + LEASE_S
            self._last_cycle = time.monotonic()
            self._stop_goal = None
            self._view.update(phase="following", following=True, owner=self._owner,
                              stop_confirmed=False, reason="", targets_received=self._view["targets_received"] + 1)
            return {"ok": True, "lease": self._lease, "lease_s": LEASE_S, **self.snapshot()}
        return {"ok": True, **self.snapshot()}

    def _cycle(self):
        self._read()
        if not self._view["following"]:
            return
        now = time.monotonic()
        if now >= self._deadline:
            self._halt("follow heartbeat expired")
            return
        self._healthy()
        if not all(self._torque.values()):
            raise RuntimeError("torque state changed while following")
        current = {n: _to_deg(self._states[n].pos, self._cal[n]) for n in JOINTS}
        if self._target != self._validated_target:
            for n in JOINTS:
                _to_raw(self._target[n], self._cal[n])
            self._validate_path(current, self._target)
            self._validated_target = dict(self._target)
        dt, self._last_cycle = min(PERIOD_S * 2, now - self._last_cycle), now
        raw = {}
        for n in JOINTS:
            c = self._cal[n]
            previous = _to_deg(self._last_goal[n], c)
            goal = previous + max(-MAX_DPS * dt, min(MAX_DPS * dt, self._target[n] - previous))
            raw[n] = _to_raw(goal, c)
        output = self._governor.step(raw, self._states, self._torque, now)
        if (not output.follow_allowed or output.retreat
                or any(j.state in ("probe", "hold", "protect", "grip", "stale")
                       for j in output.joints.values())):
            self._halt(output.stop_reason or "governor protection", fault=True)
            return
        # Governor never gets to initiate cooling or to bypass the 15 deg/s
        # limiter. Clamp interventions are checked like every other target.
        bounded = {}
        for n in JOINTS:
            last = self._last_goal[n]
            step = max(0, int(MAX_DPS * dt * TICKS_PER_DEG * output.speed_scale))
            bounded[n] = min(last + step, max(last - step, output.goals[n]))
            if not self._cal[n]["range_min"] <= bounded[n] <= self._cal[n]["range_max"]:
                raise RuntimeError("governor target outside calibration")
        end = {n: _to_deg(bounded[n], self._cal[n]) for n in JOINTS}
        self._validate_path(current, end)
        self._write(bounded)

    def _validate_path(self, current, end):
        steps = max(1, math.ceil(max(abs(end[n] - current[n]) for n in JOINTS) / 0.5))
        for i in range(steps + 1):
            q = [current[n] + (end[n] - current[n]) * i / steps for n in profile.JOINT_NAMES]
            ok, message, _ = safety.check_angles(q)
            if not ok:
                raise ValueError(message)

    def _run(self):
        while True:
            with self._lock:
                if self._closed:
                    if self._backend is not None:
                        self._halt("link closed")
                        self._backend.close()
                        self._backend = None
                    self._view.update(connected=False, phase="disconnected")
                    while not self._queue.empty():
                        pending = self._queue.get_nowait()
                        pending[-1]["error"] = "hardware link is closed"
                        pending[-2].set()
                    return
            try:
                item = self._queue.get(timeout=PERIOD_S)
            except queue.Empty:
                item = None
            with self._lock:
                if item is not None:
                    _, _, generation, op, payload, done, box = item
                    try:
                        box["result"] = self._handle(generation, op, payload)
                    except Exception as e:
                        box["error"] = str(e)
                        if op == "connect":
                            if self._backend is not None:
                                self._backend.close()
                                self._backend = None
                            self._invalidate()
                            self._view.update(connected=False, phase="fault", reason=str(e))
                        elif not isinstance(e, ValueError):
                            self._halt(str(e), fault=True)
                    finally:
                        done.set()
                if self._backend is not None:
                    try:
                        self._cycle()
                    except Exception as e:
                        if self._view["phase"] != "fault":
                            self._halt(str(e), fault=True)
