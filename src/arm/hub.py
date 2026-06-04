"""Hardware/virtual abstraction for the arm + camera.

`HubBase` is the protocol the HTTP server uses. `Hub` is the real implementation;
`VirtualHub` is for UI development without hardware (supports fault injection
via env var `VHUB_FAULT` so error paths stay exercisable).
"""
from __future__ import annotations
import abc, os, sys, time, threading, logging
from typing import Optional, List, Tuple

from .constants import (
    DEFAULT_CAM_INDEX, HOME_ANGLES,
    WAYPOINT_WAIT, WAYPOINT_TOLERANCE, WAYPOINT_TIMEOUT,
    STALL_WINDOW_S, STALL_MIN_PROGRESS_DEG, STALL_GRACE_S, STALL_MIN_REMAINING_DEG,
)
from .planner import plan_and_validate
from .current_monitor import CurrentMonitor

log = logging.getLogger("mycobot.hub")


class HubBase(abc.ABC):
    offline: bool = False
    io_lock: threading.Lock
    motion_lock: threading.Lock
    abort_flag: threading.Event

    # (instance-initialized in __init__)
    monitor_enabled: bool
    _monitor: Optional[CurrentMonitor]

    @abc.abstractmethod
    def angles(self) -> Optional[List[float]]: ...
    @abc.abstractmethod
    def power_ok(self) -> bool: ...
    @abc.abstractmethod
    def send_angles_and_wait(self, angles, speed: int) -> Tuple[bool, Optional[List[float]]]: ...
    @abc.abstractmethod
    def release(self) -> None: ...
    @abc.abstractmethod
    def set_gripper(self, flag: int, speed: int) -> bool: ...

    def gripper_diag(self) -> dict:
        """Read-only gripper bus probe (overridden by real Hub)."""
        return {"supported": False}

    def servo_scan(self) -> dict:
        """Read encoders of servos 1..7 (overridden by real Hub).
        Used to identify the gripper servo by hand-moving it and watching
        which encoder changes."""
        return {"supported": False}

    def mc_call(self, method: str, args: list):
        """Whitelisted raw pymycobot call for gripper/IO brute-force probing
        (overridden by real Hub)."""
        return {"supported": False}
    @abc.abstractmethod
    def solve_ik(self, coords6, seed: Optional[List[float]] = None) -> Optional[List[float]]: ...
    @abc.abstractmethod
    def live_coords(self) -> Optional[List[float]]: ...

    def solve_ik_with_mode(self, coords_or_pos, seed: Optional[List[float]] = None) -> tuple[Optional[List[float]], str]:
        """Return (angles, mode) so callers can know which fallback path succeeded.

        coords_or_pos: length 3 → position-only; length 6 → full pose (x,y,z,rx,ry,rz).
        mode ∈ {"full","relaxed_roll","position_only","failed","firmware"}.
        Calls firmware IK once (full pose only) and numeric retries once — no duplication.
        """
        from .ik_numeric import solve_with_retries
        if seed is None:
            angles = self.angles()
            if not angles: return None, "failed"
            seed = list(angles)
        # NOTE: HOME_SEED second-pass retry was removed for UI latency — it
        # doubled the worst-case wall-clock (each pass burns its own time_budget).
        # solve_with_retries already includes HOME-clamped seed perturbations
        # internally, so the second pass rarely added coverage in practice.

        def _try(s, ori):
            return solve_with_retries(coords_or_pos[:3], ori, s)

        if len(coords_or_pos) >= 6:
            # try firmware first (full pose); on success return mode="firmware"
            fw = self._firmware_ik_only(list(coords_or_pos), list(seed))
            if fw is not None:
                return fw, "firmware"
            ori = (coords_or_pos[3], coords_or_pos[4], coords_or_pos[5])
            return _try(seed, ori)
        else:
            return _try(seed, None)

    def _firmware_ik_only(self, coords6, seed):
        """Firmware solve_inv_kinematics, no numeric fallback. Returns angles or None."""
        return None  # default: subclass overrides
    @abc.abstractmethod
    def frame_jpeg(self, fresh: bool = False) -> Optional[bytes]: ...
    @abc.abstractmethod
    def shutdown(self) -> None: ...
    @abc.abstractmethod
    def get_currents(self) -> Optional[List[int]]: ...
    @abc.abstractmethod
    def get_servo_diagnostics(self) -> dict: ...

    def start_monitor(self):
        """Start current monitor (no-op if disabled or already running)."""
        if not self.monitor_enabled: return
        if self._monitor and self._monitor.is_alive(): return
        def on_over(cs):
            log.warning("過電流検出 abort 発火: %s mA", cs)
            self.abort_flag.set()
        self._monitor = CurrentMonitor(self.get_currents, on_over)
        self._monitor.start()

    def stop_monitor(self) -> None:
        m = self._monitor
        if m:
            m.stop()
            m.join(timeout=1.5)
            if m.is_alive():
                log.warning("CurrentMonitor did not exit within timeout — possible serial stall")
        self._monitor = None

    def home_blocking(self, speed: int = 25) -> None:
        """Validated home: plan from current → HOME, safety-check every step.

        After safety validation, if the path is joint-space monotonic, the
        actual motion is sent as a single firmware command (smooth, no per-
        waypoint deceleration). Falls back to chunked if non-monotonic.
        """
        from .constants import SMOOTH_SINGLE_SHOT as _smooth
        cur = self.angles()
        if cur is None:
            raise RuntimeError("cannot read current angles before home")
        waypoints, ok, msg, _bad = plan_and_validate(cur, HOME_ANGLES)
        if not ok:
            raise RuntimeError(f"home path unsafe: {msg}")
        self.abort_flag.clear()
        self.start_monitor()
        try:
            # Smooth single-shot if the validated path is monotonic
            if _smooth and len(waypoints) > 1:
                dirs = [0]*6
                mono = True
                prev = cur
                for w in waypoints:
                    for j in range(6):
                        d = w[j] - prev[j]
                        if abs(d) < 1e-6: continue
                        sign = 1 if d > 0 else -1
                        if dirs[j] == 0: dirs[j] = sign
                        elif dirs[j] != sign: mono = False; break
                    if not mono: break
                    prev = w
                if mono:
                    if self.abort_flag.is_set():
                        raise RuntimeError("home aborted")
                    reached, _ = self.send_angles_and_wait(HOME_ANGLES, speed)
                    if not reached:
                        raise RuntimeError("home single-shot readback timeout")
                    return
            for wp in waypoints:
                if self.abort_flag.is_set():
                    raise RuntimeError("home aborted")
                reached, _ = self.send_angles_and_wait(wp, speed)
                if not reached:
                    raise RuntimeError("home waypoint readback timeout")
        finally:
            self.stop_monitor()


class Hub(HubBase):
    """Real hardware hub."""

    def __init__(self, cam_index: int = DEFAULT_CAM_INDEX):
        from .client import Arm
        import cv2

        print("connecting arm...")
        self.arm = Arm()
        self.arm.power_on()
        print(f"arm ok on {self.arm.port}")

        self._cv2 = cv2
        try:
            self.cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
            for _ in range(8):
                self.cap.read(); time.sleep(0.05)
        except Exception:
            self.cap = None
            print("[warn] camera open failed; running without camera")

        self.io_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.cam_lock = threading.Lock()
        self.abort_flag = threading.Event()
        self._last_jpeg: Optional[bytes] = None
        self._cam_warned = False
        self.monitor_enabled = True
        self._monitor = None
        self.last_stall_info = None  # populated by send_angles_and_wait on stall detect

    def angles(self):
        with self.io_lock:
            a = self.arm.angles()
            return list(a) if a and a != -1 else None

    def power_ok(self) -> bool:
        with self.io_lock:
            for _ in range(2):  # one retry; firmware ACK can drop
                try:
                    r = self.arm.mc.is_power_on()
                    if r == 1:
                        return True
                    if r == 0:
                        return False
                    # r == -1 (unknown) → retry
                except Exception as e:
                    log.warning("is_power_on raised: %s", e)
                time.sleep(0.05)
            return False  # consistent -1 means we can't confirm power

    def submit_jog(self, angles, speed):
        """Coalescing teleop submit. Stores latest (angles, speed) in a single
        slot; intermediate values are dropped. A worker thread consumes the
        slot and calls send_angles serially. Returns immediately (no serial
        wait). Caller should still pre-validate safety.

        Why: send_angles blocks ~700ms on serial round-trip. At 30Hz client
        send rate, the io_lock backlogs and arm chases stale targets long
        after the user released pinch. With coalescing, the worker always
        consumes the freshest target; intermediate ones are silently dropped.
        """
        if not hasattr(self, '_jog_worker_started'):
            self._jog_latest = None
            self._jog_lock = threading.Lock()
            self._jog_event = threading.Event()
            self._jog_cycle_ms = []   # ring buffer of send_angles durations (diagnostics)
            self._real_angles = None  # cached readback for VR display (avoids HTTP io_lock contention)
            self._last_read_t = 0.0
            self._jog_thread = threading.Thread(target=self._jog_loop, daemon=True, name="jog-worker")
            self._jog_worker_started = True
            self._jog_thread.start()
        with self._jog_lock:
            self._jog_latest = (list(angles), int(speed))
        self._jog_event.set()
        return True

    def _jog_loop(self):
        # NOTE: no power_ok() in the hot loop. power_ok() is a full is_power_on()
        # serial round-trip (with retries+sleeps) — calling it every cycle doubled
        # the bus traffic and halved teleop refresh rate. Power is verified at
        # connect; if servos drop, send_angles raises and we log it.
        while True:
            self._jog_event.wait()
            self._jog_event.clear()
            with self._jog_lock:
                payload = self._jog_latest
                self._jog_latest = None
            if payload is None:
                continue
            angles, speed = payload
            try:
                t0 = time.perf_counter()
                with self.io_lock:
                    self.arm.mc.send_angles(list(angles), speed)
                dt = (time.perf_counter() - t0) * 1000.0
                self._jog_cycle_ms.append(round(dt, 1))
                if len(self._jog_cycle_ms) > 200:
                    self._jog_cycle_ms = self._jog_cycle_ms[-200:]
                # 実機角度の readback を ~6Hz でキャッシュ（VR 表示用）。送信を所有する
                # このスレッドで読むので io_lock 競合無し。HTTP /real_angles はキャッシュを返す。
                now = time.perf_counter()
                if now - self._last_read_t > 0.15:
                    self._last_read_t = now
                    with self.io_lock:
                        a = self.arm.angles()
                    if a and a != -1:
                        self._real_angles = list(a)
            except Exception as e:
                log.warning(f"jog worker send_angles: {e}")

    def send_angles_nowait(self, angles, speed):
        """Fire-and-forget send_angles for teleop/jog. No completion wait.
        Returns True iff command was issued (power on, no I/O error)."""
        if not self.power_ok():
            return False
        try:
            with self.io_lock:
                self.arm.mc.send_angles(list(angles), speed)
            return True
        except Exception as e:
            log.warning(f"send_angles_nowait: {e}")
            return False

    def set_gripper(self, flag, speed):
        """Adaptive electric gripper: flag 0=open, 1=close, 10=release.
        Fire-and-forget. Returns True iff the command was issued."""
        from .constants import GRIPPER_TYPE_ADAPTIVE
        try:
            with self.io_lock:
                self.arm.mc.set_gripper_state(int(flag), int(speed), GRIPPER_TYPE_ADAPTIVE)
            return True
        except Exception as e:
            log.warning(f"set_gripper: {e}")
            return False

    def gripper_diag(self):
        """Probe whether the firmware answers gripper protocol frames.
        Values of -1 mean 'no valid reply' (firmware/gripper not responding)."""
        mc = self.arm.mc
        out = {"supported": True}
        with self.io_lock:
            for name, fn in (
                ("version", lambda: mc.get_system_version()),
                ("gripper_mode", lambda: mc.get_gripper_mode()),
                ("set_value_reply", lambda: mc.set_gripper_value(50, 50, 1)),
                ("init_electric_reply", lambda: mc.init_electric_gripper()),
            ):
                try:
                    out[name] = fn()
                except Exception as e:
                    out[name] = f"ERR: {e}"
        return out

    def servo_scan(self):
        """Read servo positions 1..7 to identify the gripper by hand-moving it.
        get_encoder is range-limited to 1..6, so for a possible 7th (gripper)
        servo we read the present-position register (addr 56, 2-byte) directly
        via get_servo_data, which accepts ids 1..7."""
        mc = self.arm.mc
        out = {"supported": True}
        with self.io_lock:
            try:
                out["encoders_1_6"] = mc.get_encoders()
            except Exception as e:
                out["encoders_1_6"] = f"ERR: {e}"
            pos = {}
            for sid in range(1, 8):
                try:
                    pos[sid] = mc.get_servo_data(sid, 56, 1)  # present position (STS/SCS reg 0x38)
                except Exception as e:
                    pos[sid] = f"ERR: {e}"
            out["pos_reg56"] = pos
        return out

    # Whitelist of gripper/IO/servo methods safe to invoke for brute-force
    # probing. Excludes anything that moves joints or cuts power.
    _MC_CALL_WHITELIST = frozenset({
        "set_gripper_state", "set_gripper_value", "set_gripper_mode",
        "set_gripper_calibration", "init_electric_gripper", "set_electric_gripper",
        "get_gripper_mode",
        "set_pro_gripper", "set_pro_gripper_open", "set_pro_gripper_close",
        "set_pro_gripper_angle", "get_pro_gripper_status", "get_pro_gripper_angle",
        "set_basic_output", "get_basic_input", "set_digital_output",
        "get_digital_input", "set_pin_mode",
        "get_servo_data", "get_system_version", "is_gripper_moving",
    })

    def mc_call(self, method, args):
        if method not in self._MC_CALL_WHITELIST:
            raise ValueError(f"method '{method}' not in probe whitelist")
        fn = getattr(self.arm.mc, method, None)
        if fn is None:
            raise ValueError(f"pymycobot has no method '{method}'")
        with self.io_lock:
            return fn(*(args or []))

    def send_angles_and_wait(self, angles, speed):
        # power gate: don't issue a command we know will silently fail
        if not self.power_ok():
            log.warning("send_angles_and_wait: servos unpowered, refusing")
            self.abort_flag.set()
            return False, self.angles()
        with self.io_lock:
            self.arm.mc.send_angles(list(angles), speed)
        time.sleep(WAYPOINT_WAIT)
        t0 = time.time()
        power_check_cnt = 0
        # Stall detection ring buffer: (timestamp, angles)
        # When the arm hits something (table, self, joint limit, latched servo),
        # angles stop progressing while the command is still pending. This is far
        # more reliable than current monitoring (which doesn't reflect torque on
        # this firmware — observed peak 24mA even when user pushes hard).
        recent = []
        # Track what triggered termination so callers (UI / abort handler) can show it
        self.last_stall_info = None
        while time.time() - t0 < WAYPOINT_TIMEOUT:
            if self.abort_flag.is_set():
                return False, self.angles()
            power_check_cnt += 1
            if power_check_cnt % 5 == 0 and not self.power_ok():
                log.warning("power lost mid-motion → abort")
                self.abort_flag.set()
                return False, self.angles()
            cur = self.angles()
            if cur is None:
                time.sleep(0.1); continue
            self._real_angles = list(cur)  # VR 表示用キャッシュ更新（home/move 中も不透明アームが追従）
            if all(abs(c - a) <= WAYPOINT_TOLERANCE for c, a in zip(cur, angles)):
                return True, cur
            # stall detection: enough samples covering the window, max joint
            # movement across the window < threshold, and we're still far from target
            now = time.time()
            recent.append((now, cur))
            recent = [(t, a) for t, a in recent if now - t <= STALL_WINDOW_S]
            elapsed = now - t0
            if (elapsed >= STALL_GRACE_S
                and len(recent) >= 4
                and (now - recent[0][0]) >= STALL_WINDOW_S * 0.7):
                # max range of each joint over window
                max_move = max(
                    max(a[j] for _, a in recent) - min(a[j] for _, a in recent)
                    for j in range(6)
                )
                remaining = [abs(cur[j] - angles[j]) for j in range(6)]
                far_from_target = any(r > STALL_MIN_REMAINING_DEG for r in remaining)
                if max_move < STALL_MIN_PROGRESS_DEG and far_from_target:
                    stuck = [j + 1 for j in range(6) if remaining[j] > STALL_MIN_REMAINING_DEG]
                    msg = (f"motion stall: joints {stuck} stopped at {[round(cur[j-1],1) for j in stuck]}"
                           f" (target {[round(angles[j-1],1) for j in stuck]}, max_move={max_move:.2f}°)")
                    log.warning(msg)
                    self.last_stall_info = {
                        "stuck_joints": stuck,
                        "current": [round(c, 2) for c in cur],
                        "target":  [round(a, 2) for a in angles],
                        "remaining_deg": [round(r, 2) for r in remaining],
                        "max_move_in_window_deg": round(max_move, 2),
                        "window_s": STALL_WINDOW_S,
                    }
                    self.abort_flag.set()
                    return False, cur
            time.sleep(0.1)
        log.warning("waypoint readback timeout: target=%s actual=%s", angles, self.angles())
        return False, self.angles()

    def release(self):
        with self.io_lock:
            self.arm.release()

    def power_on(self):
        """Re-energize servos at the current (hand-posed) position. Used by the
        teach flow: 脱力 → 手で動かす → 確定 で、確定時にその姿勢を保持させる。"""
        with self.io_lock:
            self.arm.power_on()

    def _firmware_ik_only(self, coords6, seed):
        """Firmware IK only (no numeric fallback). Used by solve_ik_with_mode."""
        with self.io_lock:
            try:
                res = self.arm.mc.solve_inv_kinematics(list(coords6), list(seed))
                if res and res != -1:
                    return list(res)
            except Exception as e:
                log.warning("solve_inv_kinematics raised: %s", e)
        return None

    def solve_ik(self, coords6, seed=None):
        """Resolve IK for a 6-DoF target (x,y,z,rx,ry,rz).

        Order of attempts:
          1. firmware solve_inv_kinematics (full pose, fast)
          2. numeric IK with multi-seed retry + roll relaxation (natural orientation)
          3. numeric position-only as last resort
        """
        if seed is None:
            cur = self.angles()
            if not cur: return None
            seed = list(cur)
        fw = self._firmware_ik_only(list(coords6), list(seed))
        if fw is not None: return fw
        from .ik_numeric import solve_with_retries
        orientation = (coords6[3], coords6[4], coords6[5]) if len(coords6) >= 6 else None
        sol, mode = solve_with_retries(coords6[:3], orientation, seed)
        if mode != "full" and mode != "failed":
            log.info("Hub.solve_ik fallback mode=%s", mode)
        return sol

    def frame_jpeg(self, fresh: bool = False):
        """Capture and return a JPEG. fresh=True flushes the OS/driver buffer
        first (drops ~5 stale frames) — important after arm motion or pose
        changes when you want the current scene, not the cached one."""
        if self.cap is None:
            return self._last_jpeg
        with self.cam_lock:
            if fresh:
                # Drain the camera's internal buffer. cv2.VideoCapture on DSHOW
                # buffers several frames, so a single read() right after motion
                # returns an OLD frame from before the move. Reading + tossing
                # 5-6 frames forces a current capture.
                for _ in range(6):
                    self.cap.grab()  # cheap: doesn't decode, just advances
            ok, frame = self.cap.read()
            if ok:
                ok2, buf = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok2:
                    self._last_jpeg = buf.tobytes()
                    self._cam_warned = False
            elif not self._cam_warned:
                log.warning("camera read failed (subsequent failures will be silent)")
                self._cam_warned = True
            return self._last_jpeg

    def get_currents(self):
        with self.io_lock:
            try:
                cs = self.arm.mc.get_servo_currents()
                return list(cs) if cs and cs != -1 and isinstance(cs, list) else None
            except Exception:
                return None

    def get_servo_diagnostics(self, full: bool = False):
        """Batched per-servo diagnostics for UI live monitoring.

        fast mode (default): currents + is_all_servo_enable (~100-150ms total)
        full mode: + per-servo enable flags + temps + voltages (~1-2s, serial bound)

        UI polls fast at 500ms and full every few seconds.
        """
        with self.io_lock:
            out = {"currents": None, "enabled": None, "temps": None, "voltages": None, "all_enabled": None}
            try:
                cs = self.arm.mc.get_servo_currents()
                if cs and cs != -1 and isinstance(cs, list):
                    out["currents"] = list(cs)
            except Exception: pass
            try:
                r = self.arm.mc.is_all_servo_enable()
                out["all_enabled"] = 1 if r == 1 else 0 if r == 0 else None
            except Exception: pass
            if not full:
                return out
            try:
                ts = self.arm.mc.get_servo_temps()
                if ts and ts != -1 and isinstance(ts, list):
                    out["temps"] = list(ts)
            except Exception: pass
            try:
                vs = self.arm.mc.get_servo_voltages()
                if vs and vs != -1 and isinstance(vs, list):
                    out["voltages"] = list(vs)
            except Exception: pass
            try:
                en = []
                for j in range(1, 7):
                    r = self.arm.mc.is_servo_enable(j)
                    en.append(1 if r == 1 else 0 if r == 0 else None)
                out["enabled"] = en
            except Exception: pass
            return out

    def live_coords(self) -> Optional[List[float]]:
        """End-effector pose from firmware (preferred over FK; gives correct tool orientation)."""
        with self.io_lock:
            try:
                c = self.arm.coords()
                return list(c) if c and c != -1 and isinstance(c, list) and len(c) == 6 else None
            except Exception:
                return None

    def shutdown(self):
        try: self.stop_monitor()
        except Exception: pass
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception as e:
            log.warning("cap.release: %s", e)
        try:
            self.arm.close()
        except Exception as e:
            log.warning("arm.close: %s", e)


class VirtualHub(HubBase):
    """In-memory virtual arm. Supports fault injection via VHUB_FAULT env var.

    VHUB_FAULT=power → power_ok returns False
    VHUB_FAULT=timeout → send_angles_and_wait returns (False, ...)
    """

    def __init__(self):
        self._angles = list(HOME_ANGLES)
        self.io_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.abort_flag = threading.Event()
        self.offline = True
        self.monitor_enabled = True
        self._monitor = None
        self._fault = os.environ.get("VHUB_FAULT", "")
        self._gripper_flag = None  # last commanded gripper flag (None = unknown)
        print(f"[OFFLINE] virtual arm at HOME (fault={self._fault!r})")

    def angles(self):
        with self.io_lock:
            return list(self._angles)

    def power_ok(self):
        return self._fault != "power"

    def send_angles_nowait(self, angles, speed):
        if self._fault == "power":
            return False
        with self.io_lock:
            self._angles = list(angles)
        return True

    def set_gripper(self, flag, speed):
        if self._fault == "power":
            return False
        self._gripper_flag = int(flag)
        return True

    def send_angles_and_wait(self, angles, speed):
        if self._fault == "timeout":
            time.sleep(0.05)
            return False, list(self._angles)
        with self.io_lock:
            self._angles = list(angles)
        time.sleep(0.05)
        return True, list(angles)

    def release(self): pass
    def power_on(self): pass

    def solve_ik(self, coords6, seed=None):
        """Offline: numeric DLS with multi-seed retry + orientation relaxation."""
        from .ik_numeric import solve_with_retries
        if seed is None:
            seed = list(self._angles)
        orientation = (coords6[3], coords6[4], coords6[5]) if len(coords6) >= 6 else None
        sol, _mode = solve_with_retries(coords6[:3], orientation, seed)
        return sol

    def live_coords(self):
        # offline has no real cartesian; compute via FK so cartesian routes can still validate paths
        from .kinematics import end_effector
        tip = end_effector(self._angles)
        return [tip[0], tip[1], tip[2], 0.0, 0.0, 0.0]

    def frame_jpeg(self, fresh: bool = False):
        import numpy as np, cv2
        img = np.full((360, 480, 3), 30, dtype=np.uint8)
        cv2.putText(img, "OFFLINE", (140, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (180, 180, 200), 3)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else None

    def get_currents(self):
        # Offline: synthesize a constant low value (or simulate overcurrent if fault injected)
        if self._fault == "overcurrent":
            return [2000, 100, 100, 100, 100, 100]
        return [80, 80, 80, 80, 80, 80]

    def get_servo_diagnostics(self, full: bool = False):
        out = {
            "currents": self.get_currents(),
            "all_enabled": 1,
            "enabled": None, "temps": None, "voltages": None,
        }
        if full:
            out.update(enabled=[1,1,1,1,1,1], temps=[35,35,35,35,35,35], voltages=[24.0]*6)
        return out

    def shutdown(self):
        try: self.stop_monitor()
        except Exception: pass
