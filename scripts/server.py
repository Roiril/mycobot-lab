"""HTTP control server for myCobot 320 (joint-space safety).

Usage:
  python scripts/server.py                    # live, loopback only
  python scripts/server.py --offline          # virtual arm (UI dev/test)
  python scripts/server.py --bind 0.0.0.0 --token SECRET  # LAN access with auth
  python scripts/server.py --port 8000 --cam 3 --max-speed 40

Endpoints:
  GET  /                  control UI
  GET  /kinematics        DH / limits / floor (single source of truth)
  GET  /angles            current joint angles
  GET  /coords            end-effector cartesian
  GET  /power             {ok: bool}
  GET  /fk?angles=...     joint positions for arbitrary angles
  POST /check {angles}    safety check → {ok, msg, badJoints}
  POST /solve_ik {x,y,z[,rx,ry,rz]} → {ok, angles?, msg}
  POST /move {angles, speed, expected_current?} → executes joint-space path
  POST /home              validated return to HOME
  POST /abort             interrupt the currently executing /move or /home
  POST /release           release all servos (arm goes limp)
  GET  /frame.jpg         live camera frame
"""
from __future__ import annotations
import sys, json, math, time, socket, pathlib, argparse, logging
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Optional
from urllib.parse import urlparse, parse_qs

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.kinematics import joint_positions, end_effector, JOINT_LIMITS, DH  # noqa: E402
from arm.safety import check_angles  # noqa: E402
from arm.planner import plan_and_validate  # noqa: E402
from arm.path_cartesian import linear as cart_linear, lift_translate_lower  # noqa: E402
from arm.ik_path import plan_ik_path  # noqa: E402
from arm.pose_resolver import resolve_pose  # noqa: E402
from arm.hub import Hub, VirtualHub, HubBase  # noqa: E402
from arm.constants import (  # noqa: E402
    MAX_SPEED, DEFAULT_PORT, DEFAULT_CAM_INDEX, HOME_ANGLES,
    ANGLE_DRIFT_TOL, TOOL_LENGTH, FLOOR_Z, LINK_RADIUS, TABLE_MARGIN, FK_TOOL_SLOP,
    CURRENT_THRESHOLD_MA, CURRENT_POLL_HZ, SUSTAINED_OVER_COUNT,
    SAFE_MODE_CURRENT_MA, CALIBRATION_MARKER,
    GRASP_APPROACH_OFFSET_MM, GRASP_LIFT_OFFSET_MM,
    GRASP_APPROACH_SPEED_DEFAULT, GRASP_APPROACH_SPEED_MAX,
    GRASP_OFFSET_MIN_MM, GRASP_OFFSET_MAX_MM,
    TARGET_RADIUS_MIN_MM, TARGET_RADIUS_MAX_MM, TARGET_RADIUS_DEFAULT_MM,
    GRIPPER_TIP_CLEARANCE_MM,
)

log = logging.getLogger("mycobot.server")

HUB: HubBase | None = None
INDEX_HTML = ""
AUTH_TOKEN: str | None = None
SHUTTING_DOWN = False
MAX_SPEED_RUNTIME = MAX_SPEED  # CLI-overridable


def _preflight(body) -> tuple[Optional[dict], Optional[list[float]], Optional[int]]:
    """Common pre-checks for /move and /move_cartesian.
    Returns (error_payload, current_angles, speed). On success error_payload is None.
    """
    try:
        speed = int(body.get("speed", 20))
    except (TypeError, ValueError):
        return {"error": "speed not int"}, None, None
    if not 1 <= speed <= MAX_SPEED_RUNTIME:
        return {"error": f"speed must be 1..{MAX_SPEED_RUNTIME}"}, None, None
    if SHUTTING_DOWN:
        return {"error": "server shutting down", "code": 503}, None, None
    if not HUB.power_ok():
        return {"error": "サーボ未通電 (E-stop?)", "code": 503}, None, None
    cur = HUB.angles()
    if cur is None:
        return {"error": "current angles unavailable", "code": 503}, None, None
    expected = body.get("expected_current")
    if expected is not None:
        try: exp = _coerce_angles(expected)
        except ValueError as e:
            return {"error": f"expected_current: {e}"}, None, None
        drift = max(abs(c - e) for c, e in zip(cur, exp))
        if drift > ANGLE_DRIFT_TOL:
            return {"error": f"start drift {drift:.1f}° > {ANGLE_DRIFT_TOL}°、再プレビューを", "code": 409}, None, None
    return None, cur, speed


def _execute_joint_waypoints(joint_wps, speed: int) -> tuple[int, dict]:
    """Drive waypoints with monitor + abort handling. Returns (http_code, body)."""
    HUB.abort_flag.clear()
    HUB.start_monitor()
    t0 = time.time()
    try:
        for idx, wp in enumerate(joint_wps):
            if HUB.abort_flag.is_set():
                triggered = HUB._monitor and HUB._monitor.triggered
                tag = "over-current" if triggered else "user"
                body = {"error": f"aborted ({tag})", "lastIndex": idx}
                if HUB._monitor:
                    body["currents"] = HUB._monitor.last_currents
                    body["peakJoint"] = HUB._monitor.peak_joint
                    body["peakValue"] = HUB._monitor.peak_value
                return 499, body
            reached, actual = HUB.send_angles_and_wait(wp, speed)
            if not reached:
                return 503, {"error": f"waypoint {idx+1}/{len(joint_wps)} 到達タイムアウト", "lastActual": actual}
        peak = HUB._monitor.peak_currents if HUB._monitor else None
        return 200, {"angles": HUB.angles(), "elapsed": round(time.time() - t0, 2),
                     "peakCurrents": peak, "monitorEnabled": HUB.monitor_enabled}
    finally:
        HUB.stop_monitor()


def _diagnose_ik_failure(hub, position, requested_rxyz, current_angles) -> dict:
    """When IK fails entirely, diagnose why so the caller (UI or LLM) gets a structured reason.

    Returns {code, message, diagnostics, retry_hints[]}.
    Codes:
      OUT_OF_REACH           — position itself unreachable (even with arbitrary orientation)
      ORIENTATION_INFEASIBLE — position reachable, but not with the requested orientation
      SOLVER_NONCONVERGENT   — both attempts hit numeric limits (edge case)
    """
    x, y, z = position[:3]
    # Quick probe: is position reachable with any orientation? (position-only IK)
    pos_only_res, pos_only_mode = hub.solve_ik_with_mode([x, y, z], seed=current_angles)
    if pos_only_res is None:
        # Position itself unreachable
        # Estimate distance from workspace (rough: from base origin)
        r_from_base = (x*x + y*y + z*z) ** 0.5
        max_reach = 380  # approximate (matches old constants)
        return {
            "code": "OUT_OF_REACH",
            "message": f"位置 ({x:.0f},{y:.0f},{z:.0f}) はアーム到達範囲外",
            "diagnostics": {
                "distance_from_base_mm": round(r_from_base, 1),
                "approx_max_reach_mm": max_reach,
                "position_only_tried": True,
                "position_only_failed": True,
            },
            "retry_hints": [
                {"action": "move_closer", "patch": None,
                 "rationale": f"R={r_from_base:.0f}mm がアーム reach (~{max_reach}mm) を超えている。位置を base 寄りに"},
            ],
        }
    # Position reachable → orientation was the issue
    if requested_rxyz is not None:
        return {
            "code": "ORIENTATION_INFEASIBLE",
            "message": f"位置 ({x:.0f},{y:.0f},{z:.0f}) は到達可だが要求姿勢 rxyz=({requested_rxyz[0]:.0f},{requested_rxyz[1]:.0f},{requested_rxyz[2]:.0f}) は不可",
            "diagnostics": {
                "position_reachable": True,
                "position_only_angles": [round(a, 1) for a in pos_only_res],
                "requested_orientation_deg": list(requested_rxyz),
            },
            "retry_hints": [
                {"action": "use_preserve_pose", "patch": {"pose": {"kind": "preserve"}},
                 "rationale": "姿勢を捨てて位置だけ到達（手首は任意）"},
                {"action": "use_extend_toward", "patch": {"pose": {"kind": "extend_toward", "target": [x, y, z]}},
                 "rationale": "指差し姿勢に変更"},
                {"action": "use_align_top", "patch": {"pose": {"kind": "align_tool", "approach": "+z"}},
                 "rationale": "上から接近に変更"},
            ],
        }
    # Position reachable, no orientation requested, yet failed — shouldn't happen with retries
    return {
        "code": "SOLVER_NONCONVERGENT",
        "message": "IK ソルバが収束せず（位置のみ要求でも失敗）",
        "diagnostics": {"position_reachable": True, "ik_mode": "failed_after_retries"},
        "retry_hints": [
            {"action": "retry_with_perturbed_seed", "patch": None,
             "rationale": "現在角度を少し動かしてから再試行"},
        ],
    }


def _coerce_angles(raw) -> list[float]:
    if not isinstance(raw, list) or len(raw) != 6:
        raise ValueError("angles must be a length-6 list")
    out = []
    for i, v in enumerate(raw):
        try:
            f = float(v)
        except (TypeError, ValueError):
            raise ValueError(f"angles[{i}] not numeric: {v!r}") from None
        if not math.isfinite(f):
            raise ValueError(f"angles[{i}] not finite: {f}") from None
        out.append(f)
    return out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass  # silence access log

    def log_error(self, fmt, *args):
        print("[http-err] " + (fmt % args), file=sys.stderr)

    # --- helpers ---
    def _json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n).decode("utf-8") or "{}") if n else {}

    def _auth_ok(self) -> bool:
        if AUTH_TOKEN is None:
            return True
        return self.headers.get("X-Auth-Token") == AUTH_TOKEN

    # --- routing ---
    def do_GET(self):
        try:
            path = urlparse(self.path).path
            if path in ("/", "/index", "/index.html"):
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body); return
            if path == "/favicon.ico":
                self.send_response(204); self.end_headers(); return
            if path == "/kinematics":
                # Single source of truth for FK/safety; UI fetches this at boot.
                self._json(200, {
                    "dh": DH,
                    "joint_limits": JOINT_LIMITS,
                    "tool_length": TOOL_LENGTH,
                    "floor_z": FLOOR_Z, "link_radius": LINK_RADIUS,
                    "table_margin": TABLE_MARGIN, "fk_tool_slop": FK_TOOL_SLOP,
                    "home_angles": HOME_ANGLES,
                    "max_speed": MAX_SPEED_RUNTIME,
                    # grasp visualisation constants (single source for UI)
                    "grasp_approach_offset_mm": GRASP_APPROACH_OFFSET_MM,
                    "grasp_lift_offset_mm": GRASP_LIFT_OFFSET_MM,
                    "grasp_approach_speed_default": GRASP_APPROACH_SPEED_DEFAULT,
                    "grasp_approach_speed_max": GRASP_APPROACH_SPEED_MAX,
                    "grasp_offset_min_mm": GRASP_OFFSET_MIN_MM,
                    "grasp_offset_max_mm": GRASP_OFFSET_MAX_MM,
                    "target_radius_min_mm": TARGET_RADIUS_MIN_MM,
                    "target_radius_max_mm": TARGET_RADIUS_MAX_MM,
                    "target_radius_default_mm": TARGET_RADIUS_DEFAULT_MM,
                    "gripper_tip_clearance_mm": GRIPPER_TIP_CLEARANCE_MM,
                    "gripper_present": False,  # Phase 1+2: no gripper hardware probe yet
                }); return
            if path == "/angles":
                self._json(200, {"angles": HUB.angles(), "offline": HUB.offline}); return
            if path == "/coords":
                a = HUB.angles()
                self._json(200, {"coords": list(end_effector(a)) if a else None, "angles": a}); return
            if path == "/power":
                self._json(200, {"ok": HUB.power_ok()}); return
            if path == "/currents":
                import arm.constants as _c
                cs = HUB.get_currents()
                self._json(200, {
                    "currents": cs,
                    "monitor_enabled": HUB.monitor_enabled,
                    "threshold_mA": _c.CURRENT_THRESHOLD_MA,  # dynamic — may be overridden by safe-mode
                    "poll_hz": CURRENT_POLL_HZ,
                    "sustained_polls": SUSTAINED_OVER_COUNT,
                }); return
            if path == "/fk":
                q = parse_qs(urlparse(self.path).query)
                a = [float(x) for x in q.get("angles", [""])[0].split(",") if x]
                if len(a) != 6:
                    self._json(400, {"error": "angles must be 6 comma-separated values"}); return
                self._json(200, {"joints": joint_positions(a), "tip": list(end_effector(a))}); return
            if path == "/frame.jpg":
                buf = HUB.frame_jpeg()
                if not buf:
                    self.send_response(503); self.end_headers(); return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(buf)))
                self.end_headers()
                self.wfile.write(buf); return
            self.send_response(404); self.end_headers()
        except Exception as e:
            log.exception("GET %s failed", self.path)
            self._json(500, {"error": str(e)})

    def do_POST(self):
        try:
            path = urlparse(self.path).path
            # write endpoints require auth (if configured)
            if path in ("/move", "/home", "/release", "/abort") and not self._auth_ok():
                self._json(401, {"error": "auth required"}); return

            body = self._read_body()

            if path == "/check":
                try:
                    angles = _coerce_angles(body.get("angles"))
                except ValueError as e:
                    self._json(400, {"error": str(e)}); return
                ok, msg, bad = check_angles(angles)
                self._json(200, {"ok": ok, "msg": msg, "badJoints": bad}); return

            if path == "/solve_ik":
                """Body: {x, y, z, pose?: {kind:..., ...}, rx?, ry?, rz?}

                If `pose` is given, it's resolved to (rx,ry,rz) via pose_resolver.
                Else if rx/ry/rz given, used directly.
                Else: position-only (default kind=preserve).
                """
                cur = HUB.angles()
                if cur is None: raise RuntimeError("current angles unavailable")
                tip = end_effector(cur)
                try:
                    x = float(body.get("x", tip[0]))
                    y = float(body.get("y", tip[1]))
                    z = float(body.get("z", tip[2]))
                except (TypeError, ValueError):
                    self._json(400, {"error": "invalid x/y/z"}); return
                if not all(math.isfinite(v) for v in (x, y, z)):
                    self._json(400, {"error": "non-finite x/y/z"}); return
                # Resolve pose to rxyz
                pose_spec = body.get("pose")
                try:
                    if pose_spec is not None:
                        rxyz = resolve_pose(pose_spec, (x, y, z), cur)
                    elif any(k in body for k in ("rx", "ry", "rz")):
                        rxyz = (float(body.get("rx", 0)), float(body.get("ry", 0)), float(body.get("rz", 0)))
                    else:
                        rxyz = None  # position-only
                except ValueError as e:
                    self._json(400, {"error": f"pose: {e}"}); return
                if rxyz is not None and not all(math.isfinite(v) for v in rxyz):
                    self._json(400, {"error": "resolved orientation non-finite"}); return
                if rxyz is None:
                    target = [x, y, z]
                else:
                    target = [x, y, z, rxyz[0], rxyz[1], rxyz[2]]
                res, mode = HUB.solve_ik_with_mode(target)
                if res is None:
                    # Diagnose: was the failure about orientation, or true unreachability?
                    diag = _diagnose_ik_failure(HUB, (x, y, z), rxyz, cur)
                    self._json(200, {
                        "ok": False, "angles": None,
                        "resolvedOrientation": rxyz, "ikMode": mode,
                        "error": diag,                                          # structured
                        "msg": diag["message"],                                 # legacy field
                    }); return
                ok, msg, bad = check_angles(res)
                # Compute achieved orientation via FK
                from arm.pose_resolver import _matrix_to_rpy  # noqa
                from arm.kinematics import link_frames
                T = link_frames(res)[-1]
                achieved_R = [[T[0][0],T[0][1],T[0][2]],[T[1][0],T[1][1],T[1][2]],[T[2][0],T[2][1],T[2][2]]]
                achieved_rxyz = _matrix_to_rpy(achieved_R)
                resp = {"ok": ok, "angles": res if ok else None, "msg": msg,
                        "badJoints": bad, "resolvedOrientation": rxyz,
                        "achievedOrientation": list(achieved_rxyz), "ikMode": mode}
                if not ok:
                    # IK found a pose but safety check rejected it
                    resp["error"] = {
                        "code": "SAFETY_VIOLATION",
                        "message": f"IK 解が安全チェック NG: {msg}",
                        "badJoints": bad,
                        "retry_hints": [
                            {"action": "use_preserve_pose", "patch": {"pose": {"kind": "preserve"}},
                             "rationale": "現在姿勢維持で安全範囲内の解を探す"},
                            {"action": "use_align_top", "patch": {"pose": {"kind": "align_tool", "approach": "+z"}},
                             "rationale": "上から接近に切替"},
                        ],
                    }
                self._json(200, resp); return

            if path == "/move":
                try: target = _coerce_angles(body.get("angles"))
                except ValueError as e: self._json(400, {"error": str(e)}); return
                if not HUB.motion_lock.acquire(blocking=False):
                    self._json(409, {"error": "motion in progress"}); return
                try:
                    err, cur, speed = _preflight(body)
                    if err: self._json(err.pop("code", 400), err); return
                    waypoints, ok, msg, bad = plan_and_validate(cur, target)
                    if not ok:
                        self._json(422, {"error": msg, "badJoints": bad, "nWaypoints": len(waypoints)}); return
                    code, resp = _execute_joint_waypoints(waypoints, speed)
                    resp["nWaypoints"] = len(waypoints)
                    self._json(code, resp); return
                finally:
                    HUB.motion_lock.release()

            if path == "/move_cartesian":
                cart_mode = body.get("mode", "lift")
                if cart_mode not in ("linear", "lift", "auto"):
                    self._json(400, {"error": "mode must be 'linear'|'lift'|'auto'"}); return
                if not HUB.motion_lock.acquire(blocking=False):
                    self._json(409, {"error": "motion in progress"}); return
                try:
                    err, cur_j, speed = _preflight(body)
                    if err: self._json(err.pop("code", 400), err); return
                    # Resolve the actual current cartesian pose for start_pose orientation.
                    # Prefer live firmware coords (correct tool rotation); fall back to FK if unavailable.
                    live = HUB.live_coords()
                    if live and all(math.isfinite(v) for v in live):
                        start_pose = tuple(live)
                    else:
                        tip = end_effector(cur_j)
                        # FK orientation is approximate — note in response so user can detect
                        start_pose = (tip[0], tip[1], tip[2], 0.0, 0.0, 0.0)
                    # optional cartesian-tip drift check
                    expected_tip = body.get("expected_tip")
                    if expected_tip is not None:
                        try:
                            etip = [float(v) for v in expected_tip]
                            if len(etip) >= 3:
                                tip_drift = max(abs(start_pose[i] - etip[i]) for i in range(3))
                                if tip_drift > 10.0:
                                    self._json(409, {"error": f"tip drift {tip_drift:.1f}mm > 10mm"}); return
                        except Exception:
                            pass
                    try:
                        target = (
                            float(body.get("x", start_pose[0])), float(body.get("y", start_pose[1])),
                            float(body.get("z", start_pose[2])),
                            float(body.get("rx", start_pose[3])),  # default: keep current orientation
                            float(body.get("ry", start_pose[4])),
                            float(body.get("rz", start_pose[5])),
                        )
                    except (TypeError, ValueError):
                        self._json(400, {"error": "invalid coords"}); return
                    if not all(math.isfinite(v) for v in target):
                        self._json(400, {"error": "non-finite coords"}); return
                    if cart_mode == "linear":
                        cart_wps = cart_linear(start_pose, target)
                    elif cart_mode == "auto":
                        from arm.path_cartesian import auto as cart_auto
                        cart_wps = cart_auto(start_pose, target)
                    else:
                        cart_wps = lift_translate_lower(start_pose, target)
                    joint_wps, ok, msg, bad = plan_ik_path(cur_j, start_pose, cart_wps, ik=HUB.solve_ik)
                    if not ok:
                        self._json(422, {"error": f"cartesian path NG: {msg}", "badJoints": bad,
                                         "nCartWp": len(cart_wps), "nJointWp": len(joint_wps)}); return
                    code, resp = _execute_joint_waypoints(joint_wps, speed)
                    resp["nCartWp"] = len(cart_wps); resp["nJointWp"] = len(joint_wps)
                    self._json(code, resp); return
                finally:
                    HUB.motion_lock.release()

            if path == "/grasp_sequence":
                """Object-based grasp motion (Phase 1+2: no gripper actuation).

                Body: {x, y, z, radius, approach_offset?, lift_offset?, speed?, expected_current?}
                Builds 3 cartesian segments via the existing planner:
                  current → pre-grasp (z + approach_offset)
                          → grasp     (z + radius + GRIPPER_TIP_CLEARANCE)  ← tip stops ABOVE object
                          → lift      (z + lift_offset)
                Until gripper actuation is wired (Phase 3), grasp descent stops just above the
                object surface, not at the center.
                """
                try:
                    x = float(body["x"]); y = float(body["y"]); z = float(body["z"])
                    radius = float(body.get("radius", TARGET_RADIUS_DEFAULT_MM))
                except (KeyError, TypeError, ValueError):
                    self._json(400, {"error": "x,y,z required (numeric)"}); return
                if not (TARGET_RADIUS_MIN_MM <= radius <= TARGET_RADIUS_MAX_MM):
                    self._json(400, {"error": f"radius {radius} out of [{TARGET_RADIUS_MIN_MM}, {TARGET_RADIUS_MAX_MM}]"}); return
                approach_off = float(body.get("approach_offset", GRASP_APPROACH_OFFSET_MM))
                lift_off = float(body.get("lift_offset", GRASP_LIFT_OFFSET_MM))
                if not (GRASP_OFFSET_MIN_MM <= approach_off <= GRASP_OFFSET_MAX_MM):
                    self._json(400, {"error": f"approach_offset {approach_off} out of [{GRASP_OFFSET_MIN_MM}, {GRASP_OFFSET_MAX_MM}]"}); return
                if not (GRASP_OFFSET_MIN_MM <= lift_off <= GRASP_OFFSET_MAX_MM):
                    self._json(400, {"error": f"lift_offset {lift_off} out of [{GRASP_OFFSET_MIN_MM}, {GRASP_OFFSET_MAX_MM}]"}); return
                try: speed = int(body.get("speed", GRASP_APPROACH_SPEED_DEFAULT))
                except (TypeError, ValueError):
                    self._json(400, {"error": "speed not int"}); return
                if not 1 <= speed <= min(MAX_SPEED_RUNTIME, GRASP_APPROACH_SPEED_MAX):
                    self._json(400, {"error": f"grasp speed must be 1..{min(MAX_SPEED_RUNTIME, GRASP_APPROACH_SPEED_MAX)}"}); return
                # Phase 1+2: top-down only, no gripper → stop above object surface
                grasp_z = z + radius + GRIPPER_TIP_CLEARANCE_MM
                if not HUB.motion_lock.acquire(blocking=False):
                    self._json(409, {"error": "motion in progress"}); return
                try:
                    err, cur_j, _ = _preflight(body)
                    if err: self._json(err.pop("code", 400), err); return
                    live = HUB.live_coords()
                    if live and all(math.isfinite(v) for v in live):
                        start_pose = tuple(live)
                    else:
                        tip = end_effector(cur_j)
                        start_pose = (tip[0], tip[1], tip[2], 0.0, 0.0, 0.0)
                    pre_grasp = (x, y, z + approach_off, *start_pose[3:])
                    grasp     = (x, y, grasp_z,         *start_pose[3:])
                    lift      = (x, y, z + lift_off,    *start_pose[3:])
                    stages = [("pre-grasp", pre_grasp), ("approach", grasp), ("lift", lift)]
                    log.info("grasp_sequence start: target=(%.0f,%.0f,%.0f) r=%.0f stages_z=%s",
                             x, y, z, radius, [round(s[1][2]) for s in stages])

                    HUB.abort_flag.clear()
                    HUB.start_monitor()
                    t0 = time.time()
                    all_joint_wps = []
                    try:
                        prev_pose = start_pose
                        seed_joints = cur_j
                        for stage_idx, (stage_name, target) in enumerate(stages):
                            # Inter-stage drift recheck: actual position may differ from commanded prev_pose
                            # (e.g., joint readback noise). Re-read seed_joints from real arm before planning.
                            if stage_idx > 0:
                                live_now = HUB.angles()
                                if live_now is None:
                                    self._json(503, {"error": "stage 間で angles 読取失敗", "stage": stage_name}); return
                                # Use actual current as seed
                                seed_joints = live_now
                                # If the actual cartesian position drifted significantly from prev_pose, re-derive
                                live_cart = HUB.live_coords()
                                if live_cart and all(math.isfinite(v) for v in live_cart):
                                    dxyz = sum((live_cart[i] - prev_pose[i])**2 for i in range(3))**0.5
                                    if dxyz > 15.0:  # 15mm drift between stages → abort to avoid surprise
                                        self._json(409, {
                                            "error": f"stage 間ドリフト {dxyz:.1f}mm > 15mm",
                                            "stage": stage_name,
                                        }); return
                                    prev_pose = tuple(live_cart)
                            cart_wps = cart_linear(prev_pose, target)
                            joint_wps, ok, msg, bad = plan_ik_path(seed_joints, prev_pose, cart_wps, ik=HUB.solve_ik)
                            if not ok:
                                self._json(422, {
                                    "error": f"{stage_name} NG: {msg}",
                                    "badJoints": bad, "stage": stage_name,
                                    "completedStages": [s for s, _ in stages[:stage_idx]],
                                }); return
                            log.info("grasp_sequence stage=%s waypoints=%d", stage_name, len(joint_wps))
                            for idx, wp in enumerate(joint_wps):
                                if HUB.abort_flag.is_set():
                                    triggered = HUB._monitor and HUB._monitor.triggered
                                    tag = "over-current" if triggered else "user"
                                    self._json(499, {
                                        "error": f"aborted ({tag}) during {stage_name}",
                                        "stage": stage_name, "lastIndex": idx,
                                        "currents": HUB._monitor.last_currents if HUB._monitor else None,
                                        "peakJoint": HUB._monitor.peak_joint if HUB._monitor else None,
                                        "peakValue": HUB._monitor.peak_value if HUB._monitor else None,
                                    }); return
                                reached, actual = HUB.send_angles_and_wait(wp, speed)
                                if not reached:
                                    self._json(503, {
                                        "error": f"{stage_name} waypoint {idx+1}/{len(joint_wps)} 到達タイムアウト",
                                        "stage": stage_name, "lastActual": actual,
                                    }); return
                            all_joint_wps.extend(joint_wps)
                            seed_joints = joint_wps[-1]
                            prev_pose = target
                        peak = HUB._monitor.peak_currents if HUB._monitor else None
                        self._json(200, {
                            "angles": HUB.angles(),
                            "stages": [s for s, _ in stages],
                            "nJointWp": len(all_joint_wps),
                            "elapsed": round(time.time() - t0, 2),
                            "peakCurrents": peak,
                            "graspZ": grasp_z,  # tip actually stopped at this height
                        }); return
                    finally:
                        HUB.stop_monitor()
                finally:
                    HUB.motion_lock.release()

            if path == "/home":
                if not HUB.motion_lock.acquire(blocking=False):
                    self._json(409, {"error": "motion in progress"}); return
                try:
                    if not HUB.power_ok():
                        self._json(503, {"error": "サーボ未通電"}); return
                    # auto re-enable monitor on /home (defensive: don't run /home unmonitored)
                    if not HUB.monitor_enabled:
                        log.warning("/home: monitor was disabled; auto re-enabling")
                        HUB.monitor_enabled = True
                    t0 = time.time()
                    HUB.home_blocking()
                    self._json(200, {"angles": HUB.angles(), "elapsed": round(time.time() - t0, 2),
                                     "monitorReenabled": True}); return
                except Exception as e:
                    self._json(500, {"error": str(e)}); return
                finally:
                    HUB.motion_lock.release()

            if path == "/abort":
                HUB.abort_flag.set()
                self._json(200, {"ok": True}); return

            if path == "/monitor":
                enabled = bool(body.get("enabled", True))
                HUB.monitor_enabled = enabled
                if not enabled:
                    log.warning("過電流監視が無効化されました")
                self._json(200, {"ok": True, "enabled": enabled,
                                 "warning": None if enabled else "過電流監視 OFF — 衝突しても自動停止しません"}); return

            if path == "/release":
                # set abort first so any in-flight waypoint loop bails immediately
                HUB.abort_flag.set()
                # wait for motion to finish (or skip if user explicitly forces)
                force = bool(body.get("force"))
                acquired = HUB.motion_lock.acquire(timeout=0.5)
                if not acquired and not force:
                    self._json(409, {"error": "motion in progress — set {\"force\":true} to release anyway"}); return
                try:
                    HUB.release()
                    self._json(200, {"ok": True, "warning": "脱力済 — 手で支えないと落下します"}); return
                finally:
                    if acquired: HUB.motion_lock.release()

            self.send_response(404); self.end_headers()
        except Exception as e:
            log.exception("POST %s failed", self.path)
            self._json(400, {"error": str(e)})


def lan_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close(); return ip
    except Exception:
        return "127.0.0.1"


def main():
    global HUB, INDEX_HTML, AUTH_TOKEN, SHUTTING_DOWN, MAX_SPEED_RUNTIME

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    ap = argparse.ArgumentParser()
    ap.add_argument("--offline", action="store_true", help="virtual arm (no hardware)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="127.0.0.1", help="default 127.0.0.1; use 0.0.0.0 for LAN (requires --token)")
    ap.add_argument("--token", default=None, help="X-Auth-Token required for write endpoints when bound to non-loopback")
    ap.add_argument("--cam", type=int, default=DEFAULT_CAM_INDEX)
    ap.add_argument("--max-speed", type=int, default=MAX_SPEED)
    args = ap.parse_args()

    if args.bind != "127.0.0.1" and not args.token:
        sys.exit("FATAL: --bind non-loopback requires --token <secret> (refusing unauthenticated LAN exposure)")
    AUTH_TOKEN = args.token

    if not 1 <= args.max_speed <= 80:
        sys.exit(f"FATAL: --max-speed {args.max_speed} out of [1, 80]")
    MAX_SPEED_RUNTIME = args.max_speed

    # sanity: HOME pose must pass safety check
    ok, msg, _bad = check_angles(HOME_ANGLES)
    if not ok:
        sys.exit(f"FATAL: HOME_ANGLES fails safety check: {msg}")

    # calibration check — if no marker file, use conservative threshold
    marker = ROOT / CALIBRATION_MARKER
    if not marker.exists():
        import arm.constants as _c
        log.warning("校正マーカー %s が無いため、保守的閾値 %dmA に切替（既定 %dmA）", CALIBRATION_MARKER, SAFE_MODE_CURRENT_MA, _c.CURRENT_THRESHOLD_MA)
        _c.CURRENT_THRESHOLD_MA = SAFE_MODE_CURRENT_MA

    HUB = VirtualHub() if args.offline else Hub(cam_index=args.cam)
    INDEX_HTML = (ROOT / "scripts" / "ui.html").read_text(encoding="utf-8")
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    mode = "OFFLINE" if args.offline else "live"
    print(f"\n  [{mode}] http://localhost:{args.port}/")
    if args.bind == "0.0.0.0":
        print(f"  [{mode}] http://{lan_ip()}:{args.port}/  (LAN, auth=token)\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutdown: waiting for in-flight motion...")
        SHUTTING_DOWN = True
        if HUB.motion_lock.acquire(timeout=6.0):
            try:
                if not args.offline:
                    try: HUB.home_blocking(speed=25)
                    except Exception as e: print(f"home failed: {e}")
            finally:
                HUB.motion_lock.release()
        else:
            print("motion did not finish in time; skipping home")
        srv.shutdown()
        HUB.shutdown()


if __name__ == "__main__":
    main()
