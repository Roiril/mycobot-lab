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
  POST /capture_calib_frame {cam?} → save current frame into data/calib_images/<cam_id>/
"""
from __future__ import annotations
import sys, os, json, math, time, socket, pathlib, argparse, logging, datetime
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Optional
from urllib.parse import urlparse, parse_qs

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.kinematics import joint_positions, end_effector, JOINT_LIMITS, DH, URDF_LINKS  # noqa: E402
from arm.safety import check_angles  # noqa: E402
from arm.planner import plan_and_validate  # noqa: E402
from arm.path_cartesian import linear as cart_linear, lift_translate_lower  # noqa: E402
from arm.ik_path import plan_ik_path  # noqa: E402
from arm.pose_resolver import resolve_pose  # noqa: E402
from arm.hub import Hub, VirtualHub, HubBase  # noqa: E402
from arm.vision_hub import VisionHub, VirtualVisionHub  # noqa: E402
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
    WORKSPACE_REACH_MAX_MM, WORKSPACE_Z_MAX_MM,
)

log = logging.getLogger("mycobot.server")

HUB: HubBase | None = None
VISION: VisionHub | None = None
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


def _path_is_monotonic(wps) -> bool:
    """True iff every joint moves in one direction across all waypoints. If so,
    firmware can interpolate between start and final without traversing outside
    the safety-validated tube (since the tube IS the monotonic line)."""
    if len(wps) < 2: return True
    dirs = [0]*6
    prev = wps[0]
    for w in wps[1:]:
        for j in range(6):
            d = w[j] - prev[j]
            if abs(d) < 1e-6: continue
            sign = 1 if d > 0 else -1
            if dirs[j] == 0: dirs[j] = sign
            elif dirs[j] != sign: return False
        prev = w
    return True


def _execute_joint_waypoints(joint_wps, speed: int) -> tuple[int, dict]:
    """Drive waypoints with monitor + abort handling. Returns (http_code, body).

    SMOOTH_SINGLE_SHOT mode: if all waypoints are safety-validated AND the path
    is joint-space monotonic, send only the final target to firmware (single
    send_angles) and poll until reached. The firmware interpolates smoothly
    without the deceleration/pause between PATH_STEP_DEG chunks. Falls back to
    chunked execution if path changes direction in joint space.
    """
    import arm.constants as _c
    HUB.abort_flag.clear()
    HUB.last_stall_info = None
    HUB.start_monitor()
    t0 = time.time()
    try:
        # Single-shot path if eligible
        if _c.SMOOTH_SINGLE_SHOT and len(joint_wps) > 1 and _path_is_monotonic(joint_wps):
            final = joint_wps[-1]
            if HUB.abort_flag.is_set():
                return 499, {"error": "aborted (user)", "lastIndex": 0}
            reached, actual = HUB.send_angles_and_wait(final, speed)
            if not reached:
                if HUB.abort_flag.is_set():
                    stall = HUB.last_stall_info
                    triggered = HUB._monitor and HUB._monitor.triggered
                    tag = "stall" if stall else ("over-current" if triggered else "user")
                    body = {"error": f"aborted ({tag})", "lastActual": actual, "stall": stall}
                    if HUB._monitor:
                        body["currents"] = HUB._monitor.last_currents
                        body["peakJoint"] = HUB._monitor.peak_joint
                        body["peakValue"] = HUB._monitor.peak_value
                    return 499, body
                return 503, {"error": "single-shot 到達タイムアウト", "lastActual": actual,
                             "stall": HUB.last_stall_info}
            peak = HUB._monitor.peak_currents if HUB._monitor else None
            return 200, {"angles": HUB.angles(), "elapsed": round(time.time() - t0, 2),
                         "peakCurrents": peak, "monitorEnabled": HUB.monitor_enabled,
                         "smoothMode": "single_shot", "nWaypoints": len(joint_wps)}
        # Chunked path (non-monotonic or single-shot disabled)
        for idx, wp in enumerate(joint_wps):
            if HUB.abort_flag.is_set():
                stall = HUB.last_stall_info
                triggered = HUB._monitor and HUB._monitor.triggered
                if stall:
                    tag = "stall"
                elif triggered:
                    tag = "over-current"
                else:
                    tag = "user"
                body = {"error": f"aborted ({tag})", "lastIndex": idx, "stall": stall}
                if HUB._monitor:
                    body["currents"] = HUB._monitor.last_currents
                    body["peakJoint"] = HUB._monitor.peak_joint
                    body["peakValue"] = HUB._monitor.peak_value
                return 499, body
            reached, actual = HUB.send_angles_and_wait(wp, speed)
            if not reached:
                if HUB.abort_flag.is_set():
                    stall = HUB.last_stall_info
                    return 499, {"error": f"aborted ({'stall' if stall else 'user'})",
                                 "lastIndex": idx, "lastActual": actual, "stall": stall}
                return 503, {"error": f"waypoint {idx+1}/{len(joint_wps)} 到達タイムアウト",
                             "lastActual": actual, "stall": HUB.last_stall_info}
        peak = HUB._monitor.peak_currents if HUB._monitor else None
        return 200, {"angles": HUB.angles(), "elapsed": round(time.time() - t0, 2),
                     "peakCurrents": peak, "monitorEnabled": HUB.monitor_enabled,
                     "smoothMode": "chunked", "nWaypoints": len(joint_wps)}
    finally:
        HUB.stop_monitor()


_APPROX_MAX_REACH = 380.0  # arm physical reach (mm) — matches ik_numeric._MAX_REACH_MM


def _camera_dir_hint(j1_deg: float) -> str:
    """Human-readable description of which way the camera looks at the observe pose."""
    # At J1=0 camera looks -Y. Each +90° in J1 rotates camera +90° CCW.
    # j1=0  → -Y (背面側)
    # j1=90 → +X (右側)
    # j1=180→ +Y (前面側)
    # j1=-90→ -X (左側)
    th = (j1_deg + 0) % 360
    dirs = {0: "-Y (アーム背面)", 90: "+X (アーム右)", 180: "+Y (アーム前面)", 270: "-X (アーム左)"}
    if int(round(th)) in dirs:
        return dirs[int(round(th))]
    return f"base angle ≈ {th:.0f}° (atan2 by J1)"


def _append_observation_log(direction, j1, angles, flange_xyz, query, perceive_result) -> pathlib.Path:
    """Append a human-readable observation entry to data/observation_log.md.

    Returns the log path.
    """
    log = ROOT / "data" / "observation_log.md"
    log.parent.mkdir(exist_ok=True)
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    objects = perceive_result.get("objects") or []
    placeholder = any((o.get("uncalibrated") for o in objects))
    lines = []
    lines.append(f"## {ts}  direction={direction!r} J1={j1:.1f}°")
    lines.append(f"- flange (mm): ({flange_xyz[0]:.0f}, {flange_xyz[1]:.0f}, {flange_xyz[2]:.0f})")
    lines.append(f"- angles (deg): [{', '.join(f'{a:.1f}' for a in angles)}]")
    lines.append(f"- query: {query}")
    if placeholder:
        lines.append(f"- ⚠ hand-eye calibration が placeholder のため世界座標は概算値")
    if not perceive_result.get("ok", True):
        err = perceive_result.get("error") or {}
        lines.append(f"- ❌ perceive 失敗: [{err.get('code','?')}] {err.get('message','')}")
    elif not objects:
        lines.append(f"- (検出物なし)")
    else:
        lines.append(f"- 検出 {len(objects)} 件:")
        for obj in objects:
            xyz = obj.get("world_xyz_mm") or [None]*3
            label = obj.get("label", "?")
            conf = obj.get("confidence")
            radius = obj.get("radius_mm")
            cam = obj.get("source_cam", "?")
            sxyz = "(" + ", ".join(f"{v:.0f}" if v is not None else "?" for v in xyz) + ")"
            extra = []
            if conf is not None: extra.append(f"conf={conf:.2f}")
            if radius is not None: extra.append(f"r={radius:.0f}mm")
            extra.append(f"cam={cam}")
            if obj.get("out_of_workspace"): extra.append("⚠out_of_workspace")
            lines.append(f"  - {label} @ {sxyz} mm  [{' / '.join(extra)}]")
    lines.append("")
    with log.open("a", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return log


def _diagnose_ik_failure(hub, position, requested_rxyz, current_angles, *, skip_repeat_solve=False) -> dict:
    """When IK fails entirely, diagnose why so the caller (UI or LLM) gets a structured reason.

    Returns {code, message, diagnostics, retry_hints[]}.
    Codes:
      OUT_OF_REACH           — position itself unreachable (even with arbitrary orientation)
      ORIENTATION_INFEASIBLE — position reachable, but not with the requested orientation
      SOLVER_NONCONVERGENT   — both attempts hit numeric limits (edge case)

    skip_repeat_solve=True (preferred when called immediately after a failed
    solve_with_retries): infer reachability from the reach radius alone instead
    of running another full position-only IK (~0.8s saved per failed call).
    """
    x, y, z = position[:3]
    # Cheap geometric reach check first (catches gross out-of-reach instantly)
    r = (x*x + y*y + z*z) ** 0.5
    if r > _APPROX_MAX_REACH + 30:  # 30mm grace beyond nominal reach
        return {
            "code": "OUT_OF_REACH",
            "message": f"位置 ({x:.0f},{y:.0f},{z:.0f}) はアーム到達範囲外",
            "diagnostics": {"distance_from_base_mm": round(r, 1),
                            "approx_max_reach_mm": _APPROX_MAX_REACH},
            "retry_hints": [{"action": "move_closer", "patch": None,
                             "rationale": f"R={r:.0f}mm がアーム reach (~{_APPROX_MAX_REACH:.0f}mm) を超えている"}],
        }
    if skip_repeat_solve:
        # If the caller already exhausted position-only attempts in solve_with_retries,
        # the cheapest informative answer without another 0.8s probe is:
        #   - if no orientation was requested → solver couldn't even find position
        #   - if orientation was requested → likely the orientation that's infeasible
        if requested_rxyz is None:
            return {
                "code": "SOLVER_NONCONVERGENT",
                "message": "IK ソルバが収束せず（位置のみ要求でも失敗）",
                "diagnostics": {"position_reachable": "unknown", "distance_from_base_mm": round(r, 1)},
                "retry_hints": [{"action": "retry_with_perturbed_seed", "patch": None,
                                 "rationale": "現在角度を少し動かしてから再試行"}],
            }
        return {
            "code": "ORIENTATION_INFEASIBLE",
            "message": f"位置 ({x:.0f},{y:.0f},{z:.0f}) は到達可と思われるが要求姿勢 rxyz=({requested_rxyz[0]:.0f},{requested_rxyz[1]:.0f},{requested_rxyz[2]:.0f}) は不可",
            "diagnostics": {"requested_orientation_deg": list(requested_rxyz),
                            "distance_from_base_mm": round(r, 1)},
            "retry_hints": [
                {"action": "use_preserve_pose", "patch": {"pose": {"kind": "preserve"}},
                 "rationale": "姿勢を捨てて位置だけ到達（手首は任意）"},
                {"action": "use_align_top", "patch": {"pose": {"kind": "align_tool", "approach": "+z"}},
                 "rationale": "上から接近に変更"},
            ],
        }
    # legacy path (kept for any caller that still wants the expensive probe)
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


def _bad_request(message: str, diagnostics: Optional[dict] = None, retry_hints: Optional[list] = None) -> dict:
    """Structured BAD_REQUEST envelope used by /perceive input validation.

    Matches the {ok:false, error:{code, message, diagnostics, retry_hints}} shape
    that /solve_ik and vision_hub.perceive() return for non-input errors.
    """
    return {
        "ok": False,
        "error": {
            "code": "BAD_REQUEST",
            "message": message,
            "diagnostics": diagnostics or {},
            "retry_hints": retry_hints or [],
        },
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
                    "dh": DH,  # deprecated; FK now URDF-based. Kept for any old clients.
                    "urdf_links": URDF_LINKS,
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
            if path == "/debug/fk_compare":
                # Diagnostic: compare our FK against firmware get_coords() at the current pose.
                # Useful when calibrating kinematics or attaching a new tool.
                a = HUB.angles()
                fk_tip = list(end_effector(a)) if a else None
                fk_joints = joint_positions(a) if a else None
                live = HUB.live_coords()
                delta = None
                if fk_tip and live and len(live) >= 3:
                    delta = [fk_tip[i] - live[i] for i in range(3)]
                self._json(200, {
                    "angles": a,
                    "fk_tip": fk_tip,
                    "fk_joints": fk_joints,
                    "firmware_coords": live,
                    "delta_fk_minus_fw_xyz_mm": delta,
                }); return
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
            if path == "/workspace_data":
                # Reach point cloud from probe runs. Returns both the dense
                # IK-only envelope (workspace.json) and the motion-confirmed
                # subset (workspace_motion.json) if present.
                out = {"envelope": None, "motion": None}
                for key, fname in [("envelope", "workspace.json"),
                                   ("motion", "workspace_motion.json")]:
                    f = ROOT / "data" / fname
                    if f.exists():
                        try: out[key] = json.loads(f.read_text(encoding="utf-8"))
                        except Exception: pass
                self._json(200, out); return

            if path == "/servo_diagnostics":
                # Batched per-servo state for UI live monitoring.
                # ?full=1 to include temps + voltages + per-servo enable (slow,
                # ~1-2s due to is_servo_enable being one round-trip each).
                # Default is fast: currents + all_servo_enable (~100-200ms).
                # power_ok is derived from all_enabled (avoids extra serial trips
                # and avoids contention with the background CurrentMonitor).
                import arm.constants as _c
                q = parse_qs(urlparse(self.path).query)
                full = q.get("full", ["0"])[0] in ("1", "true")
                diag = HUB.get_servo_diagnostics(full=full)
                diag["monitor_enabled"] = HUB.monitor_enabled
                diag["threshold_mA"] = _c.CURRENT_THRESHOLD_MA
                # Treat any-servo-released as "power not ok" for the UI badge;
                # explicit is_power_on() is too expensive to call at 2Hz.
                diag["power_ok"] = (diag.get("all_enabled") == 1)
                self._json(200, diag); return
            if path == "/fk":
                q = parse_qs(urlparse(self.path).query)
                a = [float(x) for x in q.get("angles", [""])[0].split(",") if x]
                if len(a) != 6:
                    self._json(400, {"error": "angles must be 6 comma-separated values"}); return
                self._json(200, {"joints": joint_positions(a), "tip": list(end_effector(a))}); return
            if path == "/cameras":
                if VISION is None:
                    self._json(200, {"cameras": []}); return
                self._json(200, {
                    "cameras": VISION.registry.list(),
                    "workspace": {
                        "table_z_mm": VISION.registry.table_z_mm,
                        "table_z_uncertainty_mm": VISION.registry.table_z_uncertainty_mm,
                    },
                }); return
            if path == "/frame.jpg":
                q = parse_qs(urlparse(self.path).query)
                cam_id_arg = q.get("cam", [None])[0]
                annotate_arg = q.get("annotate", [None])[0]
                buf = None
                annotation_status = "none"
                target_cam_id = cam_id_arg
                if target_cam_id is None and VISION is not None:
                    target_cam_id = VISION.registry.default_cam_id()
                if target_cam_id is not None and VISION is not None:
                    if target_cam_id not in VISION.registry.cameras:
                        self._json(404, {"error": f"unknown camera id: {target_cam_id}",
                                         "available": list(VISION.registry.cameras.keys())}); return
                    if annotate_arg == "last":
                        ann = VISION.get_last_annotated_jpeg(target_cam_id)
                        if ann:
                            buf = ann
                            annotation_status = "last"
                    if buf is None:
                        buf = VISION.registry.get_jpeg(target_cam_id)
                if not buf:
                    # legacy fallback to Hub's own frame_jpeg
                    buf = HUB.frame_jpeg()
                if not buf:
                    self.send_response(503); self.end_headers(); return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Annotation", annotation_status)
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

            if path == "/focus_servo":
                # Re-focus a single servo (1..6). Use when a specific joint is
                # stuck after firmware protection (push event).
                arm = getattr(HUB, "arm", None)
                if arm is None:
                    self._json(409, {"error": "offline hub"}); return
                j = int(body.get("joint", 0))
                if not 1 <= j <= 6:
                    self._json(400, {"error": "joint must be 1..6"}); return
                with HUB.io_lock:
                    try: r = arm.mc.focus_servo(j); self._json(200, {"ok": True, "result": r}); return
                    except Exception as e: self._json(500, {"error": str(e)}); return

            if path == "/clear_servo_errors":
                # Safe recovery: clear latched servo error flags + re-focus all servos.
                # Does NOT release power (arm holds position). Use when send_angles
                # is silently being ignored (typical symptom after firmware overload
                # or a user push event that triggered servo protection).
                arm = getattr(HUB, "arm", None)
                if arm is None:
                    self._json(409, {"error": "offline hub has no arm"}); return
                results = {}
                with HUB.io_lock:
                    try: arm.mc.clear_error_information(); results["clear_error_information"] = "ok"
                    except Exception as e: results["clear_error_information"] = f"err: {e}"
                    try: arm.mc.focus_all_servos(); results["focus_all_servos"] = "ok"
                    except Exception as e: results["focus_all_servos"] = f"err: {e}"
                self._json(200, {"ok": True, "results": results}); return

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
                    # Diagnose using cheap heuristics (don't repeat the failed solve)
                    diag = _diagnose_ik_failure(HUB, (x, y, z), rxyz, cur, skip_repeat_solve=True)
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
                    # Rescue mode: if the CURRENT pose itself is unsafe, the normal
                    # plan_and_validate would block all motion. Allow bypass only when
                    # explicitly requested AND the target itself passes safety AND the
                    # target keeps every joint at LEAST as high (or higher) than now —
                    # i.e. we only escape upward, never deeper.
                    rescue = bool(body.get("rescue", False))
                    if rescue:
                        ok_t, msg_t, bad_t = check_angles(target)
                        if not ok_t:
                            self._json(422, {"error": f"rescue mode: 目標自体が NG: {msg_t}", "badJoints": bad_t}); return
                        # Verify "monotonic safety improvement" by FK: every joint's z must
                        # not decrease from current to target.
                        from arm.kinematics import joint_positions
                        cur_pts = joint_positions(cur); tgt_pts = joint_positions(target)
                        for ji in range(1, 7):
                            if tgt_pts[ji][2] < cur_pts[ji][2] - 5.0:  # 5mm tolerance
                                self._json(422, {"error": f"rescue mode: J{ji} が下がる ({cur_pts[ji][2]:.0f}→{tgt_pts[ji][2]:.0f}mm) - 上向きの脱出のみ許可"}); return
                        log.warning("RESCUE MODE move: cur=%s target=%s", cur, target)
                        # Build path; do NOT run per-waypoint safety check
                        from arm.planner import plan_joint_path
                        waypoints = plan_joint_path(cur, target)
                        ok, msg, bad = True, "rescue", []
                    else:
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

            if path == "/observe":
                """Move to a horizontal-camera observation pose and SAVE THE FRAME
                to disk. No automated VLM detection — the captured JPEG is meant
                to be viewed by Shubie (Claude Code) directly via the Read tool,
                who then describes what's visible and appends to the log manually.

                Body: {
                  direction: 'front'|'left'|'right'|'back' | float (J1 deg),
                  use_vlm?: bool (default false; if true and API key present, also runs Claude vision)
                }

                Observe pose = HOME [0,0,-90,0,0,0] with J1 rotated:
                  back  → J1=  0  (-Y)   ← HOME default; camera looks at -Y
                  right → J1= 90  (+X)
                  front → J1=180  (+Y)
                  left  → J1=-90  (-X)
                Flange +Z (camera optical axis under placeholder hand-eye) is
                horizontal in the base XY plane at this pose.

                Returns {ok, observe:{...}, frame_path, vlm:?}.
                """
                direction = body.get("direction", "back")
                use_vlm = bool(body.get("use_vlm", False))
                # Resolve direction → J1
                if isinstance(direction, (int, float)):
                    j1 = float(direction)
                else:
                    dirmap = {"back": 0.0, "right": 90.0, "front": 180.0, "left": -90.0}
                    j1 = dirmap.get(str(direction).lower())
                    if j1 is None:
                        self._json(400, {"error": f"unknown direction '{direction}'; expected front|left|right|back or numeric J1 deg"}); return
                # Clamp J1 to limits
                lo, hi = JOINT_LIMITS[0]
                if not lo <= j1 <= hi:
                    self._json(400, {"error": f"J1={j1}° outside limits [{lo},{hi}]"}); return
                observe_angles = [j1, 0.0, -90.0, 0.0, 0.0, 0.0]
                # Move under motion_lock
                if not HUB.motion_lock.acquire(blocking=False):
                    self._json(409, {"error": "motion in progress"}); return
                try:
                    cur = HUB.angles()
                    if cur is None:
                        self._json(503, {"error": "angles unavailable"}); return
                    waypoints, ok, msg, bad = plan_and_validate(cur, observe_angles)
                    if not ok:
                        self._json(422, {"error": f"観測姿勢への経路 NG: {msg}", "badJoints": bad}); return
                    code, mov = _execute_joint_waypoints(waypoints, speed=20)
                    if code != 200:
                        self._json(code, {**mov, "stage": "move_to_observe"}); return
                finally:
                    HUB.motion_lock.release()
                # Capture frame
                angles = HUB.angles()
                if angles is None:
                    self._json(503, {"error": "angles lost after move"}); return
                time.sleep(0.4)  # let the camera autoexposure settle at new pose
                jpg = HUB.frame_jpeg()
                from arm.kinematics import joint_positions
                pts = joint_positions(angles)
                flange = pts[6]
                ts = time.strftime("%Y%m%d_%H%M%S")
                frame_dir = ROOT / "data" / "observe_frames"
                frame_dir.mkdir(parents=True, exist_ok=True)
                dir_tag = str(direction).replace(".", "p")
                frame_path = frame_dir / f"observe_{ts}_{dir_tag}.jpg"
                if jpg:
                    frame_path.write_bytes(jpg)
                else:
                    frame_path = None
                result = {
                    "ok": True,
                    "observe": {
                        "direction": direction, "j1_deg": j1,
                        "angles_deg": [round(a, 2) for a in angles],
                        "flange_mm": [round(x, 1) for x in flange],
                        "camera_height_mm": round(flange[2], 1),
                        # camera optical-axis direction in base XY plane (assuming
                        # placeholder hand-eye = identity rotation, so +Z of flange
                        # in base coords)
                        "camera_dir_hint": _camera_dir_hint(j1),
                    },
                    "frame_path": str(frame_path.relative_to(ROOT)) if frame_path else None,
                    "frame_full_path": str(frame_path) if frame_path else None,
                }
                if use_vlm and VISION is not None:
                    try:
                        vlm_result = VISION.perceive(
                            query=body.get("query", "周囲にある物体を全て検出"),
                            angles_deg=angles,
                            allow_uncalibrated=True, save_frame=False,
                        )
                        result["vlm"] = vlm_result
                    except Exception as e:
                        result["vlm_error"] = str(e)
                self._json(200, result); return

            if path == "/perceive":
                """Body: {query, cameras?, use_table_plane?, confidence_threshold?, consensus?, refine?}
                Read-only — no motion. Returns localized objects in base coords.
                """
                if VISION is None:
                    self._json(200, {
                        "ok": False,
                        "error": {
                            "code": "CALIBRATION_MISSING",
                            "message": "vision サブシステムが初期化されていない",
                            "terminal": True,
                            "diagnostics": {},
                            "retry_hints": [
                                {"action": "configure_camera", "patch": None,
                                 "rationale": "data/calibration.json を作成してサーバ再起動"},
                            ],
                        },
                    }); return
                query = str(body.get("query", "")).strip()
                if not query:
                    self._json(400, _bad_request("query (string) required",
                                                 {"missing": "query"})); return
                cameras = body.get("cameras")
                if cameras is not None and not isinstance(cameras, list):
                    self._json(400, _bad_request("cameras must be a list of camera ids",
                                                 {"got_type": type(cameras).__name__})); return
                # Validate camera ids (if provided) are known
                if isinstance(cameras, list) and VISION is not None:
                    known = set(VISION.registry.cameras.keys())
                    unknown = [c for c in cameras if c not in known]
                    if unknown:
                        self._json(400, _bad_request(
                            f"unknown camera id(s): {unknown}",
                            {"unknown": unknown, "available": sorted(known)},
                            [{"action": "use_known_camera",
                              "patch": {"cameras": sorted(known)},
                              "rationale": "登録済みカメラのみ指定可"}])); return
                use_plane = bool(body.get("use_table_plane", True))
                try:
                    conf_thresh = float(body.get("confidence_threshold", 0.5))
                except (TypeError, ValueError):
                    self._json(400, _bad_request("confidence_threshold not numeric")); return
                if not 0.0 <= conf_thresh <= 1.0:
                    self._json(400, _bad_request("confidence_threshold must be 0..1",
                                                 {"got": conf_thresh})); return
                consensus = bool(body.get("consensus", False))
                refine = bool(body.get("refine", False))
                allow_uncalibrated = bool(body.get("allow_uncalibrated", False))
                # save_frame: body wins, query param "save_frame=true" also supported
                save_frame_flag = bool(body.get("save_frame", False))
                try:
                    qs = parse_qs(urlparse(self.path).query)
                    if qs.get("save_frame", [None])[0] in ("1", "true", "True"):
                        save_frame_flag = True
                except Exception:
                    pass
                angles = HUB.angles()
                if angles is None:
                    self._json(503, {
                        "ok": False,
                        "error": {
                            "code": "ANGLES_UNAVAILABLE",
                            "message": "current angles unavailable (servo readback)",
                            "diagnostics": {},
                            "retry_hints": [
                                {"action": "retry_after_delay", "patch": None,
                                 "rationale": "サーボ readback 一時失敗。数秒待って再試行"},
                            ],
                        },
                    }); return
                result = VISION.perceive(
                    query=query, cameras=cameras,
                    use_table_plane=use_plane,
                    confidence_threshold=conf_thresh,
                    angles_deg=angles,
                    consensus=consensus,
                    refine=refine,
                    allow_uncalibrated=allow_uncalibrated,
                    save_frame=save_frame_flag,
                )
                self._json(200, result); return

            if path == "/abort":
                HUB.abort_flag.set()
                self._json(200, {"ok": True}); return

            if path == "/capture_calib_frame":
                """Body: {cam?: str}. Saves current JPEG frame into
                data/calib_images/<cam_id>/<YYYYMMDD_HHMMSS>.jpg.
                Returns {ok, path, count} on success.
                Error envelope: {ok:false, error:{code, message, retry_hints}}.
                """
                cam_id = body.get("cam") or (VISION.registry.default_cam_id() if VISION is not None else None)
                if VISION is None:
                    self._json(503, {
                        "ok": False,
                        "error": {
                            "code": "VISION_UNAVAILABLE",
                            "message": "vision サブシステム未初期化",
                            "retry_hints": [
                                {"action": "configure_calibration", "patch": None,
                                 "rationale": "data/calibration.json を作成してサーバ再起動"},
                            ],
                        },
                    }); return
                if cam_id is None:
                    self._json(400, {
                        "ok": False,
                        "error": {
                            "code": "CAM_REQUIRED",
                            "message": "cam id 未指定 (default cam も解決できず)",
                            "retry_hints": [
                                {"action": "specify_cam", "patch": {"cam": "wrist"},
                                 "rationale": "body に cam id を指定"},
                            ],
                        },
                    }); return
                if cam_id not in VISION.registry.cameras:
                    self._json(404, {
                        "ok": False,
                        "error": {
                            "code": "UNKNOWN_CAM",
                            "message": f"unknown camera id: {cam_id}",
                            "retry_hints": [
                                {"action": "use_known_cam",
                                 "patch": {"cam": sorted(VISION.registry.cameras.keys())[0]
                                           if VISION.registry.cameras else "wrist"},
                                 "rationale": "登録済みカメラのみ指定可"},
                            ],
                        },
                    }); return
                buf = VISION.registry.get_jpeg(cam_id)
                if not buf:
                    self._json(503, {
                        "ok": False,
                        "error": {
                            "code": "FRAME_UNAVAILABLE",
                            "message": f"camera {cam_id} からフレーム取得失敗",
                            "retry_hints": [
                                {"action": "retry_after_delay", "patch": None,
                                 "rationale": "USB camera 一時失敗。数秒待って再試行"},
                            ],
                        },
                    }); return
                out_dir = ROOT / "data" / "calib_images" / cam_id
                out_dir.mkdir(parents=True, exist_ok=True)
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                fpath = out_dir / f"{ts}.jpg"
                # avoid collision within same second
                i = 0
                while fpath.exists():
                    i += 1
                    fpath = out_dir / f"{ts}_{i:02d}.jpg"
                try:
                    with open(fpath, "wb") as f:
                        f.write(buf)
                except OSError as e:
                    self._json(500, {
                        "ok": False,
                        "error": {
                            "code": "WRITE_FAILED",
                            "message": f"file write failed: {e}",
                            "retry_hints": [],
                        },
                    }); return
                count = sum(1 for p in out_dir.glob("*.jpg"))
                rel = fpath.relative_to(ROOT).as_posix()
                self._json(200, {"ok": True, "path": rel, "count": count}); return

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
            # /perceive returns structured envelopes; others stay legacy {error: ...}.
            try:
                _path = urlparse(self.path).path
            except Exception:
                _path = ""
            if _path == "/perceive":
                self._json(400, _bad_request(
                    f"request error: {type(e).__name__}",
                    {"exception_type": type(e).__name__})); return
            self._json(400, {"error": str(e)})


def _emit_preflight(args, hub, vision) -> None:
    """Print bootstrap diagnostics to stderr just before serve_forever().

    Categories:
      arm hub                — real/virtual + port + power
      ANTHROPIC_API_KEY      — set/not set (value never printed)
      vision calibration     — placeholder check per camera
      workspace.table_z_mm   — implausible value detector (< FLOOR_Z)
      cam index mismatch     — CLI --cam vs calibration entry index
    Output goes to stderr so it doesn't interleave with HTTP access log on stdout.
    """
    lines: list[str] = []
    all_ok = True

    # arm hub
    if args.offline:
        lines.append("[OK]    arm hub: virtual hub (--offline)")
    else:
        port = getattr(getattr(hub, "arm", None), "port", "unknown")
        power_ok = False
        try:
            power_ok = bool(hub.power_ok())
        except Exception:
            power_ok = False
        tag = "[OK]" if power_ok else "[!] "
        if not power_ok: all_ok = False
        lines.append(f"{tag}    arm hub: real (port={port}, power={'on' if power_ok else 'OFF (E-stop?)'})")
        if not power_ok:
            lines.append("        fix: 緊急停止ボタンを解除して再起動 (時計回りで解除)")

    # ANTHROPIC_API_KEY
    if args.offline:
        lines.append("[OK]    ANTHROPIC_API_KEY: offline mode (skip)")
    else:
        key_set = bool(os.environ.get("ANTHROPIC_API_KEY"))
        tag = "[OK]" if key_set else "[!] "
        if not key_set: all_ok = False
        lines.append(f"{tag}    ANTHROPIC_API_KEY: {'set' if key_set else 'NOT SET'}")
        if not key_set:
            lines.append('        fix: $env:ANTHROPIC_API_KEY="sk-ant-..."')

    # vision calibration
    if vision is None:
        tag = "[OK]" if args.offline else "[!] "
        if not args.offline: all_ok = False
        lines.append(f"{tag}    vision calibration: subsystem not initialized")
        if not args.offline:
            lines.append("        fix: data/calibration.json を確認しサーバ再起動")
    else:
        try:
            cams = vision.registry.list()
        except Exception:
            cams = []
        placeholder_cams = [c for c in cams if c.get("placeholder")]
        if not cams:
            lines.append("[!]     vision calibration: no cameras registered")
            all_ok = False
        elif placeholder_cams:
            ids = ",".join(c["id"] for c in placeholder_cams)
            lines.append(f"[!]     vision calibration: PLACEHOLDER (cam {ids})")
            lines.append(f"        fix: python scripts/calibrate_intrinsics.py --cam {placeholder_cams[0]['id']} --hand-eye X,Y,Z,RX,RY,RZ")
            all_ok = False
        else:
            lines.append(f"[OK]    vision calibration: ok ({len(cams)} cam(s))")

        # workspace.table_z_mm
        try:
            tz = vision.registry.table_z_mm
        except Exception:
            tz = None
        if tz is None:
            lines.append("[!]     workspace.table_z_mm: missing")
            all_ok = False
        elif tz < FLOOR_Z:
            lines.append(f"[!]     workspace.table_z_mm = {tz} | {tz} < FLOOR_Z ({FLOOR_Z})")
            lines.append("        fix: python scripts/calibrate_intrinsics.py --table-z-mm <measured>")
            all_ok = False
        else:
            lines.append(f"[OK]    workspace.table_z_mm = {tz}")

        # cam index mismatch (CLI --cam vs calibration.wrist.index)
        try:
            wrist = vision.registry.cameras.get("wrist")
            wrist_idx = getattr(wrist, "index", None) if wrist is not None else None
        except Exception:
            wrist_idx = None
        if (not args.offline) and wrist_idx is not None and args.cam != wrist_idx:
            lines.append(f"[!]     cam index mismatch: --cam {args.cam} vs calibration.wrist.index {wrist_idx}")
            lines.append("        (CLI 優先で動作)")
            # not fatal — don't flip all_ok

    sep = "=" * 64
    print(sep, file=sys.stderr)
    print("           mycobot-lab preflight", file=sys.stderr)
    print(sep, file=sys.stderr)
    if all_ok:
        print("[OK] all preflight checks passed", file=sys.stderr)
    else:
        for ln in lines:
            print(ln, file=sys.stderr)
    print(sep, file=sys.stderr)


def lan_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close(); return ip
    except Exception:
        return "127.0.0.1"


def main():
    global HUB, VISION, INDEX_HTML, AUTH_TOKEN, SHUTTING_DOWN, MAX_SPEED_RUNTIME

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

    # Initialize vision subsystem (Phase 1). Tolerate missing calibration — endpoints will
    # return structured CALIBRATION_MISSING errors instead of crashing the server.
    calib_path = ROOT / "data" / "calibration.json"
    fixtures_path = ROOT / "data" / "fixtures" / "objects.json"
    try:
        if args.offline:
            VISION = VirtualVisionHub(calibration_path=calib_path, fixtures_path=fixtures_path)
        else:
            VISION = VisionHub(calibration_path=calib_path, fixtures_path=fixtures_path, offline=False)
            # Reuse the motion Hub's already-opened wrist VideoCapture so we don't fight over /dev/video0.
            wrist_cam_id = VISION.registry.default_cam_id()
            if wrist_cam_id is not None and getattr(HUB, "cap", None) is not None:
                VISION.attach_motion_cap(wrist_cam_id, HUB.cap)
        log.info("vision initialized: cameras=%s", [c["id"] for c in VISION.registry.list()])
        # Sanity warning: table_z below FLOOR_Z by > 50mm strongly suggests an
        # un-updated placeholder workspace block in calibration.json.
        try:
            _tz = VISION.registry.table_z_mm
            if _tz < FLOOR_Z - 50:
                print(
                    f"WARNING: data/calibration.json workspace.table_z_mm={_tz}mm は FLOOR_Z={FLOOR_Z}mm を大きく下回ります。"
                    f" 実測値で更新してください（このままだとテーブル上の物体が床下と判定されます）",
                    file=sys.stderr,
                )
        except Exception:
            pass
    except Exception as e:
        log.warning("vision init failed (continuing without): %s", e)
        VISION = None

    INDEX_HTML = (ROOT / "scripts" / "ui.html").read_text(encoding="utf-8")
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    mode = "OFFLINE" if args.offline else "live"
    print(f"\n  [{mode}] http://localhost:{args.port}/")
    if args.bind == "0.0.0.0":
        print(f"  [{mode}] http://{lan_ip()}:{args.port}/  (LAN, auth=token)\n")
    try:
        _emit_preflight(args, HUB, VISION)
    except Exception as _e:
        print(f"[preflight] failed to emit summary: {_e}", file=sys.stderr)

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
        try:
            if VISION is not None:
                VISION.shutdown()
        except Exception as e:
            print(f"vision shutdown failed: {e}")
        HUB.shutdown()


if __name__ == "__main__":
    main()
