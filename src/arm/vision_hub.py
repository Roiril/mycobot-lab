"""VisionHub — binds CameraRegistry + Detector + calibration into a perception API.

Read-only with respect to the motion stack. Uses the motion `Hub` only to query
the current joint angles (so we can compute the wrist camera's base-frame pose).

`perceive(query, ...)` is the top-level entry point used by `POST /perceive`.

Returned objects use the same `LocalizedObject` shape (a dict, not a dataclass,
to keep JSON-serialization trivial).
"""
from __future__ import annotations
import datetime
import logging
import pathlib
import time
from dataclasses import dataclass
from typing import Optional, List, Dict, Any, Sequence

from .vision.camera import CameraRegistry, Camera
from .vision.detector import Detector, Detection, ClaudeVLMDetector, FixtureDetector
from .vision.localizer import localize_on_table
from .vision.transforms import (
    T_base_cam_wrist, invert, mat_mul, T_base_ee,
)
from .kinematics import end_effector
from .constants import (
    FLOOR_Z, WORKSPACE_REACH_MAX_MM, WORKSPACE_Z_MAX_MM,
)
from . import poses as _poses


PERCEIVE_LOG_DIR_NAME = "perceive_log"
PERCEIVE_LOG_MAX_FILES = 200

log = logging.getLogger("mycobot.vision_hub")


CONFIDENCE_HIGH = 0.8
CONFIDENCE_LOW = 0.5
AMBIGUITY_DELTA = 0.15
OCCLUSION_EDGE_FRAC = 0.05
MIN_BBOX_AREA_PX = 100
DEPTH_UNCERTAINTY_LIMIT_MM = 50.0


def _frame_id(cam_id: str) -> str:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return f"{cam_id}_{ts}"


def _recommended_speed(confidence: float) -> int:
    if confidence >= CONFIDENCE_HIGH:
        return 20
    if confidence >= CONFIDENCE_LOW:
        return 10
    return 5


def _observe_move_patch(pose_angles, speed=20):
    """Build a retry_hints `patch` payload that drives /move to an OBSERVE pose."""
    return {"suggested_move": {"angles": list(pose_angles), "speed": int(speed)}}


def _observe_retry_hints(reason: str):
    """Concrete observe-from-another-angle hints with actual OBSERVE angles.

    `reason` is one of: 'not_found' | 'occluded' | 'low_confidence' | 'depth_uncertain'.
    Returns a list of {action, patch, rationale} dicts ordered most-promising first.
    """
    hints = []
    if reason == "depth_uncertain":
        hints.append({
            "action": "observe_from_overhead",
            "patch": _observe_move_patch(_poses.OBSERVE_HIGH, speed=20),
            "rationale": "光線がテーブル法線に近いほど uncertainty が下がる。OBSERVE_HIGH で真上に近い視点へ",
        })
        hints.append({
            "action": "refine_with_known_object_height",
            "patch": None,
            "rationale": "Phase 2: 物体形状仮定で平面交点を補正（未実装）",
        })
        return hints
    # default reach / observe-from-another-angle set
    hints.append({
        "action": "observe_from_another_angle",
        "patch": _observe_move_patch(_poses.OBSERVE_LEFT, speed=20),
        "rationale": "OBSERVE_LEFT (base +30°) で左側面から再撮影",
    })
    hints.append({
        "action": "observe_from_another_angle",
        "patch": _observe_move_patch(_poses.OBSERVE_RIGHT, speed=20),
        "rationale": "OBSERVE_RIGHT (base -30°) で右側面から再撮影",
    })
    hints.append({
        "action": "observe_from_overhead",
        "patch": _observe_move_patch(_poses.OBSERVE_HIGH, speed=20),
        "rationale": "OBSERVE_HIGH の俯瞰視点で全体を捉える",
    })
    return hints


def _bbox_occluded(bbox, frame_w, frame_h) -> bool:
    x, y, w, h = bbox
    if w * h < MIN_BBOX_AREA_PX:
        return True
    edge_x = OCCLUSION_EDGE_FRAC * frame_w
    edge_y = OCCLUSION_EDGE_FRAC * frame_h
    if x < edge_x or y < edge_y:
        return True
    if x + w > frame_w - edge_x or y + h > frame_h - edge_y:
        return True
    return False


class VisionHub:
    """Real vision hub: live cameras + real detector."""

    def __init__(self,
                 calibration_path: pathlib.Path,
                 fixtures_path: Optional[pathlib.Path] = None,
                 detector: Optional[Detector] = None,
                 offline: bool = False):
        self.registry = CameraRegistry(calibration_path, offline=offline)
        if detector is not None:
            self.detector = detector
        elif offline:
            if fixtures_path is None:
                raise ValueError("VisionHub offline mode requires fixtures_path")
            self.detector = FixtureDetector(fixtures_path)
        else:
            self.detector = ClaudeVLMDetector()
        self.fixtures_path = fixtures_path
        self.offline = offline
        # data/ root (sibling of calibration.json)
        self._data_root = calibration_path.parent
        # per-camera annotated JPEG cache, last successful perceive
        self._last_annotated_jpeg: Dict[str, bytes] = {}

    def attach_motion_cap(self, cam_id: str, cap):
        self.registry.attach_capture(cam_id, cap)

    def _api_key_present(self) -> bool:
        """True if the underlying detector has credentials. Only meaningful for
        ClaudeVLMDetector — Fixture detector returns True (no auth needed)."""
        det = getattr(self, "detector", None)
        if isinstance(det, ClaudeVLMDetector):
            return bool(getattr(det, "_api_key", None))
        return True

    def _camera_pose(self, cam: Camera, angles_deg: Sequence[float]):
        """Return T_base_cam (4x4)."""
        if cam.role == "wrist":
            if cam.hand_eye_T_ee_cam is None:
                return None
            return T_base_cam_wrist(angles_deg, cam.hand_eye_T_ee_cam)
        # fixed camera
        return cam.T_base_cam_fixed

    # ------------------------------------------------------------------
    # Observability helpers
    # ------------------------------------------------------------------
    def _now_iso(self) -> str:
        # Asia/Tokyo (UTC+9). Avoid pulling zoneinfo on systems without tzdata.
        return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(timespec="seconds")

    def _build_base_diagnostics(self,
                                 angles_deg: Optional[Sequence[float]],
                                 cameras_used: List[str]) -> Dict[str, Any]:
        diag: Dict[str, Any] = {
            "timestamp_iso": self._now_iso(),
            "cameras_used": list(cameras_used),
            "angles_at_capture": None,
            "tip_at_capture": None,
        }
        if angles_deg is not None:
            try:
                diag["angles_at_capture"] = [float(a) for a in angles_deg]
                tip = end_effector(list(angles_deg))
                # FK returns 6-tuple (x,y,z,rx,ry,rz). Some FKs may return only xyz.
                t = list(tip)
                if len(t) >= 6:
                    diag["tip_at_capture"] = {
                        "xyz": [float(t[0]), float(t[1]), float(t[2])],
                        "rpy": [float(t[3]), float(t[4]), float(t[5])],
                    }
                elif len(t) >= 3:
                    diag["tip_at_capture"] = {
                        "xyz": [float(t[0]), float(t[1]), float(t[2])],
                        "rpy": None,
                    }
            except Exception:
                log.exception("diagnostics: FK computation failed")
        return diag

    def _save_failure_frame(self,
                             frame,
                             cam_id: str,
                             error_code: str) -> Optional[str]:
        """Save the frame to data/perceive_log/ and return relative path on success."""
        if frame is None:
            return None
        try:
            log_dir = self._data_root / PERCEIVE_LOG_DIR_NAME
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
            fname = f"{ts}_{error_code}_{cam_id}.jpg"
            fpath = log_dir / fname
            import cv2
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return None
            with open(fpath, "wb") as f:
                f.write(buf.tobytes())
            self._rotate_perceive_log(log_dir)
            try:
                return str(fpath.relative_to(self._data_root.parent))
            except Exception:
                return str(fpath)
        except Exception:
            log.exception("perceive_log: save failed (cam=%s code=%s)", cam_id, error_code)
            return None

    def _rotate_perceive_log(self, log_dir: pathlib.Path):
        try:
            files = sorted(
                [p for p in log_dir.iterdir() if p.is_file()],
                key=lambda p: p.stat().st_mtime,
            )
            excess = len(files) - PERCEIVE_LOG_MAX_FILES
            if excess > 0:
                for p in files[:excess]:
                    try:
                        p.unlink()
                    except Exception:
                        pass
        except Exception:
            pass

    def _render_annotated_jpeg(self,
                                frame,
                                ok_results: List[Dict[str, Any]]) -> Optional[bytes]:
        """Draw bboxes + labels onto a copy of the frame and return JPEG bytes."""
        try:
            import cv2
            import numpy as _np
            disp = frame.copy()
            for i, r in enumerate(ok_results):
                bbox = r.get("bbox_px")
                if not bbox or len(bbox) != 4:
                    continue
                x, y, w, h = [int(round(float(v))) for v in bbox]
                color = (0, 200, 0) if i == 0 else (0, 220, 220)
                cv2.rectangle(disp, (x, y), (x + w, y + h), color, 2)
                label = f"{r.get('label', '?')} (conf={r.get('confidence', 0):.2f})"
                cv2.putText(disp, label, (x, max(0, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            ok2, buf = cv2.imencode(".jpg", disp, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok2:
                return None
            return buf.tobytes()
        except Exception:
            log.exception("annotate render failed")
            return None

    def get_last_annotated_jpeg(self, cam_id: str) -> Optional[bytes]:
        return self._last_annotated_jpeg.get(cam_id)

    def perceive(self,
                 query: str,
                 cameras: Optional[List[str]] = None,
                 use_table_plane: bool = True,
                 confidence_threshold: float = CONFIDENCE_LOW,
                 angles_deg: Optional[Sequence[float]] = None,
                 consensus: bool = False,
                 refine: bool = False,
                 allow_uncalibrated: bool = False,
                 save_frame: bool = False) -> Dict[str, Any]:
        """Run perception on the given cameras and return objects + meta.

        `angles_deg` is required for wrist cameras (caller supplies from motion_hub).
        Returns a dict that the HTTP layer can serialize directly. Includes both
        success and structured-error branches (caller still wraps with `ok` flag).

        `allow_uncalibrated=True` lets placeholder calibrations contribute results
        (flagged with `uncalibrated: true` on each object). Default False — never
        emit motion-bound coords from un-calibrated cameras unless caller opts in.
        """
        t_start = time.time()
        # Phase 1: warn but ignore consensus/refine (still received by API)
        warnings: List[str] = []
        if consensus:
            warnings.append("consensus not implemented in Phase 1 (ignored)")
        if refine:
            warnings.append("refine not implemented in Phase 1 (ignored)")
        # camera selection
        if cameras is None:
            cameras = [cid for cid, cam in self.registry.cameras.items() if cam.calibrated or cam.placeholder]
            if not cameras:
                default = self.registry.default_cam_id()
                cameras = [default] if default else []

        # Seed top-level diagnostics — every return path merges these in.
        base_diag = self._build_base_diagnostics(angles_deg, list(cameras))
        # Per-object localize results echo and rejected list (filled below).
        per_object_localize: List[Dict[str, Any]] = []
        rejected_localize: List[Dict[str, Any]] = []
        last_frame_for_save = None
        last_cam_id_for_save = None

        def _attach_warnings(resp: Dict[str, Any]) -> Dict[str, Any]:
            # Append any frame-size-mismatch warnings produced by the cameras used.
            for cid in base_diag.get("cameras_used", []):
                _c = self.registry.get(cid)
                if _c is None or _c.frame_size_actual is None:
                    continue
                dw, dh = _c.resolution
                aw, ah = _c.frame_size_actual
                if (int(dw), int(dh)) != (int(aw), int(ah)):
                    msg = (f"frame_size_mismatch[{cid}]: declared={dw}x{dh}, "
                           f"actual={aw}x{ah}; pixel-to-ray will be inaccurate")
                    if msg not in resp.get("warnings", []):
                        resp.setdefault("warnings", []).append(msg)
            return resp

        def _finish_error(resp: Dict[str, Any], error_code: str) -> Dict[str, Any]:
            # Merge top-level diagnostics into error.diagnostics (preserve existing keys).
            err = resp.get("error", {})
            edx = err.get("diagnostics") or {}
            for k, v in base_diag.items():
                edx.setdefault(k, v)
            if per_object_localize:
                edx.setdefault("objects", per_object_localize)
            if rejected_localize:
                edx.setdefault("rejected", rejected_localize)
            # VLM diagnostics echo
            try:
                vlm_diag = dict(getattr(self.detector, "last_call_diagnostics", {}) or {})
                if vlm_diag:
                    edx.setdefault("vlm", vlm_diag)
            except Exception:
                pass
            # save failure frame (best-effort)
            frame_path = None
            if last_frame_for_save is not None:
                frame_path = self._save_failure_frame(
                    last_frame_for_save,
                    last_cam_id_for_save or "unknown",
                    error_code,
                )
            if frame_path:
                edx["frame_path"] = frame_path
            err["diagnostics"] = edx
            resp["error"] = err
            resp.setdefault("warnings", warnings)
            return _attach_warnings(resp)

        if not cameras:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "CALIBRATION_MISSING",
                    "message": "登録カメラが無い (calibration.json 確認)",
                    "terminal": True,
                    "diagnostics": {"calibration_path": str(self.registry.calibration_path)},
                    "retry_hints": [
                        {"action": "configure_camera",
                         "patch": None,
                         "rationale": "data/calibration.json に少なくとも 1 台のカメラを定義し、 scripts/calibrate_intrinsics.py を実行"},
                    ],
                },
                "warnings": warnings,
            }, "CALIBRATION_MISSING")

        # Determine if any selected camera is fully calibrated (non-placeholder).
        # If not, and caller did not allow_uncalibrated, return a dedicated terminal error.
        has_fully_calibrated = any(
            (self.registry.get(cid) is not None
             and self.registry.get(cid).calibrated
             and not self.registry.get(cid).placeholder)
            for cid in cameras
        )
        if not has_fully_calibrated and not allow_uncalibrated:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "CALIBRATION_PLACEHOLDER_ONLY",
                    "message": "選択カメラはすべて placeholder calibration。world 座標を motion に流すには校正が必要",
                    "terminal": True,
                    "diagnostics": {
                        "cameras_tried": list(cameras),
                        "calibration_path": str(self.registry.calibration_path),
                    },
                    "retry_hints": [
                        {"action": "run_calibration",
                         "patch": None,
                         "rationale": "scripts/calibrate_intrinsics.py で K, dist を更新し、placeholder=false にする"},
                        {"action": "allow_uncalibrated_explicit",
                         "patch": {"allow_uncalibrated": True},
                         "rationale": "校正前でも仮の world 座標を受け取りたい場合（各 object に uncalibrated:true 付与）"},
                    ],
                },
                "warnings": warnings,
            }, "CALIBRATION_PLACEHOLDER_ONLY")

        all_results: List[Dict[str, Any]] = []
        vlm_latency_ms = 0.0
        skipped_reasons: List[str] = []
        out_of_workspace_count = 0

        for cam_id in cameras:
            cam = self.registry.get(cam_id)
            if cam is None:
                skipped_reasons.append(f"unknown camera id: {cam_id}")
                continue
            T_base_cam = self._camera_pose(cam, angles_deg) if angles_deg is not None else self._camera_pose(cam, [0.0]*6)
            if T_base_cam is None:
                skipped_reasons.append(f"{cam_id}: no calibration (no hand_eye_T_ee_cam / T_base_cam)")
                continue
            frame = self.registry.get_frame(cam_id)
            if frame is None:
                skipped_reasons.append(f"{cam_id}: frame unavailable")
                continue
            # Track most recent (cam, frame) pair so error returns can save it.
            last_frame_for_save = frame
            last_cam_id_for_save = cam_id
            fh, fw = frame.shape[:2]
            T_cam_base = invert(T_base_cam)
            # Note: `scale_to_orig` is intentionally NOT supplied here. The detector
            # computes it from the actual downscale it applies to the frame.
            image_meta = {
                "cam_id": cam_id,
                "frame_w": fw,
                "frame_h": fh,
                "T_cam_base": T_cam_base,
                "T_base_cam": T_base_cam,
                "K": cam.intrinsics["K"],
            }
            try:
                detections = self.detector.detect(frame, query, image_meta)
            except Exception as e:
                # Log full traceback server-side; do NOT leak repr(e) into response.
                log.exception("detector raised on cam_id=%s", cam_id)
                exc_name = type(e).__name__
                # Heuristic: authentication errors from the anthropic SDK are terminal.
                terminal = False
                lname = exc_name.lower()
                if "auth" in lname or "permission" in lname or "apikey" in lname:
                    terminal = True
                if not self._api_key_present():
                    terminal = True
                return _finish_error({
                    "ok": False,
                    "error": {
                        "code": "VLM_API_ERROR",
                        "message": f"VLM API failure: {exc_name}",
                        "terminal": terminal,
                        "diagnostics": {"cam_id": cam_id, "exception_type": exc_name},
                        "retry_hints": [
                            {"action": "retry_after_delay", "patch": None,
                             "rationale": "rate limit / network 一時障害なら数秒待って再試行 (terminal=true なら不可)"},
                            {"action": "fallback_to_fixture", "patch": None,
                             "rationale": "ANTHROPIC_API_KEY 未設定または認証失敗 — offline モードに切替"},
                        ],
                    },
                    "warnings": warnings,
                }, "VLM_API_ERROR")
            # collect per-detection latency
            for d in detections:
                vlm_latency_ms = max(vlm_latency_ms, float(d.extras.get("detect_ms", 0.0)))

            frame_id_str = _frame_id(cam_id)
            for det in detections:
                if _bbox_occluded(det.bbox_px, fw, fh):
                    all_results.append({
                        "label": det.label, "bbox_px": list(det.bbox_px),
                        "confidence": det.confidence,
                        "estimated_size_class": det.estimated_size_class,
                        "source_cam": cam_id, "frame_id": frame_id_str,
                        "_status": "occluded",
                    })
                    continue
                if not use_table_plane:
                    all_results.append({
                        "label": det.label, "bbox_px": list(det.bbox_px),
                        "confidence": det.confidence,
                        "estimated_size_class": det.estimated_size_class,
                        "source_cam": cam_id, "frame_id": frame_id_str,
                        "world_xyz_mm": None, "radius_mm": None,
                        "depth_uncertainty_mm": None,
                        "_status": "no_localization",
                    })
                    continue
                loc = localize_on_table(
                    {"bbox_px": det.bbox_px,
                     "estimated_size_class": det.estimated_size_class},
                    cam, T_base_cam, table_z_mm=self.registry.table_z_mm,
                )
                # Echo localize intermediate values for observability (success or fail).
                localize_echo = {
                    "label": det.label,
                    "source_cam": cam_id,
                    "bbox_px": list(det.bbox_px),
                    "localize": dict(loc),
                }
                per_object_localize.append(localize_echo)
                if "error" in loc:
                    err_code = loc["error"]
                    # Map BBOX_OUT_OF_FRAME / BBOX_INVALID into occluded status so
                    # callers see a consistent retry path.
                    if err_code in ("BBOX_OUT_OF_FRAME", "BBOX_INVALID"):
                        all_results.append({
                            "label": det.label, "bbox_px": list(det.bbox_px),
                            "confidence": det.confidence,
                            "estimated_size_class": det.estimated_size_class,
                            "source_cam": cam_id, "frame_id": frame_id_str,
                            "_status": "occluded",
                        })
                        continue
                    all_results.append({
                        "label": det.label, "bbox_px": list(det.bbox_px),
                        "confidence": det.confidence,
                        "estimated_size_class": det.estimated_size_class,
                        "source_cam": cam_id, "frame_id": frame_id_str,
                        "_status": f"localize_failed:{err_code}",
                    })
                    continue
                # workspace uncertainty adds to depth uncertainty
                depth_unc = loc["depth_uncertainty_mm"] + self.registry.table_z_uncertainty_mm
                world = loc["xyz_base"]
                wx, wy, wz = float(world[0]), float(world[1]), float(world[2])
                # Workspace sanity check: reject anything outside arm reach cylinder
                # or below FLOOR_Z (table_z<FLOOR_Z config drift) or above WORKSPACE_Z_MAX.
                reach = (wx * wx + wy * wy) ** 0.5
                out_of_ws = (reach > WORKSPACE_REACH_MAX_MM
                             or wz < FLOOR_Z
                             or wz > WORKSPACE_Z_MAX_MM)
                if out_of_ws:
                    out_of_workspace_count += 1
                    rejected_localize.append({
                        "reason": "out_of_workspace",
                        "label": det.label, "source_cam": cam_id,
                        "world_xyz_mm": [wx, wy, wz],
                        "localize": dict(loc),
                    })
                    all_results.append({
                        "label": det.label,
                        "world_xyz_mm": [round(wx, 1), round(wy, 1), round(wz, 1)],
                        "radius_mm": round(loc["radius_mm"], 1),
                        "confidence": round(det.confidence, 3),
                        "depth_uncertainty_mm": round(depth_unc, 1),
                        "source_cam": cam_id,
                        "frame_id": frame_id_str,
                        "bbox_px": [round(v, 1) for v in det.bbox_px],
                        "estimated_size_class": det.estimated_size_class,
                        "out_of_workspace": True,
                        "uncalibrated": (cam.placeholder),
                        "_status": "out_of_workspace",
                        "_workspace_diag": {
                            "reach_mm": round(reach, 1),
                            "z_mm": round(wz, 1),
                            "limits": {
                                "reach_max_mm": WORKSPACE_REACH_MAX_MM,
                                "z_min_mm": FLOOR_Z,
                                "z_max_mm": WORKSPACE_Z_MAX_MM,
                            },
                        },
                    })
                    continue
                entry = {
                    "label": det.label,
                    "world_xyz_mm": [round(wx, 1), round(wy, 1), round(wz, 1)],
                    "radius_mm": round(loc["radius_mm"], 1),
                    "confidence": round(det.confidence, 3),
                    "depth_uncertainty_mm": round(depth_unc, 1),
                    "source_cam": cam_id,
                    "frame_id": frame_id_str,
                    "bbox_px": [round(v, 1) for v in det.bbox_px],
                    "estimated_size_class": det.estimated_size_class,
                    "_status": "ok",
                }
                if cam.placeholder:
                    entry["uncalibrated"] = True
                all_results.append(entry)

        ok_results = [r for r in all_results if r.get("_status") == "ok"]
        occluded = [r for r in all_results if r.get("_status") == "occluded"]
        oow_results = [r for r in all_results if r.get("_status") == "out_of_workspace"]

        elapsed_ms = (time.time() - t_start) * 1000.0

        if not all_results:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "OBJECT_NOT_FOUND",
                    "message": f"クエリ「{query}」に一致する物体が見つからない",
                    "diagnostics": {"cameras_tried": cameras, "skipped": skipped_reasons},
                    "retry_hints": (
                        _observe_retry_hints("not_found") + [
                            {"action": "narrow_query", "patch": None,
                             "rationale": "クエリをより具体的に（色・形・大きさを追加）"},
                            {"action": "use_different_camera",
                             "patch": {"cameras": list(self.registry.cameras.keys())},
                             "rationale": "別のカメラから観察"},
                        ]
                    ),
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
                "warnings": warnings,
            }, "OBJECT_NOT_FOUND")

        if not ok_results and occluded:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "OCCLUDED",
                    "message": "検出物体が画像端／極小サイズで信用できない",
                    "diagnostics": {"occluded_count": len(occluded), "sample": occluded[:3]},
                    "retry_hints": (
                        _observe_retry_hints("occluded") + [
                            {"action": "zoom_in", "patch": None,
                             "rationale": "アームを近づけて再撮影 (Phase 2 で refine API 予定)"},
                        ]
                    ),
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
                "warnings": warnings,
            }, "OCCLUDED")

        if not ok_results and oow_results:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "OUT_OF_WORKSPACE",
                    "message": (
                        f"検出物体 {len(oow_results)} 件すべて workspace 外 "
                        f"(reach > {WORKSPACE_REACH_MAX_MM}mm or z 範囲外 [{FLOOR_Z}, {WORKSPACE_Z_MAX_MM}])"
                    ),
                    "diagnostics": {
                        "out_of_workspace_count": len(oow_results),
                        "sample": oow_results[:3],
                        "limits": {
                            "reach_max_mm": WORKSPACE_REACH_MAX_MM,
                            "z_min_mm": FLOOR_Z,
                            "z_max_mm": WORKSPACE_Z_MAX_MM,
                        },
                    },
                    "retry_hints": _observe_retry_hints("not_found") + [
                        {"action": "verify_calibration", "patch": None,
                         "rationale": "world 座標が workspace 外 — hand-eye / intrinsics が誤っている可能性。calibration を再確認"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
                "warnings": warnings,
            }, "OUT_OF_WORKSPACE")

        # rank by confidence
        ok_results.sort(key=lambda r: r["confidence"], reverse=True)
        top = ok_results[0]

        # ambiguity check
        if len(ok_results) >= 2:
            top2 = ok_results[1]
            if top["confidence"] > confidence_threshold and top2["confidence"] > confidence_threshold:
                if top["confidence"] - top2["confidence"] < AMBIGUITY_DELTA:
                    return _finish_error({
                        "ok": False,
                        "error": {
                            "code": "MULTIPLE_AMBIGUOUS",
                            "message": f"上位 2 候補の confidence 差 < {AMBIGUITY_DELTA} (両方 > {confidence_threshold})",
                            "diagnostics": {"candidates": ok_results[:3]},
                            "retry_hints": [
                                {"action": "narrow_query", "patch": None,
                                 "rationale": "どちらか一方を識別できる属性を追加（色・位置等）"},
                                {"action": "lower_confidence_threshold",
                                 "patch": {"confidence_threshold": 0.3},
                                 "rationale": "曖昧でも top を採用したい場合"},
                            ],
                        },
                        "vlm_latency_ms": round(vlm_latency_ms, 1),
                        "elapsed_ms": round(elapsed_ms, 1),
                        "warnings": warnings,
                    }, "MULTIPLE_AMBIGUOUS")

        # confidence threshold check
        if top["confidence"] < confidence_threshold:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "LOW_CONFIDENCE",
                    "message": f"最良候補 confidence={top['confidence']:.2f} < threshold={confidence_threshold:.2f}",
                    "diagnostics": {"top": top, "all": ok_results[:3]},
                    # observe_from_another_angle first (best signal-improving action);
                    # lower_confidence_threshold is a fallback that does NOT improve
                    # detection quality — only relaxes the gate.
                    "retry_hints": _observe_retry_hints("low_confidence") + [
                        {"action": "lower_confidence_threshold",
                         "patch": {"confidence_threshold": max(0.2, top["confidence"] - 0.05)},
                         "rationale": "fallback — 検出品質は改善しない、閾値を緩めるだけ。リスクは呼出側"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
                "warnings": warnings,
            }, "LOW_CONFIDENCE")

        # depth check
        if top.get("depth_uncertainty_mm", 0) > DEPTH_UNCERTAINTY_LIMIT_MM:
            return _finish_error({
                "ok": False,
                "error": {
                    "code": "DEPTH_UNCERTAIN",
                    "message": f"depth_uncertainty {top['depth_uncertainty_mm']:.1f}mm > {DEPTH_UNCERTAINTY_LIMIT_MM}mm",
                    "diagnostics": {"top": top},
                    "retry_hints": _observe_retry_hints("depth_uncertain"),
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
                "warnings": warnings,
            }, "DEPTH_UNCERTAIN")

        # success — strip _status / private diags, populate recommended_speed
        clean = []
        for r in ok_results:
            r2 = {k: v for k, v in r.items() if not k.startswith("_")}
            clean.append(r2)

        diagnostics = dict(base_diag)
        if out_of_workspace_count > 0:
            diagnostics["out_of_workspace_excluded"] = out_of_workspace_count
        if per_object_localize:
            diagnostics["objects"] = per_object_localize
        if rejected_localize:
            diagnostics["rejected"] = rejected_localize
        try:
            vlm_diag = dict(getattr(self.detector, "last_call_diagnostics", {}) or {})
            if vlm_diag:
                diagnostics["vlm"] = vlm_diag
        except Exception:
            pass

        # Annotated JPEG cache (one per cam_id that produced ok_results)
        if last_frame_for_save is not None and last_cam_id_for_save is not None:
            ann = self._render_annotated_jpeg(last_frame_for_save, clean)
            if ann:
                self._last_annotated_jpeg[last_cam_id_for_save] = ann

        # Optional success-time frame save (explicit opt-in only).
        if save_frame and last_frame_for_save is not None:
            fp = self._save_failure_frame(last_frame_for_save,
                                           last_cam_id_for_save or "unknown",
                                           "ok")
            if fp:
                diagnostics["frame_path"] = fp

        resp = {
            "ok": True,
            "objects": clean,
            "vlm_latency_ms": round(vlm_latency_ms, 1),
            "elapsed_ms": round(elapsed_ms, 1),
            "recommended_speed": _recommended_speed(top["confidence"]),
            "consensus_used": False,  # Phase 1: single-shot only
            "warnings": warnings,
            "diagnostics": diagnostics,
        }
        return _attach_warnings(resp)

    def shutdown(self):
        try:
            self.registry.shutdown()
        except Exception:
            pass


class VirtualVisionHub(VisionHub):
    """Offline vision hub: FixtureDetector + synthetic frames."""

    def __init__(self, calibration_path: pathlib.Path, fixtures_path: pathlib.Path):
        super().__init__(calibration_path=calibration_path,
                         fixtures_path=fixtures_path,
                         detector=FixtureDetector(fixtures_path),
                         offline=True)
