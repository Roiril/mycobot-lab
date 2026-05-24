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

    def attach_motion_cap(self, cam_id: str, cap):
        self.registry.attach_capture(cam_id, cap)

    def _camera_pose(self, cam: Camera, angles_deg: Sequence[float]):
        """Return T_base_cam (4x4)."""
        if cam.role == "wrist":
            if cam.hand_eye_T_ee_cam is None:
                return None
            return T_base_cam_wrist(angles_deg, cam.hand_eye_T_ee_cam)
        # fixed camera
        return cam.T_base_cam_fixed

    def perceive(self,
                 query: str,
                 cameras: Optional[List[str]] = None,
                 use_table_plane: bool = True,
                 confidence_threshold: float = CONFIDENCE_LOW,
                 angles_deg: Optional[Sequence[float]] = None,
                 consensus: bool = False) -> Dict[str, Any]:
        """Run perception on the given cameras and return objects + meta.

        `angles_deg` is required for wrist cameras (caller supplies from motion_hub).
        Returns a dict that the HTTP layer can serialize directly. Includes both
        success and structured-error branches (caller still wraps with `ok` flag).
        """
        t_start = time.time()
        # camera selection
        if cameras is None:
            cameras = [cid for cid, cam in self.registry.cameras.items() if cam.calibrated or cam.placeholder]
            if not cameras:
                default = self.registry.default_cam_id()
                cameras = [default] if default else []

        if not cameras:
            return {
                "ok": False,
                "error": {
                    "code": "CALIBRATION_MISSING",
                    "message": "登録カメラが無い (calibration.json 確認)",
                    "diagnostics": {"calibration_path": str(self.registry.calibration_path)},
                    "retry_hints": [
                        {"action": "configure_camera",
                         "patch": None,
                         "rationale": "data/calibration.json に少なくとも 1 台のカメラを定義し、 scripts/calibrate_intrinsics.py を実行"},
                    ],
                },
            }

        all_results: List[Dict[str, Any]] = []
        vlm_latency_ms = 0.0
        skipped_reasons: List[str] = []

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
            fh, fw = frame.shape[:2]
            T_cam_base = invert(T_base_cam)
            image_meta = {
                "cam_id": cam_id,
                "frame_w": fw,
                "frame_h": fh,
                "T_cam_base": T_cam_base,
                "T_base_cam": T_base_cam,
                "K": cam.intrinsics["K"],
                "scale_to_orig": 1.0,
            }
            try:
                detections = self.detector.detect(frame, query, image_meta)
            except Exception as e:
                return {
                    "ok": False,
                    "error": {
                        "code": "VLM_API_ERROR",
                        "message": f"detector error: {e}",
                        "diagnostics": {"cam_id": cam_id},
                        "retry_hints": [
                            {"action": "retry_after_delay", "patch": None,
                             "rationale": "API 一時障害の可能性。数秒待って再試行"},
                            {"action": "fallback_to_fixture", "patch": None,
                             "rationale": "ANTHROPIC_API_KEY 未設定または SDK 障害 — offline モードに切替"},
                        ],
                    },
                }
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
                if "error" in loc:
                    all_results.append({
                        "label": det.label, "bbox_px": list(det.bbox_px),
                        "confidence": det.confidence,
                        "estimated_size_class": det.estimated_size_class,
                        "source_cam": cam_id, "frame_id": frame_id_str,
                        "_status": f"localize_failed:{loc['error']}",
                    })
                    continue
                # workspace uncertainty adds to depth uncertainty
                depth_unc = loc["depth_uncertainty_mm"] + self.registry.table_z_uncertainty_mm
                all_results.append({
                    "label": det.label,
                    "world_xyz_mm": [round(c, 1) for c in loc["xyz_base"]],
                    "radius_mm": round(loc["radius_mm"], 1),
                    "confidence": round(det.confidence, 3),
                    "depth_uncertainty_mm": round(depth_unc, 1),
                    "source_cam": cam_id,
                    "frame_id": frame_id_str,
                    "bbox_px": [round(v, 1) for v in det.bbox_px],
                    "estimated_size_class": det.estimated_size_class,
                    "_status": "ok",
                })

        ok_results = [r for r in all_results if r.get("_status") == "ok"]
        occluded = [r for r in all_results if r.get("_status") == "occluded"]

        elapsed_ms = (time.time() - t_start) * 1000.0

        if not all_results:
            return {
                "ok": False,
                "error": {
                    "code": "OBJECT_NOT_FOUND",
                    "message": f"クエリ「{query}」に一致する物体が見つからない",
                    "diagnostics": {"cameras_tried": cameras, "skipped": skipped_reasons},
                    "retry_hints": [
                        {"action": "observe_from_another_angle", "patch": None,
                         "rationale": "アームを別位置に動かして再撮影"},
                        {"action": "narrow_query", "patch": None,
                         "rationale": "クエリをより具体的に（色・形・大きさを追加）"},
                        {"action": "use_different_camera",
                         "patch": {"cameras": list(self.registry.cameras.keys())},
                         "rationale": "別のカメラから観察"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
            }

        if not ok_results and occluded:
            return {
                "ok": False,
                "error": {
                    "code": "OCCLUDED",
                    "message": "検出物体が画像端／極小サイズで信用できない",
                    "diagnostics": {"occluded_count": len(occluded), "sample": occluded[:3]},
                    "retry_hints": [
                        {"action": "observe_from_another_angle", "patch": None,
                         "rationale": "物体が画面中央に来る位置へ移動"},
                        {"action": "zoom_in", "patch": None,
                         "rationale": "アームを近づけて再撮影 (Phase 2 で refine API 予定)"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
            }

        # rank by confidence
        ok_results.sort(key=lambda r: r["confidence"], reverse=True)
        top = ok_results[0]

        # ambiguity check
        if len(ok_results) >= 2:
            top2 = ok_results[1]
            if top["confidence"] > confidence_threshold and top2["confidence"] > confidence_threshold:
                if top["confidence"] - top2["confidence"] < AMBIGUITY_DELTA:
                    return {
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
                    }

        # confidence threshold check
        if top["confidence"] < confidence_threshold:
            return {
                "ok": False,
                "error": {
                    "code": "LOW_CONFIDENCE",
                    "message": f"最良候補 confidence={top['confidence']:.2f} < threshold={confidence_threshold:.2f}",
                    "diagnostics": {"top": top, "all": ok_results[:3]},
                    "retry_hints": [
                        {"action": "observe_from_another_angle", "patch": None,
                         "rationale": "より見える位置から再撮影"},
                        {"action": "lower_confidence_threshold",
                         "patch": {"confidence_threshold": max(0.2, top["confidence"] - 0.05)},
                         "rationale": "閾値を緩めて採用 (リスクは呼出側)"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
            }

        # depth check
        if top.get("depth_uncertainty_mm", 0) > DEPTH_UNCERTAINTY_LIMIT_MM:
            return {
                "ok": False,
                "error": {
                    "code": "DEPTH_UNCERTAIN",
                    "message": f"depth_uncertainty {top['depth_uncertainty_mm']:.1f}mm > {DEPTH_UNCERTAINTY_LIMIT_MM}mm",
                    "diagnostics": {"top": top},
                    "retry_hints": [
                        {"action": "observe_from_overhead", "patch": None,
                         "rationale": "光線がテーブル法線に近いほど uncertainty が下がる。真上に近づける"},
                        {"action": "refine_with_known_object_height", "patch": None,
                         "rationale": "Phase 2: 物体形状仮定で平面交点を補正"},
                    ],
                },
                "vlm_latency_ms": round(vlm_latency_ms, 1),
                "elapsed_ms": round(elapsed_ms, 1),
            }

        # success — strip _status, populate recommended_speed
        clean = []
        for r in ok_results:
            r2 = {k: v for k, v in r.items() if not k.startswith("_")}
            clean.append(r2)

        return {
            "ok": True,
            "objects": clean,
            "vlm_latency_ms": round(vlm_latency_ms, 1),
            "elapsed_ms": round(elapsed_ms, 1),
            "recommended_speed": _recommended_speed(top["confidence"]),
            "consensus_used": False,  # Phase 1: single-shot only
        }

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
