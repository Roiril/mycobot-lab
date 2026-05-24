"""Camera registry + per-camera VideoCapture management.

Loads camera definitions from data/calibration.json, manages cv2.VideoCapture
lazily (open on first use, reuse across calls), and serves frames/jpegs.
Thread-safe via per-camera locks.

Phase 1: only the "wrist" camera is actually opened. Other roles (overhead, side)
can appear in calibration.json but won't be probed unless explicitly requested.
"""
from __future__ import annotations
import json
import threading
import time
import logging
import pathlib
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any

import numpy as np

log = logging.getLogger("mycobot.vision.camera")


@dataclass
class Camera:
    id: str
    index: int
    role: str
    resolution: tuple  # (w, h)
    intrinsics: Dict[str, Any]  # {"K": [[..]], "dist": [..]}
    hand_eye_T_ee_cam: Optional[List[List[float]]] = None  # 4x4, None for fixed cams
    T_base_cam_fixed: Optional[List[List[float]]] = None   # for non-wrist (overhead/side)
    calibrated: bool = False
    placeholder: bool = False

    @property
    def is_wrist(self) -> bool:
        return self.role == "wrist"


def _load_calibration(path: pathlib.Path) -> Dict[str, Any]:
    if not path.exists():
        return {"version": 0, "cameras": {}, "workspace": {"table_z_mm": 0.0, "table_z_uncertainty_mm": 10.0}}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _build_camera(cam_id: str, cfg: Dict[str, Any]) -> Camera:
    res = cfg.get("resolution", [640, 480])
    intrinsics = cfg.get("intrinsics", {"K": [[500, 0, 320], [0, 500, 240], [0, 0, 1]], "dist": [0, 0, 0, 0, 0]})
    hand_eye = cfg.get("hand_eye_T_ee_cam")
    fixed = cfg.get("T_base_cam")
    placeholder = bool(cfg.get("placeholder", False))
    calibrated = not placeholder and (hand_eye is not None or fixed is not None)
    return Camera(
        id=cam_id,
        index=int(cfg.get("index", 0)),
        role=cfg.get("role", "wrist"),
        resolution=(int(res[0]), int(res[1])),
        intrinsics=intrinsics,
        hand_eye_T_ee_cam=hand_eye,
        T_base_cam_fixed=fixed,
        calibrated=calibrated,
        placeholder=placeholder,
    )


class CameraRegistry:
    """Manages cv2.VideoCapture handles per camera. Lazy-opens, thread-safe.

    Wrapped around (and partially supersedes) Hub.frame_jpeg(). For backward
    compatibility, callers may also pass in an externally-managed VideoCapture
    via attach_capture() — this lets the existing Hub.cap remain primary.
    """

    def __init__(self, calibration_path: pathlib.Path, offline: bool = False):
        self.offline = offline
        self.calibration_path = calibration_path
        self.calib = _load_calibration(calibration_path)
        self.workspace = self.calib.get("workspace", {"table_z_mm": 0.0, "table_z_uncertainty_mm": 10.0})

        self.cameras: Dict[str, Camera] = {}
        for cam_id, cfg in self.calib.get("cameras", {}).items():
            try:
                self.cameras[cam_id] = _build_camera(cam_id, cfg)
            except Exception as e:
                log.warning("camera %s config invalid: %s", cam_id, e)

        # cv2.VideoCapture handles (None until opened)
        self._caps: Dict[str, Any] = {}
        self._locks: Dict[str, threading.Lock] = {cid: threading.Lock() for cid in self.cameras}
        self._last_jpeg: Dict[str, Optional[bytes]] = {cid: None for cid in self.cameras}
        self._external_cap = None  # set via attach_capture for legacy hub.cap reuse
        self._external_cam_id: Optional[str] = None

    def attach_capture(self, cam_id: str, cap):
        """Reuse an externally-managed cv2.VideoCapture (e.g. Hub.cap)."""
        self._external_cam_id = cam_id
        self._external_cap = cap

    def list(self) -> List[Dict[str, Any]]:
        out = []
        for cid, cam in self.cameras.items():
            out.append({
                "id": cid,
                "role": cam.role,
                "resolution": list(cam.resolution),
                "calibrated": cam.calibrated,
                "placeholder": cam.placeholder,
            })
        return out

    def get(self, cam_id: str) -> Optional[Camera]:
        return self.cameras.get(cam_id)

    def default_cam_id(self) -> Optional[str]:
        # Preference: wrist > first listed
        for cid, cam in self.cameras.items():
            if cam.role == "wrist":
                return cid
        return next(iter(self.cameras.keys()), None)

    def _open(self, cam_id: str):
        """Lazy-open VideoCapture. Returns the handle or None."""
        if cam_id in self._caps and self._caps[cam_id] is not None:
            return self._caps[cam_id]
        if self._external_cap is not None and cam_id == self._external_cam_id:
            return self._external_cap
        if self.offline:
            return None
        cam = self.cameras.get(cam_id)
        if cam is None:
            return None
        try:
            import cv2
            cap = cv2.VideoCapture(cam.index, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.resolution[1])
            # warmup
            for _ in range(4):
                cap.read(); time.sleep(0.05)
            self._caps[cam_id] = cap
            return cap
        except Exception as e:
            log.warning("camera %s open failed: %s", cam_id, e)
            return None

    def get_frame(self, cam_id: str) -> Optional[np.ndarray]:
        """Read a BGR frame for the given camera. Returns None on failure."""
        if cam_id not in self.cameras:
            return None
        if self.offline:
            return self._offline_frame(cam_id)
        lock = self._locks.setdefault(cam_id, threading.Lock())
        with lock:
            cap = self._open(cam_id)
            if cap is None:
                return None
            ok, frame = cap.read()
            if not ok:
                return None
            return frame

    def get_jpeg(self, cam_id: str, quality: int = 85) -> Optional[bytes]:
        """Read and JPEG-encode a frame. Returns the encoded bytes, or the last-known
        jpeg on read failure (matches Hub.frame_jpeg semantics)."""
        if cam_id not in self.cameras:
            return None
        if self.offline:
            return self._offline_jpeg(cam_id)
        try:
            import cv2
        except Exception:
            return None
        frame = self.get_frame(cam_id)
        if frame is None:
            return self._last_jpeg.get(cam_id)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            return self._last_jpeg.get(cam_id)
        self._last_jpeg[cam_id] = buf.tobytes()
        return self._last_jpeg[cam_id]

    def _offline_frame(self, cam_id: str) -> Optional[np.ndarray]:
        cam = self.cameras.get(cam_id)
        if cam is None:
            return None
        w, h = cam.resolution
        img = np.full((h, w, 3), 30, dtype=np.uint8)
        try:
            import cv2
            cv2.putText(img, f"OFFLINE [{cam_id}]", (20, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 220), 2)
        except Exception:
            pass
        return img

    def _offline_jpeg(self, cam_id: str) -> Optional[bytes]:
        img = self._offline_frame(cam_id)
        if img is None:
            return None
        try:
            import cv2
            ok, buf = cv2.imencode(".jpg", img)
            return buf.tobytes() if ok else None
        except Exception:
            return None

    def shutdown(self):
        for cid, cap in list(self._caps.items()):
            try:
                if cap is not None and cap is not self._external_cap:
                    cap.release()
            except Exception:
                pass
        self._caps.clear()

    @property
    def table_z_mm(self) -> float:
        return float(self.workspace.get("table_z_mm", 0.0))

    @property
    def table_z_uncertainty_mm(self) -> float:
        return float(self.workspace.get("table_z_uncertainty_mm", 10.0))
