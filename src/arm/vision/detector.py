"""Object detection backends.

`Detector` ABC accepts a BGR frame + natural-language query and returns a list of
`Detection`. Backends:
  - `ClaudeVLMDetector`: calls Anthropic vision API. Requires ANTHROPIC_API_KEY.
  - `FixtureDetector`: deterministic, reads fixtures from JSON. Used offline / tests.

Phase 1 keeps the prompt + parsing simple; Phase 2 may add per-class shape priors
and multi-bbox clustering.
"""
from __future__ import annotations
import abc
import base64
import io
import json
import logging
import os
import re
import time
import pathlib
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional, Sequence

import numpy as np

log = logging.getLogger("mycobot.vision.detector")


SIZE_CLASSES = ("small", "medium", "large")
DEFAULT_VLM_MODEL = "claude-sonnet-4-6"
VLM_TIMEOUT_SEC = 15.0


@dataclass
class Detection:
    label: str
    bbox_px: tuple              # (x, y, w, h) in pixels
    confidence: float           # 0..1
    estimated_size_class: str   # "small" | "medium" | "large"
    extras: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["bbox_px"] = list(self.bbox_px)
        return d


class Detector(abc.ABC):
    @abc.abstractmethod
    def detect(self, frame: np.ndarray, query: str, image_meta: Dict[str, Any]) -> List[Detection]:
        ...


# ----------------------------------------------------------------------
# Claude vision-based detector
# ----------------------------------------------------------------------

VLM_PROMPT_TEMPLATE = """あなたは画像から物体を検出して bbox を返すビジョンモデルです。

ユーザーのクエリ: 「{query}」

画像サイズ: 幅 {w} px × 高さ {h} px

応答は **JSON のみ**（コードブロック・前置き・後置き禁止）。スキーマ：
{{
  "objects": [
    {{
      "label": "<クエリに合う物体の短い名前>",
      "bbox_px": [x, y, w, h],
      "confidence": 0.0-1.0,
      "estimated_size_class": "small" | "medium" | "large"
    }}
  ]
}}

size_class の意味（物理サイズ・直径ベース）：
- small:  直径 < 3cm  (ペン、コイン、小物)
- medium: 3-8cm       (コップ、マグ、缶)
- large:  > 8cm       (箱、本、皿)

検出ガイドライン：
- クエリにマッチする物体のみ返す。無ければ "objects": []
- bbox は物体を密に囲む矩形（タイト）
- confidence は本物が含まれている確信度 (0-1)
- 物体が画像端で切れている、ぼやけている、隠れている → confidence を下げる
"""


def _bgr_to_jpeg_b64(frame: np.ndarray, max_w: int = 640, quality: int = 85) -> tuple[str, int, int]:
    """Return (base64_str, width, height) of JPEG-encoded frame, downscaled if needed."""
    try:
        import cv2
    except Exception as e:
        raise RuntimeError(f"cv2 unavailable: {e}")
    h, w = frame.shape[:2]
    scale = 1.0
    if w > max_w:
        scale = max_w / float(w)
        new_w = int(w * scale)
        new_h = int(h * scale)
        frame = cv2.resize(frame, (new_w, new_h))
        h, w = new_h, new_w
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    b64 = base64.standard_b64encode(buf.tobytes()).decode("ascii")
    return b64, w, h


def _extract_json(text: str) -> Optional[Dict[str, Any]]:
    """Try strict JSON parse, then look for a top-level {...} block as fallback."""
    if not text:
        return None
    text = text.strip()
    try:
        return json.loads(text)
    except Exception:
        pass
    # find a brace-balanced JSON object
    m = re.search(r"\{[\s\S]*\}", text)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except Exception:
        return None


class ClaudeVLMDetector(Detector):
    """Anthropic Claude vision-based detector. Lazy-imports anthropic SDK."""

    def __init__(self, model: str = DEFAULT_VLM_MODEL, api_key: Optional[str] = None):
        self.model = model
        self._api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        self._client = None  # lazy

    def _ensure_client(self):
        if self._client is not None:
            return self._client
        if not self._api_key:
            raise RuntimeError("ANTHROPIC_API_KEY not set; cannot call Claude vision API")
        try:
            import anthropic
        except ImportError as e:
            raise RuntimeError(f"anthropic SDK not installed: {e}")
        self._client = anthropic.Anthropic(api_key=self._api_key)
        return self._client

    def detect(self, frame: np.ndarray, query: str, image_meta: Dict[str, Any]) -> List[Detection]:
        t0 = time.time()
        try:
            client = self._ensure_client()
        except RuntimeError as e:
            log.warning("ClaudeVLMDetector unavailable: %s", e)
            raise
        b64, w, h = _bgr_to_jpeg_b64(frame)
        prompt = VLM_PROMPT_TEMPLATE.format(query=query, w=w, h=h)
        try:
            resp = client.messages.create(
                model=self.model,
                max_tokens=1024,
                timeout=VLM_TIMEOUT_SEC,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "image",
                         "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
                        {"type": "text", "text": prompt},
                    ],
                }],
            )
        except Exception as e:
            log.warning("Claude vision API error: %s", e)
            raise

        text = ""
        try:
            for block in resp.content:
                if getattr(block, "type", None) == "text":
                    text += block.text
        except Exception:
            text = str(resp)

        parsed = _extract_json(text)
        latency_ms = (time.time() - t0) * 1000.0
        out: List[Detection] = []
        if not parsed or "objects" not in parsed:
            log.warning("VLM returned unparseable response: %r", text[:300])
            return out
        scale = image_meta.get("scale_to_orig", 1.0)
        for obj in parsed.get("objects", []):
            try:
                bbox = obj["bbox_px"]
                if len(bbox) != 4:
                    continue
                # Rescale bbox from downscaled image back to original frame size
                x, y, bw, bh = (float(bbox[0]) * scale, float(bbox[1]) * scale,
                                float(bbox[2]) * scale, float(bbox[3]) * scale)
                conf = float(obj.get("confidence", 0.0))
                sclass = obj.get("estimated_size_class", "medium")
                if sclass not in SIZE_CLASSES:
                    sclass = "medium"
                det = Detection(
                    label=str(obj.get("label", "object")),
                    bbox_px=(x, y, bw, bh),
                    confidence=max(0.0, min(1.0, conf)),
                    estimated_size_class=sclass,
                    extras={"detect_ms": round(latency_ms, 1), "raw": obj},
                )
                out.append(det)
            except Exception as e:
                log.warning("skipping malformed VLM object: %s (%r)", e, obj)
        # store latency on the first detection's extras (or as side-channel via top-level)
        if not out:
            log.info("VLM detected 0 objects (latency %.0f ms)", latency_ms)
        return out


# ----------------------------------------------------------------------
# Fixture detector — offline / tests
# ----------------------------------------------------------------------


class FixtureDetector(Detector):
    """Generates synthetic detections by back-projecting world-space fixture
    objects into the current camera view. Lets us exercise the perception
    pipeline end-to-end without a real camera.
    """

    def __init__(self, fixtures_path: pathlib.Path):
        self.fixtures_path = fixtures_path
        self._fixtures = self._load()

    def _load(self) -> List[Dict[str, Any]]:
        if not self.fixtures_path.exists():
            return []
        with open(self.fixtures_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("objects", [])

    def reload(self):
        self._fixtures = self._load()

    def _match(self, obj: Dict[str, Any], query: str) -> bool:
        q = query.strip().lower()
        if not q:
            return True
        label = str(obj.get("label", "")).lower()
        if q in label or label in q:
            return True
        for alias in obj.get("aliases", []):
            a = str(alias).lower()
            if q in a or a in q:
                return True
        return False

    def detect(self, frame: np.ndarray, query: str, image_meta: Dict[str, Any]) -> List[Detection]:
        """Match by query, then back-project world point → pixel via image_meta's
        T_cam_base and K. image_meta MUST provide 'T_cam_base' (4x4) and 'K' (3x3).
        If not provided (e.g. legacy callers), fall back to a centered bbox.
        """
        t0 = time.time()
        out: List[Detection] = []
        h, w = frame.shape[:2]
        T_cam_base = image_meta.get("T_cam_base")
        K = image_meta.get("K")
        for obj in self._fixtures:
            if not self._match(obj, query):
                continue
            world_xyz = obj["world_xyz_mm"]
            radius = float(obj.get("radius_mm", 25.0))
            if T_cam_base is not None and K is not None:
                u, v, depth_cam = _project(T_cam_base, K, world_xyz)
                if depth_cam <= 0:
                    continue  # behind camera
                # bbox apparent size: diameter * focal / depth
                fx = float(K[0][0])
                px_diameter = max(8.0, 2.0 * radius * fx / max(50.0, depth_cam))
                bx = u - px_diameter / 2
                by = v - px_diameter / 2
                # Skip if entirely outside frame
                if bx + px_diameter < 0 or by + px_diameter < 0 or bx > w or by > h:
                    continue
                # Clamp slightly so it stays in-frame (mark occluded if clipped)
                clipped = (bx < 0 or by < 0 or bx + px_diameter > w or by + px_diameter > h)
                bbox = (max(0.0, bx), max(0.0, by),
                        min(px_diameter, float(w) - max(0.0, bx)),
                        min(px_diameter, float(h) - max(0.0, by)))
                extras = {"detect_ms": round((time.time() - t0) * 1000.0, 1),
                          "fixture_world_xyz_mm": list(world_xyz),
                          "fixture_radius_mm": radius,
                          "clipped": clipped}
            else:
                # No camera pose given → fallback to centered bbox
                bbox = (w * 0.5 - 40, h * 0.5 - 40, 80.0, 80.0)
                extras = {"detect_ms": round((time.time() - t0) * 1000.0, 1),
                          "fixture_world_xyz_mm": list(world_xyz),
                          "fixture_radius_mm": radius,
                          "fallback_centered_bbox": True}
            out.append(Detection(
                label=obj["label"],
                bbox_px=bbox,
                confidence=float(obj.get("confidence", 0.8)),
                estimated_size_class=obj.get("size_class", "medium"),
                extras=extras,
            ))
        return out


def _project(T_cam_base, K, world_xyz):
    """Project a world (base) point into pixel coords using T_cam_base (4x4) and K (3x3).
    Returns (u, v, depth_in_cam_frame)."""
    x, y, z = world_xyz
    Xc = T_cam_base[0][0]*x + T_cam_base[0][1]*y + T_cam_base[0][2]*z + T_cam_base[0][3]
    Yc = T_cam_base[1][0]*x + T_cam_base[1][1]*y + T_cam_base[1][2]*z + T_cam_base[1][3]
    Zc = T_cam_base[2][0]*x + T_cam_base[2][1]*y + T_cam_base[2][2]*z + T_cam_base[2][3]
    if Zc <= 1e-6:
        return (0.0, 0.0, Zc)
    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]
    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy
    return (u, v, Zc)
