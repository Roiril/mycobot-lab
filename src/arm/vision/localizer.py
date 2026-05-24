"""Pixel → 3D localization on a table plane.

Given:
  - 2D bbox in image pixels
  - camera intrinsics K (3x3) + dist coefficients
  - camera extrinsics T_base_cam (4x4)
  - table plane in base frame (z = table_z_mm)

We:
  1. take bbox center pixel → undistort → normalized camera ray (origin + direction)
  2. transform ray to base frame
  3. intersect with z = table_z plane → world point on table
  4. estimate object radius from size_class + bbox apparent diameter
  5. estimate depth_uncertainty conservatively from object-height ambiguity

Pure functions, no hardware deps. numpy used for matrix ops (already a project dep).
"""
from __future__ import annotations
import math
from typing import Sequence, Tuple, Dict, Any, Optional

import numpy as np

from .transforms import apply, apply_rot


# Conservative max-radius per size class (mm). Phase 1 placeholders.
SIZE_CLASS_MAX_RADIUS_MM = {
    "small":  15.0,
    "medium": 30.0,
    "large":  60.0,
}

SIZE_CLASS_DEFAULT_RADIUS_MM = {
    "small":  10.0,
    "medium": 25.0,
    "large":  40.0,
}


def pixel_to_ray(K: Sequence[Sequence[float]],
                 dist: Sequence[float],
                 uv: Tuple[float, float]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Convert a pixel (u, v) to a normalized ray in the camera frame.

    Returns (origin_cam, dir_cam). origin is (0,0,0); dir is unit vector pointing
    forward (z>0). Dist coefficients used via cv2.undistortPoints when nonzero.

    K is the 3x3 intrinsic matrix; dist is [k1,k2,p1,p2,k3].
    """
    u, v = float(uv[0]), float(uv[1])
    K_np = np.array(K, dtype=np.float64)
    dist_np = np.array(dist, dtype=np.float64).reshape(-1)

    if dist_np.size > 0 and float(np.abs(dist_np).sum()) > 1e-9:
        # Use cv2.undistortPoints for proper distortion handling
        try:
            import cv2
            pts = np.array([[[u, v]]], dtype=np.float64)
            und = cv2.undistortPoints(pts, K_np, dist_np)  # returns normalized coords (x, y) with z=1
            x_n = float(und[0, 0, 0])
            y_n = float(und[0, 0, 1])
        except Exception:
            # Fallback: no distortion compensation
            fx, fy = K_np[0, 0], K_np[1, 1]
            cx, cy = K_np[0, 2], K_np[1, 2]
            x_n = (u - cx) / fx
            y_n = (v - cy) / fy
    else:
        fx, fy = K_np[0, 0], K_np[1, 1]
        cx, cy = K_np[0, 2], K_np[1, 2]
        x_n = (u - cx) / fx
        y_n = (v - cy) / fy

    # Build direction in camera frame (z forward, x right, y down — OpenCV convention)
    d = (x_n, y_n, 1.0)
    n = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
    d = (d[0] / n, d[1] / n, d[2] / n)
    return ((0.0, 0.0, 0.0), d)


def intersect_plane(origin: Tuple[float, float, float],
                    direction: Tuple[float, float, float],
                    plane_z: float) -> Optional[Tuple[float, float, float]]:
    """Intersect a ray (origin + t*direction) with the plane z = plane_z.

    Returns the 3D point or None if the ray is parallel / hits behind origin.
    All in the same frame (caller's responsibility).
    """
    dz = direction[2]
    if abs(dz) < 1e-9:
        return None
    t = (plane_z - origin[2]) / dz
    if t <= 0:
        return None  # plane is behind the camera
    return (origin[0] + t * direction[0],
            origin[1] + t * direction[1],
            origin[2] + t * direction[2])


def estimate_radius(size_class: str,
                    bbox_px: Tuple[float, float, float, float],
                    focal_px: float,
                    depth_mm: float) -> float:
    """Estimate physical radius (mm) from size_class category + bbox apparent diameter.

    The result is the *smaller* of:
      - size_class upper bound (so we never under-estimate clearance)
      - bbox-diameter geometric estimate at given depth: radius ≈ (bbox_min_side / 2) * depth / focal

    Phase 1 keeps this simple; Phase 2 may use shape priors per-class.
    """
    if size_class not in SIZE_CLASS_MAX_RADIUS_MM:
        size_class = "medium"
    cap = SIZE_CLASS_MAX_RADIUS_MM[size_class]
    default = SIZE_CLASS_DEFAULT_RADIUS_MM[size_class]
    _, _, w, h = bbox_px
    # smaller side → more conservative (object likely tall and narrow)
    min_side = max(1.0, min(float(w), float(h)))
    if focal_px <= 0 or depth_mm <= 0:
        return default
    geom_radius = (min_side * 0.5) * (depth_mm / focal_px)
    if geom_radius <= 0:
        return default
    return float(min(cap, max(5.0, geom_radius)))


def localize_on_table(detection: Dict[str, Any],
                      camera: "Camera",  # noqa: F821 — forward reference
                      T_base_cam: Sequence[Sequence[float]],
                      table_z_mm: float = 0.0) -> Dict[str, Any]:
    """Project a 2D detection onto the world table plane.

    Returns:
      {
        "xyz_base": [x,y,z],
        "radius_mm": float,
        "depth_uncertainty_mm": float,
        "source": "table_plane",
        "depth_along_ray_mm": float,
      }

    Or {"error": code, "reason": msg} if projection fails.
    """
    bbox = detection["bbox_px"]
    bx, by, bw, bh = bbox
    # Defensive: skip non-finite bboxes or bboxes whose center is outside the
    # image. We don't know frame_w/frame_h here, so we infer them from the
    # camera's declared resolution as a coarse bound (best-effort guard).
    for _v in (bx, by, bw, bh):
        if not math.isfinite(_v):
            return {"error": "BBOX_INVALID", "reason": "bbox 値が非有限"}
    if bw <= 0 or bh <= 0:
        return {"error": "BBOX_INVALID", "reason": "bbox 幅または高さが非正"}
    cu = bx + bw * 0.5
    cv = by + bh * 0.5
    try:
        fw, fh = camera.resolution
        if not (0 <= cu <= fw and 0 <= cv <= fh):
            return {"error": "BBOX_OUT_OF_FRAME",
                    "reason": f"bbox center ({cu:.0f},{cv:.0f}) が画像範囲 ({fw}x{fh}) 外"}
    except Exception:
        pass
    K = camera.intrinsics["K"]
    dist = camera.intrinsics.get("dist", [0, 0, 0, 0, 0])

    origin_c, dir_c = pixel_to_ray(K, dist, (cu, cv))
    # Transform to base
    origin_b = apply(T_base_cam, origin_c)
    dir_b = apply_rot(T_base_cam, dir_c)

    pt = intersect_plane(origin_b, dir_b, table_z_mm)
    if pt is None:
        return {"error": "RAY_MISSES_PLANE",
                "reason": "光線が table 平面と交差しない (カメラが下を向いていない or 平行)"}

    # depth along ray in base frame
    dx = pt[0] - origin_b[0]; dy = pt[1] - origin_b[1]; dz = pt[2] - origin_b[2]
    depth_along_ray = math.sqrt(dx*dx + dy*dy + dz*dz)

    # radius estimate using camera focal length (use fx)
    fx = float(K[0][0])
    radius_mm = estimate_radius(
        detection.get("estimated_size_class", "medium"),
        bbox, fx, depth_along_ray
    )

    # depth uncertainty: dominated by unknown object height above table.
    # We assumed the bbox center lies on the table plane. If the object actually has
    # height h, the true depth differs. Use 2*radius (object diameter) as a
    # conservative bound on h. Project that into base-frame mm via similar
    # triangles (Phase 1 simplification — Phase 2 should use ray-tilt geometry).
    height_guess = 2.0 * radius_mm
    # Bound: along the ray, height_guess maps to roughly height_guess / cos(angle_to_normal).
    # Approximate cos(angle) via |dir_b.z|; clamp so we don't blow up near-horizontal.
    cos_n = max(0.2, abs(dir_b[2]))
    depth_uncertainty_mm = height_guess / cos_n

    # Re-normalize dir_b (apply_rot for an orthonormal T should already be unit,
    # but numerical drift is possible).
    dn = math.sqrt(dir_b[0]*dir_b[0] + dir_b[1]*dir_b[1] + dir_b[2]*dir_b[2])
    if dn > 1e-9:
        dir_b_unit = (dir_b[0]/dn, dir_b[1]/dn, dir_b[2]/dn)
    else:
        dir_b_unit = dir_b
    # Plane normal points +z in base frame; cos angle between ray and -normal
    # (we want |dir.z| because rays going down should give cos close to 1).
    cos_normal = abs(dir_b_unit[2])

    return {
        "xyz_base": [pt[0], pt[1], pt[2]],
        "radius_mm": radius_mm,
        "depth_uncertainty_mm": depth_uncertainty_mm,
        "source": "table_plane",
        "depth_along_ray_mm": depth_along_ray,
        # observability — optional fields used by VisionHub diagnostics
        "ray_origin_base": [origin_b[0], origin_b[1], origin_b[2]],
        "ray_dir_base": [dir_b_unit[0], dir_b_unit[1], dir_b_unit[2]],
        "cos_normal": cos_normal,
        "bbox_center_px": [cu, cv],
    }
