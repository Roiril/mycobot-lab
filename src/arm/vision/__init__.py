"""Vision package — VLM-based object detection + 3D localization on table plane.

Public surface (Phase 1):
  - Detection, Detector, ClaudeVLMDetector, FixtureDetector
  - Camera, CameraRegistry
  - localize_on_table, pixel_to_ray, intersect_plane, estimate_radius
  - T_base_ee, T_base_cam_wrist transforms
"""
from .detector import Detection, Detector, ClaudeVLMDetector, FixtureDetector
from .camera import Camera, CameraRegistry
from .localizer import (
    pixel_to_ray, intersect_plane, estimate_radius, localize_on_table,
    SIZE_CLASS_MAX_RADIUS_MM,
)
from .transforms import (
    T_base_ee, T_base_cam_wrist, identity4, mat_mul, invert,
    xyz_rpy_to_T, T_to_xyz_rpy, apply, apply_rot,
)

__all__ = [
    "Detection", "Detector", "ClaudeVLMDetector", "FixtureDetector",
    "Camera", "CameraRegistry",
    "pixel_to_ray", "intersect_plane", "estimate_radius", "localize_on_table",
    "SIZE_CLASS_MAX_RADIUS_MM",
    "T_base_ee", "T_base_cam_wrist", "identity4", "mat_mul", "invert",
    "xyz_rpy_to_T", "T_to_xyz_rpy", "apply", "apply_rot",
]
