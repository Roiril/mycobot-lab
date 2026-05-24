"""Observability tests for VisionHub diagnostics.

Covers:
  - success-path diagnostics (angles_at_capture, tip_at_capture, timestamp_iso,
    cameras_used) populated by VirtualVisionHub.perceive
  - localize echo (objects[i].localize has ray_dir_base / cos_normal /
    ray_origin_base / bbox_center_px / depth_along_ray_mm)
  - error-path diagnostics: OBJECT_NOT_FOUND includes base diagnostics, and a
    frame is saved under data/perceive_log/ with diagnostics.frame_path set
  - last_call_diagnostics existence on FixtureDetector (vlm.* fields are echoed
    but raw_text is empty / model='fixture')
"""
import unittest
import sys
import pathlib
import json
import tempfile
import math

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.vision_hub import VirtualVisionHub
from arm.constants import HOME_ANGLES


def _build_overhead_setup(tmp: pathlib.Path):
    """Reusable: overhead cam 500mm above table looking straight down."""
    calib_path = tmp / "calibration.json"
    fixtures_path = tmp / "objects.json"
    D = math.pi / 180
    rx = 180 * D
    cx_, sx_ = math.cos(rx), math.sin(rx)
    T_base_cam = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cx_, -sx_, 0.0],
        [0.0, sx_, cx_, 500.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    calib = {
        "version": 1,
        "cameras": {
            "overhead": {
                "index": 1, "role": "overhead",
                "resolution": [640, 480],
                "intrinsics": {
                    "K": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
                    "dist": [0, 0, 0, 0, 0]
                },
                "T_base_cam": T_base_cam,
                "calibrated_at": "2026-05-24T12:00:00+09:00",
            }
        },
        "workspace": {"table_z_mm": 50.0, "table_z_uncertainty_mm": 1.0}
    }
    with open(calib_path, "w", encoding="utf-8") as f:
        json.dump(calib, f)
    fixtures = {"objects": [
        {"label": "コップ", "world_xyz_mm": [50, 30, 50],
         "radius_mm": 10, "size_class": "small", "confidence": 0.9}
    ]}
    with open(fixtures_path, "w", encoding="utf-8") as f:
        json.dump(fixtures, f)
    return calib_path, fixtures_path


class TestPerceiveDiagnosticsSuccess(unittest.TestCase):
    def test_success_diagnostics_present(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="vdiag_succ_"))
        calib_path, fixtures_path = _build_overhead_setup(tmp)
        vh = VirtualVisionHub(calib_path, fixtures_path)
        res = vh.perceive(query="コップ", cameras=["overhead"],
                          confidence_threshold=0.5, angles_deg=HOME_ANGLES)
        self.assertTrue(res["ok"], f"perceive failed: {res}")
        diag = res["diagnostics"]
        self.assertIn("angles_at_capture", diag)
        self.assertEqual(len(diag["angles_at_capture"]), 6)
        self.assertIn("tip_at_capture", diag)
        self.assertIn("xyz", diag["tip_at_capture"])
        self.assertEqual(len(diag["tip_at_capture"]["xyz"]), 3)
        self.assertIn("timestamp_iso", diag)
        self.assertIn("cameras_used", diag)
        self.assertEqual(diag["cameras_used"], ["overhead"])
        # vlm echo (FixtureDetector still populates this with parse_ok=True)
        self.assertIn("vlm", diag)
        self.assertEqual(diag["vlm"]["vlm_model"], "fixture")
        self.assertTrue(diag["vlm"]["parse_ok"])

    def test_localize_intermediate_values_echoed(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="vdiag_loc_"))
        calib_path, fixtures_path = _build_overhead_setup(tmp)
        vh = VirtualVisionHub(calib_path, fixtures_path)
        res = vh.perceive(query="コップ", cameras=["overhead"],
                          confidence_threshold=0.5, angles_deg=HOME_ANGLES)
        self.assertTrue(res["ok"], f"perceive failed: {res}")
        objs = res["diagnostics"].get("objects")
        self.assertIsNotNone(objs)
        self.assertGreater(len(objs), 0)
        loc = objs[0]["localize"]
        self.assertIn("ray_origin_base", loc)
        self.assertEqual(len(loc["ray_origin_base"]), 3)
        self.assertIn("ray_dir_base", loc)
        self.assertEqual(len(loc["ray_dir_base"]), 3)
        # ray dir should be unit length
        d = loc["ray_dir_base"]
        n = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        self.assertAlmostEqual(n, 1.0, places=4)
        self.assertIn("cos_normal", loc)
        # overhead camera straight down → cos_normal close to 1
        self.assertGreater(loc["cos_normal"], 0.95)
        self.assertIn("bbox_center_px", loc)
        self.assertEqual(len(loc["bbox_center_px"]), 2)


class TestPerceiveDiagnosticsError(unittest.TestCase):
    def test_object_not_found_includes_base_diagnostics_and_frame_path(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="vdiag_err_"))
        calib_path, fixtures_path = _build_overhead_setup(tmp)
        vh = VirtualVisionHub(calib_path, fixtures_path)
        # Use a query that doesn't match the fixture label → OBJECT_NOT_FOUND
        res = vh.perceive(query="絶対に存在しないxyz",
                          cameras=["overhead"],
                          confidence_threshold=0.5,
                          angles_deg=HOME_ANGLES)
        self.assertFalse(res["ok"])
        self.assertEqual(res["error"]["code"], "OBJECT_NOT_FOUND")
        diag = res["error"]["diagnostics"]
        self.assertIn("angles_at_capture", diag)
        self.assertIn("tip_at_capture", diag)
        self.assertIn("timestamp_iso", diag)
        self.assertIn("cameras_used", diag)
        # Frame should have been saved (VirtualVisionHub produces synthetic frame)
        self.assertIn("frame_path", diag)
        # File should exist under data/perceive_log/ (frame_path may be relative)
        fp = diag["frame_path"]
        candidates = [pathlib.Path(fp), tmp.parent / fp, ROOT / fp]
        self.assertTrue(any(p.exists() for p in candidates),
                        f"saved frame not found at any of {candidates}")


if __name__ == "__main__":
    unittest.main()
