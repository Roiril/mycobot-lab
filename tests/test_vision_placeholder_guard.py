"""Placeholder-only calibration must not silently emit motion-bound coords.

- Default perceive() rejects with CALIBRATION_PLACEHOLDER_ONLY + terminal=true
- Caller can opt in via allow_uncalibrated=True; each returned object then
  carries uncalibrated=true
"""
import unittest, sys, pathlib, json, tempfile, math

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.vision_hub import VirtualVisionHub
from arm.constants import HOME_ANGLES


class TestPlaceholderGuard(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix="vision_pholder_"))
        self.calib_path = self.tmp / "calibration.json"
        self.fixtures_path = self.tmp / "objects.json"
        # Overhead-style camera but flagged placeholder=true
        rx = math.pi
        cx_, sx_ = math.cos(rx), math.sin(rx)
        T_base_cam = [
            [1.0, 0.0,  0.0, 0.0],
            [0.0, cx_, -sx_, 0.0],
            [0.0, sx_,  cx_, 500.0],
            [0.0, 0.0,  0.0, 1.0],
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
                    "placeholder": True,
                }
            },
            "workspace": {"table_z_mm": 50.0, "table_z_uncertainty_mm": 1.0}
        }
        with open(self.calib_path, "w", encoding="utf-8") as f:
            json.dump(calib, f)
        fixtures = {"objects": [
            {"label": "コップ", "world_xyz_mm": [50, 30, 50],
             "radius_mm": 10, "size_class": "small", "confidence": 0.9},
        ]}
        with open(self.fixtures_path, "w", encoding="utf-8") as f:
            json.dump(fixtures, f)

    def test_default_rejects_placeholder_only(self):
        vh = VirtualVisionHub(self.calib_path, self.fixtures_path)
        result = vh.perceive(query="コップ", cameras=["overhead"],
                             confidence_threshold=0.5, angles_deg=HOME_ANGLES)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"]["code"], "CALIBRATION_PLACEHOLDER_ONLY")
        self.assertTrue(result["error"].get("terminal"))
        # Should include the allow_uncalibrated retry hint
        actions = [h["action"] for h in result["error"]["retry_hints"]]
        self.assertIn("allow_uncalibrated_explicit", actions)

    def test_allow_uncalibrated_passes_through_with_flag(self):
        vh = VirtualVisionHub(self.calib_path, self.fixtures_path)
        result = vh.perceive(query="コップ", cameras=["overhead"],
                             confidence_threshold=0.5, angles_deg=HOME_ANGLES,
                             allow_uncalibrated=True)
        self.assertTrue(result["ok"], f"expected pass-through, got: {result}")
        self.assertGreater(len(result["objects"]), 0)
        for obj in result["objects"]:
            self.assertTrue(obj.get("uncalibrated"),
                            f"object missing uncalibrated flag: {obj}")

    def test_cameras_list_exposes_placeholder(self):
        vh = VirtualVisionHub(self.calib_path, self.fixtures_path)
        listing = vh.registry.list()
        self.assertEqual(len(listing), 1)
        self.assertTrue(listing[0]["placeholder"])
        self.assertFalse(listing[0]["calibrated"])


if __name__ == "__main__":
    unittest.main()
