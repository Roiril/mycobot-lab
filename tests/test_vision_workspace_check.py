"""perceive() must filter out localized objects outside the workspace cube.

We construct a synthetic overhead-camera calibration + fixtures whose world
positions fall outside WORKSPACE_REACH_MAX_MM, then verify:
  - if ALL detections are out-of-workspace → returns OUT_OF_WORKSPACE
  - if some are inside → success path; oow ones excluded + diagnostics counts them
"""
import unittest, sys, pathlib, json, tempfile, math

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.vision_hub import VirtualVisionHub
from arm.constants import HOME_ANGLES, WORKSPACE_REACH_MAX_MM


def _overhead_calib(table_z=50.0):
    rx = math.pi
    cx_, sx_ = math.cos(rx), math.sin(rx)
    T_base_cam = [
        [1.0, 0.0,  0.0, 0.0],
        [0.0, cx_, -sx_, 0.0],
        [0.0, sx_,  cx_, 800.0],   # 800mm above table to capture wider FoV
        [0.0, 0.0,  0.0, 1.0],
    ]
    return {
        "version": 1,
        "cameras": {
            "overhead": {
                "index": 1, "role": "overhead",
                "resolution": [640, 480],
                "intrinsics": {
                    # Wide-angle fx so we can see far-off table coords in-frame
                    "K": [[200.0, 0.0, 320.0], [0.0, 200.0, 240.0], [0.0, 0.0, 1.0]],
                    "dist": [0, 0, 0, 0, 0]
                },
                "T_base_cam": T_base_cam,
            }
        },
        "workspace": {"table_z_mm": table_z, "table_z_uncertainty_mm": 1.0}
    }


class TestWorkspaceCheck(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix="vision_ws_"))
        self.calib_path = self.tmp / "calibration.json"
        self.fixtures_path = self.tmp / "objects.json"

    def _write(self, calib, fixtures):
        with open(self.calib_path, "w", encoding="utf-8") as f:
            json.dump(calib, f)
        with open(self.fixtures_path, "w", encoding="utf-8") as f:
            json.dump(fixtures, f)

    def test_all_out_of_workspace_returns_oow_error(self):
        # Place fixture far beyond reach (700mm out — > 380mm reach limit).
        far = WORKSPACE_REACH_MAX_MM + 320
        fixtures = {"objects": [
            # Use radius large enough that bbox area > MIN_BBOX_AREA_PX even at this depth
            {"label": "コップ", "world_xyz_mm": [far, 0, 50],
             "radius_mm": 40, "size_class": "large", "confidence": 0.9},
        ]}
        self._write(_overhead_calib(), fixtures)
        vh = VirtualVisionHub(self.calib_path, self.fixtures_path)
        result = vh.perceive(query="コップ", cameras=["overhead"],
                             confidence_threshold=0.5, angles_deg=HOME_ANGLES)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"]["code"], "OUT_OF_WORKSPACE")
        self.assertIn("retry_hints", result["error"])
        self.assertGreater(len(result["error"]["retry_hints"]), 0)
        sample = result["error"]["diagnostics"]["sample"][0]
        self.assertTrue(sample.get("out_of_workspace"))

    def test_mixed_inside_and_outside_returns_only_inside(self):
        # One reachable, one far away.
        far = WORKSPACE_REACH_MAX_MM + 200
        fixtures = {"objects": [
            # Near: radius small enough that depth_uncertainty (~2r/cos) stays < 50mm
            {"label": "近いコップ", "aliases": ["cup"],
             "world_xyz_mm": [80, 30, 50],
             "radius_mm": 20, "size_class": "medium", "confidence": 0.9},
            # Far: radius large enough that bbox area > MIN_BBOX_AREA_PX at this depth
            {"label": "遠いコップ", "aliases": ["cup"],
             "world_xyz_mm": [far, 0, 50],
             "radius_mm": 40, "size_class": "large", "confidence": 0.85},
        ]}
        self._write(_overhead_calib(), fixtures)
        vh = VirtualVisionHub(self.calib_path, self.fixtures_path)
        result = vh.perceive(query="cup", cameras=["overhead"],
                             confidence_threshold=0.5, angles_deg=HOME_ANGLES)
        self.assertTrue(result["ok"], f"perceive failed: {result}")
        # Only the near one survives
        for obj in result["objects"]:
            self.assertFalse(obj.get("out_of_workspace"))
        # diagnostics reports the excluded one
        self.assertEqual(result["diagnostics"].get("out_of_workspace_excluded"), 1)


if __name__ == "__main__":
    unittest.main()
