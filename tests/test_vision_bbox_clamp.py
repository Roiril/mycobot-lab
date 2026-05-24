"""Detector and localizer must reject / clamp malformed bboxes.

Exercises `_validate_and_clamp_bbox` directly and the localizer's defensive
center-in-frame guard.
"""
import unittest, sys, pathlib, math

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.vision.detector import _validate_and_clamp_bbox
from arm.vision.localizer import localize_on_table
from arm.vision.camera import Camera


class TestBboxValidation(unittest.TestCase):
    def test_negative_size_rejected(self):
        self.assertIsNone(_validate_and_clamp_bbox(10, 10, -5, 20, 640, 480))
        self.assertIsNone(_validate_and_clamp_bbox(10, 10, 20, -5, 640, 480))
        self.assertIsNone(_validate_and_clamp_bbox(10, 10, 0, 0, 640, 480))

    def test_nan_inf_rejected(self):
        self.assertIsNone(_validate_and_clamp_bbox(float("nan"), 10, 5, 5, 640, 480))
        self.assertIsNone(_validate_and_clamp_bbox(10, 10, float("inf"), 5, 640, 480))

    def test_center_outside_frame_rejected(self):
        # x=700, w=10 → center 705 > 640 (frame_w)
        self.assertIsNone(_validate_and_clamp_bbox(700, 100, 10, 10, 640, 480))
        # Negative center
        self.assertIsNone(_validate_and_clamp_bbox(-100, 100, 10, 10, 640, 480))

    def test_partially_oob_gets_clamped(self):
        # bbox extends past right edge but center (550) is inside (< 640)
        out = _validate_and_clamp_bbox(500, 200, 100, 50, 640, 480)
        self.assertIsNotNone(out)
        x, y, w, h = out
        self.assertGreaterEqual(x, 0)
        self.assertLessEqual(x + w, 640.0 + 1e-6)
        self.assertGreater(w, 0)

    def test_extends_past_right_edge_clamps_width(self):
        # x=580 w=100 → extends to 680. Center 630 is inside frame_w=640.
        out = _validate_and_clamp_bbox(580, 200, 100, 50, 640, 480)
        self.assertIsNotNone(out)
        x, y, w, h = out
        self.assertLessEqual(x + w, 640.0 + 1e-6)

    def test_inside_box_unchanged(self):
        out = _validate_and_clamp_bbox(100, 100, 50, 50, 640, 480)
        self.assertEqual(out, (100.0, 100.0, 50.0, 50.0))


class TestLocalizerBboxGuard(unittest.TestCase):
    def _cam(self):
        return Camera(
            id="overhead", index=0, role="overhead", resolution=(640, 480),
            intrinsics={"K": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
                        "dist": [0, 0, 0, 0, 0]},
            T_base_cam_fixed=[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 800.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            calibrated=True, placeholder=False,
        )

    def test_nonfinite_bbox_returns_error(self):
        res = localize_on_table(
            {"bbox_px": (float("nan"), 240.0, 50.0, 50.0), "estimated_size_class": "small"},
            self._cam(), self._cam().T_base_cam_fixed, table_z_mm=0.0,
        )
        self.assertIn("error", res)
        self.assertEqual(res["error"], "BBOX_INVALID")

    def test_center_outside_frame_returns_error(self):
        # center way outside 640x480
        res = localize_on_table(
            {"bbox_px": (5000.0, 5000.0, 50.0, 50.0), "estimated_size_class": "small"},
            self._cam(), self._cam().T_base_cam_fixed, table_z_mm=0.0,
        )
        self.assertIn("error", res)
        self.assertEqual(res["error"], "BBOX_OUT_OF_FRAME")


if __name__ == "__main__":
    unittest.main()
