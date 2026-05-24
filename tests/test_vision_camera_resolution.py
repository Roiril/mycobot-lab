"""CameraRegistry resolution-mismatch detection + list() observability fields."""
import unittest
import sys
import pathlib
import json
import tempfile

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.vision.camera import CameraRegistry


def _write_calib(tmp: pathlib.Path, declared_w=640, declared_h=480):
    calib_path = tmp / "calibration.json"
    calib = {
        "version": 1,
        "cameras": {
            "wrist": {
                "index": 0,
                "role": "wrist",
                "resolution": [declared_w, declared_h],
                "intrinsics": {
                    "K": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
                    "dist": [0, 0, 0, 0, 0]
                },
                "hand_eye_T_ee_cam": [
                    [1.0, 0.0, 0.0, 50.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ],
                "calibrated_at": "2026-05-24T10:00:00+09:00"
            }
        },
        "workspace": {"table_z_mm": 0.0, "table_z_uncertainty_mm": 1.0}
    }
    with open(calib_path, "w", encoding="utf-8") as f:
        json.dump(calib, f)
    return calib_path


class TestResolutionMismatch(unittest.TestCase):
    def test_mismatch_records_actual_size(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="vcamres_"))
        calib_path = _write_calib(tmp, declared_w=1280, declared_h=720)
        # Offline mode bypasses cv2.VideoCapture; _offline_frame uses cam.resolution
        # so it would always match. We force mismatch by stubbing _open() with a
        # fake VideoCapture that yields a smaller frame.
        reg = CameraRegistry(calib_path, offline=False)

        class FakeCap:
            def __init__(self): self._n = 0
            def read(self):
                # actual frame 640x480 vs declared 1280x720
                return True, np.zeros((480, 640, 3), dtype=np.uint8)
            def release(self): pass

        fake = FakeCap()
        reg._caps["wrist"] = fake  # bypass _open path
        frame = reg.get_frame("wrist")
        self.assertIsNotNone(frame)
        cam = reg.get("wrist")
        self.assertEqual(cam.frame_size_actual, (640, 480))

        listing = reg.list()
        self.assertEqual(len(listing), 1)
        entry = listing[0]
        # New observability fields present
        self.assertIn("intrinsics_summary", entry)
        self.assertIsNotNone(entry["intrinsics_summary"])
        self.assertAlmostEqual(entry["intrinsics_summary"]["fx"], 500.0)
        self.assertIn("is_open", entry)
        self.assertTrue(entry["is_open"])
        self.assertIn("last_frame_age_ms", entry)
        self.assertIsNotNone(entry["last_frame_age_ms"])
        self.assertIn("hand_eye_present", entry)
        self.assertTrue(entry["hand_eye_present"])
        self.assertIn("calibrated_at", entry)
        self.assertEqual(entry["calibrated_at"], "2026-05-24T10:00:00+09:00")
        self.assertEqual(entry["frame_size_actual"], [640, 480])

    def test_match_no_actual_recorded_diff(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="vcamres_match_"))
        calib_path = _write_calib(tmp, declared_w=640, declared_h=480)
        reg = CameraRegistry(calib_path, offline=False)

        class FakeCap:
            def read(self):
                return True, np.zeros((480, 640, 3), dtype=np.uint8)
            def release(self): pass

        reg._caps["wrist"] = FakeCap()
        reg.get_frame("wrist")
        cam = reg.get("wrist")
        # actual matches declared
        self.assertEqual(cam.frame_size_actual, (640, 480))


if __name__ == "__main__":
    unittest.main()
