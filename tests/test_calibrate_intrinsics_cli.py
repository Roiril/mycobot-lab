"""Tests for scripts/calibrate_intrinsics.py CLI helpers.

Run: python -m unittest tests.test_calibrate_intrinsics_cli
"""
import json
import math
import os
import pathlib
import sys
import tempfile
import unittest
import importlib.util

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "calibrate_intrinsics", ROOT / "scripts" / "calibrate_intrinsics.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestParseHandEye(unittest.TestCase):
    def setUp(self):
        self.mod = _load_module()

    def test_valid_identity(self):
        T = self.mod.parse_hand_eye("30,0,0,0,0,0")
        self.assertEqual(len(T), 4)
        self.assertAlmostEqual(T[0][3], 30.0)
        self.assertAlmostEqual(T[1][3], 0.0)
        self.assertAlmostEqual(T[2][3], 0.0)
        # rotation = identity
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(T[i][j], 1.0 if i == j else 0.0, places=6)

    def test_valid_with_rotation(self):
        T = self.mod.parse_hand_eye("0,25,0,0,30,0")
        self.assertAlmostEqual(T[1][3], 25.0)
        # Ry(30) → R[0][0] = cos30, R[2][0] = -sin30
        self.assertAlmostEqual(T[0][0], math.cos(math.radians(30)), places=5)
        self.assertAlmostEqual(T[2][0], -math.sin(math.radians(30)), places=5)

    def test_wrong_arity_5_elements(self):
        with self.assertRaises(ValueError):
            self.mod.parse_hand_eye("1,2,3,4,5")

    def test_wrong_arity_7_elements(self):
        with self.assertRaises(ValueError):
            self.mod.parse_hand_eye("1,2,3,4,5,6,7")

    def test_non_numeric(self):
        with self.assertRaises(ValueError):
            self.mod.parse_hand_eye("a,b,c,d,e,f")

    def test_nan(self):
        with self.assertRaises(ValueError):
            self.mod.parse_hand_eye("nan,0,0,0,0,0")

    def test_inf(self):
        with self.assertRaises(ValueError):
            self.mod.parse_hand_eye("inf,0,0,0,0,0")


class TestPlaceholderHeuristic(unittest.TestCase):
    def setUp(self):
        self.mod = _load_module()

    def test_default_placeholder_matches(self):
        entry = {"hand_eye_T_ee_cam": [
            [1.0, 0.0, 0.0, 30.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]}
        self.assertTrue(self.mod._looks_like_default_placeholder(entry))

    def test_real_value_not_placeholder(self):
        entry = {"hand_eye_T_ee_cam": [
            [0.866, 0.0, 0.5, 12.0],
            [0.0,   1.0, 0.0, 5.0],
            [-0.5,  0.0, 0.866, 40.0],
            [0.0,   0.0, 0.0, 1.0],
        ]}
        self.assertFalse(self.mod._looks_like_default_placeholder(entry))


class TestUpdateCalibrationFile(unittest.TestCase):
    """Smoke-test update_calibration_file via a temp ROOT directory."""

    def setUp(self):
        self.mod = _load_module()
        self.tmpdir = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.tmpdir.name)
        (self.root / "data").mkdir()
        # repoint module ROOT to temp
        self._orig_root = self.mod.ROOT
        self.mod.ROOT = self.root

    def tearDown(self):
        self.mod.ROOT = self._orig_root
        self.tmpdir.cleanup()

    def _seed(self, entry: dict, workspace: dict | None = None):
        calib = {"version": 1, "cameras": {"wrist": entry},
                 "workspace": workspace or {"table_z_mm": 0.0, "table_z_uncertainty_mm": 5.0}}
        with open(self.root / "data" / "calibration.json", "w", encoding="utf-8") as f:
            json.dump(calib, f)

    def _read(self):
        with open(self.root / "data" / "calibration.json", "r", encoding="utf-8") as f:
            return json.load(f)

    def test_hand_eye_omitted_preserves_placeholder(self):
        self._seed({
            "index": 3, "role": "wrist", "resolution": [640, 480],
            "intrinsics": {"K": [[1,0,0],[0,1,0],[0,0,1]], "dist": [0]*5},
            "hand_eye_T_ee_cam": [[1,0,0,30],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
            "placeholder": True,
        })
        self.mod.update_calibration_file(
            "wrist",
            [[500,0,320],[0,500,240],[0,0,1]], [0]*5, [640,480],
            hand_eye_T=None, hand_eye_explicit=False,
            table_z_mm=None, table_z_unc_mm=None,
        )
        calib = self._read()
        self.assertTrue(calib["cameras"]["wrist"]["placeholder"])

    def test_hand_eye_explicit_clears_placeholder(self):
        self._seed({
            "index": 3, "role": "wrist", "resolution": [640, 480],
            "intrinsics": {"K": [[1,0,0],[0,1,0],[0,0,1]], "dist": [0]*5},
            "hand_eye_T_ee_cam": [[1,0,0,30],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
            "placeholder": True,
        })
        T = self.mod.parse_hand_eye("12,5,40,0,30,0")
        self.mod.update_calibration_file(
            "wrist",
            [[500,0,320],[0,500,240],[0,0,1]], [0]*5, [640,480],
            hand_eye_T=T, hand_eye_explicit=True,
            table_z_mm=None, table_z_unc_mm=None,
        )
        calib = self._read()
        self.assertFalse(calib["cameras"]["wrist"]["placeholder"])
        self.assertAlmostEqual(calib["cameras"]["wrist"]["hand_eye_T_ee_cam"][0][3], 12.0)

    def test_table_z_mm_updates_workspace(self):
        self._seed({
            "index": 3, "role": "wrist", "resolution": [640, 480],
            "intrinsics": {"K": [[1,0,0],[0,1,0],[0,0,1]], "dist": [0]*5},
            "hand_eye_T_ee_cam": [[1,0,0,30],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
            "placeholder": True,
        })
        self.mod.update_calibration_file(
            "wrist",
            [[500,0,320],[0,500,240],[0,0,1]], [0]*5, [640,480],
            hand_eye_T=None, hand_eye_explicit=False,
            table_z_mm=-15.0, table_z_unc_mm=None,
        )
        calib = self._read()
        self.assertAlmostEqual(calib["workspace"]["table_z_mm"], -15.0)
        # placeholder still preserved
        self.assertTrue(calib["cameras"]["wrist"]["placeholder"])

    def test_atomic_write_no_tmp_leftover(self):
        self._seed({"index": 3, "role": "wrist", "resolution": [640, 480],
                    "intrinsics": {"K": [[1,0,0],[0,1,0],[0,0,1]], "dist": [0]*5},
                    "placeholder": False})
        self.mod.update_calibration_file(
            "wrist", [[1,0,0],[0,1,0],[0,0,1]], [0]*5, [640,480],
            hand_eye_T=None, hand_eye_explicit=False,
            table_z_mm=None, table_z_unc_mm=None,
        )
        leftovers = list((self.root / "data").glob("*.tmp"))
        self.assertEqual(leftovers, [])


if __name__ == "__main__":
    unittest.main()
