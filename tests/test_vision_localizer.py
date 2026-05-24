"""Pure-function tests for vision localizer + transforms."""
import unittest, sys, pathlib, math

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.vision.localizer import (
    pixel_to_ray, intersect_plane, estimate_radius, localize_on_table,
    SIZE_CLASS_MAX_RADIUS_MM,
)
from arm.vision.transforms import (
    identity4, xyz_rpy_to_T, invert, mat_mul, apply, T_to_xyz_rpy,
)


K_DEFAULT = [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
DIST_ZERO = [0, 0, 0, 0, 0]


class TestPixelToRay(unittest.TestCase):
    def test_center_pixel_yields_forward_ray(self):
        origin, d = pixel_to_ray(K_DEFAULT, DIST_ZERO, (320.0, 240.0))
        self.assertAlmostEqual(d[0], 0.0, places=6)
        self.assertAlmostEqual(d[1], 0.0, places=6)
        self.assertAlmostEqual(d[2], 1.0, places=6)

    def test_offset_pixel_tilts_ray(self):
        # right of center → +x in cam frame
        _, d = pixel_to_ray(K_DEFAULT, DIST_ZERO, (420.0, 240.0))
        self.assertGreater(d[0], 0)
        self.assertAlmostEqual(d[1], 0.0, places=6)

    def test_ray_is_unit(self):
        _, d = pixel_to_ray(K_DEFAULT, DIST_ZERO, (100.0, 50.0))
        n = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        self.assertAlmostEqual(n, 1.0, places=6)


class TestIntersectPlane(unittest.TestCase):
    def test_ray_straight_down_hits_plane(self):
        # origin (0,0,200), dir (0,0,-1) → plane z=0 → point (0,0,0)
        pt = intersect_plane((0, 0, 200), (0, 0, -1), 0.0)
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[2], 0.0)

    def test_parallel_ray_returns_none(self):
        self.assertIsNone(intersect_plane((0, 0, 100), (1, 0, 0), 0.0))

    def test_ray_pointing_away_returns_none(self):
        # camera at z=200 looking up; plane z=0 is behind → no intersection
        self.assertIsNone(intersect_plane((0, 0, 200), (0, 0, 1), 0.0))


class TestEstimateRadius(unittest.TestCase):
    def test_caps_at_size_class_max(self):
        # huge bbox would yield huge radius; size_class small caps it
        r = estimate_radius("small", (0, 0, 1000, 1000), focal_px=500, depth_mm=300)
        self.assertLessEqual(r, SIZE_CLASS_MAX_RADIUS_MM["small"])

    def test_default_when_inputs_invalid(self):
        r = estimate_radius("medium", (0, 0, 50, 50), focal_px=0, depth_mm=0)
        self.assertGreater(r, 0)

    def test_unknown_class_falls_back_to_medium(self):
        r = estimate_radius("unknown", (0, 0, 50, 50), focal_px=500, depth_mm=300)
        self.assertLessEqual(r, SIZE_CLASS_MAX_RADIUS_MM["medium"])


class TestLocalizeOnTable(unittest.TestCase):
    def test_camera_above_table_projects_center_to_origin(self):
        """Camera at (0,0,400) looking down at table z=0. Bbox center → world (0,0,0)."""
        from arm.vision.camera import Camera
        cam = Camera(id="test", index=0, role="wrist", resolution=(640, 480),
                     intrinsics={"K": K_DEFAULT, "dist": DIST_ZERO})
        # T_base_cam: camera at (0,0,400), z-axis pointing down (rx=180)
        T = xyz_rpy_to_T(0, 0, 400, 180, 0, 0)
        det = {"bbox_px": (300, 220, 40, 40), "estimated_size_class": "medium"}
        res = localize_on_table(det, cam, T, table_z_mm=0.0)
        self.assertIn("xyz_base", res)
        self.assertAlmostEqual(res["xyz_base"][0], 0.0, delta=2.0)
        self.assertAlmostEqual(res["xyz_base"][1], 0.0, delta=2.0)
        self.assertAlmostEqual(res["xyz_base"][2], 0.0, delta=0.5)
        self.assertGreater(res["depth_uncertainty_mm"], 0)


class TestTransforms(unittest.TestCase):
    def test_invert_roundtrip(self):
        T = xyz_rpy_to_T(100, 50, 200, 10, 20, 30)
        I = mat_mul(T, invert(T))
        for i in range(4):
            for j in range(4):
                expected = 1.0 if i == j else 0.0
                self.assertAlmostEqual(I[i][j], expected, places=5)

    def test_xyzrpy_roundtrip(self):
        T = xyz_rpy_to_T(50, -20, 150, 15, 25, 35)
        x, y, z, rx, ry, rz = T_to_xyz_rpy(T)
        self.assertAlmostEqual(x, 50, places=4)
        self.assertAlmostEqual(y, -20, places=4)
        self.assertAlmostEqual(z, 150, places=4)
        # Note: Euler angles may not roundtrip exactly near gimbal lock,
        # but typical values should be close
        self.assertAlmostEqual(rx, 15, places=3)
        self.assertAlmostEqual(ry, 25, places=3)
        self.assertAlmostEqual(rz, 35, places=3)

    def test_apply_translation(self):
        T = xyz_rpy_to_T(10, 20, 30, 0, 0, 0)
        p = apply(T, (1, 2, 3))
        self.assertAlmostEqual(p[0], 11)
        self.assertAlmostEqual(p[1], 22)
        self.assertAlmostEqual(p[2], 33)


if __name__ == "__main__":
    unittest.main()
