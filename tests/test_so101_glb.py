"""Validate the generated per-link GLB assets (build_glb.py output).

Locks in reproducibility: every link named in manifest.json has a loadable,
non-empty GLB whose bounding box is a sane per-link size in mm. Skipped if
trimesh is missing or the GLBs haven't been built yet.

Geometry correctness (do the meshes form the right arm?) is established
independently by the MuJoCo render, which uses the same STL visuals + origins.

Run: python -m unittest tests.test_so101_glb
"""
import json
import unittest
import sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

GLB_DIR = ROOT / "src" / "robots" / "so101" / "meshes_glb"
MANIFEST = GLB_DIR / "manifest.json"

try:
    import trimesh  # noqa: F401
    _HAVE_TRIMESH = True
except ImportError:
    _HAVE_TRIMESH = False

# Link name -> kinematics.link_frames index it attaches to (for Phase-2 viewer).
EXPECTED_FRAME_INDEX = {
    "base_link": 0, "shoulder_link": 1, "upper_arm_link": 2,
    "lower_arm_link": 3, "wrist_link": 4, "gripper_link": 5,
}


@unittest.skipUnless(_HAVE_TRIMESH and MANIFEST.exists(),
                     "trimesh missing or GLBs not built (run build_glb.py)")
class TestGLBAssets(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads(MANIFEST.read_text())

    def test_all_arm_links_present(self):
        for link in EXPECTED_FRAME_INDEX:
            self.assertIn(link, self.manifest, msg=f"{link} missing from manifest")

    def test_glbs_load_and_are_sane(self):
        import trimesh
        for link, info in self.manifest.items():
            path = GLB_DIR / info["glb"]
            self.assertTrue(path.exists(), msg=f"{path} not found")
            loaded = trimesh.load(path)
            geom = loaded.to_geometry() if isinstance(loaded, trimesh.Scene) else loaded
            self.assertGreater(len(geom.vertices), 0, msg=f"{link} empty")
            # per-link bounding box should be a small part in mm (< 0.4 m)
            extent = geom.bounding_box.extents
            self.assertTrue(all(0.0 < e < 400.0 for e in extent),
                            msg=f"{link} bbox {extent} not a plausible mm-scale link")


if __name__ == "__main__":
    unittest.main()
