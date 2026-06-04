"""Cross-check our hand-rolled SO-101 FK against MuJoCo's ground-truth model.

MuJoCo is an independent kinematics implementation built from the same CAD as
our URDF extraction. If our profile.URDF_LINKS / kinematics.link_frames agree
with MuJoCo body positions across many poses, the FK (and the transcription in
profile.py) is correct.

Skipped automatically if mujoco is not installed (optional sim dependency), so
the core suite stays runnable without it.

Run: python -m unittest tests.test_so101_mujoco
"""
import math
import unittest
import sys, pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

try:
    import mujoco  # noqa: F401
    _HAVE_MUJOCO = True
except ImportError:
    _HAVE_MUJOCO = False

from robots.so101 import profile
from robots.so101.kinematics import joint_positions


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed (optional sim dep)")
class TestFKAgainstMuJoCo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from robots.so101.sim.mujoco_sim import So101Sim
        cls.sim = So101Sim()

    def test_fk_matches_mujoco_body_positions(self):
        rng = np.random.default_rng(3)
        poses = [[0, 0, 0, 0, 0]]
        for _ in range(30):
            poses.append([rng.uniform(0.8 * lo, 0.8 * hi)
                          for (lo, hi) in profile.JOINT_LIMITS])
        worst = 0.0
        for ang in poses:
            self.sim.set_angles_deg(ang, gripper=0.0)
            mj = self.sim.joint_positions_mm()      # base + 5 joint frames
            mine = joint_positions(ang)             # same ordering
            self.assertEqual(len(mj), len(mine))
            for a, b in zip(mj, mine):
                worst = max(worst, math.dist(a, b))
        self.assertLess(worst, 0.05, msg=f"FK vs MuJoCo max position error {worst:.4f}mm")


if __name__ == "__main__":
    unittest.main()
