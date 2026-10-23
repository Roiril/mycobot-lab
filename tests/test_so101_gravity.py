"""SO-101 gravity model vs MuJoCo's own inverse dynamics.

robots.so101.gravity is a stdlib-only re-implementation (it has to run in the
lerobot venv on the hot path). MuJoCo computes the same quantity independently
from the same MJCF: qfrc_bias at zero velocity is exactly the actuator torque
that holds the pose. Agreement over random poses proves both the frame mapping
(URDF link frames == MJCF body frames) and the sign convention.

Skipped when mujoco is not installed.
"""
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import gravity, profile  # noqa: E402

try:
    import mujoco
    import numpy as np
    _HAVE_MUJOCO = True
except ImportError:  # pragma: no cover
    _HAVE_MUJOCO = False


class TestGravityBasics(unittest.TestCase):
    def test_masses_come_from_the_mjcf(self):
        inert = gravity.link_inertials()
        total = sum(inert[b]["mass"] for b in
                    ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper",
                     "moving_jaw_so101_v1"))
        # six moving bodies, each carries an STS3215 except the jaw
        self.assertAlmostEqual(total, 0.485, delta=0.01)

    def test_base_rotation_carries_no_gravity_load(self):
        # not exactly 0: the URDF writes pi as 3.14159, tilting the pan axis by
        # ~3e-6 rad, which leaves a few micro-N*m
        for pose in ([0, 0, 0, 0, 0], [40, 60, -50, 30, 10], [-90, -30, 80, -60, 0]):
            self.assertAlmostEqual(gravity.holding_torques(pose)[0], 0.0, delta=1e-5)

    def test_payload_only_adds_load(self):
        pose = [0, 60, -60, 0, 0]
        bare = gravity.holding_torques(pose)
        loaded = gravity.holding_torques(pose, payload_kg=0.2)
        for j in (1, 2, 3):
            self.assertGreater(abs(loaded[j]), abs(bare[j]))

    def test_ticks_to_deg_follows_lerobot_degrees(self):
        # lerobot: deg = (raw - mid) * 360 / 4095, mid = range centre
        self.assertAlmostEqual(gravity.ticks_to_deg(2000, (1000, 3000)), 0.0)
        self.assertAlmostEqual(gravity.ticks_to_deg(3000, (1000, 3000)), 1000 * 360 / 4095)


@unittest.skipUnless(_HAVE_MUJOCO, "mujoco not installed (optional sim dep)")
class TestGravityAgainstMuJoCo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xml = ROOT / "src" / "robots" / "so101" / "sim" / "so101_new_calib.xml"
        cls.model = mujoco.MjModel.from_xml_path(str(xml))
        cls.data = mujoco.MjData(cls.model)

    def mj_hold(self, pose_deg, jaw_deg=0.0):
        d = self.data
        d.qpos[:] = 0.0
        d.qpos[:5] = [math.radians(a) for a in pose_deg]
        d.qpos[5] = math.radians(jaw_deg)
        d.qvel[:] = 0.0
        mujoco.mj_forward(self.model, d)
        return d.qfrc_bias[:5].copy()

    def test_matches_qfrc_bias_on_random_poses(self):
        rng = np.random.default_rng(7)
        worst = 0.0
        for _ in range(60):
            pose = [float(rng.uniform(0.9 * lo, 0.9 * hi)) for lo, hi in profile.JOINT_LIMITS]
            jaw = float(rng.uniform(-10, 90))
            mine = gravity.holding_torques(pose, jaw_deg=jaw)
            ref = self.mj_hold(pose, jaw)
            for a, b in zip(mine, ref):
                worst = max(worst, abs(a - b))
        self.assertLess(worst, 1e-4, f"gravity model vs MuJoCo worst |dtau| = {worst:.2e} N*m")

    def test_worst_case_loads_are_what_the_design_assumes(self):
        # design numbers (plan 2026-10-24): shoulder_lift ~0.86, elbow ~0.45 N*m
        worst = [0.0] * 5
        for lift in range(-100, 101, 10):
            for elbow in range(-96, 97, 8):
                for wf in range(-90, 91, 30):
                    t = gravity.holding_torques([0, lift, elbow, wf, 0])
                    worst = [max(w, abs(x)) for w, x in zip(worst, t)]
        self.assertAlmostEqual(worst[1], 0.86, delta=0.03)
        self.assertAlmostEqual(worst[2], 0.45, delta=0.03)
        self.assertLess(worst[3], 0.13)


if __name__ == "__main__":
    unittest.main()
