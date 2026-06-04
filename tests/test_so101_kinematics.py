"""SO-101 kinematics: verify the hand-extracted profile.py matches the source
URDF, and that FK is internally consistent.

The high-value check here is `test_profile_matches_urdf`: profile.py was
transcribed by hand from so101_new_calib.urdf, so we parse the URDF directly
and assert every link transform / joint limit agrees. This catches transcription
errors offline, before the hardware arrives.

Run: python -m unittest tests.test_so101_kinematics
"""
import math
import unittest
import sys, pathlib
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile
from robots.so101.kinematics import link_frames, joint_positions, end_effector, tool_frame

URDF_PATH = ROOT / "src" / "robots" / "so101" / "so101_new_calib.urdf"


def _parse_urdf_joints():
    """name -> {xyz_mm, rpy_rad, lower, upper, parent, child} for every joint."""
    tree = ET.parse(URDF_PATH)
    out = {}
    for j in tree.getroot().findall("joint"):
        name = j.get("name")
        origin = j.find("origin")
        xyz = [float(v) * 1000.0 for v in origin.get("xyz").split()]  # m -> mm
        rpy = [float(v) for v in origin.get("rpy").split()]
        lim = j.find("limit")
        lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else None
        hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else None
        out[name] = {
            "xyz": xyz, "rpy": rpy, "lower": lo, "upper": hi,
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
        }
    return out


class TestProfileMatchesURDF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.j = _parse_urdf_joints()

    def test_arm_link_transforms_match(self):
        # profile.URDF_LINKS[i] must equal the URDF joint for JOINT_NAMES[i].
        for i, name in enumerate(profile.JOINT_NAMES):
            u = self.j[name]
            p_xyz, p_rpy = profile.URDF_LINKS[i]
            for k in range(3):
                self.assertAlmostEqual(p_xyz[k], u["xyz"][k], delta=1e-2,
                    msg=f"{name} xyz[{k}]: profile {p_xyz[k]} vs urdf {u['xyz'][k]}")
                self.assertAlmostEqual(p_rpy[k], u["rpy"][k], delta=1e-4,
                    msg=f"{name} rpy[{k}]: profile {p_rpy[k]} vs urdf {u['rpy'][k]}")

    def test_tool_transform_matches(self):
        u = self.j["gripper_frame_joint"]
        t_xyz, t_rpy = profile.TOOL_TRANSFORM
        for k in range(3):
            self.assertAlmostEqual(t_xyz[k], u["xyz"][k], delta=1e-2)
            self.assertAlmostEqual(t_rpy[k], u["rpy"][k], delta=1e-4)

    def test_joint_limits_match(self):
        for i, name in enumerate(profile.JOINT_NAMES):
            u = self.j[name]
            lo, hi = profile.JOINT_LIMITS[i]
            self.assertAlmostEqual(lo, math.degrees(u["lower"]), delta=0.1,
                msg=f"{name} lower")
            self.assertAlmostEqual(hi, math.degrees(u["upper"]), delta=0.1,
                msg=f"{name} upper")

    def test_chain_is_connected(self):
        # The 5 arm joints + gripper_frame_joint form a connected base->TCP chain.
        chain = profile.JOINT_NAMES + ["gripper_frame_joint"]
        for a, b in zip(chain, chain[1:]):
            self.assertEqual(self.j[a]["child"], self.j[b]["parent"],
                msg=f"{a}.child != {b}.parent")
        self.assertEqual(self.j[chain[0]]["parent"], "base_link")
        self.assertEqual(self.j[chain[-1]]["child"], "gripper_frame_link")


class TestForwardKinematics(unittest.TestCase):
    def test_dimensions(self):
        self.assertEqual(profile.NUM_JOINTS, 5)
        self.assertEqual(len(profile.URDF_LINKS), 5)
        self.assertEqual(len(profile.JOINT_LIMITS), 5)
        self.assertEqual(len(profile.HOME_ANGLES), 5)
        self.assertEqual(len(profile.JOINT_NAMES), 5)

    def test_frame_count(self):
        frames = link_frames([0, 0, 0, 0, 0])
        self.assertEqual(len(frames), 6)  # base + 5 joints
        self.assertEqual(len(joint_positions([0, 0, 0, 0, 0])), 6)

    def test_wrong_length_raises(self):
        with self.assertRaises(ValueError):
            link_frames([0, 0, 0])  # 6-DoF habit must not silently pass

    def test_reach_is_plausible(self):
        # SO-101 reach ~0.4 m. The TCP at any in-limit pose stays within a
        # sphere bounded by the summed link lengths (~0.45 m), and is non-trivial.
        for ang in ([0, 0, 0, 0, 0], [0, -90, 90, 0, 0], [45, -30, 30, -45, 90]):
            x, y, z = end_effector(ang)
            dist = math.sqrt(x*x + y*y + z*z)
            self.assertGreater(dist, 30.0)
            self.assertLess(dist, 600.0, msg=f"tip {dist:.1f}mm at {ang} exceeds plausible reach")

    def test_tool_frame_is_rigid(self):
        # tool_frame must be a valid homogeneous transform (rotation orthonormal).
        T = tool_frame([10, -20, 30, -10, 45])
        R = [[T[i][k] for k in range(3)] for i in range(3)]
        # columns unit length
        for c in range(3):
            n = math.sqrt(sum(R[r][c] ** 2 for r in range(3)))
            self.assertAlmostEqual(n, 1.0, delta=1e-9)
        self.assertEqual([T[3][0], T[3][1], T[3][2], T[3][3]], [0.0, 0.0, 0.0, 1.0])

    def test_continuity(self):
        # A 1deg nudge on each joint moves the TCP smoothly (<25mm for ~0.4m reach).
        base = end_effector([0, 0, 0, 0, 0])
        for i in range(profile.NUM_JOINTS):
            a = [0, 0, 0, 0, 0]
            a[i] = 1.0
            p = end_effector(a)
            d = math.sqrt(sum((p[k] - base[k]) ** 2 for k in range(3)))
            self.assertLess(d, 25.0, msg=f"joint {i} 1deg -> {d:.1f}mm tip move")


if __name__ == "__main__":
    unittest.main()
