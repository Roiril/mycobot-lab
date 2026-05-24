"""Tests for arm.gestures — direction mapping, pose generation, safety."""
import unittest, math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm import gestures
from arm.kinematics import link_frames, JOINT_LIMITS
from arm.safety import check_angles
from arm.constants import CAMERA_UPRIGHT_J6_DEG


class TestDirMap(unittest.TestCase):
    def test_named_directions(self):
        self.assertEqual(gestures._dir_to_j1("back"), 0.0)
        self.assertEqual(gestures._dir_to_j1("right"), 90.0)
        self.assertEqual(gestures._dir_to_j1("front"), 165.0)
        self.assertEqual(gestures._dir_to_j1("left"), -90.0)

    def test_numeric_direction(self):
        self.assertEqual(gestures._dir_to_j1(45.0), 45.0)
        self.assertEqual(gestures._dir_to_j1(-100), -100.0)

    def test_numeric_clamped_to_safe_margin(self):
        # ±165 margin (not ±168 hardware limit)
        self.assertEqual(gestures._dir_to_j1(200), 165.0)
        self.assertEqual(gestures._dir_to_j1(-200), -165.0)

    def test_unknown_direction_falls_back_to_back(self):
        self.assertEqual(gestures._dir_to_j1("UNKNOWN"), 0.0)


class TestFace(unittest.TestCase):
    def test_face_returns_single_step(self):
        steps = gestures.face("front")
        self.assertEqual(len(steps), 1)
        ang = steps[0]["angles"]
        self.assertEqual(len(ang), 6)
        self.assertEqual(ang[5], CAMERA_UPRIGHT_J6_DEG)  # camera upright by default

    def test_face_safe(self):
        for d in ("back", "right", "front", "left"):
            steps = gestures.face(d)
            ok, msg, _ = check_angles(steps[0]["angles"])
            self.assertTrue(ok, f"face({d}) unsafe: {msg}")


class TestBow(unittest.TestCase):
    def test_bow_three_steps(self):
        steps = gestures.bow("front")
        self.assertEqual(len(steps), 3)
        # First and last should be at the "face" pose (J2=0)
        self.assertEqual(steps[0]["angles"][1], 0)
        self.assertEqual(steps[-1]["angles"][1], 0)

    def test_bow_all_steps_safe(self):
        for d in ("back", "front", "left", "right"):
            for step in gestures.bow(d, depth_deg=30):
                ok, msg, _ = check_angles(step["angles"])
                self.assertTrue(ok, f"bow({d}) step {step['label']} unsafe: {msg}")

    def test_bow_depth_clamped(self):
        # Spec says clamp 10..40
        steps_deep = gestures.bow("front", depth_deg=100)
        # bow_down step has J2 = -depth
        self.assertGreaterEqual(steps_deep[1]["angles"][1], -40)
        steps_shallow = gestures.bow("front", depth_deg=1)
        self.assertLessEqual(steps_shallow[1]["angles"][1], -10)


class TestNodWave(unittest.TestCase):
    def test_nod_safe(self):
        for step in gestures.nod("front", times=3):
            ok, msg, _ = check_angles(step["angles"])
            self.assertTrue(ok, f"nod step unsafe: {msg}")

    def test_wave_safe(self):
        for step in gestures.wave("front", times=3):
            ok, msg, _ = check_angles(step["angles"])
            self.assertTrue(ok, f"wave step unsafe: {msg}")


class TestPointAtExtending(unittest.TestCase):
    def _alignment_error_deg(self, angles, target):
        T = link_frames(angles)[-1]
        tool_z = (T[0][2], T[1][2], T[2][2])
        delta = (target[0]-T[0][3], target[1]-T[1][3], target[2]-T[2][3])
        n = math.sqrt(sum(c*c for c in delta))
        if n < 1e-6: return 999.0
        dot = sum(tool_z[i] * delta[i] / n for i in range(3))
        return math.degrees(math.acos(max(-1.0, min(1.0, dot))))

    # Pointable targets: far enough from the extending-pose flange that the
    # arm can actually aim at them. (Near-flange targets that the arm can't
    # point at are correctly refused by the >30° threshold — tested below.)
    POINTABLE = [
        [0, -700, 400],     # printer (back-up)
        [-200, -450, 200],  # monitor (back-left)
        [100, 500, 150],    # user (front)
        [600, -200, 250],   # side-back
    ]

    def test_point_at_returns_one_step_with_upright_j6(self):
        steps = gestures.point_at_extending(self.POINTABLE[0], label="X")
        self.assertEqual(len(steps), 1)
        ang = steps[0]["angles"]
        self.assertEqual(len(ang), 6)
        self.assertEqual(ang[5], CAMERA_UPRIGHT_J6_DEG)

    def test_point_at_alignment_better_than_30deg(self):
        for target in self.POINTABLE:
            steps = gestures.point_at_extending(target)
            err = self._alignment_error_deg(steps[0]["angles"], target)
            self.assertLess(err, 30.0, f"point_at({target}) alignment {err:.0f}° too high")

    def test_point_at_pose_safe(self):
        for target in self.POINTABLE:
            steps = gestures.point_at_extending(target)
            ok, msg, _ = check_angles(steps[0]["angles"])
            self.assertTrue(ok, f"point_at({target}) pose unsafe: {msg}")

    def test_point_at_within_joint_limits(self):
        for target in self.POINTABLE:
            ang = gestures.point_at_extending(target)[0]["angles"]
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                self.assertGreaterEqual(ang[i], lo, f"J{i+1}={ang[i]} below limit")
                self.assertLessEqual(ang[i], hi, f"J{i+1}={ang[i]} above limit")

    def test_point_at_refuses_near_flange_target(self):
        # A target close to where the extending-pose flange sits is unpointable
        # because the arm can't curl back to face itself
        with self.assertRaises(ValueError):
            gestures.point_at_extending([300, -100, 200])

    def test_point_at_refuses_impossible_target(self):
        # Target directly below base — should be physically unpointable
        with self.assertRaises(ValueError):
            gestures.point_at_extending([0, 0, -500])


class TestBuildDispatch(unittest.TestCase):
    def test_build_face(self):
        steps = gestures.build({"kind": "face", "direction": "front"})
        self.assertEqual(len(steps), 1)

    def test_build_bow(self):
        steps = gestures.build({"kind": "bow", "direction": "front"})
        self.assertEqual(len(steps), 3)

    def test_build_nod(self):
        steps = gestures.build({"kind": "nod"})
        self.assertGreater(len(steps), 1)

    def test_build_wave(self):
        steps = gestures.build({"kind": "wave"})
        self.assertGreater(len(steps), 2)

    def test_build_home(self):
        steps = gestures.build({"kind": "home"})
        self.assertEqual(len(steps), 1)
        self.assertEqual(steps[0]["angles"], [0, 0, -90, 0, 0, 0])

    def test_build_point_at(self):
        steps = gestures.build({"kind": "point_at", "target_xyz": [0, -700, 400]})
        self.assertEqual(len(steps), 1)

    def test_build_unknown_kind_raises(self):
        with self.assertRaises(ValueError):
            gestures.build({"kind": "dance"})


if __name__ == "__main__":
    unittest.main()
