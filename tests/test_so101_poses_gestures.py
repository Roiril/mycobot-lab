"""SO-101 named poses + gestures: every pose and every gesture waypoint must
pass safety.check_angles (joint limits + floor + self-collision).

Run: python -m unittest tests.test_so101_poses_gestures
"""
import unittest
import sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import profile, poses, gestures, safety


class TestPoses(unittest.TestCase):
    def test_all_poses_safe(self):
        for name, ang in poses.POSES.items():
            self.assertEqual(len(ang), profile.NUM_JOINTS, msg=f"{name} wrong length")
            ok, msg, _ = safety.check_angles(ang)
            self.assertTrue(ok, msg=f"pose {name} unsafe: {msg}")


class TestGestures(unittest.TestCase):
    def _assert_seq_safe(self, seq, label):
        self.assertGreater(len(seq), 0, msg=f"{label} produced no waypoints")
        for i, wp in enumerate(seq):
            self.assertEqual(len(wp), profile.NUM_JOINTS, msg=f"{label}[{i}] wrong length")
            ok, msg, _ = safety.check_angles(wp)
            self.assertTrue(ok, msg=f"{label}[{i}] unsafe: {msg}")

    def test_face_single_pose_safe(self):
        for az in (-90, -45, 0, 45, 90):
            p = gestures.face(az)
            ok, msg, _ = safety.check_angles(p)
            self.assertTrue(ok, msg=f"face({az}) unsafe: {msg}")
            # pan clamped within limits
            self.assertGreaterEqual(p[gestures.PAN], profile.JOINT_LIMITS[0][0])
            self.assertLessEqual(p[gestures.PAN], profile.JOINT_LIMITS[0][1])

    def test_face_clamps_extreme_azimuth(self):
        p = gestures.face(999)
        self.assertEqual(p[gestures.PAN], profile.JOINT_LIMITS[0][1])

    def test_point_safe(self):
        for az in (-60, 0, 60):
            ok, msg, _ = safety.check_angles(gestures.point(az))
            self.assertTrue(ok, msg=f"point({az}) unsafe: {msg}")

    def test_nod_safe(self):
        self._assert_seq_safe(gestures.nod(), "nod")

    def test_shake_safe(self):
        self._assert_seq_safe(gestures.shake(), "shake")

    def test_wave_safe(self):
        self._assert_seq_safe(gestures.wave(), "wave")

    def test_bow_safe(self):
        self._assert_seq_safe(gestures.bow(), "bow")

    def test_gesture_returns_to_base(self):
        # nod/shake/bow end back at their base pose (last waypoint == base)
        base = poses.READY
        self.assertEqual(gestures.nod(base=base)[-1], base)
        self.assertEqual(gestures.shake(base=base)[-1], base)
        self.assertEqual(gestures.bow(base=base)[-1], base)


if __name__ == "__main__":
    unittest.main()
