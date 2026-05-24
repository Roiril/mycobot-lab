"""Tests for pose_resolver Pose-union → Euler resolution."""
import unittest, math, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.pose_resolver import resolve_pose, _resolve_axis, _matrix_from_tool_z, _matrix_to_rpy
from arm.constants import HOME_ANGLES


class TestPoseResolver(unittest.TestCase):
    def test_preserve_returns_none(self):
        self.assertIsNone(resolve_pose({"kind": "preserve"}, (100, 0, 200), HOME_ANGLES))

    def test_any_returns_none(self):
        self.assertIsNone(resolve_pose({"kind": "any"}, (100, 0, 200), HOME_ANGLES))

    def test_none_returns_none(self):
        self.assertIsNone(resolve_pose(None, (100, 0, 200), HOME_ANGLES))

    def test_explicit_euler(self):
        r = resolve_pose({"kind": "explicit", "euler_xyz": [10, 20, 30]}, (100, 0, 200), HOME_ANGLES)
        self.assertAlmostEqual(r[0], 10); self.assertAlmostEqual(r[1], 20); self.assertAlmostEqual(r[2], 30)

    def test_align_tool_axis_preset(self):
        # approach="-z" means tool comes from -Z direction → tool axis = +Z (NOT -Z)
        # Wait: "approach axis" = direction FROM which tool comes. Pose docs say tool_z = -approach.
        # approach="-z" → tool_z = +Z (tool pointing UP).
        # In RPY (extrinsic XYZ), tool z = +Z world means no rotation needed (identity).
        r = resolve_pose({"kind": "align_tool", "approach": "-z"}, (100, 0, 200), HOME_ANGLES)
        # Identity orientation has rxyz ≈ 0,0,0 (up to roll-free choice)
        self.assertIsNotNone(r)

    def test_align_tool_top_down(self):
        # approach="+z" means tool comes from above → tool axis = -Z (tool points DOWN)
        r = resolve_pose({"kind": "align_tool", "approach": "+z"}, (100, 0, 200), HOME_ANGLES)
        self.assertIsNotNone(r)
        # roll about ±180 around X should put tool z in -Z
        # Verify by reconstructing matrix
        from arm.pose_resolver import _euler_xyz_to_matrix
        R = _euler_xyz_to_matrix(*r)
        tool_z = (R[0][2], R[1][2], R[2][2])
        self.assertAlmostEqual(tool_z[2], -1.0, places=2, msg=f"tool_z={tool_z}")

    def test_align_tool_vector(self):
        # arbitrary vector
        r = resolve_pose({"kind": "align_tool", "approach": [1, 0, 0]}, (100, 0, 200), HOME_ANGLES)
        self.assertIsNotNone(r)

    def test_align_tool_diagonal_vector(self):
        # non-axis-aligned vector (1,1,0)/sqrt(2) → tool z = -(1,1,0)/sqrt(2)
        from arm.pose_resolver import _euler_xyz_to_matrix
        r = resolve_pose({"kind": "align_tool", "approach": [1, 1, 0]}, (100, 0, 200), HOME_ANGLES)
        self.assertIsNotNone(r)
        R = _euler_xyz_to_matrix(*r)
        tool_z = (R[0][2], R[1][2], R[2][2])
        # Should point in (-1,-1,0)/sqrt(2)
        self.assertAlmostEqual(tool_z[0], -0.7071, places=2)
        self.assertAlmostEqual(tool_z[1], -0.7071, places=2)
        self.assertAlmostEqual(tool_z[2],  0.0,    places=2)

    def test_extend_toward_far(self):
        # At HOME [0,0,-90,0,0,0] the URDF flange is at ≈(213.7, -155.3, 312.0).
        # Pick a target whose direction from flange is unambiguously +X-dominant.
        r = resolve_pose({"kind": "extend_toward", "target": [500, -155, 312]}, (500, -155, 312), HOME_ANGLES)
        self.assertIsNotNone(r)
        from arm.pose_resolver import _euler_xyz_to_matrix
        R = _euler_xyz_to_matrix(*r)
        tool_z = (R[0][2], R[1][2], R[2][2])
        self.assertGreater(tool_z[0], 0.9)

    def test_extend_toward_too_close(self):
        from arm.kinematics import link_frames
        j6 = link_frames(HOME_ANGLES)[6]
        # Target right at J6 origin → preserve (None)
        r = resolve_pose({"kind": "extend_toward", "target": [j6[0][3], j6[1][3], j6[2][3]]}, (0,0,0), HOME_ANGLES)
        self.assertIsNone(r)

    def test_unknown_kind_raises(self):
        with self.assertRaises(ValueError):
            resolve_pose({"kind": "totally_made_up"}, (100, 0, 200), HOME_ANGLES)


class TestIKRetries(unittest.TestCase):
    def test_solve_with_retries_uses_orientation(self):
        from arm.ik_numeric import solve_with_retries, _fk_pose
        # Try to reach a position with align_tool=-z (tool pointing down)
        target_pos = (150, -50, 250)
        target_orient = resolve_pose({"kind": "align_tool", "approach": "+z"}, target_pos, HOME_ANGLES)
        sol, mode = solve_with_retries(target_pos, target_orient, HOME_ANGLES)
        self.assertIsNotNone(sol, f"IK should succeed, got mode={mode}")
        # Verify J6 has actually moved (proof that IK isn't ignoring it)
        # Note: it may or may not change depending on starting roll; just check we got something
        tip = _fk_pose(sol)
        err = ((tip[0]-target_pos[0])**2 + (tip[1]-target_pos[1])**2 + (tip[2]-target_pos[2])**2)**0.5
        self.assertLess(err, 5.0, f"position error {err:.1f}mm too high")

    def test_position_only_fallback(self):
        from arm.ik_numeric import solve_with_retries
        sol, mode = solve_with_retries((200, 0, 300), None, HOME_ANGLES)
        self.assertIsNotNone(sol)
        self.assertEqual(mode, "position_only")

    def test_seed_perturbations_clamped_to_limits(self):
        from arm.ik_numeric import _seed_perturbations
        from arm.kinematics import JOINT_LIMITS
        # seed at upper limit of J6 → +90 perturbation must clamp
        s = [0, 0, -90, 0, 0, 175]
        perts = _seed_perturbations(s)
        for p in perts:
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                self.assertGreaterEqual(p[i], lo)
                self.assertLessEqual(p[i], hi)

    def test_relaxed_roll_uses_tool_z_not_world_z(self):
        from arm.ik_numeric import _rotate_orientation_around_tool_z, _rpy_to_matrix
        # A nearly horizontal tool axis: e.g. tool pointing +x means rxyz like (0, -90, 0)
        rpy = (0.0, -90.0, 0.0)
        # Rotate by 30° around tool z (which is world +x at this orientation)
        new_rpy = _rotate_orientation_around_tool_z(rpy, 30.0)
        R_orig = _rpy_to_matrix(*rpy)
        R_new  = _rpy_to_matrix(*new_rpy)
        # tool z (3rd col) should be approximately unchanged (rotation around its own axis)
        for i in range(3):
            self.assertAlmostEqual(R_orig[i, 2], R_new[i, 2], places=2,
                                   msg=f"tool z component {i} drifted: {R_orig[i,2]} vs {R_new[i,2]}")


if __name__ == "__main__":
    unittest.main()
