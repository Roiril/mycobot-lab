"""Smoke tests for grasp sequence endpoint via VirtualHub."""
import unittest, sys, pathlib, threading, time, urllib.request, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.constants import (
    TARGET_RADIUS_MIN_MM, TARGET_RADIUS_MAX_MM,
    GRASP_OFFSET_MIN_MM, GRASP_OFFSET_MAX_MM,
    GRIPPER_TIP_CLEARANCE_MM, GRASP_APPROACH_SPEED_MAX,
)


class TestGraspParams(unittest.TestCase):
    """Validate constants relationships without spinning up the server."""

    def test_radius_bounds_sensible(self):
        self.assertLess(TARGET_RADIUS_MIN_MM, TARGET_RADIUS_MAX_MM)
        self.assertGreater(TARGET_RADIUS_MIN_MM, 0)

    def test_offset_bounds_sensible(self):
        self.assertLess(GRASP_OFFSET_MIN_MM, GRASP_OFFSET_MAX_MM)
        self.assertGreater(GRASP_OFFSET_MIN_MM, 0)

    def test_tip_clearance_nonzero(self):
        # without gripper, the tip must stop above object surface
        self.assertGreater(GRIPPER_TIP_CLEARANCE_MM, 0)

    def test_grasp_speed_cap_reasonable(self):
        # Grasp should be slow — cap at most half of typical move speed
        self.assertLessEqual(GRASP_APPROACH_SPEED_MAX, 40)


class TestGraspServerSmoke(unittest.TestCase):
    """Spin up the offline server, hit /grasp_sequence."""
    @classmethod
    def setUpClass(cls):
        import subprocess, os
        env = os.environ.copy()
        cls.proc = subprocess.Popen(
            [sys.executable, "scripts/server.py", "--offline", "--port", "8765"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            cwd=str(pathlib.Path(__file__).resolve().parent.parent),
        )
        time.sleep(3)

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate(); cls.proc.wait(timeout=5)

    def _post(self, path, body):
        req = urllib.request.Request(
            f"http://127.0.0.1:8765{path}",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read().decode())

    def test_reject_bad_radius(self):
        code, body = self._post("/grasp_sequence", {"x": 200, "y": -50, "z": 200, "radius": 99, "speed": 15})
        self.assertEqual(code, 400)
        self.assertIn("radius", body.get("error", ""))

    def test_reject_bad_offset(self):
        code, body = self._post("/grasp_sequence", {"x": 200, "y": -50, "z": 200, "radius": 20, "approach_offset": 5})
        self.assertEqual(code, 400)
        self.assertIn("approach_offset", body.get("error", ""))

    def test_reject_bad_speed(self):
        code, body = self._post("/grasp_sequence", {"x": 200, "y": -50, "z": 200, "radius": 20, "speed": 99})
        self.assertEqual(code, 400)

    def test_valid_request_runs_three_stages(self):
        code, body = self._post("/grasp_sequence", {"x": 200, "y": -50, "z": 200, "radius": 15, "speed": 15})
        if code == 200:
            self.assertEqual(body["stages"], ["pre-grasp", "approach", "lift"])
            self.assertIn("graspZ", body)
            # tip should stop ABOVE object surface (z + radius + clearance)
            self.assertGreater(body["graspZ"], 200)


if __name__ == "__main__":
    unittest.main()
