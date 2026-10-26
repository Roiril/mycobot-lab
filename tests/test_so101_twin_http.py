"""HTTP integration: real physics, mock follower, never opens serial ports."""
import importlib.util
import json
import pathlib
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
try:
    import mujoco
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "requires simulator env (.venv-so101)")
class TwinHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("so101_twin_http_host", ROOT / "scripts/so101_sim_server.py")
        cls.host = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.host)

    def setUp(self):
        from robots.so101.twin_link import HardwareLink
        from robots.so101.twin_studio import TwinStudio
        self.tmp = tempfile.TemporaryDirectory()
        self.sim = self.host.Sim()
        self.link = HardwareLink(mode="mock")
        self.sim.studio = TwinStudio(self.sim, self.link, self.tmp.name)
        self.host.SIM = self.sim
        self.server = self.host.ThreadingHTTPServer(("127.0.0.1", 0), self.host.Handler)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.link.close()
        self.tmp.cleanup()

    def request(self, op=None, path="/api/twin/cmd", headers=None, **values):
        payload = json.dumps({"op": op, "client_id": "alice", **values}).encode() if op else None
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", data=payload,
                                     headers={"Content-Type": "application/json", **(headers or {})})
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_observe_import_follow_goal_stop_produces_real_mock_output(self):
        from robots.so101.twin_studio import JOINTS
        code, state = self.request("hardware_connect")
        self.assertEqual(code, 200)
        self.assertEqual(state["hardware"]["commands_sent"], 0)
        self.assertEqual(self.request("import_actual")[0], 200)
        state = self.request(path="/api/state")[1]
        self.assertIsNotNone(state["measured_bodies"])
        self.assertLess(state["twin"]["hardware"]["max_error_deg"], 0.1)
        result = self.request("follow", ready=True)[1]
        self.assertIn("lease", result)
        self.assertTrue(result["hardware"]["following"])
        start = self.link.snapshot()["joints"][JOINTS[0]]["q"]
        code, _ = self.request("goal", path="/api/cmd", goals={JOINTS[0]: start + 3})
        self.assertEqual(code, 200)
        self.sim.studio.tick()
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            self.request("heartbeat", lease=result["lease"])
            if self.link.snapshot()["joints"][JOINTS[0]]["q"] > start + 1:
                break
            time.sleep(0.03)
        moved = self.link.snapshot()["joints"][JOINTS[0]]["q"]
        self.assertGreater(moved, start + 1)
        code, state = self.request("stop", client_id="bob")
        self.assertEqual(code, 200)
        self.assertFalse(state["hardware"]["following"])
        self.assertNotIn("lease", self.request(path="/api/twin/state")[1]["hardware"])
        after = self.link.snapshot()["commands_sent"]
        self.sim.studio.tick()
        time.sleep(0.1)
        self.assertEqual(self.link.snapshot()["commands_sent"], after)

    def test_foreign_origin_and_rebinding_rejected_before_any_write(self):
        for headers in ({"Origin": "https://example.com"}, {"Sec-Fetch-Site": "cross-site"},
                        {"Host": "attacker.example"}):
            code, _ = self.request("hardware_connect", headers=headers)
            self.assertEqual(code, 403)
            self.assertEqual(self.link.snapshot()["commands_sent"], 0)
            self.assertFalse(self.link.snapshot()["connected"])

    def test_owner_and_physics_gates_server_side(self):
        self.request("hardware_connect")
        self.request("import_actual")
        result = self.request("follow", ready=True)[1]
        self.assertIn("lease", result)
        for cmd in ({"op": "goal", "goals": {"shoulder_pan": 4}, "client_id": "bob"},
                    {"op": "world", "floor": False}, {"op": "run", "paused": True},
                    {"op": "reset"}, {"op": "torque", "on": False}):
            op = cmd.pop("op")
            code, _ = self.request(op, path="/api/cmd", **cmd)
            self.assertEqual(code, 400)
        self.request("stop")

    def test_program_save_reload_and_revision_conflict(self):
        code, _ = self.request("capture", label="構え", move_s=2, hold_s=0.5)
        self.assertEqual(code, 200)
        self.request("save")
        state = self.request(path="/api/twin/state")[1]
        self.assertEqual(len(state["program"]["steps"]), 1)
        self.assertEqual(state["revision"], state["saved_revision"])
        altered = {**state["program"], "name": "他の動作"}
        code, _ = self.request("program", program=altered, expected_revision=0)
        self.assertEqual(code, 400)
        self.assertEqual(self.request(path="/api/twin/state")[1]["program"]["name"], "新しい動作")
        self.assertEqual(self.request("load", name="新しい動作")[0], 200)

    def test_invalid_input_does_not_break_physics_or_state(self):
        for op, args in (("goal", {"goals": {"elbow_flex": float("nan")}}),
                         ("run", {"rtf": float("inf")}), ("goal", {"goals": {"unknown": 0}})):
            code, _ = self.request(op, path="/api/cmd", **args)
            self.assertEqual(code, 400)
        self.assertEqual(self.request(path="/api/state?since=bad")[0], 400)
        self.assertEqual(self.request(path="/api/state")[0], 200)

    def test_encoder_rounding_at_limits_can_be_saved(self):
        from robots.so101 import profile
        from robots.so101.twin_studio import JOINTS
        limits = profile.JOINT_LIMITS + [profile.GRIPPER_LIMIT_DEG]
        for side in (0, 1):
            code, _ = self.request("goal", path="/api/cmd", goals={n: lim[side] for n, lim in zip(JOINTS, limits)})
            self.assertEqual(code, 200)
            code, state = self.request("capture")
            self.assertEqual(code, 200)
            for a, (lo, hi) in zip(state["program"]["steps"][-1]["angles"], limits):
                self.assertGreaterEqual(a, lo)
                self.assertLessEqual(a, hi)


if __name__ == "__main__":
    unittest.main()
