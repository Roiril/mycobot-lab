"""SO-101 cockpit ✋ hand-launch glue: the pure message formatter and the
/hand-status proxy, without a real Quest, adb, or hand server.

The cockpit imports lerobot-ish deps via teleop_engine but that import works in
the test env (see test_so101_teleop_engine). We only exercise the two pieces the
hand-launch feature adds: _quest_launch_message (result dict → human summary) and
hand_status_proxy (relay :8001 /hand/status, add server_up, never raise).

Run: python -m unittest tests.test_cockpit_hand_launch
"""
from __future__ import annotations
import io
import json
import sys
import pathlib
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

import so101_cockpit_server as cockpit  # noqa: E402


class TestQuestLaunchMessage(unittest.TestCase):
    def test_not_ok_returns_error_text(self):
        msg = cockpit._quest_launch_message({"ok": False, "error": "接続中の Quest がありません"})
        self.assertEqual(msg, "接続中の Quest がありません")

    def test_not_ok_without_error_has_fallback(self):
        self.assertIn("失敗", cockpit._quest_launch_message({"ok": False}))

    def test_all_native_says_just_wear_it(self):
        msg = cockpit._quest_launch_message({
            "ok": True,
            "devices": [{"serial": "ABCDEF1234", "action": "launched_native"}],
        })
        self.assertIn("被るだけで操作できます", msg)
        self.assertNotIn("VR 開始", msg)  # native = no in-headset tap needed

    def test_mixed_native_and_browser_mentions_tap(self):
        msg = cockpit._quest_launch_message({
            "ok": True,
            "devices": [
                {"serial": "AAAA0000", "action": "launched_native"},
                {"serial": "BBBB1111", "action": "navigated"},
            ],
        })
        self.assertIn("VR 開始", msg)  # the browser device still needs a tap

    def test_browser_only_says_tap(self):
        msg = cockpit._quest_launch_message({
            "ok": True,
            "devices": [{"serial": "BBBB1111", "action": "launched"}],
        })
        self.assertIn("VR 開始", msg)

    def test_error_device_detail_is_shown(self):
        msg = cockpit._quest_launch_message({
            "ok": True,
            "devices": [
                {"serial": "AAAA0000", "action": "launched_native"},
                {"serial": "CCCC2222", "action": "error", "detail": "monkey aborted"},
            ],
        })
        self.assertIn("monkey aborted", msg)


class _FakeResp(io.BytesIO):
    """Minimal context-manager stand-in for urlopen()."""
    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
        return False


class TestHandStatusProxy(unittest.TestCase):
    def test_relays_status_and_adds_server_up(self):
        payload = {"present": True, "connected": True, "mcu_alive": True, "port": "COM9"}
        with mock.patch("urllib.request.urlopen",
                        return_value=_FakeResp(json.dumps(payload).encode())):
            out = cockpit.hand_status_proxy()
        self.assertTrue(out["server_up"])
        self.assertTrue(out["mcu_alive"])
        self.assertEqual(out["port"], "COM9")

    def test_server_down_returns_not_up_without_raising(self):
        with mock.patch("urllib.request.urlopen", side_effect=OSError("refused")):
            out = cockpit.hand_status_proxy()
        self.assertFalse(out["server_up"])
        self.assertFalse(out["mcu_alive"])
        self.assertIn("error", out)

    def test_probe_once_false_on_connection_error(self):
        with mock.patch("urllib.request.urlopen", side_effect=OSError("refused")):
            self.assertFalse(cockpit.probe_once("http://127.0.0.1:8001/"))


if __name__ == "__main__":
    unittest.main()
