"""Hand USB rescan: when the server should swap drivers.

The hand's FTDI adapter enumerates and drops independently of the ATOM, and the
driver only looks for a port when it is constructed — so without this the server
needed a manual restart per drop (2026-08-07: five in one session). This covers
the decision, which is pure; the thread around it only sleeps and swaps.

Run: python -m unittest discover tests -p "test_hand_rescan.py"
"""
import sys
import pathlib
import importlib.util
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _load_server():
    """Import scripts/server.py without running it (it is a script, not a pkg)."""
    for extra in (ROOT / "src", ROOT / "hand", ROOT / "scripts"):
        sys.path.insert(0, str(extra))
    spec = importlib.util.spec_from_file_location(
        "mycobot_server_under_test", ROOT / "scripts" / "server.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


SERVER = _load_server()
decide = SERVER.hand_rescan_decision


class TestHandRescanDecision(unittest.TestCase):
    def test_live_driver_on_an_attached_port_is_left_alone(self):
        self.assertEqual(decide(False, "COM9", {"COM9", "COM13"}, True), "keep")

    def test_live_driver_whose_port_vanished_is_dropped(self):
        # otherwise /hand/status keeps advertising a link that no longer exists
        self.assertEqual(decide(False, "COM9", {"COM13"}, False), "drop-to-virtual")

    def test_virtual_driver_takes_a_port_as_soon_as_one_appears(self):
        self.assertEqual(decide(True, None, {"COM9"}, True), "pick-up")

    def test_virtual_driver_waits_when_there_is_no_port(self):
        self.assertEqual(decide(True, None, {"COM13"}, False), "keep")

    def test_a_pinned_port_is_honoured_even_before_it_is_seen(self):
        # --hand-port COMx is the operator's assertion; make_hand reports failure
        self.assertEqual(decide(True, None, set(), True), "pick-up")

    def test_live_driver_without_a_known_port_is_left_alone(self):
        # a driver that never recorded its port must not be torn down blindly
        self.assertEqual(decide(False, None, set(), False), "keep")


if __name__ == "__main__":
    unittest.main()
