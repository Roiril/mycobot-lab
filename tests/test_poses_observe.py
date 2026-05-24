"""All OBSERVE-family poses must pass safety.check_angles.

These poses are referenced by docs/AGENT_API.md §8 and by VisionHub retry_hints
as concrete /move patches. They must remain inside joint limits + clear of the
floor + free of self-collision at all times.
"""
import unittest, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from arm.safety import check_angles
from arm import poses


class TestObservePoses(unittest.TestCase):
    def _check(self, name, angles):
        ok, msg, bad = check_angles(angles)
        self.assertTrue(ok, f"{name} fails safety: {msg} (bad={bad})")

    def test_observe_passes(self):
        self._check("OBSERVE", poses.OBSERVE)

    def test_observe_left_passes(self):
        self._check("OBSERVE_LEFT", poses.OBSERVE_LEFT)

    def test_observe_right_passes(self):
        self._check("OBSERVE_RIGHT", poses.OBSERVE_RIGHT)

    def test_observe_high_passes(self):
        self._check("OBSERVE_HIGH", poses.OBSERVE_HIGH)


if __name__ == "__main__":
    unittest.main()
