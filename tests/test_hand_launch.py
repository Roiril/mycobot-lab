"""Quest hand-launch branch logic: native VR app vs WebXR /hand fallback.

Two layers, no real hardware / no adb:
  1. Pure parsers/decision (native_installed / native_is_foreground /
     decide_native_action) — text in, verdict out.
  2. launch_hand_on_device with a FAKE adb (hand_launch.run_adb monkeypatched) so
     the native/browser/already branch order is exercised end-to-end. The native
     branch is validated against a device where `pm list` does NOT contain the app
     (→ browser fallback) as well as installed/foreground/launch cases.

Run: python -m unittest tests.test_hand_launch
"""
from __future__ import annotations
import os
import subprocess
import sys
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts" / "quest"))

import hand_launch  # noqa: E402
from hand_launch import (  # noqa: E402
    NATIVE_PKG, native_installed, native_is_foreground, decide_native_action,
    launch_hand_on_device,
)


# --- realistic adb text fixtures --------------------------------------------

PM_INSTALLED = (
    "package:com.oculus.browser\n"
    f"package:{NATIVE_PKG}\n"
    "package:com.android.settings\n"
)
PM_NOT_INSTALLED = (
    "package:com.oculus.browser\n"
    "package:com.android.settings\n"
)
# `pm list packages <pkg>` substring filter can return a near-namesake only.
PM_NAMESAKE_ONLY = f"package:{NATIVE_PKG}.debug\n"

DUMPSYS_FG = (
    "  mResumedActivity: ActivityRecord{a1b2 u0 "
    f"{NATIVE_PKG}/com.unity3d.player.UnityPlayerActivity t42}}\n"
)
DUMPSYS_BG = (
    "  mResumedActivity: ActivityRecord{a1b2 u0 "
    "com.oculus.vrshell/.MainActivity t7}\n"
)


class TestPureParsers(unittest.TestCase):
    def test_native_installed_exact(self):
        self.assertTrue(native_installed(PM_INSTALLED))
        self.assertFalse(native_installed(PM_NOT_INSTALLED))
        self.assertFalse(native_installed(""))

    def test_native_installed_rejects_namesake(self):
        # com.mycobot.handteleop.debug must NOT count as the real app.
        self.assertFalse(native_installed(PM_NAMESAKE_ONLY))

    def test_native_installed_with_apk_path_suffix(self):
        # `pm list packages -f` appends =<apk>. Still an exact package match.
        self.assertTrue(native_installed(f"package:{NATIVE_PKG}=/data/app/x/base.apk\n"))

    def test_native_is_foreground(self):
        self.assertTrue(native_is_foreground(DUMPSYS_FG))
        self.assertFalse(native_is_foreground(DUMPSYS_BG))
        self.assertFalse(native_is_foreground(""))

    def test_foreground_ignores_non_resumed_mention(self):
        # Package present in the dump but only as a paused record → not foreground.
        paused = (
            "  mResumedActivity: ActivityRecord{x u0 com.oculus.vrshell/.M t1}\n"
            f"  mLastPausedActivity: ActivityRecord{{y u0 {NATIVE_PKG}/.U t2}}\n"
        )
        self.assertFalse(native_is_foreground(paused))

    def test_decide_native_action(self):
        self.assertEqual(decide_native_action(False, False), "browser")
        self.assertEqual(decide_native_action(False, True), "browser")
        self.assertEqual(decide_native_action(True, True), "already_native")
        self.assertEqual(decide_native_action(True, False), "launched_native")


class FakeAdb:
    """Dispatch a fake `adb` on argv shape. Records every call for assertions."""

    def __init__(self, *, installed: bool, foreground: bool = False,
                 monkey_ok: bool = True, has_cdp_tab: bool = False):
        self.installed = installed
        self.foreground = foreground
        self.monkey_ok = monkey_ok
        self.has_cdp_tab = has_cdp_tab
        self.calls: list[list[str]] = []

    def _cp(self, args, rc=0, out="", err=""):
        return subprocess.CompletedProcess(args=args, returncode=rc, stdout=out, stderr=err)

    def run_adb(self, adb, args, serial=None, check=False, timeout=20.0):
        self.calls.append(list(args))
        head = args[:4]
        if head[:4] == ["shell", "pm", "list", "packages"]:
            return self._cp(args, out=(PM_INSTALLED if self.installed else PM_NOT_INSTALLED))
        if args[:3] == ["shell", "dumpsys", "activity"]:
            return self._cp(args, out=(DUMPSYS_FG if self.foreground else DUMPSYS_BG))
        if args[:2] == ["shell", "monkey"]:
            if self.monkey_ok:
                return self._cp(args, out="Events injected: 1\n")
            return self._cp(args, rc=1,
                            out="** No activities found to run, monkey aborted.\n")
        if args and args[0] in ("reverse", "forward"):
            return self._cp(args)
        if args[:3] == ["shell", "input", "keyevent"]:
            return self._cp(args)
        if args[:3] == ["shell", "am", "start"]:
            return self._cp(args, out="Starting: Intent { ... }\n")
        return self._cp(args)

    def issued(self, *prefix) -> bool:
        p = list(prefix)
        return any(c[:len(p)] == p for c in self.calls)


class TestLaunchBranch(unittest.TestCase):
    def setUp(self):
        self._orig_run = hand_launch.run_adb
        self._orig_cdp = hand_launch.cdp_pages

    def tearDown(self):
        hand_launch.run_adb = self._orig_run
        hand_launch.cdp_pages = self._orig_cdp

    def _install(self, fake: FakeAdb, cdp_raises=True):
        hand_launch.run_adb = fake.run_adb
        if cdp_raises:
            def _boom(*a, **k):
                raise OSError("no CDP forward (test)")
            hand_launch.cdp_pages = _boom

    def test_not_installed_falls_back_to_browser(self):
        # The dry-run-equivalent: pm list has no native app → browser flow runs.
        fake = FakeAdb(installed=False)
        self._install(fake)  # CDP unreachable → bootstrap via LauncherActivity
        res = launch_hand_on_device("adb", "SERIAL123", 0, deadline=None)
        self.assertEqual(res["action"], "launched")  # browser bootstrap
        # browser flow set reverse + forward, and NEVER touched monkey.
        self.assertTrue(fake.issued("reverse"))
        self.assertTrue(fake.issued("forward"))
        self.assertFalse(fake.issued("shell", "monkey"))
        # It did probe pm list first (native check ran and declined).
        self.assertTrue(fake.issued("shell", "pm", "list", "packages"))

    def test_installed_foreground_is_noop_already_native(self):
        fake = FakeAdb(installed=True, foreground=True)
        self._install(fake)
        res = launch_hand_on_device("adb", "SERIAL123", 0, deadline=None)
        self.assertEqual(res["action"], "already_native")
        # No-op: neither monkey nor reverse/forward were issued.
        self.assertFalse(fake.issued("shell", "monkey"))
        self.assertFalse(fake.issued("reverse"))
        self.assertFalse(fake.issued("forward"))

    def test_installed_background_launches_native(self):
        fake = FakeAdb(installed=True, foreground=False, monkey_ok=True)
        self._install(fake)
        res = launch_hand_on_device("adb", "SERIAL123", 0, deadline=None)
        self.assertEqual(res["action"], "launched_native")
        # Native launch set reverse then monkey; it must NOT open a browser tab.
        self.assertTrue(fake.issued("reverse"))
        self.assertTrue(fake.issued("shell", "monkey", "-p", NATIVE_PKG))
        self.assertFalse(fake.issued("forward"))
        self.assertFalse(fake.issued("shell", "am", "start"))

    def test_installed_monkey_failure_is_error(self):
        # Installed but the launch aborts → surfaced as error, not silent fallback.
        fake = FakeAdb(installed=True, foreground=False, monkey_ok=False)
        self._install(fake)
        res = launch_hand_on_device("adb", "SERIAL123", 0, deadline=None)
        self.assertEqual(res["action"], "error")
        self.assertIn("monkey", res["detail"])
        self.assertFalse(fake.issued("shell", "am", "start"))  # no browser fallback

    def test_prefer_native_false_skips_native_probe(self):
        # Opt-out: browser flow only, native never probed.
        fake = FakeAdb(installed=True, foreground=False)
        self._install(fake)
        res = launch_hand_on_device("adb", "SERIAL123", 0, deadline=None,
                                    prefer_native=False)
        self.assertEqual(res["action"], "launched")
        self.assertFalse(fake.issued("shell", "pm", "list", "packages"))
        self.assertFalse(fake.issued("shell", "monkey"))


class TestCdpNavigateLazyImport(unittest.TestCase):
    """The websocket-client import in cdp_navigate is lazy so the module (and the
    native-VR launch path) stay dependency-free. When it is absent (e.g. the SO-101
    cockpit's .venv-so101), the browser fallback must fail with a clear, actionable
    RuntimeError — never a raw ModuleNotFoundError and never at import time."""

    def test_missing_websocket_raises_actionable_runtimeerror(self):
        import builtins
        real_import = builtins.__import__

        def fake_import(name, *a, **k):
            if name == "websocket":
                raise ModuleNotFoundError("No module named 'websocket'")
            return real_import(name, *a, **k)

        builtins.__import__ = fake_import
        try:
            with self.assertRaises(RuntimeError) as cm:
                hand_launch.cdp_navigate({"webSocketDebuggerUrl": "ws://x"},
                                         "http://localhost:8001/hand")
        finally:
            builtins.__import__ = real_import
        self.assertIn("websocket-client", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
