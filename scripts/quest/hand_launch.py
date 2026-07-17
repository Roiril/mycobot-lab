"""Shared logic to open / refresh the ✋ hand-teleop page (/hand) on Quest headsets.

Consumers:
  - scripts/home_server.py   POST /quest/launch-hand  (one-click from the launcher)
  - scripts/quest/deploy_hand.py  (CLI; imports the CDP helpers here to avoid dupes)

⚠ Steady-state rule (must never be violated): when a localhost:<port> tab already
exists, NEVER `am start` (VIEW *or* Launcher) — every `am start` spawns a NEW tab
in the Oculus Browser, which piles up tabs → GPU pressure → jank. Refresh the
existing tab with CDP Page.navigate instead. `am start` is used ONLY to bootstrap
the very first tab when the browser isn't reachable over CDP / has no such tab.
See .claude/memory/feedback_quest_reload_protocol.md and qctl.py.

Bootstrap here uses the project's own launcher APK
(jp.mycobotlab.handteleop/.LauncherActivity), which opens exactly one Oculus
Browser tab at http://localhost:8001/hand. We wake the headset first so the
launch actually renders.
"""
from __future__ import annotations
import json
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adb_util import find_adb, run_adb, list_device_states  # noqa: E402

HAND_PATH = "/hand"
LAUNCHER_COMPONENT = "jp.mycobotlab.handteleop/.LauncherActivity"

# Native Unity VR app ("ロボットハンド操作VR"). Unlike the WebXR /hand page, this
# app enters immersive VR on launch (no in-headset tap, no non-worn freeze), so
# `am start` alone puts the headset in "worn = already operating" state. Preferred
# over the browser flow whenever it is installed.
NATIVE_PKG = "com.mycobot.handteleop"


# ---- pure branch logic (parses adb text output; no device I/O → unit-testable) ----

def native_installed(pm_list_output: str, pkg: str = NATIVE_PKG) -> bool:
    """True if `adb shell pm list packages` output contains an EXACT package match.

    pm prints one `package:<name>` per line (with `=<apk>` appended under `-f`).
    A substring filter (`pm list packages <pkg>`) can also return near-namesakes
    like `<pkg>.debug`, so we match the full name, not a prefix."""
    prefix = f"package:{pkg}"
    for line in pm_list_output.splitlines():
        line = line.strip()
        if line == prefix or line.startswith(prefix + "="):
            return True
    return False


def native_is_foreground(dumpsys_output: str, pkg: str = NATIVE_PKG) -> bool:
    """True if the resumed (foreground) activity belongs to `pkg`.

    Parses lines from `dumpsys activity activities` that name the resumed activity,
    e.g. `mResumedActivity: ActivityRecord{... com.mycobot.handteleop/.UnityPlayer}`.
    Only mResumedActivity / topResumedActivity lines count — the package can appear
    in the dump (paused/stopped) without being in front."""
    needle = pkg + "/"
    for line in dumpsys_output.splitlines():
        if ("mResumedActivity" in line or "topResumedActivity" in line) and needle in line:
            return True
    return False


def decide_native_action(installed: bool, foreground: bool) -> str:
    """Map (installed, foreground) → what the native path should do.

    "browser"        — native app not installed → fall back to the WebXR flow.
    "already_native" — installed and already in front → no-op.
    "launched_native"— installed but not in front → monkey-launch it.
    """
    if not installed:
        return "browser"
    if foreground:
        return "already_native"
    return "launched_native"


def hand_url(srv_port: int) -> str:
    return f"http://localhost:{srv_port}{HAND_PATH}"


# ---- CDP helpers (parameterized by port; no module globals so 2 devices coexist) ----

def cdp_pages(cdp_port: int, timeout: float = 4.0) -> list[dict]:
    with urllib.request.urlopen(f"http://localhost:{cdp_port}/json/list", timeout=timeout) as r:
        data = json.load(r)
    return [p for p in data if p.get("type") == "page"]


def cdp_navigate(page: dict, url: str, timeout: float = 6.0) -> None:
    import websocket  # lazy: only needed when we actually navigate
    ws = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=timeout,
                                     suppress_origin=True)
    try:
        ws.send(json.dumps({"id": 1, "method": "Page.navigate", "params": {"url": url}}))
        try:
            ws.recv()
        except Exception:
            pass
    finally:
        ws.close()


# ---- one-shot launch (navigate existing tab, else bootstrap one) ----

def _remaining(deadline: float | None) -> float:
    return 1e9 if deadline is None else deadline - time.monotonic()


def _clamp(default_to: float, deadline: float | None) -> float:
    return min(default_to, max(0.5, _remaining(deadline)))


def _adb(adb: str, args: list[str], serial: str, deadline: float | None,
         default_to: float) -> subprocess.CompletedProcess:
    """run_adb with its timeout clamped to the remaining budget."""
    return run_adb(adb, args, serial=serial, timeout=_clamp(default_to, deadline))


def _native_query(adb: str, serial: str, deadline: float | None) -> str | None:
    """Return "already_native" / "launched_native" if the native VR app path
    handled this device, else None to defer to the browser flow.

    Best-effort: any adb hiccup while probing → None (browser fallback), so a flaky
    `pm list` never blocks the proven WebXR path. A genuine launch FAILURE (app is
    installed but monkey aborts) is surfaced by the caller as an error, not hidden.
    """
    try:
        cp = _adb(adb, ["shell", "pm", "list", "packages", NATIVE_PKG], serial, deadline, 6.0)
    except Exception:
        return None
    if cp.returncode != 0 or not native_installed(cp.stdout or ""):
        return "browser"  # not installed (or pm failed) → browser flow

    # Installed. Is it already the foreground activity? Then we're done.
    try:
        cp = _adb(adb, ["shell", "dumpsys", "activity", "activities"], serial, deadline, 8.0)
        if cp.returncode == 0 and native_is_foreground(cp.stdout or ""):
            return "already_native"
    except Exception:
        pass  # indeterminate foreground state → just (re)launch below
    return "launched_native"


def _launch_native(adb: str, serial: str, srv_port: int, deadline: float | None,
                   res: dict) -> dict:
    """reverse tcp:<srv> then monkey-launch the native VR app (activity-name
    independent). Mutates + returns `res`."""
    try:
        cp = _adb(adb, ["reverse", f"tcp:{srv_port}", f"tcp:{srv_port}"], serial, deadline, 6.0)
        if cp.returncode != 0:
            res["detail"] = f"adb reverse failed: rc={cp.returncode} {cp.stderr.strip()}"
            return res
        _adb(adb, ["shell", "input", "keyevent", "KEYCODE_WAKEUP"], serial, deadline, 6.0)
        cp = _adb(adb, ["shell", "monkey", "-p", NATIVE_PKG,
                        "-c", "android.intent.category.LAUNCHER", "1"],
                  serial, deadline, 8.0)
        combined = (cp.stdout or "") + (cp.stderr or "")
        if cp.returncode == 0 and "aborted" not in combined and "No activities" not in combined:
            res["action"] = "launched_native"
            res["detail"] = "woke headset + launched native VR app (immersive on start)"
        else:
            res["detail"] = f"monkey launch failed: rc={cp.returncode} {combined.strip()}"
    except subprocess.TimeoutExpired:
        res["detail"] = "native app launch timed out"
    except Exception as e:
        res["detail"] = f"native app launch error: {e}"
    return res


def launch_hand_on_device(adb: str, serial: str, idx: int, *,
                          srv_port: int = 8001, cdp_base: int = 9223,
                          deadline: float | None = None,
                          prefer_native: bool = True) -> dict:
    """Open the ✋ hand teleop on one headset, preferring the native VR app.

    action is one of:
      already_native  — native VR app is already in the foreground (no-op)
      launched_native — native VR app was monkey-launched (immersive on start)
      already         — a localhost tab was already on /hand (no-op, browser flow)
      navigated       — an existing localhost tab was Page.navigate'd to /hand
      launched        — no reachable tab → woke headset + am start LauncherActivity
      error           — something failed (detail explains what)

    Native path (prefer_native, default): if `com.mycobot.handteleop` is installed
    we monkey-launch it (activity-name independent) — the app enters VR on start, so
    the operator just has to put the headset on. Not installed → browser flow below.
    """
    cdp_port = cdp_base + idx
    url = hand_url(srv_port)
    res = {"serial": serial, "cdp_port": cdp_port, "action": "error",
           "detail": "", "url": url}

    # 0. Native VR app preferred path (falls through to browser flow when absent).
    if prefer_native:
        decision = _native_query(adb, serial, deadline)
        if decision == "already_native":
            res["action"] = "already_native"
            res["detail"] = "native VR app already in foreground"
            return res
        if decision == "launched_native":
            return _launch_native(adb, serial, srv_port, deadline, res)
        # decision is "browser" (not installed) or None (probe flaked) → browser flow.

    # 1. reverse (localhost:8001 → this PC's hand server) + forward (CDP).
    try:
        cp = _adb(adb, ["reverse", f"tcp:{srv_port}", f"tcp:{srv_port}"], serial, deadline, 6.0)
        if cp.returncode != 0:
            res["detail"] = f"adb reverse failed: rc={cp.returncode} {cp.stderr.strip()}"
            return res
        cp = _adb(adb, ["forward", f"tcp:{cdp_port}", "localabstract:chrome_devtools_remote"],
                  serial, deadline, 6.0)
        if cp.returncode != 0:
            res["detail"] = f"adb forward failed: rc={cp.returncode} {cp.stderr.strip()}"
            return res
    except subprocess.TimeoutExpired:
        res["detail"] = "adb reverse/forward timed out"
        return res
    except Exception as e:
        res["detail"] = f"adb reverse/forward error: {e}"
        return res

    # 2. Inspect existing tabs over CDP. Unreachable ⇒ browser not up ⇒ bootstrap.
    host = f"localhost:{srv_port}"
    pages = None
    try:
        pages = cdp_pages(cdp_port, timeout=_clamp(4.0, deadline))
    except Exception:
        pages = None

    if pages is not None:
        on_host = [p for p in pages if host in p.get("url", "")]
        if on_host:
            page = on_host[0]
            cur = page.get("url", "")
            if HAND_PATH in cur:
                res["action"] = "already"
                res["detail"] = f"already on {cur}"
                return res
            try:
                cdp_navigate(page, url, timeout=_clamp(6.0, deadline))
                res["action"] = "navigated"
                res["detail"] = f"navigated existing tab → {HAND_PATH} (was {cur})"
            except Exception as e:
                res["detail"] = f"CDP navigate failed: {e}"
            return res
        # CDP reachable but no localhost tab → fall through to bootstrap one.

    # 3. Bootstrap via WAKEUP + LauncherActivity (opens exactly one tab).
    try:
        _adb(adb, ["shell", "input", "keyevent", "KEYCODE_WAKEUP"], serial, deadline, 6.0)
        cp = _adb(adb, ["shell", "am", "start", "-n", LAUNCHER_COMPONENT], serial, deadline, 6.0)
        combined = (cp.stdout or "") + (cp.stderr or "")
        if cp.returncode == 0 and "Error" not in combined:
            res["action"] = "launched"
            res["detail"] = "woke headset + launched hand-teleop app (new tab)"
        else:
            res["detail"] = f"am start failed: rc={cp.returncode} {combined.strip()}"
    except subprocess.TimeoutExpired:
        res["detail"] = "wake / am start timed out"
    except Exception as e:
        res["detail"] = f"launcher bootstrap error: {e}"
    return res


def launch_hand_all(*, srv_port: int = 8001, cdp_base: int = 9223,
                    deadline: float | None = None) -> dict:
    """Enumerate connected Quests and open /hand on each.

    Offline devices get one `adb reconnect offline` + a 3 s wait + a re-list before
    we give up on them. Returns {ok, devices:[per-device result], offline:[serials]}.
    Count-independent: works for 0, 1, or N headsets.
    """
    try:
        adb = find_adb()
    except Exception as e:
        return {"ok": False, "error": f"adb が見つかりません: {e}", "devices": [], "offline": []}

    try:
        states = list_device_states(adb)
    except Exception as e:
        return {"ok": False, "error": f"adb devices が失敗: {e}", "devices": [], "offline": []}

    # Offline recovery: one reconnect attempt, wait, re-list.
    if any(st == "offline" for _, st in states):
        try:
            run_adb(adb, ["reconnect", "offline"], timeout=_clamp(6.0, deadline))
        except Exception:
            pass
        time.sleep(min(3.0, max(0.0, _remaining(deadline))))
        try:
            states = list_device_states(adb)
        except Exception:
            pass

    online = [s for s, st in states if st == "device"]
    offline = [s for s, st in states if st != "device"]
    if not online:
        msg = "接続中の Quest がありません（adb devices が空）。"
        if offline:
            msg = f"Quest が offline のままです（{', '.join(offline)}）。USB を挿し直してください。"
        return {"ok": False, "error": msg, "devices": [], "offline": offline}

    results = []
    for idx, serial in enumerate(online):
        results.append(launch_hand_on_device(adb, serial, idx, srv_port=srv_port,
                                              cdp_base=cdp_base, deadline=deadline))
    ok = any(r["action"] in ("navigated", "already", "launched",
                             "launched_native", "already_native") for r in results)
    return {"ok": ok, "devices": results, "offline": offline}
