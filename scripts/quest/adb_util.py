"""Shared adb helpers for the Quest dev tooling (qctl.py, deploy_hand.py).

Locating adb on this machine is fiddly (SideQuest bundles its own, Unity bundles
another, sometimes it's on PATH). Centralize resolution + device enumeration so
both tools agree on which adb and which serials.
"""
from __future__ import annotations
import os
import shutil
import subprocess

# Known bundled adb locations on this machine (see docs/QUEST_DEV.md).
_CANDIDATES = [
    r"C:\Users\kouga\AppData\Local\Programs\SideQuest\resources\app.asar.unpacked\build\platform-tools\adb.exe",
    r"C:\Program Files\Unity\Hub\Editor\2022.3.62f2\Editor\Data\PlaybackEngines\AndroidPlayer\SDK\platform-tools\adb.exe",
    os.path.expanduser(r"~\AppData\Local\Android\Sdk\platform-tools\adb.exe"),
]


def find_adb() -> str:
    """Return a usable adb path. Order: $ADB env, PATH, known bundles.
    Raises RuntimeError with guidance if none is found."""
    env = os.environ.get("ADB")
    if env and os.path.isfile(env):
        return env
    on_path = shutil.which("adb")
    if on_path:
        return on_path
    for c in _CANDIDATES:
        if os.path.isfile(c):
            return c
    raise RuntimeError(
        "adb not found. Set $ADB, put adb on PATH, or install SideQuest. "
        "See docs/QUEST_DEV.md for the bundled path."
    )


def run_adb(adb: str, args: list[str], serial: str | None = None,
            check: bool = False, timeout: float = 20.0) -> subprocess.CompletedProcess:
    """Run `adb [-s serial] <args>`. Returns the CompletedProcess (stdout/stderr text)."""
    cmd = [adb]
    if serial:
        cmd += ["-s", serial]
    cmd += args
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=check)


# Quest headsets identify as these products in `adb devices -l`
# (Quest 3 = eureka, Quest 2 = hollywood, Quest Pro = seacliff). The user's PC
# often has non-Quest Android devices (phones) attached at the same time; the
# hand-teleop launch flow must never touch those (2026-07-17: a launch-all pass
# hit 3 attached phones — harmless no-op, but wrong scope).
QUEST_PRODUCTS = {"eureka", "hollywood", "seacliff", "monterey"}


def list_device_states(adb: str, quest_only: bool = True) -> list[tuple[str, str]]:
    """(serial, state) for attached devices, including offline/unauthorized.
    state is adb's raw word: 'device', 'offline', 'unauthorized', ...
    quest_only filters to Quest headsets via the product field of `adb devices -l`
    (offline devices report no product and are kept — resolving them is the
    caller's job and a Quest can be offline too)."""
    cp = run_adb(adb, ["devices", "-l"])
    out = []
    for line in cp.stdout.splitlines()[1:]:  # skip "List of devices attached"
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        serial, state = parts[0], parts[1]
        product = next((p.split(":", 1)[1] for p in parts[2:]
                        if p.startswith("product:")), None)
        if quest_only and product is not None and product not in QUEST_PRODUCTS:
            continue
        out.append((serial, state))
    return out


def list_devices(adb: str, quest_only: bool = True) -> list[str]:
    """Serials of devices currently in the `device` state (excludes offline/unauthorized)."""
    return [s for s, st in list_device_states(adb, quest_only=quest_only) if st == "device"]
