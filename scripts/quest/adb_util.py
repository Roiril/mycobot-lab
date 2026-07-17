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


def list_devices(adb: str) -> list[str]:
    """Serials of devices currently in the `device` state (excludes offline/unauthorized)."""
    cp = run_adb(adb, ["devices"])
    out = []
    for line in cp.stdout.splitlines()[1:]:  # skip "List of devices attached"
        line = line.strip()
        if not line or "\t" not in line:
            continue
        serial, state = line.split("\t", 1)
        if state.strip() == "device":
            out.append(serial.strip())
    return out
