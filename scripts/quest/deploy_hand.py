"""Deploy the ✋ hand-teleop page (/hand) to one or more Quest headsets in one shot.

For each connected Quest, in device order:
  1. adb -s <s> reverse tcp:<srv> tcp:<srv>
        Quest's localhost:<srv> -> this PC's hand server. This is what gives WebXR
        a secure context (localhost) without HTTPS / LAN exposure / a token.
  2. adb -s <s> forward tcp:<cdp> localabstract:chrome_devtools_remote
        PC:<cdp> -> Quest CDP. cdp = <cdp-base> + device index (9223, 9224, ...),
        matching qctl's --cdp so you can drive each headset afterwards.
  3. Reuse the existing browser tab:
        - a localhost:<srv> tab already open  -> CDP Page.navigate it to /hand
          (skipped if it's already on /hand).
        - NO such tab                          -> ONE `am start VIEW` (opens a tab).
     Rationale: `am start VIEW` opens a NEW tab every time, so firing it per deploy
     piles up tabs -> GPU pressure -> jank. We only use it to bootstrap the first
     tab; steady-state updates go through CDP navigate. See qctl.py /
     .claude/memory/feedback_quest_reload_protocol.md.

Usage:
  python scripts/quest/deploy_hand.py                     # all connected devices
  python scripts/quest/deploy_hand.py --serial S1 --serial S2
  python scripts/quest/deploy_hand.py --dry-run           # print the plan, run nothing
  python scripts/quest/deploy_hand.py --port 8001 --cdp-base 9223

Notes:
  - The hand server must already be running (scripts/server.py --offline --real-hand
    --port 8001, or via scripts/teleop_all.ps1). This tool does NOT start it.
  - Needs websocket-client for the CDP navigate step (pip install websocket-client),
    but --dry-run works without it.
"""
from __future__ import annotations
import argparse
import os
import sys

# Windows terminals default to cp932; our summary uses ● / — etc. Force UTF-8 so
# printing never crashes (display may still mojibake in a cp932 console, but the
# tool won't die mid-deploy). See ~/.claude/rules/windows-env.md.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adb_util import find_adb, run_adb, list_devices  # noqa: E402
# CDP helpers are shared with home_server's /quest/launch-hand (single source).
from hand_launch import cdp_pages, cdp_navigate  # noqa: E402


def _plan_for_device(serial: str, idx: int, srv_port: int, cdp_base: int) -> dict:
    """Assemble (but do not run) the adb/CDP steps for one device. Pure — used by
    both the dry-run printer and the executor so they can't drift."""
    cdp_port = cdp_base + idx
    url = f"http://localhost:{srv_port}/hand"
    return {
        "serial": serial,
        "cdp_port": cdp_port,
        "hand_url": url,
        "reverse": ["reverse", f"tcp:{srv_port}", f"tcp:{srv_port}"],
        "forward": ["forward", f"tcp:{cdp_port}", "localabstract:chrome_devtools_remote"],
        "view_intent": ["shell", "am", "start", "-a", "android.intent.action.VIEW",
                        "-d", url, "com.oculus.browser"],
    }


def _adb_str(adb: str, serial: str, args: list[str]) -> str:
    return " ".join([os.path.basename(adb), "-s", serial, *args])


def deploy_device(adb: str, plan: dict, srv_port: int, dry_run: bool) -> dict:
    """Execute (or, if dry_run, only describe) the plan for one device.
    Returns a result dict for the summary."""
    serial, cdp_port, url = plan["serial"], plan["cdp_port"], plan["hand_url"]
    res = {"serial": serial, "cdp_port": cdp_port, "reverse": "-", "forward": "-",
           "tab": "-", "commands": []}

    for key in ("reverse", "forward"):
        res["commands"].append(_adb_str(adb, serial, plan[key]))
        if dry_run:
            res[key] = "(dry-run)"
            continue
        cp = run_adb(adb, plan[key], serial=serial)
        res[key] = "ok" if cp.returncode == 0 else f"FAIL rc={cp.returncode} {cp.stderr.strip()}"

    if dry_run:
        # Describe the tab decision without touching the device.
        res["tab"] = f"(dry-run) navigate localhost:{srv_port} tab -> /hand, else am start VIEW"
        res["commands"].append(_adb_str(adb, serial, plan["view_intent"]) + "   # only if no tab")
        return res

    # Tab decision over CDP.
    host = f"localhost:{srv_port}"
    try:
        pages = cdp_pages(cdp_port)
    except Exception as e:
        res["tab"] = f"CDP list failed ({e}); is forward up?"
        return res
    on_host = [p for p in pages if host in p.get("url", "")]
    if on_host:
        page = on_host[0]
        cur = page.get("url", "")
        if "/hand" in cur:
            res["tab"] = f"already on /hand ({cur})"
        else:
            try:
                cdp_navigate(page, url)
                res["tab"] = f"navigated existing tab -> /hand (was {cur})"
            except Exception as e:
                res["tab"] = f"navigate failed: {e}"
    else:
        # No localhost tab at all -> bootstrap ONE (and only one) via VIEW intent.
        res["commands"].append(_adb_str(adb, serial, plan["view_intent"]))
        cp = run_adb(adb, plan["view_intent"], serial=serial)
        res["tab"] = ("opened new tab via am start VIEW"
                      if cp.returncode == 0 else f"VIEW FAIL rc={cp.returncode} {cp.stderr.strip()}")
    return res


def main():
    ap = argparse.ArgumentParser(description="Deploy /hand to one or more Quest headsets.")
    ap.add_argument("--serial", action="append", default=None,
                    help="target this serial (repeatable). Default: all connected devices.")
    ap.add_argument("--port", type=int, default=8001, help="hand server port (default 8001)")
    ap.add_argument("--cdp-base", type=int, default=9223,
                    help="CDP forward base port; device i gets cdp-base+i (default 9223)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the adb/CDP plan without running anything")
    args = ap.parse_args()

    try:
        adb = find_adb()
    except RuntimeError as e:
        sys.exit(f"FATAL: {e}")

    if args.serial:
        devices = args.serial
    else:
        try:
            devices = list_devices(adb)
        except Exception as e:
            sys.exit(f"FATAL: `adb devices` failed: {e}")
    if not devices:
        sys.exit("no Quest devices found (adb devices empty). Connect + authorize USB, or pass --serial.")

    print(f"adb: {adb}")
    print(f"devices ({len(devices)}): {', '.join(devices)}")
    print(f"hand server port: {args.port}   CDP base: {args.cdp_base}")
    print("=" * 64)

    results = []
    for idx, serial in enumerate(devices):
        plan = _plan_for_device(serial, idx, args.port, args.cdp_base)
        results.append(deploy_device(adb, plan, args.port, args.dry_run))

    # Human-readable summary.
    print()
    for r in results:
        print(f"● {r['serial']}  (CDP :{r['cdp_port']})")
        print(f"    reverse tcp:{args.port} : {r['reverse']}")
        print(f"    forward tcp:{r['cdp_port']} : {r['forward']}")
        print(f"    tab                 : {r['tab']}")
    print()
    if args.dry_run:
        print("dry-run — commands that WOULD run:")
        for r in results:
            for c in r["commands"]:
                print(f"    {c}")
    else:
        print(f"done. Each headset: open the tab, tap 「VR 開始」, show your right hand.")
        print(f"Drive a specific headset afterwards, e.g.:")
        for r in results:
            print(f"    python scripts/quest/qctl.py check --port {args.port} --cdp {r['cdp_port']}")


if __name__ == "__main__":
    main()
