#!/usr/bin/env python3
"""✋ HAND first-line triage — tell "port opens" apart from "MCU is alive".

Why this exists (2026-07-17 incident): the hand's M5 ATOM Lite talks over a
SEPARATE FTDI USB-serial adapter (VID 0403), powered independently of the ATOM
(ATOM = USB-C, FTDI = its own USB). So the COM port opens fine even when the
ATOM is unpowered — hand_driver / server.py report "connected: true", and the
teleop 't' command is silent by design, so the UI looked healthy while the real
hand moved 0 mm.

This tool sends the firmware's ACK-producing commands and actually reads the
replies, then prints a plain-language verdict:

  1. autodetect the FTDI port (reuses hand_driver.find_hand_port)
  2. open + DTR reset, wait out the boot, capture the boot banner
  3. send "tspd 40 6"  -> expect "tele step=..." ACK   (does NOT move servos)
  4. send "open"       -> expect "open" ACK             (MOVES servos; --no-open to skip)

Safe to run while the MCU is dead: nothing moves without a live ATOM. Read-only
with respect to config — it only tunes the teleop ramp (which the driver does
anyway on connect).

Usage:
    python scripts/hand_diag.py                 # autodetect, full check
    python scripts/hand_diag.py --port COM9      # force a port
    python scripts/hand_diag.py --no-open        # skip the servo-moving test
"""
from __future__ import annotations

import argparse
import io
import sys
import time
from pathlib import Path

# cp932 Windows terminals mangle the emoji/JP output; force UTF-8 stdout.
try:
    sys.stdout.reconfigure(encoding="utf-8")  # py3.7+
except Exception:  # pragma: no cover - very old python / redirected stream
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
    except Exception:
        pass

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "hand"))  # ✋ HAND driver lives here (separate robot)

from hand_driver import (  # noqa: E402
    find_hand_port, BAUD, PING_CMD, PING_ACK_TOKEN,
)

try:
    import serial  # noqa: E402
except ImportError:
    serial = None

RESET_WAIT_S = 2.3   # ATOM auto-resets on serial open (DTR); wait for boot
READ_WINDOW_S = 1.0  # how long to wait for a command's ACK


def _read_for(ser, seconds: float) -> str:
    """Accumulate whatever the port emits over the next `seconds`."""
    end = time.monotonic() + seconds
    buf = b""
    while time.monotonic() < end:
        n = getattr(ser, "in_waiting", 0)
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace").strip()


def main() -> int:
    ap = argparse.ArgumentParser(description="Hand MCU liveness triage")
    ap.add_argument("--port", default=None, help="COM port (default: autodetect)")
    ap.add_argument("--no-open", action="store_true",
                    help="skip the 'open' test (which physically moves the servos)")
    args = ap.parse_args()

    print("=== ✋ hand_diag — MCU liveness triage ===")

    if serial is None:
        print("✗ pyserial 未インストール。`pip install pyserial` を実行してから再試行。")
        return 2

    port = args.port or find_hand_port()
    if not port:
        print("✗ ハンドの COM ポートが見つからない（FTDI/Arduino が一つも列挙されない）。")
        print("  → USB ケーブル・FTDI アダプタの接続を確認。`python -m serial.tools.list_ports -v`")
        return 2
    print(f"• ポート: {port} @ {BAUD} baud" + ("（自動検出）" if not args.port else "（指定）"))

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2, write_timeout=1.0)
    except Exception as e:
        print(f"✗ ポート {port} を開けない: {e}")
        print("  → 他プロセス（server.py / Arduino IDE のシリアルモニタ）が掴んでいないか確認。")
        return 2
    print(f"✓ ポート {port} は開けた（= FTDI アダプタは見えている。MCU 生存はこれだけでは不明）")

    try:
        # 1) DTR reset + boot banner ------------------------------------------
        ser.dtr = True
        ser.reset_input_buffer()
        print(f"• DTR リセット送出。ブート待ち {RESET_WAIT_S}s …")
        time.sleep(RESET_WAIT_S)
        banner = _read_for(ser, 0.3)
        banner_ok = "ready" in banner.lower()
        if banner_ok:
            print(f"✓ ブートバナー受信: {banner!r}")
        else:
            print(f"△ ブートバナー無し（受信: {banner!r}）"
                  "  ← ATOM 無給電だとここが空になる")

        # 2) tspd ACK (non-moving) --------------------------------------------
        ser.reset_input_buffer()
        ser.write((PING_CMD + "\n").encode("ascii"))
        tspd_reply = _read_for(ser, READ_WINDOW_S)
        tspd_ok = PING_ACK_TOKEN in tspd_reply
        if tspd_ok:
            print(f"✓ '{PING_CMD}' に ACK: {tspd_reply!r}（サーボは動かさない生存確認）")
        else:
            print(f"✗ '{PING_CMD}' に応答なし（受信: {tspd_reply!r}）")

        # 3) open ACK (moving) ------------------------------------------------
        open_ok = None
        if args.no_open:
            print("• 'open' テストは --no-open によりスキップ（サーボを動かさない）")
        else:
            print("• 'open' 送出（MCU が生きていれば全指が開く。~1.5s ブロッキング）…")
            ser.reset_input_buffer()
            ser.write(b"open\n")
            open_reply = _read_for(ser, 2.5)
            open_ok = "open" in open_reply.lower()
            if open_ok:
                print(f"✓ 'open' に ACK: {open_reply!r}（指が開いたはず）")
            else:
                print(f"✗ 'open' に応答なし（受信: {open_reply!r}）")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    # --- verdict -------------------------------------------------------------
    print("\n--- 判定 ---")
    alive = tspd_ok  # the tspd ACK is the authoritative, non-moving signal
    if alive:
        print("✅ MCU は生きている（ACK 応答あり）。ハンドは稼働可能。")
        rc = 0
    else:
        print("🔴 FTDI は見えるが MCU は沈黙。ポートが開けても ATOM は応答していない。")
        print("   最有力: ATOM Lite の USB-C 給電が入っていない/抜けている。")
        print("   確認手順:")
        print("     1. ATOM Lite の USB-C ケーブルが PC か給電源に挿さっているか")
        print("     2. ATOM 本体の LED が点いているか（ファーム更新後は boot=緑）")
        print("     3. 8Servos Unit の外部 5V 端子台に給電があるか（指を動かすのに必要）")
        print("     4. 給電後にもう一度: python scripts/hand_diag.py")
        rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
