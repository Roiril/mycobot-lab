"""Python driver for the Hiwonder 5-finger robot hand (Arduino Uno firmware).

This is the ✋ HAND — a SEPARATE robot from the 🦾 myCobot ARM. Different MCU
(Arduino Uno, not M5Stack), different COM port, different baud (9600, not
115200), different power (external 6V). See hand/HANDOFF.md and CLAUDE.md.

Talks to hand/hand_control/hand_control.ino over serial:
  - normalized teleop:  set_bends([b0..b4])  bend 0=open/straight, 1=closed/curled
  - raw microseconds:   set_fingers_us([u0..u4])
  - presets:            open() / close() / neutral()

Teleop uses the firmware's non-blocking "t u0 u1 u2 u3 u4" command so a ~25Hz
stream from hand-tracking never piles up (the firmware ramps toward the latest
target between serial reads).

Usage:
    from hand_driver import HandDriver, VirtualHand, make_hand
    hand = make_hand()          # autodetect Arduino, or VirtualHand if none
    hand.set_bends([0,0,0,0,0]) # fully open
    hand.close(); hand.open()
    hand.shutdown()
"""
from __future__ import annotations

import threading
import time
from typing import Optional, Sequence

try:
    import serial  # pyserial (already a project dependency)
    from serial.tools import list_ports
except ImportError:  # allow import in environments without pyserial (offline UI dev)
    serial = None
    list_ports = None

NUM_FINGERS = 5
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]  # 0..4

# Per-finger microsecond endpoints. MUST mirror hand_control.ino. The firmware
# is the source of truth for the safe range; these are the same values so the
# host clamps identically and the normalized bend maps to real travel.
#                       finger:  0(thumb) 1     2     3     4
MIN_US  = [1000, 600, 600, 600, 600]
MAX_US  = [2000, 2400, 2400, 2400, 2400]
OPEN_US = [2000, 2400, 2400, 2400, 2400]   # bend = 0.0 (extended)
CLOSE_US = [1000, 600, 600, 600, 600]      # bend = 1.0 (curled)

BAUD = 9600
RESET_WAIT_S = 2.3   # Arduino auto-resets on serial open (DTR); wait for boot
DEFAULT_PORT_HINT = None

# Teleop ramp tuning sent to the firmware on connect via "tspd <us> <ms>".
# Larger us / smaller ms = faster follow (helps the thumb keep up). Firmware
# defaults are 25us/8ms; these ~2x the slew rate. Re-applied every connect.
TELEOP_STEP_US = 40
TELEOP_STEP_MS = 6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def bend_to_us(finger: int, bend: float) -> int:
    """Map normalized bend [0=open .. 1=closed] to microseconds for one finger,
    interpolating between that finger's OPEN_US and CLOSE_US endpoints."""
    b = _clamp(float(bend), 0.0, 1.0)
    us = OPEN_US[finger] + b * (CLOSE_US[finger] - OPEN_US[finger])
    return int(round(_clamp(us, MIN_US[finger], MAX_US[finger])))


def clamp_us(finger: int, us: float) -> int:
    return int(round(_clamp(float(us), MIN_US[finger], MAX_US[finger])))


# --- Arduino port autodetect -------------------------------------------------
# The hand's Arduino Uno must NOT be confused with the myCobot's CH9102 serial.
# Match Arduino signatures (VID 0x2341 = Arduino LLC; CH340/USB-SERIAL clones).
# Explicitly reject the CH9102 description used by the arm's adapter.
_ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86}  # Arduino LLC, Arduino SRL, QinHeng(CH340)
_ARM_ADAPTER_HINTS = ("CH9102",)          # myCobot 320 USB-serial — never the hand


def find_hand_port() -> Optional[str]:
    """Return the most likely Arduino Uno COM port for the hand, or None."""
    if list_ports is None:
        return None
    candidates = []
    for p in list_ports.comports():
        desc = (p.description or "") + " " + (getattr(p, "product", "") or "")
        if any(h in desc for h in _ARM_ADAPTER_HINTS):
            continue  # this is the arm's adapter, skip
        vid = getattr(p, "vid", None)
        score = 0
        if vid in _ARDUINO_VIDS:
            score += 10
        d = desc.lower()
        if "arduino" in d:
            score += 8
        if "ch340" in d or "usb-serial" in d or "usb serial" in d:
            score += 3
        if score > 0:
            candidates.append((score, p.device))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][1]


class HandBase:
    """Shared API for real + virtual hands. Bends: 0=open .. 1=closed."""

    NUM_FINGERS = NUM_FINGERS
    FINGER_NAMES = FINGER_NAMES

    def __init__(self):
        self.offline = True
        self.port: Optional[str] = None
        self._cur_us = [1500] * NUM_FINGERS  # last commanded targets
        self._lock = threading.Lock()
        self._last_send_mono = 0.0

    # --- high-level ---
    def set_bends(self, bends: Sequence[float]) -> list[int]:
        if len(bends) != NUM_FINGERS:
            raise ValueError(f"bends must be length {NUM_FINGERS}")
        us = [bend_to_us(i, bends[i]) for i in range(NUM_FINGERS)]
        return self.set_fingers_us(us)

    def set_fingers_us(self, us: Sequence[int]) -> list[int]:
        if len(us) != NUM_FINGERS:
            raise ValueError(f"us must be length {NUM_FINGERS}")
        clamped = [clamp_us(i, us[i]) for i in range(NUM_FINGERS)]
        self._send_teleop(clamped)
        self._cur_us = clamped
        return clamped

    def set_finger_us(self, finger: int, us: int) -> int:
        if not 0 <= finger < NUM_FINGERS:
            raise ValueError("finger out of range")
        target = list(self._cur_us)
        target[finger] = clamp_us(finger, us)
        self.set_fingers_us(target)
        return target[finger]

    def open(self) -> list[int]:
        return self.set_fingers_us(list(OPEN_US))

    def close(self) -> list[int]:
        return self.set_fingers_us(list(CLOSE_US))

    def neutral(self) -> list[int]:
        return self.set_fingers_us([1500] * NUM_FINGERS)

    def status(self) -> dict:
        return {
            "connected": not self.offline,
            "offline": self.offline,
            "port": self.port,
            "baud": BAUD,
            "num_fingers": NUM_FINGERS,
            "finger_names": FINGER_NAMES,
            "cur_us": list(self._cur_us),
            "min_us": list(MIN_US),
            "max_us": list(MAX_US),
            "open_us": list(OPEN_US),
            "close_us": list(CLOSE_US),
        }

    # --- subclass hooks ---
    def _send_teleop(self, us: Sequence[int]) -> None:
        raise NotImplementedError

    def shutdown(self) -> None:
        pass


class VirtualHand(HandBase):
    """No-hardware hand for UI/offline development. Records last targets only."""

    def __init__(self):
        super().__init__()
        self.offline = True
        self.port = None

    def _send_teleop(self, us: Sequence[int]) -> None:
        pass  # nothing to do; _cur_us is updated by the base


class HandDriver(HandBase):
    """Real hand over serial. Opens the Arduino, waits out the DTR reset, then
    streams non-blocking 't' teleop lines."""

    def __init__(self, port: Optional[str] = None, *, baud: int = BAUD,
                 reset_wait_s: float = RESET_WAIT_S):
        super().__init__()
        if serial is None:
            raise RuntimeError("pyserial not installed; cannot open hand")
        self.port = port or find_hand_port()
        if not self.port:
            raise RuntimeError("Arduino (hand) port not found; pass port= or check USB")
        self._ser = serial.Serial(self.port, baud, timeout=0.2, write_timeout=1.0)
        self._ser.dtr = True  # ensure reset, then wait for bootloader + setup()
        time.sleep(reset_wait_s)
        try:
            self._greeting = self._ser.read_all().decode("utf-8", "replace").strip()
        except Exception:
            self._greeting = ""
        self.offline = False
        # speed up the non-blocking teleop ramp (helps the thumb keep up)
        self._write_line(f"tspd {TELEOP_STEP_US} {TELEOP_STEP_MS}")
        time.sleep(0.05)
        self.neutral()  # land at a known pose

    def _write_line(self, line: str) -> None:
        with self._lock:
            if self._ser is None:
                return
            self._ser.write((line + "\n").encode("ascii"))

    def _send_teleop(self, us: Sequence[int]) -> None:
        # firmware: "t u0 u1 u2 u3 u4" — non-blocking, silent (no ack to read)
        self._write_line("t " + " ".join(str(int(u)) for u in us))

    def send_raw(self, line: str) -> str:
        """Send an arbitrary command and return whatever the firmware echoes
        (for open/close/n/spd which DO ack). Used for diagnostics."""
        self._write_line(line)
        time.sleep(0.05)
        with self._lock:
            return self._ser.read_all().decode("utf-8", "replace").strip() if self._ser else ""

    def shutdown(self) -> None:
        try:
            self.neutral()
            time.sleep(0.1)
        except Exception:
            pass
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                finally:
                    self._ser = None
        self.offline = True


def make_hand(port: Optional[str] = None, *, force_virtual: bool = False) -> HandBase:
    """Best-effort: return a real HandDriver if an Arduino is found, else a
    VirtualHand. Never raises on missing hardware (server stays up)."""
    if force_virtual or serial is None:
        return VirtualHand()
    try:
        return HandDriver(port=port)
    except Exception as e:  # noqa: BLE001 — degrade gracefully to virtual
        print(f"[hand] no hardware ({e}); using VirtualHand")
        return VirtualHand()


if __name__ == "__main__":
    # Quick manual test: cycle open/close on whatever is connected.
    import sys
    h = make_hand(port=sys.argv[1] if len(sys.argv) > 1 else None)
    print("status:", h.status())
    if not h.offline:
        print("open");  h.open();  time.sleep(1.5)
        print("close"); h.close(); time.sleep(1.5)
        print("wave bends");
        for k in range(20):
            b = (k % 2)
            h.set_bends([b] * NUM_FINGERS); time.sleep(0.15)
        h.neutral()
    h.shutdown()
