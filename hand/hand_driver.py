"""Python driver for the Hiwonder 5-finger robot hand.

This is the ✋ HAND — a SEPARATE robot from the 🦾 myCobot ARM and the SO-101.
Controller is an M5Stack ATOM Lite (ESP32) driving an M5 8Servos Unit over I2C.
Firmware: hand/hand_control_atom/hand_control_atom.ino (serial-compatible with
the legacy Uno firmware hand/hand_control/hand_control.ino). Different COM port,
baud 9600, external 5V servo supply. See hand/HANDOFF.md and CLAUDE.md.

Talks to the controller over serial (same protocol on either firmware):
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

import logging
import threading
import time
from typing import Optional, Sequence

try:
    import serial  # pyserial (already a project dependency)
    from serial.tools import list_ports
except ImportError:  # allow import in environments without pyserial (offline UI dev)
    serial = None
    list_ports = None

log = logging.getLogger("hand")

NUM_FINGERS = 5
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]  # 0..4

# Per-finger microsecond endpoints. MUST mirror hand_control.ino. The firmware
# is the source of truth for the safe range; these are the same values so the
# host clamps identically and the normalized bend maps to real travel.
#                       finger:  0(thumb) 1     2     3     4
# thumb MIN temporarily 500 (servo floor) to probe the true curl limit; revert
# to the safe value once found. Must match firmware MIN_US. 2026-06-05.
MIN_US  = [500, 600, 600, 600, 600]
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

# --- MCU liveness ping ---------------------------------------------------------
# The hand's ATOM Lite talks over a SEPARATE FTDI USB-serial adapter, powered
# independently (ATOM = USB-C, FTDI = its own USB). So the COM port opens fine
# even when the ATOM is unpowered — serial.Serial() succeeds and 't' teleop
# lines (which are silent by design) vanish into the void while the UI shows
# "connected". To tell "port openable" from "MCU alive" we send an ACK-producing
# command and actually read the reply.
#
# The ping command IS "tspd <us> <ms>" — the same ramp-tuning command already
# sent on connect — so pinging doubles as (re)applying the teleop tuning. The
# firmware answers "tele step=<us>us / <ms>ms"; teleop 't' lines produce no
# reply, so a concurrent follow stream can't be mistaken for the ACK.
PING_CMD = f"tspd {TELEOP_STEP_US} {TELEOP_STEP_MS}"
PING_ACK_TOKEN = "tele step"   # substring of the firmware's tspd reply
PING_TIMEOUT_S = 0.5           # max wait for the ACK before declaring silence
PING_INTERVAL_S = 10.0         # re-ping at most this often from status()


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


# --- hand controller port autodetect -----------------------------------------
# The hand's controller (M5Stack ATOM Lite, ESP32-PICO-D4) must NOT be confused
# with the OTHER robots' serial adapters sharing this PC:
#   - myCobot 320 arm : CH9102 (VID 1A86 / PID 55D4)
#   - SO-101 follower  : CH343  (VID 1A86 / PID 55D3)   <- shares VID 1A86!
# The ATOM Lite enumerates as an FTDI USB-serial (VID 0403, "USB Serial Port").
# So: accept FTDI 0403 + legacy Arduino VIDs, and REJECT the arm/SO-101 adapters
# by description (CH9102 / CH343) so VID 1A86 can't accidentally match them.
# Confirm a port is really the ATOM with esptool chip-id if ever in doubt.
_HAND_VIDS = {0x0403, 0x2341, 0x2A03, 0x1A86}  # FTDI(ATOM), Arduino LLC/SRL, QinHeng(CH340)
_OTHER_ROBOT_HINTS = ("CH9102", "CH343")        # myCobot arm / SO-101 — never the hand


def find_hand_port() -> Optional[str]:
    """Return the most likely COM port for the hand controller, or None.

    Targets the M5Stack ATOM Lite (FTDI USB-serial). Falls back to legacy
    Arduino signatures. Explicitly skips the myCobot (CH9102) and SO-101 (CH343)
    adapters so they are never picked as the hand.
    """
    if list_ports is None:
        return None
    candidates = []
    for p in list_ports.comports():
        desc = (p.description or "") + " " + (getattr(p, "product", "") or "")
        if any(h in desc for h in _OTHER_ROBOT_HINTS):
            continue  # this is the arm's / SO-101's adapter, skip
        vid = getattr(p, "vid", None)
        score = 0
        if vid in _HAND_VIDS:
            score += 10
        d = desc.lower()
        if "arduino" in d:
            score += 8
        if vid == 0x0403 or "usb serial port" in d or "ftdi" in d:
            score += 6  # ATOM Lite USB-serial (FTDI)
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
        # True unless a liveness ping proves the MCU is silent. VirtualHand and
        # any non-serial hand leave this True (nothing to disprove).
        self.mcu_alive = True
        self._ping_lock = threading.Lock()   # serialize pings; never blocks teleop
        self._last_ping_mono = 0.0

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
        self._maybe_ping()
        return {
            "connected": not self.offline,
            "offline": self.offline,
            "mcu_alive": self.mcu_alive,
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
    def _maybe_ping(self) -> None:
        """Periodic MCU liveness recheck. No-op for hands with no live serial
        link (VirtualHand); HandDriver overrides it."""

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
        # Verify the MCU actually answers (the FTDI adapter opens even when the
        # ATOM is unpowered). The ping command IS "tspd 40 6", so this also
        # applies the teleop ramp tuning that used to be sent blind here.
        alive = self.ping()
        if not alive:
            log.warning(
                "hand: port %s opens but the ATOM is silent (no ACK to '%s'). "
                "The FTDI USB-serial adapter is powered independently of the "
                "ATOM — check the ATOM's USB-C power. Keeping the connection "
                "open in case power comes up; run scripts/hand_diag.py to "
                "confirm.", self.port, PING_CMD)
        time.sleep(0.05)
        self.neutral()  # land at a known pose

    def ping(self, timeout_s: float = PING_TIMEOUT_S) -> bool:
        """Prove the MCU is alive by sending an ACK-producing command and
        reading the reply. Updates and returns ``self.mcu_alive``.

        Interleaves safely with the silent 't' teleop stream: the serial lock
        is held only for sub-millisecond buffer ops (write, in_waiting read),
        never across the wait for the reply, so a 40Hz follow stream is not
        stalled. Only one ping runs at a time — a concurrent caller returns the
        last known value instead of piling a second command onto the bus."""
        if serial is None:
            return self.mcu_alive
        if not self._ping_lock.acquire(blocking=False):
            return self.mcu_alive  # another ping in flight; don't double-send
        try:
            with self._lock:
                if self._ser is None:
                    return self.mcu_alive
                try:
                    self._ser.reset_input_buffer()  # drop any stale ACK bytes
                    self._ser.write((PING_CMD + "\n").encode("ascii"))
                except Exception as e:  # noqa: BLE001
                    log.debug("hand ping write failed: %s", e)
                    self.mcu_alive = False
                    self._last_ping_mono = time.monotonic()
                    return False
            deadline = time.monotonic() + timeout_s
            buf = ""
            alive = False
            while time.monotonic() < deadline:
                chunk = ""
                with self._lock:                 # brief: read whatever arrived
                    if self._ser is None:
                        break
                    try:
                        n = self._ser.in_waiting
                        if n:
                            chunk = self._ser.read(n).decode("utf-8", "replace")
                    except Exception as e:  # noqa: BLE001
                        log.debug("hand ping read failed: %s", e)
                        break
                if chunk:
                    buf += chunk
                    if PING_ACK_TOKEN in buf:
                        alive = True
                        break
                else:
                    time.sleep(0.008)            # yield to teleop between polls
            self.mcu_alive = alive
            self._last_ping_mono = time.monotonic()
            return alive
        finally:
            self._ping_lock.release()

    def _maybe_ping(self) -> None:
        if time.monotonic() - self._last_ping_mono >= PING_INTERVAL_S:
            self.ping()

    def _write_line(self, line: str) -> None:
        with self._lock:
            if self._ser is None:
                return
            self._ser.write((line + "\n").encode("ascii"))

    def _send_teleop(self, us: Sequence[int]) -> None:
        # firmware: "t u0 u1 u2 u3 u4" — non-blocking, silent (no ack to read)
        line = "t " + " ".join(str(int(u)) for u in us)
        with self._lock:
            if self._ser is None:
                return
            # Latest-wins at the OS TX-buffer level: if the previous line(s)
            # haven't finished transmitting (9600 baud ≈ 26ms per 't' line),
            # drop THIS frame instead of queueing it. An unbounded backlog is
            # what starved the liveness ping's ACK during VR streaming
            # (false "MCU応答なし", 2026-07-17) and adds pure latency —
            # a stale finger frame has no value once a fresher one exists.
            try:
                if self._ser.out_waiting > 8:
                    return
            except Exception:
                pass  # out_waiting unsupported -> keep old always-send behavior
            self._ser.write((line + "\n").encode("ascii"))

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
