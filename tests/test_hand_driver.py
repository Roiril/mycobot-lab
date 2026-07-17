"""✋ HAND driver: MCU liveness ping (mocked serial).

The hand's ATOM Lite is reached through a SEPARATELY powered FTDI USB-serial
adapter, so the COM port opens even when the ATOM is dead. These tests verify
HandDriver.ping() tells the two apart via the firmware's tspd ACK, without any
real hardware (the serial port is a fake), and that a silent teleop 't' stream
never gets mistaken for the ACK.

Run: python -m unittest tests.test_hand_driver
"""
import types
import unittest
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "hand"))

import hand_driver  # noqa: E402
from hand_driver import HandDriver, VirtualHand, PING_ACK_TOKEN  # noqa: E402


class FakeSerial:
    """Minimal pyserial stand-in. When alive, replies to ACK-producing commands
    (tspd/open/close/n) like the firmware does; teleop 't' lines stay silent.
    When dead, emits nothing (mimics an unpowered ATOM behind a live FTDI)."""

    def __init__(self, port, baud, timeout=0.2, write_timeout=1.0, *, alive=True):
        self.port = port
        self.baud = baud
        self.is_open = True
        self.dtr = False
        self._alive = alive
        self._rx = bytearray()
        self._boot = b"ready. ...\n" if alive else b""
        self.written = []

    def read_all(self):
        data = bytes(self._boot) + bytes(self._rx)
        self._boot = b""
        self._rx = bytearray()
        return data

    def reset_input_buffer(self):
        self._boot = b""
        self._rx = bytearray()

    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, n):
        chunk = bytes(self._rx[:n])
        del self._rx[:n]
        return chunk

    def write(self, data):
        self.written.append(bytes(data))
        line = data.decode("ascii", "replace").strip()
        if self._alive:
            if line.startswith("tspd"):
                self._rx += b"tele step=40us / 6ms\n"
            elif line in ("open", "close", "n"):
                self._rx += (line + "\n").encode()
            # "t u0..u4" -> silent, no reply (teleop hot path)
        return len(data)

    def close(self):
        self.is_open = False


def _serial_module(alive):
    def Serial(port, baud, timeout=0.2, write_timeout=1.0):
        return FakeSerial(port, baud, timeout=timeout,
                          write_timeout=write_timeout, alive=alive)
    return types.SimpleNamespace(Serial=Serial)


class TestHandPing(unittest.TestCase):
    def setUp(self):
        self._orig_serial = hand_driver.serial

    def tearDown(self):
        hand_driver.serial = self._orig_serial

    def _driver(self, alive):
        hand_driver.serial = _serial_module(alive)
        return HandDriver(port="COM_TEST", reset_wait_s=0.0)

    def test_alive_mcu_pings_true(self):
        d = self._driver(alive=True)
        try:
            self.assertTrue(d.mcu_alive)
            st = d.status()
            self.assertTrue(st["mcu_alive"])
            self.assertTrue(st["connected"])
        finally:
            d.shutdown()

    def test_dead_mcu_connected_but_not_alive(self):
        # The whole point: port opens (connected) yet MCU is silent (not alive).
        d = self._driver(alive=False)
        try:
            self.assertFalse(d.mcu_alive)
            st = d.status()
            self.assertTrue(st["connected"])     # FTDI port opened fine
            self.assertFalse(st["mcu_alive"])    # but the ATOM never answered
        finally:
            d.shutdown()

    def test_ping_returns_bool_and_updates_flag(self):
        d = self._driver(alive=True)
        try:
            self.assertIs(d.ping(), True)
            # flip the fake to dead and re-ping
            d._ser._alive = False
            self.assertIs(d.ping(timeout_s=0.1), False)
            self.assertFalse(d.mcu_alive)
        finally:
            d.shutdown()

    def test_silent_teleop_not_mistaken_for_ack(self):
        # A teleop 't' write between pings produces no bytes, so it must not be
        # read as the tspd ACK, and a real ping right after still succeeds.
        d = self._driver(alive=True)
        try:
            d.set_fingers_us([1500, 1500, 1500, 1500, 1500])  # silent 't ...'
            self.assertTrue(d.ping())
            # the ACK token really is what proves liveness
            self.assertIn(PING_ACK_TOKEN, "tele step=40us / 6ms")
        finally:
            d.shutdown()


class TestVirtualHand(unittest.TestCase):
    def test_virtual_hand_reports_alive(self):
        # VirtualHand has nothing to disprove -> mcu_alive stays True (compat).
        v = VirtualHand()
        st = v.status()
        self.assertTrue(st["mcu_alive"])
        self.assertFalse(st["connected"])


if __name__ == "__main__":
    unittest.main()
