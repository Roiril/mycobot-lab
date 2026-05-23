"""myCobot 320-M5 client wrapper. Handles port discovery, power, motion, close."""
from __future__ import annotations
import time
from contextlib import contextmanager
from typing import Iterator, Sequence

from pymycobot import MyCobot320
from serial.tools import list_ports

from .constants import MAX_SPEED, DEFAULT_SPEED

BAUD = 115200


def find_port() -> str:
    """Pick the CH9102 / USB-Serial port for the arm. Raises if not found."""
    for p in list_ports.comports():
        desc = (p.description or "").upper()
        if "CH9102" in desc or "USB-SERIAL" in desc or "USB-ENHANCED-SERIAL" in desc:
            return p.device
    raise RuntimeError(f"no CH9102/USB-Serial port found. ports={[p.device for p in list_ports.comports()]}")


class Arm:
    def __init__(self, port: str | None = None, baud: int = BAUD):
        self.port = port or find_port()
        self.baud = baud
        self.mc = MyCobot320(self.port, baud)
        time.sleep(0.5)

    # --- lifecycle ---
    def power_on(self) -> None:
        self.mc.power_on()
        time.sleep(1.0)
        if self.mc.is_power_on() != 1:
            raise RuntimeError("power_on failed (is_power_on != 1)")

    def power_off(self) -> None:
        self.mc.power_off()

    def release(self) -> None:
        """Release all servos (arm goes limp — make sure it's supported)."""
        self.mc.release_all_servos()

    def close(self) -> None:
        """Close the underlying serial port."""
        try:
            sp = getattr(self.mc, "_serial_port", None)
            if sp is not None and getattr(sp, "is_open", False):
                sp.close()
        except Exception as e:
            print(f"[warn] arm.close: {e}")

    # --- state ---
    def version(self): return self.mc.get_system_version()
    def angles(self):  return self.mc.get_angles()
    def coords(self):  return self.mc.get_coords()

    # --- motion ---
    def move(self, angles: Sequence[float], speed: int = DEFAULT_SPEED, wait: float | None = None) -> None:
        self._check_speed(speed)
        self.mc.send_angles(list(angles), speed)
        if wait is None:
            wait = self._estimate_duration(speed)
        time.sleep(wait)

    def move_joint(self, joint: int, deg: float, speed: int = DEFAULT_SPEED, wait: float = 2.5) -> None:
        self._check_speed(speed)
        self.mc.send_angle(joint, deg, speed)
        time.sleep(wait)

    def move_to(self, coords, speed: int = DEFAULT_SPEED, mode: int = 0, wait: float = 2.0) -> None:
        self._check_speed(speed)
        if len(coords) != 6:
            raise ValueError("coords must be length-6 [x,y,z,rx,ry,rz]")
        self.mc.send_coords(list(coords), speed, mode)
        time.sleep(wait)

    @staticmethod
    def _check_speed(speed: int) -> None:
        if not 1 <= speed <= MAX_SPEED:
            raise ValueError(f"speed must be in [1, {MAX_SPEED}], got {speed}")

    @staticmethod
    def _estimate_duration(speed: int) -> float:
        return max(2.0, 5.0 - speed * 0.03)


@contextmanager
def connect(port: str | None = None) -> Iterator[Arm]:
    """Open arm and power on. Caller is responsible for moving to a safe pose before exit."""
    arm = Arm(port)
    arm.power_on()
    try:
        yield arm
    finally:
        arm.close()
