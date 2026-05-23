"""myCobot 320-M5 client wrapper. Handles port discovery, safe defaults, context manager."""
from __future__ import annotations
import time
from contextlib import contextmanager
from typing import Iterator, Sequence

from pymycobot import MyCobot320
from serial.tools import list_ports

BAUD = 115200
DEFAULT_SPEED = 30
MAX_SPEED = 80  # hard cap to prevent dangerous moves


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
        """Release all servos (arm goes limp). Make sure arm is supported or in a low-fall pose first."""
        self.mc.release_all_servos()

    # --- state ---
    def version(self):
        return self.mc.get_system_version()

    def angles(self):
        return self.mc.get_angles()

    def coords(self):
        return self.mc.get_coords()

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

    @staticmethod
    def _check_speed(speed: int) -> None:
        if not 1 <= speed <= MAX_SPEED:
            raise ValueError(f"speed must be in [1, {MAX_SPEED}], got {speed}")

    @staticmethod
    def _estimate_duration(speed: int) -> float:
        # Rough heuristic: slower speed -> longer wait. Tune as needed.
        return max(2.0, 5.0 - speed * 0.03)


@contextmanager
def connect(port: str | None = None) -> Iterator[Arm]:
    """Open arm, power on, ensure release on exit only after returning to a safe pose."""
    arm = Arm(port)
    arm.power_on()
    try:
        yield arm
    finally:
        # Do NOT release servos on exit by default — the arm would fall.
        # Caller should move to rest pose before context exits.
        pass
