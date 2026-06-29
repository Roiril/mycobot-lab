"""SO-101 motor driver abstraction.

The SO-101's Feetech STS3215 bus is driven by LeRobot's `SO101Follower` /
`FeetechMotorsBus` — we wrap that rather than re-implementing the Feetech serial
protocol (staying LeRobot-action-format compatible keeps the dataset/policy
ecosystem available later). `lerobot` is an OPTIONAL, heavy dependency
(Python>=3.12, PyTorch); it is imported lazily inside the real driver only.

This is the seam the future `SO101Hub` (HubBase implementation) sits on:
  - `LerobotSo101Driver` : real hardware (lazy lerobot import)
  - `MockSo101Driver`    : in-memory, for offline dev + tests (no deps)

Unit conventions (match lerobot SO101Follower defaults):
  - 5 arm joints in canonical order (profile.JOINT_NAMES), DEGREES.
  - gripper as 0..100 (RANGE_0_100): 0 = one end, 100 = other (per calibration).
"""
from __future__ import annotations
import abc
from typing import Dict, List, Optional, Sequence

from . import profile
from .safety import clamp_angles


def angles_to_action(angles: Sequence[float], gripper: Optional[float] = None) -> Dict[str, float]:
    """Canonical ordered list (deg) -> lerobot action dict {"<joint>.pos": v}."""
    if len(angles) != profile.NUM_JOINTS:
        raise ValueError(f"angles must be length {profile.NUM_JOINTS}")
    act = {f"{name}.pos": float(a) for name, a in zip(profile.JOINT_NAMES, angles)}
    if gripper is not None:
        act[f"{profile.GRIPPER_NAME}.pos"] = float(gripper)
    return act


def wrap_deg(a: float) -> float:
    """Wrap any degree reading into [-180, 180). lerobot reports full-turn
    joints (wrist_roll, range 0..4095) as 0..360+, which would fall outside
    profile.JOINT_LIMITS and trip the safety checks."""
    return ((float(a) + 180.0) % 360.0) - 180.0


def observation_to_angles(obs: Dict[str, float]) -> List[float]:
    """lerobot observation dict -> canonical ordered arm angles (deg)."""
    return [wrap_deg(obs[f"{name}.pos"]) for name in profile.JOINT_NAMES]


class So101DriverBase(abc.ABC):
    @abc.abstractmethod
    def connect(self) -> None: ...
    @abc.abstractmethod
    def disconnect(self) -> None: ...
    @abc.abstractmethod
    def read_angles(self) -> List[float]:
        """5 arm joint angles (deg), canonical order."""
    @abc.abstractmethod
    def read_gripper(self) -> Optional[float]:
        """Gripper 0..100, or None if unknown."""
    @abc.abstractmethod
    def write_angles(self, angles: Sequence[float], gripper: Optional[float] = None) -> None: ...
    @abc.abstractmethod
    def set_torque(self, enabled: bool) -> None: ...

    # True if the driver does its own velocity-profiled motion (real servos),
    # so the controller can send a single smooth goal instead of streaming
    # waypoints. Mock/sim leave this False to keep streamed animation.
    streams_smoothly: bool = False

    def set_speed_dps(self, dps: float) -> None:
        """Set the motion speed (deg/s) for the next move. No-op unless the
        driver has hardware velocity profiling."""
        return None

    def release(self) -> None:
        self.set_torque(False)

    def torque_on(self) -> Optional[bool]:
        """Last known torque state (None = unknown). Subclasses track it in
        set_torque/connect/disconnect; used by the UI to show 脱力中."""
        return getattr(self, "_torque", None)

    def ping(self) -> Optional[dict]:
        """Bus liveness probe: {id: name} of servos answering right now.
        None = not supported (mock/sim have no bus)."""
        return None


class MockSo101Driver(So101DriverBase):
    """In-memory driver for offline development and tests. Clamps to joint
    limits (so it behaves like a real arm that cannot exceed them) and holds
    the last commanded gripper value."""

    def __init__(self, start: Optional[Sequence[float]] = None):
        self._angles = clamp_angles(list(start) if start else list(profile.HOME_ANGLES))
        self._gripper: Optional[float] = None
        self._torque = False
        self._connected = False

    def connect(self) -> None:
        self._connected = True
        self._torque = True

    def disconnect(self) -> None:
        self._torque = False
        self._connected = False

    def read_angles(self) -> List[float]:
        return list(self._angles)

    def read_gripper(self) -> Optional[float]:
        return self._gripper

    def write_angles(self, angles: Sequence[float], gripper: Optional[float] = None) -> None:
        if not self._connected:
            raise RuntimeError("driver not connected")
        if not self._torque:
            raise RuntimeError("torque disabled; call set_torque(True) first")
        self._angles = clamp_angles(list(angles))
        if gripper is not None:
            self._gripper = max(0.0, min(100.0, float(gripper)))

    def set_torque(self, enabled: bool) -> None:
        self._torque = bool(enabled)


class LerobotSo101Driver(So101DriverBase):
    """Real hardware driver backed by lerobot's SO101Follower. lerobot is
    imported lazily in connect() so importing this module never pulls PyTorch.

    NOTE: untested against hardware as of writing — verify against the installed
    lerobot version (the so_follower API was still settling). Bring-up steps:
    `lerobot-find-port`, `lerobot-setup-motors`, `lerobot-calibrate` first.
    """

    def __init__(self, port: str, robot_id: str = "so101_follower"):
        self.port = port
        self.robot_id = robot_id
        self._robot = None
        self._torque: Optional[bool] = None

    streams_smoothly = True  # STS3215 has hardware velocity profiling

    def connect(self) -> None:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # lazy
        cfg = SO101FollowerConfig(port=self.port, id=self.robot_id)
        self._robot = SO101Follower(cfg)
        self._robot.connect(calibrate=False)  # expects prior lerobot-calibrate
        self._torque = True  # lerobot connect() enables torque
        # Gentle acceleration ramp = smoother starts/stops (default 254 = snappy/jerky).
        try:
            self._robot.bus.sync_write("Acceleration", 30, normalize=False)
        except Exception:
            pass

    def set_speed_dps(self, dps: float) -> None:
        if self._robot is None:
            return
        # STS3215 Goal_Velocity unit ≈ ticks/s (4096 ticks/rev). deg/s -> ticks/s
        # = dps * 4096/360 ≈ dps * 11.4. Clamp to a safe band.
        vel = max(60, min(2400, round(float(dps) * 11.4)))
        try:
            self._robot.bus.sync_write("Goal_Velocity", vel, normalize=False)
        except Exception:
            pass

    def disconnect(self) -> None:
        if self._robot is not None:
            self._robot.disconnect()  # disables torque
            self._robot = None
            self._torque = False

    def _require(self):
        if self._robot is None:
            raise RuntimeError("driver not connected")
        return self._robot

    def read_angles(self) -> List[float]:
        return observation_to_angles(self._require().get_observation())

    def read_gripper(self) -> Optional[float]:
        obs = self._require().get_observation()
        key = f"{profile.GRIPPER_NAME}.pos"
        return float(obs[key]) if key in obs else None

    def write_angles(self, angles: Sequence[float], gripper: Optional[float] = None) -> None:
        if gripper is not None:
            # Clamp like the mock does — lerobot would otherwise pass any float
            # (incl. out-of-range) straight to the serial layer.
            gripper = max(0.0, min(100.0, float(gripper)))
        self._require().send_action(angles_to_action(angles, gripper))

    def set_torque(self, enabled: bool) -> None:
        bus = getattr(self._require(), "bus", None)
        if bus is None:
            raise RuntimeError("lerobot robot exposes no .bus for torque control")
        bus.enable_torque() if enabled else bus.disable_torque()
        self._torque = bool(enabled)

    def ping(self) -> Optional[dict]:
        robot = self._robot
        if robot is None:
            return None
        found = robot.bus.broadcast_ping() or {}
        names = {m.id: n for n, m in robot.bus.motors.items()}
        return {int(i): names.get(int(i), "?") for i in found}
