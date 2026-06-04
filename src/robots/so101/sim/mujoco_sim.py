"""MuJoCo-backed virtual SO-101 — pre-hardware development substrate.

The MJCF + meshes ship with TheRobotStudio/SO-ARM100 (Apache-2.0, vendored under
this directory). `pip install mujoco` provides a Windows wheel with a bundled GL
library, so this runs on Win11 with no extra setup.

Uses:
  - Independent ground-truth check of our hand-rolled FK (see
    tests/test_so101_mujoco.py — body xpos matches kinematics.joint_positions
    to <0.01 mm).
  - A "virtual arm with real geometry" the rest of the stack can drive before
    hardware arrives (a richer VirtualHub backend).
  - Offscreen rendering of the real arm shape (render()).

mujoco is an OPTIONAL dependency (requirements-sim.txt); import it lazily so the
core SO-101 modules stay importable without it.
"""
from __future__ import annotations
import math
import pathlib
from typing import List, Optional, Sequence

from .. import profile  # so101.profile

_SCENE = pathlib.Path(__file__).with_name("scene.xml")
_MODEL_ONLY = pathlib.Path(__file__).with_name("so101_new_calib.xml")

# MJCF body name per arm joint output frame, in canonical joint order.
_BODY_BY_JOINT = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper"]


class So101Sim:
    """Thin wrapper around a MuJoCo SO-101 model (5 arm joints + gripper)."""

    def __init__(self, with_scene: bool = True):
        import mujoco  # lazy
        self._mj = mujoco
        path = _SCENE if with_scene else _MODEL_ONLY
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self._renderer = None
        self.forward()

    # --- state ---
    def set_angles_deg(self, angles: Sequence[float], gripper: Optional[float] = None) -> None:
        """Set the 5 arm joints (deg). gripper: optional jaw angle (deg)."""
        if len(angles) != profile.NUM_JOINTS:
            raise ValueError(f"angles must be length {profile.NUM_JOINTS}")
        for i, a in enumerate(angles):
            self.data.qpos[i] = math.radians(a)
        if gripper is not None:
            self.data.qpos[profile.NUM_JOINTS] = math.radians(gripper)
        self.forward()

    def get_angles_deg(self) -> List[float]:
        return [math.degrees(self.data.qpos[i]) for i in range(profile.NUM_JOINTS)]

    def forward(self) -> None:
        self._mj.mj_forward(self.model, self.data)

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._mj.mj_step(self.model, self.data)

    def body_pos_mm(self, body: str) -> List[float]:
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, body)
        if bid < 0:
            raise KeyError(f"no body '{body}'")
        return [float(c) * 1000.0 for c in self.data.xpos[bid]]

    def joint_positions_mm(self) -> List[List[float]]:
        """Base + per-joint output-frame origins (mm), matching the order of
        kinematics.joint_positions (which also prepends the base origin)."""
        base = self.body_pos_mm("base")
        return [base] + [self.body_pos_mm(b) for b in _BODY_BY_JOINT]

    # --- rendering ---
    def _framed_camera(self):
        """Free camera framed on the arm (it is small: ~0.4 m reach)."""
        cam = self._mj.MjvCamera()
        cam.type = self._mj.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.10, 0.0, 0.20]
        cam.distance = 0.55
        cam.azimuth = 140.0
        cam.elevation = -20.0
        return cam

    def render(self, width: int = 640, height: int = 480,
               camera: Optional[str] = None) -> "object":
        """Return an HxWx3 uint8 RGB array of the current pose.
        Offscreen GL; on some headless Windows shells a GL context may be
        unavailable — callers should be ready for a RuntimeError.
        camera: named MJCF camera, or None for a free camera framed on the arm.
        """
        if self._renderer is None:
            self._renderer = self._mj.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data, camera=camera if camera is not None else self._framed_camera())
        return self._renderer.render()

    def save_png(self, path: str, **kw) -> None:
        img = self.render(**kw)
        try:
            import imageio.v2 as imageio
            imageio.imwrite(path, img)
        except ImportError:
            from PIL import Image
            Image.fromarray(img).save(path)


class MujocoSo101Driver:
    """So101DriverBase backed by the MuJoCo model — a virtual arm with real
    geometry. Kinematic (sets qpos directly), so commanded poses render exactly.
    Lets the control stack (controller.py) run + be watched fully offline.

    Implements the So101DriverBase interface structurally (duck-typed) to avoid
    pulling mujoco into driver.py's import path.
    """

    def __init__(self):
        self.sim = So101Sim()
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
        return self.sim.get_angles_deg()

    def read_gripper(self) -> Optional[float]:
        return self._gripper

    def _gripper_0_100_to_deg(self, v: float) -> float:
        lo, hi = profile.GRIPPER_LIMIT_DEG
        return lo + (max(0.0, min(100.0, v)) / 100.0) * (hi - lo)

    def write_angles(self, angles: Sequence[float], gripper: Optional[float] = None) -> None:
        if not self._connected:
            raise RuntimeError("driver not connected")
        if not self._torque:
            raise RuntimeError("torque disabled; call set_torque(True) first")
        clamped = [max(lo, min(hi, float(a))) for a, (lo, hi) in zip(angles, profile.JOINT_LIMITS)]
        gdeg = None
        if gripper is not None:
            self._gripper = max(0.0, min(100.0, float(gripper)))
            gdeg = self._gripper_0_100_to_deg(self._gripper)
        self.sim.set_angles_deg(clamped, gripper=gdeg)

    def set_torque(self, enabled: bool) -> None:
        self._torque = bool(enabled)

    def release(self) -> None:
        self.set_torque(False)

    def render(self, **kw):
        return self.sim.render(**kw)
