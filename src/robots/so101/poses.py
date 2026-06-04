"""Named joint poses for SO-101 (5-DoF, degrees).

All poses are validated against safety.check_angles in tests. Angles are in the
canonical joint order (profile.JOINT_NAMES). HOME = the calibration midpoint
(all zeros); the arm is extended forward there, not tucked — SO-101 has no
spring-loaded rest like a tucked industrial arm.

Camera-relative poses (OBSERVE_*) are PROVISIONAL: the wrist-camera mount on the
SO-101 isn't fixed yet, so the "good viewing angle" will need re-tuning once the
camera is mounted and hand-eye calibrated.
"""
from __future__ import annotations
from . import profile

HOME = list(profile.HOME_ANGLES)            # [0,0,0,0,0] — extended forward
READY = [0.0, -35.0, 45.0, -15.0, 0.0]      # raised, slightly bent neutral
UP = [0.0, -90.0, 30.0, 0.0, 0.0]           # pointing upward (stretch)
FORWARD_LOW = [0.0, -20.0, 55.0, -30.0, 0.0]  # reaching forward + low

# PROVISIONAL — re-tune after the wrist camera is mounted/calibrated.
OBSERVE = [0.0, -50.0, 40.0, 10.0, 0.0]

POSES = {
    "HOME": HOME,
    "READY": READY,
    "UP": UP,
    "FORWARD_LOW": FORWARD_LOW,
    "OBSERVE": OBSERVE,
}
