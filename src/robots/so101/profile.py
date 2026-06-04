"""SO-101 (SO-ARM101) kinematic + hardware profile — extracted reference data.

Source: TheRobotStudio/SO-ARM100 `Simulation/SO101/so101_new_calib.urdf`
(downloaded 2026-06-04, kept alongside as `so101_new_calib.urdf`).

This is plain extracted data, not yet wired into a RobotConfig (Phase 1/2 of
.agent/plans/2026-06-04_so101-integration.md). It exists so FK / the 3D UI can
be built and verified offline before hardware arrives.

Conventions (same as src/arm/kinematics.py so the parameterized kinematics can
consume both): each link is a fixed parent→child transform (xyz_mm, rpy_rad)
followed by a revolute joint about the child frame's z-axis. URDF values are in
meters/radians; xyz here is converted to mm. rpy kept as raw URDF radians.

Arm = 5 DoF (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll).
The 6th servo (gripper) opens the jaw and is NOT part of the FK arm chain.
"""
from __future__ import annotations
import math

PI = math.pi

# Canonical joint order (matches lerobot SO101Follower motor order 1..5).
# The lerobot API is per-named-joint dicts; this list is the ordered <-> dict
# mapping used by the (future) SO101Hub.
JOINT_NAMES = [
    "shoulder_pan",   # motor1, base rotation
    "shoulder_lift",  # motor2
    "elbow_flex",     # motor3
    "wrist_flex",     # motor4
    "wrist_roll",     # motor5
]
GRIPPER_NAME = "gripper"  # motor6, separate (0-100 normalized in lerobot)

NUM_JOINTS = 5

# URDF parent->child transforms (xyz_mm, rpy_rad). Joint angle rotates child z.
# Order: base->j1(shoulder_pan), j1->j2(shoulder_lift), ... j4->j5(wrist_roll).
URDF_LINKS = [
    ((  38.8353,  -0.0000090,  62.4),    ( 3.14159,  0.0,        -3.14159)),  # base       -> shoulder_pan
    (( -30.3992, -18.2778,    -54.2),    (-1.5708,  -1.5708,      0.0)),       # shoulder   -> shoulder_lift
    ((-112.57,   -28.0,         0.0),    ( 0.0,      0.0,         1.5708)),    # upper_arm  -> elbow_flex
    ((-134.9,      5.2,         0.0),    ( 0.0,      0.0,        -1.5708)),    # lower_arm  -> wrist_flex
    ((   0.0,    -61.1,        18.1),    ( 1.5708,   0.0486795,   3.14159)),   # wrist      -> wrist_roll
]

# Fixed transform from the wrist_roll output frame (gripper_link) to the
# gripper TCP frame (URDF `gripper_frame_joint`, fixed). This is the SO-101
# equivalent of myCobot's TOOL_LENGTH offset, but full 6-DoF (not just +z).
TOOL_TRANSFORM = ((-7.9, -0.218121, -98.1274), (0.0, 3.14159, 0.0))  # mm, rad

# Hardware joint limits (degrees), converted from URDF radians.
JOINT_LIMITS = [
    (-110.0,  110.0),   # shoulder_pan  (+-1.91986)
    (-100.0,  100.0),   # shoulder_lift (+-1.74533)
    ( -96.8,   96.8),   # elbow_flex    (+-1.69)
    ( -95.0,   95.0),   # wrist_flex    (+-1.65806)
    (-157.2,  162.8),   # wrist_roll    (-2.74385 .. 2.84121)
]
# Gripper jaw range (deg) for reference; lerobot commands it as 0-100 normalized.
GRIPPER_LIMIT_DEG = (-10.0, 100.0)  # (-0.174533 .. 1.74533)

# Provisional rest pose. lerobot has no fixed firmware HOME like myCobot; after
# calibration, 0deg = each joint's calibrated midpoint. Tune on hardware.
HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0]

# --- Safety geometry (PROVISIONAL — tune on hardware) ---
# Joint limits above are EXACT (from URDF). These collision/floor params are
# best-effort estimates for the STS3215-based arm and MUST be re-measured once
# the hardware is mounted. Conservative-small clearances so the self-collision
# check does not false-reject normal poses before it is tuned.
SAFETY = {
    "floor_z_mm": 0.0,        # table plane = base mount height; confirm on mount
    "link_radius_mm": 25.0,   # approx half-width of the servo/bracket links
    "self_clearance_mm": 35.0,
    "tool_clearance_mm": 25.0,  # gripper jaws are thinner than the arm links
    "degenerate_link_mm": 15.0,  # links shorter than this are skipped in pairs
}

# --- Hardware / bus facts (carry these into the driver; do NOT reuse myCobot's) ---
HARDWARE = {
    "servo": "Feetech STS3215",
    "voltage": "12V",              # 12V SKU (confirmed 2026-06-04); ~30 kg-cm
    "baud": 1_000_000,             # NOT 115200
    "encoder_ticks": 4096,         # 12-bit absolute, 0-4095 over ~360deg
    "bus_class": "lerobot.motors.feetech.FeetechMotorsBus",
    "robot_class": "lerobot.robots.so_follower.SO101Follower",
    "units_default": "degrees",    # use_degrees=True; gripper is RANGE_0_100
    # Windows: enumerates as COMx. lerobot-find-port / lerobot-setup-motors /
    # lerobot-calibrate are the bring-up CLIs. Waveshare board: jumpers on B.
}
