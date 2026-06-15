"""Write the SO-101 calibration file from already-recorded values, WITHOUT any
bus I/O (so a flaky serial link can't block the save). Reuses lerobot's own
MotorCalibration + Robot._save_calibration() serialization so the format matches
exactly what the real driver expects.

Values came from the live calibration GUI (homing offsets already written to the
motors' EEPROM by the homing step; ranges recorded by sweeping).
"""
from __future__ import annotations
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.motors import MotorCalibration

# name -> (id, homing_offset, range_min, range_max)   (wrist_roll = full turn)
# 2026-06-10 third (final) calibration — clean encoder domain:
#   - servo power-cycle cleared multiturn counts (root cause of rounds 1-2)
#   - homing sign fixed: Present = raw - Homing_Offset, so off = u - 2048
#   - neutral = model zero pose; all joints read exactly 2048 at neutral
#   - ranges recorded in offset-applied domain: all within 0..4095, ~2048-centered
DATA = {
    "shoulder_pan":  (1, -1974,  874, 3159),
    "shoulder_lift": (2,    53,  815, 3145),
    "elbow_flex":    (3,  1042,  859, 3065),
    "wrist_flex":    (4,   563,  911, 3193),
    "wrist_roll":    (5,  1990,    0, 4095),
    "gripper":       (6, -1719, 1605, 3086),
}

robot = SO101Follower(SO101FollowerConfig(port="COM_UNUSED", id="so101_follower"))
robot.calibration = {
    name: MotorCalibration(id=i, drive_mode=0, homing_offset=off,
                           range_min=rmin, range_max=rmax)
    for name, (i, off, rmin, rmax) in DATA.items()
}
robot._save_calibration()  # file write only — no bus
print("SAVED", robot.calibration_fpath)
