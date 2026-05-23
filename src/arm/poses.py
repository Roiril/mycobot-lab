"""Named joint poses (degrees). All 6 joints in order J1..J6."""

# Upright neutral, end effector pointing up-forward. Safe for clearance check.
HOME = [0, 0, -90, 0, 0, 0]

# Compact rest pose — folded, low profile. Use before power_off / release.
REST = [0, 90, -130, 40, 0, 0]

# Ready-to-grasp horizontal reach forward.
READY = [0, -30, -60, 0, 0, 0]

# Greet wave — used for demos.
WAVE_A = [0, 0, -90, 30, 30, 0]
WAVE_B = [0, 0, -90, -30, -30, 0]
